"""Send one configurable Nav2 goal after SLAM and navigation are ready."""

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node

from mir100_navigation.goal_math import quaternion_from_yaw


class GoalSender(Node):
    def __init__(self) -> None:
        super().__init__("mir100_goal_sender")
        self.declare_parameter("goal_x", 6.0)
        self.declare_parameter("goal_y", 0.0)
        self.declare_parameter("goal_yaw", 0.0)
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("action_name", "/navigate_to_pose")

        self.goal_x = float(self.get_parameter("goal_x").value)
        self.goal_y = float(self.get_parameter("goal_y").value)
        self.goal_yaw = float(self.get_parameter("goal_yaw").value)
        map_topic = str(self.get_parameter("map_topic").value)
        action_name = str(self.get_parameter("action_name").value)

        self._map_ready = False
        self._goal_sent = False
        self._action_client = ActionClient(self, NavigateToPose, action_name)
        self._map_subscription = self.create_subscription(
            OccupancyGrid, map_topic, self._map_callback, 10
        )
        self._ready_timer = self.create_timer(1.0, self._try_send_goal)

    def _map_callback(self, msg: OccupancyGrid) -> None:
        self._map_ready = msg.info.width > 0 and msg.info.height > 0 and any(
            value >= 0 for value in msg.data
        )

    def _try_send_goal(self) -> None:
        if self._goal_sent or not self._map_ready:
            return
        if not self._action_client.wait_for_server(timeout_sec=0.0):
            return

        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = "map"
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = self.goal_x
        goal.pose.pose.position.y = self.goal_y
        qx, qy, qz, qw = quaternion_from_yaw(self.goal_yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        self._goal_sent = True
        self.get_logger().info(
            f"Sending Nav2 goal ({self.goal_x:.2f}, {self.goal_y:.2f}, "
            f"{self.goal_yaw:.2f}) in map frame"
        )
        future = self._action_client.send_goal_async(
            goal, feedback_callback=self._feedback_callback
        )
        future.add_done_callback(self._goal_response_callback)
        self._ready_timer.cancel()

    def _goal_response_callback(self, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Nav2 rejected the autonomous goal")
            return

        self.get_logger().info("Nav2 accepted the autonomous goal")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _feedback_callback(self, _feedback_msg) -> None:
        pass

    def _result_callback(self, future) -> None:
        result = future.result()
        self.get_logger().info(f"Navigation finished with status {result.status}")


def main() -> None:
    rclpy.init()
    node = GoalSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
