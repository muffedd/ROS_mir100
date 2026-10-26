#!/usr/bin/env python3
"""Send consecutive Nav2 goals and save their actual terminal statuses."""
import argparse
import json
import time
from pathlib import Path

import rclpy
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--goals', nargs='+', default=['6,0', '6,2'])
    parser.add_argument('--timeout', type=float, default=180.0)
    parser.add_argument('--report', default='/tmp/mir100-repeat-goals.json')
    args = parser.parse_args()
    goals = [tuple(map(float, goal.split(','))) for goal in args.goals]
    if any(len(goal) != 2 for goal in goals):
        parser.error('Each goal must be x,y')

    rclpy.init()
    node = Node('mir100_navigation_regression')
    node.set_parameters([Parameter('use_sim_time', value=True)])
    data = {'map': False, 'frames': {}, 'received': {}}
    report = {'goals': [], 'success': False}
    subscriptions = []

    def receive(topic, message):
        data['received'][topic] = time.monotonic()
        if topic == 'map':
            data['map'] = message.info.width > 0 and message.info.height > 0
        elif topic in ('odom', 'scan'):
            data['frames'][topic] = message.header.frame_id

    for topic, kind in [('odom', Odometry), ('scan', LaserScan), ('clock', Clock)]:
        subscriptions.append(node.create_subscription(
            kind, '/' + topic, lambda msg, name=topic: receive(name, msg),
            qos_profile_sensor_data))
    subscriptions.append(node.create_subscription(
        OccupancyGrid, '/map', lambda msg: receive('map', msg),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)))
    action = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    lifecycle = [node.create_client(GetState, '/' + name + '/get_state')
                 for name in ('controller_server', 'bt_navigator')]

    def spin_until(future, seconds):
        deadline = time.monotonic() + seconds
        while not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        return future.done()

    def ready():
        deadline = time.monotonic() + 90
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            now = time.monotonic()
            fresh = all(now - data['received'].get(name, 0) < 5
                        for name in ('clock', 'odom', 'scan'))
            if not (fresh and data['map'] and action.server_is_ready()):
                continue
            active = True
            for client in lifecycle:
                if not client.service_is_ready():
                    active = False
                    break
                future = client.call_async(GetState.Request())
                if not spin_until(future, 2) or future.result().current_state.id != 3:
                    active = False
                    break
            if active:
                assert data['frames']['odom'] == 'odom', data['frames']
                assert data['frames']['scan'] == 'front_laser_link', data['frames']
                return
        raise TimeoutError('Map, live sensors, or active Nav2 lifecycle unavailable')

    try:
        for x, y in goals:
            ready()
            item = {'goal': [x, y], 'samples': []}
            report['goals'].append(item)
            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = 'map'
            goal.pose.header.stamp = node.get_clock().now().to_msg()
            goal.pose.pose.position.x = x
            goal.pose.pose.position.y = y
            goal.pose.pose.orientation.w = 1.0

            def feedback(message):
                value = message.feedback
                p = value.current_pose.pose.position
                item['pose'] = [p.x, p.y]
                item['remaining'] = value.distance_remaining
                item['recoveries'] = value.number_of_recoveries

            sent = action.send_goal_async(goal, feedback_callback=feedback)
            if not spin_until(sent, 10) or not sent.result().accepted:
                raise RuntimeError('Nav2 rejected or did not acknowledge the goal')
            handle = sent.result()
            result = handle.get_result_async()
            started = last_log = time.monotonic()
            print('ACCEPTED', [x, y], flush=True)
            while not result.done() and time.monotonic() - started < args.timeout:
                rclpy.spin_once(node, timeout_sec=0.1)
                now = time.monotonic()
                if now - last_log >= 10:
                    sample = {key: item.get(key) for key in ('pose', 'remaining', 'recoveries')}
                    item['samples'].append(sample)
                    print('PROGRESS', sample, flush=True)
                    last_log = now
            if not result.done():
                spin_until(handle.cancel_goal_async(), 5)
                raise TimeoutError('Navigation goal exceeded timeout')
            outcome = result.result()
            item.update(status=outcome.status, error_code=outcome.result.error_code,
                        error_msg=outcome.result.error_msg,
                        wall_seconds=time.monotonic() - started)
            print('RESULT', json.dumps(item), flush=True)
            if outcome.status != 4:
                raise RuntimeError('Goal did not finish with SUCCEEDED status')
        report['success'] = True
    except Exception as error:
        report['error'] = str(error)
        print('FAILURE', str(error), flush=True)
    finally:
        report['frames'] = data['frames']
        output = Path(args.report)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(report, indent=2) + '\n')
        node.destroy_node()
        rclpy.shutdown()
    return 0 if report['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
