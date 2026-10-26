# HARD-MAP TEST LAUNCH (2026-10-26): identical to slam_navigation.launch.py but loads config/nav2_params_hardtest.yaml (conservative radius test). Original launch file unchanged.
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from mir100_navigation.nav2_config import merge_nav2_params


def generate_launch_description():
    package_share = get_package_share_directory("mir100_navigation")
    nav2_share = get_package_share_directory("nav2_bringup")
    description_share = FindPackageShare("mir_description")

    with open(os.path.join(nav2_share, "params", "nav2_params.yaml"), encoding="utf-8") as stream:
        nav2_defaults = yaml.safe_load(stream)
    with open(os.path.join(package_share, "config", "nav2_params_hardtest.yaml"), encoding="utf-8") as stream:
        nav2_overrides = yaml.safe_load(stream)
    merged_nav2_params = merge_nav2_params(nav2_defaults, nav2_overrides)
    merged_nav2_params["bt_navigator"]["ros__parameters"]["default_nav_to_pose_bt_xml"] = os.path.join(
        package_share,
        "behavior_trees",
        "click_to_navigate.xml",
    )
    with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False, encoding="utf-8") as stream:
        yaml.safe_dump(merged_nav2_params, stream, sort_keys=False)
        nav2_params = stream.name

    world = LaunchConfiguration("world")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    yaw = LaunchConfiguration("yaw")
    gui = LaunchConfiguration("gui")
    use_sim_time = LaunchConfiguration("use_sim_time")

    gazebo_launch = PathJoinSubstitution(
        [description_share, "launch", "gazebo.launch.py"]
    )
    slam_launch = PathJoinSubstitution(
        [FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"]
    )
    nav2_launch = PathJoinSubstitution(
        [FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"]
    )
    slam_params = PathJoinSubstitution(
        [FindPackageShare("mir100_navigation"), "config", "slam_toolbox.yaml"]
    )
    ekf_params = PathJoinSubstitution(
        [FindPackageShare("mir100_navigation"), "config", "ekf.yaml"]
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("mir100_navigation"), "rviz", "navigation.rviz"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [description_share, "worlds", "hard_world.sdf"]
                ),
            ),
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("z", default_value="0.15"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument(
                "gui",
                default_value="true",
                description="Launch the Gazebo GUI client; set false for headless runs.",
            ),
            DeclareLaunchArgument("goal_x", default_value="6.0"),
            DeclareLaunchArgument("goal_y", default_value="0.0"),
            DeclareLaunchArgument("goal_yaw", default_value="0.0"),
            DeclareLaunchArgument("send_goal", default_value="false"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gazebo_launch),
                launch_arguments={
                    "world": world,
                    "x": x,
                    "y": y,
                    "z": z,
                    "yaw": yaw,
                    "gui": gui,
                }.items(),
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="mir100_gz_bridge",
                arguments=[
                    "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
                    "/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
                    "/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
                    "/back_scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
                    "/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
                ],
                output="screen",
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="mir100_front_scan_bridge",
                arguments=["/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"],
                parameters=[{"override_frame_id": "front_laser_link"}],
                output="screen",
            ),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                parameters=[ekf_params, {"use_sim_time": use_sim_time}],
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(slam_launch),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "slam_params_file": slam_params,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_launch),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "autostart": "true",
                    "params_file": nav2_params,
                    "use_composition": "False",
                }.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="mir100_navigation_rviz",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(LaunchConfiguration("launch_rviz")),
                output="screen",
            ),
            Node(
                package="mir100_navigation",
                executable="send_goal",
                name="mir100_goal_sender",
                parameters=[
                    {
                        "goal_x": ParameterValue(
                            LaunchConfiguration("goal_x"), value_type=float
                        ),
                        "goal_y": ParameterValue(
                            LaunchConfiguration("goal_y"), value_type=float
                        ),
                        "goal_yaw": ParameterValue(
                            LaunchConfiguration("goal_yaw"), value_type=float
                        ),
                        "use_sim_time": use_sim_time,
                    }
                ],
                condition=IfCondition(LaunchConfiguration("send_goal")),
                output="screen",
            ),
        ]
    )
