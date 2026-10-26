from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_joint_state_publisher = LaunchConfiguration("use_joint_state_publisher")
    xacro_file = PathJoinSubstitution(
        [FindPackageShare("mir_description"), "urdf", "mir_ros2.urdf.xacro"]
    )
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("mir_description"), "rviz", "mir_description.rviz"]
    )
    robot_description = Command(["xacro", " ", xacro_file])

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("use_joint_state_publisher", default_value="true"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "use_sim_time": use_sim_time,
                    }
                ],
                output="screen",
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                condition=IfCondition(use_joint_state_publisher),
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            ),
        ]
    )
