from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    world = LaunchConfiguration("world")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")
    yaw = LaunchConfiguration("yaw")
    package_share = FindPackageShare("mir_description")
    xacro_file = PathJoinSubstitution(
        [package_share, "urdf", "mir_ros2.urdf.xacro"]
    )
    robot_description = Command(["xacro", " ", xacro_file])
    ros_gz_launch = PathJoinSubstitution(
        [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [package_share, "worlds", "empty.sdf"]
                ),
            ),
            DeclareLaunchArgument("x", default_value="0.0"),
            DeclareLaunchArgument("y", default_value="0.0"),
            DeclareLaunchArgument("z", default_value="0.15"),
            DeclareLaunchArgument("yaw", default_value="0.0"),
            SetEnvironmentVariable(
                name="GZ_SIM_RESOURCE_PATH",
                value=[
                    package_share,
                    ":",
                    PathJoinSubstitution([package_share, ".."]),
                    ":",
                    EnvironmentVariable("GZ_SIM_RESOURCE_PATH", default_value=""),
                ],
            ),
            SetEnvironmentVariable(
                name="IGN_GAZEBO_RESOURCE_PATH",
                value=[
                    package_share,
                    ":",
                    PathJoinSubstitution([package_share, ".."]),
                    ":",
                    EnvironmentVariable("IGN_GAZEBO_RESOURCE_PATH", default_value=""),
                ],
            ),
            SetEnvironmentVariable(name="QT_QPA_PLATFORM", value="xcb"),
            SetEnvironmentVariable(name="GDK_BACKEND", value="x11"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ros_gz_launch),
                launch_arguments={"gz_args": ["-r -s ", world]}.items(),
            ),
            TimerAction(
                period=5.0,
                actions=[
                    ExecuteProcess(
                        cmd=["gz", "sim", "--force-version", "8", "-g", "-r", world],
                        additional_env={
                            "QT_QPA_PLATFORM": "xcb",
                            "GDK_BACKEND": "x11",
                            "WAYLAND_DISPLAY": "",
                            "XDG_SESSION_TYPE": "x11",
                            "QT_OPENGL": "desktop",
                            "QT_XCB_GL_INTEGRATION": "xcb_glx",
                        },
                        output="screen",
                    ),
                ],
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[
                    {
                        "robot_description": ParameterValue(robot_description, value_type=str),
                        "use_sim_time": True,
                    }
                ],
                output="screen",
            ),
            Node(
                package="ros_gz_sim",
                executable="create",
                arguments=[
                    "-name",
                    "mir100",
                    "-topic",
                    "robot_description",
                    "-x",
                    x,
                    "-y",
                    y,
                    "-z",
                    z,
                    "-Y",
                    yaw,
                ],
                output="screen",
            ),
        ]
    )
