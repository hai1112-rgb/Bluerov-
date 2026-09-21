from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    mode = LaunchConfiguration("mode")
    use_sim_time = LaunchConfiguration("use_sim_time")

    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                FindPackageShare("bluerov2_description"),
                "/robots/bluerov2_",
                mode,
                ".xacro",
                " namespace:=",
                namespace,
            ]
        ),
        value_type=str,
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="bluerov2"),
            DeclareLaunchArgument("mode", default_value="default"),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                namespace=namespace,
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "publish_robot_description": True,
                        "robot_description": robot_description,
                    }
                ],
            ),
        ]
    )
