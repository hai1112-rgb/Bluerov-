from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="bluerov2"),
            DeclareLaunchArgument("config_file", default_value=""),
            DeclareLaunchArgument("tam_file", default_value=""),
            DeclareLaunchArgument("print_thruster_status", default_value="false"),
            Node(
                package="bluerov2_control",
                executable="virtual_thruster_manager.py",
                namespace=LaunchConfiguration("namespace"),
                name="virtual_thruster_manager",
                output="screen",
                parameters=[
                    LaunchConfiguration("config_file"),
                    {
                        "tam_file": LaunchConfiguration("tam_file"),
                        "print_thruster_status": LaunchConfiguration("print_thruster_status"),
                    },
                ],
            ),
        ]
    )
