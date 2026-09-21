from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="bluerov2"),
            DeclareLaunchArgument("config_file", default_value=""),
            Node(
                package="bluerov2_control",
                executable="simple_sensor_suite.py",
                namespace=LaunchConfiguration("namespace"),
                name="simple_sensor_suite",
                output="screen",
                parameters=[LaunchConfiguration("config_file")],
            ),
        ]
    )
