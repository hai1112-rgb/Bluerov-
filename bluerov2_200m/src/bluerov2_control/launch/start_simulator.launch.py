from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="bluerov2"),
            DeclareLaunchArgument("config_file", default_value=""),
            DeclareLaunchArgument("print_physics_status", default_value="false"),
            Node(
                package="bluerov2_control",
                executable="simple_dynamics_sim.py",
                namespace=LaunchConfiguration("namespace"),
                name="simple_dynamics_sim",
                output="screen",
                parameters=[
                    LaunchConfiguration("config_file"),
                    {"print_physics_status": LaunchConfiguration("print_physics_status")},
                ],
            ),
        ]
    )
