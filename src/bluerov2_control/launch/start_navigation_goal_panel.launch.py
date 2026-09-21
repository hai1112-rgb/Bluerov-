from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="bluerov2"),
            Node(
                package="bluerov2_control",
                executable="navigation_goal_panel.py",
                namespace=namespace,
                name="navigation_goal_panel",
                output="screen",
                parameters=[
                    {
                        "odom_topic": "pose_gt",
                        "planned_path_topic": "planned_path",
                        "goal_pose_topic": "/goal_pose",
                        "depth_setpoint_topic": "/depth_setpoint",
                    }
                ],
            ),
        ]
    )
