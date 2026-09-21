from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="bluerov2"),
            DeclareLaunchArgument("control_config", default_value=""),
            DeclareLaunchArgument("thruster_config", default_value=""),
            DeclareLaunchArgument("tam_file", default_value=""),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("bluerov2_control"), "/launch/start_thruster_manager.launch.py"]
                ),
                launch_arguments={
                    "namespace": LaunchConfiguration("namespace"),
                    "config_file": LaunchConfiguration("thruster_config"),
                    "tam_file": LaunchConfiguration("tam_file"),
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("bluerov2_control"), "/launch/start_autonomous_controller.launch.py"]
                ),
                launch_arguments={
                    "namespace": LaunchConfiguration("namespace"),
                    "config_file": LaunchConfiguration("control_config"),
                }.items(),
            ),
        ]
    )
