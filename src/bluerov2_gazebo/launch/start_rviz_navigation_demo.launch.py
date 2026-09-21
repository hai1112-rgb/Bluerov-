from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    control_config = LaunchConfiguration("controller_config")
    thruster_config = LaunchConfiguration("thruster_config")
    simulator_config = LaunchConfiguration("simulator_config")
    sensor_config = LaunchConfiguration("sensor_config")
    localization_config = LaunchConfiguration("localization_config")
    environment_config = LaunchConfiguration("environment_config")
    rviz_config = LaunchConfiguration("rviz_config")
    rviz_glx_vendor = LaunchConfiguration("rviz_glx_vendor")
    open_rviz = LaunchConfiguration("open_rviz")
    enable_autopilot = LaunchConfiguration("enable_autopilot")
    open_goal_panel = LaunchConfiguration("open_goal_panel")
    print_thruster_status = LaunchConfiguration("print_thruster_status")
    print_physics_status = LaunchConfiguration("print_physics_status")

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="bluerov2"),
            DeclareLaunchArgument(
                "controller_config",
                default_value=[FindPackageShare("bluerov2_control"), "/config/autonomous_controller.yaml"],
            ),
            DeclareLaunchArgument(
                "thruster_config",
                default_value=[FindPackageShare("bluerov2_control"), "/config/thruster_manager.yaml"],
            ),
            DeclareLaunchArgument(
                "simulator_config",
                default_value=[FindPackageShare("bluerov2_control"), "/config/simulator.yaml"],
            ),
            DeclareLaunchArgument(
                "sensor_config",
                default_value=[FindPackageShare("bluerov2_control"), "/config/sensors.yaml"],
            ),
            DeclareLaunchArgument(
                "localization_config",
                default_value=[FindPackageShare("bluerov2_control"), "/config/localization.yaml"],
            ),
            DeclareLaunchArgument(
                "environment_config",
                default_value=[FindPackageShare("bluerov2_gazebo"), "/config/rviz_environment.yaml"],
            ),
            DeclareLaunchArgument(
                "rviz_config",
                default_value=[FindPackageShare("bluerov2_gazebo"), "/rviz/autonomous.rviz"],
            ),
            DeclareLaunchArgument(
                "rviz_glx_vendor",
                default_value="mesa",
                description="GLX vendor for RViz. Mesa avoids broken NVIDIA GLX contexts on hybrid-GPU Wayland sessions.",
            ),
            DeclareLaunchArgument("open_rviz", default_value="true"),
            DeclareLaunchArgument("enable_autopilot", default_value="true"),
            DeclareLaunchArgument("open_goal_panel", default_value="true"),
            DeclareLaunchArgument("print_thruster_status", default_value="true"),
            DeclareLaunchArgument("print_physics_status", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("bluerov2_description"), "/launch/upload_bluerov2.launch.py"]
                ),
                launch_arguments={"namespace": namespace}.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("bluerov2_control"), "/launch/start_simulator.launch.py"]
                ),
                launch_arguments={
                    "namespace": namespace,
                    "config_file": simulator_config,
                    "print_physics_status": print_physics_status,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("bluerov2_control"), "/launch/start_thruster_manager.launch.py"]
                ),
                launch_arguments={
                    "namespace": namespace,
                    "config_file": thruster_config,
                    "tam_file": [FindPackageShare("bluerov2_control"), "/config/TAM.yaml"],
                    "print_thruster_status": print_thruster_status,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("bluerov2_control"), "/launch/start_sensor_suite.launch.py"]
                ),
                launch_arguments={
                    "namespace": namespace,
                    "config_file": sensor_config,
                }.items(),
            ),
            Node(
                package="bluerov2_control",
                executable="simple_sonar_sim.py",
                namespace=namespace,
                name="simple_sonar_sim",
                output="screen",
                parameters=[
                    {
                        "input_odom_topic": "sim/pose_gt",
                        "scene_file": environment_config,
                        "downward_max_range": 60.0,
                        "spherical_max_range": 45.0,
                        "terrain_voxel_size": 0.45,
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("bluerov2_control"), "/launch/start_localizer.launch.py"]
                ),
                launch_arguments={
                    "namespace": namespace,
                    "config_file": localization_config,
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [FindPackageShare("bluerov2_control"), "/launch/start_autonomous_controller.launch.py"]
                ),
                condition=IfCondition(enable_autopilot),
                launch_arguments={
                    "namespace": namespace,
                    "config_file": control_config,
                }.items(),
            ),
            Node(
                package="bluerov2_control",
                executable="environment_markers.py",
                namespace=namespace,
                name="environment_markers",
                output="screen",
                parameters=[
                    {
                        "frame_id": "world",
                        "marker_topic": "environment_markers",
                        "scene_file": environment_config,
                        "publish_rate": 0.2,
                    }
                ],
            ),
            TimerAction(
                period=2.5,
                condition=IfCondition(open_rviz),
                actions=[
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        name="rviz2",
                        output="screen",
                        additional_env={"__GLX_VENDOR_LIBRARY_NAME": rviz_glx_vendor},
                        arguments=["-d", rviz_config],
                    )
                ],
            ),
            TimerAction(
                period=3.0,
                condition=IfCondition(open_goal_panel),
                actions=[
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
                    )
                ],
            ),
        ]
    )
