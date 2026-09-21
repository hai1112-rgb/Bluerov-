#!/usr/bin/env python3
"""Launch Gazebo + autonomous controller for the 200 m BlueROV2 scenario."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _bridge_arguments(namespace):
    return [
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        ["/", namespace, "/pose_gt@nav_msgs/msg/Odometry[gz.msgs.Odometry"],
        "/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
        ["/", namespace, "/magnetometer@sensor_msgs/msg/MagneticField[gz.msgs.Magnetometer"],
        ["/", namespace, "/sonar/forward@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"],
        ["/", namespace, "/sonar/downward@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan"],
        ["/", namespace, "/camera/front/image@sensor_msgs/msg/Image[gz.msgs.Image"],
        ["/", namespace, "/camera/front/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"],
        ["/", namespace, "/camera/front/image/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"],
        ["/", namespace, "/thrusters/thruster_0/input@std_msgs/msg/Float64]gz.msgs.Double"],
        ["/", namespace, "/thrusters/thruster_1/input@std_msgs/msg/Float64]gz.msgs.Double"],
        ["/", namespace, "/thrusters/thruster_2/input@std_msgs/msg/Float64]gz.msgs.Double"],
        ["/", namespace, "/thrusters/thruster_3/input@std_msgs/msg/Float64]gz.msgs.Double"],
        ["/", namespace, "/thrusters/thruster_4/input@std_msgs/msg/Float64]gz.msgs.Double"],
        ["/", namespace, "/thrusters/thruster_5/input@std_msgs/msg/Float64]gz.msgs.Double"],
    ]


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    gz_render_engine = LaunchConfiguration("gz_render_engine")
    glx_vendor = LaunchConfiguration("glx_vendor")
    software_gl = LaunchConfiguration("software_gl")
    qt_quick_backend = LaunchConfiguration("qt_quick_backend")
    gz_gui = LaunchConfiguration("gz_gui")
    open_rviz = LaunchConfiguration("open_rviz")
    show_debug_hull = LaunchConfiguration("show_debug_hull")
    visualize_sonars = LaunchConfiguration("visualize_sonars")
    controller_config = LaunchConfiguration("controller_config")
    spawn_z = LaunchConfiguration("spawn_z")

    share_control = FindPackageShare("bluerov2_control")
    share_gazebo = FindPackageShare("bluerov2_gazebo")
    share_description = FindPackageShare("bluerov2_description")

    world_file = PathJoinSubstitution([share_gazebo, "worlds", "bluerov2_demo.sdf"])
    gui_config = PathJoinSubstitution([share_gazebo, "config", "gazebo_underwater_gui.config"])
    model_xacro = PathJoinSubstitution([share_description, "models", "bluerov2_gz.sdf.xacro"])
    rviz_config = PathJoinSubstitution([share_gazebo, "rviz", "autonomous.rviz"])
    environment_config = PathJoinSubstitution([share_gazebo, "config", "rviz_environment.yaml"])

    gz_sim_headless = ExecuteProcess(
        cmd=["gz", "sim", "-r", "-s", world_file],
        output="screen",
        name="gz_sim_server",
        condition=UnlessCondition(gz_gui),
    )

    gz_sim_gui = ExecuteProcess(
        cmd=["gz", "sim", "-r", "--render-engine", gz_render_engine, "--gui-config", gui_config, world_file],
        output="screen",
        name="gz_sim_gui",
        condition=IfCondition(gz_gui),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("namespace", default_value="bluerov2"),
            DeclareLaunchArgument("gz_render_engine", default_value="ogre"),
            DeclareLaunchArgument("glx_vendor", default_value="mesa"),
            DeclareLaunchArgument("software_gl", default_value="0"),
            DeclareLaunchArgument("qt_quick_backend", default_value="opengl"),
            DeclareLaunchArgument("gz_gui", default_value="false"),
            DeclareLaunchArgument("open_rviz", default_value="false"),
            DeclareLaunchArgument("show_debug_hull", default_value="false"),
            DeclareLaunchArgument("visualize_sonars", default_value="false"),
            DeclareLaunchArgument("spawn_z", default_value="-200.0"),
            DeclareLaunchArgument(
                "controller_config",
                default_value=[share_control, "/config/autonomous_controller.yaml"],
            ),
            SetEnvironmentVariable("GZ_RENDER_ENGINE", gz_render_engine),
            SetEnvironmentVariable("__GLX_VENDOR_LIBRARY_NAME", glx_vendor),
            SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", software_gl),
            SetEnvironmentVariable("QT_QPA_PLATFORM", "xcb"),
            SetEnvironmentVariable("QT_QUICK_BACKEND", qt_quick_backend),
            gz_sim_headless,
            gz_sim_gui,
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([share_description, "/launch/upload_bluerov2.launch.py"]),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": "true",
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([share_control, "/launch/start_thruster_manager.launch.py"]),
                launch_arguments={
                    "namespace": namespace,
                    "config_file": [share_control, "/config/thruster_manager.yaml"],
                    "tam_file": [share_control, "/config/TAM.yaml"],
                }.items(),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([share_control, "/launch/start_autonomous_controller.launch.py"]),
                launch_arguments={
                    "namespace": namespace,
                    "config_file": controller_config,
                }.items(),
            ),
            TimerAction(
                period=1.5,
                actions=[
                    Node(
                        package="ros_gz_sim",
                        executable="create",
                        name="spawn_bluerov2_in_gazebo",
                        output="screen",
                        arguments=[
                            "-name",
                            namespace,
                            "-string",
                            Command(
                                [
                                    FindExecutable(name="xacro"),
                                    " ",
                                    model_xacro,
                                    " namespace:=",
                                    namespace,
                                    " show_debug_hull:=",
                                    show_debug_hull,
                                    " visualize_sonars:=",
                                    visualize_sonars,
                                ]
                            ),
                            "-z",
                            spawn_z,
                        ],
                    )
                ],
            ),
            TimerAction(
                period=2.0,
                actions=[
                    Node(
                        package="ros_gz_bridge",
                        executable="parameter_bridge",
                        name="ros_gz_bridge_bluerov2",
                        output="screen",
                        arguments=_bridge_arguments(namespace),
                    )
                ],
            ),
            TimerAction(
                period=2.5,
                condition=IfCondition(open_rviz),
                actions=[
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
                    )
                ],
            ),
            TimerAction(
                period=3.0,
                condition=IfCondition(open_rviz),
                actions=[
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        name="rviz2",
                        output="screen",
                        arguments=["-d", rviz_config],
                    )
                ],
            ),
        ]
    )
