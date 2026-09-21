#!/usr/bin/env python3
"""Launch the BlueROV2 200 m Gazebo world with headless-safe defaults."""

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
    spawn_z = LaunchConfiguration("spawn_z")

    gazebo_pkg = FindPackageShare("bluerov2_gazebo")
    control_pkg = FindPackageShare("bluerov2_control")
    description_pkg = FindPackageShare("bluerov2_description")

    world_file = PathJoinSubstitution([gazebo_pkg, "worlds", "bluerov2_demo.sdf"])
    gui_config = PathJoinSubstitution([gazebo_pkg, "config", "gazebo_underwater_gui.config"])
    model_xacro = PathJoinSubstitution([description_pkg, "models", "bluerov2_gz.sdf.xacro"])
    thruster_config = PathJoinSubstitution([control_pkg, "config", "thruster_manager.yaml"])
    tam_file = PathJoinSubstitution([control_pkg, "config", "TAM.yaml"])
    rviz_config = PathJoinSubstitution([gazebo_pkg, "rviz", "autonomous.rviz"])
    environment_config = PathJoinSubstitution([gazebo_pkg, "config", "rviz_environment.yaml"])

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

    spawn_bluerov2 = TimerAction(
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
    )

    ros_gz_bridge = TimerAction(
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
    )

    upload_robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([description_pkg, "/launch/upload_bluerov2.launch.py"]),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": "true",
        }.items(),
    )

    thruster_manager = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([control_pkg, "/launch/start_thruster_manager.launch.py"]),
        launch_arguments={
            "namespace": namespace,
            "config_file": thruster_config,
            "tam_file": tam_file,
        }.items(),
    )

    rviz = TimerAction(
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
    )

    environment_markers = TimerAction(
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
    )

    print_help = ExecuteProcess(
        cmd=[
            "bash",
            "-lc",
            "sleep 2 && "
            "echo && "
            "echo '================================================' && "
            "echo ' BlueROV2 200 m Gazebo world is ready.' && "
            "echo '================================================' && "
            "echo 'Default mode is headless Gazebo server: gz_gui:=false.' && "
            "echo 'Open Gazebo GUI with: gz_gui:=true.' && "
            "echo 'GUI defaults to the stable OGRE renderer. If needed, override with gz_render_engine:=ogre2.' && "
            "echo 'Open RViz with: open_rviz:=true.' && "
            "echo 'Use show_debug_hull:=true to inspect the collision hull.' && "
            "echo 'Use visualize_sonars:=true to render sonar rays.' && "
            "echo 'Fish patrol motion is built into the Gazebo world.' && "
            "echo 'FPV camera image topic: /bluerov2/camera/front/image' && "
            "echo '================================================' && "
            "echo",
        ],
        output="screen",
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
            SetEnvironmentVariable("GZ_RENDER_ENGINE", gz_render_engine),
            SetEnvironmentVariable("__GLX_VENDOR_LIBRARY_NAME", glx_vendor),
            SetEnvironmentVariable("LIBGL_ALWAYS_SOFTWARE", software_gl),
            SetEnvironmentVariable("QT_QPA_PLATFORM", "xcb"),
            SetEnvironmentVariable("QT_QUICK_BACKEND", qt_quick_backend),
            gz_sim_headless,
            gz_sim_gui,
            upload_robot_description,
            spawn_bluerov2,
            ros_gz_bridge,
            thruster_manager,
            environment_markers,
            rviz,
            print_help,
        ]
    )
