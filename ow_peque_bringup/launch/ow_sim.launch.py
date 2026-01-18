#!/usr/bin/env python3
"""
Archivo de lanzamiento para simulación del robot ow_peque.
Se ejecuta en: PC

Este launch file inicia la simulación en Gazebo con integración completa de ros2_control:
- Simulador Gazebo Harmonic
- Robot State Publisher
- Controladores (joint_broad, omni_wheel_drive_controller)
- Teleop (joy + teleop_twist_joy)
- Visualización RViz2
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    gz_args = LaunchConfiguration("gz_args")
    world = LaunchConfiguration("world")
    entity_name = LaunchConfiguration("entity_name")
    controller_manager = LaunchConfiguration("controller_manager")

    # Configuración de RViz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ow_peque_description"), "rviz", "urdf_config.rviz"]
    )

    # Descripción del robot
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ow_peque_description"), "urdf", "ow.urdf.xacro"]
            ),
            " ",
            "sim_mode:=true",
        ]
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"robot_description": robot_description_content},
        ],
        output="screen",
    )

    # Simulador Gazebo
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={"gz_args": [gz_args, " ", world]}.items(),
    )

    # Spawn del robot en Gazebo
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", entity_name, "-topic", "robot_description"],
        output="screen",
    )

    # Bridge del reloj de simulación
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # Spawner del Joint State Broadcaster
    joint_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad", "--controller-manager", controller_manager],
        output="screen",
    )

    # Spawner del Omni Wheel Drive Controller
    omni_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["omni_wheel_drive_controller", "--controller-manager", controller_manager],
        output="screen",
    )

    # Nodo de rampa de velocidad (suaviza comandos)
    velocity_bridge = Node(
        package="ow_control",
        executable="linear_ramp",
        name="velocity_bridge",
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[
            ("cmd_vel_input", "/cmd_vel"),
            ("cmd_vel_out", "/omni_wheel_drive_controller/cmd_vel"),
        ],
    )

    # Nodo Joy para joystick
    joy_node = Node(
        package="joy",
        executable="joy_node",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # Nodo Teleop
    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("ow_peque_description"), "config", "ps5_controller.yaml"]
            ),
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Usar tiempo de simulación.",
            ),
            DeclareLaunchArgument(
                "gz_args",
                default_value="-r -v 4",
                description="Argumentos para gz sim (por defecto despausa la simulación).",
            ),
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ow_peque_description"), "worlds", "empty.sdf"]
                ),
                description="Ruta al archivo SDF del mundo.",
            ),
            DeclareLaunchArgument(
                "entity_name",
                default_value="ow_peque",
                description="Nombre de la entidad en Gazebo.",
            ),
            DeclareLaunchArgument(
                "controller_manager",
                default_value="/controller_manager",
                description="Namespace del controller manager.",
            ),
            gz_sim,
            clock_bridge,
            robot_state_publisher,
            spawn_entity,
            velocity_bridge,
            joy_node,
            teleop_node,
            rviz_node,
            TimerAction(period=2.0, actions=[joint_broadcaster_spawner]),
            TimerAction(period=4.0, actions=[omni_spawner]),
        ]
    )
