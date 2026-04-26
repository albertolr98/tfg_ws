#!/usr/bin/env python3
"""
Archivo de lanzamiento para hardware real.
Se ejecuta en: Raspberry Pi

Este launch file inicia todos los nodos necesarios para operar el robot ow_peque real:
- Robot State Publisher (URDF con sim_mode:=false)
- Controller Manager con plugin OmnidriveSystemHardware
- Joint State Broadcaster
- Omni Wheel Drive Controller
- Opcional: Teleop (joy + teleop_twist_joy) - normalmente se ejecuta en PC
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Argumentos de lanzamiento
    enable_teleop = LaunchConfiguration("enable_teleop")

    # Descripción del robot (URDF con plugin de hardware real)
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ow_peque_description"), "urdf", "ow.urdf.xacro"]
            ),
            " ",
            "sim_mode:=false",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Configuración de controladores
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("ow_peque_description"), "config", "controllers.yaml"]
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # Controller Manager (ros2_control)
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="screen",
    )

    # Spawner del Joint State Broadcaster
    joint_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Spawner del Omni Wheel Drive Controller (inicia después del joint broadcaster)
    omni_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "omni_wheel_drive_controller",
            "--controller-manager",
            "/controller_manager",
        ],
        output="screen",
    )

    # Lanzar joint broadcaster en cuanto arranca el controller manager
    delay_joint_broadcaster = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_broadcaster_spawner],
        )
    )

    # Lanzar omni controller cuando el joint broadcaster termina su spawn
    delay_omni_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_broadcaster_spawner,
            on_exit=[omni_controller_spawner],
        )
    )

    # Nodo Joy (opcional, para teleop - normalmente se ejecuta en PC)
    joy_node = Node(
        package="joy",
        executable="joy_node",
        output="screen",
        condition=IfCondition(enable_teleop),
    )

    # Nodo Teleop (opcional - normalmente se ejecuta en PC)
    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("ow_peque_description"), "config", "ps5_controller.yaml"]
            ),
        ],
        remappings=[
            ("cmd_vel", "/omni_wheel_drive_controller/cmd_vel"),
        ],
        output="screen",
        condition=IfCondition(enable_teleop),
    )

    return LaunchDescription(
        [
            # Argumentos
            DeclareLaunchArgument(
                "enable_teleop",
                default_value="false",
                description="Habilitar teleop con joystick (normalmente se ejecuta en PC).",
            ),
            # Nodos principales
            robot_state_publisher,
            controller_manager,
            delay_joint_broadcaster,
            delay_omni_controller,
            # Teleop (opcional, normalmente se ejecuta en PC)
            TimerAction(period=5.0, actions=[joy_node]),
            TimerAction(period=5.0, actions=[teleop_node]),
        ]
    )
