#!/usr/bin/env python3
"""
Archivo de lanzamiento para estación de operador.
Se ejecuta en: PC

Este launch file proporciona las utilidades de la estación de operador:
- RViz2 para visualización
- Teleop (joy + teleop_twist_joy)
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Argumentos de lanzamiento
    enable_teleop = LaunchConfiguration("enable_teleop")

    # Configuración de RViz2
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("ow_description"), "rviz", "urdf_config.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    # Nodo Joy (opcional, para teleop)
    joy_node = Node(
        package="joy",
        executable="joy_node",
        output="screen",
        condition=IfCondition(enable_teleop),
    )

    # Nodo Teleop (opcional)
    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("ow_description"), "config", "ps5_controller.yaml"]
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
                default_value="true",
                description="Habilitar teleop con joystick.",
            ),
            # Nodos principales
            rviz_node,
            # Teleop (opcional)
            TimerAction(period=1.0, actions=[joy_node]),
            TimerAction(period=1.0, actions=[teleop_node]),
        ]
    )
