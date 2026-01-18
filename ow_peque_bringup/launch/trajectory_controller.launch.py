#!/usr/bin/env python3
"""
Archivo de lanzamiento para controlador de trayectorias con joystick.
Se ejecuta en: PC

Lanza el nodo trajectory_controller junto con el nodo joy para
controlar diferentes trayectorias con los botones del mando.

Mapeo de botones (PS5):
- ▢ Square  → Trayectoria cuadrada
- ○ Circle  → Trayectoria circular
- △ Triangle → Trayectoria triangular
- ✕ Cross   → Trayectoria en X
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Argumentos configurables
    side_length = LaunchConfiguration('side_length')
    circle_radius = LaunchConfiguration('circle_radius')
    velocity = LaunchConfiguration('velocity')
    pause_time = LaunchConfiguration('pause_time')

    # Nodo Joy
    joy_node = Node(
        package='joy',
        executable='joy_node',
        output='screen',
    )

    # Nodo controlador de trayectorias
    trajectory_controller = Node(
        package='ow_control',
        executable='trajectory_controller',
        name='trajectory_controller',
        output='screen',
        parameters=[{
            'side_length': side_length,
            'circle_radius': circle_radius,
            'velocity': velocity,
            'pause_time': pause_time,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'side_length',
            default_value='0.5',
            description='Longitud del lado (cuadrado/triángulo) en metros.',
        ),
        DeclareLaunchArgument(
            'circle_radius',
            default_value='0.25',
            description='Radio del círculo en metros.',
        ),
        DeclareLaunchArgument(
            'velocity',
            default_value='0.2',
            description='Velocidad lineal en m/s.',
        ),
        DeclareLaunchArgument(
            'pause_time',
            default_value='1.0',
            description='Tiempo de pausa entre segmentos en segundos.',
        ),
        joy_node,
        TimerAction(period=1.0, actions=[trajectory_controller]),
    ])
