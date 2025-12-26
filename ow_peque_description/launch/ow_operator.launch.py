#!/usr/bin/env python3
"""
Operator Station Launch File.
Runs on: PC

This launch file provides the operator station utilities:
- RViz2 for visualization
- Optional: Teleop (joy + teleop_twist_joy)
- Optional: Digital Twin (Gazebo mirror)
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    enable_teleop = LaunchConfiguration("enable_teleop")
    enable_digital_twin = LaunchConfiguration("enable_digital_twin")

    # RViz2
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("ow_peque_description"), "rviz", "config.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    # Joy node (optional, for teleop)
    joy_node = Node(
        package="joy",
        executable="joy_node",
        output="screen",
        condition=IfCondition(enable_teleop),
    )

    # Teleop node (optional)
    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("ow_peque_description"), "config", "ps5_controller.yaml"]
            ),
        ],
        output="screen",
        condition=IfCondition(enable_teleop),
    )

    # Digital Twin (optional)
    digital_twin = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ow_peque_description"), "launch", "ow_digital_twin.launch.py"]
            )
        ),
        condition=IfCondition(enable_digital_twin),
    )

    return LaunchDescription(
        [
            # Arguments
            DeclareLaunchArgument(
                "enable_teleop",
                default_value="true",
                description="Enable joystick teleop (joy + teleop_twist_joy).",
            ),
            DeclareLaunchArgument(
                "enable_digital_twin",
                default_value="false",
                description="Enable Gazebo digital twin alongside RViz.",
            ),
            # Core
            rviz_node,
            # Teleop (optional)
            TimerAction(period=1.0, actions=[joy_node]),
            TimerAction(period=1.0, actions=[teleop_node]),
            # Digital Twin (optional)
            digital_twin,
        ]
    )
