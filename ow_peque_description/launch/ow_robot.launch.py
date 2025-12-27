#!/usr/bin/env python3
"""
Launch file for real robot hardware.
Runs on: Raspberry Pi

This launch file starts all necessary nodes to operate the real ow_peque robot:
- Robot State Publisher (URDF with sim_mode:=false)
- Controller Manager with OmnidriveSystemHardware plugin
- Joint State Broadcaster
- Omni Wheel Drive Controller
- Optional: Teleop (joy + teleop_twist_joy) - usually run on PC instead
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Launch arguments
    enable_teleop = LaunchConfiguration("enable_teleop")

    # Robot description (URDF with real hardware plugin)
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

    # Controller configuration
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

    # Joint State Broadcaster spawner
    joint_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Omni Wheel Drive Controller spawner (starts after joint broadcaster)
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

    # Delay omni controller until joint broadcaster is ready
    delay_omni_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_broadcaster_spawner,
            on_exit=[omni_controller_spawner],
        )
    )

    # Joy node (optional, for teleop - usually run on PC instead)
    joy_node = Node(
        package="joy",
        executable="joy_node",
        output="screen",
        condition=IfCondition(enable_teleop),
    )

    # Teleop node (optional - usually run on PC instead)
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
            # Arguments
            DeclareLaunchArgument(
                "enable_teleop",
                default_value="false",
                description="Enable joystick teleop (usually run on PC instead).",
            ),
            # Core nodes
            robot_state_publisher,
            controller_manager,
            # Wait for controller_manager to be ready before spawning controllers
            TimerAction(period=2.0, actions=[joint_broadcaster_spawner]),
            delay_omni_controller,
            # Teleop (optional, usually run on PC instead)
            TimerAction(period=5.0, actions=[joy_node]),
            TimerAction(period=5.0, actions=[teleop_node]),
        ]
    )
