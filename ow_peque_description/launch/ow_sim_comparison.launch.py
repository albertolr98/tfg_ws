#!/usr/bin/env python3
"""
Simulation Comparison Launch File.
Runs on: PC

This launch file runs Gazebo simulation alongside the real robot for behavior comparison.
The simulation runs in namespace /sim to avoid conflicts with the real robot.

Usage:
  1. First launch the robot: ros2 launch ow_peque_description ow_robot.launch.py
  2. Then launch this: ros2 launch ow_peque_description ow_sim_comparison.launch.py

The cmd_vel_relay node forwards commands to BOTH controllers.
"""
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
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
    gz_args = LaunchConfiguration("gz_args")
    world = LaunchConfiguration("world")

    # Robot description for simulation WITH NAMESPACE
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ow_peque_description"), "urdf", "ow.urdf.xacro"]
            ),
            " ",
            "sim_mode:=true",
            " ",
            "sim_namespace:=/sim",  # Simulation controllers in /sim namespace
            " ",
            "controller_config:=controllers_sim.yaml",  # Use namespaced config
        ]
    )

    # Gazebo simulator
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments={"gz_args": [gz_args, " ", world]}.items(),
    )

    # Robot State Publisher for simulation in /sim namespace
    # Note: We disable TF publishing to avoid conflicts with real robot's TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="sim",
        parameters=[
            {"use_sim_time": False},
            {"robot_description": robot_description_content},
            {"publish_frequency": 0.0},  # Disable TF publishing
        ],
        output="screen",
    )

    # Clock bridge
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-name", "ow_peque_sim", "-topic", "/sim/robot_description"],
        output="screen",
    )

    # Controller spawners for Gazebo simulation (in /sim namespace)
    joint_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad", "--controller-manager", "/sim/controller_manager"],
        output="screen",
    )

    omni_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["omni_wheel_drive_controller", "--controller-manager", "/sim/controller_manager"],
        output="screen",
    )

    # Joy node
    joy_node = Node(
        package="joy",
        executable="joy_node",
        output="screen",
    )

    # Teleop node - publishes to /cmd_vel
    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        parameters=[
            PathJoinSubstitution(
                [FindPackageShare("ow_peque_description"), "config", "ps5_controller.yaml"]
            ),
        ],
        # Keep publishing to /cmd_vel, relay will distribute
        output="screen",
    )

    # Relay node - republishes /cmd_vel to BOTH controllers
    cmd_vel_relay = Node(
        package="ow_control",
        executable="cmd_vel_relay",
        name="cmd_vel_relay",
        parameters=[
            {
                "targets": [
                    "/omni_wheel_drive_controller/cmd_vel",  # Real robot (direct)
                    "/sim/cmd_vel_raw",  # Simulation (through ramp)
                ],
                "use_stamped": True,
            }
        ],
        output="screen",
    )

    # Velocity ramp node for simulation (to match real robot behavior)
    # The TMC driver has built-in ramps, so simulation needs this to behave similarly
    sim_velocity_ramp = Node(
        package="ow_control",
        executable="linear_ramp",
        name="sim_velocity_ramp",
        namespace="sim",
        parameters=[{"use_sim_time": False}],
        remappings=[
            ("cmd_vel_input", "/sim/cmd_vel_raw"),
            ("cmd_vel_out", "/sim/omni_wheel_drive_controller/cmd_vel"),
        ],
        output="screen",
    )

    # RViz for visualization
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("ow_peque_description"), "rviz", "urdf_config.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gz_args",
                default_value="-r -v 2",
                description="Arguments for gz sim.",
            ),
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ow_peque_description"), "worlds", "empty_realtime.sdf"]
                ),
                description="Path to the SDF world file.",
            ),
            # Gazebo
            gz_sim,
            clock_bridge,
            robot_state_publisher,
            TimerAction(period=3.0, actions=[spawn_entity]),
            # Spawn controllers for Gazebo in /sim namespace
            TimerAction(period=5.0, actions=[joint_broadcaster_spawner]),
            TimerAction(period=6.0, actions=[omni_controller_spawner]),
            # Teleop + Relay + Ramp
            TimerAction(period=7.0, actions=[joy_node]),
            TimerAction(period=7.0, actions=[teleop_node]),
            TimerAction(period=7.0, actions=[cmd_vel_relay]),
            TimerAction(period=7.0, actions=[sim_velocity_ramp]),
            # RViz
            rviz_node,
        ]
    )
