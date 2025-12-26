#!/usr/bin/env python3
"""
Digital Twin Launch File for Gazebo.
Runs on: PC (Operator Station)

This launch file starts Gazebo as a digital twin that mirrors the real robot state.
The robot in Gazebo reflects positions received from the real robot's /joint_states.

Key differences from ow_peque_sim.launch.py:
- Uses use_sim_time:=false (wall clock, not simulation time)
- Uses sim_mode:=false (no gz_ros2_control plugin - we don't control, just mirror)
- Robot state publisher subscribes to /joint_states from real robot
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
    # Launch arguments
    gz_args = LaunchConfiguration("gz_args")
    world = LaunchConfiguration("world")
    entity_name = LaunchConfiguration("entity_name")

    # Robot description WITHOUT gz_ros2_control (sim_mode:=false)
    # We don't need control - the digital twin just mirrors the real robot
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("ow_peque_description"), "urdf", "ow.urdf.xacro"]
            ),
            " ",
            "sim_mode:=false",  # No Gazebo control plugin needed
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

    # Robot State Publisher in 'sim' namespace
    # This publishes TF transforms based on /joint_states from real robot
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="sim",
        parameters=[
            {"use_sim_time": False},
            {"robot_description": robot_description_content},
        ],
        # Remap to listen to real robot's joint states
        remappings=[
            ("joint_states", "/joint_states"),
        ],
        output="screen",
    )

    # Clock bridge (for Gazebo time reference)
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    # Spawn robot entity in Gazebo
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", entity_name,
            "-topic", "/sim/robot_description",
        ],
        output="screen",
    )

    # Joint State Bridge: forwards joint positions to Gazebo model
    # This uses gz-transport to set joint positions in the spawned model
    joint_state_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/joint_states@sensor_msgs/msg/JointState]gz.msgs.Model",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            # Arguments
            DeclareLaunchArgument(
                "gz_args",
                default_value="-r -v 2",
                description="Arguments for gz sim (default: run, low verbosity).",
            ),
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("ow_peque_description"), "worlds", "empty_realtime.sdf"]
                ),
                description="Path to the optimized SDF world file.",
            ),
            DeclareLaunchArgument(
                "entity_name",
                default_value="ow_peque_twin",
                description="Name for the spawned Gazebo entity.",
            ),
            # Launch Gazebo
            gz_sim,
            clock_bridge,
            # Robot state publisher (mirrors real robot TF)
            robot_state_publisher,
            # Spawn robot in Gazebo
            TimerAction(period=3.0, actions=[spawn_entity]),
            # Bridge joint states to Gazebo
            TimerAction(period=4.0, actions=[joint_state_bridge]),
        ]
    )

