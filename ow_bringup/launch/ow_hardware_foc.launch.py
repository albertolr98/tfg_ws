import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Get the path to the URDF file
    urdf_path = PathJoinSubstitution(
        [FindPackageShare("ow_peque_description"), "urdf", "ow.urdf.xacro"]
    )

    # Get the path to the controllers file
    controllers_path = PathJoinSubstitution(
        [FindPackageShare("ow_peque_description"), "config", "controllers.yaml"]
    )

    # Process the URDF file
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            urdf_path,
            " ",
            "sim_mode:=false",
        ]
    )

    # Create the robot state publisher node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description_content}],
        output="screen",
    )

    # Create the controller manager node
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[controllers_path],
        output="screen",
    )

    # Create the joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # Create the omni wheel drive controller spawner
    omni_wheel_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["omni_wheel_drive_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # --- New Nodes for FOC ---

    # Joy Node
    joy_node = Node(
        package="joy",
        executable="joy_node",
        output="screen",
    )

    # Teleop Node
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
    )

    # Velocity Bridge (Linear Ramp + FOC)
    velocity_bridge = Node(
        package="ow_control",
        executable="linear_ramp",
        name="velocity_bridge",
        remappings=[
            ("cmd_vel_input", "/cmd_vel"),
            ("cmd_vel_out", "/omni_wheel_drive_controller/cmd_vel"),
        ],
        output="screen",
    )

    # Placeholder for IMU Driver
    # IMPORTANT: You must run your IMU driver separately or uncomment/configure this block
    # Ensure it publishes sensor_msgs/Imu messages to the /imu topic
    """
    imu_node = Node(
        package="your_imu_package",
        executable="your_imu_driver",
        output="screen",
        remappings=[("imu_data", "/imu")],
    )
    """

    # Create the launch description
    ld = LaunchDescription()

    # Add the nodes to the launch description
    ld.add_action(robot_state_publisher_node)
    ld.add_action(controller_manager_node)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(omni_wheel_drive_controller_spawner)
    
    ld.add_action(joy_node)
    ld.add_action(teleop_node)
    ld.add_action(velocity_bridge)
    # ld.add_action(imu_node)

    return ld
