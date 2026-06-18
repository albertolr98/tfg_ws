#!/usr/bin/env python3
"""
Launch de calibración de odometría (simulación).

Arranca en paralelo con ow_sim.launch.py (no lo reemplaza).

Uso:
    ros2 launch ow_peque_bringup cpm_odom_sim.launch.py
    ros2 launch ow_peque_bringup cpm_odom_sim.launch.py gz_model:=ow_peque
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gz_model = LaunchConfiguration("gz_model")

    # Bridge: gz PosePublisher → ROS geometry_msgs/Pose
    # El plugin PosePublisher (en mobile_base_gazebo.xacro) publica a
    # /model/<name>/pose como gz.msgs.Pose. El bridge usa el mismo nombre en ROS.
    gz_pose_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            ["/model/", gz_model, "/pose@geometry_msgs/msg/Pose[gz.msgs.Pose"]
        ],
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    cpm_odom_node = Node(
        package="ow_control",
        executable="cpm_odom",
        parameters=[
            {"use_sim_time": True},
            {"gz_poses_topic": ["/model/", gz_model, "/pose"]},
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "gz_model",
                default_value="ow_peque",
                description="Nombre de la entidad del robot en Gazebo.",
            ),
            gz_pose_bridge,
            cpm_odom_node,
        ]
    )
