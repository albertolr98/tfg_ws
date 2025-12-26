from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import PushRosNamespace, SetRemap
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Include the standard simulation launch file
    # But wrap it in a namespace and remappings
    
    sim_launch = GroupAction(
        actions=[
            PushRosNamespace("sim"),
            SetRemap("/sim/cmd_vel_input", "/cmd_vel_input"), # Listen to global input
            
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution(
                        [FindPackageShare("ow_peque_description"), "launch", "ow_peque_sim.launch.py"]
                    )
                ),
                launch_arguments={
                    "launch_teleop": "false", # Disable sim teleop to avoid conflict
                    "frame_prefix": "sim_",   # Prefix TF frames
                    "entity_name": "ow_peque_sim",
                    "controller_manager": "/sim/controller_manager", # Namespaced CM
                    "cmd_vel_input_topic": "/cmd_vel_input", # Global joystick input
                    "cmd_vel_output_topic": "/sim/omni_wheel_drive_controller/cmd_vel", # Namespaced controller
                    "namespace": "/sim",
                    "config_file": PathJoinSubstitution(
                        [FindPackageShare("ow_peque_description"), "config", "controllers_sim.yaml"]
                    ),
                }.items(),
            )
        ]
    )

    return LaunchDescription([
        sim_launch,
    ])
