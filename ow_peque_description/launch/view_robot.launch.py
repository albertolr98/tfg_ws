from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Launch file para visualizar y editar el URDF del robot.
    
    Lanza:
    - robot_state_publisher: Publica el URDF y las transformaciones TF
    - joint_state_publisher_gui: GUI para mover los joints interactivamente
    - rviz2: Visualización 3D del robot
    - rqt_graph: Visualización del grafo de nodos ROS
    """
    
    # Configuración de RViz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ow_peque_description"), "rviz", "urdf_config.rviz"]
    )

    # Robot description desde xacro
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution(
                [FindPackageShare("ow_peque_description"), "urdf", "ow.urdf.xacro"]
            ),
            " ", "sim_mode:=true",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Robot State Publisher - publica el URDF y las transformaciones TF
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # Joint State Publisher GUI - permite mover los joints desde una GUI
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )

    # RViz2 - visualización 3D
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    # RQt Graph - visualización del grafo de nodos
    rqt_graph_node = Node(
        package="rqt_graph",
        executable="rqt_graph",
        name="rqt_graph",
        output="screen",
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
        rqt_graph_node,
    ])
