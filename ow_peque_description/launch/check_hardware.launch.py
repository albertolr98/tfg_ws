from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 1. Procesar la descripción del robot (URDF) con sim_mode:=false
    # Es vital que sim_mode sea false para cargar tu plugin de hardware real
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution(
                [FindPackageShare("ow_peque_description"), "urdf", "ow.urdf.xacro"]
            ),
            " ", "sim_mode:=false",
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # 2. Configuración de los controladores
    robot_controllers = PathJoinSubstitution(
        [FindPackageShare("ow_peque_description"), "config", "controllers.yaml"]
    )

    # 3. Nodo Robot State Publisher (¡FALTABA ESTE!)
    # Publica el URDF en el tópico /robot_description y las transformaciones estáticas
    node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    # 4. Nodo Controller Manager (El núcleo de ros2_control)
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        output="screen",
    )

    # 5. Spawners para los controladores
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad", "--controller-manager", "/controller_manager"],
    )

    omni_wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["omni_wheel_drive_controller", "--controller-manager", "/controller_manager"],
    )

    # Asegura que el controlador omni arranque después del joint broadcaster
    delay_omni_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[omni_wheel_controller_spawner],
        )
    )

    # 6. Nodo de Rampa Lineal (Suavizado de velocidad)
    # Intercepta /cmd_vel_input y publica en /cmd_vel suavizado
    linear_ramp_node = Node(
        package="ow_control",
        executable="linear_ramp",
        name="linear_ramp_node",
        remappings=[
            ("cmd_vel_out", "/omni_wheel_drive_controller/cmd_vel"),
        ]
    )

    return LaunchDescription([
        node_robot_state_publisher,  # <--- Añadido aquí
        control_node,
        joint_state_broadcaster_spawner,
        delay_omni_controller,
        linear_ramp_node,
    ])
