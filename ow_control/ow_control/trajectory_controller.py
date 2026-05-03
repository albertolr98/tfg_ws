#!/usr/bin/env python3
"""
Controlador de trayectorias con joystick y teleoperación integrada.

Permite ejecutar trayectorias predefinidas (cuadrado, círculo, triángulo, X)
mediante botones del mando PlayStation, con control manual mediante sticks
analógicos. Incluye deadman switch de seguridad.

Autor: Alberto López
"""

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from enum import Enum, auto


class TrajectoryType(Enum):
    """Tipos de trayectorias disponibles."""

    NONE = auto()
    SQUARE = auto()
    CIRCLE = auto()
    TRIANGLE = auto()
    X_PATTERN = auto()


class TrajectoryState(Enum):
    """Estados de la máquina de estados."""

    IDLE = auto()
    MOVING = auto()
    PAUSED = auto()


class PS5Controller:
    """Mapeo de botones y ejes del mando DualSense (PS5)."""

    BUTTON_CROSS = 0
    BUTTON_CIRCLE = 1
    BUTTON_SQUARE = 2
    BUTTON_TRIANGLE = 3
    BUTTON_L1 = 9

    AXIS_LEFT_X = 0
    AXIS_LEFT_Y = 1
    AXIS_RIGHT_X = 2

    DEADZONE = 0.15


class TrajectoryControllerNode(Node):
    """Controlador de trayectorias con joystick y deadman switch."""

    def __init__(self):
        super().__init__("trajectory_controller")
        self._declare_parameters()
        self._init_state()
        self._init_ros_interfaces()
        self._log_startup_info()

    def _declare_parameters(self):
        """Declara los parámetros configurables del nodo."""
        self.declare_parameter("side_length", 0.5)
        self.declare_parameter("circle_radius", 0.25)
        self.declare_parameter("velocity", 0.2)
        self.declare_parameter("max_linear_velocity", 0.5)
        self.declare_parameter("max_angular_velocity", 3.0)
        self.declare_parameter("pause_time", 1.0)
        self.declare_parameter("position_tolerance", 0.02)

        self.side_length = self.get_parameter("side_length").value
        self.circle_radius = self.get_parameter("circle_radius").value
        self.velocity = self.get_parameter("velocity").value
        self.max_linear_vel = self.get_parameter("max_linear_velocity").value
        self.max_angular_vel = self.get_parameter("max_angular_velocity").value
        self.pause_time = self.get_parameter("pause_time").value
        self.position_tolerance = self.get_parameter("position_tolerance").value

    def _init_state(self):
        """Inicializa las variables de estado."""
        self.trajectory_type = TrajectoryType.NONE
        self.state = TrajectoryState.IDLE
        self.segment_index = 0
        self.start_x = 0.0
        self.start_y = 0.0
        self.pause_start_time = None
        self.circle_distance = 0.0
        self.last_x = 0.0
        self.last_y = 0.0
        self.waypoints = []

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0
        self.odom_received = False

        self.last_buttons = [0] * 20
        self.manual_vx = 0.0
        self.manual_vy = 0.0
        self.manual_wz = 0.0
        self.deadman_pressed = False
        self.has_manual_input = False

    def _init_ros_interfaces(self):
        """Inicializa suscriptores, publicadores y timers."""
        self.odom_sub = self.create_subscription(
            Odometry, "/omni_wheel_drive_controller/odom", self._odom_callback, 10
        )
        self.joy_sub = self.create_subscription(Joy, "/joy", self._joy_callback, 10)
        self.cmd_vel_pub = self.create_publisher(
            TwistStamped, "/omni_wheel_drive_controller/cmd_vel", 10
        )
        self.control_timer = self.create_timer(0.05, self._control_loop)

    def _log_startup_info(self):
        """Muestra información de inicio."""
        self.get_logger().info("Controlador de trayectorias iniciado.")
        self.get_logger().info("L1 = DEADMAN SWITCH (mantener pulsado)")
        self.get_logger().info("Botones: ▢=Cuadrado, ○=Círculo, △=Triángulo, ✕=X")

    def _odom_callback(self, msg: Odometry):
        """Actualiza la posición y orientación del robot desde odometría."""
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)
        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info("Odometría recibida. Listo.")

    def _apply_deadzone(self, value: float) -> float:
        """Aplica zona muerta a un valor de eje analógico."""
        if abs(value) < PS5Controller.DEADZONE:
            return 0.0
        sign = 1.0 if value > 0 else -1.0
        return (
            sign
            * (abs(value) - PS5Controller.DEADZONE)
            / (1.0 - PS5Controller.DEADZONE)
        )

    def _joy_callback(self, msg: Joy):
        """Procesa entrada del joystick."""
        if len(msg.buttons) < 10 or len(msg.axes) < 4:
            return

        self._process_deadman_switch(msg)
        self._process_manual_control(msg)
        self._process_trajectory_buttons(msg)
        self.last_buttons = list(msg.buttons)

    def _process_deadman_switch(self, msg: Joy):
        """Procesa el estado del deadman switch (L1)."""
        was_pressed = self.deadman_pressed
        self.deadman_pressed = msg.buttons[PS5Controller.BUTTON_L1] == 1

        if was_pressed and not self.deadman_pressed:
            if self.trajectory_type != TrajectoryType.NONE:
                self.get_logger().info("L1 soltado - Trayectoria cancelada.")
            self._cancel_trajectory()

    def _process_manual_control(self, msg: Joy):
        """Procesa el control manual con sticks analógicos."""
        self.manual_vx = (
            self._apply_deadzone(msg.axes[PS5Controller.AXIS_LEFT_Y])
            * self.max_linear_vel
        )
        self.manual_vy = (
            self._apply_deadzone(msg.axes[PS5Controller.AXIS_LEFT_X])
            * self.max_linear_vel
        )
        self.manual_wz = (
            self._apply_deadzone(msg.axes[PS5Controller.AXIS_RIGHT_X])
            * self.max_angular_vel
        )
        self.has_manual_input = any(
            abs(v) > 0.01 for v in [self.manual_vx, self.manual_vy, self.manual_wz]
        )

    def _process_trajectory_buttons(self, msg: Joy):
        """Procesa los botones de inicio de trayectorias."""
        if not self.deadman_pressed:
            return

        button_map = {
            PS5Controller.BUTTON_SQUARE: TrajectoryType.SQUARE,
            PS5Controller.BUTTON_CIRCLE: TrajectoryType.CIRCLE,
            PS5Controller.BUTTON_TRIANGLE: TrajectoryType.TRIANGLE,
            PS5Controller.BUTTON_CROSS: TrajectoryType.X_PATTERN,
        }

        for button, traj_type in button_map.items():
            if msg.buttons[button] == 1 and self.last_buttons[button] == 0:
                self._start_trajectory(traj_type)
                break

    def _start_trajectory(self, traj_type: TrajectoryType):
        """Inicia una nueva trayectoria."""
        if not self.odom_received:
            self.get_logger().warn("Esperando odometría...")
            return

        self.trajectory_type = traj_type
        self.segment_index = 0
        self.circle_distance = 0.0
        self.last_x = self.current_x
        self.last_y = self.current_y

        names = {
            TrajectoryType.SQUARE: "CUADRADO",
            TrajectoryType.CIRCLE: "CÍRCULO",
            TrajectoryType.TRIANGLE: "TRIÁNGULO",
            TrajectoryType.X_PATTERN: "X",
        }
        self.get_logger().info(f"Iniciando trayectoria: {names[traj_type]}")
        self._generate_waypoints(traj_type)
        self._start_segment()

    def _generate_waypoints(self, traj_type: TrajectoryType):
        """Genera los waypoints absolutos para la trayectoria seleccionada."""
        self.waypoints = []
        x, y = self.current_x, self.current_y

        if traj_type == TrajectoryType.SQUARE:
            # Cuadrado: +X, +Y, -X, -Y
            self.waypoints = [
                (x + self.side_length, y),
                (x + self.side_length, y + self.side_length),
                (x, y + self.side_length),
                (x, y),
            ]
        elif traj_type == TrajectoryType.TRIANGLE:
            # Triángulo equilátero con base horizontal desplazada
            h = self.side_length * math.sqrt(3) / 2
            self.waypoints = [
                (x, y + self.side_length),
                (x + h, y + self.side_length / 2),
                (x, y),
            ]
        elif traj_type == TrajectoryType.X_PATTERN:
            # Lazo en X
            self.waypoints = [
                (x + self.side_length, y - self.side_length),
                (x, y - self.side_length),
                (x + self.side_length, y),
                (x, y),
            ]
        elif traj_type == TrajectoryType.CIRCLE:
            # Círculo: generamos N puntos para mayor precisión
            num_points = 36
            cx, cy = x, y + self.circle_radius
            for i in range(1, num_points + 1):
                angle = -math.pi / 2 + (2 * math.pi * i / num_points)
                px = cx + self.circle_radius * math.cos(angle)
                py = cy + self.circle_radius * math.sin(angle)
                self.waypoints.append((px, py))

    def _cancel_trajectory(self):
        """Cancela la trayectoria actual."""
        self.trajectory_type = TrajectoryType.NONE
        self.state = TrajectoryState.IDLE
        self._stop_robot()

    def _start_segment(self):
        """Inicia un nuevo segmento de trayectoria."""
        self.start_x = self.current_x
        self.start_y = self.current_y
        self.state = TrajectoryState.MOVING

    def _start_pause(self):
        """Inicia una pausa entre segmentos."""
        self.state = TrajectoryState.PAUSED
        self.pause_start_time = self.get_clock().now()
        self._stop_robot()

    def _get_distance_traveled(self) -> float:
        """Calcula la distancia recorrida desde el inicio del segmento."""
        dx = self.current_x - self.start_x
        dy = self.current_y - self.start_y
        return math.sqrt(dx * dx + dy * dy)

    def _get_pause_elapsed(self) -> float:
        """Retorna el tiempo transcurrido en pausa."""
        if self.pause_start_time is None:
            return 0.0
        return (self.get_clock().now() - self.pause_start_time).nanoseconds / 1e9

    def _publish_velocity(self, vx: float = 0.0, vy: float = 0.0, wz: float = 0.0):
        """Publica un comando de velocidad."""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.angular.z = wz
        self.cmd_vel_pub.publish(msg)

    def _stop_robot(self):
        """Detiene el robot."""
        self._publish_velocity()

    def _get_velocity_to_target(
        self, tx: float, ty: float
    ) -> tuple[float, float, bool]:
        """Calcula velocidades para ir a un objetivo y retorna si ha llegado."""
        dx = tx - self.current_x
        dy = ty - self.current_y
        dist = math.sqrt(dx * dx + dy * dy)

        if dist < self.position_tolerance:
            return 0.0, 0.0, True

        current_vel = self.velocity
        if dist < 0.05:
            current_vel = max(0.05, self.velocity * (dist / 0.05))

        # Velocidad en frame mundo
        vx_world = (dx / dist) * current_vel
        vy_world = (dy / dist) * current_vel

        # Transformar a frame cuerpo (base_link) usando el yaw actual
        cos_yaw = math.cos(self.current_yaw)
        sin_yaw = math.sin(self.current_yaw)
        vx_body =  vx_world * cos_yaw + vy_world * sin_yaw
        vy_body = -vx_world * sin_yaw + vy_world * cos_yaw

        return vx_body, vy_body, False

    def _control_loop(self):
        """Bucle de control principal."""
        if not self.deadman_pressed:
            self._stop_robot()
            return

        if self.has_manual_input:
            self._publish_velocity(self.manual_vx, self.manual_vy, self.manual_wz)
            return

        if not self.odom_received or self.trajectory_type == TrajectoryType.NONE:
            self._stop_robot()
            return

        if self.state == TrajectoryState.PAUSED:
            self._stop_robot()
            if self._get_pause_elapsed() >= self.pause_time:
                self._start_segment()
            return

        trajectory_handlers = {
            TrajectoryType.SQUARE: self._execute_square,
            TrajectoryType.CIRCLE: self._execute_circle,
            TrajectoryType.TRIANGLE: self._execute_triangle,
            TrajectoryType.X_PATTERN: self._execute_x_pattern,
        }
        trajectory_handlers.get(self.trajectory_type, lambda: None)()

    def _execute_square(self):
        """Ejecuta trayectoria cuadrada basada en waypoints."""
        if self.segment_index >= len(self.waypoints):
            self.get_logger().info("¡Cuadrado completado!")
            self._cancel_trajectory()
            return

        target = self.waypoints[self.segment_index]
        vx, vy, arrived = self._get_velocity_to_target(target[0], target[1])

        if arrived:
            self.segment_index += 1
            if self.segment_index >= len(self.waypoints):
                self.get_logger().info("¡Cuadrado completado!")
                self._cancel_trajectory()
            else:
                self._start_pause()
        else:
            self._publish_velocity(vx, vy)

    def _execute_circle(self):
        """Ejecuta trayectoria circular basada en waypoints."""
        if self.segment_index >= len(self.waypoints):
            self.get_logger().info("¡Círculo completado!")
            self._cancel_trajectory()
            return

        target = self.waypoints[self.segment_index]
        vx, vy, arrived = self._get_velocity_to_target(target[0], target[1])

        if arrived:
            self.segment_index += 1
            # Para el círculo no hacemos pausas entre puntos para suavidad
            if self.segment_index >= len(self.waypoints):
                self.get_logger().info("¡Círculo completado!")
                self._cancel_trajectory()
        else:
            self._publish_velocity(vx, vy)

    def _execute_triangle(self):
        """Ejecuta trayectoria triangular basada en waypoints."""
        if self.segment_index >= len(self.waypoints):
            self.get_logger().info("¡Triángulo completado!")
            self._cancel_trajectory()
            return

        target = self.waypoints[self.segment_index]
        vx, vy, arrived = self._get_velocity_to_target(target[0], target[1])

        if arrived:
            self.segment_index += 1
            if self.segment_index >= len(self.waypoints):
                self.get_logger().info("¡Triángulo completado!")
                self._cancel_trajectory()
            else:
                self._start_pause()
        else:
            self._publish_velocity(vx, vy)

    def _execute_x_pattern(self):
        """Ejecuta trayectoria en forma de X basada en waypoints."""
        if self.segment_index >= len(self.waypoints):
            self.get_logger().info("¡X completada!")
            self._cancel_trajectory()
            return

        target = self.waypoints[self.segment_index]
        vx, vy, arrived = self._get_velocity_to_target(target[0], target[1])

        if arrived:
            self.segment_index += 1
            if self.segment_index >= len(self.waypoints):
                self.get_logger().info("¡X completada!")
                self._cancel_trajectory()
            else:
                self._start_pause()
        else:
            self._publish_velocity(vx, vy)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Interrumpido por el usuario")
    finally:
        if rclpy.ok():
            node._stop_robot()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
