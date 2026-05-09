#!/usr/bin/env python3
"""
Controlador de trayectorias con joystick y teleoperación integrada.

Permite ejecutar trayectorias predefinidas (cuadrado, círculo, triángulo, X)
mediante botones del mando PlayStation, con control manual mediante sticks
analógicos. Incluye deadman switch de seguridad.

Autor: Alberto López
"""

import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from enum import Enum, auto


@dataclass
class Waypoint:
    x: float
    y: float
    is_stop: bool = True
    # Ángulo objetivo relativo al yaw inicial de la trayectoria.
    # None = sin control de orientación en este waypoint.
    target_yaw: Optional[float] = None


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
        self.declare_parameter("path_tolerance", 0.04)
        self.declare_parameter("min_velocity", 0.03)
        self.declare_parameter("k_pos", 1.2)
        self.declare_parameter("k_yaw", 2.0)
        self.declare_parameter("angular_tolerance", 0.05)

        self.side_length = self.get_parameter("side_length").value
        self.circle_radius = self.get_parameter("circle_radius").value
        self.velocity = self.get_parameter("velocity").value
        self.max_linear_vel = self.get_parameter("max_linear_velocity").value
        self.max_angular_vel = self.get_parameter("max_angular_velocity").value
        self.pause_time = self.get_parameter("pause_time").value
        self.position_tolerance = self.get_parameter("position_tolerance").value
        self.path_tolerance = self.get_parameter("path_tolerance").value
        self.min_velocity = self.get_parameter("min_velocity").value
        self.k_pos = self.get_parameter("k_pos").value
        self.k_yaw = self.get_parameter("k_yaw").value
        self.angular_tolerance = self.get_parameter("angular_tolerance").value

    def _init_state(self):
        """Inicializa las variables de estado."""
        self.trajectory_type = TrajectoryType.NONE
        self.state = TrajectoryState.IDLE
        self.segment_index = 0
        self.start_x = 0.0
        self.start_y = 0.0
        self.start_yaw = 0.0
        self.pause_start_time = None
        self.last_x = 0.0
        self.last_y = 0.0
        self.waypoints: list[Waypoint] = []

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
        self.last_x = self.current_x
        self.last_y = self.current_y
        self.start_yaw = self.current_yaw

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
        """Genera los waypoints para la trayectoria seleccionada.

        Cada Waypoint indica si el robot debe detenerse (is_stop) y el ángulo
        objetivo relativo al yaw inicial (target_yaw=None para sin restricción).
        Los waypoints de forma (is_stop=False) definen la trayectoria pero el
        robot los atraviesa a velocidad plena sin detenerse.
        """
        self.waypoints = []
        x, y = self.current_x, self.current_y

        if traj_type == TrajectoryType.SQUARE:
            self.waypoints = [
                Waypoint(x + self.side_length, y, is_stop=True, target_yaw=0.0),
                Waypoint(x + self.side_length, y + self.side_length, is_stop=True, target_yaw=0.0),
                Waypoint(x, y + self.side_length, is_stop=True, target_yaw=0.0),
                Waypoint(x, y, is_stop=True, target_yaw=0.0),
            ]
        elif traj_type == TrajectoryType.TRIANGLE:
            h = self.side_length * math.sqrt(3) / 2
            self.waypoints = [
                Waypoint(x, y + self.side_length, is_stop=True, target_yaw=0.0),
                Waypoint(x + h, y + self.side_length / 2, is_stop=True, target_yaw=0.0),
                Waypoint(x, y, is_stop=True, target_yaw=0.0),
            ]
        elif traj_type == TrajectoryType.X_PATTERN:
            self.waypoints = [
                Waypoint(x + self.side_length, y - self.side_length, is_stop=True, target_yaw=0.0),
                Waypoint(x, y - self.side_length, is_stop=True, target_yaw=0.0),
                Waypoint(x + self.side_length, y, is_stop=True, target_yaw=0.0),
                Waypoint(x, y, is_stop=True, target_yaw=0.0),
            ]
        elif traj_type == TrajectoryType.CIRCLE:
            num_points = 36
            cx, cy = x, y + self.circle_radius
            for i in range(1, num_points + 1):
                angle = -math.pi / 2 + (2 * math.pi * i / num_points)
                px = cx + self.circle_radius * math.cos(angle)
                py = cy + self.circle_radius * math.sin(angle)
                is_last = i == num_points
                self.waypoints.append(
                    Waypoint(px, py, is_stop=is_last, target_yaw=0.0 if is_last else None)
                )

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

    def _get_velocity_to_target(self, wp: Waypoint) -> tuple[float, float, float, bool]:
        """Calcula velocidades para ir al waypoint y retorna si ha llegado.

        Para waypoints de parada (is_stop=True): velocidad proporcional a la
        distancia y tolerancia de posición ajustada. Para waypoints de forma
        (is_stop=False): velocidad plena y tolerancia ampliada.
        El control angular se aplica simultáneamente a la traslación cuando
        el waypoint define target_yaw.
        """
        dx = wp.x - self.current_x
        dy = wp.y - self.current_y
        dist = math.sqrt(dx * dx + dy * dy)

        tol = self.position_tolerance if wp.is_stop else self.path_tolerance
        pos_arrived = dist < tol

        vx, vy = 0.0, 0.0
        if not pos_arrived:
            if wp.is_stop:
                current_vel = min(self.velocity, max(self.min_velocity, self.k_pos * dist))
            else:
                current_vel = self.velocity
            vx_world = (dx / dist) * current_vel
            vy_world = (dy / dist) * current_vel
            cos_yaw = math.cos(self.current_yaw)
            sin_yaw = math.sin(self.current_yaw)
            vx =  vx_world * cos_yaw + vy_world * sin_yaw
            vy = -vx_world * sin_yaw + vy_world * cos_yaw

        wz = 0.0
        yaw_arrived = True
        if wp.target_yaw is not None:
            target_yaw_abs = self.start_yaw + wp.target_yaw
            yaw_err = math.atan2(
                math.sin(target_yaw_abs - self.current_yaw),
                math.cos(target_yaw_abs - self.current_yaw),
            )
            yaw_arrived = abs(yaw_err) < self.angular_tolerance
            if not yaw_arrived:
                wz = max(-self.max_angular_vel, min(self.max_angular_vel, self.k_yaw * yaw_err))

        return vx, vy, wz, pos_arrived and yaw_arrived

    def _get_trajectory_name(self) -> str:
        names = {
            TrajectoryType.SQUARE: "Cuadrado",
            TrajectoryType.CIRCLE: "Círculo",
            TrajectoryType.TRIANGLE: "Triángulo",
            TrajectoryType.X_PATTERN: "X",
        }
        return names.get(self.trajectory_type, "Trayectoria")

    def _execute_current_trajectory(self):
        """Ejecutor genérico de trayectorias por waypoints.

        Itera waypoints consecutivos de forma (is_stop=False) sin pausa entre
        ellos, evitando el hueco de un tick que causaría tirones en el círculo.
        """
        while self.segment_index < len(self.waypoints):
            wp = self.waypoints[self.segment_index]
            vx, vy, wz, arrived = self._get_velocity_to_target(wp)

            if not arrived:
                self._publish_velocity(vx, vy, wz)
                return

            self.segment_index += 1
            if self.segment_index >= len(self.waypoints):
                self.get_logger().info(f"¡{self._get_trajectory_name()} completado!")
                self._cancel_trajectory()
                return

            if wp.is_stop:
                self._start_pause()
                return

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

        self._execute_current_trajectory()


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
