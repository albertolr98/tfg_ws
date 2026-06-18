import math
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped


class LinearRampNode(Node):
    """Replica software de la rampa de aceleración del TMC5160, operando al nivel de rueda."""

    def __init__(self):
        super().__init__("linear_ramp_node")
        self._declare_parameters()
        self._build_kinematics()
        self._init_state()
        self._init_ros_interfaces()

    def _declare_parameters(self):
        self.declare_parameter("ramp_time", 0.5)
        self.declare_parameter("frequency", 20.0)
        self.declare_parameter("robot_radius", 0.21)
        self.declare_parameter("wheel_radius", 0.05)
        self.declare_parameter("wheel_offset", 0.0)
        self.declare_parameter("num_wheels", 3)

        self.ramp_time = self.get_parameter("ramp_time").value
        self.frequency = self.get_parameter("frequency").value
        self.robot_radius = self.get_parameter("robot_radius").value
        self.wheel_radius = self.get_parameter("wheel_radius").value
        self.wheel_offset = self.get_parameter("wheel_offset").value
        self.num_wheels = self.get_parameter("num_wheels").value

        self.total_steps = self.frequency * self.ramp_time
        self.step_period = 1.0 / self.frequency

    def _build_kinematics(self):
        """Construye las matrices de cinemática directa e inversa."""
        angle_bw_wheels = 2.0 * math.pi / self.num_wheels
        # Matriz cinemática inversa A: ω = A @ V / r_w
        # Fila i: [sin(θ_i), -cos(θ_i), -R]
        A = np.zeros((self.num_wheels, 3))
        for i in range(self.num_wheels):
            theta = angle_bw_wheels * i + self.wheel_offset
            A[i, 0] = math.sin(theta)
            A[i, 1] = -math.cos(theta)
            A[i, 2] = -self.robot_radius
        self._A = A
        # Pseudoinversa para la cinemática directa: V = A_pinv @ (ω * r_w)
        self._A_pinv = np.linalg.pinv(A)

    def _init_state(self):
        self.current_wheel_vel = np.zeros(self.num_wheels)
        self.target_wheel_vel = np.zeros(self.num_wheels)
        self.increments = np.zeros(self.num_wheels)

    def _init_ros_interfaces(self):
        self.sub = self.create_subscription(
            TwistStamped, "cmd_vel_input", self._cmd_vel_callback, 10
        )
        self.pub = self.create_publisher(TwistStamped, "cmd_vel_out", 10)
        self.timer = self.create_timer(self.step_period, self._control_loop)

    def _body_to_wheel(self, vx: float, vy: float, wz: float) -> np.ndarray:
        """Cinemática inversa: velocidades de cuerpo → velocidades angulares de rueda (rad/s)."""
        V = np.array([vx, vy, wz])
        return (self._A @ V) / self.wheel_radius

    def _wheel_to_body(self, wheel_vel: np.ndarray) -> np.ndarray:
        """Cinemática directa: velocidades angulares de rueda → velocidades de cuerpo."""
        return self._A_pinv @ (wheel_vel * self.wheel_radius)

    def _step(self, current: float, target: float, increment: float) -> float:
        remaining = target - current
        if abs(remaining) > abs(increment):
            return current + increment
        return target

    def _cmd_vel_callback(self, msg: TwistStamped):
        self.target_wheel_vel = self._body_to_wheel(
            msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z
        )
        self.increments = (self.target_wheel_vel - self.current_wheel_vel) / self.total_steps

    def _control_loop(self):
        for i in range(self.num_wheels):
            self.current_wheel_vel[i] = self._step(
                self.current_wheel_vel[i], self.target_wheel_vel[i], self.increments[i]
            )

        V = self._wheel_to_body(self.current_wheel_vel)

        out = TwistStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = "base_link"
        out.twist.linear.x = float(V[0])
        out.twist.linear.y = float(V[1])
        out.twist.angular.z = float(V[2])
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LinearRampNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
