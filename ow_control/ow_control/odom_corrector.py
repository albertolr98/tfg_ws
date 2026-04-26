import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomCorrectorNode(Node):
    def __init__(self):
        super().__init__("odom_corrector")

        # Factor que corrige el sobreconteo del encoder durante la rotación.
        # Se mide como: ángulo_físico / ángulo_reportado_por_odom_bruta
        # Ejemplo: robot gira 360° físicamente, odom_bruta reporta 372.86° → factor = 360/372.86
        self.declare_parameter("rotation_slip_factor", 1.0)
        self.declare_parameter("input_topic", "/omni_wheel_drive_controller/odom")
        self.declare_parameter("output_topic", "/odom")
        self.declare_parameter("publish_tf", True)

        self._factor = self.get_parameter("rotation_slip_factor").value
        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._last_stamp = None

        self._sub = self.create_subscription(
            Odometry,
            self.get_parameter("input_topic").value,
            self._odom_cb,
            10,
        )
        self._pub = self.create_publisher(
            Odometry,
            self.get_parameter("output_topic").value,
            10,
        )

        if self.get_parameter("publish_tf").value:
            self._tf_broadcaster = TransformBroadcaster(self)
        else:
            self._tf_broadcaster = None

    def _odom_cb(self, msg: Odometry):
        stamp = rclpy.time.Time.from_msg(msg.header.stamp)

        if self._last_stamp is None:
            self._last_stamp = stamp
            return

        dt = (stamp - self._last_stamp).nanoseconds * 1e-9
        self._last_stamp = stamp

        if dt <= 0.0:
            return

        # Velocidades en el frame del robot; se corrige solo la angular
        vx = msg.twist.twist.linear.x
        vy = msg.twist.twist.linear.y
        omega = msg.twist.twist.angular.z * self._factor

        # Integración exacta por arco (igual que el controlador original)
        dx = vx * dt
        dy = vy * dt
        dtheta = omega * dt
        theta_old = self._theta
        self._theta += dtheta
        if abs(dtheta) < 1e-6:
            self._x += dx * math.cos(theta_old) - dy * math.sin(theta_old)
            self._y += dx * math.sin(theta_old) + dy * math.cos(theta_old)
        else:
            self._x += (dx / dtheta) * (math.sin(self._theta) - math.sin(theta_old)) + (
                dy / dtheta
            ) * (math.cos(self._theta) - math.cos(theta_old))
            self._y += -(dx / dtheta) * (
                math.cos(self._theta) - math.cos(theta_old)
            ) + (dy / dtheta) * (math.sin(self._theta) - math.sin(theta_old))

        qz = math.sin(self._theta / 2.0)
        qw = math.cos(self._theta / 2.0)

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = msg.header.frame_id
        out.child_frame_id = msg.child_frame_id
        out.pose.pose.position.x = self._x
        out.pose.pose.position.y = self._y
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw
        out.twist.twist.linear.x = vx
        out.twist.twist.linear.y = vy
        out.twist.twist.angular.z = omega
        out.pose.covariance = msg.pose.covariance
        out.twist.covariance = msg.twist.covariance
        self._pub.publish(out)

        if self._tf_broadcaster:
            tf = TransformStamped()
            tf.header.stamp = msg.header.stamp
            tf.header.frame_id = msg.header.frame_id
            tf.child_frame_id = msg.child_frame_id
            tf.transform.translation.x = self._x
            tf.transform.translation.y = self._y
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self._tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = OdomCorrectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
