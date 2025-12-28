#!/usr/bin/env python3
"""
Command Velocity Relay Node.

Subscribes to /cmd_vel and republishes to multiple controller topics,
allowing simultaneous control of real robot and simulation for comparison.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped


class CmdVelRelay(Node):
    """Relays cmd_vel to multiple controller topics."""

    def __init__(self):
        super().__init__("cmd_vel_relay")

        # Declare parameters for target topics
        self.declare_parameter("targets", ["/omni_wheel_drive_controller/cmd_vel"])
        self.declare_parameter("use_stamped", True)

        targets = self.get_parameter("targets").get_parameter_value().string_array_value
        use_stamped = self.get_parameter("use_stamped").get_parameter_value().bool_value

        # Create publishers for each target
        self._pubs = []
        msg_type = TwistStamped if use_stamped else Twist
        for target in targets:
            pub = self.create_publisher(msg_type, target, 10)
            self._pubs.append(pub)
            self.get_logger().info(f"Relaying to: {target}")

        # Subscribe to /cmd_vel
        self.subscription = self.create_subscription(
            msg_type,
            "/cmd_vel",
            self.cmd_vel_callback,
            10,
        )
        self.get_logger().info("CmdVelRelay ready - listening on /cmd_vel")

    def cmd_vel_callback(self, msg):
        """Republish to all targets."""
        for pub in self._pubs:
            pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
