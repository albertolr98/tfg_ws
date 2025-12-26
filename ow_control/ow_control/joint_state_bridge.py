#!/usr/bin/env python3
"""
Joint State Bridge Node for Digital Twin.

This node bridges joint states from the real robot to Gazebo,
allowing the simulation to mirror the physical robot's pose.

Subscriptions:
    - joint_states_in (sensor_msgs/JointState): Joint states from real robot

Publications:
    - joint_states_out (sensor_msgs/JointState): Forwarded joint states (for sim)

Note: For full Gazebo integration, this node should also set joint positions
via gz-transport. Current implementation just forwards the messages.
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState


class JointStateBridge(Node):
    """Bridges joint states from real robot to simulation."""

    def __init__(self):
        super().__init__("joint_state_bridge")

        # QoS for sensor data
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Subscriber to real robot joint states
        self.subscription = self.create_subscription(
            JointState,
            "joint_states_in",
            self.joint_state_callback,
            qos,
        )

        # Publisher to simulation
        self.publisher = self.create_publisher(
            JointState,
            "joint_states_out",
            qos,
        )

        # Track connection status
        self._last_msg_time = None
        self._connected = False

        # Status timer
        self.create_timer(5.0, self.status_callback)

        self.get_logger().info("Joint State Bridge initialized")
        self.get_logger().info("  Subscribing to: joint_states_in")
        self.get_logger().info("  Publishing to:  joint_states_out")

    def joint_state_callback(self, msg: JointState):
        """Forward joint states from real robot to simulation."""
        # Update connection status
        if not self._connected:
            self.get_logger().info("Receiving joint states from real robot")
            self._connected = True

        self._last_msg_time = self.get_clock().now()

        # Forward the message
        self.publisher.publish(msg)

    def status_callback(self):
        """Periodic status check."""
        if self._last_msg_time is None:
            self.get_logger().warn("No joint states received from real robot yet")
            return

        # Check for stale data
        elapsed = (self.get_clock().now() - self._last_msg_time).nanoseconds / 1e9
        if elapsed > 2.0:
            self.get_logger().warn(
                f"No joint states received for {elapsed:.1f}s - robot disconnected?"
            )
            self._connected = False


def main(args=None):
    rclpy.init(args=args)
    node = JointStateBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
