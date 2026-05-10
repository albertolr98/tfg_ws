#!/usr/bin/env python3
"""
Herramienta de calibración de odometría.

Compara Gazebo (ground truth via PosePublisher plugin) contra /odom para
tres métricas: rotación, desplazamiento X y desplazamiento Y.

Resumen final:
    ROT Gz/Od: X.XXX | X Gz/Od: X.XXX | Y Gz/Od: X.XXX

Uso típico:
  - Rotación pura  → ajusta robot_radius  = robot_radius / R_rot
  - Traslación Y   → ajusta wheel_radius  = wheel_radius * R_y
  - Traslación X   → igual que Y (misma corrección)

Requiere cpm_odom_sim.launch.py activo (bridge + plugin PosePublisher).
"""

import math
from pathlib import Path

import matplotlib.pyplot as plt
import rclpy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from rclpy.node import Node


class CpmOdomNode(Node):
    def __init__(self):
        super().__init__("cpm_odom")

        self.declare_parameter("gz_poses_topic", "/model/ow_peque/pose")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("plot_output", str(Path.home() / "cpm_odom_plot.png"))

        gz_poses_topic = self.get_parameter("gz_poses_topic").value
        odom_topic = self.get_parameter("odom_topic").value
        self._plot_path = self.get_parameter("plot_output").value

        # --- rotación ---
        self._gz_yaw_prev = None
        self._gz_yaw_total = 0.0
        self._od_yaw_prev = None
        self._od_yaw_total = 0.0

        # --- posición ---
        self._gz_x0 = self._gz_y0 = None
        self._gz_dx = self._gz_dy = 0.0
        self._od_x0 = self._od_y0 = None
        self._od_dx = self._od_dy = 0.0

        # --- series temporales ---
        self._t0 = None
        self._times: list[float] = []
        self._gz_rot: list[float] = []
        self._od_rot: list[float] = []
        self._gz_x: list[float] = []
        self._od_x: list[float] = []
        self._gz_y: list[float] = []
        self._od_y: list[float] = []

        self.create_subscription(Pose, gz_poses_topic, self._gz_cb, 10)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.create_timer(1.0, self._print_status)

        self.get_logger().info(f"GT: {gz_poses_topic} | Odom: {odom_topic}")
        self.get_logger().info("Pulsa Ctrl+C para generar las gráficas.")

    # ------------------------------------------------------------------

    @staticmethod
    def _quat_to_yaw(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _wrap(delta: float) -> float:
        return math.atan2(math.sin(delta), math.cos(delta))

    @staticmethod
    def _ratio_str(gz: float, od: float, threshold: float = 0.05) -> str:
        if abs(od) > threshold:
            return f"{gz / od:.3f}"
        return "---"

    # ------------------------------------------------------------------

    def _gz_cb(self, msg: Pose):
        # posición
        if self._gz_x0 is None:
            self._gz_x0 = msg.position.x
            self._gz_y0 = msg.position.y
            self._t0 = self.get_clock().now().nanoseconds * 1e-9
        self._gz_dx = msg.position.x - self._gz_x0
        self._gz_dy = msg.position.y - self._gz_y0

        # rotación
        yaw = self._quat_to_yaw(msg.orientation)
        if self._gz_yaw_prev is None:
            self._gz_yaw_prev = yaw
            return
        self._gz_yaw_total += self._wrap(yaw - self._gz_yaw_prev)
        self._gz_yaw_prev = yaw

        # grabar serie temporal
        t = self.get_clock().now().nanoseconds * 1e-9 - self._t0
        self._times.append(t)
        self._gz_rot.append(math.degrees(self._gz_yaw_total))
        self._od_rot.append(math.degrees(self._od_yaw_total))
        self._gz_x.append(self._gz_dx)
        self._od_x.append(self._od_dx)
        self._gz_y.append(self._gz_dy)
        self._od_y.append(self._od_dy)

    def _odom_cb(self, msg: Odometry):
        # posición
        if self._od_x0 is None:
            self._od_x0 = msg.pose.pose.position.x
            self._od_y0 = msg.pose.pose.position.y
        self._od_dx = msg.pose.pose.position.x - self._od_x0
        self._od_dy = msg.pose.pose.position.y - self._od_y0

        # rotación
        yaw = self._quat_to_yaw(msg.pose.pose.orientation)
        if self._od_yaw_prev is None:
            self._od_yaw_prev = yaw
            return
        self._od_yaw_total += self._wrap(yaw - self._od_yaw_prev)
        self._od_yaw_prev = yaw

    def _print_status(self):
        rot_gz = math.degrees(self._gz_yaw_total)
        rot_od = math.degrees(self._od_yaw_total)
        self.get_logger().info(
            f"ROT Gz/Od: {self._ratio_str(rot_gz, rot_od, 1.0):>6} "
            f"| X Gz/Od: {self._ratio_str(self._gz_dx, self._od_dx):>6} "
            f"| Y Gz/Od: {self._ratio_str(self._gz_dy, self._od_dy):>6}"
        )

    # ------------------------------------------------------------------

    def plot_and_save(self):
        if len(self._times) < 10:
            print("Datos insuficientes para generar gráficas.")
            return

        rot_gz = math.degrees(self._gz_yaw_total)
        rot_od = math.degrees(self._od_yaw_total)
        r_rot = self._ratio_str(rot_gz, rot_od, 1.0)
        r_x = self._ratio_str(self._gz_dx, self._od_dx)
        r_y = self._ratio_str(self._gz_dy, self._od_dy)
        title = f"ROT Gz/Od: {r_rot}  |  X Gz/Od: {r_x}  |  Y Gz/Od: {r_y}"

        fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True)
        fig.suptitle(title, fontsize=12, fontweight="bold")

        def _plot_pair(ax, gz_data, od_data, ylabel, label_gz="Gazebo (GT)", label_od="Odometría (/odom)"):
            ax.plot(self._times, gz_data, label=label_gz, color="steelblue", linewidth=1.5)
            ax.plot(self._times, od_data, label=label_od, color="darkorange",
                    linestyle="--", linewidth=1.5)
            ax.set_ylabel(ylabel)
            ax.legend(loc="upper left", fontsize=8)
            ax.grid(True, alpha=0.4)

        _plot_pair(axes[0], self._gz_rot, self._od_rot, "Rotación acumulada (deg)")
        axes[0].set_title("Rotación")

        _plot_pair(axes[1], self._gz_x, self._od_x, "Desplazamiento X (m)")
        axes[1].set_title("Traslación X")

        _plot_pair(axes[2], self._gz_y, self._od_y, "Desplazamiento Y (m)")
        axes[2].set_title("Traslación Y")
        axes[2].set_xlabel("Tiempo (s)")

        plt.tight_layout()
        plt.savefig(self._plot_path, dpi=150, bbox_inches="tight")

        print(f"\n{'='*60}")
        print(title)
        print(f"  → robot_radius_nuevo = robot_radius / R_rot")
        print(f"  → wheel_radius_nuevo = wheel_radius * R_y  (o R_x)")
        print(f"Gráfica guardada: {self._plot_path}")
        print(f"{'='*60}")
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    node = CpmOdomNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.plot_and_save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
