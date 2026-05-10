#!/usr/bin/env python3
"""
Herramienta de calibración de odometría.

Compara la rotación real de Gazebo (ground truth via PosePublisher plugin) con
la estimada por el controlador (/odom). Al terminar con Ctrl+C genera gráficas
y muestra el resumen final:

    ROT Gz: +XXXX.X deg  Od: +XXXX.X deg  R: X.XXX

El ratio R se usa para ajustar robot_radius en controllers.yaml:
    r_nuevo = r_actual / R

Requiere que cpm_odom_sim.launch.py esté activo (bridge + plugin PosePublisher).
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

        self._gz_yaw_prev = None
        self._gz_yaw_total = 0.0
        self._od_yaw_prev = None
        self._od_yaw_total = 0.0

        self._t0 = None
        self._times: list[float] = []
        self._gz_degs: list[float] = []
        self._od_degs: list[float] = []

        self.create_subscription(Pose, gz_poses_topic, self._gz_cb, 10)
        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.create_timer(1.0, self._print_status)

        self.get_logger().info(f"GT: {gz_poses_topic} | Odom: {odom_topic}")
        self.get_logger().info(
            "Aplica rotación pura y pulsa Ctrl+C para generar las gráficas."
        )

    # ------------------------------------------------------------------

    @staticmethod
    def _quat_to_yaw(q) -> float:
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _wrap(delta: float) -> float:
        return math.atan2(math.sin(delta), math.cos(delta))

    # ------------------------------------------------------------------

    def _gz_cb(self, msg: Pose):
        yaw = self._quat_to_yaw(msg.orientation)
        if self._gz_yaw_prev is None:
            self._gz_yaw_prev = yaw
            self._t0 = self.get_clock().now().nanoseconds * 1e-9
            return
        self._gz_yaw_total += self._wrap(yaw - self._gz_yaw_prev)
        self._gz_yaw_prev = yaw

        t = self.get_clock().now().nanoseconds * 1e-9 - self._t0
        self._times.append(t)
        self._gz_degs.append(math.degrees(self._gz_yaw_total))
        self._od_degs.append(math.degrees(self._od_yaw_total))

    def _odom_cb(self, msg: Odometry):
        yaw = self._quat_to_yaw(msg.pose.pose.orientation)
        if self._od_yaw_prev is None:
            self._od_yaw_prev = yaw
            return
        self._od_yaw_total += self._wrap(yaw - self._od_yaw_prev)
        self._od_yaw_prev = yaw

    def _print_status(self):
        gz_deg = math.degrees(self._gz_yaw_total)
        od_deg = math.degrees(self._od_yaw_total)
        if abs(od_deg) > 1.0:
            r = gz_deg / od_deg
            self.get_logger().info(
                f"ROT Gz: {gz_deg:+.1f} deg  Od: {od_deg:+.1f} deg  R: {r:.3f}"
            )
        else:
            self.get_logger().info(
                f"ROT Gz: {gz_deg:+.1f} deg  Od: {od_deg:+.1f} deg  R: ---"
            )

    # ------------------------------------------------------------------

    def plot_and_save(self):
        if len(self._times) < 10:
            print("Datos insuficientes para generar gráficas.")
            return

        gz_final = math.degrees(self._gz_yaw_total)
        od_final = math.degrees(self._od_yaw_total)
        ratio_str = f"{gz_final / od_final:.3f}" if abs(od_final) > 1.0 else "---"
        title = f"ROT Gz: {gz_final:+.1f}°  Od: {od_final:+.1f}°  R: {ratio_str}"

        fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)
        fig.suptitle(title, fontsize=13, fontweight="bold")

        axes[0].plot(self._times, self._gz_degs, label="Gazebo (GT)", color="steelblue", linewidth=1.5)
        axes[0].plot(self._times, self._od_degs, label="Odometría (/odom)", color="darkorange",
                     linestyle="--", linewidth=1.5)
        axes[0].set_ylabel("Rotación acumulada (deg)")
        axes[0].set_title("Rotación acumulada")
        axes[0].legend()
        axes[0].grid(True, alpha=0.4)

        errors = [g - o for g, o in zip(self._gz_degs, self._od_degs)]
        axes[1].plot(self._times, errors, color="crimson", linewidth=1.2)
        axes[1].axhline(0.0, color="gray", linestyle="--", alpha=0.5)
        axes[1].set_xlabel("Tiempo (s)")
        axes[1].set_ylabel("Error Gz − Od (deg)")
        axes[1].set_title("Error de rotación")
        axes[1].grid(True, alpha=0.4)

        plt.tight_layout()
        plt.savefig(self._plot_path, dpi=150, bbox_inches="tight")
        print(f"\n{'='*60}")
        print(title)
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
