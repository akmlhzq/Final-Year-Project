#!/usr/bin/env python3
"""
ROS 2 node: follow the nearest object seen by the RPLIDAR C1.

  sllidar_ros2 driver --/scan--> this node --UART "L,R\n"--> ESP32 --> BTS7960

ROS convention: angle 0 = front (x axis), positive = LEFT (counter-clockwise).

Run (after sourcing ROS 2 and starting the LiDAR driver):
    python3 lidar_follow_ros2.py
    python3 lidar_follow_ros2.py --ros-args -p max_speed:=100 -p angle_offset_deg:=180
"""

import math
import serial
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


def clamp(v, lo, hi):
    return max(lo, min(hi, v))


class LidarFollow(Node):
    def __init__(self):
        super().__init__("lidar_follow")

        # ---------- parameters (change with --ros-args -p name:=value) ----------
        self.declare_parameter("esp_port", "/dev/ttyAMA0")
        self.declare_parameter("esp_baud", 115200)
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("angle_offset_deg", 0.0)  # add 90/180/270 if LiDAR front != robot front
        self.declare_parameter("fov_deg", 90.0)          # look +-fov around the front
        self.declare_parameter("min_dist", 0.25)         # m
        self.declare_parameter("max_dist", 1.5)          # m
        self.declare_parameter("target_dist", 0.7)       # m, distance to keep
        self.declare_parameter("kp_turn", 2.0)           # PWM per degree
        self.declare_parameter("kp_fwd", 200.0)          # PWM per meter
        self.declare_parameter("max_speed", 150)         # forward limit (0-255)
        self.declare_parameter("max_turn", 120)
        self.declare_parameter("angle_deadband_deg", 5.0)
        self.declare_parameter("lost_scans", 3)

        p = self.get_parameter
        self.offset = math.radians(p("angle_offset_deg").value)
        self.fov = math.radians(p("fov_deg").value)
        self.min_dist = p("min_dist").value
        self.max_dist = p("max_dist").value
        self.target_dist = p("target_dist").value
        self.kp_turn = p("kp_turn").value
        self.kp_fwd = p("kp_fwd").value
        self.max_speed = p("max_speed").value
        self.max_turn = p("max_turn").value
        self.deadband = p("angle_deadband_deg").value
        self.lost_limit = p("lost_scans").value
        self.lost = 0

        self.esp = serial.Serial(p("esp_port").value, p("esp_baud").value, timeout=0.1)
        self.create_subscription(
            LaserScan, p("scan_topic").value, self.on_scan, qos_profile_sensor_data
        )
        self.get_logger().info("lidar_follow started, waiting for /scan ...")

    # ---------------------------------------------------------------
    def send(self, left, right):
        self.esp.write(f"{int(left)},{int(right)}\n".encode())

    def find_target(self, scan: LaserScan):
        """Return (angle_deg, dist_m) of the nearest object in front, or None."""
        pts = []
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r) or r < self.min_dist or r > self.max_dist:
                continue
            a = scan.angle_min + i * scan.angle_increment + self.offset
            a = math.atan2(math.sin(a), math.cos(a))  # normalize to -pi..pi
            if abs(a) <= self.fov:
                pts.append((a, r))
        if not pts:
            return None
        nearest = min(r for _, r in pts)
        cluster = [(a, r) for a, r in pts if r - nearest < 0.15]  # average -> less jitter
        ang = sum(a for a, _ in cluster) / len(cluster)
        dist = sum(r for _, r in cluster) / len(cluster)
        return math.degrees(ang), dist

    def on_scan(self, scan: LaserScan):
        target = self.find_target(scan)
        if target is None:
            self.lost += 1
            if self.lost >= self.lost_limit:
                self.send(0, 0)
            return
        self.lost = 0

        angle, dist = target
        fwd = clamp(self.kp_fwd * (dist - self.target_dist), 0, self.max_speed)
        turn = 0.0 if abs(angle) < self.deadband else clamp(
            self.kp_turn * angle, -self.max_turn, self.max_turn
        )
        # target on the left (+angle) -> right wheel faster, left wheel slower
        left = clamp(fwd - turn, -255, 255)
        right = clamp(fwd + turn, -255, 255)
        self.send(left, right)
        self.get_logger().info(
            f"target {angle:6.1f} deg {dist:5.2f} m -> L={int(left):4d} R={int(right):4d}",
            throttle_duration_sec=0.5,
        )

    def destroy_node(self):
        try:
            self.send(0, 0)
            self.esp.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = LidarFollow()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
