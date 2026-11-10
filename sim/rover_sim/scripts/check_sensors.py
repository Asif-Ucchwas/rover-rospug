#!/usr/bin/env python3
"""Task 1.4: check that the simulated camera (RGB + depth) and IMU publish sensible data.
Usage (sim already running):  ros2 run rover_sim check_sensors.py"""
import math, sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, Imu


class C(Node):
    def __init__(self):
        super().__init__('check_sensors'); self.rgb = self.dep = self.imu = None; self.cnt = {'rgb': 0, 'dep': 0, 'imu': 0}; self.gz = []
        self.create_subscription(Image, '/camera/image_raw', self.on_rgb, 10)
        self.create_subscription(Image, '/camera/depth', self.on_dep, 10)
        self.create_subscription(Imu, '/imu', self.on_imu, 50)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def on_rgb(self, m): self.rgb = m; self.cnt['rgb'] += 1
    def on_dep(self, m): self.dep = m; self.cnt['dep'] += 1
    def on_imu(self, m): self.imu = m; self.cnt['imu'] += 1; self.gz.append(m.angular_velocity.z)

    def spin_for(self, s):
        t = time.time()
        while time.time() - t < s: rclpy.spin_once(self, timeout_sec=0.02)


def main():
    rclpy.init(); n = C(); t0 = time.time()
    while (n.rgb is None or n.dep is None or n.imu is None) and time.time() - t0 < 30: rclpy.spin_once(n, timeout_sec=0.1)
    ok = True
    if n.rgb is None: print('FAIL: no /camera/image_raw'); ok = False
    if n.dep is None: print('FAIL: no /camera/depth'); ok = False
    if n.imu is None: print('FAIL: no /imu'); ok = False
    if not ok: sys.exit(2)
    for k in n.cnt: n.cnt[k] = 0
    n.gz.clear(); t1 = time.time(); n.spin_for(3.0); dt = time.time() - t1
    print(f'rates: camera {n.cnt["rgb"] / dt:.1f} Hz, depth {n.cnt["dep"] / dt:.1f} Hz, imu {n.cnt["imu"] / dt:.0f} Hz')
    img = np.frombuffer(n.rgb.data, dtype=np.uint8); print(f'camera: {n.rgb.width}x{n.rgb.height} {n.rgb.encoding}, mean brightness {img.mean():.0f}/255 (0 = black image = problem)')
    d = np.frombuffer(n.dep.data, dtype=np.float32).reshape(n.dep.height, n.dep.width); f = d[np.isfinite(d)]
    print(f'depth: {n.dep.width}x{n.dep.height}, centre pixel {d[n.dep.height // 2, n.dep.width // 2]:.2f} m, {100 * f.size / d.size:.0f}% of pixels have a distance')
    a = n.imu.linear_acceleration; print(f'imu at rest: accel z {a.z:.2f} m/s2 (expect about 9.81), gyro z {np.mean(n.gz):+.3f} rad/s (expect about 0)')
    cmd = Twist(); cmd.angular.z = 0.5; n.gz.clear(); t2 = time.time()
    while time.time() - t2 < 3.0: n.pub.publish(cmd); n.spin_for(0.05)
    n.pub.publish(Twist()); n.spin_for(0.5)
    print(f'imu while turning at 0.5 rad/s: gyro z {np.mean(n.gz[len(n.gz) // 4: -len(n.gz) // 6]):+.3f} rad/s')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
