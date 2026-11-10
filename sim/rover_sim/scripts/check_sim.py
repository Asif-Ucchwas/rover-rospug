#!/usr/bin/env python3
"""Drive the rover straight at the wall in a gap world and report whether it got through.
Pass/fail is judged from /ground_truth (the physics engine's true pose; /odom only adds up wheel speed and would say 'passed' even for a rover stuck on a wall).
Usage (sim already running):  ros2 run rover_sim check_sim.py --distance 12 --expect pass|block|any"""
import argparse, math, sys, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class Check(Node):
    def __init__(self):
        super().__init__('check_sim'); self.odom = None; self.wheel = None; self.scan = None
        self.create_subscription(Odometry, '/ground_truth', lambda m: setattr(self, 'odom', m), 10)
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'wheel', m), 10)
        self.create_subscription(LaserScan, '/scan', lambda m: setattr(self, 'scan', m), 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def spin_for(self, s):
        t = time.time()
        while time.time() - t < s: rclpy.spin_once(self, timeout_sec=0.05)


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--distance', type=float, default=12.0)
    ap.add_argument('--speed', type=float, default=0.4); ap.add_argument('--expect', choices=['pass', 'block', 'any'], default='any')
    ap.add_argument('--timeout', type=float, default=90.0)
    a = ap.parse_args(); rclpy.init(); n = Check()
    t0 = time.time()
    while (n.odom is None or n.scan is None) and time.time() - t0 < 30: rclpy.spin_once(n, timeout_sec=0.1)
    if n.odom is None or n.scan is None:
        print('FAIL: no /ground_truth or /scan received (is the sim running? bridge up? was the package rebuilt?)'); sys.exit(2)
    x0 = n.odom.pose.pose.position.x; y0 = n.odom.pose.pose.position.y; w0 = n.wheel.pose.pose.position.x if n.wheel else 0.0
    finite = [r for r in n.scan.ranges if math.isfinite(r)]
    print(f'lidar ok: {len(n.scan.ranges)} beams, nearest return {min(finite):.2f} m' if finite else 'lidar: no returns')
    best = 0.0; last_progress = time.time(); last_best = 0.0; cmd = Twist(); cmd.linear.x = a.speed
    while time.time() - t0 < a.timeout:
        n.pub.publish(cmd); n.spin_for(0.1)
        d = n.odom.pose.pose.position.x - x0; best = max(best, d)
        if best > last_best + 0.02: last_best = best; last_progress = time.time()
        if best >= a.distance or time.time() - last_progress > 6.0: break
    n.pub.publish(Twist()); n.spin_for(0.3)
    through = best >= a.distance
    if n.wheel: print(f'wheel odometry says {n.wheel.pose.pose.position.x - w0:.2f} m (it only adds up wheel speed)')
    print(f'true motion {best:.2f} m along x (target {a.distance} m) -> {"PASSED THROUGH" if through else "BLOCKED"}')
    ok = a.expect == 'any' or (a.expect == 'pass') == through
    print('RESULT:', 'OK' if ok else 'UNEXPECTED'); rclpy.shutdown(); sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
