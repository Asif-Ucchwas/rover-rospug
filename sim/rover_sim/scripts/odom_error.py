#!/usr/bin/env python3
"""Task 1.3: drive the rover 10 m straight (open loop, like a person holding the forward key), then spin once in place,
and compare wheel odometry (/odom) with the physics engine's true pose (/ground_truth). Writes a CSV and prints the error.
Usage (sim already running):  ros2 run rover_sim odom_error.py --distance 10 --out odom_error_10m.csv"""
import argparse, csv, math, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def yaw_of(m):
    q = m.pose.pose.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class N(Node):
    def __init__(self):
        super().__init__('odom_error'); self.gt = None; self.od = None
        self.create_subscription(Odometry, '/ground_truth', lambda m: setattr(self, 'gt', m), 10)
        self.create_subscription(Odometry, '/odom', lambda m: setattr(self, 'od', m), 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def spin_for(self, s):
        t = time.time()
        while time.time() - t < s: rclpy.spin_once(self, timeout_sec=0.02)

    def pos(self, m): return m.pose.pose.position.x, m.pose.pose.position.y


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('--distance', type=float, default=10.0)
    ap.add_argument('--speed', type=float, default=0.5); ap.add_argument('--out', default='odom_error_10m.csv')
    a = ap.parse_args(); rclpy.init(); n = N(); t0 = time.time()
    while (n.gt is None or n.od is None) and time.time() - t0 < 30: rclpy.spin_once(n, timeout_sec=0.1)
    if n.gt is None or n.od is None:
        print('FAIL: no /ground_truth or /odom (is the sim running?)'); raise SystemExit(2)
    g0 = n.pos(n.gt); o0 = n.pos(n.od); gy0 = yaw_of(n.gt); oy0 = yaw_of(n.od); rows = []; nxt = 0.5
    cmd = Twist(); cmd.linear.x = a.speed; t1 = time.time()
    while time.time() - t1 < 4 * a.distance / a.speed:
        n.pub.publish(cmd); n.spin_for(0.05)
        gx, gy = n.pos(n.gt); ox, oy = n.pos(n.od)
        true_d = math.hypot(gx - g0[0], gy - g0[1]); odom_d = math.hypot(ox - o0[0], oy - o0[1])
        if true_d >= nxt:
            rows.append((round(true_d, 3), round(odom_d, 3), round(odom_d - true_d, 3), round(gy - g0[1], 3),
                         round(math.degrees(yaw_of(n.gt) - gy0), 2))); nxt += 0.5
        if true_d >= a.distance: break
    n.pub.publish(Twist()); n.spin_for(1.0)
    gx, gy = n.pos(n.gt); ox, oy = n.pos(n.od)
    true_d = math.hypot(gx - g0[0], gy - g0[1]); odom_d = math.hypot(ox - o0[0], oy - o0[1])
    err = odom_d - true_d
    print(f'STRAIGHT: true {true_d:.3f} m, wheel odometry {odom_d:.3f} m, error {err:+.3f} m ({100 * err / true_d:+.2f} %)')
    print(f'          sideways drift {gy - g0[1]:+.3f} m, heading change {math.degrees(yaw_of(n.gt) - gy0):+.2f} deg')
    # one full turn in place at 0.5 rad/s
    sy = yaw_of(n.gt); so = yaw_of(n.od); gturn = 0.0; oturn = 0.0; pg = sy; po = so; spin = Twist(); spin.angular.z = 0.5; t2 = time.time()
    wrap = lambda d: math.atan2(math.sin(d), math.cos(d))
    while abs(gturn) < 2 * math.pi and time.time() - t2 < 40:
        n.pub.publish(spin); n.spin_for(0.05)
        yg = yaw_of(n.gt); yo = yaw_of(n.od); gturn += wrap(yg - pg); oturn += wrap(yo - po); pg = yg; po = yo
    n.pub.publish(Twist()); n.spin_for(1.0)
    gturn += wrap(yaw_of(n.gt) - pg); oturn += wrap(yaw_of(n.od) - po)
    print(f'SPIN:     true {math.degrees(gturn):.1f} deg, wheel odometry {math.degrees(oturn):.1f} deg, error {math.degrees(oturn - gturn):+.1f} deg')
    with open(a.out, 'w', newline='') as f:
        w = csv.writer(f); w.writerow(['true_m', 'odom_m', 'odom_minus_true_m', 'sideways_m', 'heading_deg']); w.writerows(rows)
        w.writerow([]); w.writerow(['summary', 'straight_true_m', round(true_d, 3), 'straight_odom_m', round(odom_d, 3),
                                    'spin_true_deg', round(math.degrees(gturn), 1), 'spin_odom_deg', round(math.degrees(oturn), 1)])
    print('saved', a.out); rclpy.shutdown()


if __name__ == '__main__':
    main()
