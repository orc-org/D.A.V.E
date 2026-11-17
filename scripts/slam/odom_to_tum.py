#!/usr/bin/env python3
"""Write an odometry topic to a TUM trajectory file ("t x y z qx qy qz qw", t = header stamp in seconds).

For scoring a LIO run with eval_trajectory.py. With a replayed simulation bag the header stamps are
simulation time; pass `--est-clock sim` to eval_trajectory.py so it maps them onto the bag's receive times.

Usage:
    python3 scripts/slam/odom_to_tum.py OUT.txt [--topic /Odometry]
Stop with Ctrl-C; every message is flushed as it arrives.
"""
import argparse

import rclpy
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('out')
    ap.add_argument('--topic', default='/Odometry')
    a, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = Node('odom_to_tum')
    count = [0]
    with open(a.out, 'w') as f:
        def cb(m):
            p, q = m.pose.pose.position, m.pose.pose.orientation
            t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            f.write(f'{t:.9f} {p.x:.6f} {p.y:.6f} {p.z:.6f} {q.x:.6f} {q.y:.6f} {q.z:.6f} {q.w:.6f}\n')
            f.flush()
            count[0] += 1
        node.create_subscription(Odometry, a.topic, cb, 100)
        try:
            rclpy.spin(node)
        except (KeyboardInterrupt, ExternalShutdownException):
            pass
    print(f'{count[0]} poses written to {a.out}')
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
