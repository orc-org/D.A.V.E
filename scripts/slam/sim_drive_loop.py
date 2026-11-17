#!/usr/bin/env python3
"""Drive the simulated rover around a closed rectangle and stop, using SIMULATION time.

Publishes geometry_msgs/Twist on /cmd_vel_sim (the input of the Gazebo skid-steer plugin). Timing
uses /clock, not the wall clock, because under WSL2 software rendering the simulator runs slower
than real time and a wall-clock schedule would drive the wrong distances.

The path is `legs` repetitions of: straight `side` metres, then a 90 degree left turn, so with the
default 4 legs the rover ends where it started and facing the same way (a loop closure test).
Open-loop: the actual path depends on wheel slip in the simulator, so use the recorded ground truth
(/sim/ground_truth_poses) rather than the commanded path for evaluation.

Usage:  scripts/slam/sim_drive_loop.py [--side 6.0] [--speed 0.5] [--turn-rate 0.5] [--legs 4]
(source ROS and the workspace first; the simulation must be running with use_sim_time)
"""
import argparse
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rosgraph_msgs.msg import Clock


class LoopDriver(Node):
    def __init__(self, a):
        super().__init__('sim_drive_loop')
        self.pub = self.create_publisher(Twist, '/cmd_vel_sim', 10)
        self.create_subscription(Clock, '/clock', self.on_clock, 10)
        # (duration in sim seconds, linear m/s, angular rad/s)
        self.plan = []
        for _ in range(a.legs):
            self.plan.append((a.side / a.speed, a.speed, 0.0))
            self.plan.append((math.pi / 2 / a.turn_rate, 0.0, a.turn_rate))
        self.plan.append((1.0, 0.0, 0.0))  # settle
        self.t0 = None
        self.finished = False

    def on_clock(self, msg):
        t = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self.t0 is None:
            self.t0 = t + 2.0  # short pause so the bridge and sensors are up
        elapsed = t - self.t0
        cmd = Twist()
        if elapsed >= 0.0:
            for duration, lin, ang in self.plan:
                if elapsed < duration:
                    cmd.linear.x, cmd.angular.z = lin, ang
                    break
                elapsed -= duration
            else:
                self.finished = True
        self.pub.publish(cmd)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--side', type=float, default=6.0, help='leg length, metres')
    p.add_argument('--speed', type=float, default=0.5, help='m/s')
    p.add_argument('--turn-rate', type=float, default=0.5, help='rad/s')
    p.add_argument('--legs', type=int, default=4)
    a, ros_args = p.parse_known_args()
    rclpy.init(args=ros_args)
    node = LoopDriver(a)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    node.pub.publish(Twist())
    print('loop finished' if node.finished else 'interrupted')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
