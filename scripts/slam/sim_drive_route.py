#!/usr/bin/env python3
"""Drive the simulated rover along a waypoint route in CLOSED LOOP on the simulator's ground truth.

For odometry tests: the estimators are under test, not the controller, so steering on the true pose is
fair, and it makes the route repeatable (unlike sim_drive_loop.py's open-loop timing, whose E1 loop
missed its start by 2.5 m). The default route in the slam_yard world starts and ends at the spawn point
(0, 0), so the true start-to-end gap is ~0 and the estimate's end error is its loop drift:

    (0,0) -> (0,-4.6) -> LAPS x [(6.6,-4.6) (6.6,1.0) (-6.8,1.0) (-6.8,-4.6) (0,-4.6)] -> (0,0)

One lap is 38 m, so 3 laps plus the approach legs are about 123 m. Every leg keeps >= 1 m from the
crates, drums and walls of worlds/slam_yard.sdf.

Control (from /sim/ground_truth_poses, model `dave_rover`): heading error e to the current waypoint;
turn in place when |e| > 0.5 rad, else drive at speed * cos(e) with yaw rate k * e (clamped); a
waypoint counts as reached within 0.35 m. Stops (zero command) if the pose stops arriving for 1 s.

Usage:  scripts/slam/sim_drive_route.py [--laps 3] [--speed 0.8] [--turn-rate 0.8]
(the simulation must be running with the slam_yard world; publishes /cmd_vel_sim)
"""
import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage

LAP = [(6.6, -4.6), (6.6, 1.0), (-6.8, 1.0), (-6.8, -4.6), (0.0, -4.6)]


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class RouteDriver(Node):
    def __init__(self, a):
        super().__init__('sim_drive_route')
        self.a = a
        self.route = [(0.0, -4.6)] + LAP * a.laps + [(0.0, 0.0)]
        self.idx = 0
        self.finished = False
        self.start_sim = None
        self.sim_t = None
        self.last_pose_wall = None
        self.travelled = 0.0
        self.last_xy = None
        self.pub = self.create_publisher(Twist, '/cmd_vel_sim', 10)
        self.create_subscription(Clock, '/clock', self.on_clock, 10)
        self.create_subscription(TFMessage, '/sim/ground_truth_poses', self.on_pose, 10)
        self.create_timer(0.2, self.watchdog)

    def on_clock(self, msg):
        self.sim_t = msg.clock.sec + msg.clock.nanosec * 1e-9
        if self.start_sim is None:
            self.start_sim = self.sim_t + 2.0   # let the sensors and the bag recorder settle

    def watchdog(self):
        # wall-clock check: no pose for 1 s means the simulator or bridge stopped; do not keep driving
        if self.last_pose_wall is not None and time.monotonic() - self.last_pose_wall > 1.0:
            self.pub.publish(Twist())

    def on_pose(self, msg):
        tf = next((t for t in msg.transforms if t.child_frame_id == self.a.model), None)
        if tf is None or self.finished:
            return
        self.last_pose_wall = time.monotonic()
        x, y = tf.transform.translation.x, tf.transform.translation.y
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        if self.last_xy is not None:
            self.travelled += math.hypot(x - self.last_xy[0], y - self.last_xy[1])
        self.last_xy = (x, y)

        cmd = Twist()
        if self.start_sim is None or self.sim_t is None or self.sim_t < self.start_sim:
            self.pub.publish(cmd)
            return
        tx, ty = self.route[self.idx]
        d = math.hypot(tx - x, ty - y)
        if d < 0.35:
            self.idx += 1
            if self.idx == len(self.route):
                self.finished = True
                self.pub.publish(cmd)
                return
            if self.idx % len(LAP) == 1:
                lap = (self.idx - 1) // len(LAP)
                self.get_logger().info(f'waypoint {self.idx}/{len(self.route)} (lap {min(lap + 1, self.a.laps)}), '
                                       f'{self.travelled:.1f} m travelled')
            tx, ty = self.route[self.idx]
            d = math.hypot(tx - x, ty - y)
        e = wrap(math.atan2(ty - y, tx - x) - yaw)
        cmd.angular.z = max(-self.a.turn_rate, min(self.a.turn_rate, 2.0 * e))
        if abs(e) < 0.5:
            last = self.idx == len(self.route) - 1
            cmd.linear.x = self.a.speed * math.cos(e)
            if last:
                cmd.linear.x = min(cmd.linear.x, 0.5 * d + 0.05)   # ease into the final stop
        self.pub.publish(cmd)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--laps', type=int, default=3)
    p.add_argument('--speed', type=float, default=0.8, help='m/s')
    p.add_argument('--turn-rate', type=float, default=0.8, help='max rad/s')
    p.add_argument('--model', default='dave_rover')
    a, ros_args = p.parse_known_args()
    rclpy.init(args=ros_args)
    node = RouteDriver(a)
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    node.pub.publish(Twist())
    print(f'route finished, {node.travelled:.1f} m travelled' if node.finished else 'interrupted')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
