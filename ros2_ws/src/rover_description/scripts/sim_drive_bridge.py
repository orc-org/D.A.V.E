#!/usr/bin/env python3
"""
simulation drive bridge node for d.a.v.e.
bridges drive commands to modern gazebo (gz_sim / ign_gazebo) skid-steer controller
supports direct /cmd_vel forwarding as well as /motor/* skid-steer reconstruction
"""

import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class SimDriveBridge(Node):
    def __init__(self):
        super().__init__('sim_drive_bridge')

        # tunable speed limits for gazebo
        self.declare_parameter('max_linear_speed', 1.5)   # m/s at 100% motor throttle
        self.declare_parameter('max_angular_speed', 2.0)  # rad/s at 100% steering
        self.declare_parameter('rate_hz', 50.0)

        self.max_lin = float(self.get_parameter('max_linear_speed').value)
        self.max_ang = float(self.get_parameter('max_angular_speed').value)
        rate_hz = float(self.get_parameter('rate_hz').value)

        # motor values (-1.0 to 1.0)
        self.fl = 0.0
        self.fr = 0.0
        self.rl = 0.0
        self.rr = 0.0

        # wall-clock monotonic timestamps for robust timing
        self.last_motor_time = 0.0
        self.last_cmd_vel_time = 0.0
        self.stopped = True

        # subscriptions to drive_node output topics
        self.sub_fl = self.create_subscription(Float32, '/motor/front_left', self.cb_fl, 10)
        self.sub_fr = self.create_subscription(Float32, '/motor/front_right', self.cb_fr, 10)
        self.sub_rl = self.create_subscription(Float32, '/motor/rear_left', self.cb_rl, 10)
        self.sub_rr = self.create_subscription(Float32, '/motor/rear_right', self.cb_rr, 10)

        # direct /cmd_vel subscription
        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cb_cmd_vel, 10)

        # publisher to gazebo skid-steer controller
        self.pub_twist = self.create_publisher(Twist, '/cmd_vel_sim', 10)

        # continuous stream timer for smooth simulation dynamics
        self.timer = self.create_timer(1.0 / rate_hz, self.update_and_publish)

        self.get_logger().info('sim_drive_bridge node ready (wall-clock monotonic timer active)')

    def cb_fl(self, msg: Float32):
        self.fl = float(msg.data)
        self.last_motor_time = time.monotonic()

    def cb_fr(self, msg: Float32):
        self.fr = float(msg.data)
        self.last_motor_time = time.monotonic()

    def cb_rl(self, msg: Float32):
        self.rl = float(msg.data)
        self.last_motor_time = time.monotonic()

    def cb_rr(self, msg: Float32):
        self.rr = float(msg.data)
        self.last_motor_time = time.monotonic()

    def cb_cmd_vel(self, msg: Twist):
        self.last_cmd_vel_time = time.monotonic()
        # Direct forwarding: scale linear and angular by max speed limits
        twist = Twist()
        twist.linear.x = float(msg.linear.x * self.max_lin)
        twist.angular.z = float(msg.angular.z * self.max_ang)
        self.pub_twist.publish(twist)
        self.stopped = False

    def update_and_publish(self):
        now = time.monotonic()

        # If direct /cmd_vel was received within last 300ms, let cb_cmd_vel control motion
        if (now - self.last_cmd_vel_time) < 0.3:
            return

        # Otherwise, check if /motor/* topics from drive_node (e.g. /joy) are active
        if (now - self.last_motor_time) < 0.5:
            # skid-steer unmixing from 4 motor values
            left = (self.fl + self.rl) * 0.5
            right = (self.fr + self.rr) * 0.5

            throttle = (left + right) * 0.5
            steering = (right - left) * 0.5

            twist = Twist()
            twist.linear.x = float(throttle * self.max_lin)
            twist.angular.z = float(steering * self.max_ang)

            self.pub_twist.publish(twist)
            self.stopped = False
        elif not self.stopped:
            # Publish stop once when all inputs become inactive
            stop_twist = Twist()
            self.pub_twist.publish(stop_twist)
            self.stopped = True


def main(args=None):
    rclpy.init(args=args)
    node = SimDriveBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
