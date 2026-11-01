#!/usr/bin/env python3
"""
simulation drive bridge node for d.a.v.e.
subscribes to /motor/* topics from drive_node and publishes /cmd_vel_sim to modern gazebo
"""

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

        # subscriptions to drive_node output topics
        self.sub_fl = self.create_subscription(Float32, '/motor/front_left', self.cb_fl, 10)
        self.sub_fr = self.create_subscription(Float32, '/motor/front_right', self.cb_fr, 10)
        self.sub_rl = self.create_subscription(Float32, '/motor/rear_left', self.cb_rl, 10)
        self.sub_rr = self.create_subscription(Float32, '/motor/rear_right', self.cb_rr, 10)

        # publisher to gazebo skid-steer controller
        self.pub_twist = self.create_publisher(Twist, '/cmd_vel_sim', 10)

        # timer loop for continuous smooth command streaming to gazebo
        self.timer = self.create_timer(1.0 / rate_hz, self.update_and_publish)

        self.get_logger().info('sim_drive_bridge node ready')

    def cb_fl(self, msg: Float32):
        self.fl = msg.data

    def cb_fr(self, msg: Float32):
        self.fr = msg.data

    def cb_rl(self, msg: Float32):
        self.rl = msg.data

    def cb_rr(self, msg: Float32):
        self.rr = msg.data

    def update_and_publish(self):
        # average left and right sides
        left = (self.fl + self.rl) * 0.5
        right = (self.fr + self.rr) * 0.5

        # in drive_node skid-steer mixing:
        # left = throttle - steering
        # right = throttle + steering
        throttle = (left + right) * 0.5
        steering = (right - left) * 0.5

        twist = Twist()
        twist.linear.x = float(throttle * self.max_lin)
        twist.angular.z = float(steering * self.max_ang)

        self.pub_twist.publish(twist)


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
