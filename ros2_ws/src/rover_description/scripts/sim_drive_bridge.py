#!/usr/bin/env python3
"""Simulation drive bridge connecting SI /cmd_vel and /motor/* topics to Gazebo DiffDrive."""

import math
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32


class SimDriveBridge(Node):
    """Bridge SI /cmd_vel or normalized /motor/* wheel commands into Gazebo /cmd_vel_sim."""

    def __init__(self):
        super().__init__('sim_drive_bridge')

        self.declare_parameter('cmd_vel_is_si', True)
        self.declare_parameter('subscribe_cmd_vel', True)
        self.declare_parameter('wheel_separation', 0.60)
        self.declare_parameter('max_wheel_speed', 1.0)
        self.declare_parameter('max_linear_speed', 1.5)
        self.declare_parameter('max_angular_speed', 2.0)
        self.declare_parameter('rate_hz', 50.0)

        self.cmd_vel_is_si = bool(self.get_parameter('cmd_vel_is_si').value)
        self.subscribe_cmd_vel = bool(self.get_parameter('subscribe_cmd_vel').value)
        self.wheel_separation = max(0.05, float(self.get_parameter('wheel_separation').value))
        self.max_wheel_speed = max(0.05, float(self.get_parameter('max_wheel_speed').value))
        self.max_lin = float(self.get_parameter('max_linear_speed').value)
        self.max_ang = float(self.get_parameter('max_angular_speed').value)
        rate_hz = float(self.get_parameter('rate_hz').value)

        self.fl = 0.0
        self.fr = 0.0
        self.rl = 0.0
        self.rr = 0.0

        self.last_motor_ns = None
        self.last_cmd_vel_ns = None
        self.stopped = True

        self.sub_fl = self.create_subscription(Float32, '/motor/front_left', self.cb_fl, 10)
        self.sub_fr = self.create_subscription(Float32, '/motor/front_right', self.cb_fr, 10)
        self.sub_rl = self.create_subscription(Float32, '/motor/rear_left', self.cb_rl, 10)
        self.sub_rr = self.create_subscription(Float32, '/motor/rear_right', self.cb_rr, 10)
        # Disable direct /cmd_vel when drive_node is active so /control_mode, /sensitivity, and /joy arbitration stay authoritative
        self.sub_cmd_vel = (
            self.create_subscription(Twist, '/cmd_vel', self.cb_cmd_vel, 10)
            if self.subscribe_cmd_vel
            else None
        )
        self.pub_twist = self.create_publisher(Twist, '/cmd_vel_sim', 10)
        self.timer = self.create_timer(1.0 / max(1.0, rate_hz), self.update_and_publish)

        self.get_logger().info(
            f'sim_drive_bridge ready (cmd_vel_is_si={self.cmd_vel_is_si}, '
            f'subscribe_cmd_vel={self.subscribe_cmd_vel}, wheel_sep={self.wheel_separation:.2f} m)'
        )

    def _now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def cb_fl(self, msg: Float32):
        if math.isfinite(msg.data):
            self.fl = float(msg.data)
            self.last_motor_ns = self._now_ns()

    def cb_fr(self, msg: Float32):
        if math.isfinite(msg.data):
            self.fr = float(msg.data)
            self.last_motor_ns = self._now_ns()

    def cb_rl(self, msg: Float32):
        if math.isfinite(msg.data):
            self.rl = float(msg.data)
            self.last_motor_ns = self._now_ns()

    def cb_rr(self, msg: Float32):
        if math.isfinite(msg.data):
            self.rr = float(msg.data)
            self.last_motor_ns = self._now_ns()

    def cb_cmd_vel(self, msg: Twist):
        if not (math.isfinite(msg.linear.x) and math.isfinite(msg.angular.z)):
            return
        self.last_cmd_vel_ns = self._now_ns()
        twist = Twist()
        if self.cmd_vel_is_si:
            twist.linear.x = max(-self.max_lin, min(self.max_lin, float(msg.linear.x)))
            twist.angular.z = max(-self.max_ang, min(self.max_ang, float(msg.angular.z)))
        else:
            twist.linear.x = float(msg.linear.x * self.max_lin)
            twist.angular.z = float(msg.angular.z * self.max_ang)
        self.pub_twist.publish(twist)
        self.stopped = False

    def update_and_publish(self):
        now_ns = self._now_ns()
        if self.last_cmd_vel_ns is not None and (now_ns - self.last_cmd_vel_ns) < 300_000_000:
            return

        if self.last_motor_ns is not None and (now_ns - self.last_motor_ns) < 500_000_000:
            left = (self.fl + self.rl) * 0.5
            right = (self.fr + self.rr) * 0.5
            v_left = left * self.max_wheel_speed
            v_right = right * self.max_wheel_speed

            twist = Twist()
            twist.linear.x = max(-self.max_lin, min(self.max_lin, 0.5 * (v_left + v_right)))
            twist.angular.z = max(-self.max_ang, min(self.max_ang, (v_right - v_left) / self.wheel_separation))
            self.pub_twist.publish(twist)
            self.stopped = False
        elif not self.stopped:
            self.pub_twist.publish(Twist())
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
