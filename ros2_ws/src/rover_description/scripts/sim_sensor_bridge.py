#!/usr/bin/env python3
"""Bridge Gazebo simulation topics to hardware-equivalent ROS 2 sensor topics.

Publishes:
  - /imu/data_raw   (sensor_msgs/msg/Imu)      from /lidar/imu
  - /imu/euler      (geometry_msgs/msg/Vector3) roll/pitch/yaw in degrees from /lidar/imu
  - /wheel_states   (sensor_msgs/msg/JointState) left/right wheel angular speeds (rad/s)
  - /gps/fix        (sensor_msgs/msg/NavSatFix)  synthetic WGS-84 GPS from /odom ENU pose
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, JointState, NavSatFix, NavSatStatus

WGS84_A = 6378137.0
WGS84_E2 = 0.0066943799901413165


def quat_to_euler_deg(qx: float, qy: float, qz: float, qw: float):
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = max(-1.0, min(1.0, 2.0 * (qw * qy - qz * qx)))
    pitch = math.asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def enu_to_wgs84(east_m: float, north_m: float, up_m: float, lat0_deg: float, lon0_deg: float, alt0_m: float):
    lat0_rad = math.radians(lat0_deg)
    sin_lat = math.sin(lat0_rad)
    denom_sq = 1.0 - WGS84_E2 * sin_lat * sin_lat
    r_n = WGS84_A / math.sqrt(denom_sq)
    r_m = WGS84_A * (1.0 - WGS84_E2) / (denom_sq ** 1.5)
    lat_deg = lat0_deg + math.degrees(north_m / r_m)
    lon_deg = lon0_deg + math.degrees(east_m / (r_n * max(math.cos(lat0_rad), 1e-6)))
    return float(lat_deg), float(lon_deg), float(alt0_m + up_m)


class SimSensorBridge(Node):
    def __init__(self):
        super().__init__('sim_sensor_bridge')
        self.declare_parameter('datum_latitude', 51.46)
        self.declare_parameter('datum_longitude', -112.70)
        self.declare_parameter('datum_altitude', 792.0)
        self.declare_parameter('wheel_radius', 0.15)
        self.declare_parameter('wheel_separation', 0.60)
        self.declare_parameter('gps_rate_hz', 5.0)
        self.declare_parameter('gps_variance_m2', 0.25)

        self.datum_lat = float(self.get_parameter('datum_latitude').value)
        self.datum_lon = float(self.get_parameter('datum_longitude').value)
        self.datum_alt = float(self.get_parameter('datum_altitude').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_separation = float(self.get_parameter('wheel_separation').value)
        self.gps_period_s = 1.0 / max(float(self.get_parameter('gps_rate_hz').value), 0.5)
        self.gps_var = float(self.get_parameter('gps_variance_m2').value)

        self.latest_odom = None
        self.has_joint_vel = False

        self.pub_imu_raw = self.create_publisher(Imu, '/imu/data_raw', 10)
        self.pub_imu_euler = self.create_publisher(Vector3, '/imu/euler', 10)
        self.pub_wheel_states = self.create_publisher(JointState, '/wheel_states', 10)
        self.pub_gps = self.create_publisher(NavSatFix, '/gps/fix', 10)

        self.sub_imu = self.create_subscription(Imu, '/lidar/imu', self.imu_cb, 10)
        self.sub_js = self.create_subscription(JointState, '/joint_states', self.joint_states_cb, 10)
        self.sub_odom = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        self.gps_timer = self.create_timer(self.gps_period_s, self.publish_gps)

    def imu_cb(self, msg: Imu):
        raw = Imu()
        raw.header = msg.header
        raw.orientation = msg.orientation
        raw.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
        raw.angular_velocity = msg.angular_velocity
        raw.angular_velocity_covariance = [0.04, 0.0, 0.0, 0.0, 0.04, 0.0, 0.0, 0.0, 0.04]
        raw.linear_acceleration = msg.linear_acceleration
        raw.linear_acceleration_covariance = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.pub_imu_raw.publish(raw)

        q = msg.orientation
        if (q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w) > 1e-6:
            r_deg, p_deg, y_deg = quat_to_euler_deg(q.x, q.y, q.z, q.w)
            self.pub_imu_euler.publish(Vector3(x=float(r_deg), y=float(p_deg), z=float(y_deg)))

    def joint_states_cb(self, msg: JointState):
        if len(msg.name) != len(msg.velocity) or not msg.velocity:
            return
        names, vels = [], []
        for n, v in zip(msg.name, msg.velocity):
            if ('left' in n or 'right' in n) and math.isfinite(v):
                names.append(n)
                vels.append(float(v))
        if len(names) >= 2:
            self.has_joint_vel = True
            js = JointState()
            js.header = msg.header
            js.name = names
            js.velocity = vels
            self.pub_wheel_states.publish(js)

    def odom_cb(self, msg: Odometry):
        self.latest_odom = msg
        if not self.has_joint_vel and self.wheel_radius > 0.0:
            v = msg.twist.twist.linear.x
            w = msg.twist.twist.angular.z
            if math.isfinite(v) and math.isfinite(w):
                wl = (v - 0.5 * w * self.wheel_separation) / self.wheel_radius
                wr = (v + 0.5 * w * self.wheel_separation) / self.wheel_radius
                js = JointState()
                js.header = msg.header
                js.name = [
                    'front_left_wheel_joint', 'rear_left_wheel_joint',
                    'front_right_wheel_joint', 'rear_right_wheel_joint',
                ]
                js.velocity = [float(wl), float(wl), float(wr), float(wr)]
                self.pub_wheel_states.publish(js)

    def publish_gps(self):
        if self.latest_odom is None:
            return
        p = self.latest_odom.pose.pose.position
        lat, lon, alt = enu_to_wgs84(
            p.x, p.y, p.z, self.datum_lat, self.datum_lon, self.datum_alt
        )
        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = 'gps_link'
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = lat
        fix.longitude = lon
        fix.altitude = alt
        fix.position_covariance = [
            self.gps_var, 0.0, 0.0,
            0.0, self.gps_var, 0.0,
            0.0, 0.0, self.gps_var * 2.0,
        ]
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.pub_gps.publish(fix)


def main(args=None):
    rclpy.init(args=args)
    node = SimSensorBridge()
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
