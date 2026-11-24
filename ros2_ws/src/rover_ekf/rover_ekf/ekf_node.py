#!/usr/bin/env python3
import math
import os
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix, JointState
from geometry_msgs.msg import Vector3, Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from tf2_ros import TransformBroadcaster

WGS84_A = 6378137.0
WGS84_E2 = 0.0066943799901413165


def load_wheel_radius_from_config(default: float = 0.15, config_path: str = None, logger=None) -> float:
    """Load robot_dimensions.wheel_radius from rover_description/config/physical_params.yaml."""
    try:
        import yaml
        if config_path is None:
            from ament_index_python.packages import get_package_share_directory
            config_path = os.path.join(
                get_package_share_directory('rover_description'),
                'config',
                'physical_params.yaml',
            )
        with open(config_path, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
        dims = data.get('robot_dimensions') or {}
        radius = float(dims['wheel_radius'])
        if math.isfinite(radius) and radius > 0.0:
            return radius
        if logger is not None:
            logger.warn(f'Missing or invalid robot_dimensions.wheel_radius in {config_path}; using {default:.3f} m')
    except Exception as exc:
        if logger is not None:
            logger.warn(f'Could not load wheel_radius from physical_params.yaml ({exc}); using {default:.3f} m')
    return float(default)


def wrap_angle_rad(angle: float) -> float:
    """Wrap angle in radians to [-pi, pi]."""
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def wgs84_enu_offset(lat_deg: float, lon_deg: float, lat0_deg: float, lon0_deg: float):
    """Convert geodetic delta relative to (lat0, lon0) into local ENU (east_m, north_m) using WGS-84 radii."""
    lat0_rad = math.radians(lat0_deg)
    sin_lat = math.sin(lat0_rad)
    denom_sq = 1.0 - WGS84_E2 * sin_lat * sin_lat
    r_n = WGS84_A / math.sqrt(denom_sq)
    r_m = WGS84_A * (1.0 - WGS84_E2) / (denom_sq ** 1.5)
    x_east = math.radians(lon_deg - lon0_deg) * r_n * math.cos(lat0_rad)
    y_north = math.radians(lat_deg - lat0_deg) * r_m
    return float(x_east), float(y_north)


def joseph_update(x: np.ndarray, P: np.ndarray, H: np.ndarray, y: np.ndarray, R: np.ndarray, wrap_angle_idx=None):
    """Apply EKF update using the symmetric Joseph covariance form to preserve PSD guarantees."""
    S = H @ P @ H.T + R
    K = P @ H.T @ np.linalg.inv(S)
    x_new = x + K @ y
    if wrap_angle_idx is not None:
        x_new[wrap_angle_idx] = wrap_angle_rad(float(x_new[wrap_angle_idx]))
    I_KH = np.eye(len(x)) - K @ H
    P_new = I_KH @ P @ I_KH.T + K @ R @ K.T
    P_new = 0.5 * (P_new + P_new.T)
    return x_new, P_new


def predict_unicycle_state(x: np.ndarray, P: np.ndarray, Q: np.ndarray, dt: float, nominal_dt: float):
    """Advance unicycle state [X, Y, theta, v, omega] and covariance over dt."""
    if dt <= 0.0:
        return x.copy(), P.copy()
    X, Y, theta, v, omega = float(x[0]), float(x[1]), float(x[2]), float(x[3]), float(x[4])
    decay = 0.98 ** (dt / max(nominal_dt, 1e-4))
    x_next = np.array([
        X + v * math.cos(theta) * dt,
        Y + v * math.sin(theta) * dt,
        wrap_angle_rad(theta + omega * dt),
        v * decay,
        omega * decay,
    ])
    F = np.array([
        [1.0, 0.0, -v * math.sin(theta) * dt, math.cos(theta) * dt, 0.0],
        [0.0, 1.0,  v * math.cos(theta) * dt, math.sin(theta) * dt, 0.0],
        [0.0, 0.0, 1.0, 0.0, dt],
        [0.0, 0.0, 0.0, decay, 0.0],
        [0.0, 0.0, 0.0, 0.0, decay],
    ])
    P_next = F @ P @ F.T + Q * (dt / max(nominal_dt, 1e-4))
    P_next = 0.5 * (P_next + P_next.T)
    return x_next, P_next


def compute_wheel_forward_omega(names, velocities):
    """Compute balanced forward wheel angular speed (rad/s), requiring both L and R sides when named."""
    if names and len(names) == len(velocities):
        left_vals = []
        right_vals = []
        for name, val in zip(names, velocities):
            if not math.isfinite(val):
                continue
            lname = name.lower()
            if 'left' in lname:
                left_vals.append(float(val))
            elif 'right' in lname:
                right_vals.append(float(val))
        if left_vals and right_vals:
            return 0.5 * (sum(left_vals) / len(left_vals) + sum(right_vals) / len(right_vals))
        return None

    speeds = [float(v) for v in velocities if math.isfinite(v)]
    if len(speeds) < 2:
        return None
    return sum(speeds) / len(speeds)


def euler_rad_to_quaternion(roll: float, pitch: float, yaw: float):
    """Convert euler angles in radians to quaternion (x, y, z, w)."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy

    return float(qx), float(qy), float(qz), float(qw)


class RoverEKFNode(Node):
    """
    ROS 2 Extended Kalman Filter (EKF) Node fusing:
    1. Wheel Odometry (/cmd_vel or motor topics)
    2. IMU Orientation & Angular Rates (/imu/euler & /imu/data_raw)
    3. GPS Absolute Position (/gps/fix)
    
    5D State Vector x = [X, Y, theta, v, omega]^T
    """

    def __init__(self):
        super().__init__('rover_ekf_node')

        # Parameters
        self.declare_parameter('frequency', 20.0)
        self.declare_parameter('topic_imu_euler', '/imu/euler')
        self.declare_parameter('topic_imu_raw', '/imu/data_raw')
        self.declare_parameter('topic_gps_fix', '/gps/fix')
        self.declare_parameter('topic_cmd_vel', '/cmd_vel')
        self.declare_parameter('topic_ekf_odom', '/ekf/odometry')
        self.declare_parameter('topic_ekf_euler', '/ekf/filtered_euler')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_link')
        # Set false for GNSS-denied runs (CIRC Meteorite Search bans GPS): the fix topic is then
        # not even subscribed, so a stale or stray fix cannot leak into the pose.
        self.declare_parameter('use_gps', True)
        self.use_gps = bool(self.get_parameter('use_gps').value)
        # /imu/euler yaw is counter-clockwise with 0 wherever ~/calibrate was called ("true north" by the
        # team's procedure); the state uses ENU (0 = east). theta = imu_yaw + offset, so 90 for a north tare.
        # (It used to be taken as ENU directly, putting dead reckoning 90 degrees off the GPS axes.)
        self.declare_parameter('imu_yaw_offset_deg', 90.0)
        # Wheel speed comes from /wheel_states when present, or /motor/* commands scaled by max wheel speed (m/s).
        self.declare_parameter('speed_per_motor_command', 1.0)
        # Disable when another node/simulator (e.g. Gazebo DiffDrive) already owns the odom -> base TF edge.
        self.declare_parameter('publish_tf', True)
        self.publish_tf = bool(self.get_parameter('publish_tf').value)
        # Measured wheel speed from vesc_can_driver_node (/wheel_states, wheel rad/s from VESC ERPM) is
        # preferred over the commands whenever it is fresh. Defaults to rover_description/config/physical_params.yaml
        # (0.15 m) to stay aligned with vesc_can_driver_node unless overridden via ROS parameter.
        self.declare_parameter('topic_wheel_states', '/wheel_states')
        default_wheel_radius = load_wheel_radius_from_config(default=0.15, logger=self.get_logger())
        self.declare_parameter('wheel_radius', default_wheel_radius)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        # /cmd_vel is a command, not a measurement, and was fused in addition to /motor/* (double counting)
        self.declare_parameter('fuse_cmd_vel', False)
        # Local frame origin. NaN = the first GPS fix. Set it to NAVnode's home waypoint so both share a frame.
        self.declare_parameter('datum_latitude', float('nan'))
        self.declare_parameter('datum_longitude', float('nan'))
        self.imu_yaw_offset = math.radians(float(self.get_parameter('imu_yaw_offset_deg').value))
        self.speed_per_command = float(self.get_parameter('speed_per_motor_command').value)
        self.fuse_cmd_vel = bool(self.get_parameter('fuse_cmd_vel').value)
        datum_lat = float(self.get_parameter('datum_latitude').value)
        datum_lon = float(self.get_parameter('datum_longitude').value)

        self.freq = float(self.get_parameter('frequency').value)
        self.dt = 1.0 / max(self.freq, 1.0)
        self.topic_imu_euler = str(self.get_parameter('topic_imu_euler').value)
        self.topic_imu_raw = str(self.get_parameter('topic_imu_raw').value)
        self.topic_gps_fix = str(self.get_parameter('topic_gps_fix').value)
        self.topic_cmd_vel = str(self.get_parameter('topic_cmd_vel').value)
        self.topic_ekf_odom = str(self.get_parameter('topic_ekf_odom').value)
        self.topic_ekf_euler = str(self.get_parameter('topic_ekf_euler').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.child_frame_id = str(self.get_parameter('child_frame_id').value)

        # 5D EKF State: x = [X, Y, theta, v, omega]^T
        self.x = np.zeros(5)
        
        # State Covariance Matrix P (5x5)
        self.P = np.diag([1.0, 1.0, 0.1, 0.5, 0.1])

        # Process Noise Covariance Q (5x5)
        self.Q = np.diag([0.05, 0.05, 0.01, 0.1, 0.05])

        # Measurement Noise Covariances R
        self.R_imu = np.diag([0.01, 0.005])   # [theta_imu, omega_imu]
        self.R_wheel = np.array([[0.08]])      # [v_wheel] from commands
        self.R_wheel_measured = np.array([[0.02]])   # [v_wheel] from VESC ERPM (slip still unmodelled)
        self.R_gps = np.diag([1.5, 1.5])      # [X_gps, Y_gps] in meters

        # Sensor inputs
        self.gps_origin = None  # (lat0, lon0, alt0)
        if math.isfinite(datum_lat) and math.isfinite(datum_lon):
            self.gps_origin = (datum_lat, datum_lon, 0.0)
        self.last_imu_roll = 0.0
        self.last_imu_pitch = 0.0

        # Subscriptions
        self.sub_imu_euler = self.create_subscription(
            Vector3, self.topic_imu_euler, self.imu_euler_callback, 10
        )
        self.sub_imu_raw = self.create_subscription(
            Imu, self.topic_imu_raw, self.imu_raw_callback, 10
        )
        self.sub_gps = None
        if self.use_gps:
            self.sub_gps = self.create_subscription(
                NavSatFix, self.topic_gps_fix, self.gps_fix_callback, 10
            )
        else:
            self.get_logger().warn("use_gps is false: GNSS is NOT fused (position is dead reckoning only).")
        self.sub_cmd_vel = None
        if self.fuse_cmd_vel:
            self.sub_cmd_vel = self.create_subscription(
                Twist, self.topic_cmd_vel, self.cmd_vel_callback, 10
            )

        # Motor velocity subscriptions for direct wheel odometry
        self.sub_fl_motor = self.create_subscription(Float32, '/motor/front_left', self.fl_motor_cb, 10)
        self.sub_fr_motor = self.create_subscription(Float32, '/motor/front_right', self.fr_motor_cb, 10)
        self.sub_rl_motor = self.create_subscription(Float32, '/motor/rear_left', self.rl_motor_cb, 10)
        self.sub_rr_motor = self.create_subscription(Float32, '/motor/rear_right', self.rr_motor_cb, 10)

        self.fl_v, self.fr_v, self.rl_v, self.rr_v = 0.0, 0.0, 0.0, 0.0
        self.last_motor_time = None          # ROS clock seconds of the last /motor/* message
        self.measured_speed = 0.0
        self.last_wheel_state_time = None    # ROS clock seconds of the last /wheel_states message
        self.sub_wheel_states = self.create_subscription(
            JointState, str(self.get_parameter('topic_wheel_states').value), self.wheel_states_cb, 10)
        self.last_predict_time = None

        # Publishers & TF Broadcaster
        self.pub_odom = self.create_publisher(Odometry, self.topic_ekf_odom, 10)
        self.pub_euler = self.create_publisher(Vector3, self.topic_ekf_euler, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer loop for EKF Prediction & Output
        self.timer = self.create_timer(self.dt, self.timer_ekf_loop)

        self.get_logger().info(
            f"Rover EKF Sensor Fusion Node active @ {self.freq} Hz. "
            f"Fusing Wheel Odometry, IMU ({self.topic_imu_euler}), and GPS ({self.topic_gps_fix})."
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _stamp_now_if_valid(self):
        now_s = self._now_sec()
        return now_s if now_s > 0.0 else None

    def fl_motor_cb(self, msg):
        if math.isfinite(msg.data):
            self.fl_v = float(msg.data)
            self.last_motor_time = self._stamp_now_if_valid()

    def fr_motor_cb(self, msg):
        if math.isfinite(msg.data):
            self.fr_v = float(msg.data)
            self.last_motor_time = self._stamp_now_if_valid()

    def rl_motor_cb(self, msg):
        if math.isfinite(msg.data):
            self.rl_v = float(msg.data)
            self.last_motor_time = self._stamp_now_if_valid()

    def rr_motor_cb(self, msg):
        if math.isfinite(msg.data):
            self.rr_v = float(msg.data)
            self.last_motor_time = self._stamp_now_if_valid()

    def wheel_states_cb(self, msg: JointState):
        """Measured wheel angular speeds (rad/s); balanced across left/right sides x radius."""
        forward_omega = compute_wheel_forward_omega(msg.name, msg.velocity)
        if forward_omega is None:
            return
        self.measured_speed = forward_omega * self.wheel_radius
        self.last_wheel_state_time = self._stamp_now_if_valid()

    def imu_euler_callback(self, msg: Vector3):
        """Measurement update for IMU Orientation (Yaw, Roll, Pitch)."""
        if not (math.isfinite(msg.x) and math.isfinite(msg.y) and math.isfinite(msg.z)):
            return
        self.last_imu_roll = math.radians(msg.x)
        self.last_imu_pitch = math.radians(msg.y)
        imu_yaw_rad = wrap_angle_rad(math.radians(msg.z) + self.imu_yaw_offset)   # to ENU

        H = np.array([[0.0, 0.0, 1.0, 0.0, 0.0]])
        y = np.array([wrap_angle_rad(imu_yaw_rad - float(self.x[2]))])
        self.x, self.P = joseph_update(self.x, self.P, H, y, self.R_imu[0:1, 0:1], wrap_angle_idx=2)

    def imu_raw_callback(self, msg: Imu):
        """Measurement update for IMU Angular Velocity."""
        omega_z = msg.angular_velocity.z
        if not math.isfinite(omega_z):
            return
        H = np.array([[0.0, 0.0, 0.0, 0.0, 1.0]])
        y = np.array([float(omega_z)]) - H @ self.x
        self.x, self.P = joseph_update(self.x, self.P, H, y, self.R_imu[1:2, 1:2])

    def cmd_vel_callback(self, msg: Twist):
        """Measurement update from linear velocity command or wheel odometry."""
        if math.isfinite(msg.linear.x):
            self.update_wheel_odometry(float(msg.linear.x))

    def update_wheel_odometry(self, v_wheel: float, R=None):
        """Apply EKF measurement update for forward speed v_wheel (R defaults to the command noise)."""
        if not math.isfinite(v_wheel):
            return
        if R is None:
            R = self.R_wheel
        H = np.array([[0.0, 0.0, 0.0, 1.0, 0.0]])
        y = np.array([float(v_wheel)]) - H @ self.x
        self.x, self.P = joseph_update(self.x, self.P, H, y, R)

    def gps_fix_callback(self, msg: NavSatFix):
        """Measurement update for absolute GPS Position."""
        if msg.status.status < 0 or not (math.isfinite(msg.latitude) and math.isfinite(msg.longitude)):
            return
        if abs(msg.latitude) < 1e-4 and abs(msg.longitude) < 1e-4:
            return

        # Set reference origin on first valid GPS fix when no external datum was supplied
        if self.gps_origin is None:
            self.gps_origin = (msg.latitude, msg.longitude, msg.altitude)
            self.get_logger().info(
                f"EKF GPS Reference Origin set: Lat={msg.latitude:.6f}, Lon={msg.longitude:.6f}"
            )
            return

        lat0, lon0, _ = self.gps_origin
        x_east, y_north = wgs84_enu_offset(msg.latitude, msg.longitude, lat0, lon0)

        z = np.array([x_east, y_north])
        H = np.array([
            [1.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 1.0, 0.0, 0.0, 0.0],
        ])
        y = z - H @ self.x
        R = self.R_gps
        if msg.position_covariance_type != NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            var_e, var_n = msg.position_covariance[0], msg.position_covariance[4]
            if var_e > 0.0 and var_n > 0.0 and np.isfinite(var_e) and np.isfinite(var_n):
                R = np.diag([var_e, var_n])
        self.x, self.P = joseph_update(self.x, self.P, H, y, R)

    def timer_ekf_loop(self):
        """EKF Prediction Step, synchronous wheel measurement update, and state publication."""
        now_s = self._now_sec()
        if now_s <= 0.0:
            return
        if self.last_predict_time is None:
            self.last_predict_time = now_s
            dt = self.dt
        else:
            dt = min(max(now_s - self.last_predict_time, 0.0), 0.5)
            self.last_predict_time = now_s

        # 1. Predict forward before applying current-tick wheel speed so steady velocity is not shrunk by decay
        self.x, self.P = predict_unicycle_state(self.x, self.P, self.Q, dt, self.dt)

        # 2. Wheel speed update (measured VESC/joint speeds preferred; motor commands as fallback)
        if self.last_wheel_state_time is not None and 0.0 <= (now_s - self.last_wheel_state_time) < 0.3:
            self.update_wheel_odometry(self.measured_speed, self.R_wheel_measured)
        elif self.last_motor_time is not None and 0.0 <= (now_s - self.last_motor_time) < 0.5:
            command = ((self.fl_v + self.rl_v) + (self.fr_v + self.rr_v)) * 0.25
            self.update_wheel_odometry(command * self.speed_per_command)

        # Publish Fused EKF Outputs
        now = self.get_clock().now().to_msg()
        qx, qy, qz, qw = euler_rad_to_quaternion(self.last_imu_roll, self.last_imu_pitch, self.x[2])

        # 1. Publish Odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.frame_id
        odom.child_frame_id = self.child_frame_id

        odom.pose.pose.position.x = float(self.x[0])
        odom.pose.pose.position.y = float(self.x[1])
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x = float(self.x[3])
        odom.twist.twist.angular.z = float(self.x[4])

        pose_cov = np.zeros((6, 6))
        pose_cov[0:2, 0:2] = self.P[0:2, 0:2]
        pose_cov[5, 5] = self.P[2, 2]
        odom.pose.covariance = pose_cov.flatten().tolist()

        twist_cov = np.zeros((6, 6))
        twist_cov[0, 0] = self.P[3, 3]
        twist_cov[5, 5] = self.P[4, 4]
        odom.twist.covariance = twist_cov.flatten().tolist()

        self.pub_odom.publish(odom)

        # 2. Publish Filtered Euler Angles (in degrees)
        euler_vec = Vector3()
        euler_vec.x = float(math.degrees(self.last_imu_roll))
        euler_vec.y = float(math.degrees(self.last_imu_pitch))
        euler_vec.z = float(math.degrees(self.x[2]))
        self.pub_euler.publish(euler_vec)

        # 3. Broadcast TF Transform (odom -> base_link) when EKF owns that TF edge
        if self.publish_tf:
            tf_stamped = TransformStamped()
            tf_stamped.header.stamp = now
            tf_stamped.header.frame_id = self.frame_id
            tf_stamped.child_frame_id = self.child_frame_id
            tf_stamped.transform.translation.x = float(self.x[0])
            tf_stamped.transform.translation.y = float(self.x[1])
            tf_stamped.transform.translation.z = 0.0
            tf_stamped.transform.rotation.x = qx
            tf_stamped.transform.rotation.y = qy
            tf_stamped.transform.rotation.z = qz
            tf_stamped.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(tf_stamped)


def main(args=None):
    rclpy.init(args=args)
    node = RoverEKFNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Rover EKF Node stopped cleanly.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
