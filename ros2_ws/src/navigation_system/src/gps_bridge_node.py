#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import String
import serial

def verify_nmea_checksum(line: str) -> bool:
    """Validate NMEA 0183 checksum (*XX)."""
    if not line.startswith('$') or '*' not in line:
        return False
    try:
        data, checksum_str = line[1:].split('*', 1)
        expected_checksum = int(checksum_str[:2], 16)
        calc_checksum = 0
        for char in data:
            calc_checksum ^= ord(char)
        return calc_checksum == expected_checksum
    except ValueError:
        return False

def nmea_to_decimal(raw_val: str, direction: str) -> float:
    """
    Convert NMEA coordinate format (DDMM.MMMM / DDDMM.MMMM) to Decimal Degrees.
    """
    if not raw_val:
        return 0.0
    val = float(raw_val)
    deg = int(val / 100)
    minutes = val - (deg * 100)
    decimal = deg + (minutes / 60.0)
    if direction in ['S', 'W']:
        decimal = -decimal
    return decimal

# GGA fix quality -> (NavSatStatus, horizontal UERE in metres). sigma_horizontal = HDOP * UERE.
# The UERE figures are typical, not measured for this receiver: tune them (parameters below) from a
# static recording. Quality 6 is the receiver's own dead-reckoning estimate and 0 is no fix; both are
# reported as STATUS_NO_FIX, which rover_ekf ignores.
GGA_QUALITY = {
    1: (NavSatStatus.STATUS_FIX, 'uere_standalone_m'),        # single-point GNSS
    2: (NavSatStatus.STATUS_SBAS_FIX, 'uere_dgps_m'),         # DGPS / SBAS
    4: (NavSatStatus.STATUS_GBAS_FIX, 'uere_rtk_fixed_m'),    # RTK fixed
    5: (NavSatStatus.STATUS_GBAS_FIX, 'uere_rtk_float_m'),    # RTK float
}


class GPSBridgeNode(Node):
    def __init__(self):
        super().__init__('gps_bridge_node')

        # Declare ROS 2 parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud', 38400)
        self.declare_parameter('frame_id', 'gps_link')

        self.port = self.get_parameter('port').get_parameter_value().string_value
        self.baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        for name, default in (('uere_standalone_m', 2.5), ('uere_dgps_m', 1.0),
                              ('uere_rtk_float_m', 0.3), ('uere_rtk_fixed_m', 0.02)):
            self.declare_parameter(name, default)
        # RMC carries no fix quality, HDOP or altitude, so it is only used when the receiver sends no GGA
        self.last_gga_time = None

        # ROS 2 Publishers
        self.fix_pub = self.create_publisher(NavSatFix, '/gps/fix', 10)
        self.raw_nmea_pub = self.create_publisher(String, '/gps/nmea_raw', 10)
        # self.info_for_navigation_display = self.create_publisher(DisplayInfo, '/gps/display_info', 10)

        self.buffer = ""

        self.get_logger().info(f"Opening serial port {self.port} at {self.baud} baud...")

        try:
            # Set timeout=0 for non-blocking reads to avoid blocking the ROS 2 executor thread
            self.ser = serial.Serial(self.port, self.baud, timeout=0)
            self.get_logger().info(f"Successfully connected to GPS device at {self.port}")
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port {self.port}: {e}")
            self.ser = None

        # Polling timer (20 Hz / 50ms)
        self.timer = self.create_timer(0.05, self.read_serial_data)

    def read_serial_data(self):
        if not rclpy.ok() or self.ser is None or not self.ser.is_open:
            return

        try:
            waiting = self.ser.in_waiting
            if waiting > 0:
                raw_bytes = self.ser.read(waiting)
                text = raw_bytes.decode('ascii', errors='ignore')
                self.buffer += text

                while '\n' in self.buffer:
                    line, self.buffer = self.buffer.split('\n', 1)
                    line = line.strip()

                    if not line or not line.startswith('$'):
                        continue

                    # Publish raw NMEA string
                    nmea_msg = String()
                    nmea_msg.data = line
                    self.raw_nmea_pub.publish(nmea_msg)

                    # Validate NMEA checksum
                    if not verify_nmea_checksum(line):
                        continue

                    # Parse relevant NMEA sentences
                    if line.startswith(('$GNGGA', '$GPGGA')):
                        self.parse_gga(line)
                    elif line.startswith(('$GNRMC', '$GPRMC')):
                        self.parse_rmc(line)

        except Exception as e:
            if rclpy.ok():
                self.get_logger().warn(f"Error reading/parsing serial data: {e}")

    def parse_gga(self, line: str):
        parts = line.split(',')
        if len(parts) < 10:
            return

        self.last_gga_time = self.get_clock().now()
        try:
            quality = int(parts[6]) if parts[6] else 0
        except ValueError:
            quality = 0

        fix_msg = NavSatFix()
        fix_msg.header.stamp = self.get_clock().now().to_msg()
        fix_msg.header.frame_id = self.frame_id
        fix_msg.status.service = NavSatStatus.SERVICE_GPS

        if quality not in GGA_QUALITY:
            # Used to publish nothing, so "no fix" and "bridge dead" looked the same downstream; quality 6
            # (dead reckoning) was published as a real fix. Now an explicit NO_FIX; consumers must check status.
            # Position is 0.0 rather than NaN: rosbridge serialises NaN as a bare token the browser cannot parse.
            fix_msg.status.status = NavSatStatus.STATUS_NO_FIX
            fix_msg.latitude = fix_msg.longitude = fix_msg.altitude = 0.0
            fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
            if rclpy.ok():
                self.fix_pub.publish(fix_msg)
            self.get_logger().info(f"GPS Status: no usable fix (GGA quality {quality})", throttle_duration_sec=5.0)
            return

        try:
            lat = nmea_to_decimal(parts[2], parts[3])
            lon = nmea_to_decimal(parts[4], parts[5])
            alt = float(parts[9]) if parts[9] else 0.0
            hdop = float(parts[8]) if parts[8] else None

            status, uere_param = GGA_QUALITY[quality]
            fix_msg.status.status = status
            fix_msg.latitude = lat
            fix_msg.longitude = lon
            fix_msg.altitude = alt

            if hdop is not None and hdop > 0.0:
                # Approximate covariance from HDOP (ENU, row-major); vertical error is taken as 2x horizontal.
                sigma_h = hdop * self.get_parameter(uere_param).value
                var_h = sigma_h * sigma_h
                fix_msg.position_covariance = [var_h, 0.0, 0.0,
                                               0.0, var_h, 0.0,
                                               0.0, 0.0, 4.0 * var_h]
                fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            else:
                fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

            if rclpy.ok():
                self.fix_pub.publish(fix_msg)
                self.get_logger().info(f"Published Fix: Lat={lat:.6f}, Lon={lon:.6f}, Alt={alt:.2f}m, "
                                       f"quality={quality}, HDOP={hdop}", throttle_duration_sec=2.0)

        except (ValueError, IndexError) as e:
            if rclpy.ok():
                self.get_logger().warn(f"Failed to parse GGA line: {line} - Error: {e}")

    def parse_rmc(self, line: str):
        parts = line.split(',')
        if len(parts) < 8:
            return

        status = parts[2]
        if status != 'A':
            return

        # Every epoch used to be published twice (GGA, then RMC with altitude 0), so the EKF fused each fix
        # twice and the altitude alternated. RMC is now a fallback for receivers that send no GGA.
        if self.last_gga_time is not None and \
                (self.get_clock().now() - self.last_gga_time).nanoseconds < 2e9:
            return

        try:
            lat = nmea_to_decimal(parts[3], parts[4])
            lon = nmea_to_decimal(parts[5], parts[6])

            fix_msg = NavSatFix()
            fix_msg.header.stamp = self.get_clock().now().to_msg()
            fix_msg.header.frame_id = self.frame_id
            fix_msg.status.status = NavSatStatus.STATUS_FIX
            fix_msg.status.service = NavSatStatus.SERVICE_GPS
            fix_msg.latitude = lat
            fix_msg.longitude = lon
            fix_msg.altitude = 0.0   # RMC has no altitude
            fix_msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN

            if rclpy.ok():
                self.fix_pub.publish(fix_msg)

        except (ValueError, IndexError) as e:
            if rclpy.ok():
                self.get_logger().warn(f"Failed to parse RMC line: {line} - Error: {e}")

    def destroy_node(self):
        if hasattr(self, 'ser') and self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("Closed serial connection.")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GPSBridgeNode()
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
