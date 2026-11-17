#!/usr/bin/env python3
"""Add the per-point `time` field that LiDAR-inertial odometry expects to the simulated cloud.

The real Unitree L2 driver publishes x, y, z, intensity, ring and a per-point `time` (seconds since the
scan start), which LIO uses to undo the motion during each 0.1 s sweep (deskew). Gazebo's gpu_lidar
publishes x, y, z, intensity, ring only, and captures each scan at ONE instant, so there is no motion
distortion to undo.

This node republishes /lidar/points as /lidar/points_timed with fields x, y, z, intensity (float32),
time (float32, seconds) and ring (uint16), dropping no-return (non-finite) points. Every point gets the
SAME small time (`point_time`, default 1 microsecond):
  - all points at one instant = no deskew, which is correct for the simulator;
  - it must not be 0: FAST-LIO treats an all-zero time field as "missing" and invents times from each
    point's azimuth, which would "correct" distortion that is not there (up to v*0.1 s and w*0.1 s).

Usage (simulation or a replayed simulation bag):
    python3 scripts/slam/lidar_point_time.py --ros-args -p use_sim_time:=true
Parameters: input_topic (/lidar/points), output_topic (/lidar/points_timed), point_time (1e-6 s).
Not for the real L2: its driver already provides true per-point times.
"""
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField

_NP = {PointField.INT8: np.int8, PointField.UINT8: np.uint8, PointField.INT16: np.int16,
       PointField.UINT16: np.uint16, PointField.INT32: np.int32, PointField.UINT32: np.uint32,
       PointField.FLOAT32: np.float32, PointField.FLOAT64: np.float64}

OUT_DTYPE = np.dtype([('x', '<f4'), ('y', '<f4'), ('z', '<f4'), ('intensity', '<f4'),
                      ('time', '<f4'), ('ring', '<u2'), ('_pad', '<u2')])   # 24 bytes per point
OUT_FIELDS = [PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
              PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
              PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
              PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
              PointField(name='time', offset=16, datatype=PointField.FLOAT32, count=1),
              PointField(name='ring', offset=20, datatype=PointField.UINT16, count=1)]


def read_field(msg, raw, name):
    f = next((f for f in msg.fields if f.name == name), None)
    if f is None:
        return None
    dt = np.dtype(_NP[f.datatype]).newbyteorder('>' if msg.is_bigendian else '<')
    return np.ndarray(shape=(len(raw) // msg.point_step,), dtype=dt, buffer=raw,
                      offset=f.offset, strides=(msg.point_step,))


class LidarPointTime(Node):
    def __init__(self):
        super().__init__('lidar_point_time')
        self.declare_parameter('input_topic', '/lidar/points')
        self.declare_parameter('output_topic', '/lidar/points_timed')
        self.declare_parameter('point_time', 1e-6)
        self.point_time = float(self.get_parameter('point_time').value)
        self.pub = self.create_publisher(PointCloud2, self.get_parameter('output_topic').value, qos_profile_sensor_data)
        self.create_subscription(PointCloud2, self.get_parameter('input_topic').value, self.cb, qos_profile_sensor_data)

    def cb(self, msg):
        raw = bytes(msg.data)
        x, y, z = (read_field(msg, raw, n) for n in ('x', 'y', 'z'))
        if x is None or y is None or z is None:
            self.get_logger().error('input cloud has no x/y/z', throttle_duration_sec=5.0)
            return
        inten = read_field(msg, raw, 'intensity')
        ring = read_field(msg, raw, 'ring')
        keep = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        out = np.zeros(int(keep.sum()), dtype=OUT_DTYPE)
        out['x'], out['y'], out['z'] = x[keep], y[keep], z[keep]
        if inten is not None:
            out['intensity'] = inten[keep]
        if ring is not None:
            out['ring'] = ring[keep]
        out['time'] = self.point_time

        o = PointCloud2()
        o.header = msg.header
        o.height, o.width = 1, len(out)
        o.fields = OUT_FIELDS
        o.is_bigendian = False
        o.point_step = OUT_DTYPE.itemsize
        o.row_step = o.point_step * o.width
        o.is_dense = True
        o.data = out.tobytes()
        self.pub.publish(o)


def main():
    rclpy.init()
    node = LidarPointTime()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
