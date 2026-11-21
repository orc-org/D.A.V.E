from imu.telemetry import quaternion_to_euler_deg


def test_quaternion_to_euler_deg_identifies_axes():
    r, p, y = quaternion_to_euler_deg(0.0, 0.0, 0.0, 1.0)
    assert abs(r) < 1e-6 and abs(p) < 1e-6 and abs(y) < 1e-6

    # +90 deg rotation around Z axis
    import math
    s = math.sin(math.pi / 4.0)
    c = math.cos(math.pi / 4.0)
    r, p, y = quaternion_to_euler_deg(0.0, 0.0, s, c)
    assert abs(r) < 1e-5
    assert abs(p) < 1e-5
    assert abs(y - 90.0) < 1e-5
