import math
import numpy as np

from rover_ekf.ekf_node import (
    compute_wheel_forward_omega,
    joseph_update,
    predict_unicycle_state,
    wgs84_enu_offset,
    wrap_angle_rad,
)


def test_wgs84_enu_offset_matches_meridional_and_parallel_radii():
    # At mid-latitude (51.47 N, Drumheller / Calgary scale), 1 deg north is ~111,260 m on WGS-84,
    # whereas 1 deg east scales by cos(lat) * R_N (~69,395 m)
    e, n = wgs84_enu_offset(51.47 + 1e-3, -112.71 + 1e-3, 51.47, -112.71)
    assert 69.0 < e < 70.0
    assert 111.0 < n < 112.0


def test_joseph_update_preserves_symmetry_and_psd():
    x = np.array([1.0, -2.0, 3.1, 0.5, 0.1])
    P = np.diag([2.0, 2.0, 0.4, 0.3, 0.2])
    H = np.array([[0.0, 0.0, 1.0, 0.0, 0.0]])
    y = np.array([wrap_angle_rad(-3.1 - x[2])])
    R = np.array([[1e-4]])

    x_upd, P_upd = joseph_update(x, P, H, y, R, wrap_angle_idx=2)
    assert abs(x_upd[2]) <= math.pi
    assert np.allclose(P_upd, P_upd.T, atol=1e-12)
    eigvals = np.linalg.eigvalsh(P_upd)
    assert np.all(eigvals > 0.0)


def test_compute_wheel_forward_omega_requires_both_sides_when_named():
    names = ['front_left_wheel_joint', 'rear_left_wheel_joint',
             'front_right_wheel_joint', 'rear_right_wheel_joint']
    assert math.isclose(compute_wheel_forward_omega(names, [4.0, 4.0, 6.0, 6.0]), 5.0)

    # Missing right side should reject asymmetrical estimate
    assert compute_wheel_forward_omega(
        ['front_left_wheel_joint', 'rear_left_wheel_joint'], [4.0, 4.0]
    ) is None

    # Named mixed with NaN on one side rejects unless other wheel on same side is finite
    assert math.isclose(
        compute_wheel_forward_omega(names, [4.0, float('nan'), 6.0, float('nan')]),
        5.0,
    )
    assert compute_wheel_forward_omega(names, [4.0, 4.0, float('nan'), float('nan')]) is None


def test_predict_then_wheel_update_does_not_underestimate_steady_velocity():
    x = np.array([0.0, 0.0, 0.0, 1.0, 0.0])
    P = np.diag([0.1, 0.1, 0.05, 0.2, 0.05])
    Q = np.diag([0.01, 0.01, 0.005, 0.05, 0.01])

    for _ in range(40):
        x, P = predict_unicycle_state(x, P, Q, dt=0.05, nominal_dt=0.05)
        H = np.array([[0.0, 0.0, 0.0, 1.0, 0.0]])
        y = np.array([1.0]) - H @ x
        x, P = joseph_update(x, P, H, y, np.array([[0.02]]))

    assert abs(x[3] - 1.0) < 0.01
    assert abs(x[0] - 2.0) < 0.1
