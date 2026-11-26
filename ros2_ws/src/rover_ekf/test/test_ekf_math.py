import math
import numpy as np

from rover_ekf.ekf_node import (
    compute_wheel_forward_omega,
    joseph_update,
    load_wheel_radius_from_config,
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


def test_load_wheel_radius_from_config_matches_physical_params(monkeypatch):
    assert math.isclose(load_wheel_radius_from_config(default=0.99), 0.15)
    assert math.isclose(
        load_wheel_radius_from_config(default=0.15, config_path='/nonexistent/physical_params.yaml'),
        0.15,
    )
    import ament_index_python.packages as ament_pkgs
    monkeypatch.setattr(
        ament_pkgs,
        'get_package_share_directory',
        lambda _pkg: '/nonexistent/rover_description_share',
    )
    assert math.isclose(load_wheel_radius_from_config(default=0.12), 0.12)


def test_compute_wheel_forward_omega_ignores_non_wheel_joints():
    names = [
        'front_left_wheel_joint', 'rear_left_wheel_joint',
        'front_right_wheel_joint', 'rear_right_wheel_joint',
        'left_finger_joint', 'right_finger_joint',
    ]
    assert math.isclose(compute_wheel_forward_omega(names, [4.0, 4.0, 6.0, 6.0, 0.0, 0.0]), 5.0)


def test_compute_wheel_forward_omega_rejects_standalone_arm_joints():
    assert compute_wheel_forward_omega(
        ['arm_joint', 'left_finger_joint', 'right_finger_joint'],
        [0.0, 0.0, 0.0],
    ) is None


def test_joseph_update_handles_singular_innovation_nonfinite_and_wrapping():
    x = np.array([0.0, 0.0, 3.1, 0.0, 0.0])
    P = np.diag([1.0, 1.0, 0.2, 0.5, 0.1])
    H = np.array([[0.0, 0.0, 1.0, 0.0, 0.0]])

    # Singular innovation covariance S = 0 -> no crash, state preserved
    x_sing, P_sing = joseph_update(x, np.zeros((5, 5)), H, np.array([0.5]), np.array([[0.0]]), wrap_angle_idx=2)
    assert np.allclose(x_sing, x)
    assert np.allclose(P_sing, np.zeros((5, 5)))

    # Non-finite innovation -> no crash, state preserved
    x_nan, P_nan = joseph_update(x, P, H, np.array([float('nan')]), np.array([[0.01]]), wrap_angle_idx=2)
    assert np.allclose(x_nan, x)
    assert np.allclose(P_nan, P)

    # Out-of-range theta after update wraps to [-pi, pi]
    x_wrap, _ = joseph_update(x, P, H, np.array([0.5]), np.array([[0.01]]), wrap_angle_idx=2)
    assert -math.pi <= x_wrap[2] <= math.pi
