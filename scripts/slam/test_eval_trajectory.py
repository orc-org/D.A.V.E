import math
import numpy as np
import pytest

from eval_trajectory import score_trajectory


def _yaw_quat(yaw_rad):
    return np.array([0.0, 0.0, math.sin(0.5 * yaw_rad), math.cos(0.5 * yaw_rad)])


def test_score_trajectory_rejects_disjoint_clocks():
    t_gt = np.linspace(1700000000.0, 1700000020.0, 50)
    p_gt = np.column_stack([np.linspace(0.0, 20.0, 50), np.zeros(50), np.zeros(50)])
    q_gt = np.tile([0.0, 0.0, 0.0, 1.0], (50, 1))

    # Estimate carries sim stamps starting near t=0 while bag gt uses Unix epoch receive time
    est = np.column_stack([
        np.linspace(0.0, 20.0, 50),
        np.linspace(0.0, 20.0, 50),
        np.zeros((50, 2)),
        np.tile([0.0, 0.0, 0.0, 1.0], (50, 1)),
    ])
    with pytest.raises(ValueError, match='try --est-clock sim'):
        score_trajectory(est, t_gt, p_gt, gt_q=q_gt, sim_recv=None)


def test_score_trajectory_detects_vector_closure_error_when_scalar_gap_matches():
    n = 40
    t = np.linspace(0.0, 20.0, n)
    p_gt = np.column_stack([np.linspace(0.0, 12.0, n), np.zeros(n), np.zeros(n)])
    q_gt = np.tile([0.0, 0.0, 0.0, 1.0], (n, 1))

    # Estimate folds backwards at end so |p[-1] - p[0]| == |p_gt[-1] - p_gt[0]| (= 12m),
    # wait: invert displacement sign at end relative to first segment via slight offset,
    # or check vector error differs from scalar gap when path ends near -12 vs +12.
    p_est = p_gt.copy()
    p_est[-1, 0] = -12.0  # scalar length is 12.0 m (scalar err ~ 0), vector residual is ~24 m!
    est = np.column_stack([t, p_est, q_gt])

    m = score_trajectory(est, t, p_gt, gt_q=q_gt, segment_m=5.0)
    assert m['loop_closure_scalar_m'] < 0.01
    assert m['loop_closure_vec_m'] > 20.0


def test_score_trajectory_reports_se3_rotation_error():
    n = 40
    t = np.linspace(0.0, 20.0, n)
    p_gt = np.column_stack([np.linspace(0.0, 20.0, n), np.zeros(n), np.zeros(n)])
    q_gt = np.array([_yaw_quat(0.0) for _ in t])

    # Estimate introduces 30 deg linear yaw drift over the run while positions are exact
    q_est = np.array([_yaw_quat(math.radians(30.0 * k / (n - 1))) for k in range(n)])
    est = np.column_stack([t, p_gt, q_est])
    m = score_trajectory(est, t, p_gt, gt_q=q_gt, segment_m=10.0)
    assert m['ate_rmse_m'] < 1e-6
    assert m['ate_rot_rmse_deg'] > 5.0
    assert m['rpe_rot_mean_deg'] > 5.0


def test_rpe_translation_is_local_frame_when_orientations_are_available():
    # Ground truth drives 60 m straight along +x. The estimate integrates the right forward motion but its
    # heading drifts linearly to 20 deg, so its path curves. Inside any 10 m segment the body-frame motion is
    # almost right (only ~3 deg of new drift); the world-frame comparison charges the whole accumulated heading
    # error to every segment instead.
    n = 601
    t = np.linspace(0.0, 60.0, n)
    s = np.linspace(0.0, 60.0, n)
    p_gt = np.column_stack([s, np.zeros(n), np.zeros(n)])
    q_gt = np.tile([0.0, 0.0, 0.0, 1.0], (n, 1))

    psi = math.radians(20.0) * s / 60.0
    ds = np.diff(s, prepend=0.0)
    p_est = np.column_stack([np.cumsum(np.cos(psi) * ds), np.cumsum(np.sin(psi) * ds), np.zeros(n)])
    q_est = np.array([_yaw_quat(a) for a in psi])

    m_local = score_trajectory(np.column_stack([t, p_est, q_est]), t, p_gt, gt_q=q_gt, segment_m=10.0)
    m_world = score_trajectory(np.column_stack([t, p_est]), t, p_gt, gt_q=q_gt, segment_m=10.0)

    assert m_local['rpe_frame'] == 'local'
    assert m_world['rpe_frame'] == 'world'
    assert m_local['rpe_mean_m'] < 0.5 * m_world['rpe_mean_m']
