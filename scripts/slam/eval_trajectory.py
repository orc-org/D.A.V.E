#!/usr/bin/env python3
"""Score an odometry trajectory against the simulator's ground truth.

Inputs
  --est   a TUM-format trajectory ("t x y z qx qy qz qw" per line), e.g. KISS-ICP's *_poses_tum.txt
          (its timestamps are the bag receive times, so no clock conversion is needed)
  --bag   the UNCOMPRESSED rosbag2 directory the estimate was computed from; it must contain
          /sim/ground_truth_poses (see ros2_ws/src/rover_description/launch/sim.launch.py)
  --model name of the rover in the simulator (default dave_rover)
  --est-clock  'bag' (default): estimate timestamps are bag receive times (KISS-ICP's pipeline);
               'sim': they are simulation time (header stamps, e.g. FAST-LIO's /Odometry written by
               odom_to_tum.py); they are mapped to receive times through the bag's /clock messages

Metrics (positions in metres, orientations in degrees)
  ATE RMSE          RMS rigid-aligned (Umeyama) Euclidean translation error
  ATE rot RMSE      RMS orientation error after global Umeyama rotation + fixed sensor extrinsic fit
  z drift           largest height excursion both in raw estimate frame and relative to aligned ground truth
  loop closure err  vector closure residual |(aligned end - start) - (gt end - start)| and % of path
  RPE 10 m          Sturm et al. (2012) relative translation and rotation error per 10 m of true travel;
                    translation is compared in each segment's start frame when orientations are available
                    ("local"), otherwise in the aligned world frame ("world")
Requires: numpy, rosbags (pip install --user numpy "rosbags==0.9.23").
"""
import argparse
import math
import sys
from pathlib import Path

import numpy as np


def read_ground_truth(bag, model):
    from rosbags.highlevel import AnyReader
    times, pos, quats = [], [], []
    with AnyReader([Path(bag)]) as reader:
        conns = [c for c in reader.connections if c.topic == '/sim/ground_truth_poses']
        if not conns:
            raise ValueError('bag has no /sim/ground_truth_poses topic')
        for conn, t, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)
            for tf in msg.transforms:
                if tf.child_frame_id == model:
                    v = tf.transform.translation
                    q = tf.transform.rotation
                    times.append(t * 1e-9)
                    pos.append((v.x, v.y, v.z))
                    quats.append((q.x, q.y, q.z, q.w))
    if not times:
        raise ValueError(f'model "{model}" not found in the ground-truth poses')
    order = np.argsort(times)
    return np.array(times)[order], np.array(pos)[order], np.array(quats)[order]


def read_clock_map(bag):
    """(sim_time, receive_time) pairs from the bag's /clock topic, both in seconds."""
    from rosbags.highlevel import AnyReader
    sim, recv = [], []
    with AnyReader([Path(bag)]) as reader:
        conns = [c for c in reader.connections if c.topic == '/clock']
        if not conns:
            raise ValueError('--est-clock sim needs a /clock topic in the bag')
        for conn, t, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)
            sim.append(msg.clock.sec + msg.clock.nanosec * 1e-9)
            recv.append(t * 1e-9)
    if len(sim) < 2:
        raise ValueError(f'--est-clock sim needs at least 2 /clock messages in the bag, found {len(sim)}')
    sim, recv = np.array(sim), np.array(recv)
    order = np.argsort(sim)
    return sim[order], recv[order]


def umeyama(src, dst):
    """Rigid transform (R, t) minimising |R src + t - dst|, no scale."""
    mu_s, mu_d = src.mean(0), dst.mean(0)
    cov = (dst - mu_d).T @ (src - mu_s) / len(src)
    u, _, vt = np.linalg.svd(cov)
    s = np.eye(3)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        s[2, 2] = -1
    r = u @ s @ vt
    return r, mu_d - r @ mu_s


def project_so3(mat):
    """Project a 3x3 matrix onto the nearest proper rotation matrix in SO(3)."""
    u, _, vt = np.linalg.svd(mat)
    s = np.eye(3)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        s[2, 2] = -1
    return u @ s @ vt


def quat_to_rotmat(q):
    """Convert normalized [qx, qy, qz, qw] quaternion array into 3x3 rotation matrix."""
    n = np.linalg.norm(q)
    if not np.isfinite(n) or n < 1e-12:
        return np.eye(3)
    x, y, z, w = q / n
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),       2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w),       1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w),       2.0 * (y * z + x * w),       1.0 - 2.0 * (x * x + y * y)],
    ])


def slerp_quats(query_t, ref_t, ref_q):
    """Sample unit quaternions [qx, qy, qz, qw] via spherical linear interpolation."""
    idx = np.searchsorted(ref_t, query_t, side='right') - 1
    idx = np.clip(idx, 0, len(ref_t) - 2)
    out = np.empty((len(query_t), 4))
    for k, i in enumerate(idx):
        dt = ref_t[i + 1] - ref_t[i]
        alpha = float(np.clip((query_t[k] - ref_t[i]) / dt, 0.0, 1.0)) if dt > 1e-12 else 0.0
        q0 = ref_q[i] / max(np.linalg.norm(ref_q[i]), 1e-12)
        q1 = ref_q[i + 1] / max(np.linalg.norm(ref_q[i + 1]), 1e-12)
        dot = float(np.dot(q0, q1))
        if dot < 0.0:
            q1 = -q1
            dot = -dot
        if dot > 0.9995:
            q = (1.0 - alpha) * q0 + alpha * q1
        else:
            theta = math.acos(min(1.0, dot))
            sin_t = math.sin(theta)
            q = (math.sin((1.0 - alpha) * theta) * q0 + math.sin(alpha * theta) * q1) / sin_t
        out[k] = q / max(np.linalg.norm(q), 1e-12)
    return out


def rotation_angle_deg(R1, R2):
    """Geodesic angle in degrees between two SO(3) rotation matrices."""
    trace_val = float(np.trace(R1.T @ R2))
    cos_a = max(-1.0, min(1.0, 0.5 * (trace_val - 1.0)))
    return math.degrees(math.acos(cos_a))


def path_length(p):
    if len(p) < 2:
        return 0.0
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())


def score_trajectory(est, gt_t, gt_p, gt_q=None, sim_recv=None, segment_m=10.0):
    """Evaluate position and SE(3) rotation metrics; raises ValueError on empty timestamp overlap."""
    est = np.atleast_2d(est)
    if est.shape[0] < 2 or est.shape[1] < 4:
        raise ValueError(f'estimate trajectory needs at least 2 poses with >=4 columns, got shape {est.shape}')
    if not np.all(np.isfinite(est)):
        raise ValueError('estimate trajectory contains NaN or Inf values')

    et, ep = est[:, 0], est[:, 1:4]
    has_quats = est.shape[1] >= 8 and gt_q is not None and len(gt_q) == len(gt_t)
    eq = est[:, 4:8] if has_quats else None

    if sim_recv is not None:
        sim, recv = sim_recv
        inside = (et >= sim[0]) & (et <= sim[-1])
        if inside.sum() < 2:
            raise ValueError(
                f'fewer than 2 estimate timestamps fall inside bag /clock [{sim[0]:.3f}, {sim[-1]:.3f}] '
                f'(est span [{et[0]:.3f}, {et[-1]:.3f}])'
            )
        et = np.interp(et[inside], sim, recv)
        ep = ep[inside]
        if eq is not None:
            eq = eq[inside]

    keep = (et >= gt_t[0]) & (et <= gt_t[-1])
    if keep.sum() < 2:
        raise ValueError(
            f'timestamp overlap with ground truth has {int(keep.sum())} pose(s) '
            f'(est [{et[0]:.3f}, {et[-1]:.3f}] vs gt [{gt_t[0]:.3f}, {gt_t[-1]:.3f}]; '
            f'try --est-clock sim if estimates use header sim stamps)'
        )
    et, ep = et[keep], ep[keep]
    if eq is not None:
        eq = eq[keep]
    g = np.stack([np.interp(et, gt_t, gt_p[:, i]) for i in range(3)], axis=1)

    r, t = umeyama(ep, g)
    ea = ep @ r.T + t
    err = np.linalg.norm(ea - g, axis=1)

    sub = slice(None, None, 10)
    true_len_raw = path_length(g)
    est_len_raw = path_length(ep)
    true_len_sub = path_length(g[sub])
    est_len_sub = path_length(ep[sub])

    vec_closure_err = float(np.linalg.norm((ea[-1] - ea[0]) - (g[-1] - g[0])))
    scalar_gap_err = float(abs(np.linalg.norm(ep[-1] - ep[0]) - np.linalg.norm(g[-1] - g[0])))
    loop_pct = (100.0 * vec_closure_err / true_len_sub) if true_len_sub > 1e-6 else float('nan')

    metrics = {
        'poses_compared': int(len(et)),
        'true_path_m': true_len_raw,
        'est_path_m': est_len_raw,
        'scale_factor': (est_len_raw / true_len_raw) if true_len_raw > 1e-6 else float('nan'),
        'est_path_sub_m': est_len_sub,
        'true_path_sub_m': true_len_sub,
        'ate_rmse_m': float(np.sqrt((err ** 2).mean())),
        'ate_max_m': float(err.max()),
        'end_error_est_m': float(np.linalg.norm(ep[-1] - ep[0])),
        'end_error_gt_m': float(np.linalg.norm(g[-1] - g[0])),
        'z_drift_raw_m': float(np.abs(ep[:, 2] - ep[0, 2]).max()),
        'z_drift_aligned_m': float(np.abs(ea[:, 2] - g[:, 2]).max()),
        'loop_closure_vec_m': vec_closure_err,
        'loop_closure_scalar_m': scalar_gap_err,
        'loop_drift_pct': loop_pct,
        'segment_m': float(segment_m),
    }

    R_est_aligned = None
    R_gt_seq = None
    if eq is not None and len(gt_t) >= 2:
        gq = slerp_quats(et, gt_t, gt_q)
        R_est_world = np.array([r @ quat_to_rotmat(q) for q in eq])
        R_gt_seq = np.array([quat_to_rotmat(q) for q in gq])
        # Fit static sensor-to-body extrinsic rotation so sensor-frame conventions cancel cleanly
        H_bs = np.einsum('nij,njk->ik', R_est_world.transpose(0, 2, 1), R_gt_seq)
        R_bs = project_so3(H_bs)
        R_est_aligned = R_est_world @ R_bs
        rot_errs = np.array([rotation_angle_deg(R_est_aligned[k], R_gt_seq[k]) for k in range(len(et))])
        metrics['ate_rot_rmse_deg'] = float(np.sqrt((rot_errs ** 2).mean()))
        metrics['ate_rot_max_deg'] = float(rot_errs.max())

    dist = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(g, axis=0), axis=1))])
    j = np.searchsorted(dist, dist + segment_m)
    ok = j < len(dist)
    if ok.any():
        i_idx, j_idx = np.nonzero(ok)[0], j[ok]
        d_est = ea[j_idx] - ea[i_idx]
        d_gt = g[j_idx] - g[i_idx]
        if R_est_aligned is not None:
            # Sturm et al. (2012): compare the displacements in each segment's own start frame, so heading
            # error accumulated before the segment is not counted as translation error inside it.
            d_est = np.einsum('nji,nj->ni', R_est_aligned[i_idx], d_est)
            d_gt = np.einsum('nji,nj->ni', R_gt_seq[i_idx], d_gt)
        metrics['rpe_frame'] = 'local' if R_est_aligned is not None else 'world'
        rpe_trans = np.linalg.norm(d_est - d_gt, axis=1)
        metrics['rpe_mean_m'] = float(rpe_trans.mean())
        metrics['rpe_max_m'] = float(rpe_trans.max())
        metrics['rpe_pct'] = float(100.0 * rpe_trans.mean() / segment_m)
        metrics['rpe_segments'] = int(len(rpe_trans))
        if R_est_aligned is not None:
            rpe_rot = []
            for a_i, b_i in zip(i_idx, j_idx):
                dR_est = R_est_aligned[a_i].T @ R_est_aligned[b_i]
                dR_gt = R_gt_seq[a_i].T @ R_gt_seq[b_i]
                rpe_rot.append(rotation_angle_deg(dR_est, dR_gt))
            rpe_rot = np.array(rpe_rot)
            metrics['rpe_rot_mean_deg'] = float(rpe_rot.mean())
            metrics['rpe_rot_max_deg'] = float(rpe_rot.max())

    return metrics


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--est', required=True)
    ap.add_argument('--bag', required=True)
    ap.add_argument('--model', default='dave_rover')
    ap.add_argument('--est-clock', choices=('bag', 'sim'), default='bag')
    a = ap.parse_args()

    est = np.loadtxt(a.est)
    try:
        sim_recv = read_clock_map(a.bag) if a.est_clock == 'sim' else None
        gt_t, gt_p, gt_q = read_ground_truth(a.bag, a.model)
        m = score_trajectory(est, gt_t, gt_p, gt_q=gt_q, sim_recv=sim_recv)
    except ValueError as exc:
        sys.exit(f'eval_trajectory error: {exc}')

    print(f'poses compared        : {m["poses_compared"]}')
    print(f'true path length      : {m["true_path_m"]:.2f} m')
    print(f'estimated path length : {m["est_path_m"]:.2f} m   (scale {m["scale_factor"]:.3f})')
    print(f'path length, every 10th pose: estimate {m["est_path_sub_m"]:.2f} m vs true {m["true_path_sub_m"]:.2f} m')
    print(f'ATE RMSE              : {m["ate_rmse_m"]:.3f} m   (max {m["ate_max_m"]:.3f} m)')
    if 'ate_rot_rmse_deg' in m:
        print(f'ATE rotation RMSE     : {m["ate_rot_rmse_deg"]:.3f} deg (max {m["ate_rot_max_deg"]:.3f} deg)')
    print(f'end error (estimate)  : {m["end_error_est_m"]:.3f} m')
    print(f'end error (true path) : {m["end_error_gt_m"]:.3f} m   (nonzero means the open-loop drive did not close)')
    print(f'z drift (raw/aligned) : {m["z_drift_raw_m"]:.3f} m / {m["z_drift_aligned_m"]:.3f} m')
    print(f'loop drift (vec/scal) : {m["loop_closure_vec_m"]:.3f} m ({m["loop_drift_pct"]:.2f} %) | scalar {m["loop_closure_scalar_m"]:.3f} m')
    if 'rpe_mean_m' in m:
        rot_str = ''
        if 'rpe_rot_mean_deg' in m:
            rot_str = f' | rot mean {m["rpe_rot_mean_deg"]:.3f} deg (max {m["rpe_rot_max_deg"]:.3f} deg)'
        print(
            f'RPE {m["segment_m"]:.0f} m ({m["rpe_frame"]:5s})      : mean {m["rpe_mean_m"]:.3f} m ({m["rpe_pct"]:.2f} %), '
            f'max {m["rpe_max_m"]:.3f} m ({m["rpe_segments"]} segments){rot_str}'
        )


if __name__ == '__main__':
    main()
