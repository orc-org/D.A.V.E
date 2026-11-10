#!/usr/bin/env python3
"""Score an odometry trajectory against the simulator's ground truth.

Inputs
  --est   a TUM-format trajectory ("t x y z qx qy qz qw" per line), e.g. KISS-ICP's *_poses_tum.txt
          (its timestamps are the bag receive times, so no clock conversion is needed)
  --bag   the UNCOMPRESSED rosbag2 directory the estimate was computed from; it must contain
          /sim/ground_truth_poses (see ros2_ws/src/rover_description/launch/sim.launch.py)
  --model name of the rover in the simulator (default dave_rover)

A rosbag recorded with scripts/slam/record_bag.sh is zstd-compressed, which the `rosbags` Python
library cannot read; convert it first (docs/slam/research-plan.md, E1):
  printf 'output_bags:\n- uri: OUT_DIR\n  storage_id: sqlite3\n  all: true\n' > conv.yaml
  ros2 bag convert -i COMPRESSED_BAG -o conv.yaml

Metrics (positions only, metres)
  ATE RMSE   root-mean-square distance after the best rigid alignment (Umeyama, no scale) of the
             estimate onto the ground truth; the estimate starts at the origin of its own frame
  end error  distance between the first and last estimated position when the true path is a closed
             loop (smaller is better); the ground-truth value is printed for reference
  scale      estimated path length divided by true path length (1.0 is perfect); inflated by
             frame-to-frame jitter, so the every-10th-pose lengths are printed as well
  z drift    largest height excursion of the estimate on flat ground
Requires: numpy, rosbags (pip install --user numpy "rosbags==0.9.23").
"""
import argparse
import sys
from pathlib import Path

import numpy as np


def read_ground_truth(bag, model):
    from rosbags.highlevel import AnyReader
    times, pos = [], []
    with AnyReader([Path(bag)]) as reader:
        conns = [c for c in reader.connections if c.topic == '/sim/ground_truth_poses']
        if not conns:
            sys.exit('bag has no /sim/ground_truth_poses topic')
        for conn, t, raw in reader.messages(connections=conns):
            msg = reader.deserialize(raw, conn.msgtype)
            for tf in msg.transforms:
                if tf.child_frame_id == model:
                    v = tf.transform.translation
                    times.append(t * 1e-9)
                    pos.append((v.x, v.y, v.z))
    if not times:
        sys.exit(f'model "{model}" not found in the ground-truth poses')
    return np.array(times), np.array(pos)


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


def path_length(p):
    return float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--est', required=True)
    ap.add_argument('--bag', required=True)
    ap.add_argument('--model', default='dave_rover')
    a = ap.parse_args()

    est = np.loadtxt(a.est)
    et, ep = est[:, 0], est[:, 1:4]
    gt_t, gt_p = read_ground_truth(a.bag, a.model)

    # ground truth sampled at the estimate's timestamps (linear interpolation)
    keep = (et >= gt_t[0]) & (et <= gt_t[-1])
    et, ep = et[keep], ep[keep]
    g = np.stack([np.interp(et, gt_t, gt_p[:, i]) for i in range(3)], axis=1)

    r, t = umeyama(ep, g)
    err = np.linalg.norm((ep @ r.T + t) - g, axis=1)
    print(f'poses compared        : {len(et)}')
    print(f'true path length      : {path_length(g):.2f} m')
    print(f'estimated path length : {path_length(ep):.2f} m   (scale {path_length(ep) / path_length(g):.3f})')
    # Summing every frame-to-frame step counts sensor jitter as travel (worst while stationary or turning),
    # so also report the length with every 10th pose, which is the fairer scale measure.
    sub = slice(None, None, 10)
    print(f'path length, every 10th pose: estimate {path_length(ep[sub]):.2f} m vs true {path_length(g[sub]):.2f} m')
    print(f'ATE RMSE              : {np.sqrt((err ** 2).mean()):.3f} m   (max {err.max():.3f} m)')
    print(f'end error (estimate)  : {np.linalg.norm(ep[-1] - ep[0]):.3f} m')
    print(f'end error (true path) : {np.linalg.norm(g[-1] - g[0]):.3f} m   (nonzero means the open-loop drive did not close)')
    print(f'z drift (estimate)    : {np.abs(ep[:, 2] - ep[0, 2]).max():.3f} m')


if __name__ == '__main__':
    main()
