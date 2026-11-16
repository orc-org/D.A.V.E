# SLAM data-capture and benchmarking scripts

Helpers for the SLAM program described in [`docs/slam/`](../../docs/slam/README.md). They do not run any SLAM algorithm; they make experiments repeatable. The shell scripts were syntax-checked only and none has been run on the rover hardware; `record_bag.sh`, `sim_drive_loop.py` and `eval_trajectory.py` were run against the Gazebo simulation in WSL2.

| Script | Purpose | Used by |
|---|---|---|
| [`jetson_perf_setup.sh`](jetson_perf_setup.sh) | Show or set the Jetson power mode and clocks; lists the modes defined on this JetPack | [E0](../../docs/slam/research-plan.md#e0--bring-up-and-calibration) |
| [`record_bag.sh`](record_bag.sh) | Record LiDAR, IMU, GNSS, wheel commands, TF and the existing rover estimates into a zstd-compressed bag, with run metadata | [E1–E3](../../docs/slam/research-plan.md#e1--odometry-bench-off) |
| [`bench_tegrastats.sh`](bench_tegrastats.sh) | Log CPU, GPU, RAM, temperature and power with `tegrastats` while an experiment runs | [E1, E3](../../docs/slam/research-plan.md#e1--odometry-bench-off) |
| [`sim_drive_loop.py`](sim_drive_loop.py) | Drive the simulated rover around a rectangle on simulation time (needs the sim running) | E1 pipeline check |
| [`eval_trajectory.py`](eval_trajectory.py) | Score a TUM-format trajectory against the simulator ground truth in a bag (ATE, path length, end error, z drift). `--est-clock sim` for estimates stamped in simulation time (mapped through the bag's `/clock`) | E1 |
| [`lidar_point_time.py`](lidar_point_time.py) | Add the per-point `time` field LIO needs to the simulated cloud (`/lidar/points` → `/lidar/points_timed`); all points get the same time because Gazebo captures each scan at one instant | E1 (simulation only) |
| [`odom_to_tum.py`](odom_to_tum.py) | Write an odometry topic (default FAST-LIO's `/Odometry`) to a TUM file for `eval_trajectory.py` | E1 |
| [`fast_lio/setup.sh`](fast_lio/setup.sh) | Clone FAST-LIO2 (ROS2 branch, pinned commit) into `~/slam_ws`, add the message-only `livox_ros_driver2` stub, build at low priority | E1 |
| [`fast_lio/sim_l2.yaml`](fast_lio/sim_l2.yaml) | FAST-LIO2 parameters for the simulated L2-like sensor (notes for the real L2 at the end) | E1 |
| [`slam.repos`](slam.repos) | `vcs` import list of candidate external repositories (FAST-LIO2 pinned, Unitree SDK, GS-LIVO, nvblox, community L2 driver) for an overlay workspace `slam_ws/` | [code-map](../../docs/slam/code-map.md) |

## FAST-LIO2 on the E1 simulation bag

```bash
scripts/slam/fast_lio/setup.sh                 # once: clone (pinned), stub, build (~20 min at low priority)
scripts/slam/fast_lio/run_bag.sh BAG /tmp/run1  # BAG = an uncompressed simulation bag
```

`run_bag.sh` starts `lidar_point_time.py`, `odom_to_tum.py` and `fastlio_mapping` (with `sim_l2.yaml`), plays the bag in real time with `--clock`, samples FAST-LIO's CPU and memory, and scores the result with `eval_trajectory.py --est-clock sim`.

Status 2026-11-15: built in WSL2 (19 min, one package at a time) and run on the E1 bag; results in [research-plan.md](../../docs/slam/research-plan.md#e1--odometry-bench-off). `lidar_point_time.py` was checked on the bag, and `--est-clock sim` by scoring the KISS-ICP trajectory converted to simulation time (same ATE, 0.426 m).

## Typical session

```bash
scripts/slam/jetson_perf_setup.sh                       # note the power mode in your log
BAG_DIR=~/bags CAMERA_TOPICS="/image_raw" scripts/slam/record_bag.sh siteA_run1
# ... drive the route, Ctrl-C to stop ...

scripts/slam/bench_tegrastats.sh pointlio_siteA_run1 300 &   # logs for 5 minutes
ros2 bag play ~/bags/siteA_run1                              # while the algorithm under test runs
```

Bags, benchmark logs and `slam_ws/` are ignored by git (see the root `.gitignore`).
