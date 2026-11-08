# SLAM data-capture and benchmarking scripts

Helpers for the SLAM program described in [`docs/slam/`](../../docs/slam/README.md). They do not run any SLAM algorithm; they make experiments repeatable. All were syntax-checked only and have **not been run on the rover hardware**.

| Script | Purpose | Used by |
|---|---|---|
| [`jetson_perf_setup.sh`](jetson_perf_setup.sh) | Show or set the Jetson power mode and clocks; lists the modes defined on this JetPack | [E0](../../docs/slam/research-plan.md#e0--bring-up-and-calibration) |
| [`record_bag.sh`](record_bag.sh) | Record LiDAR, IMU, GNSS, wheel commands, TF and the existing rover estimates into a zstd-compressed bag, with run metadata | [E1–E3](../../docs/slam/research-plan.md#e1--odometry-bench-off) |
| [`bench_tegrastats.sh`](bench_tegrastats.sh) | Log CPU, GPU, RAM, temperature and power with `tegrastats` while an experiment runs | [E1, E3](../../docs/slam/research-plan.md#e1--odometry-bench-off) |
| [`slam.repos`](slam.repos) | `vcs` import list of candidate external repositories (Unitree SDK, GS-LIVO, nvblox, community L2 driver) for an overlay workspace `slam_ws/` | [code-map](../../docs/slam/code-map.md) |

## Typical session

```bash
scripts/slam/jetson_perf_setup.sh                       # note the power mode in your log
BAG_DIR=~/bags CAMERA_TOPICS="/image_raw" scripts/slam/record_bag.sh siteA_run1
# ... drive the route, Ctrl-C to stop ...

scripts/slam/bench_tegrastats.sh pointlio_siteA_run1 300 &   # logs for 5 minutes
ros2 bag play ~/bags/siteA_run1                              # while the algorithm under test runs
```

Bags, benchmark logs and `slam_ws/` are ignored by git (see the root `.gitignore`).
