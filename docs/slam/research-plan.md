# Research plan

How to turn [architecture.md](architecture.md) into evidence. Each experiment has a question, a method, the metric that decides it, and what happens if it fails. Scripts referenced here are listed in [code-map.md](code-map.md); the ones marked *planned* do not exist yet.

## Objectives

1. **Engineering goal.** The rover navigates autonomously between GNSS waypoints while avoiding obstacles it senses itself (LAYER 1 + 2).
2. **Research goal.** Show whether a LiDAR-seeded surfel-Gaussian terrain map gives traversability that is as accurate as, or better than, an elevation-map baseline while fitting within an 8 GB edge budget (LAYER 3).

The first goal must succeed independently of the second.

## Novelty claim (and how to test it)

Candidate claim **[I]**:

> A LiDAR-seeded surfel-Gaussian map on an 8 GB Jetson that derives traversability from surface normals alone, without a semantic network, and is compared against an elevation-map baseline on rover-style terrain.

Prior art that bounds it (details in [literature.md](literature.md#closest-prior-art-to-the-proposed-contribution-splatblox)):

- **Splatblox** already does Gaussian traversability for outdoor navigation, but with CLIPSeg semantics, on a laptop GPU, with no released code.
- **GS-LIVO** already runs LiDAR-seeded Gaussian mapping on a Jetson (Orin NX 16 GB), but for mapping and odometry, not traversability.

Before committing to the claim, do a proper novelty search for 2025–2026 papers on "Gaussian traversability Jetson", "surfel Gaussian terrain", and "LiDAR Gaussian elevation map". The searches done so far were broad **[?]**.

If the claim fails (someone has done it, or the Gaussian layer does not beat the baseline), the fallback contribution is a **well-measured Orin Nano benchmark** of Layers 1–3 on a real rover, which is also useful and publishable as a systems paper **[I]**.

## Decisions needed

| # | Decision | Needed before | Default if undecided |
|---|---|---|---|
| D1 | ~~Which camera~~ **Resolved:** design to the [sensor contract](architecture.md#sensor-contract-camera-agnostic); any depth camera meeting it is acceptable. Remaining question: its measured depth range, noise and time offset | E0 camera calibration, any photometric work | Treat the camera as optional until its contract values are measured |
| D2 | L2 or L1 | Purchase | L2 |
| D3 | Is Layer 3 a thesis/paper contribution or optional extra? | E3 scope | Optional extra |
| D4 | Nav2 adoption or extend `NAVnode` | E4 | Nav2 for control, `NAVnode` for waypoints |
| D5 | Ground truth: RTK GNSS or total-station/mocap? | E1 | RTK if available, else loop-closure error |
| D6 | Competition task list (which autonomy tasks matter) | Planning priorities | Waypoint navigation with obstacles |

## E0 — Bring-up and calibration

**Question:** can the sensors be read reliably on the Jetson, and are the extrinsics and time offsets known?

Tasks:
1. Flash JetPack 6.2.x; set the power mode; record `nvpmodel -q` output ([`scripts/slam/jetson_perf_setup.sh`](../../scripts/slam/jetson_perf_setup.sh)).
2. Build the Unitree driver for Ubuntu 22.04 / Humble; confirm the topics and message types; measure the real IMU rate and scan rate (the manual could not be read **[?]**).
3. Put the L2 on its own interface/subnet; confirm it does not disturb DDS ([platform.md](platform.md#integration-problems-found-on-paper)).
4. Measure LiDAR mount pose; add the link to the URDF; verify the LiDAR–IMU transform with a rotation-only test.
5. If a camera is used: intrinsics, LiDAR–camera extrinsics, and the time offset.

Exit criterion: a rosbag of 10 minutes of static + driving data replays with sane TF and no dropped scans.

## E1 — Odometry bench-off

**Question:** which LiDAR-inertial odometry runs best on the Orin Nano Super with the Unitree L2?

Candidates: Point-LIO ROS 2 (Unitree port), a FAST-LIO2 ROS 2 port, DLIO, KISS-ICP, GLIM **[S]**.

Method: record several routes with [`scripts/slam/record_bag.sh`](../../scripts/slam/record_bag.sh), including a loop that returns to the start and a stretch with RTK GNSS. Replay each algorithm on identical bags while logging load with [`scripts/slam/bench_tegrastats.sh`](../../scripts/slam/bench_tegrastats.sh). Use the same point-cloud thinning for everyone.

| Metric | How |
|---|---|
| Absolute trajectory error | vs RTK GNSS where available |
| Loop-closure drift | end-point error on returning loops, per metre travelled |
| Vertical drift | z error on flat ground (a reported weakness of FAST-LIO2 on a Unitree-class platform **[S]**) |
| CPU use, RAM, output rate, latency | `tegrastats`, ROS timestamps |
| Robustness | start-up delay, behaviour with open sky vs tall obstacles, behaviour when the arm moves |

**First pipeline check (simulation only, 2026-11-09).** One loop in the simulated yard (`worlds/slam_yard.sdf`, driven by `scripts/slam/sim_drive_loop.py`, recorded with `record_bag.sh`, scored by `scripts/slam/eval_trajectory.py`) was run through KISS-ICP 1.3.0 (LiDAR only, default settings, `pip install --user kiss-icp`). Result on 493 scans: ATE RMSE 0.43 m over a 16.5 m path (max 1.4 m); length with every 10th pose 16.5 m vs 16.4 m true; height drift 0.13 m; 52 Hz and 70 MB on the development PC. The first-pass 1.59 path-length ratio was jitter, not drift: about 3.75 m of the estimated travel happened while the rover was almost still or turning in place. **This proves the record, convert, run and score pipeline works. It says nothing about the Orin Nano or real data**: the simulated cloud has no noise model validated against the L2, no motion distortion and only 16 rings. The other candidates, IMU use and the Jetson timing are still untested. The open-loop drive did not close its loop (2.5 m end gap), so the loop-closure metric needs a closed-loop controller or a longer, repeated route.

**FAST-LIO2 set-up (2026-11-15).** Candidate chosen for the first LiDAR-inertial run: the official FAST-LIO2 ROS 2 branch (HKU-MARS, GPL-2.0, pinned to `cc3c9e8b`). Point-LIO, official and Unitree's port, is ROS 1 only, and DLIO's ROS 2 branch is stale. Its Velodyne input reads exactly the L2 driver's fields (`x y z intensity time ring`). It was cloned and audited: no downloads or shell commands; files are written only when `pcd_save_en`/`runtime_pos_log_enable` are on, and one `map_save` service writes into its own folder. Supporting pieces are in [`scripts/slam/`](../../scripts/slam/README.md#fast-lio2-on-the-e1-simulation-bag): a build script with a message-only `livox_ros_driver2` stub, a simulation config, a node adding the per-point `time` field, a TUM writer, and `--est-clock sim` in the evaluator. Built in WSL2 and run on the same E1 bag (2026-11-15, `scripts/slam/fast_lio/run_bag.sh`):

| Same bag, 16.5 m loop | FAST-LIO2 (LiDAR + IMU) | KISS-ICP (LiDAR only) |
|---|---|---|
| ATE RMSE (max) | **0.076 m** (0.18 m) | 0.426 m (1.43 m) |
| Gap between first and last pose (true gap 2.513 m) | 2.517 m | 3.111 m |
| Path length, every 10th pose (true 16.43 m) | 16.21 m | 16.53 m |
| Height drift | 0.136 m | 0.125 m |
| Poses / scans | 490 / 493 (first scans used for IMU initialisation) | 493 / 493 |
| Load on the development PC (busy) | 69 % of one core mean, 149 MB peak RSS | 52 Hz offline, 70 MB |

FAST-LIO2 is about 5.6× better on ATE, and its end gap matches the true gap to 4 mm. **This is still simulation only**: 16 rings, Gaussian range noise of 0.02 m, an IMU with **zero** noise or bias in `/tmp/bags/yard_raw` (`std = 0`), a static yard, a single 16.5 m loop that ended 2.513 m open, and instantaneous scans. It ranks the pipelines; it does not predict field accuracy or Orin Nano load.

**Follow-up closed-loop noisy-IMU run (`e1b_route`, inspected 2026-11-16):** `sim_drive_route.py` (3 laps, `124.27 m` true path) with Gazebo IMU Gaussian noise/bias (`gyro mean ~[-0.0021, -0.0025, 0.0012], std ~0.0019`, `acc bias ~[0.048, -0.030, 9.850], std ~0.03`) ran at ~0.25x real-time (`1880` scans over `751.60 s` wall clock) and replayed at `RATE=3`. Re-scored with the upgraded SE(3) evaluator (`eval_trajectory.py`, 1875 poses): `ATE RMSE 0.028 m (max 0.093 m)`, `ATE rotation RMSE 0.380 deg (max 1.356 deg)`, `z drift (raw/aligned) 0.109 m / 0.050 m`, vector `loop drift 0.096 m (0.08 %)`, and `SE(3) RPE 10 m: mean 0.028 m (0.28 %), max 0.134 m | rot mean 0.466 deg`. **Remediated tooling (`2026-11-16`):** `run_bag.sh` now signals `$FLPID` on cleanup and fails on empty `fastlio_tum.txt`; `eval_trajectory.py` evaluates SE(3) orientations/RPE, aligned Z drift, and vector loop residuals while failing cleanly on disjoint clocks. Remaining simulation caveat: Gazebo captures each scan at one instant (`lidar_point_time.py` uses `1e-6 s`), so neither `yard_raw` nor `e1b_route` tests real L2 helical rolling-sweep deskewing or open-terrain Z-drift.

Gate: pick the algorithm with the lowest drift that holds ≥ 10 Hz pose output within the Layer 1 budget. If none does, thin the cloud, lower the L2 rotation rate, or move to MAXN_SUPER and re-measure before changing hardware.

## E2 — Fusion with GNSS and wheels

**Question:** does adding LIO to `rover_ekf` improve the global pose over GNSS + IMU + wheel speed alone?

Method: implement the velocity update and the aligned pose update ([architecture.md](architecture.md#fusion--one-global-pose-existing-code-extended)); compare three configurations on the E1 bags: (a) current EKF, (b) + LIO velocity, (c) + LIO pose.

Also settle the yaw-convention and frame-origin questions in [known-issues.md](../known-issues.md#2-open--needs-measurement-or-a-team-decision) (items 4–6) as part of this experiment.

Gate: (c) or (b) must beat (a) on drift and not degrade when GNSS is degraded.

## E3 — Planner map and the Gaussian layer

Two stages, so that Layer 2 ships even if Layer 3 does not.

**E3a — Baseline.** Build the elevation/traversability grid (Layer 2). Label traversable vs non-traversable ground on recorded segments (manual annotation, plus where the rover actually drove). Metrics: precision/recall of the blocked class, map update rate, memory.

**E3b — Gaussian layer.** Install the candidate implementation (GS-LIVO is the only public embedded-class one found; verify it builds against CUDA 12.6 and JetPack 6.2 **[?]**) or write a minimal LiDAR-seeded surfel map.

Scaling study on the Nano: sweep the Gaussian cap (for example 10 k, 20 k, 50 k, 100 k) and image size; record update rate, CPU/GPU memory, and thermal behaviour in 7 W, 15 W, 25 W and MAXN_SUPER modes.

Comparison: traversability from Gaussian normals vs the baseline on the same labelled segments, at equal memory and equal update rate.

| Metric | Why |
|---|---|
| Precision/recall of blocked cells | Safety and progress |
| Surface normal error vs a reference mesh/point cloud | Is the Gaussian surface actually better? |
| Memory (CPU/GPU), update rate, power | The edge claim |
| Failure cases | Overhanging vegetation, thin obstacles, steep edges, glare on the LiDAR |

Gate: continue with Layer 3 only if it matches the baseline within the budget **and** shows a benefit (for example better edge or slope accuracy at the same memory). Otherwise stop and report the negative result.

## E4 — Closed-loop navigation

**Question:** does the rover reach waypoints, avoiding obstacles, using the stack?

Method: waypoints and obstacles on a field course; run with the bridge (`AddLocalObstacle`) first, then Nav2. Metrics: success rate, path length ratio, minimum obstacle clearance, interventions, time. At least 10 runs per condition (Splatblox used 3–10 per scenario, which is too few to claim much **[M]**).

## E5 — Stretch topics (only if E1–E4 succeed)

- Terrain-physics estimate from wheel slip using MPM-style priors (PhysGaussian/GASP/PhysGS **[S]**). Needs wheel encoders or an equivalent speed signal.
- Moving-object rejection and dynamic mapping (4D/deformation literature **[S]**).
- Camera-only fallback (VGGT-style or cuVSLAM) for LiDAR failure.

## Data management

- One bag per run; name `<site>_<yyyymmdd>_<run>`; keep a spreadsheet of site, weather, mount configuration, power mode, git commit.
- Point clouds are large; use zstd compression (the record script does) and expect gigabytes per hour **[I]**.
- Raw bags do not belong in git; `bags/`, `*.db3`, `*.mcap` and `slam_ws/` are in the root `.gitignore`. Keep bags on external storage and record their location in the run log.

## Risks

| Risk | Likelihood **[I]** | Mitigation |
|---|---|---|
| Unitree ROS 2 driver does not build on Humble / JetPack 6 | Medium | Use the C++ SDK directly and write a thin ROS 2 wrapper |
| 8 GB shared memory exhausted | High for Layer 3 | Gaussian cap, CPU-side map, kill switch |
| LIO drifts vertically or fails on open flat terrain | Medium | GNSS + wheel fusion; try LiDAR-only (KISS-ICP) and tighter IMU weighting |
| LiDAR cannot see the ground near the rover (blind zone, mounting, arm shadow) | Medium | Mount geometry study in E0; second pass with a tilted mount |
| Direct sunlight or dust reduces LiDAR returns | Low–medium | > 100 klux rating is encouraging **[P]**; verify outdoors |
| Time offset between LiDAR and camera spoils photometric refinement | High if a camera is used | Measure and store the offset; run Layer 3 geometry-only |
| Novelty already claimed | Medium | Novelty search before committing; fallback to systems benchmark |
| Thermal throttling in the enclosure | Medium | Heat-sink/fan plan; test sustained load for 30 minutes |
| Scope creep from the topic list | High | Priorities in [literature.md](literature.md#summary-table); only E5 touches deferred topics |

## Reading list (in order)

1. Unitree L2 manual (the PDF could not be machine-read; read it by hand) and the `unilidar_sdk2` README.
2. GS-LIVO paper (Sections III–IV and Tables II–IV).
3. Splatblox paper, especially Sections III-B and III-C.
4. 2DGS paper (losses and normal definition).
5. nvblox documentation and the Nav2 + Isaac Perceptor tutorial.
6. The odometry candidates' papers and READMEs: Point-LIO, FAST-LIO2, DLIO, KISS-ICP, GLIM.
7. Continual 3DGS on Orin Nano (arXiv 2603.08499) for what already fits the board.

## Checklist for the first month

- [ ] Camera model confirmed (D1)
- [ ] L2 ordered and a mount designed
- [ ] JetPack 6.2.x flashed; performance script run and logged
- [ ] Unitree driver builds on Humble; message types and rates recorded
- [ ] First 10-minute bag recorded and replayable
- [ ] Odometry candidates installed and running on that bag
- [ ] Novelty search written up in this folder
