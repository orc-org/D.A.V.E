# Proposed architecture

This is a design, not an implementation. Nothing described under "planned" exists in the repository yet; [code-map.md](code-map.md) separates what exists from what is proposed. Labels follow [README.md](README.md#evidence-labels); all numbers in this page marked **[I]** are targets to be validated on the board, not measurements.

## Principle: separate tracking from mapping

Gaussian-splatting SLAM papers often use the Gaussian map to track the camera. On an 8 GB shared-memory board that couples the rover's pose to the most expensive component. The design below keeps pose, planning map and Gaussian map independent, so each can fail or be switched off without taking down the others.

```
                         ┌───────────────────────────────────────────────────────────┐
 Unitree L2 ─ cloud+IMU ─►  LAYER 1  LiDAR-inertial odometry (CPU)                    │
 camera ───── image ─────►           deskewed cloud, pose, velocity                   │
                         └───────┬───────────────────────────────┬───────────────────┘
                                 │ pose/twist                    │ deskewed cloud + pose
                                 ▼                               ▼
  GNSS ──► rover_ekf (extended) ──► map→odom→base_link     LAYER 2  Planner map (baseline)
  wheel speed ─┘                          │                 elevation grid and/or nvblox ESDF
                                          │                          │
                                          │                          ▼
                                          │                 traversability → Nav2 costmap
                                          │                          ▲
                                          │                 LAYER 3  Gaussian terrain map (research, optional)
                                          ▼                          surfel Gaussians seeded by LiDAR,
                                  NAVnode / Nav2 / drive_node        normals → slope/roughness → traversability
```

Each layer publishes standard ROS 2 messages and has a **kill switch**: if a layer exceeds its budget the layer below remains valid.

## Sensor contract (camera-agnostic)

The code is written against these canonical topics. A launch file remaps whatever a specific driver publishes onto them, so changing the LiDAR or camera changes one remap, not the algorithms.

| Canonical topic | Type | Required? | Notes |
|---|---|---|---|
| `/lidar/points` | `sensor_msgs/PointCloud2` | yes | Per-point timestamps preferred for deskewing; frame `lidar_link` |
| `/lidar/imu` | `sensor_msgs/Imu` | yes (LiDAR odometry) | Unitree: `unilidar/imu`, frame `unilidar_imu`; canonical frame `lidar_imu_link` |
| `/camera/color/image` | `sensor_msgs/Image` | optional | Used by the Gaussian layer and for classification |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | with colour | Intrinsics and distortion |
| `/camera/depth/image` | `sensor_msgs/Image` (32FC1 metres or 16UC1 mm) | optional | Aligned to colour if possible; otherwise a separate `camera_info` |
| `/camera/depth/camera_info` | `sensor_msgs/CameraInfo` | with depth | |

Hard requirements for any camera **[I]**:

1. Stamped images on the same clock as the LiDAR (one machine clock via `chrony`/PTP, or a measured fixed offset stored in `extrinsics.yaml`).
2. A calibrated pose `base_link → camera_link` and intrinsics in `CameraInfo`.
3. Known depth range and noise, set in the camera's config file so the mapper can weight it (a 4–5 m infrared camera and a 10 m stereo camera must not be trusted equally).

Properties that *vary* by camera and are handled by configuration, not code: shutter type (rolling shutter degrades photometric refinement), depth technology (infrared depth degrades in sunlight), resolution and rate (the Gaussian layer downscales to a fixed working size), and whether the camera has an IMU (not needed; the LiDAR IMU is used).

All consumers must also run with the camera absent: Layer 1 needs only the LiDAR, Layer 2 works from the LiDAR alone, and Layer 3 falls back to geometry-only.

## Layer 1 — LiDAR-inertial odometry

- **Input:** `unilidar/cloud` and `unilidar/imu` from the driver ([platform.md](platform.md#software-support)). The Unitree driver names its frames `unilidar_lidar` and `unilidar_imu` ([platform.md](platform.md#software-support)); the launch file maps them to the canonical `lidar_link` and `lidar_imu_link` (with the SDK's fixed IMU offset) so other LiDARs can be swapped in.
- **Candidates** (decided by the bench-off in [research-plan.md](research-plan.md#e1--odometry-bench-off)): Point-LIO ROS 2 port with Unitree support; FAST-LIO2 ROS 2 ports; DLIO; KISS-ICP (LiDAR only); GLIM **[S]**. All run on the CPU; the GPU stays free for Layer 3.
- **Output:** `nav_msgs/Odometry` with covariance; the deskewed cloud in the sensor frame; **no TF**. The LIO node must not publish `odom → base_link`; `rover_ekf` owns that transform (see [Frames](#frames-and-calibration)).
- **Dynamic points:** the arm, the mast and people must not enter the map. Mask a configurable body-shaped region in the sensor frame (the arm's reach comes from `physical_params.yaml`) and reject points with large temporal inconsistency **[I]**.
- **Why LiDAR first:** the L2 has an IMU and a 30 m range; it does not need a synchronised stereo pair; and non-repetitive scanning gives denser coverage as the rover moves **[P]/[I]**.

## Fusion — one global pose (existing code, extended)

The repository already has a pose filter, [`rover_ekf`](../../ros2_ws/src/rover_ekf/rover_ekf/ekf_node.py) (5-state: X, Y, θ, v, ω; see [state-estimation.md](../state-estimation.md)). Proposed extensions **[I]**:

1. Add a **velocity update** from LIO twist: forward speed → `v`, yaw rate → `ω`. This is frame-independent, so it avoids the heading-convention question in [known-issues.md](../known-issues.md#2-open--needs-hardware-verification-measurement-or-autonomy-development) until that is settled.
2. Add a **pose update** from LIO after aligning the LIO frame to the GNSS frame at start-up (a one-time yaw/offset estimate while moving).
3. Keep GNSS as the global anchor (`map` frame) and publish `map → odom` from a small alignment node so LIO drift does not accumulate relative to GNSS.
4. If the 2-D state proves too limited on slopes, graduate to a factor-graph smoother (for example GTSAM) as a second step; do not start there.

Remove the `world → base_link` transform from `motion_coordinator` when running with this stack (it conflicts with the EKF's `odom → base_link`).

## Layer 2 — planner map (baseline)

Goal: let the rover drive autonomously with **no research risk**.

| Option | What | Notes |
|---|---|---|
| A. nvblox | GPU TSDF/ESDF from the LiDAR cloud, Nav2 costmap layer | Official support for LiDAR; 8 GB Nano is the safer target **[S]**. Needs the Isaac ROS container/toolchain on JetPack 6.x |
| B. 2.5-D elevation grid | Height, slope, roughness and step per cell on a rolling grid | Better fit for rough, open terrain than a full 3-D voxel map. Candidate packages (`grid_map`, a GPU elevation mapper) are from background knowledge and not verified **[I]** |

Recommendation: implement B as the reference baseline (cheap, transparent, and it is the comparison point for Layer 3), and evaluate A for its Nav2 integration.

**Depth camera in Layer 2.** When present, depth images are integrated alongside the LiDAR (nvblox accepts depth cameras and 3-D LiDAR at once **[S]**; the elevation grid takes a second point source) to fill the near field that the LiDAR misses. Each source carries its own noise model and maximum range.

Output (planned): `/terrain/traversability` as a `nav_msgs/OccupancyGrid` (0–100 cost) in the `odom` or `map` frame.

### Closing the loop to the wheels

`NAVnode` today computes range and bearing but never produces a velocity command ([navigation.md](../navigation.md), [known-issues.md](../known-issues.md#3-incomplete-by-design)). Two ways to use the new maps:

- **Bridge (day one):** a small node reads the traversability grid, finds blocked regions near the route, and calls the existing obstacle services ([`AddLocalObstacle`](../../ros2_ws/src/navigation_interfaces/srv/AddLocalObstacle.srv)) with circles. Cheap, uses everything that exists, but the circle abstraction throws away shape.
- **Nav2 (proper):** run Nav2 with a costmap layer from Layer 2 and a controller that publishes `/cmd_vel`. `drive_node` already accepts `/cmd_vel` ([drive-system.md](../drive-system.md)). `NAVnode` becomes the waypoint source. This is the target architecture **[I]**.

A last-resort **safety stop** should be independent of both: a few lines of code counting LiDAR points in a corridor ahead of the rover and zeroing the wheels (`drive_node` already stops when `/control_mode` is not `"drive"`; no node publishes it today).

## Layer 3 — Gaussian terrain map (research)

Purpose: a compact, continuous terrain surface with normals, seeded from LiDAR, that yields traversability without a heavy semantic network.

**Representation.** Surfel Gaussians (2DGS-style flat disks): mean, normal, two tangent scales, opacity, optional colour. The normal is available directly from the LiDAR local surface and is refined by optimisation.

**Seeding** (as in GS-LIVO **[M]**): voxel-hash the deskewed cloud; for each occupied voxel create or update one Gaussian whose normal comes from a PCA of the voxel's points and whose in-plane scale follows the voxel size.

**Refinement.** Optional photometric refinement from the camera, restricted to a sliding window of Gaussians in view; skip entirely when no camera is available.

**Traversability features** from each Gaussian, binned onto the same 2.5-D grid as the baseline **[I]**:

| Feature | From | Meaning |
|---|---|---|
| Slope | angle between the normal and vertical, `arccos(n_z)` | Climbable? |
| Roughness | spread of normals and height residuals within a cell | Rocks, ruts |
| Step | height difference to neighbouring cells | Curbs, ledges |
| Confidence | number and opacity of Gaussians per cell | Unknown ≠ free |

These combine into a cost in [0, 1] and are published as the same `OccupancyGrid`, so Layer 3 is a drop-in replacement for the baseline.

**Hard limits** (proposed, **[I]**): a cap on active Gaussians chosen from the memory budget below; the full map lives in CPU memory and only the working set is on the GPU (GS-LIVO's design); the layer is disabled if it breaches its latency budget.

**What is deliberately left out of v1:** CLIPSeg or any semantic network (Splatblox spends 479 ms per frame on it **[M]**), VGGT, 4D/dynamic Gaussians, MPM. See [literature.md](literature.md).

## Frames and calibration

Planned transform tree:

```
map ──(GNSS alignment)──► odom ──(rover_ekf)──► base_link ──► lidar_link
                                                    │            └─► lidar_imu_link (fixed: −0.007698, −0.014655, 0.00667 m)
                                                    └──► camera_link ──► camera_optical_frame
```

- `base_link → lidar_link` and `base_link → camera_link` are **measured extrinsics**, stored in a config file (planned: `ros2_ws/src/rover_slam/config/extrinsics.yaml`) and published by `robot_state_publisher`/static transforms. `rover_description/urdf/sensors.xacro` now defines `lidar_link`, `lidar_imu_link` (currently `[0, 0, -0.02]` m placeholder offset in sim rather than the real L2 `[−0.007698, −0.014655, 0.00667]` m), `camera_link`, and `camera_optical_frame`.
- LiDAR–camera calibration (extrinsic rotation/translation) and **time offset** are required before any photometric work. No PPS or PTP was found for either Unitree sensor **[?]**, so measure the offset with a flashing-target or motion-correlation method and store it.
- LiDAR–IMU: use the SDK transform, then verify with a rotation-only test.

## Topics (planned)

| Topic | Type | Producer | Consumers |
|---|---|---|---|
| `unilidar/cloud` → `/lidar/points` | `sensor_msgs/PointCloud2` (assumed) | Unitree driver (remapped) | Layer 1 |
| `unilidar/imu` → `/lidar/imu` | `sensor_msgs/Imu` (assumed) | Unitree driver (remapped) | Layer 1 |
| `/camera/color/image`, `/camera/depth/image` and their `camera_info` | `Image`, `CameraInfo` | camera driver (remapped) | Layers 2, 3 (optional) |
| `/slam/odometry` | `nav_msgs/Odometry` | Layer 1 | `rover_ekf`, dashboard |
| `/slam/cloud_deskewed` | `sensor_msgs/PointCloud2` | Layer 1 | Layers 2, 3 |
| `/ekf/odometry` | `nav_msgs/Odometry` | `rover_ekf` (exists) | planners, dashboard |
| `/terrain/traversability` | `nav_msgs/OccupancyGrid` | Layer 2 or 3 | Nav2 / obstacle bridge |
| `/terrain/elevation` | grid message (TBD) | Layer 2 | Layer 3 evaluation, dashboard |
| `/gaussian_map/stats` | custom or `std_msgs/String` | Layer 3 | benchmarking, dashboard |

Existing topics and services these connect to are in [architecture.md](../architecture.md).

## Compute and memory budget (targets, not measurements)

All figures **[I]**. They exist so that experiments have a pass/fail line; replace them with measured numbers after [E1–E3](research-plan.md).

| Component | CPU | RAM | GPU memory | Rate |
|---|---|---|---|---|
| OS, ROS 2 graph, rosbridge, dashboard helper | ~0.5 core | ~1.5 GB | – | – |
| Unitree driver | < 0.3 core | < 0.2 GB | – | 5.55 Hz scans (L2 default), IMU stream |
| Layer 1 odometry | ≤ 1.5 cores | ≤ 1.0 GB | – | ≥ 10 Hz pose |
| `rover_ekf` + alignment | < 0.2 core | small | – | 20 Hz |
| Layer 2 baseline | ≤ 1.0 core | ≤ 0.8 GB | ≤ 0.5 GB if nvblox | ≥ 2 Hz map update |
| Layer 3 Gaussians (optional) | ≤ 1.0 core | ≤ 1.5 GB (CPU-side map) | ≤ 1.5 GB working set | ≥ 2 Hz update |
| Headroom | | ≥ 1.5 GB | | |

Reference points: GS-LIVO on a 16 GB Orin NX reached about 3 Hz outdoors; Splatblox used 4 GB VRAM for 100 000 Gaussians on a laptop GPU **[M]**. Both suggest the Layer 3 budget above will require a Gaussian cap in the tens of thousands.

## Operating modes and degradation

| Condition | Behaviour |
|---|---|
| Normal | Layers 1–2 always on; Layer 3 on when its budget is met |
| LiDAR cloud missing (cold start, cable) | `rover_ekf` runs on IMU + wheel + GNSS; planner map freezes; speed limited |
| GNSS lost | LIO keeps `odom` consistent; `map → odom` frozen; global waypoints unavailable until reacquired |
| LIO diverges (covariance or jump test) | Reinitialise; fall back to GNSS + wheel; raise an operator alert via `/process_manager/status` |
| Layer 3 over budget | Stop Layer 3; the baseline keeps feeding the costmap |
| Thermal throttling | Drop to a lower-rate configuration; log the event |

## Where this plugs into the dashboard and launch system

- The dashboard process manager ([`dashboard_helper_node.py`](../../ros2_ws/src/arm_controller/arm_controller/dashboard_helper_node.py)) lists every managed node in one dictionary; new SLAM nodes get an entry there so they can be started from the browser.
- `start_dashboard_system.sh` and `start_sim_system.sh` stay the entry points; a SLAM launch file (planned) would be started through the process manager rather than adding a third script.
- The simulation ([rover_description](../../ros2_ws/src/rover_description/README.md)) now includes Gazebo LiDAR, IMU and RGB-D plugins (`urdf/sensors.xacro`), though GPS, `/wheel_states`, `/imu/euler`, the arm URDF, and rolling scan-sweep timing remain unmodeled (see [`../system-handover.md`](../system-handover.md)).
