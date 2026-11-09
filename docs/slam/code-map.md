# Code map

Which file does what in the SLAM program. **Exists** means it is in the repository now; **Planned** means it is proposed in [architecture.md](architecture.md) and has no file yet. Planned items are in plain text on purpose, so no link in this page points at something that does not exist.

## Existing code the SLAM work builds on

| Concern | File | What to use or change |
|---|---|---|
| Pose filter (GNSS, IMU, wheel speed) | [`rover_ekf/ekf_node.py`](../../ros2_ws/src/rover_ekf/rover_ekf/ekf_node.py) | Add LIO velocity and pose updates ([architecture](architecture.md#fusion--one-global-pose-existing-code-extended)). Maths: [state-estimation.md](../state-estimation.md) |
| EKF launch/parameters | [`rover_ekf/launch/ekf.launch.py`](../../ros2_ws/src/rover_ekf/launch/ekf.launch.py) | Add topic parameters for the new inputs |
| IMU reading and tare | [`IMU/imu/telemetry.py`](../../ros2_ws/src/IMU/imu/telemetry.py) | The rover's own IMU; the L2 has its own |
| IMU-only filter | [`IMU/imu/kalman_filter.py`](../../ros2_ws/src/IMU/imu/kalman_filter.py) | Reference only; not needed once LIO exists |
| GNSS → `NavSatFix` | [`navigation_system/src/gps_bridge_node.py`](../../ros2_ws/src/navigation_system/src/gps_bridge_node.py) | Global anchor for the `map` frame |
| Waypoints, route editing, bearing | [`navigation_system/src/NAVnode.cpp`](../../ros2_ws/src/navigation_system/src/NAVnode.cpp) | Waypoint source; cannot drive the rover on its own ([navigation.md](../navigation.md)) |
| Obstacle input (bridge) | [`navigation_interfaces/srv/AddLocalObstacle.srv`](../../ros2_ws/src/navigation_interfaces/srv/AddLocalObstacle.srv) | Day-one way to feed sensed obstacles to `NAVnode` |
| Local ↔ geodetic maths | [`navigation_system/src/Waypoint.cpp`](../../ros2_ws/src/navigation_system/src/Waypoint.cpp), test [`scripts/tests/waypoint_conversion_test.cpp`](../../scripts/tests/waypoint_conversion_test.cpp) | Frame alignment must reuse these conversions |
| Wheel command output | [`drive_package/src/drive_node.cpp`](../../ros2_ws/src/drive_package/src/drive_node.cpp) | Accepts `/cmd_vel` from a planner ([drive-system.md](../drive-system.md)) |
| Motor driver | [`drive_package/src/vesc_can_driver_node.cpp`](../../ros2_ws/src/drive_package/src/vesc_can_driver_node.cpp) | No encoder feedback is published; see [known-issues](../known-issues.md) |
| Robot model, dimensions, simulation | [`rover_description/`](../../ros2_ws/src/rover_description/README.md), [`config/physical_params.yaml`](../../ros2_ws/src/rover_description/config/physical_params.yaml), [`urdf/`](../../ros2_ws/src/rover_description/urdf/), [`launch/sim.launch.py`](../../ros2_ws/src/rover_description/launch/sim.launch.py) | Add the LiDAR link and a simulated LiDAR |
| Process manager (start/stop nodes from the browser) | [`arm_controller/dashboard_helper_node.py`](../../ros2_ws/src/arm_controller/arm_controller/dashboard_helper_node.py) | Add an entry per SLAM node |
| Dashboard | [`dashboard/app.js`](../../dashboard/app.js), [`dashboard/index.html`](../../dashboard/index.html) | Show map/traversability overlays and SLAM health |
| DDS/network settings | [`fastdds_profiles.xml`](../../fastdds_profiles.xml) | Keep the LiDAR off these peers' subnet ([platform.md](platform.md#integration-problems-found-on-paper)) |
| Launch scripts | [`start_dashboard_system.sh`](../../start_dashboard_system.sh), [`start_sim_system.sh`](../../start_sim_system.sh) | Entry points; do not add a third |
| Arm geometry that may shadow the LiDAR | [`arm_controller/kinematics.py`](../../ros2_ws/src/arm_controller/arm_controller/kinematics.py), [arm doc](../arm-and-kinematics.md) | Body mask for Layer 1 |

## New files in this change

| File | Purpose |
|---|---|
| [`docs/slam/`](README.md) | This documentation |
| [`scripts/slam/record_bag.sh`](../../scripts/slam/record_bag.sh) | Dataset recording |
| [`scripts/slam/bench_tegrastats.sh`](../../scripts/slam/bench_tegrastats.sh) | Load logging during experiments |
| [`scripts/slam/jetson_perf_setup.sh`](../../scripts/slam/jetson_perf_setup.sh) | Power mode / clocks inspection |
| [`scripts/slam/slam.repos`](../../scripts/slam/slam.repos) | Candidate external repositories |
| [`scripts/slam/README.md`](../../scripts/slam/README.md) | How the scripts fit together |

Environment, tool choices and the research-tier/rover-tier split are recorded in [decisions.md](decisions.md); that page also lists the planned launch profiles (`competition`, `research`).

## Planned code (does not exist yet)

Names are proposals. Create the package only when its first node exists, so that `colcon build` in `ros2_ws` keeps working.

| Planned path | Purpose | Layer / experiment |
|---|---|---|
| `ros2_ws/src/rover_slam/config/extrinsics.yaml` | Measured LiDAR and camera poses relative to `base_link`, and time offsets | E0 |
| `ros2_ws/src/rover_slam/launch/slam.launch.py` | Starts driver, odometry, map layers; launched from the process manager | Layer 1–3 |
| `ros2_ws/src/rover_slam/rover_slam/alignment_node.py` | Estimates the LIO-to-GNSS alignment; publishes `map → odom` | Fusion, E2 |
| `ros2_ws/src/rover_slam/rover_slam/traversability_node.py` | Elevation/traversability grid (Layer 2 baseline) | E3a |
| `ros2_ws/src/rover_slam/rover_slam/obstacle_bridge.py` | Blocked regions → `AddLocalObstacle` calls | E4 |
| `ros2_ws/src/rover_slam/rover_slam/safety_stop.py` | Corridor point-count stop, independent of all other layers | E4 |
| `ros2_ws/src/gaussian_terrain/` | LiDAR-seeded surfel Gaussian map and traversability (Layer 3) | E3b |
| `ros2_ws/src/rover_description/urdf/lidar.xacro` | LiDAR link and a simulated sensor | E0 |
| `scripts/slam/eval_traj.py` | Trajectory error against RTK GNSS or loop closure | E1, E2 |
| `scripts/slam/label_traversability.py` | Helper to label traversable ground on recorded segments | E3 |
| `docs/slam/results/` | One page per experiment with data, plots and the decision taken | all |

## Documentation conventions for new code

1. Every new node gets a section here (file → role → topics) and in [architecture.md](../architecture.md) when its topics join the system.
2. Every number that drives a decision is recorded in the matching experiment page under `docs/slam/results/` with the bag name, git commit and power mode.
3. Change [known-issues.md](../known-issues.md) when an item is fixed or a new limitation is found.
4. Keep the evidence labels (`[P]`, `[S]`, `[M]`, `[I]`, `[?]`) in research pages.
