# Project and code review (2026-11-15)

A full pass over the ROS 2 workspace, scripts and dashboard, judged against the CIRC 2027 tasks ([competition.md](competition.md)) and the SLAM plan ([slam/README.md](slam/README.md)). Everything here is local and not pushed.

**Verdict in one paragraph.** Teleoperation (drive, cameras, dashboard, arm by hand) is in workable shape. Nothing in the repository can yet drive the rover by itself: there is no closed-loop controller, the navigation node computes range and bearing but publishes no motion, the position estimate is GNSS-only for navigation, and the two estimators (`rover_ekf`, the IMU AUKF) disagree on frames and origins. The Demolition task (autonomous) and the Meteorite task (no GNSS) therefore need the SLAM work, a controller and a few safety features before they are possible. The NAVnode bugs that made it unusable with a real receiver are fixed and covered by a test.

## Fixed in this review

Details are in [known-issues.md §1](known-issues.md#1-fixed-while-consolidating-local-changes-not-pushed). Summary:

| Component | Fix | Verified by |
|---|---|---|
| `NAVnode` NMEA parsing | RMC/GGA/VTG split into fields; RMC no longer discards the buffer; speed (knots → km/h), course and date are read | `scripts/tests/navnode_nmea_smoke_test.sh` (11/11 pass): speed 18.52 km/h from 10 kn |
| `NAVnode` serial | Bytes kept across 200 ms ticks; speed/heading held between sentences | same test (speed was 0 before) |
| `NAVnode` targeting | A waypoint added after start-up becomes the target; removed/cleared targets are dropped; end of route is remembered | same test: range 10.2 m to a waypoint 10 m east of home (was 10¹² m) |
| `NAVnode` track log | Stores copies (was a dangling pointer and a double free); logs each point | built; shutdown path not exercised by the test |
| `NAVnode` services | `removeLastWaypoint` checks the route it edits; route-select success message is set | code read |
| `rover_ekf` | Yaw-only IMU update (the 3-row update compared roll/pitch with x/y); `use_gps:=false` for GNSS-denied runs | unit run: yaw converges to 90.0°, P stays positive definite; with `use_gps:=false` there is no `/gps/fix` subscription |

## Open findings by severity

**Critical** = can hurt the rover or people, or makes a task impossible. **Major** = wrong results. **Minor** = cleanup.

### Safety and drive

| Sev. | Finding | Where | Task impact |
|---|---|---|---|
| ~~Critical~~ Fixed | **No command timeout (deadman).** `vesc_can_driver_node` held the last target forever, so a dropped Wi-Fi link or a crashed dashboard left the rover driving. It now stops any wheel whose command is older than `command_timeout` (0.5 s), immediately and without ramping ([drive-system.md](drive-system.md)). Tested in WSL without a CAN bus: no stop while commands arrive at 10 Hz, and a stop after the publisher ends. | `drive_package/src/vesc_can_driver_node.cpp` | All tasks; required before any autonomy |
| Critical | **`motion_coordinator` publishes `cmd_vel` in auto mode**, so the arm node can drive the rover. Its base pose is commands × 3.0, not a measurement. | `arm_controller/.../motion_coordinator_node.py` | All tasks; do not run auto mode on the real rover |
| Major | `/cmd_vel` is normalised (−1…1) in this repo, but metric mode and the EKF treat it as m/s. Wheel radius 0.10 m (driver) vs 0.15 m (URDF). | `vesc_can_driver_node.cpp`, `rover_ekf` | Odometry and any controller tuning |
| Major | No VESC status readback, so no wheel odometry. VESC CAN status frames carry ERPM; read them. | `vesc_can_driver_node.cpp` | SLAM (wheel prior), Meteorite |
| ~~Minor~~ Fixed | Both drive nodes logged at INFO on every message (20 Hz from `/joy`). Now DEBUG or throttled to 1 Hz. | `drive_node.cpp`, `vesc_can_driver_node.cpp` | Log noise on the Jetson |

### State estimation

| Sev. | Finding | Where | Task impact |
|---|---|---|---|
| Major | **IMU AUKF double-integrates acceleration into position**, subtracts gravity in the body frame and never corrects orientation (gyro only). Its position and odometry outputs drift without bound; use only its orientation, or replace it with the BNO085's own fusion. | `IMU/imu/kalman_filter.py` | Meteorite (dead reckoning) |
| ~~Major~~ Fixed | **`telemetry.py` silently published synthetic data on `/imu/data_raw` when the hardware failed**, which looked like a working, level, stationary IMU. Synthetic data now needs `sim_mode:=true`; otherwise a missing library or sensor logs an error, publishes nothing and retries every 2 s, and a sensor that dies at run time stops publishing after about 1 s of failed reads ([state-estimation.md](state-estimation.md)). Tested in WSL (no libraries: 0 messages; `sim_mode:=true`: 11 Hz received; injected read failures: 19 repeats, then stop). | `IMU/imu/telemetry.py` | All autonomy |
| Major | `rover_ekf` uses a fixed `dt` and wheel commands as measured speed (the GPS covariance is now taken from the message when present). Its origin is the first fix; NAVnode's is the home waypoint. | `rover_ekf/ekf_node.py` | Demolition, Reactor (geofence in the wrong frame) |
| Major | Yaw conventions conflict: the EKF uses counter-clockwise from east (ENU), the IMU tare sets 0 = north, and CIRC bearings are from **true** north (magnetic declination is not applied anywhere). | EKF, `telemetry.py`, NAVnode | Bearings in Mining/Meteorite reports |
| Major | Two TF parents for `base_link` (`world` from `motion_coordinator`, `odom` from `rover_ekf`). | both | Breaks RViz/Nav2 when run together |
| ~~Major~~ Fixed | `gps_bridge_node` reported every fix as `STATUS_FIX` with no covariance, published GGA quality 6 (the receiver's dead-reckoning estimate) as a real fix, published each epoch twice (GGA, then RMC with altitude 0), and published nothing without a fix. Now: status from the GGA quality (FIX / SBAS / GBAS for RTK), an HDOP × UERE covariance, explicit `STATUS_NO_FIX` for quality 0 and 6, RMC only as a fallback; `rover_ekf` uses that covariance instead of its fixed 1.5 m², and the dashboard keeps the last good position on no fix ([navigation.md](navigation.md#gps_bridge_node-fix-status-and-covariance)). Tested in WSL with checksummed sentences; the UERE values are typical figures, not measured for this receiver. | `gps_bridge_node.py`, `ekf_node.py`, `dashboard/app.js` | EKF trusted invalid fixes |

### Navigation

| Sev. | Finding | Where | Task impact |
|---|---|---|---|
| Critical | **No path follower.** Nothing turns range/bearing into `cmd_vel`; the `FollowRoute` action accepts any goal, does nothing, and races the timer on `currentPosition`. | `NAVnode.cpp` | Demolition (autonomous), any waypoint task |
| Major | `addLocal*` silently sets home to the current position if home is unset. | `NAVnode.cpp` | Wrong frame without warning |
| Major | The ResetHome password is a constant in the source. Fine for a team tool, not a security control. | `Waypoint.hpp` | — |
| Major | `SerialPort::read()` is a raw read (≤ 256 bytes). NAVnode now buffers; other users of `SerialPort` must too. | `serial_communication` | — |
| Major | No polygon geofence (Reactor Recovery storm sectors); obstacles are only circles typed in by hand. | — | Reactor Recovery |
| Minor | Comment says queue depth 1, code uses 10; track log path relative to the working directory. | `NAVnode.cpp` | — |

### Dashboard and network

| Sev. | Finding | Where | Task impact |
|---|---|---|---|
| Major | **Needs the internet**: Google Fonts, jsdelivr (roslib, three, ros3d), unpkg (Leaflet) and online map tiles. A field site without uplink gets a blank dashboard. Vendor the scripts into the repo and pre-cache tiles. | `dashboard/index.html` | All tasks |
| Major | rosbridge is unauthenticated, and `dashboard_helper` starts and kills processes through unauthenticated services. Keep the rover network closed. | `dashboard/`, `dashboard_helper` | — |
| Major | No judge-click target UI (Demolition: judges pick targets on a map). | — | Demolition |

### Arm

| Sev. | Finding | Where | Task impact |
|---|---|---|---|
| Major | One actuated joint; the IK is a coordinate-wise numeric search; `move_to_point` returns success immediately; the default target (25, 0, 2) mixes units. | `arm_controller` | Mining sample handling, Reactor Recovery |

### Missing capabilities for CIRC 2027

| Need | Task | Notes |
|---|---|---|
| Closed-loop controller (pure pursuit on `NAVnode` targets, or Nav2) | Demolition, all waypoint driving | Needs a pose at ≥ 10 Hz: LIO output fused into the EKF |
| ArUco detector (`DICT_4X4_100`) with range/bearing | Demolition, Meteorite | `cv2.aruco` on the existing camera streams |
| GNSS-free localisation | Meteorite | LIO + wheel + IMU; `rover_ekf use_gps:=false` is the switch |
| Emitter kill switch (lights, IR projectors of a depth camera, LiDAR?) | Prison Break high-security area | Check whether the chosen depth camera's IR projector counts as emitted light; the L2 LiDAR emits 905 nm |
| Polygon geofence | Reactor Recovery | Point-in-polygon on the local frame, enforced in the controller |
| True-north bearings | Mining, Meteorite reports | Apply declination to IMU yaw |
| Offline dashboard | All | See above |
| Mass budget (50 kg target) | All | Track LiDAR + camera + mounts |

## SLAM status

| Item | State |
|---|---|
| Sensor contract (`/lidar/points`, `/lidar/imu`, `/camera/*`) | Defined ([slam/architecture.md](slam/architecture.md)); simulated LiDAR, IMU and RGB-D publish in Gazebo Fortress |
| Pipeline (record → convert → run → score) | Works in simulation: KISS-ICP ATE 0.43 m on a 16.5 m loop ([slam/research-plan.md](slam/research-plan.md#e1--odometry-bench-off)) |
| LiDAR-inertial candidates (Point-LIO, FAST-LIO2, DLIO) | Not built: cloning/building external code needs the user's permission. DLIO was reviewed: low build risk, moderate runtime risk (unauthenticated `save_pcd`, unbounded memory, stale `feature/ros2` branch); evaluate only in WSL with `ROS_LOCALHOST_ONLY=1`. Point-LIO (Unitree's port) is the closest match to the L2 |
| Simulation realism | Missing: point time field (deskew), noise model, closed-loop driving for loop-closure metrics |
| Integration | Missing: LIO pose/velocity into `rover_ekf`, map → `odom` TF, obstacle output to the controller |

**Recommended order:**

1. ~~Deadman timeout in the drive path~~ (done 2026-11-15).
2. ~~Make `telemetry.py`'s synthetic fallback opt-in~~ and ~~fix `gps_bridge` status/covariance~~ (both done 2026-11-15). Still to do: record the receiver standing still for 10 min to set the UERE parameters.
3. Add a per-point time field to the simulated cloud and build Point-LIO (needs permission), then run E1 against KISS-ICP.
4. Fuse LIO into `rover_ekf` (E2), with one TF tree and one origin.
5. Pure-pursuit controller on NAVnode targets in simulation, then the ArUco node.
6. Vendor dashboard assets; add the judge-click UI and the geofence.

## How this review was done

Each package's source was read. Fixes were tested in WSL2 (Ubuntu 22.04, ROS 2 Humble): `colcon build`, `scripts/tests/navnode_nmea_smoke_test.sh` (socat-fed NMEA), and a direct run of the EKF node. Findings marked "code read" were not exercised on hardware. Nothing was tested on the Jetson.
