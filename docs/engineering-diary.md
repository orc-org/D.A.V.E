# Engineering Diary & Critical Fix Audit

This diary logs the development, consolidation, fixes, benchmark runs, and engineering audits performed on D.A.V.E., **with an explicit critical evaluation of whether each fix was necessary, whether it solved the root cause, and how it compares to robotics literature.**

---

## Diary Timeline

### 2026-11-06 — `2924889` & `4e2fda4`: Porting Drive Modes & `navnode_fix`
* **Commits:** `2924889` (`drive_node` control modes & sensitivity), `4e2fda4` (`navnode_fix` port).
* **What happened:**
  1. Ported `/control_mode` and `/sensitivity` (`ultra = 0.25`, `fine = 0.5`, `coarse = 1.0`, default `0.5`) to `drive_node.cpp` (`stopMotors()` when leaving `"drive"`), **and silently flipped Xbox triggers** from `rt_norm - lt_norm` (RT forward) to `lt_norm - rt_norm` (LT forward).
  2. Ported `navnode_fix` memory/collision patches into `NAVnode.cpp` (null checks on `node->point`, `distanceToNextWaypoint < 1e-9` guard, `segmentLength < 1e-9` guard + `std::clamp` in `calculatePointOfInterest`, `delete` → `new Waypoint` copies for `lastFiveWaypoints`, `SelectRouteToEdit` name fix, `routeToEdit` usage in remove waypoint services).
* **Critical Evaluation:**
  * *Porting modes (`2924889`):* Needed for shared arm/drive safety, **but** defaulting sensitivity to `0.5` was wrong (reverted to `1.0` the next day), flipping L2/R2 contradicted [dashboard/app.js](../dashboard/app.js) without documenting it in the commit, and `msg->axes[0..5]` remained unguarded against `<6`-axis controllers (segfault risk).
  * *`navnode_fix` (`4e2fda4`):* Stopped immediate crashes (`delete[]`, null deref, divide-by-zero), **yet left fundamental bugs alive:** `findAlternateRoute(int first, int second)` in [NAVnode.cpp](../ros2_ws/src/navigation_system/src/NAVnode.cpp) still lacks a `length < 1e-9` check and recurses with no cycle/depth limit (overlapping circular obstacles can overflow the stack), `obstacles` raw pointers leak in `~NAVnode`, and ad-hoc geometric circle detours ignore kinodynamic constraints (unlike ROS 2 Nav2 costmaps + DWA/MPPI).

---

### 2026-11-07 — `5654b8c`: Large Consolidation & Coordinate Math Fixes (35 files)
* **What changed:**
  * `Waypoint.cpp`: converted `home_latitude`/`home_longitude` deg → rad inside `EarthCentred_To_Local` and `local_To_EarthCentred`; fixed `(lon, lat, alt)` arg order in `resetHome` / `updatePositonData`; replaced `(int x, int y)` + radian return with `double` + degree return in static helpers; added [waypoint_conversion_test.cpp](../scripts/tests/waypoint_conversion_test.cpp).
  * `NAVnode.cpp` / `NAVDisplay.cpp`: added `nmeaToDegrees(ddmm)` (`stod` instead of `stoi`); replaced `lon > 0` with `lon != 0`; replaced `asin(dy/dx)` with `atan2(dx, dy) mod 2π`; separated `SelectRouteToFollow` / `SelectRouteToEdit` and aligned `NAVDisplayInfo` topic name.
  * `rover_ekf/ekf_node.py`: changed wheel sum `* 0.5` → `* 0.25`; stopped feeding `self.x[2]` back into `imu_raw_callback` (`1×5` yaw-rate update).
  * `imu.launch.py`, `kinematics.py`, `SerialPort.hpp`, `.gitattributes`: launch-coupled filter `dt`, lazy `matplotlib` import, deduplicated `SerialPort.hpp`, LF line endings, default `sensitivity_ = 1.0`.
* **Critical Evaluation:**
  * **Coordinate & EKF fixes were 100% essential.** Evaluating ENU rotations on degree values corrupted every coordinate in Alberta (`~51.46° N`), `lon > 0` blocked all Western Hemisphere fixes, and self-state pseudo-measurements in `ekf_node.py` artificially collapsed state covariance $P$ on zero innovation—classic filter inconsistency (Bar-Shalom; Maybeck).
  * **Where `5654b8c` stayed half-finished:**
    1. `Waypoint.cpp` still repeats a 5-pass fixed-point ECEF→geodetic loop rather than standard closed-form Bowring (1976) / Vermeille (2002), and drops ENU Up ($U=0$), introducing tangent-plane altitude error over range.
    2. `waypoint_conversion_test.cpp` checks 1 NW-hemisphere point against crude spherical constants (`111132 / 111320 cos φ`) with a loose `0.5 m` tolerance—masking a `14.9 cm` spherical-vs-WGS-84 discrepancy (`69.508 m` vs `69.359 m`) while leaking `home`.
    3. `NAVnode` still parsed NMEA by brittle substring search (`"A,"` / `"N,"` landing on commas) until `7472638`.
    4. Setting `imu.launch.py` `dt = 1 / publish_rate` patched a symptom on `kalman_filter.py` without addressing that `kalman_filter` completely ignores actual message header timestamps and relies on fixed `self.dt`.

---

### 2026-11-08 — `dde8ff8`, `ee309c7`, `3745cf3`: WSL2 Build Bugs & NMEA Smoke Test
* **What changed:**
  * `dde8ff8`: `SerialPort::write(char*)` replaced `sizeof(message)` (8-byte pointer size!) with `std::strlen(message)`.
  * `ee309c7`: `ListOfWaypoints::removePoint` moved `return false;` outside the `while` loop; separated `++start` from `substr(start+2)` sequence point; removed non-ament `raylib` from `ament_target_dependencies`.
  * `3745cf3`: added [navnode_nmea_smoke_test.sh](../scripts/tests/navnode_nmea_smoke_test.sh).
* **Critical Evaluation:**
  * **All needed.** Compiler warnings caught the `sizeof(char*)` truncation bug (`dde8ff8`); `removePoint` could previously delete *only* the head node (`ee309c7`).
  * **Remaining debt in `serial_communication` & CMake:** `SerialPort::write(std::string)` still leaks `new char[]`; `XLRnode` / `MorseBridgeNode` sleep `1500 ms` on ROS callback threads; `XLR_User_input.cpp` blocks forever in `while(true)` inside its constructor before `rclcpp::spin()`; `navigation_system/CMakeLists.txt` still cross-compiles `SerialPort.cpp` directly via `${ROS2_WS}` path hacks.

---

### 2026-11-09 — `9b7eab7`, `fb1b156`, `f8a4c7b`, `d1fd3d1`: Script Hardening & First Gazebo LiDAR Pipeline
* **What changed:**
  * `9b7eab7` & `fb1b156`: fixed infinite `shift 2` hang and eliminated `eval` in `receive_stream.sh`; added VCM init inside `trigger_sweep`, `_af_lock`, `daemon=True`, and `parse_known_args` in `video_streamer.py`; changed `200000` → `20000 Hz` default and added `25 kHz` safety cap in `test_dm556y_steppers.py`; blocked `test_gps.py` (`readline()` vs 100% spin, file later removed though still referenced in READMEs); clarified `reset_arm_stepper_gpios.py` (`GPIO.cleanup()` floats pins); added `MIN_FREE_GB`, missing-topic check, and `4 GB` splits in `record_bag.sh`; added startup liveness/port checks in `start_*_system.sh`; hardened `navnode_nmea_smoke_test.sh` (`SIGINT` + `pkill` + `$WORK` isolation).
  * `f8a4c7b` & `d1fd3d1`: added Gazebo `gpu_lidar` (`360×16`, 10 Hz), `200 Hz` IMU, RGB-D camera, `slam_yard.sdf`, `sim_drive_loop.py`, and `eval_trajectory.py`; ran KISS-ICP 1.3.0 E1 baseline (`ATE 0.426 m` over `16.5 m`, `52 Hz`, `70 MB`).
* **Critical Evaluation:**
  * **Script hardening fixes were high-value practical operations engineering.**
  * **Simulator & E1 baseline critique:** Gazebo bridged *only* perception sensors + `/odom` + `/cmd_vel_sim` (`gazebo_control.xacro` publishing `odom → base_footprint`). No `/gps/fix`, no `/wheel_states`, no `/imu/euler`, and `rover_arm.xacro` was left commented out—meaning the simulator could test LIO in isolation, **not the rover's real EKF/NAV/arm stack**. Running KISS-ICP (`LiDAR-only`) proved Xu et al. (2022)'s thesis in reverse: sparse `16`-ring non-spinning-style scanning without IMU priors yielded `0.426 m` ATE and `1.43 m` max error on a tiny 16.5 m box.

---

### 2026-11-15 — `7472638` through `d56a23b`: Major Consolidation Review & FAST-LIO2 E1 Run
This single day landed 9 major commits addressing code review findings and getting FAST-LIO2 running:

1. **`7472638` (`NAVnode` parser/target/track-log & `ekf_node` yaw/`use_gps`):**
   * *Needed:* Fixed broken RMC parsing (`nmeaFields` + `4 KB` buffer), added `validateTarget()`, fixed `pointsVisited` double-free (`new Waypoint(*currentPosition)`), fixed unreachable `SelectRoute*` return string and `removeLastWaypoint` wrong-route bug, fixed `ekf_node` self-state yaw update, and added `use_gps`.
   * *Critique:* `nmeaFields` still skipped NMEA `*hh` checksum validation and GGA quality `6` rejection; kept dual GNSS serial ownership conflict with `gps_bridge_node.py`; `removeLocalObstacle_helper`/`removeGeodeticObstacle_helper` kept `(int, int)` signatures.
2. **`e618c7c` (`vesc_can_driver_node` deadman timeout):**
   * *Needed, but incomplete for real slopes:* Using `RCL_STEADY_TIME` for `0.5 s` timeout is best practice, **but** timeout sets `comm_id = 1` (`SET_CURRENT = 0` = freewheel coast) instead of VESC `CAN_PACKET_SET_CURRENT_BRAKE` (`comm_id = 2`), leaving the rover unbraked on inclines.
3. **`6ec6679` (`IMU/telemetry.py` no silent synthetic fallback):**
   * *Needed:* Eliminating silent fake-IMU data prevented catastrophic silent autonomy errors.
   * *Remaining flaw:* Enabled `BNO_REPORT_ROTATION_VECTOR` + magnetometer (EMI-sensitive) instead of `BNO_REPORT_GAME_ROTATION_VECTOR` (SH-2 `0x08`); resets `read_failures` even when `None` is returned; mixes tared Euler with untared raw quaternion.
4. **`f4de53d` (`gps_bridge_node.py` fix status, HDOP covariance, RMC dedup, dashboard no-fix guard):**
   * *Needed & solid:* Matches ROS `NavSatStatus` semantics (`STATUS_NO_FIX` for quality `0`/`6` with `0,0` for rosbridge JSON safety), avoids double epoch fusion (`GGA + RMC`).
   * *Minor gaps:* Ignores `$GNGST` error ellipses, timestamps with arrival clock, lacks serial reconnect.
5. **`e205531` (`motion_coordinator`, `ekf_node`, `kalman_filter`, `NAVnode`):**
   * *Good:* `allow_base_motion:=false` stop-gap prevented rogue arm driving; `world → odom` fallback avoided dual `base_link` TF parent *except* against Gazebo's `base_footprint`; EKF measured `dt`, single `/motor/*` source, zero-speed fusion, `imu_yaw_offset_deg`, `datum_*`; timer-based `FollowRoute` ended detached-thread UAF; `ensureHome()` stopped blind `(0,0)` home init.
   * *Band-aids / Architecture flaws left intact:*
     * `kalman_filter.py` Euler unwrap (`±180°`) and body-frame tilt gravity compensation are band-aids on a mathematically broken $SO(3)$ filter (adds body $\omega$ straight to Euler angles, integrates body acceleration into body position without $R_{nb}$, double-filters BNO085's internal MEKF). Disabling `/imu/odometry` by default essentially admits the node shouldn't drive navigation.
     * `ekf_node.py` kept non-Joseph covariance $P = (I - KH)P$ (unlike `robot_localization`, Moore & Stouch 2014), wall-clock `time.monotonic()` (fails sim/replay), first-fix GPS drop, spherical Earth radius `6371000 m`, and planar 2-D assumptions.
6. **`74fc8f2` (CAN `STATUS 1` → `/wheel_states`, EKF wheel speed preference):**
   * *Needed concept, broken edge cases:* Parsing CAN ID `(9 << 8) | vesc_id` follows VESC protocol, **but** [ekf_node.py](../ros2_ws/src/rover_ekf/rover_ekf/ekf_node.py) keeps `wheel_radius = 0.10 m` (vs URDF `0.15 m`) and `wheel_states_cb` averages any `>=2` wheels without checking left/right symmetry (unbalanced CAN loss converts turning rate into fake straight speed).
7. **`5114de7` (Offline vendored dashboard):**
   * *Essential:* Eliminates field internet dependency (`fetch.sh --verify` passes), pins `roslib@1.4.1`, restores OSM attribution. Still lacks automated JS/lint testing in CI and site tile caching.
8. **`1e56edf` & `d56a23b` (FAST-LIO2 ROS 2 `cc3c9e8b` + `e1_fastlio`):**
   * *Algorithm pick justified by literature (Xu et al., IEEE T-RO, 2022):* FAST-LIO2 tightly couples IESKF + `ikd-Tree` raw registration and is dramatically better suited to Unitree L2 than LiDAR-only KISS-ICP (`0.076 m` vs `0.426 m` ATE on `yard_raw`).
   * *Critical E1 test flaws:* Inspected `/tmp/bags/yard_raw` directly: **IMU had zero noise and zero bias** (`gyro std=0`, `accel std=0`), route didn't close (`2.513 m` end gap), and `lidar_point_time.py` hardcoded `1e-6 s` point times—completely bypassing FAST-LIO2 backward motion deskewing (`IMU_Processing.hpp`) and Unitree L2 scan/clock behavior.

---

### 2026-11-16 (Uncommitted Working Tree + `e1b_route` & Full Stack Audit)
* **Uncommitted changes evaluated:**
  * `physical_params.yaml` & `sensors.xacro`: added uncalibrated Gaussian white noise + static/dynamic bias to Gazebo IMU (`gyro std 0.002, bias 0.003±0.001`; `accel std 0.03, bias 0.05±0.02`) → **Good direction** (removes zero-noise illusion), though Gazebo IMU `orientation` remains fixed `[0,0,0,1]` (`std = 0`) and values aren't measured from a physical Unitree L2.
  * `sim_drive_route.py` (`3`-lap closed route, `~124 m` in `slam_yard.sdf`) & `run_bag.sh` (`RATE` env var): **Important forensics found during verification:**
    1. Gazebo ran `e1b_route` at **`0.25x` real-time** (`1880` scans took `751.60 s` wall time), and `e1b` replayed at **`--rate 3`** (`e1b/fastlio/play.log`), invalidating wall-clock load benchmarking for Jetson transfer.
    2. **`run_bag.sh` process orphan bug:** `kill -INT $FL` signals the `ros2 run` wrapper instead of `$FLPID`, leaving `fastlio_mapping` and its monitoring subshell alive indefinitely—observed directly as **`27,464` lines** (`~16 hours`, `7.9%` CPU, `151,728 KB` RSS) in `/tmp/e1b/fastlio/usage.log`.
    3. `eval_trajectory.py` improvements (`loop drift`, `RPE 10 m`): Still deviates from Sturm et al. (*IROS* 2012) benchmark definitions: default `--est-clock bag` crashes with `SVD did not converge` on FAST-LIO output; uses bag receive times instead of physics time; completely discards TUM quaternions; computes `RPE` after global Umeyama alignment rather than SE(3) relative transforms; compares scalar norm differences for `loop drift` (hiding orthogonal end gaps, reporting `0.03%` on `yard_raw`'s non-closed `2.513 m` path); computes `z drift` on unaligned raw $z$; and omits the IMU-to-`base_footprint` lever arm.
    4. Even with IMU noise enabled, `e1b` (`ATE RMSE 0.028 m`, `z drift 0.109 m`, `1875` poses) remains an **artificial closed-box floor** (4 close walls, flat ground, zero scan distortion, zero wheel-slip disturbances). Real Unitree L2 bags on open hilly terrain remain mandatory.

---

### 2026-11-16 (Batch Remediation, Unit/Smoke Test Expansion & Gazebo Full-Stack Enablement)
* **What changed across the workspace:**
  1. **Drive Safety & Unified SI `/cmd_vel` ([drive_math.hpp](../ros2_ws/src/drive_package/include/drive_package/drive_math.hpp), [drive_node.cpp](../ros2_ws/src/drive_package/src/drive_node.cpp), [vesc_can_driver_node.cpp](../ros2_ws/src/drive_package/src/vesc_can_driver_node.cpp), [sim_drive_bridge.py](../ros2_ws/src/rover_description/scripts/sim_drive_bridge.py)):** Extracted shared `drive_math.hpp` helper; guarded `msg->axes.size() < 6` and NaNs; standardized `right_trigger_forward:=true` (R2 forward, matching `dashboard/app.js`) and consistent CCW turn polarity when reversing; converted `/cmd_vel` mixing/unmixing to SI (`wheel_separation = 0.60 m`, `max_wheel_speed = 1.0 m/s`); added active `CAN_PACKET_SET_CURRENT_BRAKE` (`comm_id = 2`, `brake_current = 5.0 A`, `brake_on_zero = true`), `[-max_erpm, max_erpm]` clamping in metric mode, and `/control_mode` lockout inside `vesc_can_driver_node`. Added 4 GTest cases (`test_drive_math.cpp`).
  2. **IMU & Rover EKF Hardening ([telemetry.py](../ros2_ws/src/IMU/imu/telemetry.py), [kalman_filter.py](../ros2_ws/src/IMU/imu/kalman_filter.py), [imu.launch.py](../ros2_ws/src/IMU/launch/imu.launch.py), [ekf_node.py](../ros2_ws/src/rover_ekf/rover_ekf/ekf_node.py)):** Switched BNO085 default to magnetometer-immune `BNO_REPORT_GAME_ROTATION_VECTOR` (`use_magnetometer:=false`) and fixed `read_failures` reset only on finite valid samples; gated legacy `kalman_filter` behind `run_kalman_filter:=false` and fixed its timestamp `dt`, $R_{nb}$ world accel rotation, and ZYX Euler rate matrix; updated `rover_ekf` to ROS clock (`self.get_clock().now()`), symmetric Joseph covariance update, predict-before-wheel-update, L/R-balanced `/wheel_states` pairing, WGS-84 tangent-plane radii, and `publish_tf` param. Added `test_ekf_math.py` (4/4 pytest) and `test_imu_math.py` (1/1 pytest).
  3. **Navigation, Serial & Enigma Fixes ([nav_nmea.hpp](../ros2_ws/src/navigation_system/include/navigation_system/nav_nmea.hpp), [NAVnode.cpp](../ros2_ws/src/navigation_system/src/NAVnode.cpp), [NAVDisplay.cpp](../ros2_ws/src/navigation_system/src/NAVDisplay.cpp), [gps_bridge_node.py](../ros2_ws/src/navigation_system/src/gps_bridge_node.py), [SerialPort.cpp](../ros2_ws/src/serial_communication/src/SerialPort.cpp), [XLR_User_input.cpp](../ros2_ws/src/serial_communication/src/XLR_User_input.cpp), [EnigmaMachine.cpp](../ros2_ws/src/enigma_machine/src/EnigmaMachine.cpp)):** Added `nav_nmea.hpp` `*hh` XOR checksum verification and GGA `{1,2,4,5}` quality gating to `NAVnode` (`verify_nmea_checksum:=true`), optional `/gps/fix` subscription (`use_gps_fix_topic`), 16-read serial draining, bounded `findAlternateRoute`, `double` obstacle helpers, `~NAVnode()` memory cleanup, manual-step `routeFinished` updates, and `ResetHome` coordinate rebasing (`Obstacle::rebaseToHome()`); made `raylib` optional in `navigation_system/CMakeLists.txt` (`NAVnode` builds headless); protected `NAVDisplay` cross-thread state with `state_mutex_`, wired live `currentPosition`, fixed `removeLast()` empty guard and response bindings; added 2 s serial reconnect in `gps_bridge_node`; fixed `SerialPort` `portIsOpen`/zero-leak `write`, `XLR_User_input` background stdin thread, and `EnigmaMachine` `TrieNode` null initialization. Verified with `test_nav_math` (3/3 GTest), `waypoint_conversion_test`, and updated `navnode_nmea_smoke_test.sh` (14/14 assertions including corrupted `*FF` NMEA checksum rejection).
  4. **Arm Stepper & Coordinator Timing/Unit Fixes ([motion_coordinator_node.py](../ros2_ws/src/arm_controller/arm_controller/motion_coordinator_node.py), [arm_stepper_driver_node.py](../ros2_ws/src/arm_controller/arm_controller/arm_stepper_driver_node.py), [arm_stepper_arduino_driver_node.py](../ros2_ws/src/arm_controller/arm_controller/arm_stepper_arduino_driver_node.py)):** Standardized `time.monotonic()`; normalized `/gripper_state` raw steps in `motion_coordinator` and aligned SI fallback scaling (`0.60 m`, `1.0 m/s`); fixed `execute_gripper_goal` missing attribute, `1.0 - step/max` polarity, and single-threaded `rate.sleep()` deadlock via `MultiThreadedExecutor` + cancel/15s abort; moved Arduino DTR `connect()` to a daemon thread; separated logical angle tracking from `*_invert_dir` hardware polarity (`test_arm_math.py` 1/1 pytest).
  5. **Gazebo Full-Stack Bridge & SLAM Eval Tooling ([sim_sensor_bridge.py](../ros2_ws/src/rover_description/scripts/sim_sensor_bridge.py), [sim.launch.py](../ros2_ws/src/rover_description/launch/sim.launch.py), [run_bag.sh](../scripts/slam/fast_lio/run_bag.sh), [eval_trajectory.py](../scripts/slam/eval_trajectory.py)):** Added `use_sim_stack:=true` option bridging `/wheel_states`, `/gps/fix`, and `/imu/euler` cleanly to `rover_ekf` (`publish_tf:=false`) and `NAVnode` (`use_gps_fix_topic:=true`); fixed `$FLPID` shutdown and empty-output guard in `run_bag.sh`; upgraded `eval_trajectory.py` to output ATE rotation RMSE, SE(3) 10 m RPE (translation + deg), vector loop closure error, aligned Z drift, and clear overlap errors (`test_eval_trajectory.py` 3/3 pytest).
* **Critical Evaluation Against Verification:**
  * All 16 ROS 2 packages compiled in WSL2 (`colcon build`, headless `NAVnode` without `raylib`).
  * `colcon test` across `drive_package`, `imu`, `rover_ekf`, `navigation_system`, and `arm_controller` ran `24` test items (`0` errors, `0` failures, `4` skipped in `drive_package` `cppcheck` because `cppcheck` binary is not installed; all `13/13` unit tests passed), `scripts/slam/test_eval_trajectory.py` (`3/3` pytest passed), `navnode_nmea_smoke_test.sh` (`14/14` checks passed), `waypoint_conversion_test.cpp` passed, `dashboard/vendor/fetch.sh --verify` passed, re-scoring `/tmp/bags/e1b_route` (`1875` poses) produced `ATE 0.028 m / 0.380 deg`, `SE(3) RPE 10 m: 0.028 m / 0.466 deg`, `vector loop 0.096 m (0.08 %)`, and headless Gazebo `sim.launch.py use_sim_stack:=true` under Mesa software GL verified live end-to-end `/wheel_states`, `/gps/fix`, `/imu/euler`, `/ekf/odometry`, and `/NAVDisplayInfo` response to `/cmd_vel`.
  * Physical rover bench checks (`can0`/VESCs, BNO085, UART/udev, pin 32/37, Unitree L2 hardware) and closed-loop autonomy remain unexecuted in hardware by environment scope.
