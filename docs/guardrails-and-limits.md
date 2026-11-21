# System Guardrails, Cautions & Latency Limits

This reference catalogs **every safety guardrail enforced in code**, **remaining hardware/autonomy gaps**, **verified rate/timeout/latency limits**, and **compute/network constraints** for D.A.V.E. Every number below comes directly from source inspection or measured run logs; items marked **UNVERIFIED ON HARDWARE** have been tested in WSL2 unit/smoke/Gazebo tests or inspected in source only.

---

## 1. Active Guardrails vs. Remaining Gaps

### Safety & Hardware Protection

| Domain | Implemented Guardrail (Exact Behavior) | Remaining Gap (Needs Hardware Wiring / Autonomy Stack) | Source |
|---|---|---|---|
| **Wheel Drive Deadman** | `vesc_can_driver_node` enforces `command_timeout = 0.5 s` on `RCL_STEADY_TIME` (`enforceTimeout`) and subscribes `/control_mode` (`enable_control_mode_lock = true`). On timeout, non-`"drive"` mode, zero velocity (`\|v\| < 0.01` or `\|erpm\| < 10`), or NaN, sends `CAN_PACKET_SET_CURRENT_BRAKE` (`comm_id = 2`, `brake_current = 5.0 A` when `brake_on_zero = true`). | Brake current (`5.0 A`) and SocketCAN recovery are **untested on physical VESCs**. | [vesc_can_driver_node.cpp](../ros2_ws/src/drive_package/src/vesc_can_driver_node.cpp), [drive_math.hpp](../ros2_ws/src/drive_package/include/drive_package/drive_math.hpp) |
| **Wheel Speed Ceiling** | `computeVescCanCommand` clamps ERPM to `[-max_erpm, +max_erpm]` (`7000` default) in **both** normalized and metric modes; `min_erpm = 100` torque floor; `ramp_rate = 1.0 /s`. | Default hardware `wheel_radius = 0.10 m` still disagrees with URDF `0.15 m` until physical tires are measured; negative wheel CAN IDs disable channels without startup error. | [vesc_can_driver_node.cpp](../ros2_ws/src/drive_package/src/vesc_can_driver_node.cpp) |
| **Joystick Input Guard** | `drive_node` enforces `current_mode_ == "drive"` lockout (`stopMotors()`), `axes.size() < 6` stop fallback, `std::isfinite` validation, `0.05` deadzone, `right_trigger_forward = true` (R2 forward), consistent CCW yaw in reverse, `0.4 s` active `/joy` suppression of idle zero `/cmd_vel`, and SI `/cmd_vel` (`W = 0.60 m`, `max_wheel_speed = 1.0 m/s`); `input_viewer.py` guards `<6`-axis messages. | Trigger and turn sign still need an elevated-wheel operator bench sign-off on hardware. | [drive_node.cpp](../ros2_ws/src/drive_package/src/drive_node.cpp), [drive_math.hpp](../ros2_ws/src/drive_package/include/drive_package/drive_math.hpp) |
| **Sim Drive Watchdog** | `sim.launch.py` sets `subscribe_cmd_vel:=false` on `sim_drive_bridge.py` whenever `use_drive_node:=true`, forcing Gazebo driving through `drive_node` lockouts and sensitivity; `sim_drive_bridge.py` stops if `/cmd_vel` (`< 0.3 s`) or `/motor/*` (`< 0.5 s`) is stale on ROS clock; passes SI `/cmd_vel` (clamped) and unmixes `/motor/*` (`W = 0.60 m`). | No simulated wheel slip, grade dynamics, or CAN faults in Gazebo `DiffDrive`. | [sim_drive_bridge.py](../ros2_ws/src/rover_description/scripts/sim_drive_bridge.py), [sim.launch.py](../ros2_ws/src/rover_description/launch/sim.launch.py) |
| **Arm Base Motion Lock** | `motion_coordinator_node` defaults `allow_base_motion = False` (auto IK moves arm only); uses `time.monotonic()`; normalizes raw-step `/gripper_state` (`0..40000`) cleanly; SI `0.60 m`/`1.0 m/s` fallback. | If `allow_base_motion:=true`, no software joint torque limits or LiDAR collision stop yet; `0.5 s` EKF timeout switches TF to open-loop fallback. | [motion_coordinator_node.py](../ros2_ws/src/arm_controller/arm_controller/motion_coordinator_node.py) |
| **Arm Stepper Watchdogs** | Manual hold expires after `0.4 s` on `time.monotonic()`; Arduino firmware enforces `WATCHDOG_TIMEOUT_MS = 500` on serial commands; `MultiThreadedExecutor` + `15 s` cancelable action server; background daemon reconnect; logical `*_invert_dir` isolation. | **Pure open loop** (no limit switches or encoders); `100 kHz` GPIO step-rate floor (`arm_stepper_driver`) and `USE_GPIO_TRIGGER = true` (`50 kHz`) on Arduino await hardware testing. | [arm_stepper_driver_node.py](../ros2_ws/src/arm_controller/arm_controller/arm_stepper_driver_node.py), [arm_stepper_arduino_driver_node.py](../ros2_ws/src/arm_controller/arm_controller/arm_stepper_arduino_driver_node.py) |
| **Emergency GPIO Script** | `reset_arm_stepper_gpios.py` drives pins `[33, 18, 22, 32, 13, 16]` LOW for `0.1 s`; `lights.py`, `servo_node.py` (`destroy_node`), and `arm_stepper_driver_node.py` scope `GPIO.cleanup([...])` to their own pin lists so one node's exit cannot reset another node's pins. | `GPIO.cleanup()` returns released pins to **floating inputs**—external hardware pull-downs are mandatory on DM556Y/Arduino inputs. | [reset_arm_stepper_gpios.py](../scripts/utilities/reset_arm_stepper_gpios.py), [lights.py](../ros2_ws/src/light_package/light_package/lights.py), [servo_node.py](../ros2_ws/src/servo_controller/servo_controller/servo_node.py) |
| **Stepper Test Cap** | `test_dm556y_steppers.py` refuses frequencies `<= 0` or `> 25000 Hz` without `--force`. | Hardcodes `PUL_PIN = 15, DIR_PIN = 13`, contradicting node pin map (`33/18`, `32/13`). **Confirm wiring before running.** | [test_dm556y_steppers.py](../scripts/hardware_tests/test_dm556y_steppers.py) |
| **IMU Failure Isolation** | `telemetry.py` (`sim_mode = false`, `use_magnetometer = false` 6-DoF Game Rotation Vector) stops publishing if hardware is missing or stalls (`read_failures >= publish_rate`, reset only on finite valid packets), reconnecting every `2.0 s`. | Untested on physical BNO085 hardware; yaw drifts slowly without magnetometer (by design, fused by EKF/GPS outdoors). | [telemetry.py](../ros2_ws/src/IMU/imu/telemetry.py) |
| **GNSS Quality Gating** | Both `gps_bridge_node.py` (with `2 s` serial reconnect) and `NAVnode` (`nav_nmea.hpp`, `verify_nmea_checksum = true`) verify NMEA `*hh` XOR checksums and accept GGA quality `{1, 2, 4, 5}` only (rejecting `0` and DR `6`); `NAVnode` supports `/gps/fix` directly (`use_gps_fix_topic:=true`) and `dashboard_helper_node.py` passes it automatically when `gps_bridge` is active. | CLI launches still default `gps_bridge_node` and `NAVnode` serial mode to `/dev/ttyUSB0`; pass `use_gps_fix_topic:=true` or separate `udev` ports when invoking `NAVnode` outside `dashboard_helper`/`sim.launch.py`. | [gps_bridge_node.py](../ros2_ws/src/navigation_system/src/gps_bridge_node.py), [nav_nmea.hpp](../ros2_ws/src/navigation_system/include/navigation_system/nav_nmea.hpp), [dashboard_helper_node.py](../ros2_ws/src/arm_controller/arm_controller/dashboard_helper_node.py) |
| **Home / Origin Guard** | `NAVnode::ensureHome()` refuses local/geodetic/ECEF add services before a fix; `ResetHome` rebases stored routes, obstacles, and history cleanly (`rebaseWaypointsToHome`); `findAlternateRoute` bounded in depth and zero-length. | Plaintext password (`741021`) in header; unauthenticated `rosbridge` exposes services over LAN/Wi-Fi. | [NAVnode.cpp](../ros2_ws/src/navigation_system/src/NAVnode.cpp), [Waypoint.hpp](../ros2_ws/src/navigation_system/include/navigation_system/Waypoint.hpp) |
| **Logging & Storage Guard** | `record_bag.sh` aborts if `< MIN_FREE_GB` (`20 GB`) and splits at `4 GB`; `run_bag.sh` signals `$FLPID` on shutdown with `SIGTERM` fallback and fails on empty `fastlio_tum.txt`; `~NAVnode()` frees `obstacles`, `currentPosition`, `GNSS`. | `NAVnode` writes `captainsLogs` relative to CWD. | [record_bag.sh](../scripts/slam/record_bag.sh), [run_bag.sh](../scripts/slam/fast_lio/run_bag.sh) |

---

## 2. Verified Rates, Periods, Timeouts & Latency Budgets

> **Caution:** Numbers in *Current Code Value* are hard constants in the repository. Items marked `[sim-measured]` or `[spec]` indicate whether they come from Gazebo runs, hardware datasheets, or design targets.

### Control, Actuation & Dashboard Timing

| Component | Parameter / Signal | Code Value | Clock Source | Latency / Starvation Risk |
|---|---|---|---|---|
| **Dashboard UI Loop** | `startControlLoopTimer()` (`app.js`) | `100 ms` (`10.0 Hz`, SI `/cmd_vel`) | Browser `setInterval` | Single Wi-Fi stall `> 500 ms` (`5` missed packets) trips VESC active-brake deadman; `> 400 ms` trips stepper manual hold. |
| **Joy Teleop** | `/joy` nominal rate (driver assumption) | `20 Hz` (`50 ms`) | Host OS | `10` missed frames trip `0.5 s` deadman. |
| **VESC CAN TX/RX** | `publish_rate` (`updateLoop`) | `50.0 Hz` (`20 ms`) | Wall timer + `RCL_STEADY_TIME` | Reads max `200` CAN frames/tick; ramps `0.02/tick` (`1.0 s` 0→full, `2.0 s` +1→−1); timeout brakes at `5.0 A`. |
| **VESC Deadman** | `command_timeout` | `0.5 s` | `RCL_STEADY_TIME` | Stale wheel command triggers immediate `SET_CURRENT_BRAKE` (when `brake_on_zero=true`). |
| **VESC Status Readback** | `status_timeout` (`STATUS 1` @ `50 Hz`) | `0.5 s` | `RCL_STEADY_TIME` | `rover_ekf` requires >=1 finite left and >=1 finite right wheel when named `/wheel_states` are used, rejecting one-sided CAN loss. |
| **Sim Drive Bridge** | `timer` / freshness windows | `20 Hz`; `/cmd_vel` `0.3 s`, `/motor/*` `0.5 s` | Node ROS clock | Respects `use_sim_time`; zero velocity published when stale. |
| **Motion Coordinator** | `timer` / `odom_timeout_s` | `20 Hz` (`50 ms`); `odom < 0.5 s` | `time.monotonic()` | `0.5 s` EKF timeout switches TF broadcaster from `world→odom` to fallback `world→base_link`. |
| **Arm GPIO / Arduino Steppers** | `control_timer` / manual hold / keepalive / timeout | `50 Hz` (`20 ms`); manual `< 0.4 s`; keepalive `> 0.1 s`; action `15 s` timeout | `time.monotonic()` / Arduino `millis()` | Reconnect sleeps on background daemon thread (`3.0 s` retry backoff); `M` serial mode inactive while Arduino `USE_GPIO_TRIGGER = true`. |

### Sensing, Estimation & Navigation Timing

| Component | Parameter / Signal | Code Value | Clock Source | Latency / Starvation Risk |
|---|---|---|---|---|
| **BNO085 Telemetry** | `publish_rate` / retry / failure cutoff | `20 Hz`; I²C `400 kHz`; retry `2.0 s`; disconnect `max(publish_rate, 1)` = `20` misses/errors | Timer + `time.monotonic()` | Synchronous I²C reads inside callback; `0.2 s` init sleep; `read_failures` resets only on finite fresh readings. |
| **IMU 15-State UKF** | `imu.launch.py` `run_kalman_filter` | Off by default (`false`); `step_dt ∈ [1e-3, 0.5] s` from stamp | Message `header.stamp` | Only intended for diagnostic/standalone testing; leaves body-frame AUKF off in normal ops. |
| **GPS Bridge** | Serial poll / reconnect / RMC dedup / baud | `20 Hz`; reconnect `2.0 s`; RMC fallback `< 2.0 s`; `38400` baud | Wall timer + `time.monotonic()` + ROS clock | `stamp` records ROS parse time, not GNSS PPS epoch; typical NMEA UART delay uncompensated. |
| **Rover EKF** | `frequency` / `dt` clamp / freshness windows | `20 Hz`; `dt ∈ [0, 0.5] s`; `/wheel_states < 0.3 s`; `/motor/* < 0.5 s` | ROS clock (`self.get_clock().now()`) | Supports `use_sim_time:=true`; predicts before wheel update; skips ticks before initial nonzero clock. |
| **NAVnode** | `mainTimer` / serial drain / `FollowRoute` feedback | `5 Hz` (`200 ms`); up to `16 × 256 B` drain/tick (`4096 B` cap); feedback `1 Hz` | Wall timer | Arrival threshold `2.0 m`; when `use_gps_fix_topic:=true`, reads `/gps/fix` cleanly without opening serial UART. |
| **Video & Morse** | GStreamer UDP (`5000,5002,5004`, secondary `5001`) | `30 fps` default (`60 fps` CSI Morse mode); `latency=0 drop-on-latency=true` | `time.monotonic()` (`morse_recorder.py`) | `morse_recorder.py` uses `_state_lock` and defaults `show_debug_window:=false` for headless safety; first-pulse dot timing assumption remains. |

### Network, DDS & Jetson Compute / Storage Budgets

| Resource | Hard Limit / Configuration | Source / Measurement | Caution |
|---|---|---|---|
| **DDS Transport** | `UDPv4` only (`useBuiltinTransports = false`), SHM disabled, `1 MiB` send/recv socket buffers, `maxMessageSize = 65500 B`, static peers `127.0.0.1`, `192.168.1.87`, `192.168.1.88` | [fastdds_profiles.xml](../fastdds_profiles.xml) | Large `PointCloud2` / raw images exceeding UDP fragmentation/socket limits stall over Wi-Fi; external peers cannot discover Jetson when profile is set. |
| **Jetson Orin Nano Super** `[spec]` | `8 GB` shared LPDDR5 (OS + ROS + GPU together), 6-core Cortex-A78AE, `7W/15W/25W/MAXN_SUPER` | [slam/platform.md](slam/platform.md) | 8 GB pool is hard cap for mapping/Gaussian layers; leave `>= 1.5 GB` headroom (`slam/architecture.md`). |
| **Unitree L2** `[spec]` | `5.55 Hz` rotation (~`180 ms`/scan), `64,000 eff. pts/s`, `~500 Hz` IMU report rate, no hardware PTP/PPS, `~10 s` warmup | [slam/platform.md](slam/platform.md) | `unilidar_sdk2` Ethernet host IP defaults to `192.168.1.2`, conflicting with DDS `192.168.1.87/.88` on a single NIC; serial fallback `/dev/ttyACM0` clashes with Arduino stepper. |
| **FAST-LIO2 (Sim E1/E1b)** `[sim-measured]` | E1 (`yard_raw`, `16.43 m`): `ATE 0.076 m`, `69%` of 1 core, `149 MB` RSS.<br>E1b (`e1b_route`, `124.27 m`, updated evaluator): `ATE 0.028 m` (`max 0.093 m`), `ATE rot 0.380 deg`, `SE(3) RPE 10 m: 0.028 m (0.28 %) / 0.466 deg`, vector loop `0.096 m (0.08 %)`. | [slam/research-plan.md](slam/research-plan.md) | Dev PC (`i7-12650H` WSL2) numbers do **not** establish Jetson Orin Nano ARM timing. Gazebo Mesa software renders at `~0.25x` real time. |
| **Bag Recording** | `MIN_FREE_GB = 20`, `BAG_SPLIT_BYTES = 4,000,000,000`, `zstd` compression | [record_bag.sh](../scripts/slam/record_bag.sh) | E1b uncompressed sqlite3 bag was `504.9 MiB` for `188 sim s` (~`2.7 MB/sim-s`). |

---

## 3. Critical Operational Cautions for Any New Engineer

1. **Confirm Physical Wheel Radius (`0.10 m` vs `0.15 m`):** Simulation uses `0.15 m`; hardware defaults in `vesc_can_driver_node` and `rover_ekf` remain `0.10 m`. Update whichever doesn't match the physical tires before driving outdoors.
2. **Never run `gps_bridge_node` and serial-mode `NAVnode` on the same UART:** Set `use_gps_fix_topic:=true` on `NAVnode` (or map dedicated `udev` symlinks) so only one node opens the GNSS receiver.
3. **Never power DM556Y/Arduino steppers without external pull-down resistors:** Jetson pin `GPIO.cleanup()` leaves pins floating (`input`), and limit switches are absent in firmware/software.
4. **Never rely on `/imu/odometry` or uncalibrated `/imu/data_raw` on slopes without verification:** `kalman_filter.py` is disabled by default (`run_kalman_filter:=false`); `rover_ekf` is 2-D planar only.
5. **Never confuse Gazebo `e1b` accuracy (`0.028 m`) with outdoor Unitree L2 readiness:** Real L2 bags on open uneven terrain with helical rolling sweeps and Jetson `tegrastats` measurements are still mandatory before autonomy trials.
