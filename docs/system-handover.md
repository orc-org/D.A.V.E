# System Handover Manual (Technical Takeover Guide)

This document is the technical onboarding reference for any engineer taking over **D.A.V.E.** (Okanagan Rover Craft, ROS 2 Humble on NVIDIA Jetson Orin Nano Super, with a WSL2 Ubuntu 22.04 + Gazebo Fortress simulation tier). It describes **what actually exists and works today**, **what was fixed vs. what remains untested on physical hardware**, **how data, frames, units, and clocks flow across packages**, and **what must be verified before the rover drives outdoors**.

For timing ceilings and active/missing safety interlocks, see [`guardrails-and-limits.md`](guardrails-and-limits.md). For the chronological commit-by-commit rationale and technical review, see [`engineering-diary.md`](engineering-diary.md). For remaining hardware and autonomy tasks, see [`remediation-roadmap.md`](remediation-roadmap.md).

---

## 1. Subsystem Maturity & Field Readiness Matrix

| Subsystem | Code Location | Current State | What Works Today | What Is Broken, Missing, or Untested on Rover |
|---|---|---|---|---|
| **Teleop Mixing** | `drive_package/include/drive_package/drive_math.hpp`, `src/drive_node.cpp` | Hardened (verified by `test_drive_math` GTest) | Mixes `/joy` and SI `/cmd_vel` (`m/s`, `rad/s`, `W=0.60 m`) to four `/motor/*` topics; supports `/control_mode`, `/sensitivity`, `axes.size() < 6` safety stop, NaN rejection, R2-forward polarity (`right_trigger_forward:=true`), and consistent CCW yaw in reverse. | Not yet bench-verified with physical gamepad + hardware VESC bus together. |
| **VESC CAN Driver** | `drive_package/src/vesc_can_driver_node.cpp`, `include/drive_package/drive_math.hpp` | Hardened (verified by `test_drive_math` GTest; **untested on VESCs**) | SocketCAN TX (`SET_RPM`, `SET_CURRENT`, `SET_CURRENT_BRAKE`, `SET_DUTY`), `0.5 s` steady-clock deadman sending `CAN_PACKET_SET_CURRENT_BRAKE` (`comm_id=2`, default `5.0 A`), `[-max_erpm, max_erpm]` clamp in metric mode, independent `/control_mode` lockout, `CAN_PACKET_STATUS` (`0x09`) → `/wheel_states`. | **Measure physical tires:** default `wheel_radius = 0.10 m` still disagrees with `physical_params.yaml` (`0.15 m`) by `1.5×`; untested on physical VESC hardware (`STATUS 1` enabled in VESC Tool required). |
| **IMU Reader** | `IMU/imu/telemetry.py` | Hardened (`test_imu_math` pytest; **untested on BNO085**) | Defaults to magnetometer-free `BNO_REPORT_GAME_ROTATION_VECTOR` (`use_magnetometer:=false`), explicit `sim_mode:=true` synthetic flag (no silent fallback), strict failure counter (`read_failures` resets only on finite valid data), reconnect loop on I²C drop. | Untested on physical BNO085 bus 7; by design `/imu/euler` applies tare offsets while `/imu/data_raw` quaternion remains untared. |
| **IMU 15-State UKF** | `IMU/imu/kalman_filter.py`, `launch/imu.launch.py` | **Opt-in / Diagnostic (`run_kalman_filter:=false`)** | Fixed dynamic stamp `dt`, $R_{nb}$ world acceleration rotation, and ZYX Euler rate matrix; excluded from default launch (`imu.launch.py`). | Redundant over BNO085 onboard SH-2 MEKF and `rover_ekf`; double-integrated acceleration still drifts without bound (`/imu/odometry` off by default). |
| **Rover 5-State EKF** | `rover_ekf/rover_ekf/ekf_node.py` | Hardened (`test_ekf_math` 4/4 pytest; live in Gazebo `use_sim_stack:=true`) | 20 Hz `[X, Y, θ, v, ω]` on ROS clock (`self.get_clock().now()`, works under `use_sim_time`), Joseph covariance update, predict-before-wheel-update, L/R-balanced `/wheel_states` pairing, WGS-84 ENU offsets, `publish_tf` toggle, `use_gps` toggle. | Planar 2-D kinematic model (no 3-D pitch/roll hill odometry); hardware default `wheel_radius = 0.10 m` vs `0.15 m` URDF (overridden to `0.15` in Gazebo launch); verify `imu_yaw_offset_deg` on physical rover. |
| **GNSS Bridge** | `navigation_system/src/gps_bridge_node.py` | Hardened (**untested on UART HW**) | Checksum-verified NMEA `$GPGGA`/`$GNGGA`; `GGA_QUALITY` (`{1,2,4,5}`) mapping to `NavSatStatus`; HDOP² covariance; explicit `STATUS_NO_FIX`; 2 s RMC fallback; 2 s serial disconnect/reconnect recovery. | Defaults to `/dev/ttyUSB0` (clashes with `NAVnode` in serial mode and `morse_bridge_node`); stamps with ROS arrival clock rather than GNSS PPS. |
| **Waypoint / Route Nav** | `navigation_system/src/NAVnode.cpp`, `include/navigation_system/nav_nmea.hpp`, `NAVDisplay.cpp` | Hardened (`test_nav_math` 3/3 GTest + 14/14 NMEA smoke test + Gazebo `/gps/fix`) | WGS-84 ECEF↔ENU math, `*hh` XOR checksum verification + GGA `{1,2,4,5}` quality gating, `use_gps_fix_topic` `/gps/fix` subscriber, 16-chunk serial drain, bounded `findAlternateRoute`, `double` obstacle helpers, destructor cleanup, `ResetHome` rebase (`Obstacle::rebaseToHome`), headless build without `raylib`, `NAVDisplay` `state_mutex_` + live rover pose. | **Does not drive** (no `/cmd_vel` publisher); `NAVDisplay` (optional when `raylib` missing) not yet run interactively; `captainsLogs` still writes relative to CWD. |
| **Arm IK & Coordinator** | `arm_controller/motion_coordinator_node.py`, `kinematics.py` | Hardened (`test_arm_math` pytest) | 1-DOF wrist pitch + planar base kinematics; `allow_base_motion:=false` lock; `time.monotonic()` everywhere; normalizes `/gripper_state` raw steps (`0..40000`) cleanly; SI `0.60 m` / `1.0 m/s` fallback; `/ekf/odometry` → `world → odom` when fresh. | `move_to_point` action still completes immediately without closed-loop completion waiting. |
| **Arm / Gripper Steppers** | `arm_stepper_driver_node.py`, `arm_stepper_arduino_driver_node.py`, `arm_stepper_arduino.ino` | Software fixed (**untested on Jetson GPIO / Arduino HW**) | `MultiThreadedExecutor` + non-blocking action loops with cancel/15s abort; initialized consistent `current_gripper_pos`; `time.monotonic()`; daemon-thread Arduino serial reconnect (no timer stalls); separated logical vs `*_invert_dir` hardware polarity. | **Pure open loop** (no limit switches or encoders); `100 kHz` GPIO floor (`200 kHz` nominal) untouched pending Jetson hardware bench check; `USE_GPIO_TRIGGER = true` (`50 kHz`) on Arduino firmware. |
| **Web Dashboard & Helper** | `dashboard/`, `dashboard_helper_node.py`, `start_dashboard_system.sh` | Working for teleop | Offline vendored JS/CSS/fonts (`fetch.sh --verify` passes); local tile folder + OSM fallback; matches SI `/cmd_vel` (`±0.8 m/s`, `±1.0 rad/s`) and R2-forward polarity. | Unauthenticated rosbridge (`:9090`); `/battery/status` gauge has no live publisher; `dashboard/app.js` unlinted (no Node.js in WSL env); offline competition tiles not populated. |
| **Mission Peripherals** | `rover_video_streamer`, `enigma_machine`, `light_package`, `servo_controller`, `serial_communication` | Partial / Patched (**untested on HW**) | Fixed `SerialPort` `portIsOpen`, destructor array/fd cleanup, zero-leak `write(std::string)`; fixed `XLR_User_input` stdin thread; fixed `EnigmaMachine` `TrieNode()` null pointers. | Pin 32 collision (`gripper_step_pin` vs `jetson_morse`); Pin 37 missing from `dave_custom_pins.dts`; `morse_recorder` unmutexed/wall-clock timing remains. |
| **Gazebo Sim & SLAM** | `rover_description/`, `scripts/slam/` | Full sim stack enabled (headless verified in WSL2) | `sim.launch.py use_sim_stack:=true` runs `sim_sensor_bridge.py` + `rover_ekf` (`wheel_radius:=0.15`, `publish_tf:=false`) + `NAVnode` (`use_gps_fix_topic:=true`); SI `sim_drive_bridge.py`; clean `$FLPID` shutdown in `run_bag.sh`; SE(3) + 10 m RPE + vector loop + clock-check in `eval_trajectory.py` (`test_eval_trajectory.py` 3/3 + `e1b_route` verified). | No real terrain slopes, wheel slip, or L2 helical scan distortion (`lidar_point_time.py` uses `1e-6 s`); `rover_arm.xacro` still commented out (`column_link` overlaps `lidar_link` at `x=0, z=0.40`). |

---

## 2. End-to-End Architecture & Runtime Contracts

```
Teleop / Autonomy Sources                   Mixing & Low-Level Control                     Hardware / Gazebo
─────────────────────────                   ──────────────────────────                     ─────────────────
/joy (20 Hz) ───────────────────────────┐
dashboard (:9090 → /cmd_vel @ 10 Hz SI) ┼─► drive_node ──► /motor/{fl,fr,rl,rr} [-1..1] ──┬─► vesc_can_driver_node ──► can0 (50 Hz)
motion_coordinator (/cmd_vel [m/s]) ────┘   (v ± w*0.30/1.0)                              │      └─► /wheel_states (CAN STATUS 1)
                                                                                          └─► sim_drive_bridge.py ──► /cmd_vel_sim (Gazebo)

Sensors & State Estimation                  World / Frame Owners                           Navigation & Visualization
──────────────────────────                  ────────────────────                           ──────────────────────────
GNSS (/dev/ttyUSB0, 38400) ─► gps_bridge_node ─► /gps/fix ────────────┐
BNO085 (I2C bus 7, 0x4A)   ─► telemetry (6DoF) ─► /imu/data_raw,/euler ├─► rover_ekf ──► /ekf/odometry (+ TF odom→base_link on HW)
Unitree L2 / Gazebo LiDAR  ─► /lidar/points, /lidar/imu ─► FAST-LIO2 (slam_ws overlay; /Odometry not yet fused into rover_ekf)
GNSS (/dev/ttyUSB0 or /gps/fix when use_gps_fix_topic:=true) ─► NAVnode (5 Hz) ──► NAVDisplayInfo ──► NAVDisplay (optional raylib)
```

### Key Runtime Contracts

1. **Unified `/cmd_vel` Contract (SI Units):**
   * All `/cmd_vel` publishers and subscribers (`dashboard/app.js`, `motion_coordinator_node.py`, `drive_node.cpp`, `sim_drive_bridge.py`) now operate in SI units (`linear.x` in m/s, `angular.z` in rad/s).
   * `drive_node` mixes via `v ± ω · (wheel_separation / 2)` normalized by `max_wheel_speed` (`0.60 m` and `1.0 m/s` by default); `sim_drive_bridge` passes SI `/cmd_vel` straight through (clamped to `max_lin/max_ang`) and unmixes `/motor/*` back cleanly.
2. **Trigger & Turn Polarity:**
   * Both `drive_node.cpp` (`right_trigger_forward:=true`) and `dashboard/app.js` use **Right Trigger (R2) = forward**, **Left Trigger (L2) = reverse**. Left stick `+steering` always turns CCW (`right > left`) both forward and reverse.
3. **Remaining Hardware Parameter Discrepancy (`0.10 m` vs `0.15 m` Wheel Radius):**
   * `physical_params.yaml` and Gazebo URDF/launch use `wheel_radius = 0.15 m`; hardware defaults in `vesc_can_driver_node.cpp` and `rover_ekf/ekf_node.py` remain `0.10 m` until physical wheels are measured on the rover.
4. **TF Tree Ownership:**
   * **On Hardware:** `robot_state_publisher` owns `base_link → *`; `rover_ekf` owns `odom → base_link` (`publish_tf:=true`); `motion_coordinator` publishes identity `world → odom` while `/ekf/odometry` is fresh (`< 0.5 s`).
   * **In Gazebo (`sim.launch.py use_sim_stack:=true`):** Gazebo `DiffDrive` owns `odom → base_footprint`, `robot_state_publisher` owns `base_footprint → base_link → *`, and `rover_ekf` sets `publish_tf:=false` so `/ekf/odometry` runs without dual-parent TF collisions.
5. **Time Clocks:**
   * `vesc_can_driver_node` uses `RCL_STEADY_TIME` for hardware deadman/CAN safety; `rover_ekf` and `sim_drive_bridge` use ROS node clock (`self.get_clock().now()`, supporting `use_sim_time:=true`); `motion_coordinator` and stepper drivers use `time.monotonic()`.
6. **Hardware Resource Collisions Remaining:**
   * `/dev/ttyUSB0` default shared across `gps_bridge_node`, serial-mode `NAVnode` (`use_gps_fix_topic:=false`), and `morse_bridge_node` (use `use_gps_fix_topic:=true` or `udev` symlinks).
   * Jetson Header **Pin 32** shared by `gripper_step_pin` and `jetson_morse.cpp`; **Pin 37** absent from `dave_custom_pins.dts`.
   * Subnet **`192.168.1.x`**: `fastdds_profiles.xml` peers vs. `unilidar_sdk2` default `192.168.1.2`.

---

## 3. Simulator vs. Real Rover Gap Summary

| Real Rover Feature | Present in Gazebo (`sim.launch.py`)? | Impact on Software Testing |
|---|---|---|
| Skids-steer base + wheels | Yes (`DiffDrive`, `0.15 m` wheels, `0.60 m` sep, `mu=0.8`) | Planar `/cmd_vel_sim` motion only (no rough-terrain slip or wheel sinkage). |
| Unitree L2 point cloud timing | **No** (`gpu_lidar`, `360x16` @ 10 Hz, instantaneous capture) | `lidar_point_time.py` injects flat `1e-6 s` stamps; does **not** exercise helical rolling-sweep deskewing or UDP clock drift. |
| Built-in L2 IMU (500 Hz) | **Partial** (`200 Hz`, uncalibrated synthetic noise/bias, constant identity quat) | Real L2 runs `~500 Hz` (`1 kHz` internal) with hardware offset `(-0.007698, -0.014655, 0.00667)` m. |
| GNSS, VESC wheel speeds, BNO085 Euler | **Synthetic (`use_sim_stack:=true`)** | `sim_sensor_bridge.py` bridges `/wheel_states`, `/imu/euler`, and `/gps/fix` into `rover_ekf` and `NAVnode`; lacks hardware UART/I2C faults, CAN dropouts, and multipath. |
| 1-DOF Arm URDF (`rover_arm.xacro`) | **Commented out** | `column_link` (`x=0, z=0.10..0.50`) collides with `lidar_link` (`x=0, z=0.40`) without physical CAD offsets; arm LiDAR occlusion untested. |
| Autonomous loop closure | **Open loop / teleop** (`sim_drive_route.py` uses ground truth) | Still no path-following controller converting `NAVnode`/planner targets into `/cmd_vel`. |

---

## 4. Mandatory Bench Bring-Up Sequence (Before First Field Drive)

Do **not** place the rover on the ground with autonomy or teleop enabled until these checks pass with wheels elevated off the floor:

1. **Measure & align constants:** Measure physical tire radius (`0.10 m` vs `0.15 m`) and update `physical_params.yaml`, `vesc_can_driver_node`, and `rover_ekf` if needed. Confirm VESC `pole_pairs` (`7`) and `gear_ratio` (`1.0`).
2. **Verify CAN bus & VESC signs (`wheels elevated`):** Configure `can0` (`500 kbps`), enable VESC Tool `STATUS 1` (`50 Hz`) on IDs `53, 44, 48, 7`, drive slowly forward (`0.1 m/s`), confirm positive `/wheel_states.velocity` on all 4 wheels and ERPM clamping.
3. **Verify deadman active braking & mode lock:** Kill `/joy`/teleop publisher while spinning wheels and confirm VESC switches to `SET_CURRENT_BRAKE` (`5.0 A` default) within `0.5 s`; verify `/control_mode` != `"drive"` brakes via `vesc_can_driver_node` directly.
4. **Verify Jetson pin & serial wiring detached from mechanical stops:** Confirm Pin 32 (gripper vs Morse), Pin 37 overlay, Pin 15 vs 32/33 (`test_dm556y_steppers.py`), DM556Y/Arduino pull-down resistors, and assign `udev` symlinks (or run `NAVnode` with `use_gps_fix_topic:=true`).
5. **Confirm IMU + EKF signs (`spin rover CCW in place`):** Confirm `/imu/euler.z` increases monotonically under CCW rotation and verify `imu_yaw_offset_deg` aligns magnetic/game-tare zero to ENU East against GPS motion before autonomous runs.
