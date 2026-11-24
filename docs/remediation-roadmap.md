# Remediation Roadmap (Remaining Work & Verified Software Progress)

This roadmap tracks what has been fixed and verified in software/simulation versus what remains required for **physical hardware commissioning** and **autonomous field navigation**.

---

## Priority 0 — Physical Safety & Hardware Blockers (Before Field Power-On)

1. ~~**Active Braking on VESC Deadman & Stop (`vesc_can_driver_node.cpp`)**~~ — **[Software Fixed & GTest Verified; Verify on VESCs]**
   * *Implemented:* `CAN_PACKET_SET_CURRENT_BRAKE` (`comm_id = 2`, default `brake_current = 5.0 A`, `brake_on_zero = true`) on deadman/zero stop; `[-max_erpm, max_erpm]` clamping in metric mode; independent `/control_mode` lockout inside `vesc_can_driver_node`.
2. **Measure Physical Wheel Radius & Resolve Package Coupling (`vesc_can_driver_node.cpp`, `physical_params.yaml`, `ekf_node.py`)** — **[Software Parameterized; Hardware Measurement Open]**
   * *Implemented:* Added `axes.size() >= 6` safety fallback, unified `right_trigger_forward = true` (R2 forward, matching `dashboard/app.js`), consistent CCW yaw in reverse, and loaded `wheel_radius` (`0.15 m`) from `physical_params.yaml` in both `vesc_can_driver_node.cpp` and `rover_ekf/ekf_node.py`.
   * *Remaining Hardware / Build Action:* Measure actual tire radius outdoors (`0.15 m` vs `0.10 m`), verify elevated wheels spin forward on R2, and resolve `drive_package`'s undeclared runtime dependency on `rover_description` (`physical_params.yaml`).
3. ~~**Unify `/cmd_vel` Units (`m/s` + `rad/s`) Everywhere**~~ — **[Done & Gazebo Verified]**
   * *Implemented:* Standardized SI `(m/s, rad/s)` across `dashboard/app.js`, `motion_coordinator_node.py`, `drive_node.cpp` (`W = 0.60 m`, `max_wheel_speed = 1.0 m/s`), and `sim_drive_bridge.py`.
4. **Resolve Pin Clashes, Hardware Pull-Downs & Udev Rules** — **[Open — Hardware Bench Task]**
   * *Remaining Hardware Action:* Reconcile Jetson Pin 32 (`gripper_step_pin` vs `jetson_morse.cpp`), verify Pin 37 (`servo2_right_pin`) in `dave_custom_pins.dts`, reconcile `test_dm556y_steppers.py` pins (`15/13`) against `arm_stepper_driver_node.py` (`33/18`, `32/13`), add external hardware pull-downs on DM556Y/Arduino inputs, and create `/etc/udev/rules.d/` symlinks (`/dev/rover_gnss`, `/dev/rover_arm`, `/dev/rover_morse`).

---

## Priority 1 — State Estimation & Control Integrity

5. ~~**Disable Default `IMU/kalman_filter.py` & Switch BNO085 to Game Rotation Vector (`telemetry.py`)**~~ — **[Done in Software; Verify on BNO085]**
   * *Implemented:* `run_kalman_filter` defaults to `'false'` in `imu.launch.py` (with $R_{nb}$ world rotation, ZYX Euler rate matrix, and stamp `dt` fixed if run standalone); `telemetry.py` defaults to `use_magnetometer = False` (`BNO_REPORT_GAME_ROTATION_VECTOR`) and resets `read_failures` strictly on valid finite packets.
6. **Clean State Estimation Upgrade (`rover_ekf` Hardened; `robot_localization` / 3-D ESKF Future Upgrade)** — **[Hardened in Software & Gazebo; 3-D Slope Upgrade Open]**
   * *Implemented:* `ekf_node.py` updated to ROS clock (`use_sim_time` compatible), Joseph covariance form, predict-before-wheel-update, balanced L/R `/wheel_states` pairing, WGS-84 tangent-plane offsets, and `publish_tf` parameter.
   * *Remaining Upgrade:* Adopt standard 3-D `robot_localization` (`ekf_localization_node` + `navsat_transform_node`) or GTSAM factor-graph smoothing with FAST-LIO2 odometry (`E2`) for steep slopes (`> 10°`).
7. ~~**Fix `arm_controller` Unit Clashes & Executor Blocking**~~ — **[Software Fixed & Pytest Verified; Open-Loop HW Untested]**
   * *Implemented:* `time.monotonic()` everywhere; `/gripper_state` normalized cleanly inside `motion_coordinator_node.py`; SI wheel fallback (`0.60 m`, `1.0 m/s`); `MultiThreadedExecutor` + `15 s` cancelable action server + initialized `current_gripper_pos` in both stepper nodes; background daemon reconnect and separated logical vs `*_invert_dir` polarity in Arduino driver.
   * *Remaining Hardware Action:* Confirm step-rate behavior on Jetson GPIO (`100 kHz` floor vs `200 kHz` param) and Arduino `USE_GPIO_TRIGGER` setting; add physical limit switches/encoders later if closed-loop arm control is desired.

---

## Priority 2 — Navigation & Autonomy Loop Closure

8. ~~**Harden `NAVnode`, `NAVDisplay`, `gps_bridge_node`, `SerialPort`, `EnigmaMachine`**~~ — **[Done & Verified in GTest / Smoke Test]**
   * *Implemented:* `nav_nmea.hpp` `*hh` XOR checksum verification + GGA `{1,2,4,5}` quality gating (`verify_nmea_checksum`), `use_gps_fix_topic` `/gps/fix` subscription, 16-read serial draining, bounded `findAlternateRoute`, `double` obstacle helpers, `~NAVnode()` cleanup, `ResetHome` rebase (`Obstacle::rebaseToHome`), headless `NAVnode` CMake build (`raylib` optional), `NAVDisplay` `state_mutex_` + live `currentPosition`, `gps_bridge_node` 2 s serial reconnect, `SerialPort` zero-leak `write`, `XLR_User_input` background stdin thread, and `EnigmaMachine` initialized `TrieNode`.
9. **Close the Autonomy Loop (`NAVnode` / Layer 2 → Nav2 or Regulated Pure Pursuit → `/cmd_vel`)** — **[Open]**
   * Implement a path-following controller consuming `/ekf/odometry` (or LIO-fused pose) + `NAVnode` targets/waypoints and publishing SI `/cmd_vel`, alongside an independent LiDAR forward-corridor safety stop (`slam/architecture.md`), CIRC Reactor Recovery polygon geofencing, and `battery_monitor_node` (or remove dead `/battery/status` dashboard UI).

---

## Priority 3 — Simulation Fidelity

10. **Simulation Upgrades (`rover_description`)** — **[Full Sensor/EKF/NAV Bridge Done; Terrain & Arm Extrinsic Open]**
    * *Implemented:* `sim_sensor_bridge.py` (`use_sim_stack:=true` in `sim.launch.py`) now drives `/wheel_states`, `/imu/euler`, `/imu/data_raw`, `/gps/fix`, `rover_ekf` (`publish_tf:=false`), and `NAVnode` (`use_gps_fix_topic:=true`) cleanly end to end in Gazebo.
    * *Remaining Simulation Work:* Resolve `rover_arm.xacro` CAD mount offsets against `lidar_link` (`x=0, z=0.40`) and re-enable it for LiDAR self-occlusion testing; add rough/sloped outdoor terrain SDF worlds and rolling-sweep L2 scan timing generation.

---

## Priority 4 — SLAM Evaluation & Hardware Bring-Up Gate

11. ~~**Fix `run_bag.sh` & `eval_trajectory.py`**~~ — **[Done & Verified against `e1b_route` + `pytest`]**
    * *Implemented:* `run_bag.sh` signals `$FLPID` on exit and guards against empty output; `eval_trajectory.py` computes ATE rotation error, SE(3) 10 m RPE (translation + deg), vector loop closure residual, aligned Z drift, and clear `--est-clock` overlap errors.
12. **Hardware Bring-Up Gate (`E0` / `E1`)** — **[Open — Needs Jetson Orin Nano + Unitree L2]**
    * Flash Jetson Orin Nano Super, build `unilidar_sdk2` for Humble on a dedicated NIC/subnet, measure real L2 scan rate (`~5.55 Hz`), IMU rate, CPU/RAM via `bench_tegrastats.sh`, check FastDDS hybrid multicast (`239.255.0.1`) bandwidth on the antenna link, and run FAST-LIO2 vs Point-LIO/KISS-ICP on real outdoor uneven-terrain bags + stationary 10-minute GNSS UERE calibration.
13. **Expand Frontend CI Coverage** — **[Backend GTest/Pytest Done; JS Lint Open]**
    * Add Node/ESLint syntax/lint check for `dashboard/app.js` in CI and populate offline competition map tiles into `dashboard/tiles/`.
