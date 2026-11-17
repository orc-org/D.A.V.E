# Known issues, limitations and open questions

Three groups: what was fixed while consolidating the repository, what is still open and needs someone with the hardware, and where the code is incomplete by design. A final section sketches the missing perception layer (SLAM).

## 1. Fixed while consolidating (local changes, not pushed)

| Area | Problem | Fix |
|---|---|---|
| Waypoint maths | Home latitude/longitude are degrees, but the ECEF↔local rotation used them as radians; every local ↔ ECEF/geodetic conversion was wrong for real GPS positions | Convert to radians inside the rotation; verified by `scripts/tests/waypoint_conversion_test.cpp` |
| Waypoint maths | `local_To_EarthCentred` / `local_To_GeodeticApprox` took `int` arguments (truncating metres) and returned radians | `double` arguments; geodetic result in degrees |
| `NAVnode` | NMEA latitude/longitude parsed with `stoi` (drops decimals) and never converted from `ddmm.mmmm` | `stod` plus degrees-and-minutes conversion |
| `NAVnode` | Position update required `longitude > 0`, impossible in the western hemisphere, so the position never updated | Test for non-zero |
| `NAVnode` | Geodetic `Waypoint` constructor takes (longitude, latitude); two call sites passed (latitude, longitude) | Argument order fixed |
| `NAVnode` | Bearing to waypoint used `asin(dy/dx)`, NaN whenever \|dy\| > \|dx\| | `atan2(dx, dy)` normalised to [0, 2π) |
| `NAVnode` | Debug print showed latitude twice | Prints longitude |
| `NAVnode` | Two services registered as `SelectRoute`, so the second never worked; `NAVDisplay` called `SelectRouteToFollow` which did not exist | Services are `SelectRouteToFollow` and `SelectRouteToEdit` |
| `NAVnode`/`NAVDisplay` | `NAVnode` published `NAVDisplayInfo` but `NAVDisplay` subscribed to `DisplayInfo`, so the map never received data | Display subscribes to `NAVDisplayInfo` |
| `NAVnode` | Collision-handling bugs from `navnode_fix`: array freed with `delete`, null dereferences, divide-by-zero, wrong vector in a detour offset, `previous` vs `nextNode` mix-up; remove-waypoint services edited the wrong route; "last five waypoints" aliased one object | Ported from `navnode_fix` and applied |
| `ListOfWaypoints` | `removePoint` returned `false` inside its loop, so it only ever examined the first waypoint (and had no return at all for an empty list). `RemoveLocalWaypoint` could therefore only remove the head of a route | Return moved after the loop |
| `NAVnode` | The GNSS date was built from `substr(++start, 2) + … substr(start + 2, 2) + …`, whose operands are evaluated in unspecified order | Increment separated from the expression |
| `navigation_system` build | `NAVDisplay` listed `raylib` in `ament_target_dependencies`, which turned raylib's include flag into a bogus `-D-I…` compiler definition | Removed; raylib is linked as a CMake target |
| `serial_communication` | `SerialPort::write(char*)` passed `sizeof(message)` (the pointer size, 8 bytes) as the length, so every serial message (XLR link, Morse bridge) was truncated to 8 bytes or padded with whatever followed it in memory. Found by the compiler warning in the first WSL2 build | `strlen(message)`, with a null check and error report |
| `rover_ekf` | Wheel speed was twice the four-wheel mean | `× 0.25` |
| `rover_ekf` | Yaw-rate update fed the state's own heading back as a measurement (zero innovation, shrinking covariance) | Updates yaw rate only |
| IMU launch | Filter used `dt = 0.1` while telemetry publishes at 20 Hz | Launch sets `dt = 1 / publish_rate` |
| `kinematics.py` | `matplotlib` imported at module load, so the arm node failed where it is not installed | Imported lazily inside `plot_chain` |
| `arm_controller` | `package.xml` missed `python3-yaml` and `rover_description` (config lookup at start-up) | Dependencies added |
| Packages | `TODO` licences and descriptions in seven packages | Filled in (licence MIT, matching the root `LICENSE`) |
| `serial_communication` | Two identical copies of `SerialPort.hpp` | One copy in `include/serial_communication/` |
| `drive_node` | Brought in `/control_mode` and `/sensitivity` handling from the web branches; the original branch defaulted to 0.5 power | Default 1.0 (full power) so behaviour is unchanged until something publishes `/sensitivity` |
| Repo | Shell scripts checked out with CRLF line endings would not run on Linux | `.gitattributes` forces LF for scripts and source |
| README | Out-of-date pinout and a typo in the Morse command | Rewritten to match the code |
| `receive_stream.sh` | `-p` as the last argument (or a non-number) made `shift 2` fail, so the option loop spun forever | Argument checked, exits with an error |
| `video_streamer.py` | `--autofocus-on-start` never worked: the focus motor is initialised lazily on the first frame, but the start-up trigger checked `has_focus_motor` before that | `trigger_sweep` initialises the motor first |
| `test_dm556y_steppers.py` | Default frequency 200000 Hz while the comment and RPM figure say 20 kHz (10× too fast for a bring-up test) | 20000 Hz; pin-number comments no longer contradict the constants |
| `test_gps.py` | Polled `in_waiting` in a loop with no sleep (100 % of a core) | Blocks in `readline()` |
| `test_imu_bus.py` | Said "Bus 7" regardless of what it opened and only proved the chip answered | Reads five samples; message no longer claims a bus |
| `reset_arm_stepper_gpios.py` | Claimed the pins were "forced LOW" at the end, but `GPIO.cleanup()` releases them (floating) | Message says what it does |
| `launch_cam0_60fps.sh` | Hard-coded `/home/orc/D.A.V.E./…` path and IP | Path relative to the script; `HOST`/`PORT` environment overrides; made executable |
| `start_dashboard_system.sh` | Unquoted `pkill -f python.*http.server` (the shell could expand the glob) | Quoted |
| `bench_tegrastats.sh` | A non-numeric duration crashed with a shell error | Validated |
| `navnode_nmea_smoke_test.sh` | Fixed `sleep`s; NAVnode could outlive the test (SIGTERM to the `ros2 run` wrapper); track log written into whatever directory the test was started from | Polls for readiness, SIGINT plus a leftover check, runs in a temp directory; `captainsLogs/` git-ignored |
| Failure points (effectiveness pass) | `record_bag.sh` could fill the disk, record nothing without a warning, and lose the whole bag on a crash | Refuses below `MIN_FREE_GB`, warns about unpublished topics, splits at 4 GB |
| | `bench_tegrastats.sh` left an empty log if `tegrastats` lacked permission | Checks the process is alive after 2 s and warns on an empty log |
| | `start_dashboard_system.sh` / `start_sim_system.sh` printed success even if a server failed to bind or died | Port-in-use check (dashboard) and a 3 s liveness check that prints the log tail |
| | `receive_stream.sh` built its pipeline as a string and ran it through `eval` | Argument array, checks `gst-launch-1.0` exists |
| | `video_streamer.py` rejected `--ros-args`; two simultaneous focus triggers could start two sweeps | `parse_known_args`; the in-progress flag is claimed under a lock |
| | `test_dm556y_steppers.py` accepted any frequency | Refuses above 25 kHz unless `--force` |
| Code review ([code-review.md](code-review.md)) | `NAVnode`: a buffer containing an RMC sentence was discarded (the parser stepped onto a comma and `stod` threw), so speed, heading and date were never set and receivers that send RMC lost their position too | Field-based RMC/GGA/VTG parsing with hemisphere handling; position only updates on a valid fix |
| | `NAVnode`: `nextWaypoint` was set once in the constructor, while every route was empty, so no waypoint added later was ever targeted; removing a waypoint left it dangling | `validateTarget()` each tick and in the manual target services; route/direction changes restart targeting; the end of a route is remembered |
| | `NAVnode`: serial reads are not line-buffered (≤ 256 bytes per tick), so sentences split between ticks were lost, and speed/heading reset to 0 on any tick without RMC | Bytes are kept across ticks; speed and heading hold until the next RMC/VTG |
| | `NAVnode`: the track log stored `currentPosition` itself, which is deleted every tick and again by the list node (dangling pointers, double free at shutdown); the log printed `currentPosition` on every line | Stores a copy; prints each logged point |
| | `NAVnode`: `removeLastWaypoint` checked the followed route but removed from the edited one; `SelectRouteToFollow`/`SelectRouteToEdit` never set their success message (unreachable code after `default`) | Checks the edited route; message moved out of the switch |
| | `rover_ekf`: the IMU update put the measured yaw into a 3-row measurement against the full state, comparing roll/pitch with x/y | Yaw-only update (1×5 `H`) |
| | `rover_ekf`: no way to run without GNSS (CIRC Meteorite task bans it) | `use_gps` parameter; when false, `/gps/fix` is not subscribed and a warning says position is dead reckoning |
| | `navnode_nmea_smoke_test.sh` fed only GGA, which hid the RMC bug, and passed even when no position was parsed | Feeds RMC + GGA; checks a position was parsed, the RMC speed, and that an added waypoint is targeted |
| | `vesc_can_driver_node` held the last command forever (no deadman), so a dropped link left the rover driving | `command_timeout` (0.5 s) stops a stale wheel immediately; per-message INFO logs in both drive nodes lowered |
| | IMU `telemetry.py` fell back to synthetic data silently when the BNO085 or its libraries were missing, and repeated the last frame forever if the sensor died | Synthetic data only with `sim_mode:=true`; otherwise nothing is published, an error is logged and the connection is retried |
| | `gps_bridge_node` labelled every fix `STATUS_FIX` with no covariance, passed on dead-reckoning (quality 6) fixes, published each epoch twice and was silent without a fix | Status from GGA quality, HDOP-based covariance (used by `rover_ekf`), explicit `STATUS_NO_FIX`, RMC only as a fallback ([navigation.md](navigation.md#gps_bridge_node-fix-status-and-covariance)) |
| Base review (2026-11-15) | `motion_coordinator` could drive the rover in auto mode on a made-up base pose, published a second TF parent for `base_link`, and failed to start without matplotlib | Arm-only auto mode unless `allow_base_motion:=true`; EKF pose and `world → odom` when `/ekf/odometry` is fresh; lazy matplotlib import |
| | `rover_ekf`: fixed `dt`, the same command fused twice, zero speed skipped, IMU yaw (north tare) used as ENU, origin always the first fix | Measured `dt`; `/motor/*` once, zero included; `imu_yaw_offset_deg`; `datum_*` parameters |
| | IMU AUKF: orientation never corrected, gravity removed along body z regardless of tilt, drifting `/imu/odometry` on by default | Fuses the BNO085 quaternion; tilt-aware gravity; odometry opt-in |
| | `NAVnode`: the add services set home silently (even without a fix); `FollowRoute` ran in a detached thread reading freed memory | Refused without a fix, warned with one; goal tracked by the timer |
| | No wheel speed feedback: the EKF fused motor commands as if measured | `vesc_can_driver_node` reads VESC CAN STATUS frames → `/wheel_states`; `rover_ekf` prefers it (enable CAN status on the VESCs) |
| Docs | `slam/architecture.md` gave the Unitree driver's frames as `lidar_link`/`lidar_imu_link` (those are the canonical names); the SDK uses `unilidar_lidar`/`unilidar_imu` | Corrected |

## 2. Open — needs hardware verification, measurement, or autonomy development

1. **Physical wheel radius (`0.10 m` vs `0.15 m`).** `vesc_can_driver_node` and `rover_ekf` still default to `0.10 m` for hardware, while `physical_params.yaml`, URDF, and `sim.launch.py` use `0.15 m`. Measure the real tire before driving outdoors and update whichever disagrees.
2. **Drive hardware bench check (R2/L2, brake hold, ERPM clamp).** Software now defaults to `right_trigger_forward:=true` (R2 forward, matching `dashboard/app.js`), guards `axes.size() < 6`, aligns SI `/cmd_vel`, clamps metric ERPM, and sends `CAN_PACKET_SET_CURRENT_BRAKE` (`5.0 A`) on timeout/zero; **still untested on elevated physical wheels with SocketCAN VESCs**.
3. **IMU hardware check (`imu_yaw_offset_deg` & magnetometer).** `telemetry.py` now defaults to 6-DoF `BNO_REPORT_GAME_ROTATION_VECTOR` (`use_magnetometer:=false`) with strict `read_failures` counting, and `imu.launch.py` disables the redundant AUKF (`run_kalman_filter:=false`); still confirm on the physical BNO085 that yaw increases on CCW turn and verify tare alignment against GPS outdoors.
4. **Planar EKF limitation & 3-D slope navigation.** Software fixes (ROS clock sync, Joseph covariance form, predict-before-update, L/R wheel pairing, WGS-84 offsets, `publish_tf` param) are verified in unit tests and Gazebo, **but** `rover_ekf` remains a planar 5-state `[X, Y, θ, v, ω]` filter (not full 3-D `robot_localization` or LiDAR-fused `SE(3)`).
5. **Hardware pin & serial port collisions.** Pin 37 is absent from `dave_custom_pins.dts`, Pin 15 is unused in the overlay, Pin 32 is claimed by both `gripper_step_pin` and `jetson_morse.cpp`, `test_dm556y_steppers.py` uses pins `15/13` vs node defaults `33/18`, and `gps_bridge_node` / serial-mode `NAVnode` / `morse_bridge_node` default to `/dev/ttyUSB0` (avoidable on GNSS by setting `use_gps_fix_topic:=true` on `NAVnode` or assigning `udev` rules).
6. **Arm open-loop step rate & hardware.** Software unit/clock/blocking bugs (`current_gripper_pos`, `1.0 - step/max` direction, `MultiThreadedExecutor` + cancelable 15s action loop, daemon-thread Arduino reconnect, `*_invert_dir` separation, `/gripper_state` step normalization in `motion_coordinator`) are fixed; physical Jetson `100 kHz` GPIO step-rate floor and open-loop lack of limit switches/encoders remain open on hardware.
7. **Battery gauge & dashboard linting.** The dashboard subscribes to `/battery/status`, which has no live publisher; offline competition tiles must be populated into `dashboard/tiles/`; `dashboard/app.js` has not been CI-linted (no Node.js in WSL env); rosbridge is unauthenticated.
8. **Script findings left open** (need hardware verification or design choices):
   - `test_dm556y_steppers.py`: confirm physical pin wiring (`15/13` vs `33/18`, `32/13`) before powering steppers.
   - `NAVnode`: still writes `captains log <date>.txt` into `src/navigation_system/captainsLogs` relative to CWD.
   - `morse_recorder.py`: assumes first pulse is a dot, uses wall clock `time.time()`, and calls `cv2.imshow` without mutex protection from the capture thread.
   - `video_streamer.py`: `--width`/`--height` remain strings; `receive_stream.bat` exits `0` on unknown option.
   - `start_sim_system.sh` / `start_dashboard_system.sh`: write logs to fixed `/tmp` paths and do not stop `rosapi`.
   - `pyflakes` unused imports/variables in legacy scripts do not affect behavior.
9. **Gazebo & Unitree L2 hardware gaps:**
   - Gazebo simulates flat-yard planar driving only (no rough terrain slip, no rolling helical L2 scan timing — `lidar_point_time.py` uses `1e-6 s`), and `rover_arm.xacro` stays disabled until its CAD offset vs `lidar_link` (`x=0, z=0.40`) is resolved.
   - Unitree L2 physical bring-up (`unilidar_sdk2`, `192.168.1.2` vs DDS subnet, Jetson `tegrastats`, open uneven terrain Z-drift) remains untested on hardware.
10. **Removed as deprecated by the cleanup branch:** `GPSnode.cpp`, `vesc_driver_node.cpp`, `stepper_driver_node.cpp`, `py_node.py`, `module_to_import.py`, `battery_monitor_node.py`, and binary artifacts (`jetson_morse`, `.dtbo`). They remain in git history if needed.

## 3. Incomplete by design

- **The navigation system cannot drive the rover.** `NAVnode` computes range and bearing to the next waypoint and advances through the route, but nothing converts that into `cmd_vel`. The `FollowRoute` action restarts the followed route and reports progress until it is finished, but does not drive. An operator, or new code, has to close the loop.
- **Obstacles are entered by hand.** There is no sensing of the environment (no lidar, depth camera or stereo), so avoidance only works against circles an operator typed in.
- **Position is GNSS-only for navigation.** `NAVnode` does not use the EKF; with no GNSS fix it has no position.
- **Detours are geometry, not planning.** See [navigation.md](navigation.md).
- **The IMU filter's position is not usable** (body-frame double integration), so `/imu/odometry` is off by default; its orientation now follows the BNO085. See [state-estimation.md](state-estimation.md).
- **Arm control is open loop.** Auto mode runs inverse kinematics against the EKF pose when `/ekf/odometry` is fresh (otherwise a pose integrated from wheel commands, for visualisation), and moves the arm only unless `allow_base_motion:=true`; the stepper position is open loop. See [arm-and-kinematics.md](arm-and-kinematics.md).
- **`move_to_point` is a stub** that reports success without waiting for the motion.
- **Tests.** Core packages carry behavioral GTest/pytest suites (`test_drive_math`, `test_nav_math`, `test_ekf_math`, `test_imu_math`, `test_arm_math`, `scripts/slam/test_eval_trajectory.py`) alongside `waypoint_conversion_test.cpp` and `navnode_nmea_smoke_test.sh`. Hardware drivers (CAN, I²C, UART, GPIO) and `dashboard/app.js` remain unverified by CI/physical bench tests.

## 4. The missing layer: perception and SLAM

What the rover has today: two CSI cameras and a USB camera (streamed, not processed except the Morse flash detector), a BNO085 IMU, a GNSS receiver, and open-loop wheel commands (no encoders in the software). What it lacks for autonomous navigation is a consistent map and a pose that does not depend on GNSS alone.

Existing hooks a SLAM or mapping node can plug into:

| Need | Existing interface |
|---|---|
| Better pose | Publish `nav_msgs/Odometry` like `/ekf/odometry`, plus TF `odom → base_link`; or feed `rover_ekf` an extra measurement |
| Obstacles | Call `AddLocalObstacle` / `AddGeodeticObstacle` (circles) — a crude bridge from a map to `NAVnode` |
| Driving | Publish `/cmd_vel` (the dashboard already does), or write `/motor/*` directly |
| Camera data | `video_streamer` can relay an Image topic and the sources are already on the Jetson |

Directions worth considering, roughly in order of risk:

1. **GNSS-aided factor-graph odometry** (for example GTSAM): fuse IMU preintegration, wheel speed and GNSS in one smoother. Smallest hardware change, big gain over the two disconnected filters.
2. **Visual-inertial SLAM** on the existing cameras (ORB-SLAM3, OpenVINS, or Isaac ROS Visual SLAM, which is accelerated on Jetson). Needs a calibrated, time-synchronised camera–IMU pair; stereo or depth gives metric scale.
3. **Terrain-aware mapping** (2.5-D elevation map with traversability cost) feeding a real planner such as Nav2 with a custom costmap layer. Matters for rough ground where flat 2-D maps fail.
4. **LiDAR-inertial SLAM** (FAST-LIO2 / LIO-SAM) if a lidar can be added. Most robust, but new hardware.

Update: a sensor choice has been made (Jetson Orin Nano Super with a Unitree 4D LiDAR), and the options above have been researched in detail. See [slam/README.md](slam/README.md) for the platform notes, literature review, proposed three-layer design and experiment plan. Nothing from that design is implemented yet.
