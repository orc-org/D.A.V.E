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

## 2. Open — needs measurement or a team decision

1. **Trigger direction.** The ported `drive_node` makes the *left* trigger forward and the *right* trigger backward (the earlier firmware had them the other way). Confirm with whoever drives before the next field test.
2. **Wheel radius.** `vesc_can_driver_node` defaults to 0.10 m but `physical_params.yaml` (and so the URDF/simulation) says 0.15 m. Commanded ground speed is off by 1.5× if the wrong one is used. Measure the real wheel.
3. **Speed ceiling.** In metric mode the ERPM is not clamped to `max_erpm` (7000). A full-stick command (1.0 m/s) asks for ≈ 9549 ERPM. Decide whether to clamp.
4. **Yaw convention.** `rover_ekf` assumes heading is counter-clockwise from east; the IMU tare sets "heading 0 = north". Verify which holds on the rover (see [state-estimation.md](state-estimation.md)).
5. **Coordinate origins.** The EKF's origin is its first GPS fix; `NAVnode`'s is the home waypoint. They are not the same frame, and nothing reconciles them.
6. **TF.** `motion_coordinator` and `rover_ekf` both publish a parent for `base_link` (`world` and `odom`). Do not run them together, or pick one.
7. **Pin map.** Pin 37 (servo 2) is not in the device-tree overlay; pin 15 is in the overlay but no longer used; pin 32 is claimed by both the gripper STEP and the Morse servo test. Confirm the wiring (see [hardware.md](hardware.md)).
8. **Arm step rate.** The step frequency is floored at 10 kHz, which makes the arm move at ≈ 0.785 rad/s rather than the nominal 0.5 rad/s unless the Arduino firmware limits it.
9. **Battery gauge.** The dashboard subscribes to `/battery/status`, but `battery_monitor_node` (a BLE BMS reader that needed an uninstalled `battery` module) was deleted as deprecated in the cleanup. Either restore it with its dependency or remove the gauge.
10. **Shared serial ports.** `gps_bridge_node`, `NAVnode` and `morse_bridge_node` all default to `/dev/ttyUSB0`. Assign stable device names.
11. **Licences.** The repository is MIT. Several packages still declare Apache-2.0 and many maintainer emails are placeholders (`@todo.todo`, `@example.com`). Decide on one licence.
12. **Dashboard needs the internet** (CDN scripts, map tiles) and rosbridge is unauthenticated.
13. **Removed as deprecated by the cleanup branch:** `GPSnode.cpp` (an older 840-line GPS node, superseded by `gps_bridge_node.py` and the serial code in `NAVnode`), `vesc_driver_node.cpp` (serial VESC driver), `stepper_driver_node.cpp`, `py_node.py`, `module_to_import.py`, `battery_monitor_node.py`, and the committed `jetson_morse` binary and `.dtbo` files. They are in git history if needed.

## 3. Incomplete by design

- **The navigation system cannot drive the rover.** `NAVnode` computes range and bearing to the next waypoint and advances through the route, but nothing converts that into `cmd_vel`. The `FollowRoute` action accepts any goal and does nothing. An operator, or new code, has to close the loop.
- **Obstacles are entered by hand.** There is no sensing of the environment (no lidar, depth camera or stereo), so avoidance only works against circles an operator typed in.
- **Position is GNSS-only for navigation.** `NAVnode` does not use the EKF; with no GNSS fix it has no position.
- **Detours are geometry, not planning.** See [navigation.md](navigation.md).
- **The IMU filter's orientation is never corrected.** See [state-estimation.md](state-estimation.md).
- **Arm control is simulated.** Auto mode runs inverse kinematics against a base pose integrated from wheel commands, not a measured pose; the stepper position is open loop. See [arm-and-kinematics.md](arm-and-kinematics.md).
- **`move_to_point` and `FollowRoute` are stubs** that report success without waiting for the motion.
- **Tests.** There are lint tests only, plus the new coordinate-conversion test. Nothing exercises the C++ navigation, the EKF or the arm.

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
