# Architecture

## Layers

```
 Operator (browser dashboard, PS5 controller, NAVDisplay map)
        │  rosbridge websocket :9090 / HTTP :8000 / DDS
        ▼
 ┌───────────── ROS 2 graph on the Jetson ─────────────┐
 │  control   drive_node, motion_coordinator            │
 │  actuation vesc_can_driver_node, arm_stepper_*,      │
 │            lights, servo_node, morse_bridge_node     │
 │  sensing   telemetry (IMU), gps_bridge_node, XLRnode │
 │  estimation kalman_filter (IMU UKF), ekf_node        │
 │  navigation NAVnode (waypoints/obstacles/bearing)    │
 │  video     video_streamer (GStreamer → UDP)          │
 └──────────────────────────────────────────────────────┘
        │  CAN0 · I²C bus 7 · USB serial · GPIO · CSI/V4L2
        ▼
 VESC motor controllers · BNO085 IMU · GNSS · DM556Y steppers · lights · servos · cameras
```

## Packages (`ros2_ws/src`)

| Package | Language | Role |
|---|---|---|
| `drive_package` | C++ | `drive_node` (joystick/`cmd_vel` → per-wheel command), `vesc_can_driver_node` (wheel command → VESC CAN frames) |
| `arm_controller` | Python | `motion_coordinator` (kinematics + IK + simulated pose), `arm_stepper_driver` (direct GPIO), `arm_stepper_arduino_driver` (serial to Arduino), `dashboard_helper` (process manager), `kinematics.py` |
| `arm_interfaces` | IDL | `ArmStatus` msg, `CalibrateArm` srv, `MoveToPoint` and `GripperCommand` actions |
| `IMU` | Python | `telemetry` (BNO085 reader/simulator), `kalman_filter` (15-state unscented Kalman filter) |
| `rover_ekf` | Python | 5-state EKF fusing wheel speed, IMU and GPS; publishes `odom → base_link` |
| `navigation_system` | C++ + Python | `NAVnode` (routes, obstacles, bearing), `NAVDisplay` (raylib map), `gps_bridge_node.py` (NMEA → `NavSatFix`) |
| `navigation_interfaces` | IDL | ~25 services, `DisplayInfo` msg, `NavigationAction` |
| `serial_communication` / `serial_interfaces` | C++ / IDL | `SerialPort` class, `XLRnode` (+`XLR_UI`), `morse_bridge_node`, serial service definitions |
| `enigma_machine` / `enigma_machine_interfaces` | C++/Python / IDL | Morse recorder (video → Morse) and Enigma encode/decode actions |
| `rover_video_streamer` | Python | GStreamer H.264/MJPEG camera streaming, autofocus |
| `rover_description` | xacro/URDF | Robot model, Gazebo simulation, `physical_params.yaml` |
| `light_package` | Python | `/lights` (Bool) → GPIO pin 12 |
| `servo_controller` | Python | `servo_commands` (String) → two bidirectional servo outputs (camera pan/tilt) |
| `input_viewer_package` | Python | `/joy` → RViz markers |

Outside the workspace: `dashboard/` (web UI), `fastdds_profiles.xml`, device-tree overlays (`*.dts`), `scripts/` (hardware tests, utilities), `start_dashboard_system.sh`, `start_sim_system.sh`.

## Data flow

```
/joy ──► drive_node ──► /motor/{front_left,front_right,rear_left,rear_right} ──► vesc_can_driver_node ──► CAN0 ──► VESCs
            ▲  ▲                         │                  │                                         │
 /cmd_vel ──┘  └ /control_mode,/sensitivity                 ├──► rover_ekf (/wheel_states preferred) ◄┘
 (motion_coordinator / dashboard)                           └──► motion_coordinator (visual odometry fallback)

BNO085 ─► telemetry ─► /imu/data_raw, /imu/euler ─┬─► kalman_filter (opt-in: run_kalman_filter:=true) ─► /imu/data_filtered, /imu/odometry (off by default)
                                                  └─► rover_ekf ◄─ /gps/fix ◄─ gps_bridge_node ◄─ GNSS (USB serial)
                                                          └─► /ekf/odometry, /ekf/filtered_euler, TF odom→base_link (HW)

GNSS (serial or /gps/fix when use_gps_fix_topic:=true) ─► NAVnode (5 Hz) ─► NAVDisplayInfo ─► NAVDisplay (optional raylib)

Gazebo sim.launch.py (use_sim_stack:=true) ─► sim_sensor_bridge ─► /wheel_states, /imu/data_raw, /imu/euler, /gps/fix ─► rover_ekf (publish_tf:=false) + NAVnode
```

> Planned perception layers (LiDAR-inertial odometry, planner map, Gaussian terrain map) are described in [slam/architecture.md](slam/architecture.md). They are not part of the graph below yet.

## Topic, service and action map

Topics (publisher → subscribers):

| Topic | Type | Publisher | Subscribers |
|---|---|---|---|
| `/joy` | `sensor_msgs/Joy` | joy driver (external) | `drive_node`, `input_viewer` |
| `/control_mode` | `std_msgs/String` (`"drive"`/other) | none in the current dashboard | `drive_node`, `vesc_can_driver_node` (`enable_control_mode_lock:=true`, brakes if not `"drive"`) |
| `/sensitivity` | `std_msgs/String` (`ultra`/`fine`/`coarse`) | none in the current dashboard | `drive_node` (scales output ×0.25/0.5/1.0; default 1.0) |
| `/cmd_vel` | `geometry_msgs/Twist` (`m/s`, `rad/s`) | `dashboard/app.js`, `motion_coordinator` (auto mode) | `drive_node`, `rover_ekf` (`fuse_cmd_vel:=false`), `sim_drive_bridge` |
| `/motor/{front_left,front_right,rear_left,rear_right}` | `Float32` | `drive_node` | `vesc_can_driver_node`, `rover_ekf` (fallback), `motion_coordinator`, `sim_drive_bridge` |
| `/wheel_states` | `sensor_msgs/JointState` | `vesc_can_driver_node`, `sim_sensor_bridge` | `rover_ekf` (preferred over commands; balanced L/R pairs) |
| `/imu/data_raw` | `sensor_msgs/Imu` | `telemetry` (6-DoF Game RV default), `sim_sensor_bridge` | `kalman_filter` (opt-in), `rover_ekf` |
| `/imu/euler` | `geometry_msgs/Vector3` (degrees) | `telemetry`, `sim_sensor_bridge` | `rover_ekf` |
| `/imu/data_filtered`, `/imu/odometry`, `/imu/filtered_euler` | `Imu`, `Odometry`, `Vector3` | `kalman_filter` (off by default) | dashboard (`/imu/odometry` off unless `publish_odometry:=true`) |
| `/gps/fix`, `/gps/nmea_raw` | `NavSatFix`, `String` | `gps_bridge_node`, `sim_sensor_bridge` | `rover_ekf`, `NAVnode` (`use_gps_fix_topic:=true`), dashboard |
| `/ekf/odometry`, `/ekf/filtered_euler` | `Odometry`, `Vector3` | `rover_ekf` | `motion_coordinator`, dashboard |
| `/lidar/points`, `/lidar/imu`, `/camera/*`, `/odom`, `/cmd_vel_sim`, `/sim/ground_truth_poses` | `PointCloud2`, `Imu`, `Image`/`CameraInfo`, `Odometry`, `Twist`, `TFMessage` | `sim.launch.py` (`ros_gz_bridge`) / `sim_drive_bridge` | FAST-LIO2 / SLAM tools / `sim_sensor_bridge` (Gazebo simulation only) |
| `arm_target` | `geometry_msgs/Point` | dashboard | `motion_coordinator` |
| `arm_status`, `arm_global_pos` | `ArmStatus`, `Point` | `motion_coordinator` | dashboard |
| `joint_states` | `sensor_msgs/JointState` | `motion_coordinator` (and Gazebo `JointStatePublisher`) | arm steppers, `robot_state_publisher`, `sim_sensor_bridge` |
| `/auto_mode` (Bool), `/arm_manual_vel` (Float32) | | dashboard | `motion_coordinator` (and steppers for `/arm_manual_vel`) |
| `/gripper_target`, `/gripper_manual_vel` (Float32) | | dashboard | arm stepper driver |
| `/gripper_state` (Float32, **raw steps**; `motion_coordinator` normalizes via `gripper_max_steps`) | | arm stepper driver | `motion_coordinator`, dashboard |
| `NAVDisplayInfo` | `navigation_interfaces/DisplayInfo` | `NAVnode` | `NAVDisplay` |
| `/morse_command` | `std_msgs/String` | `enigma_node` | `morse_bridge_node` |
| `servo_commands` | `std_msgs/String` | dashboard | `servo_node` |
| `/lights` | `std_msgs/Bool` | dashboard | `lights` |
| `/process_manager/status` | `std_msgs/String` (JSON) | `dashboard_helper` | dashboard |

Services: `calibrate_arm` (`CalibrateArm`); `~/reset_filter`, `~/get_state` (IMU filter); `~/calibrate`, `~/calibrate_all` (IMU telemetry); `Converse`, `Receive`, `Transmit` (`XLRnode`); `morse_converse` (`morse_bridge_node`); `GetPassword`, `SetMessage`, `TransmitCached` (`enigma_node`); the ~33 `NAVnode` services listed in [navigation.md](navigation.md); process start/stop `SetBool` services in `dashboard_helper`.

Actions: `move_to_point` (`MoveToPoint`), `gripper_command` (`GripperCommand`), `FollowRoute` (`NavigationAction`, a stub — see [known-issues.md](known-issues.md)), `Decode` / `Encode` (`enigma_node`).

## TF frames

`robot_state_publisher` publishes the URDF links under `base_link` (`rover_arm.xacro` is currently commented out in simulation). On hardware, `rover_ekf` publishes `odom → base_link`; when `/ekf/odometry` is fresh (`< 0.5 s`), `motion_coordinator` publishes identity `world → odom` (`world → odom → base_link`), otherwise falling back to `world → base_link`. In Gazebo simulation, `gazebo_control.xacro` publishes `odom → base_footprint` directly on `/tf`, so `sim.launch.py use_sim_stack:=true` starts `rover_ekf` with `publish_tf:=false` to avoid a dual-parent TF collision while still publishing `/ekf/odometry`. Full TF details are in [system-handover.md](system-handover.md).
