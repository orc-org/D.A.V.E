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
            ▲  ▲                         │                  │
 /cmd_vel ──┘  └ /control_mode,/sensitivity               ├──► rover_ekf (wheel speed)
 (from motion_coordinator IK)                              └──► motion_coordinator (visual odometry)

BNO085 ─► telemetry ─► /imu/data_raw, /imu/euler ─┬─► kalman_filter ─► /imu/data_filtered, /imu/odometry, /imu/filtered_euler
                                                  └─► rover_ekf ◄─ /gps/fix ◄─ gps_bridge_node ◄─ GNSS (USB serial)
                                                          └─► /ekf/odometry, /ekf/filtered_euler, TF odom→base_link

GNSS ─► NAVnode (opens its own serial port) ─► NAVDisplayInfo ─► NAVDisplay (raylib map)
```

## Topic, service and action map

Topics (publisher → subscribers):

| Topic | Type | Publisher | Subscribers |
|---|---|---|---|
| `/joy` | `sensor_msgs/Joy` | joy driver (external) | `drive_node`, `input_viewer` |
| `/control_mode` | `std_msgs/String` (`"drive"`/other) | none in the current dashboard (came from the old `web_interface`) | `drive_node` (stops motors if not `"drive"`) |
| `/sensitivity` | `std_msgs/String` (`ultra`/`fine`/`coarse`) | none in the current dashboard | `drive_node` (scales output ×0.25/0.5/1.0; default 1.0) |
| `/cmd_vel` | `geometry_msgs/Twist` | `motion_coordinator` (auto mode) | `drive_node`, `rover_ekf`, `sim_drive_bridge` |
| `/motor/{front_left,front_right,rear_left,rear_right}` | `Float32` | `drive_node` | `vesc_can_driver_node`, `rover_ekf`, `motion_coordinator` |
| `/imu/data_raw` | `sensor_msgs/Imu` | `telemetry` | `kalman_filter`, `rover_ekf` |
| `/imu/euler` | `geometry_msgs/Vector3` (degrees) | `telemetry` | `rover_ekf` |
| `/imu/data_filtered`, `/imu/odometry`, `/imu/filtered_euler` | `Imu`, `Odometry`, `Vector3` | `kalman_filter` | dashboard |
| `/gps/fix`, `/gps/nmea_raw` | `NavSatFix`, `String` | `gps_bridge_node` | `rover_ekf`, dashboard |
| `/ekf/odometry`, `/ekf/filtered_euler` | `Odometry`, `Vector3` | `rover_ekf` | dashboard |
| `arm_target` | `geometry_msgs/Point` | dashboard | `motion_coordinator` |
| `arm_status`, `arm_global_pos` | `ArmStatus`, `Point` | `motion_coordinator` | dashboard |
| `joint_states` | `sensor_msgs/JointState` | `motion_coordinator` | arm steppers, `robot_state_publisher` |
| `/auto_mode` (Bool), `/arm_manual_vel` (Float32) | | dashboard | `motion_coordinator` |
| `/gripper_target`, `/gripper_manual_vel` (Float32) | | dashboard | arm stepper driver |
| `/gripper_state` (Float32, 0–1) | | arm stepper driver | `motion_coordinator`, dashboard |
| `NAVDisplayInfo` | `navigation_interfaces/DisplayInfo` | `NAVnode` | `NAVDisplay` |
| `/morse_command` | `std_msgs/String` | `enigma_node` | `morse_bridge_node` |
| `servo_commands` | `std_msgs/String` | dashboard | `servo_node` |
| `/lights` | `std_msgs/Bool` | dashboard | `lights` |
| `/process_manager/status` | `std_msgs/String` (JSON) | `dashboard_helper` | dashboard |

Services: `calibrate_arm` (`CalibrateArm`); `~/reset_filter`, `~/get_state` (IMU filter); `~/calibrate`, `~/calibrate_all` (IMU telemetry); `Converse`, `Receive`, `Transmit` (`XLRnode`); `morse_converse` (`morse_bridge_node`); `GetPassword`, `SetMessage`, `TransmitCached` (`enigma_node`); the ~33 `NAVnode` services listed in [navigation.md](navigation.md); process start/stop `SetBool` services in `dashboard_helper`.

Actions: `move_to_point` (`MoveToPoint`), `gripper_command` (`GripperCommand`), `FollowRoute` (`NavigationAction`, a stub — see [known-issues.md](known-issues.md)), `Decode` / `Encode` (`enigma_node`).

## TF frames

`robot_state_publisher` publishes the URDF tree under `base_link`. Two nodes also publish a parent for `base_link`: `motion_coordinator` (`world → base_link`, simulated from wheel commands) and `rover_ekf` (`odom → base_link`). Running both at once gives `base_link` two parents. See [known-issues.md](known-issues.md).
