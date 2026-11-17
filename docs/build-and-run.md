# Build and run

The workspace targets **Ubuntu 22.04 + ROS 2 Humble** (the Jetson, or a Linux/WSL2 machine). It does not build on native Windows.

## Dependencies

ROS packages (via `rosdep` or `apt`): `rclcpp`, `rclpy`, `rclcpp_action`, `rclcpp_components`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `std_srvs`, `visualization_msgs`, `tf2_ros`, `cv_bridge`, `xacro`, `rviz2`, `robot_state_publisher`, `rosbridge_server`, `ros_gz*`/Gazebo for simulation.

System libraries: `raylib` (NAVDisplay), GStreamer with `x264enc` and, on the Jetson, `nvarguscamerasrc` (video), OpenCV, SocketCAN (`can-utils`).

Python (`pip install -r requirements.txt`): `pykalman`, `pyserial`, `numpy`, `scipy`. Also needed by individual nodes: `pyyaml` and `matplotlib` (arm controller), `Jetson.GPIO` (lights, servos, GPIO stepper driver), and the Adafruit CircuitPython BNO08x libraries plus `adafruit_extended_bus` (IMU on hardware).

## Build

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

`navigation_system` finds `serial_interfaces` through `ros2_ws/install` (its CMake adds `${ROS2_WS}/install` to the prefix path and includes a header from there). If a first full build fails in `navigation_system`, build the interface packages first and then the rest:

```bash
colcon build --packages-select serial_interfaces navigation_interfaces arm_interfaces enigma_machine_interfaces
colcon build --symlink-install
```

### Verified build status (WSL2, Ubuntu 22.04, Humble)

All 16 packages build in WSL2 Humble (`NAVnode` now builds headless when `raylib` is absent; `NAVDisplay` builds automatically whenever `raylib` is installed, e.g. from `~/opt/raylib`). [`scripts/tests/navnode_nmea_smoke_test.sh`](../scripts/tests/navnode_nmea_smoke_test.sh) exercises `NAVnode` end to end without hardware alongside [`scripts/tests/integration_adversarial_test.py`](../scripts/tests/integration_adversarial_test.py) (multi-node ROS 2 seam tests) and the GTest/pytest suites (`test_drive_math`, `test_nav_math`, `test_ekf_math`, `test_imu_math`, `test_arm_math`, and `scripts/slam/test_eval_trajectory.py`).

## One-command launchers

```bash
./start_dashboard_system.sh   # rosbridge :9090 + dashboard_helper + dashboard on :8000
./start_sim_system.sh         # Gazebo + RViz + dashboard (no hardware)
```

Open `http://<jetson>:8000` and connect to `ws://<jetson>:9090`. From the dashboard's process manager you can start individual nodes.

## Running nodes by hand

```bash
# Drive
ros2 run drive_package drive_node
ros2 run drive_package vesc_can_driver_node --ros-args -p fl_can_id:=53 -p fr_can_id:=44 -p rl_can_id:=48 -p rr_can_id:=7

# Sensing and estimation
ros2 launch imu imu.launch.py sim_mode:=false      # telemetry + AUKF, dt follows publish_rate
ros2 run navigation_system gps_bridge_node.py --ros-args -p port:=/dev/ttyUSB0 -p baud:=38400
ros2 launch rover_ekf ekf.launch.py

# Navigation
ros2 run navigation_system NAVnode --ros-args -p gnss_port:=/dev/ttyUSB1   # use a different port from gps_bridge_node
ros2 run navigation_system NAVDisplay

# Arm
ros2 run arm_controller motion_coordinator
ros2 run arm_controller arm_stepper_arduino_driver   # or arm_stepper_driver for the GPIO variant

# Morse task
ros2 run rover_video_streamer video_streamer --camera-type csi --device 0 --host <base-ip> --port 5000
ros2 run enigma_machine morse_recorder.py
ros2 run enigma_machine enigma_node
ros2 run serial_communication morse_bridge_node
```

Run `ros2 run <pkg>` with no executable to list what a package installs. (The process manager runs `video_streamer.py` as a plain Python script; the entry point does the same thing.)

## Hardware bring-up

```bash
sudo ip link set can0 type can bitrate 500000 restart-ms 100 && sudo ip link set up can0   # VESCs
dtc -@ -I dts -O dtb -o dave_custom_pins.dtbo dave_custom_pins.dts                          # pin overlay
dtc -@ -I dts -O dtb -o can0_overlay.dtbo can0_overlay.dts                                  # CAN pin-mux overlay
```

Pins, CAN IDs and serial ports are in [hardware.md](hardware.md). Standalone hardware checks live in [`scripts/hardware_tests/`](../scripts/hardware_tests/README.md); helper scripts in `scripts/utilities/`.

## Tests

- `ros2_ws`: `colcon test --packages-select drive_package navigation_system rover_ekf imu arm_controller` runs C++ GTests (`test_drive_math` 4 cases, `test_nav_math` 3 cases), Python unit tests (`test_ekf_math` 4 cases, `test_imu_math` 1 case, `test_arm_math` 1 case), and ament lint/cmake/xml checks (`cppcheck` skipped where absent).
- `scripts/tests/waypoint_conversion_test.cpp` — no-ROS numeric test of WGS-84 coordinate conversions.
- `scripts/tests/navnode_nmea_smoke_test.sh` — end-to-end virtual-UART NMEA smoke test for `NAVnode` (including corrupted `*FF` checksum rejection, `ResetHome`, RMC speed, waypoint targeting, and `FollowRoute`).
- `scripts/tests/integration_adversarial_test.py` — multi-node ROS 2 Humble seam suite exercising drive lockouts, sensitivity scaling, `/joy` vs `/cmd_vel` arbitration, EKF slow-clock & one-sided wheel rejection, `NAVnode` `/gps/fix` + `ResetHome` + `RemoveLocal` (`routeToEdit`), and arm stepper/coordinator URDF & base-lock boundaries.
- `scripts/slam/test_eval_trajectory.py` — `pytest` suite covering `eval_trajectory.py` clock mismatch checks, vector loop closure, and SE(3) rotation metrics.

## SLAM data capture and benchmarking

`scripts/slam/` holds helpers for recording datasets and benchmarking SLAM; see [`scripts/slam/README.md`](../scripts/slam/README.md) and [slam/research-plan.md](slam/research-plan.md).

## Simulation

`ros2 launch rover_description sim.launch.py` starts Gazebo, RViz, `robot_state_publisher`, `drive_node`, and `sim_drive_bridge.py` (`subscribe_cmd_vel:=false` when `use_drive_node:=true` so `/control_mode` and `/sensitivity` are enforced; `cmd_vel_is_si:=true`, `wheel_separation:=0.60`). Pass `use_sim_stack:=true` to also run `sim_sensor_bridge.py`, `rover_ekf` (`wheel_radius:=0.15`, `publish_tf:=false`), and `NAVnode` (`use_gps_fix_topic:=true`) end to end in Gazebo. The robot model is parameterized from `physical_params.yaml` (`rover_arm.xacro` stays disabled until CAD offsets against `lidar_link` are resolved). See [`ros2_ws/src/rover_description/README.md`](../ros2_ws/src/rover_description/README.md).

### Simulated perception sensors

[`urdf/sensors.xacro`](../ros2_ws/src/rover_description/urdf/sensors.xacro) adds a LiDAR (modelled on the Unitree L2: 360 x 96 degrees, 30 m, 360 x 16 samples at 10 Hz), its IMU and a generic 640 x 480 depth camera; the numbers are in the `sensors:` block of `physical_params.yaml`. `sim.launch.py` bridges them to the SLAM sensor contract ([slam/architecture.md](slam/architecture.md)): `/lidar/points`, `/lidar/imu`, `/camera/color/image`, `/camera/depth/image` and the two `camera_info` topics. The world loads the Gazebo sensor and IMU systems ([`worlds/empty.sdf`](../ros2_ws/src/rover_description/worlds/empty.sdf)).

Verified in WSL2 (headless, `ros2 launch rover_description sim.launch.py use_rviz:=false use_drive_node:=false gz_flags:="-r -s --headless-rendering"`): all six topics publish, the cloud is in `lidar_link` (360 points wide), the depth image is `32FC1`. **WSL2 caveat:** the GPU OpenGL driver crashes Gazebo's ogre2 renderer (`Ogre::UnimplementedException`), so Mesa software rendering is required (`export LIBGL_ALWAYS_SOFTWARE=1`; `start_sim_system.sh` does this automatically under WSL). It works but is slow: measured about 3.9 Hz for the LiDAR (10 Hz requested), 55 Hz for the IMU (200 Hz) and 4-6 Hz for the cameras, so treat the simulation as a functional test, not a timing test. Recorded bags are zstd-compressed by `record_bag.sh`, which the Python `rosbags` library (used by KISS-ICP and `eval_trajectory.py`) cannot read; convert them with `ros2 bag convert` first (command in the header of `scripts/slam/eval_trajectory.py`). KISS-ICP 1.3.0 also needs `rosbags==0.9.23`. The sensor models have no calibrated noise or motion distortion, and the GUI and RViz were not run.
