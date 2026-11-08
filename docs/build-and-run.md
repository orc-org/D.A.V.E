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

- `scripts/tests/waypoint_conversion_test.cpp` — no-ROS numeric test of the coordinate conversions; the build command is in the file header. It passes against the current `Waypoint.cpp`.
- Four Python packages carry the standard ament lint tests (copyright, flake8, pep257). There are no behavioural tests for the drive, arm, EKF or navigation code.

## SLAM data capture and benchmarking

`scripts/slam/` holds helpers for recording datasets and logging Jetson load; see [`scripts/slam/README.md`](../scripts/slam/README.md) and [slam/research-plan.md](slam/research-plan.md). They have been syntax-checked only.

## Simulation

`ros2 launch rover_description sim.launch.py` starts Gazebo, RViz, `robot_state_publisher`, the drive node and `sim_drive_bridge.py`, which re-publishes `/cmd_vel` as `/cmd_vel_sim` for the simulated skid-steer plugin. The robot model, including the arm (`rover_arm.xacro`), is parameterised from `physical_params.yaml`. See [`ros2_ws/src/rover_description/README.md`](../ros2_ws/src/rover_description/README.md).
