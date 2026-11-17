# D.A.V.E.
Introducing: D.A.V.E.! Okanagan Rover Craft's First Rover developed using the ROS2 Humble Hawksbill Framework! :) :)

## Overview
**D.A.V.E. (Dry-land Autonomous Vehicular Explorer)** runs on **ROS 2 Humble** across physical Jetson/VESC hardware, a **Modern Gazebo (Gazebo Fortress / `ros_gz`) + RViz2** simulation stack, and a browser-based **Web Dashboard** connected over `rosbridge_websocket`.

## Repository Layout
* [`ros2_ws/`](ros2_ws) — ROS 2 Humble workspace (`rover_description`, `drive_package`, `arm_controller`, `navigation_system`, `rover_ekf`, `IMU`, `rover_video_streamer`, `serial_communication`, `servo_controller`, `light_package`, and message interfaces).
* [`dashboard/`](dashboard) — Web dashboard frontend (`index.html`, `app.js`, telemetry, teleop, and Process Manager UI).
* [`scripts/`](scripts) — Helper utilities and hardware test scripts (`hardware_tests/`, `utilities/`).
* [`start_sim_system.sh`](start_sim_system.sh) — Launches Modern Gazebo, RViz2, `drive_node`, `sim_drive_bridge.py`, `rosbridge_websocket` (`9090`), `dashboard_helper`, and the dashboard HTTP server (`http://localhost:8000`).
* [`start_dashboard_system.sh`](start_dashboard_system.sh) — Launches the dashboard backend (`rosbridge_websocket`, `dashboard_helper`, HTTP server) for physical rover operation without Gazebo.
* [`fastdds_profiles.xml`](fastdds_profiles.xml) — FastDDS UDPv4 hybrid multicast + unicast profile used by both launcher scripts.
* [`requirements.txt`](requirements.txt) — Python workspace dependencies (`pykalman`, `pyserial`, `numpy`, `scipy`).

## Quickstart (Simulation + Dashboard)
1. Install ROS 2 Humble prerequisites (`ros-humble-ros-gz`, `ros-humble-rosbridge-server`, etc.) and Python dependencies as documented in [`ros2_ws/src/rover_description/README.md`](ros2_ws/src/rover_description/README.md#prerequisites).
2. Build the workspace from `ros2_ws/`:
   ```bash
   cd ros2_ws
   source /opt/ros/humble/setup.bash
   colcon build --symlink-install --packages-up-to rover_description arm_controller
   source install/setup.bash
   ```
   *(Or run `colcon build --symlink-install` if all optional dependencies such as `raylib` for `navigation_system` are installed.)*
3. From the repository root, launch the full simulation and web dashboard:
   ```bash
   ./start_sim_system.sh
   ```
   Then open `http://localhost:8000` in your browser.

## Documentation & Guides
* **Simulation Setup & Package Guide:** [`ros2_ws/src/rover_description/README.md`](ros2_ws/src/rover_description/README.md)
* **Simulation Architecture & Extension Recipes:** [`ros2_ws/src/rover_description/SIMULATION_ARCHITECTURE.md`](ros2_ws/src/rover_description/SIMULATION_ARCHITECTURE.md)
* **Team Demo & Live Code Walkthrough:** [`ros2_ws/src/rover_description/ROS2_TEAM_DEMO_GUIDE.md`](ros2_ws/src/rover_description/ROS2_TEAM_DEMO_GUIDE.md)

---

## Hardware Reference: Jetson Nano Pinout

Adafruit IMU: Pins 3, 5

XLR: Pins, 8, 10

Stepper Arm: Pins 18, 22, 33

Stepper Hand: Pins 13, 15, 16

Lights: Pin 12

Camera Servos: 21, 23, 27, 29


# _Code to run morse servo_ (note: It uses pin 32 on the jetson nano)
 1. ``` g++ jetson_morse.cpp -o jetson_morse -pthread```
 2. ``` sudo ./jetson_moore ```