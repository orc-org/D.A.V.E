# rover_description

this package defines the physical identity, 3D model (URDF/Xacro), modern gazebo simulation (ros_gz), and visualization settings for the D.A.V.E. rover.

> [!NOTE]
> * For a comprehensive deep dive into the simulation design, topic routing, coordinate frames (TF2), DDS configurations, and developer extension recipes (adding LiDAR, cameras, robotic arm control, custom worlds), see [SIMULATION_ARCHITECTURE.md](SIMULATION_ARCHITECTURE.md).
> * For a step-by-step live walkthrough script showing how data flows from the Web Dashboard through ROS 2 to either the simulated Gazebo wheels or physical VESC CAN wheels (with exact file and line references), see [ROS2_TEAM_DEMO_GUIDE.md](ROS2_TEAM_DEMO_GUIDE.md).

## prerequisites

install modern gazebo (gazebo fortress), ROS 2 Humble simulation/web dependencies, and workspace python packages:

```bash
sudo apt-get update
sudo apt-get install -y \
  ros-humble-ros-gz \
  ros-humble-xacro \
  ros-humble-robot-state-publisher \
  ros-humble-rviz2 \
  ros-humble-rosbridge-server \
  ros-humble-joy \
  ros-humble-teleop-twist-keyboard
```

From inside `ros2_ws` (with `/opt/ros/humble/setup.bash` sourced), you can also resolve ROS dependencies and Python packages:

```bash
rosdep install --from-paths src --ignore-src -r -y
pip3 install -r ../requirements.txt
```

## build

always build from inside the `ros2_ws` directory (replace `<repo-root>` with your clone path, e.g. `~/D.A.V.E.`):

```bash
cd <repo-root>/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

> [!TIP]
> If the full workspace build fails on `navigation_system` because `raylib` is not installed on your machine, build only the simulation and dashboard packages instead:
> ```bash
> colcon build --symlink-install --packages-up-to rover_description arm_controller
> ```

## running the simulation

### option 1: standalone simulation (gazebo + rviz)

to launch modern gazebo, rviz, robot state publisher, drive node, and the simulation bridge:

```bash
ros2 launch rover_description sim.launch.py
```

optional launch arguments:
* `use_rviz:=false` - disable rviz GUI
* `use_drive_node:=false` - disable drive_node and motor bridge
* `world:=<path>` - use a custom modern gazebo sdf world file

### option 2: full system (simulation + web dashboard)

from the project root, launch everything together with one script:

```bash
./start_sim_system.sh
```

this starts:
* modern gazebo simulation (`ign gazebo` / `ros_gz_sim`)
* rosbridge websocket server (port 9090)
* dashboard helper node (`arm_controller/dashboard_helper`, process manager backend)
* web dashboard server (http://localhost:8000)
* rviz2
* drive node and motor bridge

open `http://localhost:8000` in your browser to monitor telemetry and drive.

## using the web dashboard

* **Manual vs. Auto Mode:** Teleop (`W`/`A`/`S`/`D`, on-screen buttons, and browser gamepad driving) only publishes `/cmd_vel` when the dashboard is in **MANUAL** mode (`autoMode == false`), which is the default on page load.
* **Browser Gamepad Wake-Up:** Browsers do not expose USB/Bluetooth controllers to `navigator.getGamepads()` until you interact with the controller—plug in/connect your gamepad and press any button or move a stick while the dashboard tab is focused.
* **Process Manager Safety (Sim vs. Hardware):**
  * Do **not** start **Motion Coordinator** (`motion_coordinator`) while Gazebo is running; it publishes a competing `world -> base_link` TF that breaks RViz's `odom -> base_footprint -> base_link` tree.
  * Hardware-only entries in the Process Manager (such as **VESC BLDC Wheel Driver**, **Arm & Gripper Steppers**, and camera streamers) are for the physical Jetson/rover hardware and will fail or conflict if started on a local simulation-only laptop.

## network & `ROS_DOMAIN_ID`

* [`./start_sim_system.sh`](../../../start_sim_system.sh) sets `FASTRTPS_DEFAULT_PROFILES_FILE` to [`fastdds_profiles.xml`](../../../fastdds_profiles.xml), which uses UDPv4 (disabling FastDDS SHM) with hybrid multicast (`239.255.0.1`), `127.0.0.1`, and the hardware rover/base-station peer IPs (`192.168.1.87` / `192.168.1.88`).
* All terminals inspecting topics (`ros2 topic list`, `ros2 topic echo`) must share the same `ROS_DOMAIN_ID` as the launched simulation (default `0` if unset). If you edit `ROS_DOMAIN_ID` on one shell, export the same ID everywhere or run against the default domain.

## controlling the rover

the simulation connects directly to `drive_node` from `drive_package`. you can drive the rover using any of the following methods:

### 1. web dashboard
open `http://localhost:8000` and use the on-screen buttons, W/A/S/D keys, or a gamepad plugged into your browser.

### 2. gamepad (ps5 / usb controller)
connect your controller to your machine and run:

```bash
ros2 run joy joy_node
```

`drive_node` reads trigger throttle and left stick steering to drive the wheels.

### 3. keyboard in terminal
in a separate terminal:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

press `i` / `,` for forward and backward, `j` / `l` for turning.

## package contents

* **`urdf/rover.urdf.xacro`**: main robot description file configuring the 4-wheel mobile chassis, collision geometry, and inertia.
* **`urdf/rover_arm.xacro`**: modular robotic arm and gripper definitions with collision and inertial elements.
* **`urdf/gazebo_control.xacro`**: modern gazebo plugins (`ignition::gazebo::systems::DiffDrive` and `JointStatePublisher`).
* **`urdf/inertial_macros.xacro`**: standard formulas for box, cylinder, and wheel inertia tensors.
* **`worlds/empty.sdf`**: modern gazebo sdf world with physics and scene broadcaster plugins.
* **`scripts/sim_drive_bridge.py`**: bridges `/motor/*` topics from `drive_node` to gazebo's `/cmd_vel_sim`.
* **`launch/sim.launch.py`**: launches modern gazebo, spawns the rover, starts rviz, and runs `drive_node` and the motor bridge.
* **`launch/view_robot.launch.py`**: launches purely rviz without physics simulation.
* **`config/physical_params.yaml`**: single source of truth for robot physical dimensions (meters and kilograms).

## re-enabling the robotic arm

the arm is currently modularized to keep the basic 4-wheel driving demo simple. to re-attach the arm:

1. open `urdf/rover.urdf.xacro`
2. uncomment the include line near the bottom:
   ```xml
   <!-- to attach the robotic arm, simply uncomment the line below: -->
   <xacro:include filename="rover_arm.xacro" />
   ```
3. rebuild the workspace with `colcon build --symlink-install`.

## troubleshooting & logs

When running [`./start_sim_system.sh`](../../../start_sim_system.sh), background component output is redirected to `/tmp/`:

* **Gazebo / RViz / Drive (`sim.launch.py`):** `/tmp/gazebo_sim.log`
* **Rosbridge WebSocket (`9090`):** `/tmp/rosbridge_sim.log`
* **Dashboard Helper (`dashboard_helper`):** `/tmp/dashboard_helper.log`
* **Web Dashboard HTTP Server (`8000`):** `/tmp/webserver_sim.log`

Quick checks if something looks wrong:
* **Workspace not built error:** `./start_sim_system.sh` exits immediately if `ros2_ws/install/setup.bash` does not exist—run `colcon build --symlink-install` inside `ros2_ws` first.
* **Empty RViz or rover not moving:** Inspect `/tmp/gazebo_sim.log` for missing packages (`ros_gz`, `xacro`, `drive_package`) or run `ros2 topic echo /cmd_vel` in a sourced terminal to confirm commands arrive.
* **Dashboard says disconnected:** Check `/tmp/rosbridge_sim.log` (port `9090`) and `/tmp/dashboard_helper.log`.
* **Stale processes after crash:** Re-run `./start_sim_system.sh` (its header cleans up leftover `ign gazebo`, `rviz2`, `rosbridge_websocket`, `dashboard_helper`, and `http.server` processes automatically).
