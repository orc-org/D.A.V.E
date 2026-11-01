# rover_description

this package defines the physical identity, 3D model (URDF/Xacro), modern gazebo simulation (ros_gz), and visualization settings for the D.A.V.E. rover.

## prerequisites

install modern gazebo (gazebo fortress) and teleop tools for ros 2 humble:

```bash
sudo apt-get update
sudo apt-get install -y ros-humble-ros-gz ros-humble-teleop-twist-keyboard
```

## build

always build from inside the `ros2_ws` directory:

```bash
cd ~/Coding/D.A.V.E/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

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
* modern gazebo simulation
* rosbridge websocket server (port 9090)
* web dashboard server (http://localhost:8000)
* rviz2
* drive node and motor bridge

open `http://localhost:8000` in your browser to monitor telemetry and drive.

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
