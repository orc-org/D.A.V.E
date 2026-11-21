# D.A.V.E. ROS 2 Team Demo & Walkthrough Guide

This presenter's guide walks through a complete live demonstration of how data flows through **D.A.V.E.'s ROS 2 architecture**—starting from a button press or controller input in the **Web Dashboard**, passing through `drive_node`, and branching out to either the **Simulated Gazebo Wheels** or the **Physical VESC CAN Bus Wheels**.

It also covers the critical ROS 2 configurations and middleware quirks every team member should know before writing or debugging nodes.

---

## 0. Pre-Demo Preparation Checklist

### A. Files to Open in VS Code Tabs (In Order of Presentation)
Open these 8 files in your editor before starting the presentation so you can click through them in sequence:

1. **[`dashboard/app.js`](../../../dashboard/app.js)** — Browser input capture (`WASD` / PS5 Gamepad) & WebSocket publishing to `/cmd_vel`.
2. **[`ros2_ws/src/drive_package/src/drive_node.cpp`](../drive_package/src/drive_node.cpp)** — Skid-steer mixer converting `/cmd_vel` into 4 wheel topics (`/motor/*`).
3. **[`ros2_ws/src/rover_description/scripts/sim_drive_bridge.py`](scripts/sim_drive_bridge.py)** — *(Sim Path)* Bridge converting `/cmd_vel` and `/motor/*` to `/cmd_vel_sim`.
4. **[`ros2_ws/src/rover_description/launch/sim.launch.py`](launch/sim.launch.py)** — *(Sim Path)* `ros_gz_bridge` mapping between ROS 2 DDS and Gazebo Transport.
5. **[`ros2_ws/src/rover_description/urdf/gazebo_control.xacro`](urdf/gazebo_control.xacro)** — *(Sim Path)* Gazebo `DiffDrive` and `JointStatePublisher` physics plugins.
6. **[`ros2_ws/src/drive_package/src/vesc_can_driver_node.cpp`](../drive_package/src/vesc_can_driver_node.cpp)** — *(Hardware Path)* Linux SocketCAN driver converting `/motor/*` into VESC CAN frames on `can0`.
7. **[`fastdds_profiles.xml`](../../../fastdds_profiles.xml)** — Hybrid DDS network discovery configuration.
8. **[`ros2_ws/src/rover_description/rviz/default.rviz`](rviz/default.rviz)** — RViz QoS durability and coordinate frame configuration.

### B. Terminals to Open
* **Terminal 1 (System Launcher):** At the repository root (`D.A.V.E.`) ready to run `./start_sim_system.sh`.
* **Terminal 2 (Live Inspector):** Sourced with ROS 2 Humble and workspace (`source /opt/ros/humble/setup.bash && source ros2_ws/install/setup.bash`) for running `ros2 topic list` and `ros2 topic echo`.

---

## 1. The Big Picture: Unified Input, Dual Backend

Start by showing the team this mental model: **the frontend and drive mixer do not care whether the rover is simulated or physical.** Both simulation and real hardware share the exact same upstream pipeline and split only at the final actuator stage:

```mermaid
flowchart TD
    subgraph Input_Stage ["1. Operator Input Layer"]
        Browser["Web Dashboard (http://localhost:8000)<br/>WASD Keys / Virtual Joystick / PS5 Gamepad"]
        Rosbridge["rosbridge_websocket (Port 9090)<br/>JSON over WebSocket -> ROS 2 DDS"]
    end

    subgraph Mixing_Stage ["2. ROS 2 Drive Mixing Layer"]
        CmdVel["Topic: /cmd_vel<br/>(geometry_msgs/msg/Twist)"]
        DriveNode["drive_node (C++)<br/>Skid-Steer Mixer"]
        MotorTopics["Topics: /motor/front_left, /motor/front_right,<br/>/motor/rear_left, /motor/rear_right<br/>(std_msgs/msg/Float32: -1.0 to +1.0)"]
    end

    subgraph Sim_Branch ["3A. Simulation Path (Gazebo + RViz)"]
        SimBridge["sim_drive_bridge.py<br/>Scales to m/s & rad/s + 500ms Watchdog"]
        CmdVelSim["Topic: /cmd_vel_sim<br/>(geometry_msgs/msg/Twist)"]
        ParamBridge["ros_gz_bridge (parameter_bridge)<br/>ROS 2 Twist -> Gazebo ignition.msgs.Twist"]
        GzDiffDrive["Gazebo DiffDrive & JointStatePublisher<br/>Spins 4 Wheel Joints & Publishes /odom, /tf, /joint_states"]
    end

    subgraph HW_Branch ["3B. Physical Rover Path (Jetson + CAN Bus)"]
        VescNode["vesc_can_driver_node (C++)<br/>50Hz Slew-Rate Ramping & ERPM/Duty Conversion"]
        SocketCAN["Linux SocketCAN (can0)<br/>29-bit Extended CAN Frames (IDs: 53, 44, 48, 7)"]
        PhysicalWheels["4x VESC BLDC Motor Controllers<br/>Physical Rover Wheels"]
    end

    Browser -->|"JSON Twist (10Hz)"| Rosbridge
    Rosbridge --> CmdVel
    CmdVel --> DriveNode
    DriveNode --> MotorTopics

    MotorTopics -->|"SI Skid-Steer Unmix"| SimBridge
    SimBridge --> CmdVelSim
    CmdVelSim --> ParamBridge
    ParamBridge --> GZ_Out["Gazebo Physics & RViz"]
    ParamBridge -.-> GzDiffDrive

    MotorTopics --> VescNode
    VescNode --> SocketCAN
    SocketCAN --> PhysicalWheels
    MotorTopics -.->|"Feedback Loop"| Browser
```

---

## 2. Step-by-Step Live Walkthrough & Code References

### Step 1: Launch & Show Discovery in Terminal
1. In **Terminal 2**, before starting the system, run:
   ```bash
   ros2 topic list
   ```
   *Point out that only the default ROS 2 infrastructure topics (`/parameter_events`, `/rosout`) exist.*
2. In **Terminal 1**, start the full system:
   ```bash
   ./start_sim_system.sh
   ```
   *Open [`start_sim_system.sh`](../../../start_sim_system.sh#L40-L60) and show lines **40–60**: this single script starts 4 things: `rosbridge_websocket` (port 9090), `dashboard_helper` (process manager backend), Python `http.server` (port 8000), and `sim.launch.py` (Gazebo, RViz, `robot_state_publisher`, `drive_node`, and `sim_drive_bridge`).*
3. In **Terminal 2**, run `ros2 topic list` again so the team sees all 21 topics appear live.

---

### Step 2: Dashboard Input $\rightarrow$ `/cmd_vel`
**Open File:** [`dashboard/app.js`](../../../dashboard/app.js)

1. **WebSocket Connection ([Lines 179–238](../../../dashboard/app.js#L179-L238) & [Line 1338](../../../dashboard/app.js#L1338)):**
   * Explain that browsers cannot speak native ROS 2 DDS UDP packets directly. Instead, `app.js` uses `ROSLIB.Ros` to open a WebSocket connection to `ws://localhost:9090` (`rosbridge_websocket`).
2. **Advertising `/cmd_vel` ([Lines 241–246](../../../dashboard/app.js#L241-L246)):**
   * Show `cmdVelPub`: declares a publisher on topic `/cmd_vel` with type `geometry_msgs/msg/Twist`.
3. **Reading Controller / Keyboard Inputs ([Lines 774–895](../../../dashboard/app.js#L774-L895)):**
   * Show `startControlLoopTimer()`: runs every `100ms` (10 Hz).
   * **Lines 790–802:** Reads the browser Gamepad API—Left Stick X (`gp.axes[0]`) for steering, Left Stick Y (`gp.axes[1]`) or `R2`/`L2` triggers (`gp.buttons[7]` / `gp.buttons[6]`) for throttle.
   * **Lines 843–847:** Computes `linearSpeed` and `angularSpeed`.
   * **Lines 967–1004:** Shows `calculateKeyboardVelocities()`, which does the exact same calculation if using `W/A/S/D` or arrow keys.
4. **Publishing the Message ([Lines 1006–1014](../../../dashboard/app.js#L1006-L1014)):**
   * Show `publishDriveCommand()`: packages `linear.x` (forward/back) and `angular.z` (yaw turn) into a `ROSLIB.Message` and calls `cmdVelPub.publish(msg)`.

**Live Demo Action:**
In **Terminal 2**, run:
```bash
ros2 topic echo /cmd_vel
```
Press `W` or pull `R2` in the dashboard at `http://localhost:8000` and show the live `linear.x` and `angular.z` values streaming in the terminal.

---

### Step 3: Drive Node Mixing (`/cmd_vel` $\rightarrow$ `/motor/*`)
**Open Files:** [`ros2_ws/src/drive_package/src/drive_node.cpp`](../drive_package/src/drive_node.cpp) & [`ros2_ws/src/drive_package/include/drive_package/drive_math.hpp`](../drive_package/include/drive_package/drive_math.hpp)

1. **Subscriptions & Publishers ([`drive_node.cpp` Lines 29–58](../drive_package/src/drive_node.cpp#L29-L58)):**
   * Show that `JoyDrive` (`drive_node`) subscribes to `/joy`, `/cmd_vel`, `/control_mode`, and `/sensitivity`.
   * Creates 4 `std_msgs/msg/Float32` publishers: `/motor/front_left`, `/motor/front_right`, `/motor/rear_left`, `/motor/rear_right`.
2. **SI Differential Mixing & Mode Lockout ([`drive_node.cpp` Lines 140–165](../drive_package/src/drive_node.cpp#L140-L165) & [`drive_math.hpp` Lines 61–87](../drive_package/include/drive_package/drive_math.hpp#L61-L87)):**
   * Focus on `cmdVelCallback` and `mixTwistCommand()`:
     * Ignores `/cmd_vel` if `/control_mode` is not `"drive"` (e.g. when operating the arm), and ignores zero-twist heartbeats for `0.4s` while a physical `/joy` controller is actively driving.
     * Computes SI left/right wheel speeds $v_L = v - \omega (W/2)$ and $v_R = v + \omega (W/2)$, normalizes by `max_wheel_speed`, scales by `/sensitivity` (`ultra`/`fine`/`coarse`), and clamps to `[-1.0, 1.0]`.
   * Publishes those normalized floats to all 4 `/motor/*` topics via `publishWheelPair()`.
3. **Closed-Loop UI Feedback ([`dashboard/app.js` Lines 432–466](../../../dashboard/app.js#L432-L466)):**
   * Flip back to `dashboard/app.js`: point out that the dashboard's 4 wheel power bars subscribe to `/motor/front_left`, etc. coming back from `drive_node`! If `drive_node` is stopped or locked out in `"arm"` mode, the power bars won't move.

**Live Demo Action:**
In **Terminal 2**, run:
```bash
ros2 topic echo /motor/front_left
```
Drive forward and turn in the dashboard to show the float output changing between `-1.0` and `1.0`.

---

### Step 4A: How Topics Reach the *Simulated* Wheels (Gazebo & RViz)
Now explain what happens to those commands inside the simulator across 3 files:

#### 1. [`ros2_ws/src/rover_description/scripts/sim_drive_bridge.py`](scripts/sim_drive_bridge.py)
* **Lines 19–34:** Declares `max_linear_speed` (`1.5 m/s`), `max_angular_speed` (`2.0 rad/s`), `wheel_separation` (`0.60 m`), `max_wheel_speed` (`1.0 m/s`), and `subscribe_cmd_vel` (`False` when `drive_node` runs).
* **Lines 39–52:** Subscribes to `/motor/*` (and only subscribes to `/cmd_vel` when `use_drive_node:=false`) and publishes `/cmd_vel_sim`.
* **Lines 88–115 (`update_and_publish`):**
  * **Lines 93–104:** Reconstructs SI linear velocity $v_x = \frac{1}{2}(v_L + v_R)$ and angular velocity $\omega_z = (v_R - v_L) / W$ from the 4 `/motor/*` topics and publishes `Twist` on `/cmd_vel_sim`.
  * **Lines 105–107:** Safety watchdog—if inputs stop arriving for more than `0.5s`, publishes an explicit zero-velocity `Twist` so the rover doesn't coast forever in Gazebo.

#### 2. [`ros2_ws/src/rover_description/launch/sim.launch.py`](launch/sim.launch.py)
* **Lines 71–121 (`ros_gz_bridge` & `depth_info_bridge`):**
  * Explain that Gazebo Fortress has its own internal C++ transport layer (`ignition.msgs`).
  * `parameter_bridge` translates ROS 2 messages to/from Gazebo messages:
    * `]` means **ROS 2 $\rightarrow$ Gazebo**: `/cmd_vel_sim@geometry_msgs/msg/Twist]ignition.msgs.Twist`
    * `[` means **Gazebo $\rightarrow$ ROS 2**: `/odom`, `/clock`, `/tf`, `/joint_states`, plus the simulated 3D LiDAR (`/lidar/points`), IMU (`/lidar/imu`), and RGB-D camera topics (`/camera/*`).

#### 3. [`ros2_ws/src/rover_description/urdf/gazebo_control.xacro`](urdf/gazebo_control.xacro)
* **Lines 7–40 (`DiffDrive` plugin):**
  * **Lines 12–17:** Binds the 4 URDF wheel joints (`front_left_wheel_joint`, `rear_left_wheel_joint`, `front_right_wheel_joint`, `rear_right_wheel_joint`).
  * **Lines 34–39:** Listens to `/cmd_vel_sim` inside Gazebo, applies torque to spin the 4 wheels in the physics engine, and outputs odometry to `/odom` and the chassis pose (`odom -> base_footprint`) to `/tf` at 50 Hz.
* **Lines 43–52 (`JointStatePublisher` plugin):**
  * Reads the simulated wheel rotation angles at 50 Hz and publishes them to `/joint_states`.
  * `robot_state_publisher` reads `/joint_states` and updates `/tf`, which makes the wheels visibly spin in **RViz2**.

---

### Step 4B: How the Exact Same Topics Reach the *Physical* Wheels (CAN Bus)
**Open Files:** [`ros2_ws/src/drive_package/src/vesc_can_driver_node.cpp`](../drive_package/src/vesc_can_driver_node.cpp) & [`ros2_ws/src/drive_package/include/drive_package/drive_math.hpp`](../drive_package/include/drive_package/drive_math.hpp)

Explain to the team that on the real rover, `drive_node` doesn't change at all—instead of `sim_drive_bridge.py`, we run `vesc_can_driver_node`:

1. **Wheel CAN IDs, Shared `wheel_radius`, & Safety Parameters ([`vesc_can_driver_node.cpp` Lines 29–64](../drive_package/src/vesc_can_driver_node.cpp#L29-L64)):**
   * Uses Linux network interface `can0`.
   * Each physical VESC motor controller has a unique hardware CAN ID (lines **45–48**): Front Left `53`, Front Right `44`, Rear Left `48`, Rear Right `7`.
   * Loads `wheel_radius` (`0.15 m`) from [`physical_params.yaml`](config/physical_params.yaml#L10) via `loadWheelRadiusFromConfig()` ([lines 166–196](../drive_package/src/vesc_can_driver_node.cpp#L166-L196)) so simulation and hardware never drift out of sync.
   * Declares `command_timeout` (`0.5 s` deadman stop) and `brake_current` (`5.0 A` active brake on zero/timeout).
2. **Subscribing to `/motor/*` & `/control_mode` Lockout ([Lines 111–145](../drive_package/src/vesc_can_driver_node.cpp#L111-L145)):**
   * Subscribes to `/motor/front_left`, `/motor/front_right`, `/motor/rear_left`, `/motor/rear_right`, rejects NaN values or non-`"drive"` `/control_mode` states, and records steady-clock timestamps per wheel.
3. **50 Hz Deadman Check, Slew-Rate Ramping & CAN Status Readback ([Lines 254–369](../drive_package/src/vesc_can_driver_node.cpp#L254-L369)):**
   * Reads incoming VESC `STATUS_1` CAN frames (`readStatusFrames`) and publishes measured wheel velocity and current on `/wheel_states` (`sensor_msgs/msg/JointState`) for `rover_ekf`.
   * Enforces the `0.5 s` deadman timeout (`enforceTimeout`) and smoothly ramps `current_fl_` toward `target_fl_` based on `ramp_rate_`.
4. **Control Mode Conversion ([`drive_math.hpp` Lines 89–144](../drive_package/include/drive_package/drive_math.hpp#L89-L144)):**
   * `computeVescCanCommand()` supports 3 VESC modes plus active braking:
     * **Active Brake (`comm_id = 2`, `CAN_PACKET_SET_CURRENT_BRAKE`):** Applies `brake_current` (`5000 mA`) when stopped or timed out (`brake_on_zero:=true`) so the rover holds position on slopes.
     * **`erpm` mode (`comm_id = 3`, `CAN_PACKET_SET_RPM`):** Converts velocity (`m/s`) into electrical RPM using `wheel_radius` (`0.15 m`), `pole_pairs` (`2`), and `gear_ratio` (`50:1`), clamped to `[-max_erpm, +max_erpm]`.
     * **`current` (`comm_id = 1`) & `duty` (`comm_id = 0`) modes:** Scales by `max_current` (`14A * 1000`) or `max_duty` (`0.95 * 100000`).
5. **Packing & Transmitting the Raw 29-Bit Extended CAN Frame ([`vesc_can_driver_node.cpp` Lines 198–252](../drive_package/src/vesc_can_driver_node.cpp#L198-L252)):**
   * **`initCanSocket`:** Opens a raw Linux `PF_CAN` socket on `can0` in non-blocking mode (`O_NONBLOCK`).
   * **`sendVescCanFrame`:** Packs the 29-bit extended CAN identifier (`(comm_id << 8) | can_id | CAN_EFF_FLAG`), big-endian encodes the 32-bit integer payload into `frame.data[0..3]`, and writes the `can_frame` struct to the CAN socket.

---

## 3. ROS 2 Configurations & Quirks to Explain to the Team

Use this section at the end of the demo to save your teammates hours of debugging common ROS 2 pitfalls:

### Quirk 1: FastDDS Discovery & The "4-Node Unicast Trap"
* **Reference File:** [`fastdds_profiles.xml` (Lines 15–38)](../../../fastdds_profiles.xml#L15-L38) and [`start_sim_system.sh` (Line 7)](../../../start_sim_system.sh#L7)
* **What to explain:**
  1. **Why we disable Shared Memory (`<useBuiltinTransports>false</useBuiltinTransports>`):** FastDDS's default POSIX Shared Memory (`SHM`) transport on Ubuntu can leave locked files in `/dev/shm` when a node crashes or is killed, blocking future nodes from talking. Forcing `udp_transport` with 1 MB buffers solves this.
  2. **Why we need Hybrid Discovery (`239.255.0.1` + Unicast IPs):** Long-range directional Wi-Fi antennas between the base station (`192.168.1.88`) and rover Jetson (`192.168.1.87`) often drop multicast packets, so we list their unicast IPs. However, in FastDDS, unicast IPs without explicit port numbers only probe the first **4 participant IDs** (`7410–7416`). By also including `239.255.0.1` (multicast) in `<metatrafficMulticastLocatorList>` and `<initialPeersList>`, local nodes (like our 8+ simulation nodes) can discover an unlimited number of peers while keeping the unicast fallback for the field antenna.

### Quirk 2: QoS (Quality of Service) Mismatches Silently Drop Data
* **Reference Files:**
  * [`drive_node.cpp` (Lines 12–26)](../drive_package/src/drive_node.cpp#L12-L26)
  * [`default.rviz` (Lines 198–204)](rviz/default.rviz#L198-L204)
* **What to explain:**
  * Unlike ROS 1, ROS 2 publishers and subscribers have **QoS policies** (Reliability and Durability). If a publisher and subscriber have incompatible QoS, **ROS 2 will not throw an error—they just silently ignore each other.**
  * **Example in our codebase:** `robot_state_publisher` publishes the URDF model on `/robot_description` only **once** at startup using `Durability: Transient Local` (which latches the last message for late joiners). In [`default.rviz` line 200](rviz/default.rviz#L200), RViz's `RobotModel` display **must** be set to `Durability Policy: Transient Local`—when it was set to `Volatile`, RViz missed the startup message and rendered an empty screen.

### Quirk 3: Simulation Time (`use_sim_time`) vs. Steady Clock
* **Reference Files:**
  * [`sim.launch.py` (Lines 71–192)](launch/sim.launch.py#L71-L192)
  * [`sim_drive_bridge.py` (Lines 54–115)](scripts/sim_drive_bridge.py#L54-L115)
* **What to explain:**
  * In simulation, Gazebo publishes simulated time on `/clock`. Nodes launched with `use_sim_time:=true` use `/clock` for ROS timers, TF timestamps, and `self.get_clock().now()` instead of the computer's wall clock.
  * **Gotcha 1:** Never pass `use_sim_time:=true` to `parameter_bridge` itself ([`sim.launch.py` line 71](launch/sim.launch.py#L71)), because `parameter_bridge` is the node that bridges `/clock` from Gazebo into ROS 2!
  * **Gotcha 2:** Hardware drivers like `drive_node` and `vesc_can_driver_node` use `RCL_STEADY_TIME` (`steady_.now()`) for deadman and arbitration timeouts so wall-clock safety timeouts cannot be jumped by NTP adjustments.

### Quirk 4: TF2 Single-Parent Rule (`publish_tf:=false` in Simulation)
* **Reference Files:**
  * [`gazebo_control.xacro` (Lines 36–38)](urdf/gazebo_control.xacro#L36-L38)
  * [`sim.launch.py` (Lines 164–179)](launch/sim.launch.py#L164-L179)
  * [`motion_coordinator_node.py`](../arm_controller/arm_controller/motion_coordinator_node.py)
* **What to explain:**
  * In ROS 2 TF2, every coordinate frame forms a strict tree where each child frame can only have **one parent**.
  * In Gazebo simulation, the TF chain `odom -> base_footprint -> base_link -> *_wheel_link` is published by Gazebo's `DiffDrive` plugin and `robot_state_publisher`.
  * Therefore, when `rover_ekf` runs in simulation (`use_sim_stack:=true`), [`sim.launch.py` line 176](launch/sim.launch.py#L176) sets `publish_tf:=False`, and `motion_coordinator_node.py` also defaults `publish_tf` to `False` so `base_link` never gets competing parents.

### Quirk 5: How the Dashboard Process Manager Tracks Nodes
* **Reference File:** [`dashboard_helper_node.py` (Lines 27–51 & 168–195)](../arm_controller/arm_controller/dashboard_helper_node.py#L27-L51)
* **What to explain:**
  * When team members add a new node that they want controllable from the web dashboard UI, they register its command and `pgrep` pattern in `self.processes` inside [`dashboard_helper_node.py`](../arm_controller/arm_controller/dashboard_helper_node.py#L27-L149).
  * `dashboard_helper` checks `pgrep -f <pattern>` once per second ([`is_running` on line 168](../arm_controller/arm_controller/dashboard_helper_node.py#L168)) and publishes the status JSON to `/process_manager/status`. That is why `Robot State Publisher` and `Drive Node` automatically light up green as **RUNNING** when `sim.launch.py` starts them!

---

## 4. Handy Live Commands Cheat Sheet (For Q&A)

Keep these ready in **Terminal 2** if teammates ask questions during the demo:

```bash
# 1. List all currently active topics and their message types
ros2 topic list -t

# 2. See which nodes are publishing and subscribing to a specific topic (plus QoS)
ros2 topic info /motor/front_left -v

# 3. Watch live velocity commands coming from the dashboard
ros2 topic echo /cmd_vel

# 4. Watch live scaled velocity commands entering Gazebo
ros2 topic echo /cmd_vel_sim

# 5. Measure the publishing frequency (Hz) of Gazebo odometry or joint states
ros2 topic hz /odom
ros2 topic hz /joint_states

# 6. Manually inject a drive command from the terminal without the dashboard
ros2 topic pub --rate 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.3}}"
```
