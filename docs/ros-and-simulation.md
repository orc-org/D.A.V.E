# ROS 2 and simulation: what they are, what they cost, and how they compare

A plain-language primer for the team, tied to how this repository uses them. Statements from general knowledge rather than from sources read during this project are marked **[background]**; verify them before relying on them for a decision.

## What ROS 2 is (and is not)

ROS 2 ("Robot Operating System") is **not an operating system and not a simulator.** It is middleware plus a toolbox:

| Piece | What it does | Where this repo uses it |
|---|---|---|
| **Nodes** | Small programs that each do one job | `drive_node`, `rover_ekf`, `NAVnode`, `telemetry` … ([architecture.md](architecture.md)) |
| **Topics** | Publish/subscribe streams of typed messages | `/motor/front_left`, `/imu/data_raw`, `/gps/fix` |
| **Services / actions** | Request–reply calls / long-running goals with feedback | `AddLocalObstacle`, `ResetHome`, `gripper_command` |
| **Parameters** | Runtime configuration per node | `wheel_radius`, `can_interface`, `gnss_port` |
| **Message definitions** | Shared data types in `.msg/.srv/.action` files | `navigation_interfaces`, `arm_interfaces` |
| **tf2** | A tree of coordinate frames with time | `odom → base_link` from `rover_ekf` |
| **Launch files** | Start and wire many nodes together | `imu.launch.py`, `ekf.launch.py`, `sim.launch.py` |
| **rosbag** | Record and replay all topics | `scripts/slam/record_bag.sh` ([scripts/slam](../scripts/slam/README.md)) |
| **Tools** | RViz (3-D view), `ros2 topic echo`, rosbridge (web access) | The dashboard talks to rosbridge on port 9090 ([dashboard-and-network.md](dashboard-and-network.md)) |
| **Ecosystem** | Ready-made packages: Nav2, nvblox, LiDAR odometry, drivers | The SLAM plan ([slam/architecture.md](slam/architecture.md)) is built from these |

Under the hood, nodes talk over DDS (a standard data-distribution layer). This repo uses eProsima Fast DDS and configures it in [`fastdds_profiles.xml`](../fastdds_profiles.xml).

**Why it matters for this project:** the things we want to reuse — Nav2 for planning, Isaac ROS nvblox and cuVSLAM for GPU mapping and odometry, the community LiDAR-inertial odometry packages, the Unitree SDK's ROS wrapper — are all ROS packages. Choosing ROS buys that ecosystem, not a particular algorithm.

## Do we need Linux?

**For the rover: yes, in practice.** Details:

- The Jetson runs Linux (JetPack 6.x, Ubuntu 22.04 base) and ROS 2 **Humble** is the matching release. The repo already assumes this ([build-and-run.md](build-and-run.md)). Isaac ROS (nvblox, cuVSLAM) targets Jetson and Linux.
- The hardware interfaces are Linux-specific: SocketCAN (`can0`) for the motor controllers, `Jetson.GPIO`, sysfs PWM, device-tree overlays, `/dev/tty*` serial ports ([hardware.md](hardware.md)).
- This repository's workspace does not build on native Windows (noted since the first survey).

**For development laptops:** Ubuntu 22.04 natively, a dual boot, or a virtual machine all work. On Windows, WSL2 runs Linux ROS 2 well for code, simulation and replaying bags, but USB devices, CAN, GPIO and real-time behaviour do not pass through easily, so anything touching hardware belongs on the Jetson. ROS 2 also has Windows and macOS support at lower priority than Ubuntu **[background; see REP-2000]**, but none of our dependencies (Isaac ROS, the Unitree SDK build, SocketCAN) are validated there.

Practical recommendation: Ubuntu 22.04 everywhere the team writes ROS code (dual boot or a spare machine is the least painful), and use Windows only for documentation and non-ROS work.

## Limitations of ROS 2

| Limitation | Why it matters here |
|---|---|
| **Not real-time by default.** Standard Linux scheduling, executors and DDS give no hard timing guarantees | Fine for navigation at 10–20 Hz. Do not put motor safety on a ROS callback alone; the VESC controllers and a hardware e-stop must protect the robot independently |
| **Tied to Ubuntu releases and short-lived distributions.** Humble matches Ubuntu 22.04; each distribution has an end-of-life date **[background: Humble ends in 2027]** | The Jetson's JetPack version fixes the Ubuntu version, which fixes the ROS release. Plan upgrades together |
| **DDS configuration is fiddly.** Discovery across networks, multicast, shared memory | This repo already hit it: `fastdds_profiles.xml` disables shared memory (noted as a fix for an IPC bug) and hard-codes peer IPs ([dashboard-and-network.md](dashboard-and-network.md)). A second network for the LiDAR must not disturb it ([slam/platform.md](slam/platform.md#integration-problems-found-on-paper)) |
| **Large messages are costly.** A point-cloud topic copied between processes costs CPU and memory bandwidth unless zero-copy/composition is used | 64 000 points/s from the L2 plus a depth camera on a six-core board. Run odometry and mapping in one process (a composable node) where possible **[background]** |
| **No built-in security by default.** DDS security exists but is off | rosbridge is unauthenticated; keep the rover network private ([dashboard-and-network.md](dashboard-and-network.md)) |
| **Time and frames are easy to get wrong.** tf2 needs consistent timestamps and exactly one parent per frame | Existing conflict: `motion_coordinator` and `rover_ekf` both publish a parent for `base_link` ([known-issues.md](known-issues.md)) |
| **Build system complexity.** `colcon` + CMake + `package.xml` must agree | We already fixed missing dependencies in `arm_controller` and a duplicated header ([known-issues.md](known-issues.md)) |
| **Python nodes are slower and share one interpreter lock per process** | Keep heavy per-point work in C++ or CUDA nodes |
| **Tooling for debugging distributed timing is limited** | Use rosbags and the logging scripts rather than live guesses |

None of these is a reason to avoid ROS for this rover; they are reasons to keep safety outside it and to measure on the real board.

## Is ROS "better than MuJoCo" or other popular simulators?

They are different kinds of tool, so the comparison is a category error:

- **ROS 2** connects software components and provides robot algorithms and tools.
- **A simulator / physics engine** (Gazebo, MuJoCo, Isaac Sim, Webots, Genesis …) imitates the world: physics, sensors, terrain.

You normally use **both**: ROS runs your robot software, and a simulator stands in for the hardware, exchanging the same topics (`/cmd_vel` in, sensor topics out). The software does not change when you move from simulation to the real robot.

### Simulator comparison for this project

Ratings are for *our* needs: a wheeled outdoor rover, a 3-D LiDAR and a depth camera, ROS 2 Humble, a Jetson target, SLAM and navigation development. Rows without a source are **[background]** and should be confirmed before committing.

| Simulator | Strengths | Weaknesses for us |
|---|---|---|
| **Gazebo (gz sim)** — already in the repo (`sim.launch.py`, `start_sim_system.sh`) | First-class ROS 2 integration; lidar and depth-camera sensor plugins; terrain/heightmap worlds; runs on a modest GPU or CPU | Physics and rendering fidelity are lower than the others; granular/soft terrain is not modelled; sensor noise realism needs tuning |
| **MuJoCo** | Fast, accurate contact dynamics; excellent for control research and reinforcement learning; very light | Not ROS-native (community bridges exist); rendering and sensor models (LiDAR, depth with noise) are basic, so LiDAR needs a separate plugin (GS-Playground's authors point to a separate "MuJoCo-LiDAR" repository for this **[P]**); rigid-body only, so no sand or mud |
| **NVIDIA Isaac Sim** | Photorealistic RTX sensors (LiDAR, depth), good ROS 2 bridge, built for synthetic data and Isaac ROS workflows | Needs a desktop RTX GPU, heavy, not for the Jetson; GS-Playground's paper reports Isaac Sim running out of memory at 1280×720 in their benchmark **[P]** |
| **Webots, CoppeliaSim** | Easy to start, decent ROS 2 support | Smaller ecosystems for outdoor terrain and LiDAR SLAM |
| **GS-Playground, Genesis, ManiSkill-style GPU simulators** | Very high throughput for reinforcement learning | GS-Playground is rigid-body, legged/arm focused, x86 only, partial code release ([slam/literature.md](slam/literature.md#4-gs-playground)) |
| **MPM-based simulators (Taichi MPM, PhysGaussian-style)** | Model sand, snow and soft ground | Research tools; no ROS integration; not for real-time rover control |

### What simulation can and cannot do for SLAM

- It can test **integration**: frames, topics, launch files, the planner loop, failure handling, without risking the hardware.
- It cannot replace **real data** for SLAM evaluation. Simulated LiDAR and depth are cleaner than the real sensors, and real terrain, dust, sunlight and vibration are what break odometry. The plan therefore leans on recorded bags from the real rover ([slam/research-plan.md](slam/research-plan.md)), with Gazebo for integration tests.
- Physics-heavy ideas (sinkage, slip on soft ground) do need a simulator that models deformable terrain; none of the ROS-native ones do.

### Recommendation

1. **Keep ROS 2 Humble.** No alternative gives the Nav2 / nvblox / LiDAR-odometry / Unitree ecosystem, and the rover already runs on it.
2. **Keep Gazebo as the integration simulator** and add a simulated 3-D LiDAR and depth camera to the robot model (planned in [slam/code-map.md](slam/code-map.md)).
3. **Use real bags as the evidence for SLAM claims.**
4. **Add MuJoCo or Isaac Sim only for a specific need:** MuJoCo for fast contact/control experiments, Isaac Sim if photorealistic synthetic data on a desktop GPU becomes valuable. Treat a deformable-terrain simulator as a research tool for the stretch goal, not as part of the rover stack.

## Where to look next

- The package layout and topics: [architecture.md](architecture.md).
- How to build and launch: [build-and-run.md](build-and-run.md).
- The SLAM program: [slam/README.md](slam/README.md).
