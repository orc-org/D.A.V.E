# D.A.V.E. documentation

D.A.V.E. is Okanagan Rover Craft's first rover, built on ROS 2 Humble and run on a Jetson (Orin Nano). These documents describe the code as it exists in this repository: what each part does, the math behind it, and what is known to be incomplete.

Each page was written from the source, not from the old per-package READMEs, which are still the best place for package-level run commands. Where code and older docs disagree, the page says so.

| Page | What it covers |
|---|---|
| [architecture.md](architecture.md) | Packages, how data flows between them, topic/service/action map |
| [hardware.md](hardware.md) | Jetson header pinout, CAN, serial devices, device-tree overlays, network addresses |
| [drive-system.md](drive-system.md) | Joystick mixing, VESC CAN protocol, the ERPM formula, ramping |
| [navigation.md](navigation.md) | Waypoints/obstacles, coordinate systems (geodetic, ECEF, local ENU), bearing, detour geometry, NMEA parsing |
| [state-estimation.md](state-estimation.md) | IMU unscented Kalman filter and the rover EKF, with the matrices |
| [arm-and-kinematics.md](arm-and-kinematics.md) | Homogeneous-transform kinematics, numeric inverse kinematics, stepper step/frequency math |
| [mission-tasks.md](mission-tasks.md) | Morse recorder / Enigma machine, camera streaming, lights, camera servos |
| [dashboard-and-network.md](dashboard-and-network.md) | rosbridge, the process manager, the web dashboard, DDS configuration |
| [build-and-run.md](build-and-run.md) | Dependencies, building, launch scripts, simulation |
| [known-issues.md](known-issues.md) | Limitations, open questions for the hardware team, and the SLAM gap |
| [ros-and-simulation.md](ros-and-simulation.md) | What ROS 2 is, whether Linux is required, its limitations, and how it compares with MuJoCo, Gazebo, Isaac Sim and other simulators |
| [slam/README.md](slam/README.md) | **SLAM and perception program**: platform research (Jetson Orin Nano Super, Unitree L2), literature review, proposed architecture, research plan, code map, sources |

## Conventions used in these pages

- **Pin numbers** are Jetson 40-pin header *physical (BOARD)* numbers unless stated otherwise.
- **Angles** are radians inside the math and degrees at ROS message boundaries that say so (for example `/imu/euler`).
- **Local coordinates** are metres east (`x`) and north (`y`) of the home waypoint.
- File links are relative to the repository root.

## Status of this documentation

Nothing in this repository was compiled or run on a Jetson while writing these pages. The coordinate-conversion fixes described in [navigation.md](navigation.md) were checked with a small numeric test; everything else was read, not executed.
