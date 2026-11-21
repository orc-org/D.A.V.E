# D.A.V.E. documentation

D.A.V.E. is Okanagan Rover Craft's first rover, built on ROS 2 Humble and run on a Jetson (Orin Nano). These documents describe the code as it exists in this repository: what each part does, the math behind it, and what is known to be incomplete.

Each page was written from the source, not from the old per-package READMEs, which are still the best place for package-level run commands. Where code and older docs disagree, the page says so.

| Page | What it covers |
|---|---|
| [system-handover.md](system-handover.md) | **Technical takeover manual**: subsystem maturity, topic/TF/unit/clock contracts, hardware clashes, simulator vs. real-rover gaps, pre-drive bench runbook |
| [guardrails-and-limits.md](guardrails-and-limits.md) | **Safety guardrails & limits**: active vs. missing interlocks, verified timers/timeouts/rates, DDS/Jetson/L2 resource budgets, operational cautions |
| [engineering-diary.md](engineering-diary.md) | **Chronological diary & fix audit**: commit-by-commit rationale, necessity & literature critique (`2924889`–`d56a23b` + `2026-11-16` audit) |
| [remediation-roadmap.md](remediation-roadmap.md) | **Prioritized engineering backlog**: P0 physical safety blockers through P4 SLAM/simulation/testing gates |
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
| [code-review.md](code-review.md) | Full code review (2026-11-15): fixes made, open findings by severity and the CIRC task each blocks, SLAM status |
| [competition.md](competition.md) | CIRC 2027 tasks and rules, and the capability each one demands of the rover and code |
| [ros-and-simulation.md](ros-and-simulation.md) | What ROS 2 is, whether Linux is required, its limitations, and how it compares with MuJoCo, Gazebo, Isaac Sim and other simulators |
| [slam/README.md](slam/README.md) | **SLAM and perception program**: platform research (Jetson Orin Nano Super, Unitree L2), literature review, proposed architecture, research plan, code map, sources |

## Conventions used in these pages

- **Pin numbers** are Jetson 40-pin header *physical (BOARD)* numbers unless stated otherwise.
- **Angles** are radians inside the math and degrees at ROS message boundaries that say so (for example `/imu/euler`).
- **Local coordinates** are metres east (`x`) and north (`y`) of the home waypoint.
- File links are relative to the repository root.

## Status of this documentation

All 16 ROS 2 packages (`NAVnode` headless without requiring `raylib`), unit/smoke tests (`test_drive_math`, `test_nav_math`, `test_ekf_math`, `test_imu_math`, `test_arm_math`, `test_eval_trajectory.py`, `navnode_nmea_smoke_test.sh`, `waypoint_conversion_test.cpp`, `dashboard/vendor/fetch.sh --verify`), the multi-node integration suite (`scripts/tests/integration_adversarial_test.py`), and headless Gazebo `use_sim_stack:=true` verification ran in WSL2 on `2026-11-16`. Hardware drivers (VESC CAN bus, physical BNO085 IMU, GNSS UART, DM556Y steppers, Jetson cameras, Unitree L2) have **not** yet been exercised on the physical rover. See [system-handover.md](system-handover.md) and [guardrails-and-limits.md](guardrails-and-limits.md) before energizing motors.
