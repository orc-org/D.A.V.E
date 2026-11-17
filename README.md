# D.A.V.E.

Okanagan Rover Craft's first rover, built on the ROS 2 Humble Hawksbill framework and run on a Jetson Orin Nano.

Full documentation is in [`docs/`](docs/README.md). If you are taking over the project, start with:
- [`docs/system-handover.md`](docs/system-handover.md) — technical onboarding, subsystem maturity, signal/TF/unit contracts, simulator vs. real-rover gaps, and pre-ground bench checks
- [`docs/guardrails-and-limits.md`](docs/guardrails-and-limits.md) — safety guardrails (active vs. missing), timeouts, rates, latency limits, and DDS/Jetson budgets
- [`docs/engineering-diary.md`](docs/engineering-diary.md) — chronological diary and commit-by-commit fix audit vs. robotics literature
- [`docs/remediation-roadmap.md`](docs/remediation-roadmap.md) — prioritized backlog (`P0`–`P4`) of fixes, additions, and retirements

The SLAM/perception program (Jetson Orin Nano Super + Unitree LiDAR) is documented in [`docs/slam/`](docs/slam/README.md).

## Repository layout

| Path | Contents |
|---|---|
| `ros2_ws/src/` | ROS 2 packages (drive, arm, IMU, EKF, navigation, serial, Morse/Enigma, video, lights, servos, simulation) |
| `dashboard/` | Browser control dashboard (talks to rosbridge) |
| `scripts/hardware_tests/`, `scripts/utilities/`, `scripts/tests/` | Bench tests, helper scripts, the coordinate-conversion test |
| `dave_custom_pins.dts`, `can0_overlay.dts` | Jetson device-tree overlays (compile with `dtc`; the `.dtbo` outputs are not tracked) |
| `fastdds_profiles.xml` | DDS discovery/transport configuration |
| `start_dashboard_system.sh`, `start_sim_system.sh` | One-command launchers (hardware / simulation) |
| `jetson_morse.cpp` | Standalone Morse-key servo program (pin 32) |

## Pinout (Jetson 40-pin header, physical numbers)

| Function | Pins |
|---|---|
| Adafruit BNO085 IMU (I²C bus 7) | 3 (SDA), 5 (SCL) |
| XLR link (UART) | 8, 10 |
| Lights | 12 |
| Arm stepper | DIR 18, ENABLE 22, STEP 33 (PWM2) |
| Gripper stepper | DIR 13, ENABLE 16, STEP 32 (PWM0) |
| Camera servos | servo 1: 24 (left), 23 (right); servo 2: 21 (left), 37 (right) |
| Morse key servo (`jetson_morse`) | 32 |

This table follows the code defaults. The earlier README listed different servo pins and gripper STEP 15; if the wiring harness disagrees with this table, see [docs/hardware.md](docs/hardware.md#discrepancies-you-should-resolve-against-the-physical-wiring).

## Quick start

```bash
cd ros2_ws && colcon build --symlink-install && source install/setup.bash
cd .. && ./start_dashboard_system.sh      # then open http://<jetson-ip>:8000
```

Simulation without hardware: `./start_sim_system.sh`. Details in [docs/build-and-run.md](docs/build-and-run.md).

## Morse servo program

```bash
g++ jetson_morse.cpp -o jetson_morse -pthread
sudo ./jetson_morse
```

Uses pin 32 on the Jetson, so it cannot run at the same time as the gripper stepper.
