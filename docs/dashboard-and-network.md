# Dashboard and network

## Components

```
Browser (operator laptop)                         Jetson
 dashboard/index.html + app.js  ──ws://<ip>:9090──►  rosbridge_websocket  ──► ROS 2 graph
        ▲  http://<jetson>:8000                      dashboard_helper  (process manager)
        └──────────────── python3 -m http.server ────┘
 UDP video (5000, 5002, 5004)  ◄───────────────────  video_streamer
```

`start_dashboard_system.sh` starts, in this order: `rosbridge_websocket` on port 9090, `arm_controller dashboard_helper`, and `python3 -m http.server 8000 --directory dashboard`. It first kills leftover instances, sources `/opt/ros/humble` and `ros2_ws/install`, and sets `FASTRTPS_DEFAULT_PROFILES_FILE` to the repository's `fastdds_profiles.xml`. `start_sim_system.sh` is the Gazebo equivalent and deliberately unsets the profile so discovery uses default multicast.

## The web dashboard (`dashboard/`)

A static page (vanilla JavaScript, about 3 500 lines in `app.js`) that connects to rosbridge with `roslib`. Enter the rosbridge WebSocket URL in the connect box; it reconnects automatically on load.

Panels (from `index.html`): location and GPS navigation (Leaflet map with the rover path), IMU telemetry and attitude, camera start-up, Enigma/Morse decoder, system telemetry, wheel power monitor, control-mode selection, system process monitor, sensitivity settings, and controls.

It publishes: `/cmd_vel`, `/arm_target`, `/auto_mode`, `/arm_manual_vel`, `/gripper_manual_vel`, `/gripper_target`, `/lights`, `/servo_commands`, `/gps/set_target`, and calls the IMU, Morse and Enigma services. It subscribes to the arm, motor, IMU, GPS, EKF, TF, battery and process-status topics listed in [architecture.md](architecture.md).

Driving from the browser works by publishing `/cmd_vel`, which `drive_node` mixes into wheel commands. A browser gamepad (Gamepad API) can also drive; sensitivity is applied in the browser, so `drive_node`'s own `/sensitivity` and `/control_mode` topics are not used by this dashboard.

### Dependencies loaded from the internet

`index.html` loads `roslib`, `eventemitter2`, `three`, `ros3d` (jsDelivr), Leaflet (unpkg) and a Google font at page load. The base-station laptop therefore needs internet access, or these files must be vendored into `dashboard/`. The Leaflet map also loads OpenStreetMap tiles from the internet.

## `dashboard_helper` — process manager

Starts and stops the rover's nodes on demand from the dashboard, using one `SetBool` service per process (`/process_manager/toggle_<name>`) and publishing the state of every managed process as JSON on `/process_manager/status`. Process liveness is checked with a `pgrep`-style match on the command line. Each start runs through `bash -c "source /opt/ros/humble/setup.bash; source <repo>/ros2_ws/install/setup.bash; exec <command>"`.

Managed processes: robot state publisher, motion coordinator, drive node, VESC driver, both stepper drivers, four camera streams (CSI cam 0 at 30 and 60 fps, CSI cam 1, USB), GPS bridge, NAVnode, IMU telemetry and filter (individually or via the launch file), rover EKF, Morse recorder, Enigma node, XLR node and Morse bridge. The VESC process is started with the default wheel CAN IDs hard-coded in the helper.

## DDS / network configuration (`fastdds_profiles.xml`)

- Transport: UDPv4 only (built-in transports disabled, so shared memory is off — noted in the script as the fix for an inter-process-communication bug).
- Socket buffers: 1 MiB send and receive; maximum message size 65 500 bytes.
- Discovery: static initial peers at `127.0.0.1`, `192.168.1.87` and `192.168.1.88`.

To use a different network, edit those addresses. Machines not in the list will not discover the Jetson through this profile.

| Port | Protocol | Purpose |
|---|---|---|
| 9090 | TCP/WebSocket | rosbridge |
| 8000 | TCP/HTTP | dashboard static files |
| 5000, 5002, 5004 | UDP | camera streams to the base station |
| 5001 | UDP (localhost) | duplicate stream for `morse_recorder` |
| 5005 | UDP | remote autofocus trigger |

## Security note

rosbridge has no authentication and the helper can start arbitrary configured nodes. Keep the rover network private; do not expose ports 9090 or 8000 to the internet.
