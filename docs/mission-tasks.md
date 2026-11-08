# Mission tasks and peripherals

This page covers the Morse/Enigma task (CIRC 2026 "heist"), camera streaming, lights, camera servos and the controller viewer.

## Morse code: recording a flashing light → text

Node: `morse_recorder.py` (package `enigma_machine`). Video comes from UDP port 5001 on localhost by default (the second output of `video_streamer`), or from an Image topic (`/image_raw`) or a local camera (`source` parameter: `udp`, `topic`, `camera`).

### Light detection

Each frame is downscaled to about 320×180, converted to grey, and thresholded at `brightness_threshold` (default 240). The light is "on" when

$$ N_{bright} = \#\{\text{pixels} \ge 240\} > \text{pixel\_count\_threshold}\ (150) $$

`N_bright` is published on `/morse_recorder/pixel_count` so the dashboard can be used to calibrate both thresholds; they can be changed at run time with `ros2 param set`.

### Timing → dits and dahs

Pulse timing is self-calibrating; no absolute clock is assumed. A sliding window keeps the last 20 "on" durations. With $d_{min}$ the shortest duration in the window:

| Measurement | Rule | Symbol |
|---|---|---|
| Pulse of duration $d$ | $d > 2\,d_{min}$ | dah `-` (called "daw" in the code) |
| | otherwise (and always the very first pulse) | dit `*` |
| Gap before the next pulse, $g$ | $g > 5.5\,d_{min}$ | word gap: two spaces |
| | $g > 1.8\,d_{min}$ | letter gap: one space |

If no light is seen for more than 10 s, the buffer is cleared when the next message starts. The string is published on `/morse_code` (`enigma_machine_interfaces/Morse`) and `/morse_code_str` (`String`) after every symbol, and `~/reset` clears it manually.

A weakness: the first pulse is assumed to be a dit. A message beginning with a dah mislabels the baseline until a shorter pulse is seen.

### String format

Dits are `*`, dahs are `-`, one space separates letters, two spaces separate words.

## Enigma machine (`enigma_node`, C++)

Written by Isaac Meyer (March 2026) for the 2026 CIRC mission: read the terminal password from a flashing light, and type the vault password on a physical Morse key.

- **Decode** (Morse → English): a binary trie. Each dit or dah selects one of the two children; the node reached at a letter gap holds the letter. Lookup cost is $O(\text{symbols in the letter})$.
- **Encode** (English → Morse): an array indexed directly by ASCII code, so each character is $O(1)$; the Morse for a message is built by concatenation. A configurable suffix ("stop") can be appended depending on `encodeCase`.
- **Interfaces**: services `GetPassword`, `SetMessage`, a `Trigger` for transmitting the cached message, and `Decode` / `Encode` actions. The node subscribes to `Morse` messages from the recorder and stores the decoded password. Encoded messages are published as a string on `/morse_command`.
- **Actuation**: `morse_bridge_node` (in `serial_communication`) forwards `/morse_command` to an Arduino over serial (default `/dev/ttyUSB0`, 115200 baud) and exposes the `morse_converse` service. The Arduino drives the servo that presses the Morse key. A standalone Jetson sysfs-PWM program, `jetson_morse.cpp` (pin 32), was an earlier approach; `scripts/hardware_tests/pwmtest.cpp` is its test.

The encode and decode actions run their work on detached threads.

## Camera streaming (`rover_video_streamer`)

`video_streamer.py` captures a camera and sends H.264 (x264enc) or MJPEG over UDP using GStreamer.

| Argument | Default | Meaning |
|---|---|---|
| `--host` | `192.168.1.10` | Receiver IP (the process manager overrides this with `192.168.1.87`) |
| `--port` | 5000 | UDP port to the receiver |
| `--width` / `--height` / `--fps` | 1280 / 720 / 30 | Capture size and rate |
| `--bitrate` | 2000 | H.264 kbps |
| `--camera-type` | `v4l2` | `v4l2`, `csi` (Jetson `nvarguscamerasrc`) or custom `gstreamer` |
| `--device` | `0` | Index or V4L2 path |
| `--source` | `camera` | `camera`, or `topic` to relay an Image topic |
| `--secondary-port` | 5001 | Duplicate stream to `127.0.0.1` for `morse_recorder` |
| `--trigger-port` | 5005 | UDP port listening for remote one-shot autofocus triggers |

CSI cameras 0 and 1 have voice-coil focus motors reachable on I²C buses 10 and 9, address `0x0c`; the streamer can sharpen focus automatically by measuring image sharpness. Process-manager presets stream CSI cam 0 on 5000 (also a 60 fps variant), CSI cam 1 on 5002 and a USB camera on 5004.

## Lights

`light_package/lights.py` listens on `/lights` (`Bool`) and drives header pin 12 HIGH or LOW. The pin is set LOW at start-up and on shutdown.

## Camera servos

`servo_controller/servo_node.py` listens on `servo_commands` (`String`) and drives two on/off direction pairs (not PWM positioning): for servo 1 or 2, `servoN_left` sets the left pin HIGH and the right pin LOW, `servoN_right` the reverse, and `servoN_stop` sets both LOW; `stop_all` stops everything. Pins: servo 1 = 24 (left) / 23 (right), servo 2 = 21 (left) / 37 (right). These are bang-bang motion commands for a continuous-rotation pan/tilt mechanism or an H-bridge, so position has to be inferred from the camera.

## Controller viewer

`input_viewer_package` subscribes to `/joy` and publishes `/joy_markers` for RViz so you can see button and stick state next to the robot model.
