# Drive system

Four wheels, one VESC motor controller per wheel, skid-steer (left pair and right pair always get the same command). Two nodes in `drive_package`:

```
/joy  ─┐
       ├─► drive_node ─► /motor/front_left, rear_left, front_right, rear_right (Float32, −1..1)
/cmd_vel ┘                                   │
                                             ▼
                              vesc_can_driver_node ─► CAN0 ─► VESCs
```

## `drive_node` — command mixing

Inputs: `/joy`, `/cmd_vel`, `/control_mode`, `/sensitivity`. Outputs: four `Float32` topics, one per wheel. Left wheels share one value and right wheels share another.

### Joystick (`/joy`)

Axis 0 is the left stick X (`steering`, positive = left in ROS `Joy`), axes 4 and 5 are the left and right triggers (rest = +1, fully pressed = −1).

Trigger normalisation and throttle (`right_trigger_forward:=true` default):

$$ t_L = \frac{1 - a_4}{2},\quad t_R = \frac{1 - a_5}{2},\quad \text{throttle} = t_R - t_L \in [-1, 1] $$

Right trigger (R2) is forward and left trigger (L2) is reverse, matching `dashboard/app.js` (set `right_trigger_forward:=false` only if legacy inverted polarity is explicitly needed; see [known-issues.md](known-issues.md)). If $|\text{throttle}| < 0.05$ it is set to 0.

If the throttle is non-zero, differential mixing scales by $|\text{throttle}|$ so $+s$ (left stick) turns CCW consistently both forward and in reverse:

$$ \text{left} = \text{throttle} - |\text{throttle}|\,s,\qquad \text{right} = \text{throttle} + |\text{throttle}|\,s $$

If the throttle is zero, the stick alone spins the rover in place (with its own 0.05 deadzone):

$$ \text{left} = -s,\qquad \text{right} = +s $$

Both values are then multiplied by the sensitivity factor and clamped to $[-1, 1]$. Messages with fewer than 6 axes or non-finite values stop the motors.

### Velocity commands (`/cmd_vel`)

For `Twist` input (`cmd_vel_is_si:=true` default, with `wheel_separation = 0.60 m` and `max_wheel_speed = 1.0 m/s`):

$$ \text{left} = \frac{v - \omega\,(W / 2)}{v_{\max,\text{wheel}}},\qquad \text{right} = \frac{v + \omega\,(W / 2)}{v_{\max,\text{wheel}}} $$

where $v$ is `linear.x` (m/s) and $\omega$ is `angular.z` (rad/s); sensitivity is applied and the result is clamped to $[-1, 1]$. Idle near-zero `/cmd_vel` packets yield to active `/joy` driving for `0.4 s` on `RCL_STEADY_TIME` so a background `10 Hz` zero heartbeat does not fight controller driving.

### Mode and sensitivity

| Topic | Values | Effect |
|---|---|---|
| `/control_mode` (String) | `"drive"` or anything else | Anything other than `"drive"` zeroes all four wheels and ignores `/joy` and `/cmd_vel`. Default is `"drive"`. |
| `/sensitivity` (String) | `"ultra"` / `"fine"` / `"coarse"` | Scale factor 0.25 / 0.5 / 1.0. Default 1.0. |

No node in this repository currently publishes `/control_mode` or `/sensitivity` automatically; they are actuated externally when locking out drive during arm tasks or toggling teleop precision (`vesc_can_driver_node` also enforces `/control_mode`).

## `vesc_can_driver_node` — wheel command → VESC

Subscribes to the four `/motor/*` topics, ramps each command toward its target, and writes one extended-ID CAN frame per wheel at `publish_rate` (50 Hz).

**Deadman (`command_timeout`, default 0.5 s).** A wheel whose last command is older than this is set to zero at once, without ramping, and a warning is logged. Every command source must therefore publish continuously: the dashboard sends `/cmd_vel` at 10 Hz in manual mode and `joy_node` repeats at 20 Hz by default, while a node that publishes only on change will be stopped after 0.5 s. The wheels also stay stopped until the first command arrives. The timeout uses a steady clock, so it is unaffected by simulation time. `command_timeout:=0` disables it (bench tests only). The simulator's `sim_drive_bridge` has equivalent 0.3 s / 0.5 s timeouts.

**Measured wheel speed (`/wheel_states`).** Each loop the driver also reads the VESCs' CAN STATUS frames (packet 9, extended ID $(9 \ll 8) \,|\, \text{id}$; 8 bytes big-endian: int32 ERPM, int16 current × 10, int16 duty × 1000) and publishes `sensor_msgs/JointState` on `/wheel_states` with the joint names `front_left_wheel_joint` … `rear_right_wheel_joint`:

$$ \omega_{wheel} = \frac{\text{ERPM}}{p \cdot G}\cdot\frac{2\pi}{60}\ \text{rad/s},\qquad \text{effort} = \text{motor current (A)} $$

with $p$ = `pole_pairs`, $G$ = `gear_ratio`. A wheel without a STATUS frame for `status_timeout` (0.5 s) is left out; with none reporting, nothing is published. `rover_ekf` turns it into forward speed with its own `wheel_radius` (mean of the reporting wheels × radius) and prefers it over the commands.

Rover set-up (not done in code): in VESC Tool, enable **CAN status message STATUS 1** on every VESC (for example at 50 Hz). Check the sign once: drive forward slowly and every `/wheel_states` velocity must be positive. A VESC whose motor direction is inverted in its configuration may report the opposite sign. Tested on a virtual CAN bus in WSL only (ERPM 9549 → 10.0 rad/s → EKF 0.98 m/s).

### Ramp limiting

Each tick the commanded value may change by at most

$$ \Delta_{\max} = \text{ramp\_rate}\cdot\frac{1}{\text{publish\_rate}} = 1.0 \cdot 0.02 = 0.02 $$

so going from 0 to full takes 1 s and full forward to full reverse takes 2 s.

### Control modes (`control_mode` parameter)

| Mode | VESC packet (`comm_id`) | Value sent |
|---|---|---|
| `erpm` (default) | `SET_RPM` = 3, or `SET_CURRENT_BRAKE` = 2 (`brake_current = 5.0 A`) on zero/timeout (`SET_CURRENT` = 1 value 0 if `brake_on_zero:=false`) | electrical RPM or brake hold, see below |
| `current` | `SET_CURRENT` = 1 | `cmd × max_current × 1000` (milliamps; `max_current` = 14 A) |
| anything else (duty) | `SET_DUTY` = 0 | `cmd × max_duty × 100000` (`max_duty` = 0.95) |

### ERPM formula (metric mode, the default)

With `use_metric_speed = true`, the wheel command is interpreted as a ground speed in m/s:

$$ \text{ERPM} = v\cdot\frac{60}{2\pi r}\cdot p\cdot G $$

with wheel radius $r$ = 0.15 m (loaded from `rover_description/config/physical_params.yaml`), pole pairs $p$ = 2 and gearbox ratio $G$ = 50. That is **6366.2 ERPM per m/s**, so a full-stick command of 1.0 asks for 1 m/s (≈ 6366 ERPM; confirm physical tire radius and VESC `pole_pairs` / `gear_ratio` on hardware).

With `use_metric_speed = false`, the command is a fraction of `max_erpm` (7000): $\text{ERPM} = \text{cmd}\cdot 7000$.

Behaviours in `erpm` mode:

- On deadman timeout, non-`"drive"` `/control_mode`, NaN input, $|v| < 0.01$, or $|\text{ERPM}| < 10$, `SET_CURRENT_BRAKE` (`5000 mA` by default when `brake_on_zero = true`) is sent to hold position instead of coasting (`SET_CURRENT` = 0 when `brake_on_zero = false`).
- Otherwise, if $|\text{ERPM}| < \text{min\_erpm}$ (100) it is raised to ±100 for stable start-up commutation.
- `max_erpm` (7000 default) **is** enforced as a ceiling (`[-max_erpm, +max_erpm]`) in both metric and normalized modes. See [guardrails-and-limits.md](guardrails-and-limits.md).

### CAN frame layout

```
29-bit extended ID  = (comm_id << 8) | can_id
DLC = 4
payload             = int32 value, big-endian
```

Wheel CAN IDs default to front-left 53, front-right 44, rear-left 48, rear-right 7 (a negative ID disables that wheel). SocketCAN on `can0`; if the socket cannot be opened the node logs a warning and runs in simulation mode (it publishes nothing to the bus).

## Related

CAN setup commands are in [hardware.md](hardware.md); the full package README is at [`ros2_ws/src/drive_package/README.md`](../ros2_ws/src/drive_package/README.md).
