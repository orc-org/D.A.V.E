# Arm, gripper and kinematics

Package: `arm_controller` (Python), interfaces in `arm_interfaces`.

```
 dashboard ──► arm_target (Point) ──► motion_coordinator ──► joint_states ──► arm_stepper_driver ─┐
               /auto_mode, /arm_manual_vel                  └► cmd_vel (auto mode)                   ├─► DM556Y stepper drivers
 dashboard ──► /gripper_target, /gripper_manual_vel ───────────────────────► arm_stepper_driver ───┘
                                                              ▲ /gripper_state ◄─────────────────────┘
```

## Kinematic chain (`kinematics.py`)

`Component3D` is one link in a tree. Each link has a translation offset $(o_x, o_y, o_z)$ from its parent and three joint angles $(\alpha, \beta, \gamma)$ about $x$, $y$, $z$, each with an optional limit (degrees) and a `fixed` flag.

### Forward kinematics

The local transform is a $4\times4$ homogeneous matrix: translation times $R_z R_y R_x$.

$$ T_{local} = \begin{bmatrix} I & o \\ 0 & 1\end{bmatrix}\begin{bmatrix} R_z(\gamma)R_y(\beta)R_x(\alpha) & 0\\ 0 & 1\end{bmatrix} $$

$$ T_{global} = T_{parent,global}\,T_{local},\qquad p = T_{global}[0{:}3,\ 3] $$

### The arm as configured (`motion_coordinator`, dimensions from `rover_description/config/physical_params.yaml`)

| Link | Parent | Offset | Free joints |
|---|---|---|---|
| Rover Base | – | world pose (x, y) | yaw only (x and y rotation limited to 0) |
| Column | Base | z = 0.10 m | fixed |
| Mount | Column | z = 0.40 m | fixed |
| Wrist | Mount | x = 0.30 m | **pitch ($\beta$) only** |
| Tool Tip | Wrist | x = 0.45 m | fixed |

So the arm has one actuated joint (wrist pitch) plus the rover's own position and heading; reaching a 3-D point therefore relies on driving the rover.

### Numeric inverse kinematics

`inverse_kinematics()` is a gradient-descent-like coordinate search on the distance $d = |p_{tool} - p_{target}|$. For each free axis of each joint, one at a time:

$$ g = \frac{d(\theta + \epsilon) - d(\theta)}{\epsilon},\qquad \theta \leftarrow \theta - \operatorname{clip}(\eta\,g,\ \pm\Delta_{max}) $$

with probe $\epsilon$ = 0.1°, learning rate $\eta$ (2.0 in auto mode), and a per-iteration angle clamp $\Delta_{max}$ = 1°. It stops when $d < 0.1$ (same length units as the offsets, i.e. metres) or after `max_iterations` = 20. Because it is coordinate-wise on finite differences, it is not guaranteed to converge and can stall in local minima.

With `move_base=True` it also drives the rover:

1. Turn toward the target: $\Delta\psi = \operatorname{wrap}(\operatorname{atan2}(y_t - y_b,\ x_t - x_b) - \psi_b)$, clamped to $\Delta_{max}$ per iteration.
2. Probe a 0.1 m step forward along the new heading, compute the same gradient, and move $-0.5\,\eta\,g$ along the heading, clamped to ±0.1 m.

It returns the total linear and angular change.

### From IK to `cmd_vel` (auto mode)

`motion_coordinator` runs at 20 Hz. In auto mode the IK deltas are turned into velocities, limited and low-pass filtered:

$$ u_v = \operatorname{clip}\!\left(\frac{\Delta s}{\Delta t}, \pm 0.8\ \text{m/s}\right),\ \ u_\omega = \operatorname{clip}\!\left(\frac{\Delta\psi}{\Delta t}, \pm 1.0\ \text{rad/s}\right),\qquad c_k = \alpha\,u + (1-\alpha)\,c_{k-1},\ \ \alpha = 0.15 $$

and published as `Twist` on `cmd_vel`, **only when `allow_base_motion:=true`** (default false since the 2026-11-15 review). Otherwise auto mode solves the IK for the arm alone (`move_base=False`, base pose restored afterwards) and publishes nothing on `cmd_vel`: the arm node used to be able to drive the rover on a made-up pose. Enable it only in simulation or with a real pose on `/ekf/odometry`. In manual mode the wrist angle integrates `/arm_manual_vel`.

### Base pose: measured when the EKF runs

When `/ekf/odometry` (parameter `odom_topic`) arrived less than 0.5 s ago, the base pose is taken from it, and the node publishes TF `world → odom` (identity) so that the tree is `world → odom → base_link` with `rover_ekf` owning `odom → base_link`. Otherwise the pose is **not measured**: the node integrates it from the `/motor/*` wheel commands and publishes `world → base_link` itself (it logs which mode it is in when it changes):

$$ v = \frac{(fl+rl) + (fr+rr)}{4}\cdot 3,\quad \omega = \frac{(fr+rr) - (fl+rl)}{4}\cdot 3 $$

$$ \psi \mathrel{+}= \omega\,\Delta t,\quad x \mathrel{+}= v\cos\psi\,\Delta t,\quad y \mathrel{+}= v\sin\psi\,\Delta t $$

The factor 3 is a "visual simulation" scale, not a calibration. After each IK solve the base pose is restored, so it is only ever updated by this integration. In this mode the arm's "global position" describes a simulated rover, not the GPS/EKF pose. `matplotlib` is loaded only for the optional debug plot (`show_plot` parameter); it used to be imported at start-up, so the node failed wherever matplotlib was not installed.

### Actions and services

- `calibrate_arm` (`CalibrateArm`): zeroes the base pose and wrist angle.
- `move_to_point` (`MoveToPoint`): sets the target point and immediately reports success; it does not wait for the arm to arrive.

## Stepper motion (DM556Y drivers)

### Conversions

Given gear ratio $G$, driver microsteps per motor revolution $S$ and lead-screw pitch $P$:

$$ \text{arm: } k_a = \frac{G\,S}{2\pi} = \frac{50\cdot 1600}{2\pi} \approx 12\,732\ \text{steps/rad} $$

$$ \text{gripper: } k_g = \frac{S}{P} = \frac{1600}{0.008\ \text{m}} = 200\,000\ \text{steps/m} $$

The gripper stroke is 0.2 m, which is $0.2\cdot 200\,000 = 40\,000$ steps (25 screw revolutions). The normalised gripper position $g \in [0,1]$ is `steps / (k_g · 0.2)`; commands convert with `finger_linear_pos = (1 − g_target) · 0.2` m.

### Step frequency

Nominal rates: arm 0.5 rad/s → $0.5\cdot 12\,732 \approx 6\,366$ Hz; gripper 0.05 m/s → $0.05\cdot 200\,000 = 10\,000$ Hz. The drivers use

$$ f_{step} = \max\big(10\,000\ \text{Hz},\ v_{max}\,k\big) $$

so the arm actually steps at 10 kHz (≈ 0.785 rad/s), faster than its nominal 0.5 rad/s unless the Arduino firmware limits it. Position is tracked by dead-reckoning step counts at the nominal rate; there is no encoder or limit-switch feedback.

### Two drivers, same interface

| Driver | Transport | Behaviour |
|---|---|---|
| `arm_stepper_driver` | Jetson GPIO | Sets DIR; holds STEP HIGH (or LOW if `active_low`) as a "move" trigger; an Arduino generates the actual pulses. Falls back to a no-hardware mode if `Jetson.GPIO` is missing. |
| `arm_stepper_arduino_driver` | USB serial | Sends `M <arm_dir> <arm_freq> <grip_dir> <grip_freq>\n` (integers), at least every 100 ms as a watchdog; `S\n` stops; reconnects every 3 s. |

Both subscribe to `joint_states`, expose the `gripper_command` action (`GripperCommand`), publish `/gripper_state`, and respect the same per-axis parameters (`arm_gear_ratio`, `arm_motor_steps`, `arm_max_velocity_rad_s`, `gripper_lead_pitch`, `gripper_motor_steps`, `gripper_max_velocity_m_s`, `*_invert_dir`). Pin assignments are in [hardware.md](hardware.md).
