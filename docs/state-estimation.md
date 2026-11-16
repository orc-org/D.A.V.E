# State estimation

Two independent filters exist today. They do not feed each other.

| Filter | Package/node | State | Inputs | Outputs |
|---|---|---|---|---|
| IMU unscented Kalman filter | `IMU/kalman_filter` | 15 | accel + gyro + BNO085 orientation | `/imu/data_filtered`, `/imu/filtered_euler` (`/imu/odometry` only with `publish_odometry:=true`) |
| Rover EKF | `rover_ekf/ekf_node` | 5 | wheel speed, IMU yaw and yaw rate, GPS | `/ekf/odometry`, `/ekf/filtered_euler`, TF `odom → base_link` |

## IMU telemetry (`telemetry`)

Reads a BNO085 on I²C bus 7, address `0x4A`, enabling accelerometer, gyroscope, rotation vector and magnetometer reports. It publishes at `publish_rate` (20 Hz default):

- `/imu/data_raw` — `Imu` with the sensor's raw quaternion, gyro (rad/s), accel (m/s²) and fixed diagonal covariances (accel 1.0, gyro 0.04, orientation 0.01).
- `/imu/euler` — roll, pitch, yaw in **degrees** after subtracting tare offsets.
- `/imu/telemetry_str` — formatted text for the dashboard.

Quaternion → Euler uses the standard ZYX (yaw-pitch-roll) conversion. The `~/calibrate` service stores the current yaw as a tare offset so that the current heading reads 0° ("true north"); `~/calibrate_all` tares roll and pitch as well. On an occasional read error it re-publishes the last valid frame; after about one second of consecutive failures it stops publishing and reconnects. Synthetic data (gravity on z, small sinusoids, identity orientation) is published **only** with `sim_mode:=true`, with a warning. Without it, a missing library or an unreachable sensor logs an error and publishes nothing, retrying the I²C connection every 2 s, so a dead IMU shows up as a silent topic rather than as a level, stationary rover (it used to fall back to synthetic data silently). `~/get_status` reports which state the node is in.

## IMU filter (`kalman_filter`) — 15-state AUKF

Implemented with `pykalman.AdditiveUnscentedKalmanFilter`.

**State**

$$ x = [\underbrace{p_x, p_y, p_z}_{\text{position}},\ \underbrace{v_x, v_y, v_z}_{\text{velocity}},\ \underbrace{a_x, a_y, a_z}_{\text{accel}},\ \underbrace{\phi, \theta, \psi}_{\text{roll, pitch, yaw}},\ \underbrace{g_x, g_y, g_z}_{\text{gyro}}]^\top $$

**Transition** (step $\Delta t$ = `dt`, damping $\gamma$ = `velocity_damping` = 0.95, gravity $g = 9.81$):

$$ g_b = g\,[-\sin\theta,\ \sin\phi\cos\theta,\ \cos\phi\cos\theta]^\top,\quad a' = a - g_b,\quad v^+ = \gamma v + a' \Delta t,\quad p^+ = p + \gamma v\,\Delta t + \tfrac12 a' \Delta t^2,\quad [\phi,\theta,\psi]^+ = [\phi,\theta,\psi] + g_{\text{gyro}}\Delta t $$

$g_b$ is gravity expressed in the body frame from the estimated roll $\phi$ and pitch $\theta$. (It used to be $[0,0,g]$ regardless of attitude, so on a slope $g\sin(\text{tilt})$ leaked into the horizontal acceleration: about 2 m/s of phantom velocity at rest on an 11.5° tilt.) Accel and gyro states are carried forward unchanged (random walk). Angles are wrapped to $[-\pi, \pi]$ after each update.

**Observation** (9-vector with `use_orientation:=true`, the default; 6-vector without):

$$ z = [a_x, a_y, a_z,\ g_x, g_y, g_z,\ \phi_m, \theta_m, \psi_m]^\top = h(x) = [x_{6:9},\ x_{12:15},\ x_{9:12}] $$

$(\phi_m, \theta_m, \psi_m)$ is the ZYX Euler conversion of the BNO085's fused quaternion in `/imu/data_raw` (the untared sensor frame, unlike `/imu/euler`). Each measured angle is unwrapped to within $\pi$ of the state before the update, so a heading crossing ±180° is a small step, not a 360° jump.

**Noise**: process $Q = 0.01\,I_{15}$ (`process_noise`), measurement $R = \operatorname{diag}(1,1,1,\ 0.04,0.04,0.04,\ \sigma^2_o,\sigma^2_o,\sigma^2_o)$ with $\sigma^2_o$ = `orientation_variance` = 0.01 rad², initial covariance $P_0 = 100\,I_{15}$, initial mean zero except $a_z = 9.81$.

**Step size.** The filter assumes a fixed $\Delta t$. The `imu.launch.py` launch file now sets `dt = 1 / publish_rate` so the filter step matches the telemetry period; run standalone, `dt` defaults to 0.1 s and must be set by hand.

**What it can and cannot do.** Orientation now follows the BNO085's own fusion, smoothed by the gyro, instead of drifting without bound as it did when only the gyro was integrated. Position and velocity are still integrals of accelerometer noise in the **body** frame (not rotated into a world frame), bounded only by the 0.95 damping, so `/imu/odometry` is **off by default** (`publish_odometry:=true` enables it with a warning) and must not be used for navigation. For heading, use `/imu/euler` (tared) or `/ekf/odometry`. Tested in WSL with synthetic data: 10 s at rest on an 11.5° tilt with the heading crossing ±180° every sample gave roll 0.200 rad, |yaw| 3.136 rad and a residual velocity of 0.06 m/s.

## Rover EKF (`ekf_node`) — 5-state

**State**

$$ x = [X,\ Y,\ \theta,\ v,\ \omega]^\top $$

$X, Y$ position in metres (east, north of the datum), $\theta$ heading in radians **counter-clockwise from east** (ENU), $v$ forward speed (m/s), $\omega$ yaw rate (rad/s). Runs at 20 Hz (`frequency`).

**Prediction** (every tick, with the **measured** step $\Delta t$ since the last tick, clamped to $[0, 0.5]$ s; it used to assume $1/f$, so a late or stalled timer mis-scaled every motion):

$$ X^+ = X + v\cos\theta\,\Delta t,\quad Y^+ = Y + v\sin\theta\,\Delta t,\quad \theta^+ = \operatorname{wrap}(\theta + \omega\,\Delta t),\quad v^+ = d\,v,\quad \omega^+ = d\,\omega,\quad d = 0.98^{\Delta t f} $$

Jacobian:

$$ F = \begin{bmatrix} 1 & 0 & -v\sin\theta\,\Delta t & \cos\theta\,\Delta t & 0\\ 0 & 1 & v\cos\theta\,\Delta t & \sin\theta\,\Delta t & 0\\ 0&0&1&0&\Delta t\\ 0&0&0&d&0\\ 0&0&0&0&d\end{bmatrix},\qquad P \leftarrow F P F^\top + Q\,\Delta t f $$

$Q = \operatorname{diag}(0.05, 0.05, 0.01, 0.1, 0.05)$ per nominal step. The damping $d$ (0.98 per nominal step) makes speed and yaw rate decay with no new measurements.

**Parameters added in the 2026-11-15 review:**

| Parameter | Default | Meaning |
|---|---|---|
| `imu_yaw_offset_deg` | 90 | $\theta = \psi_{imu} + $ offset. `/imu/euler` yaw is counter-clockwise with 0 where `~/calibrate` was called; the team tares facing true north, so 90 converts it to ENU. (It used to be taken as ENU directly, putting dead reckoning 90° off the GPS axes.) |
| `speed_per_motor_command` | 1.0 | m/s per unit `/motor/*` command: 1.0 for `vesc_can_driver_node` in metric mode, 1.5 for the simulator's `sim_drive_bridge` |
| `fuse_cmd_vel` | false | `/cmd_vel` was fused as a speed measurement *in addition to* `/motor/*` (the same command counted twice); now off |
| `datum_latitude`, `datum_longitude` | NaN | origin of $X, Y$; NaN = first GPS fix. Set to NAVnode's home so both share a frame |
| `use_gps` | true | false = GNSS is not subscribed (Meteorite task) |

**Measurement updates** (standard EKF form $K = P H^\top (H P H^\top + R)^{-1}$, $x \leftarrow x + K(z - Hx)$, $P \leftarrow (I - KH)P$; angle innovations are wrapped to $[-\pi, \pi]$):

| Source | $z$ | $H$ | $R$ |
|---|---|---|---|
| IMU yaw (`/imu/euler`, degrees → rad, + `imu_yaw_offset_deg`) | $\theta_{imu}$ | selects $\theta$ | 0.01 |
| IMU yaw rate (`/imu/data_raw`) | $\omega_z$ | selects $\omega$ | 0.005 |
| Wheel speed (`/motor/*` mean × `speed_per_motor_command`, while a command arrived < 0.5 s ago, zero included) | $v_{wheel}$ | selects $v$ | 0.08 |
| GPS (`/gps/fix`) | $[x_{east}, y_{north}]$ | selects $X, Y$ | the message's east/north variances when `position_covariance_type` is not UNKNOWN (from `gps_bridge_node`: $(\text{HDOP} \cdot \text{UERE})^2$), otherwise $\operatorname{diag}(1.5, 1.5)$; messages with `status < 0` (no fix) are ignored |

Notes on the inputs:

- **Wheel speed** is the mean of the four `/motor/*` values, $(fl + rl + fr + rr)/4$, scaled by `speed_per_motor_command`. These are **commands**, not measurements (there is no encoder or VESC feedback yet), so wheel slip and a stalled motor are invisible. A zero command is fused too (it used to be skipped, so a stopped rover kept coasting on $v$); once commands are older than 0.5 s (the drive deadman), nothing is fused.
- **GPS** uses `datum_latitude`/`datum_longitude` as the origin, or the first valid fix if they are not set (then the origin is *not* NAVnode's home waypoint), and an equirectangular approximation with Earth radius $R_E$ = 6 371 000 m: $x_{east} = \Delta\lambda\,R_E\cos\varphi_0$, $y_{north} = \Delta\varphi\,R_E$, with deltas in radians. Fixes with status < 0, NaN, or both coordinates ≈ 0 are ignored.
- **Output**: `Odometry` (position and heading; 6×6 pose covariance filled from $P$ for $X,Y,\theta$ only), Euler angles in degrees (roll/pitch passed straight through from the IMU), and the `odom → base_link` TF.

### Fixes made in this repository

- The wheel speed was `(fl+rl+fr+rr)*0.5`, which is twice the four-wheel mean. It is now `*0.25`.
- The yaw-rate update used `z = [θ_state, ω_imu]`, i.e. it fed the state's own heading back as a measurement. The innovation is zero, but the covariance of $\theta$ still shrinks on every IMU message, making the filter over-confident. It now updates $\omega$ only. The yaw update likewise updates $\theta$ only.
- 2026-11-15: measured $\Delta t$, yaw offset to ENU, commands fused once and including zero, configurable datum (table above). Tested in WSL: yaw 0 (north tare) → θ = 89.7° after 30 updates; 1 s at command 0.5 heading north → $Y$ = 0.51 m, $X$ = 0.00 m; zero commands → $v$ = 0.000; a 0.3 s timer gap at 1 m/s → 0.300 m.

### Still to check on the rover

The offset assumes `/imu/euler` yaw increases when the rover turns **left** (counter-clockwise seen from above), which is what the ZYX conversion gives for a z-up sensor. Check it once: tare facing north, turn left, and `/imu/euler` z should increase. If it decreases (sensor mounted upside down), the offset alone cannot fix it; the sign must be flipped in `telemetry.py`. If the tare is not done facing **true** north, the offset must include that error (and the magnetic declination if the tare uses a compass: roughly 13–14° east near Drumheller, AB; look up the current value from the NOAA/NRCan calculator for the competition date).
