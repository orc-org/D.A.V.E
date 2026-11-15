# State estimation

Two independent filters exist today. They do not feed each other.

| Filter | Package/node | State | Inputs | Outputs |
|---|---|---|---|---|
| IMU unscented Kalman filter | `IMU/kalman_filter` | 15 | accel + gyro | `/imu/data_filtered`, `/imu/odometry`, `/imu/filtered_euler` |
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

$$ a' = a - [0, 0, g]^\top,\qquad v^+ = \gamma v + a' \Delta t,\qquad p^+ = p + \gamma v\,\Delta t + \tfrac12 a' \Delta t^2,\qquad [\phi,\theta,\psi]^+ = [\phi,\theta,\psi] + g_{\text{gyro}}\Delta t $$

Accel and gyro states are carried forward unchanged (random walk).

**Observation** (6-vector):

$$ z = [a_x, a_y, a_z, g_x, g_y, g_z]^\top = h(x) = [x_{6:9},\ x_{12:15}] $$

**Noise**: process $Q = 0.01\,I_{15}$ (`process_noise`), measurement $R = \operatorname{diag}(1,1,1,0.04,0.04,0.04)$, initial covariance $P_0 = 100\,I_{15}$, initial mean zero except $a_z = 9.81$.

**Step size.** The filter assumes a fixed $\Delta t$. The `imu.launch.py` launch file now sets `dt = 1 / publish_rate` so the filter step matches the telemetry period; run standalone, `dt` defaults to 0.1 s and must be set by hand.

**What it can and cannot do.** Because both transition and observation are linear, the unscented transform adds cost but no accuracy over a plain Kalman filter. More importantly, the orientation states $(\phi,\theta,\psi)$ are only ever driven by integrating the gyro and are never observed, so roll, pitch and yaw drift without bound (the sensor's own fused quaternion is ignored). Gravity is removed along the world $z$ axis regardless of attitude, and position/velocity are double/single integrals of accelerometer noise, bounded only by the 0.95 damping. Treat `/imu/odometry` as a toy; use `/imu/euler` (the BNO085's own fusion) for orientation.

## Rover EKF (`ekf_node`) — 5-state

**State**

$$ x = [X,\ Y,\ \theta,\ v,\ \omega]^\top $$

$X, Y$ position in metres (east, north of the first GPS fix), $\theta$ heading in radians, $v$ forward speed (m/s), $\omega$ yaw rate (rad/s). Runs at 20 Hz (`frequency`).

**Prediction** (every tick, step $\Delta t = 1/f$):

$$ X^+ = X + v\cos\theta\,\Delta t,\quad Y^+ = Y + v\sin\theta\,\Delta t,\quad \theta^+ = \operatorname{wrap}(\theta + \omega\,\Delta t),\quad v^+ = 0.98\,v,\quad \omega^+ = 0.98\,\omega $$

Jacobian:

$$ F = \begin{bmatrix} 1 & 0 & -v\sin\theta\,\Delta t & \cos\theta\,\Delta t & 0\\ 0 & 1 & v\cos\theta\,\Delta t & \sin\theta\,\Delta t & 0\\ 0&0&1&0&\Delta t\\ 0&0&0&0.98&0\\ 0&0&0&0&0.98\end{bmatrix},\qquad P \leftarrow F P F^\top + Q $$

$Q = \operatorname{diag}(0.05, 0.05, 0.01, 0.1, 0.05)$. The 0.98 factors are a damping model: with no new measurements the speed and yaw rate decay.

**Measurement updates** (standard EKF form $K = P H^\top (H P H^\top + R)^{-1}$, $x \leftarrow x + K(z - Hx)$, $P \leftarrow (I - KH)P$; angle innovations are wrapped to $[-\pi, \pi]$):

| Source | $z$ | $H$ | $R$ |
|---|---|---|---|
| IMU yaw (`/imu/euler`, degrees → rad) | $[\theta_{imu},\ \omega]$ | rows selecting $\theta$ and $\omega$ | $\operatorname{diag}(0.01, 0.005)$ |
| IMU yaw rate (`/imu/data_raw`) | $\omega_z$ | selects $\omega$ | 0.005 |
| Wheel speed (`/motor/*` mean, or `cmd_vel.linear.x`) | $v_{wheel}$ | selects $v$ | 0.08 |
| GPS (`/gps/fix`) | $[x_{east}, y_{north}]$ | selects $X, Y$ | $\operatorname{diag}(1.5, 1.5)$ |

Notes on the inputs:

- **Wheel speed** is the mean of the four `/motor/*` values, $(fl + rl + fr + rr)/4$. These are the commanded wheel speeds in m/s when the VESC driver is in metric mode (see [drive-system.md](drive-system.md)); there is no encoder feedback.
- **GPS** uses the first valid fix as the origin (the origin is *not* the navigation system's home waypoint) and an equirectangular approximation with Earth radius $R_E$ = 6 371 000 m: $x_{east} = \Delta\lambda\,R_E\cos\varphi_0$, $y_{north} = \Delta\varphi\,R_E$, with deltas in radians. Fixes with status < 0, NaN, or both coordinates ≈ 0 are ignored.
- **Output**: `Odometry` (position and heading; 6×6 pose covariance filled from $P$ for $X,Y,\theta$ only), Euler angles in degrees (roll/pitch passed straight through from the IMU), and the `odom → base_link` TF.

### Fixes made in this repository

- The wheel speed was `(fl+rl+fr+rr)*0.5`, which is twice the four-wheel mean. It is now `*0.25`.
- The yaw-rate update used `z = [θ_state, ω_imu]`, i.e. it fed the state's own heading back as a measurement. The innovation is zero, but the covariance of $\theta$ still shrinks on every IMU message, making the filter over-confident. It now updates $\omega$ only.

### Open question for the team

The prediction treats $\theta$ as counter-clockwise from the $+X$ (east) axis. The IMU `~/calibrate` service tares yaw so that the current heading is 0° "north". If heading 0 is north in practice, the dead-reckoned $X, Y$ are rotated 90° relative to GPS and the filter will fight itself. Confirm the yaw convention on the rover (for example by driving north and watching `/imu/euler`) before trusting the fused output.
