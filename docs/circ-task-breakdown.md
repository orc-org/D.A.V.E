# CIRC 2027 programming task breakdown (template draft)

Filled in from [competition.md](competition.md) (a paraphrase of the organisers' tasks page, read 2026-11-09; **re-check against https://circ.cstag.ca/2027/tasks before submitting**) and the repository state. Items marked **[?]** are our inference, not something the organisers said. "Who owns what" is left out on purpose.

## Shared systems (referenced by every task below)

**Terrain-mapping stack** ([slam/architecture.md](slam/architecture.md)): three independent layers, each with a kill switch, so the lower layers stay valid if a higher one is switched off.

1. **Layer 1: LiDAR-inertial odometry** (Unitree L2 cloud + IMU; Point-LIO / KISS-ICP, chosen by bench-off). Gives pose with no GNSS. No TF output; `rover_ekf` owns `odom → base_link`.
2. **Layer 2: baseline planner map.** 2.5-D elevation grid (height, slope, roughness, step per cell) published as `/terrain/traversability` and fed to a Nav2 costmap. This is the map the rover actually drives on and it must work with no research risk.
3. **Layer 3: the novel part, a LiDAR-seeded surfel-Gaussian terrain map.** Voxel-hash the deskewed cloud; each occupied voxel becomes one flat 2DGS-style disk (mean, normal from a PCA of the voxel's points, two tangent scales, opacity). Traversability comes from the disks directly: slope = `arccos(n_z)`, roughness = spread of normals and height residuals, step = height difference to neighbours, confidence = Gaussian count/opacity per cell (unknown is not free). No semantic network, a Gaussian cap sized to the 8 GB budget, full map in CPU memory with only the working set on the GPU. Closest prior art is Splatblox (needs CLIPSeg, 479 ms/frame, laptop GPU, no code) and GS-LIVO (mapping, not traversability). Gated by experiment E3b: it ships only if it beats the elevation baseline within budget.
4. **Two-tier "teacher/student" strategy** ([slam/decisions.md](slam/decisions.md)): a heavy research tier on the workstation (full Gaussian maps, semantic labels, from recorded bags) produces labels that train a tiny per-cell traversability model (gradient-boosted trees or small MLP) that runs on the Jetson CPU in milliseconds. Features include **LiDAR intensity** (a free per-point reflectivity cue for rock vs clay vs wet) so it works without a camera.

Honest status: none of this has run on the Jetson. A KISS-ICP check in simulation gave 0.43 m ATE over 16.5 m; that proves the pipeline, not performance. Layers 1-2 are the priority; no task strictly requires Layer 3, it is the differentiator.

**Depth camera (possible, not bought; design to the sensor contract so any model fits).** Three jobs: (a) near-field terrain the LiDAR misses (blind zone under/ahead of the mount, arm shadow) fused into the elevation grid with its own range/noise model; (b) ArUco pose with real depth, which tightens `[Drive to Marked Waypoint]` and the marker-relative arm moves; (c) a wrist/arm camera for grasp and press alignment, giving a measured target position instead of a guessed one. Caveats: active infrared depth degrades in sunlight and is an **emitter** (a problem in Prison Break; a passive stereo camera is the safe choice there), and its time offset to the LiDAR must be measured. Everything must still run with the camera absent.

**GPS/GNSS** is the global anchor in every task except Meteorite (banned): NMEA to decimal degrees to local East-North, fused in `rover_ekf` with LIO so drift does not accumulate; declination applied for true-north bearings; a fix-quality gate so a bad fix is never trusted over LIO.

**Arm kinematics** (`arm_controller/kinematics.py`): today one actuated joint (wrist pitch) with the rover base supplying the other degrees of freedom, and a coordinate-wise numeric IK (20 iterations, can stall). Needs: an end-effector frame at the CIRC reference mark, conversion to the rules' frame (+x forward, +y right, +z up from the body front), reach/payload limits (5 kg, 90 mm grip, 1 m table), and a measured pose for `[EFF Report]`. A real multi-joint chain with analytic or damped-least-squares IK would remove the base-motion coupling; that is a design decision for the arm.

**Focus-area matrix** (where each area does the heavy lifting; ● primary, ○ supporting):

| Area | Mining | Demolition | Meteorite | Prison Break | Reactor |
|---|---|---|---|---|---|
| LiDAR mapping / odometry | ● prior map + slope | ● obstacles, odometry | ● only global pose | ○ only if emission allowed | ● rough terrain |
| Autonomy (planner, controller) | ○ assist/limits | ● fully autonomous | ● route + return | ○ keep-out, stop-and-go | ● storm keep-out |
| GPS | ○ | ● | ✗ banned | ○ | ● |
| Arm kinematics | ○ beacon | ● EFF operations | ● sample/pickup | ● key/wire/buttons | ● debris/lid/table |
| Depth camera | ○ near-field | ● markers, grasp | ○ sampling | ● passive stereo only | ○ near-field, grasp |

**Closed-loop drive** (missing today): `NAVnode` computes range/bearing but never commands the wheels; `drive_node` accepts `/cmd_vel`. Nav2 (or a small controller) on odometry must be built; Demolition and Reactor Recovery both depend on it.

**ArUco node** (missing): OpenCV `cv2.aruco`, dictionary `DICT_4X4_100`, markers at least 5 cm, results shown automatically on the UI.

**Dashboard / UI**: rosbridge web dashboard; buttons labelled `[shortname] long name`, text boxes and returned values with units. Control and telemetry must not depend on video.

---

## 1. Mining Site Remote Operations

*Source: live tasks page, read 2026-11-11. Not specified there: point values, penalty amounts, the actual bandwidth cap (50-100 Mbit/s, announced at start), the beacon signal threshold, beacon/scale/dispenser documents (given in advance).*

**Goal in plain English:** Driven entirely by operators at CIRC Central over the CIRC link, D.A.V.E. places a relay beacon near the ore dispenser to unlock it, loads washed pea rock (3-10 mm) from the dispenser (run by base operators), hauls it up and down a steep grade to the weigh scale, and unloads it, repeating until it reaches the 100 kg scoring cap without spilling any.

### 1. The inputs (what the rover needs to know)

**From judges / control station**
- Operator commands over the CIRC link: drive (`cmd_vel`), arm and gripper targets, bed/unload trigger, sensor mode switches (all command, control and video must use the CIRC link).
- Announced at the start: the bandwidth cap (50-100 Mbit/s) and the beacon signal-strength threshold.
- Given in advance: beacon size/mass, weigh scale and dispenser documents, the **2018 LiDAR point cloud (Potree 2.0)** of the site. The 2027 terrain will differ.
- Link-readiness counters from the site test endpoint (packets, data rate, throughput).
- A shared coarse live video feed (fallback driving view only).
- Whether the dispenser is unlocked (told by judges/base operators; not a rover signal **[?]**).

**From sensors**
- **LiDAR (L2) cloud + IMU:** live terrain, odometry, slope, obstacles.
- **Depth camera (possible):** near-field terrain, spout and scale alignment, bed fill height.
- **GNSS:** global position on the site (open sky expected), used to register the rover to the 2018 point cloud and to give the operator a map position.
- **IMU / odometry:** pitch and roll (rollover margin on the grade), heading.
- **Drive feedback:** VESC wheel speed, motor current, temperature, battery voltage and current.
- **Arm/gripper state:** joint angles, gripper position, payload current (beacon held or not).
- **Link health:** round-trip time, throughput, loss, time since last operator packet.
- **Load estimate:** motor current/slip and bed fill height; a load cell if fitted **[?]**.

### 2. The outputs / actions (what the robot commands)

**Mobility**
- Drive and turn on operator commands, passed through a slope/tilt-aware limiter (speed and torque capped by pitch, roll, load and motor temperature).
- Stop and hold on the grade (brake current), hold station while the dispenser fills the bed.
- Autonomous safe behaviours (not operator commands): ramp to zero on link loss, tilt-triggered stop, optional assisted "return along the path I came" **[I]**.
- Tow and return the trailer if one is used.

**Manipulation**
- Pick up the relay beacon, carry it, place it at the chosen spot near the dispenser, release and back away.
- Bed unload / tip at the scale (automatic or rover-performed; manual unload is a modification intervention).
- Optional: small arm tasks to clear a blocked spout or reposition the trailer **[?]**.

**Feedback to operators**
- Telemetry on a low-rate channel that works without video: pose, pitch/roll, slope ahead, battery, motor current/temperature, estimated load mass, link stats, arm state.
- Video at an adaptive bitrate.
- Map view: live elevation/traversability overlay, difference from the 2018 point cloud, planned route, "too steep / too rough" cells, current rover position and breadcrumb path.
- Alerts: link degraded, tilt margin low, overcurrent/thermal, bed nearly full.

### 3. Logic pipeline

#### 3a. Logical architecture (what systems, how they connect)

```
CIRC Central (operators)
     |  CIRC Backhaul (only path; 50-100 Mbit/s cap; our last-hop radio)
     v
[Comms / UI system]  rosbridge (offline dashboard, authenticated), telemetry channel
     |                video channel (adaptive bitrate)         ^ telemetry, alerts
     v                                                          |
[Safety supervisor]  link watchdog, tilt/thermal/current limits, e-stop --> overrides everything below
     |
     +--> [Drive system]  cmd_vel -> drive_node -> VESCs; slope/load-aware limiter
     +--> [Arm system]    target -> IK -> steppers; beacon pick/place; (unload mechanism)
     ^
     |  pose, terrain cost, slope
[State estimation]  rover_ekf: GNSS + IMU + wheel speed + LiDAR-inertial odometry -> map/odom/base_link
     ^                  ^
     |                  |
[LiDAR mapping stack]   [Depth camera]  near-field cloud, spout/scale/bed view
   Layer 1 LIO odometry
   Layer 2 elevation grid  <-- registered with the 2018 Potree cloud (prior map)
   Layer 3 surfel-Gaussian terrain map (optional) -> slope, roughness, step, confidence
```

Systems, each with its job:

- **Comms / UI system:** the dashboard must work offline (no CDN libraries or online tiles), rosbridge must be authenticated, and control plus telemetry must not depend on video. Video bitrate adapts to measured throughput (it is fixed at start today). Use the site test endpoint to confirm the link is ready before each attempt. Interference tolerant: another team shares the site and spectrum is not coordinated.
- **Safety supervisor (independent of the planner):** link watchdog, tilt limit from the IMU, motor current and temperature limits, e-stop. It can only reduce commands, never add them.
- **State estimation:** `rover_ekf` extended with the LiDAR-inertial velocity and pose updates (see [slam/architecture.md](slam/architecture.md)); GNSS anchors it globally so it can be registered to the 2018 cloud.
- **LiDAR mapping stack (the novel part):**
  1. Convert the 2018 Potree cloud to the same elevation-grid format (Layer 2) and register it to the site using GNSS plus a scan-to-map alignment, giving a **prior terrain map and prior slope field** for the whole route before the rover moves.
  2. Live Layer 1 odometry plus Layer 2 updates overwrite the prior where it is wrong. Prior cells carry low confidence, so unknown or changed terrain is never trusted as free.
  3. Layer 3 (optional) adds surfel normals for finer slope and roughness on the grade, and a "changed since 2018" layer (live height minus prior height) for the operators.
  4. The student traversability model (geometry + LiDAR intensity) scores loose gravel vs firm ground.
- **Depth camera system (possible):** near-field gap filling, bed fill height, spout/scale alignment.
- **Drive system:** `drive_node` and VESCs accept `/cmd_vel` through a limiter that scales speed with slope, load and temperature.
- **Arm system:** `arm_controller` kinematics for beacon pick and place; end-effector pose from joint feedback, not step counting.
- **Autonomy (assist, not full autonomy):** the task is operator-driven, so autonomy here means decision support and protection: route planning on the prior-plus-live cost map shown to the operator, a suggested path, slope-aware speed limiting, a hold-on-grade behaviour, and an optional replay of the recorded breadcrumb path back to the start on link loss.

#### 3b. Task execution (what happens, in order)

1. **Pre-run (before the clock):** load the 2018 point cloud into the prior map; register to GNSS; measure link throughput on the test endpoint; confirm bandwidth cap, set the video bitrate; start the safety supervisor and verify the link watchdog by pulling the link once.
2. **Restore the link:** bring the rover's last-hop radio up to the antenna site; operators confirm command, telemetry and video arrive on the CIRC link (Minimum Operating Capability).
3. **Pick up the beacon:** operator drives to the beacon staging position; arm picks it up with the gripper (beacon size/mass from the advance document); confirm grasp from gripper position and current.
4. **Choose the placement spot:** use the prior and live map to find a spot with line of sight toward the dispenser/antenna and a stable footing (a viewshed from the prior point cloud) **[I]**; operators drive there and place the beacon.
5. **Unlock check:** judges/base operators report the dispenser unlocked when the beacon's signal strength meets the threshold. If not, adjust the position and retry; if impossible, a modification intervention unlocks it.
6. **Position for loading:** operator brings the bed (or trailer) under the auger spout using the depth camera/LiDAR for alignment; hold station on firm ground.
7. **Load:** base operators run the auger; the rover reports estimated fill height and mass; stop the auger before spill or the load limit (per-trip load kept within what the drivetrain can climb).
8. **Haul:** operators drive the route to the scale; the safety supervisor enforces slope, tilt, current and temperature limits; the prior-plus-live map highlights steep or changed sections; slow and steady on the grade.
9. **Unload at the scale:** align at the scale, trigger the unload mechanism, confirm the bed is empty (depth camera) and no ore is spilled.
10. **Repeat** loading and hauling until 100 kg net is reached; bring the trailer (if used) back to the starting area, still latched.
11. **Log:** record bags and telemetry for the post-run review.

### 4. Subsystem breakdown (what each piece does for this task)

| Subsystem | Function in this task | Key technical requirements |
|---|---|---|
| **LiDAR scanner / mapping** | Prior map from the 2018 cloud; live elevation grid and slope; change detection vs 2018; Layer 3 normals on the grade | Mount with a clear view of the ground ahead and the grade; Potree-to-grid converter; scan-to-map registration; confidence per cell; run within the Layer 1-2 CPU/RAM budget |
| **Autonomy** | Cost map, suggested route, speed/torque limiter, hold on grade, link-loss behaviour | Independent safety supervisor; planner never overrides it; works with partial map |
| **GPS / GNSS** | Global anchor for registering the prior map; operator map position | NMEA parse and checksum, fix-quality gate, fused in `rover_ekf` so LIO drift does not accumulate; useful open-sky site assumed |
| **Arm kinematics** | Beacon pick/place; optional unload actuation | Reach and payload for the beacon (documents to come), gripper stroke, measured pose, place on uneven ground |
| **Depth camera (possible)** | Near-field terrain, bed fill height, spout and scale alignment | Short-range accuracy, sunlight tolerance, calibrated extrinsics to the LiDAR; optional (everything works without it) |
| **Drive / power** | Haul 100 kg net on a grade | Torque and thermal margin on the VESCs, brake hold, slip estimate; a trailer must be latched and at most 15 kg |
| **Comms / UI** | Reliable control over the backhaul | Offline dashboard, authentication, adaptive video, watchdog, last-hop radio |

### 5. "How does D.A.V.E. break?" (risks and traps)

- **Risk 1: link loss or collapse on the grade.** Last command held means the rover keeps driving or rolls; video-dependent UI becomes useless; two teams share spectrum and no one coordinates it. Mitigation: watchdog ramp to zero with brake hold, telemetry without video, adaptive bitrate, tested by pulling the link.
- **Risk 2: the 2018 prior map is wrong.** Terrain has changed, so a "free" prior path may now be a gully or washout; a mis-registered prior (GNSS offset, LIO drift) makes it worse. Mitigation: prior cells low confidence, live data always wins, show the change layer to operators, re-register at known features.
- **Risk 3: grade versus 100 kg.** VESC overcurrent or thermal shutdown, wheel slip on loose pea rock and gravel, rollover from a high centre of gravity on a side slope; payload mechanism (trailer vs onboard bed) still undecided. Mitigation: per-trip load sized to measured climb capability, tilt and current limiter, brake hold, test on a real slope early.
- **Risk 4: the beacon never meets the threshold.** Wrong spot (no line of sight), low-reflectance mounting, or the beacon is dropped or tipped on placement; the threshold is only announced at the start. Mitigation: viewshed on the prior cloud, place at a high, clear spot, move and retry, fall back to the modification intervention.
- **Risk 5: spills and misaligned loading.** Ore falling beside the bed earns nothing and may lower the maximum score; the operator has high latency and a coarse view. Mitigation: depth-camera fill height and alignment cue, stop auger early, a lipped bed.
- **Risk 6: LiDAR and estimation faults.** On a steep, open, featureless slope LIO can drift vertically or lose tracking; dust and direct sun reduce returns; the arm or trailer enters the scan. Mitigation: body/trailer mask, GNSS and wheel fusion, a degraded mode that limits speed.
- **Risk 7: dashboard or rosbridge fails the offline/security test.** Libraries loaded from the internet, unauthenticated control. Mitigation: bundle all assets, require a token.

---

## 2. Automated Demolition and Clearing

*Source: live tasks page, read 2026-11-11. Not specified there: points, retry deductions, time limits, end-effector height limits, TNT size/mass, obstacle sizes. ArUco dictionary (4x4_100, IDs 0-99, at least 5 cm) comes from the general rules, see [competition.md](competition.md).*

**Goal in plain English:** With no human input except a judge clicking one labelled UI button per operation, D.A.V.E. autonomously exits the base, reads a five-marker job board, drives to a worksite, proves its arm works, places a TNT stand-in, and presses the detonation button, each operation starting from the pose the previous one ended in.

### 1. The inputs (what the rover needs to know)

**From judges / control station**
- A judge's button click starts each operation; this is the only allowed input.
- Parameters typed into text boxes, in the units the rules specify: distance N m (1, 3 or 5), angle N or M degrees, absolute bearing N degrees (0 = geographic north), GNSS coordinates in WGS 84 decimal degrees, EFF open width N cm (at most 10), EFF height and horizontal distance in metres.
- Reporting requests (`[Report GNSS]`, `[Report Bearing]`, `[EFF Report]`) arrive as clicks, and judges confirm success or failure afterwards.
- Given in advance: drawings of the tower, the marker layout for `[EFF Pickup]`, and the detonation box (button plus four fixed markers); the magnetic declination for Drumheller is looked up online beforehand.
- Starting positions of each operation are marked on the ground (not a rover input, but how the field is set up).

**From sensors**
- **GNSS:** absolute position for waypoints and `[Report GNSS]`.
- **IMU + wheel speed:** heading, yaw rate, short-term distance.
- **LiDAR cloud + IMU:** odometry and obstacle/terrain map on the off-road legs.
- **Cameras:** ArUco markers (board, tower #3, detonation box, pickup reference); wrist camera for the marker-relative arm moves.
- **Depth camera (possible):** marker range, TNT piece location, button standoff distance.
- **Arm/gripper feedback:** joint angles and gripper width, to report a *measured* EFF pose.
- **Stored state:** home pose, stored marker IDs, the mission phase.

### 2. The outputs / actions (what the robot commands)

**Mobility**
- `[Drive Forward]` / `[Drive Backwards]` N m, `[Rotate]` N degrees, `[Rotate and Drive]` N m at M degrees clockwise, `[Face Bearing]` N degrees from true north.
- `[Drive to Waypoint]` (position reference within 3 m of a GNSS point; obstacle avoidance recommended, an obstacle-free slower route earns half) and `[Drive to Marked Waypoint]` (within 1 m of the tower carrying ArUco #3).

**Manipulation**
- `[Store Home]`, `[EFF Max Height]`, `[EFF Min Height]`, `[EFF Position]` (height and horizontal distance from the front of the body), `[EFF Open]` at least N cm, `[EFF Close]`.
- `[EFF Pickup]` (relative to two markers), `[EFF Squeeze]`, `[EFF Set Down]`, `[EFF Press Marked Button]` (relative to markers #N and #M on the detonation box).

**Feedback**
- `[Report GNSS]`, `[Report Bearing]`, `[EFF Report]` (position relative to the indicator package, +x forward, +y right, +z up, metres), `[Read and Store Markers]` (display the five IDs, left to right), `[Retrieve Markers]`.
- Every returned value on the UI with its unit; pass/fail states of each operation.

### 3. Logic pipeline

#### 3a. Logical architecture

```
Judge click (UI button + text boxes)
     v
[Operation server]  one action/service per button; validates units; reports result + units
     v
[Mission state]  phase, home pose, stored markers, last end pose, retry count
     |
     +--> [Motion controller] closed-loop distance/heading on odometry --> cmd_vel --> drive_node
     +--> [Autonomy planner]  waypoint -> cost map -> Nav2 path -------> cmd_vel
     +--> [Vision / ArUco]    markers, tower, box, pickup frames ------> targets
     +--> [Arm controller]    IK to frame target, grip, press ---------> steppers
     ^
[Pose estimation]  rover_ekf: GNSS + IMU + wheel + LIO; declination -> true-north heading
     ^
[LiDAR mapping stack] LIO odometry + elevation grid (Layer 3 surfels optional) [Depth camera]
```

- **Operation server:** each button maps to one operation. This is what the judge sees, and the only autonomous trigger. It enforces "no operator input after the click" by never waiting for the UI mid-operation.
- **Pose estimation:** one fused pose; the heading is published as a true-north bearing (IMU heading plus declination for Drumheller); a measured frame at the "position reference" (indicator package) so distances and reports use the same origin.
- **Motion controller:** the missing piece today (`NAVnode` does not drive). Distance moves integrate odometry with a ramp-down; rotations are a P-controller on yaw error with a deadband; `[Rotate and Drive]` = rotate then drive; `[Face Bearing]` = rotate to the absolute bearing.
- **Autonomy planner:** LiDAR scan to deskewed cloud to elevation/traversability grid (Layer 2, Layer 3 normals if enabled) to Nav2 costmap to least-cost path, with `NAVnode` as the waypoint source. Natural obstacles are costs; no route is attempted through cells of unknown confidence.
- **GNSS system:** NMEA to decimal degrees, checksum and fix-quality gate; local East-North about the stored home; range and bearing to the target.
- **Vision (ArUco):** `DICT_4X4_100`; markers ordered by image x; pose from a calibrated camera; visual servoing near the tower and the box.
- **Arm system:** IK in the rules' frame (+x forward, +y right, +z up from the body front, relative to the indicator package) with an end-effector frame at the reference mark; reports a measured pose.

#### 3b. Task execution (what happens, in order)

1. **Setup:** place at the start pose; confirm GNSS fix, LIO tracking and the stored declination; the UI shows only the labelled buttons; the home pose is stored.
2. **Phase 1, exit the base:** `[Store Home]`; run `[Drive Forward]`, `[Drive Backwards]`, `[Rotate]`, `[Rotate and Drive]`, `[Face Bearing]` as clicked; answer `[Report Bearing]` and `[Report GNSS]` whenever asked. Each ends in the marked pose for the next operation.
3. **Phase 2, jobsite board:** `[Drive to Waypoint]` uses the planner and obstacle map; `[Drive to Marked Waypoint]` drives to near the tower, finds ArUco #3, servos in to within 1 m.
4. **Phase 3, read the board:** `[Read and Store Markers]` captures the five markers left to right with the top up, displays the IDs, stores them.
5. **Phase 4, worksite:** `[Retrieve Markers]` supplies the stored IDs, which may select the next GNSS target; `[Drive to Waypoint]` across rough terrain with obstacles.
6. **Phase 5, operational check:** the arm operations in order (max, min, position, open, close, store home, report), each checked against measured feedback.
7. **Phase 6, place explosives:** drive to the GNSS location, `[EFF Pickup]` the TNT piece relative to the two markers, `[EFF Squeeze]`, travel, `[EFF Set Down]` at the designated place.
8. **Phase 7, detonate:** drive to the Detonation Station, find the four markers on the box, compute the button position from markers #N and #M, `[EFF Press Marked Button]`.
9. **Reset handling:** after a failed attempt the rover can be reset to the starting pose without penalty (later attempts lose points); every operation is repeatable and idempotent.
10. **Log:** record bags and operation results for tuning.

### 4. Subsystem breakdown

| Subsystem | Function in this task | Key technical requirements |
|---|---|---|
| **LiDAR mapping / odometry** | Accurate distance and heading for the move commands; obstacle map for the off-road legs | LIO in the EKF; deskew; arm/body mask; elevation grid (Layer 2); Layer 3 normals optional for rough terrain |
| **Autonomy** | Operation server, motion controller, route planner, mission state | Closed-loop move/rotate; Nav2 or equivalent; idempotent operations; no waiting for a human |
| **GPS / GNSS** | `[Report GNSS]`, waypoints within 3 m and 1 m, true-north reference | NMEA parse, fix-quality gate, fusion with LIO, declination for Drumheller, local frame about home |
| **Arm kinematics** | All `[EFF ...]` operations | EFF frame at the reference mark in the rules' frame; measured (not stepped) pose; reach to the board/box; IK that converges; payload for TNT |
| **Depth camera (possible)** | Marker range, grasp and button standoff | Calibrated to the arm/wrist; short-range accuracy; works without it |
| **Vision** | ArUco read, order, tower and box servoing | `DICT_4X4_100`; lighting tolerance; pose from camera intrinsics |
| **UI** | One labelled button per operation, text boxes with units | `[shortname] long name`; every value with its unit |

### 5. "How does D.A.V.E. break?" (risks and traps)

- **Risk 1: open-loop distance and heading.** No encoders in software, wheel slip on loose ground; `[Drive Forward] 5 m` is only as good as odometry. LIO drift or a bad initialisation corrupts every later move.
- **Risk 2: bearing convention.** An unresolved yaw convention plus forgetting declination makes every `[Report Bearing]` and `[Face Bearing]` consistently wrong; magnetometer disturbance from motors skews the heading.
- **Risk 3: error stacking between operations.** Each operation starts where the last one ended; a few degrees of residual error becomes metres at the next waypoint; resets cost points.
- **Risk 4: ArUco failures.** Glare, shadows, motion blur or a partly framed marker; board read out of order if not square; wrong dictionary or marker size.
- **Risk 5: arm IK and frame errors.** Coordinate-wise numeric IK can stall; frames differ from the rules' frame; one actuated joint means base error becomes arm error; reporting a stepper count as a position fails `[EFF Report]`.
- **Risk 6: GNSS multipath or poor fix.** A tens-of-metres error misses the 3 m and 1 m criteria; mitigate with the fix gate and LIO bridging.
- **Risk 7: route planning.** Natural obstacles or unknown cells; planner trusts a gap that is not passable; the half-points obstacle-free route is the fallback.

---

## 3. Meteorite Search

*Source: live tasks page, read 2026-11-11. Not specified there: points, time limits, Geiger connector pinout and mounting (released later), the map (released later). Wheel odometry and rangefinding are named as allowed alternatives to GPS.*

**Goal in plain English:** With **no GPS used for any part of the task**, D.A.V.E. locates three of five impact sites using a map and a Geiger counter, photographs each from two angles at least 90 degrees apart plus a panorama, recovers a fragment and a 10-100 g soil sample, records each site's distance and direction from the start and two landmarks, and returns to the start.

### 1. The inputs (what the rover needs to know)

**From judges / control station**
- The site map (released later) and the Geiger counter's connector details (released later).
- The operator's mission plan; the start point is where the rover begins.
- The pre-mission briefing is written before the event (plan, sensors, experiments); the post-mission report is due within two hours of task time.

**From sensors**
- **LiDAR + IMU:** the only global-ish pose source and the terrain map (rangefinding is allowed; confirm LiDAR counts **[?]**).
- **Wheel odometry:** speed and short-term distance.
- **IMU heading:** direction for reports and the route map's cardinal directions.
- **Geiger counter (¼" TRS):** radiation count rate.
- **Cameras:** site photos, panorama, landmark identification.
- **Depth camera (possible):** soil sampling height, fragment pickup.
- **Arm/gripper feedback:** sampling tool state.
- **No GNSS.** The GNSS receiver is physically disconnected or unpowered for this task, not just ignored in software **[I]**.

### 2. The outputs / actions (what the robot commands)

**Mobility**
- Drive to candidate sites, hold at a site, rotate in place for photos and panorama, return to the start point.

**Manipulation**
- Pick up a meteorite fragment; scoop 10-100 g of surface soil (bentonite clay); keep each sample isolated; stow safely.

**Feedback**
- Per site: two photos at least 90 degrees apart, a panorama (at least 180 degrees for points, 360 degrees for full points), Geiger reading, a position relative to the start (distance and direction), and distance and direction to at least two landmarks.
- Route map with start, sites, landmarks and obstacles, with scale and cardinal directions; logs and measurements for the report; honest failure notes.

### 3. Logic pipeline

#### 3a. Logical architecture

```
Map (given later) + Geiger counter (TRS)
     v
[Mission planner]  site order, route, time and battery budget
     v
[Autonomy / Nav2]  start-frame goals --> cost map --> cmd_vel --> drive_node
     ^
[Pose estimation]  rover_ekf, GNSS input OFF: IMU + wheel + LIO, start frame = origin
     ^
[LiDAR mapping stack]  LIO + elevation grid + landmark map; surfel-Gaussian layer, re-localise on return
[Documentation system] stamped photos, panorama, landmark ranges
[Arm system]           fragment pickup, soil scoop
[Logging]              bags, CSV of sites, route map export
```

- **GNSS-free localisation:** Layer 1 LiDAR-inertial odometry is the pose source; `rover_ekf` runs with the GNSS update disabled by configuration; the **start frame** is the origin and all site and landmark positions are reported in it.
- **Mapping stack (the novel part):** a surfel-Gaussian terrain map doubles as a landmark map. Stable disks cluster into rocks and posts, which give range and bearing from sites and let the rover **re-localise against its own map** on the return leg, bounding drift without GNSS. The student traversability model (geometry plus LiDAR intensity) steers around soft clay and steep cuts.
- **Heading and cardinal directions without GNSS:** IMU heading with declination (or a measured reference at the start) for the route map's north arrow; magnetometer disturbance from the motors is a risk.
- **Search system:** order the sites from the map; Geiger counts versus position build a heat map to refine the search near a site.
- **Documentation system:** at each site stop, capture pose-stamped photos at least 90 degrees apart, rotate in place for the panorama, record the landmark ranges and bearings.
- **Arm system:** fragment pickup and a scoop tool tested against bentonite, with sample isolation.
- **Logging:** every run recorded; the route map and measurements for the report come from the logs.

#### 3b. Task execution (what happens, in order)

1. **Pre-run:** disable GNSS (hardware and config); verify LIO tracking; set the start frame and heading reference; test the Geiger reader and the sampling tool on bentonite.
2. **Plan:** read the released map, choose three target sites (five if time allows), set the route and a time and battery budget.
3. **Search:** drive to the first site along the planned route; refine with the Geiger heat map; stop on a confirmed site.
4. **Document:** two photos at least 90 degrees apart, the panorama (180 or 360 degrees), the Geiger reading, then landmark ranges and bearings; record the site's position in the start frame.
5. **Recover:** pick up one fragment; collect a 10-100 g soil sample; keep samples separate.
6. **Repeat** for the other sites.
7. **Return:** drive back to the start point using LIO and a map re-localisation, with all samples and fragments.
8. **Report:** within two hours write the report from the logs (route map, plan changes, measurements, locations with calculations, site findings, sample and fragment descriptions, honest failure notes).

### 4. Subsystem breakdown

| Subsystem | Function in this task | Key technical requirements |
|---|---|---|
| **LiDAR mapping / odometry** | Only pose source; landmark map; re-localisation on return | LIO with the GNSS update off; landmark extraction; loop closure on revisits; works on flat, featureless ground |
| **Autonomy** | Route to sites and back, hold, stop-and-capture | Start-frame goals; Nav2 on the elevation grid; slip-aware speed |
| **GPS / GNSS** | **Banned.** Make sure nothing leaks into the pose | Remove or unpower the receiver; confirm `NAVnode` and `rover_ekf` run without it; no GNSS-dependent topics |
| **Arm kinematics** | Fragment pickup, soil scoop | Reach to the ground; scoop tool motion; sample isolation |
| **Depth camera (possible)** | Fragment and sampling-spot localisation near the ground | Short-range accuracy; not required |
| **Documentation / logging** | Photos, panorama, report data | Pose-stamped images; rotate-in-place capture; bag and CSV logs |

### 5. "How does D.A.V.E. break?" (risks and traps)

- **Risk 1: odometry drift with no absolute fix.** On flat, featureless, slippery clay LIO can slide or drift vertically; site positions can be off by metres by the end of the run. Mitigation: landmark re-localisation, loop closure on the return, a drift bound reported honestly.
- **Risk 2: a GNSS dependency leaks in.** `NAVnode` and `rover_ekf` both use GNSS today; a stale fix silently entering the pose breaks the ban. Mitigation: hardware disconnect, separate launch profile, a test that the system runs with no GNSS.
- **Risk 3: heading and cardinal directions.** Without GNSS the north reference comes from the IMU; motor magnetic fields and the declination skew every "direction" measurement.
- **Risk 4: clay.** Bentonite is sticky and slick when wet; wheels clog and slip, the scoop jams, samples mix.
- **Risk 5: time and battery.** The map arrives late, the route is poor, panoramas and photos cost time, and the 45-60 minute window plus the two-hour report limit leave little slack.
- **Risk 6: Geiger interface.** Pinout and mounting arrive late; the reader code is untested; a bad connection loses the main search cue.

---

## 4. Prison Break (night)

*Source: live tasks page, read 2026-11-11. Not specified there: points, time limits, TRS pinouts, light-detection thresholds, the alarm time window, whether operators drive or the rover is autonomous **[?]**. The "no emitted light" rule is stated for the **high-security area**, not necessarily the whole prison.*

**Goal in plain English:** At night, D.A.V.E. uses the given prison map to collect the four exit-code sections, cut the correct wire on the exterior circuit panel to disable the cameras, enter the alarm-deactivation code on the large-button panel in time, and leave through the keypad door, without being detected in the high-security area.

### 1. The inputs (what the rover needs to know)

**From judges / control station**
- The prison map at the start; the exit code is assembled from clues.
- Serial port details for the security console (female ¼" TRS; pinout and position released later; we bring the male connector).
- Operator commands during the run (how the task is driven is not stated **[?]**).

**From sensors**
- **IMU + wheel odometry:** dead-reckoned pose; the rover's only pose source where emitters are off.
- **LiDAR (L2) and depth camera:** pose and mapping where emission is allowed. Both emit infrared and must be off in the high-security area.
- **Low-light / passive cameras:** illuminated markers around the area, color-pattern clues on the keypad, the joystick terminal screen, the alarm-type display.
- **Serial (TRS) line:** the security console's Linux-like terminal.
- **Arm/gripper feedback and wrist camera:** for the key, the wire cutter, the buttons.

### 2. The outputs / actions (what the robot commands)

**Mobility**
- Slow, deliberate driving; stop-and-go so the rover does not stay on a pressure plate; avoid spotlight fields of view; hold still.

**Manipulation**
- Insert, turn and pull the key at the mechanical safe; push the joystick (it does not move; panning speed follows the force applied); cut the correct wire on the Exterior Circuit Panel; press a sequence of large buttons on the vertical panel; press the exit keypad.

**Feedback**
- The terminal text from the console, the four code sections, the alarm type and the computed deactivation code, a countdown timer, and per-sensor emitter status (on or off).

### 3. Logic pipeline

#### 3a. Logical architecture

```
Prison map (start) + operator
     v
[Operator UI]   map overlay, keep-out zones, timer, sensor-emitter switches
     v
[Safety / emission supervisor]  kills LiDAR + depth camera in the high-security area; e-stop
     v
[Pose estimation]  rover_ekf: IMU + wheel (+ LIO where emitters are on); handoff on entry
     v
[Drive]  slow, stop-and-go            [Arm]  key, joystick, wire cutter, buttons
[Vision] passive cameras, color patterns, screens   [Serial] console terminal
[Code calculator]  4 sections + alarm type --> keypad / deactivation code
```

- **Emission-aware sensing:** a software kill switch per emitter plus a physical power switch **[I]**; a status indicator for the operator; the high-security area is entered with emitters off.
- **Pose handoff:** build and use LIO outside the high-security area, record the pose on entry, then dead-reckon on IMU and wheel odometry through the area, correcting with the illuminated markers the area provides.
- **Mapping stack:** the prison map is registered to the LIO frame at the start; keep-out polygons (pressure plates, spotlight fields of view, dwell timers) are added to the cost map.
- **Serial terminal:** the existing `SerialPort` plus a terminal tool for the console.
- **Vision:** passive cameras read the color-pattern clues, screen digits and alarm display; the joystick panning is driven by a force command proportional to the speed wanted.
- **Arm:** key insertion, joystick push, wire cut, button presses at the right height and standoff.
- **Code calculator:** pre-written; takes the four sections and the alarm type and returns the exit code and deactivation code, so nothing is computed by hand under the countdown.

#### 3b. Task execution (what happens, in order)

1. **Setup:** register the prison map to the start pose; test emitter kill switches; test the serial cable and terminal tool.
2. **Navigate to the clue locations** (any order): security console, access-pass terminal, mechanical safe, high-security area.
3. **Security console:** connect the male TRS cable, read the code section from the terminal.
4. **Access-pass terminal:** push the joystick to pan the screen, read the long code.
5. **Safe:** insert the key, turn it, pull the door, read the section.
6. **High-security area:** switch LiDAR and depth camera off, drive on the illuminated markers and the pose handoff, avoid spotlights, do not dwell on pressure plates, read the section.
7. **Exterior Circuit Panel:** identify the correct wire, cut it (starts the alarm countdown), read the alarm type.
8. **Deactivate:** compute the code and press the large buttons within the time window.
9. **Exit:** enter the assembled exit code on the large keypad and drive out.
10. **Log:** record emitter on/off states and timings.

### 4. Subsystem breakdown

| Subsystem | Function in this task | Key technical requirements |
|---|---|---|
| **LiDAR mapping / odometry** | Pose and map outside the high-security area; hands off on entry | Hardware and software kill switch; registered to the given prison map; works from a recorded start |
| **Autonomy** | Slow stop-and-go motion, keep-out zones, dwell timer, e-stop | Operator-in-the-loop likely **[?]**; polygons for plates and spotlights |
| **GPS / GNSS** | Not useful indoors or on the given map; unlikely to help | Do not depend on it |
| **Arm kinematics** | Key, joystick, wire cutter, buttons | Fine positioning; wrist camera; tool for cutting; force-controlled joystick push |
| **Depth camera (possible)** | Standoff and alignment for the arm; only outside the emission area or **passive stereo** only | Emitting depth cameras are off in the high-security area |
| **Vision / serial** | Color patterns, screens, the terminal | Low-light camera performance; serial pinout |

### 5. "How does D.A.V.E. break?" (risks and traps)

- **Risk 1: our sensors are emitters.** The L2 LiDAR and an infrared depth camera count as emitted light in the high-security area. Mitigation: kill switches, dead reckoning through the area, a passive stereo or low-light camera.
- **Risk 2: pose loss in the dark.** After the handoff the pose is IMU and wheel only; slip and turn error make the rover stray onto a pressure plate or into a spotlight field of view.
- **Risk 3: fine manipulation.** The wrong wire cut or a missed button ends the run; the arm has one actuated joint and open-loop steppers.
- **Risk 4: reading in low light.** Color patterns and screen digits with a rolling-shutter camera and only exempt indicator lights; one wrong code section fails the exit.
- **Risk 5: the countdown.** The deactivation code must be entered within a short time window after the cut; a slow computation or UI delay misses it.
- **Risk 6: dwell and spotlight timing.** A stall, link lag or a slow turn keeps the rover on a plate or in a spotlight field of view.

---

## 5. Reactor Recovery

*Source: live tasks page, read 2026-11-11. Not specified there: points, deductions, time limits, the actual map, storm chart and debris catalogue (issued at the start), the warning-light color, the beacon receiver pinout and protocol, the receiver mount drawings.*

**Goal in plain English:** Using the start-of-mission sector map, dust-storm chart and debris catalogue, D.A.V.E. removes debris to the right containment zone and then carries the astronaut above the ground to the medical bay over very difficult terrain, never entering a sector while a storm is active there.

### 1. The inputs (what the rover needs to know)

**From judges / control station**
- **Mission briefing at start of task time:** the sector map with GNSS boundaries (WGS 84 decimal degrees; some boundaries also have landmarks), the per-sector storm chart (active times and Mild or Severe), the debris catalogue (shape sketch, locating method, special handling, target containment zone), and the astronaut's GNSS position.
- The warning-light color (announced at the start); the beacon receiver pinout and protocol (released later).
- A written crew-member description for some debris.

**From sensors**
- **GNSS:** position, sector membership, GNSS-point debris, astronaut position.
- **LiDAR + IMU:** odometry, terrain map, slope and tilt on very difficult terrain.
- **Depth camera (possible):** near-field terrain, grasp alignment, table and container height.
- **Cameras:** flashing or rotating warning-light color, text-description landmarks, the lid.
- **Beacon receiver (UART over TRS):** approximate distance to a beacon-tagged object; needs open sky.
- **Arm/gripper feedback:** grasp state and payload; carry feedback for the astronaut.
- **A mission clock** synchronised to the storm chart.

### 2. The outputs / actions (what the robot commands)

**Mobility**
- Drive to debris along sector-safe routes; wait outside a sector until its storm ends; slow and tilt-limited when carrying the astronaut; leave a sector immediately when a storm turns active.

**Manipulation**
- Grasp debris (at most 5 kg, mostly under 1 kg, grip features up to 80 mm); place it on the ground zone, an elevated surface up to 1 m, or in a container (open the lid); keep must-not-touch-ground items off the ground until placed; carry the astronaut above ground to the medical bay.

**Feedback**
- Sector storm status and countdown, the sector the rover is in, the next safe window, debris cue type and progress, beacon distance, detected light color, tilt margin, and the mission state.

### 3. Logic pipeline

#### 3a. Logical architecture

```
Briefing (map, storm chart, catalogue, astronaut GNSS)
     v
[Mission planner]  debris order, zone per object, storm windows, battery
     v
[Geofence supervisor]  polygon keep-out per sector x time; severe-storm stop
     v
[Autonomy / Nav2]  cost map + keep-out layer --> cmd_vel --> drive_node
     ^
[Pose: rover_ekf  GNSS + IMU + wheel + LIO]
[LiDAR mapping stack]  elevation grid + surfel-Gaussian normals + student traversability model
[Search handlers]  GNSS point | light color | beacon distance | text description
[Arm / carry system]  grasp, zone placement, lid, astronaut cradle
```

- **Geofence supervisor:** sectors become polygons in the local frame (existing geodetic-to-local maths); a sector is a Nav2 keep-out region whenever its storm window is active; the mission clock drives it. `NAVnode` obstacles are circles only, so a polygon layer is new work.
- **Mapping stack (where the novel part matters most):** very difficult terrain is scored from the elevation grid and the surfel-Gaussian layer (slope, roughness, step, confidence), with the student model using LiDAR intensity to tell loose rock or vegetation from firm ground; tilt and rollover margins use the IMU, especially with the astronaut on board.
- **Search handlers:** GNSS point (drive to it); warning light (blink and color detector); beacon (read the UART distance, climb the distance gradient, sky view required); text description (operator plus landmarks).
- **Mission planner:** orders debris to minimise travel and avoid storm windows, replans when a window changes; leaves the rescue until all debris is cleared for full points.
- **Arm / carry system:** grasp, placement by zone type, lid opening, and an astronaut cradle or scoop that keeps the astronaut above the ground.

#### 3b. Task execution (what happens, in order)

1. **Briefing intake:** load the sector polygons, storm windows, catalogue and astronaut position; start the mission clock; mark the starting area, containment zones and medical bay (never storm-affected).
2. **Plan:** choose the first debris and a route that avoids sectors with active or soon-active storms.
3. **Locate:** use the handler that matches the debris (GNSS, light color, beacon, or description).
4. **Approach:** drive on the traversability and keep-out cost map; if a storm turns active, leave the sector (a Severe storm triggers a judge stop).
5. **Pick up:** grasp by the grip feature; keep must-not-touch-ground items lifted.
6. **Deliver:** to the zone matching its contamination level (ground, table up to 1 m, or container with lid); the wrong zone earns partial points only.
7. **Repeat** until the debris is cleared (listed debris only; unlisted items earn nothing).
8. **Rescue:** drive to the astronaut's GNSS position over the very difficult terrain, lift the astronaut above the ground, carry them to the medical bay.
9. **Log:** record routes, storm windows and decisions.

### 4. Subsystem breakdown

| Subsystem | Function in this task | Key technical requirements |
|---|---|---|
| **LiDAR mapping / odometry** | Terrain scoring on very difficult ground; slope and tilt; confidence for unknown cells | LIO, elevation grid, Layer 3 normals, student model; tilt and rollover limits |
| **Autonomy** | Storm-aware routing, geofence, search behaviours, mission planner | Polygon keep-out by time; replanning; Nav2 with the geofence layer; independent stop |
| **GPS / GNSS** | Sector membership, debris and astronaut positions, route | Sector polygons in WGS 84; fix gate; fusion with LIO; the astronaut location is GNSS |
| **Arm kinematics** | Grasp 5 kg and up to 80 mm, place on a table up to 1 m, open a lid, hold the astronaut | Reach to 1 m height; payload and torque; carry mechanism; measured pose |
| **Depth camera (possible)** | Grasp alignment, table and container height, near-field terrain | Short-range accuracy; sunlight tolerance; not required |
| **Search / vision** | Warning-light color, beacon receiver, text interpretation | Color and blink detector; UART reader; mast with sky view for the receiver |

### 5. "How does D.A.V.E. break?" (risks and traps)

- **Risk 1: storm timing.** A storm window starts while the rover is inside or entering a sector; entering a sector with an active storm costs points and a Severe storm triggers a judge stop and a modification intervention. Mitigation: a clock-driven geofence with a safety margin, and a safe-exit route.
- **Risk 2: terrain beyond the rover.** Tip-over or high-centring on very difficult terrain with an astronaut of unknown mass; the traversability model labels an impassable cell as passable. Mitigation: confidence layer, tilt limit, and a conservative cost for unknown cells.
- **Risk 3: GNSS and beacon limits.** GNSS is weak near obstacles or cover; the beacon receiver needs open sky; chasing it in clutter gives wrong distance trends.
- **Risk 4: arm reach and payload.** Lifting 5 kg to a 1 m table, a lid that needs two motions, and a gripper lead screw and speed limit that are untested; the astronaut cradle design is unknown.
- **Risk 5: detection errors.** Sun glare confuses the warning-light color threshold; the text description is misread.
- **Risk 6: ordering.** Rescuing before all the debris is cleared loses the full-points condition.

---

## Open questions to settle before finalising

1. **Emitters:** the page says the no-light rule applies in the Prison Break high-security area (including infrared). Is an off-the-shelf LiDAR allowed at all, and can the L2 and any depth camera stay on elsewhere in that task?
2. **Meteorite:** does LiDAR count as the allowed "rangefinding", and is a GNSS receiver that is merely powered off acceptable or must it be removed?
3. **Prison Break:** is it operator-driven or autonomous? The page does not say.
4. **Reactor Recovery:** astronaut mass and size, the beacon receiver pinout, and the mount drawings.
5. **Mining:** beacon, scale and dispenser documents, the signal threshold, and how the rover learns the dispenser unlocked.
6. **Scoring** (still "to come"), so sequencing effort matches points.
7. **Internal:** the yaw convention in `rover_ekf` and the heading source (needed for true-north bearings in Demolition and the north arrow in Meteorite).
