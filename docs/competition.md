# Competition: CIRC 2027 tasks and what they ask of the rover

Source: the organisers' pages [tasks](https://circ.cstag.ca/2027/tasks/) and [rules](https://circ.cstag.ca/2027/rules/), read on 2026-11-09 and **paraphrased** here. The pages change; re-read them before any design decision. The scoring rubric was still marked "to come", so no point values or per-task time limits are known. Items marked **[?]** are our inference, not something the organisers said.

Purpose of this page: when a task needs a capability the rover or code does not have, it should be visible here, with a pointer to where it would be built. The "what we have" column refers to the current repository state ([known-issues.md](known-issues.md) lists what does not work).

## Key dates

| Date | Event |
|---|---|
| 2026-11-02 | Expression-of-interest deadline (passed) |
| 2027-06-18 | Safety report due |
| 2027-08-06 | Registration (11:00–17:00); first (night) tasks about 21:00 |
| 2027-08-06 to 08-09 | CIRC Summer 2027, Badlands Community Facility, Drumheller, Alberta |

## Rules that constrain the whole design

| Rule (paraphrased) | Consequence for us |
|---|---|
| Normally under 50 kg; 1 % bonus per full kg under, 1 % penalty per kg over; over 70 kg not allowed on sites | Every sensor, the Jetson and the arm have a mass cost. Weigh the rover early. |
| Fits through a 0.8 m door; two people can carry it | Mast and LiDAR placement must fold or be removable **[?]** |
| At least one hour of continuous run time on onboard power; no tether | Power budget: motors + Jetson (25 W mode) + LiDAR; see [slam/platform.md](slam/platform.md) |
| Tasks run 45–60 min; battery swap is a modification intervention | One battery must last a task |
| Must handle rain, mud, snow, dark, wind | Sensors and connectors need sealing; vision-only perception fails at night |
| Substantially built and programmed by the team; buying complex modules discouraged (a motor controller is fine, a whole arm is not) | The arm and gripper are ours. Confirm with the organisers that an off-the-shelf LiDAR is acceptable **[?]** before buying |
| Must lift/drag/manipulate up to 5 kg and grip objects up to 90 mm wide | Gripper stroke ≥ 90 mm and payload 5 kg are requirements for the arm ([arm-and-kinematics.md](arm-and-kinematics.md)); the gripper lead screw and 0.05 m/s limit are untested against this |
| Wireless at all times while scoring; reserved 5 GHz channels 140–165 (unapproved use can zero a run) | Do not use those channels in testing without approval |
| "CIRC Backhaul" tasks: 100BASE-TX Ethernet to the antenna site, MTU 1500, we provide the last hop; no cell or satellite | The rover must work over a plain IP link; see Mining below |
| Minimum operating capability: fully drivable and manipulable with telemetry over the link, video optional | Control and telemetry must not depend on video |
| Autonomous means no operator input at all; ArUco from dictionary **4x4_100** (IDs 0–99), at least 5 cm; recognition must run on the rover or base computer and show results automatically | An ArUco node is mandatory (OpenCV `cv2.aruco`, `DICT_4X4_100`); none exists in the repository |
| Distances are real-terrain odometry; bearings are degrees from **true north**; GNSS is WGS 84 decimal degrees | Needs declination handling (Drumheller), see Demolition |
| Rover needs the CIRC indicator package (ground and orientation reference); the end effector needs a reference mark | Mechanical/mounting item for the arm, and a frame in the URDF |
| Illuminated kill switch, hard stops, removable lithium batteries (Rover Safety Requirements, updated for 2027; Chemical Safety Requirements new) | Hardware items not covered here; read `/2027/rules/roversafety/` and `/2027/rules/chemicalsafety/` (not yet read) |
| Following crew keep 5 m away, no team imagery may be used to aid a task other than rover cameras | No drones or outside maps beyond what the organisers hand out |

## The five tasks

### 1. Mining Site Remote Operations

Restore the control link, place a relay beacon near the ore dispenser, run the dispenser (a remote screw auger), haul up to 100 kg of washed pea rock (3–10 mm) to a weigh scale, optionally with a trailer. Driven by operators at CIRC Central over the CIRC Backhaul, bandwidth capped between 50 and 100 Mbit/s; a coarse shared live video exists as a fallback view. A steep grade separates the sites; a 2018 LiDAR point cloud is provided and the terrain may have changed.

| Capability needed | What we have | Gap / where to build |
|---|---|---|
| Operate through an IP backhaul from a remote site | rosbridge + web dashboard ([dashboard-and-network.md](dashboard-and-network.md)) | The dashboard needs the internet for its libraries and map tiles and rosbridge is unauthenticated ([known-issues.md](known-issues.md) item 12); it must work **offline** on the backhaul. Add video rate adaptation (the `rover_video_streamer` bitrate is fixed at start). |
| Place a relay beacon (mass and size documents to come) | arm and gripper | A beacon holder or pick-and-place routine |
| Carry 100 kg net of ore, on a steep grade; trailer ≤ 15 kg, no onboard battery, latched, returned to the start | drive system | **Biggest open mechanical question.** The VESC/motor limits and the 50 kg rover target are untested against a 100 kg payload on a slope. Decide trailer vs. onboard bed early. |
| Unload at the scale (manual unload is a penalty) | none | Tipping or hopper mechanism |
| Plan using the 2018 point cloud | none | Potree 2.0 data; the SLAM layer 2 elevation map is the natural consumer ([slam/architecture.md](slam/architecture.md)) |
| Spilled ore earns nothing | – | Bed design |

### 2. Automated Demolition and Clearing

Fully autonomous after setup; the **only** input allowed is a judge clicking a button on our UI. Phases: exit the base, travel to the job board, read five ArUco markers, travel to the worksite, rover operational check (end-effector commands), place explosives (pick a TNT stand-in and place it), detonate (press a button on a box). Retries after a reset cost points; operations can run in any order within a phase.

UI requirements: buttons labelled `[shortname] long name` in English; text boxes with the rule's units; every returned value shown with units.

| Operation (button) | What we have | Gap / where to build |
|---|---|---|
| `[Store Home]`, `[Report GNSS]`, `[Report Bearing]` | `NAVnode` home waypoint and GNSS parsing ([navigation.md](navigation.md)) | Add reporting services; bearing must be **true north** (apply declination to the IMU heading; the yaw convention is already an open question, [known-issues.md](known-issues.md) item 4) |
| `[Drive Forward]` 1/3/5 m, `[Drive Backwards]`, `[Rotate]`, `[Rotate and Drive]`, `[Face Bearing]` | open-loop wheel commands; no encoders in software; `rover_ekf` | Closed-loop distance and heading control on odometry. This does not exist: `NAVnode` cannot drive the rover. LiDAR-inertial odometry ([slam/architecture.md](slam/architecture.md), layer 1) is the accurate source. |
| `[Drive to Waypoint]` (within 3 m), natural obstacles, route without obstacles scores half | `NAVnode` waypoints and hand-entered circular obstacles | A route follower plus real obstacle sensing (layer 2 map, Nav2). |
| `[Drive to Marked Waypoint]` (within 1 m of the tower with ArUco #3) | none | ArUco detection plus visual servoing near a GNSS point |
| `[Read and Store Markers]`, `[Retrieve Markers]` (five markers, one row, left to right) | none | ArUco node (`DICT_4X4_100`), ordered by image x; store and return IDs; the next GNSS target may depend on them |
| `[EFF Max/Min Height]`, `[EFF Position]` (metres, +x forward, +y right, +z up, from the front of the body, relative to the indicator package), `[EFF Open]` ≥ N cm (N ≤ 10), `[EFF Close]`, `[EFF Report]` | numeric inverse kinematics in `arm_controller/kinematics.py`; stepper control open loop | The reference frame conventions differ from the arm code's; reporting needs a measured, not integrated, pose ([arm-and-kinematics.md](arm-and-kinematics.md)). Add an end-effector frame in the URDF at the reference mark. |
| `[EFF Pickup]` relative to two ArUco markers, `[EFF Squeeze]`, `[EFF Set Down]`, `[EFF Press Marked Button]` (markers on the detonation box) | none | Marker-relative grasp and push; needs a wrist or arm camera and calibration |
| Judge-click UI | the dashboard | Not built; a separate page with exactly these labelled buttons is simplest. |

Everything must be autonomous; manual operations earn nothing. This task is the main driver for closing the loop in `NAVnode`/`FollowRoute`.

### 3. Meteorite Search

**GPS is banned for every part of this task.** Find three of five impact sites with a map (released later), onboard sensors and an external Geiger counter (1/4" TRS connector, pinout later); document each (two photographs at least 90° apart plus a panorama, at least 180° for points and 360° for full points); recover a meteorite fragment; collect a 10–100 g soil sample; record each site's position relative to the start and at least two landmarks (distance and direction); return to start. A pre-mission briefing (plan, sensors, experiments) is due before the event and a post-mission report within two hours afterwards. Soil is mostly bentonite clay.

| Capability needed | What we have | Gap / where to build |
|---|---|---|
| Navigation and relative positions **without GNSS** | `NAVnode` and `rover_ekf` both depend on GNSS | This is the strongest case for the SLAM program: LiDAR-inertial odometry gives metre-level relative positions with no GNSS. Odometry drift on slippery clay makes wheel-only dead reckoning poor. Record a start frame and report positions in it. [slam/research-plan.md](slam/research-plan.md) |
| Panorama ≥ 360° and two-angle photos | CSI cameras and `rover_video_streamer` | Pan mechanism or stitched rotation; image capture service with pose stamps |
| Geiger counter over TRS | `serial_communication` has serial plumbing | A reader node once the pinout is published |
| Soil sample 10–100 g and fragment pickup | gripper | A scoop or sampling tool, tested on bentonite |
| Report with route map, calculations, honest failure reporting | `scripts/slam/record_bag.sh`, logs | Keep bags and logs from every run: the report needs the route map and measurements |

### 4. Prison Break (night)

Escape "Rover Jail": read part of a four-part exit code from a security console over a serial port (female 1/4" TRS on the console), from a joystick-panned screen, from a mechanical safe (key beside it: insert, turn, pull), and from a high-security area; cut the correct wire on an exterior panel, then enter a computed deactivation code on a large-button panel before an alarm countdown ends. In the dark. **Any light emitted on the ground, including infrared, is detected**; spotlights detect the rover in their view; pressure plates trigger if it stays too long. Indicator lights and an illuminated e-stop are exempt. A map of the prison is given at the start.

| Capability needed | What we have | Gap / where to build |
|---|---|---|
| Navigate in the dark without emitting light | none | **Risk [?]:** a LiDAR emits near-infrared and a structured-light or time-of-flight depth camera projects infrared. Whether that counts as "emitted light" must be asked of the organisers before the sensor purchase. A passive stereo or low-light camera does not emit. Plan for the answer being "yes". |
| Serial terminal over TRS | `SerialPort` class | Male TRS cable, a terminal tool, and the pinout when published |
| Fine manipulation: key, joystick, wire cutter, large buttons | 5-axis arm kinematics | Dexterous tooling and a wrist camera; the biggest mechanical item for this task |
| Minimise dwell time on pressure plates | – | Stop-and-go planning in the operator workflow |
| Read a screen (code) | cameras | Camera quality in low light |

### 5. Reactor Recovery

Given at the start: a sector map with GNSS boundaries, a dust-storm forecast per sector, a debris catalogue and the astronaut's GNSS position. Remove debris (≤ 5 kg, mostly < 1 kg; larger items have a grip feature ≤ 80 mm; some must not touch the ground) to the right containment zone (ground, a table up to 1 m high, or a container that may need a lid opened), then carry the astronaut off the ground to the Medical Bay over very difficult terrain. Entering a sector with an active storm loses points; a *Severe* storm triggers a judge emergency stop and a modification intervention. Debris is located by GNSS point, a flashing warning light of a stated colour, a tracking beacon (UART over a 1/4" TRS, approximate distance, needs open sky), or a text description.

| Capability needed | What we have | Gap / where to build |
|---|---|---|
| Keep out of storm sectors (polygons) | `NAVnode` obstacles are **circles** with integer radius ([navigation.md](navigation.md)) | A polygon keep-out layer (Nav2 keep-out costmap filter, or a polygon check in `NAVnode`); sectors arrive as GNSS boundaries, so the geodetic ↔ local maths is reusable |
| Go to GNSS points | `NAVnode` waypoints | Route follower (see Demolition) |
| Warning-light colour detection | none | Colour/blink detector (can reuse the thresholding idea in `morse_recorder.py`) |
| Tracking beacon over TRS UART | `SerialPort` | Reader node, distance-based search behaviour |
| Grasp 5 kg and ≤ 90 mm; place on a 1 m table; open a lid | gripper | Gripper range and reach to 1 m height; check `arm_controller` limits |
| Carry an astronaut above ground | – | Unknown payload (not specified); a scoop or cradle **[?]**. Terrain is very difficult: this stresses the drive and the elevation/traversability layer. |

## What this means for the work already planned

1. **The no-GNSS Meteorite task and the steep, changed terrain of Mining justify the SLAM program** and make the LiDAR-inertial odometry (layer 1) the top priority, ahead of the Gaussian terrain layer, which no task asks for.
2. **Closing the control loop is the largest missing piece** of the existing stack: distance/bearing moves, waypoint following and `FollowRoute` ([known-issues.md](known-issues.md) section 3) are required by Demolition and Reactor Recovery.
3. **ArUco (4x4_100) detection is required, not optional,** for Demolition; it is also useful for docking-style alignment elsewhere.
4. **Night operation (Prison Break) may rule out emitting sensors** for that task. Keep the LiDAR/depth camera easy to switch off and have a passive-camera fallback; ask the organisers.
5. **Remote operation over a bandwidth-capped IP link** (Mining) needs the dashboard to work offline and the control path to survive without video.
6. **Not in the 2027 task list:** Morse-code transmission/reception. The Morse bridge, `morse_recorder` and `jetson_morse` are not needed for these tasks; keep them but do not spend time on them.
7. **Open questions for the organisers:** is an off-the-shelf LiDAR acceptable; does LiDAR/IR count as emitted light in Prison Break; trailer and beacon documents; TRS pinouts; indicator-package dimensions; scoring rubric.

## How to keep this page current

Re-read the tasks and rules pages each time the organisers publish a change (the rules have a change log), update the tables, and note the date at the top. Rover-side changes that a task forces should be recorded in [decisions.md](slam/decisions.md) (SLAM-related) or [known-issues.md](known-issues.md) (everything else) with a link back here.
