# Decisions: environment, tools and the two-tier strategy

Where [architecture.md](architecture.md) says *what* the system is, this page says *which tool goes in each slot*, *how the team develops and diagnoses*, and *how one research-grade stack and one rover-grade stack share work*. Evidence labels as in [README.md](README.md#evidence-labels): **[P]** primary, **[S]** search/secondary, **[M]** model summary, **[I]** recommendation, **[background]** general knowledge not checked during this project.

## Guiding rule: keep it lean, and write our own where it is cheap

The tool table in section 2 is a menu, not a shopping list. The team can, and for fun and learning intends to, write many of these pieces itself. So:

1. **Adopt a dependency only when it removes real work or risk.** Drivers, ROS 2, rosbag, tf2 and a planner are worth adopting; a large framework used for one feature is not.
2. **Prefer small, readable, replaceable code** behind the shared interfaces (the [sensor contract](architecture.md#sensor-contract-camera-agnostic) and the `/terrain/traversability` output). A home-grown node and a third-party node should be swappable without touching anything else.
3. **Candidates for writing ourselves** (each is small enough to be a good first project, and each has a published baseline to compare against): the elevation/traversability grid, the corridor safety stop, the GNSS alignment node, the obstacle bridge, the LiDAR–IMU body mask, the minimal surfel Gaussian map, the student traversability model, and trajectory-evaluation scripts. See [code-map.md](code-map.md#planned-code-does-not-exist-yet).
4. **Do not write ourselves:** the vendor LiDAR driver, DDS, the real-time/safety firmware on the motor controllers, and a general SLAM back end unless the bench-off shows nothing adequate.
5. **Every third-party component must justify itself against a baseline** in the experiments ([research-plan.md](research-plan.md)); if our own version is within the agreed margin, keep ours.
6. **Install lean:** start from `ros-base` plus only what the workspace needs, and add Gazebo, RViz extras or Isaac ROS when the matching phase begins, not before.

## 1. Development and diagnosis environment

### Findings on this machine (checked 2026-11-08)

| Item | Value | Consequence |
|---|---|---|
| Windows | 11 (build 26300) | Supports WSL2 with mirrored networking and WSLg GUI **[background]** |
| WSL | Default version 2, **no distribution installed**, no `.wslconfig` | Nothing to migrate; set it up cleanly |
| GPU | NVIDIA GeForce RTX 4060 Laptop (Windows reports 4 GB through a 32-bit field; the card normally has 8 GB, confirm with `nvidia-smi`) + Intel UHD | CUDA is available inside WSL2 for Gaussian and GPU-mapping experiments on the workstation tier |
| RAM / CPU | 15.7 GB, i7-12650H (10 cores) | WSL2 defaults to about half the RAM; raise it for simulation + SLAM replay |

### Roles

| Machine | Role | Runs |
|---|---|---|
| **Windows host** | Editing, documentation, browser dashboard, Foxglove/web tools | No ROS |
| **WSL2 Ubuntu 22.04** | Development, Gazebo integration tests, bag replay, offline experiments (the *research tier*, below) | ROS 2 Humble, `colcon build`, Gazebo, CUDA |
| **Jetson Orin Nano Super** | The rover's brain (the *rover tier*) and the source of truth for hardware behaviour | Drivers, odometry, mapping, planning, safety |
| **Raspberry Pi (optional)** | Bench or field gateway, **not** in the data path | See below |

### Setting up WSL2 (untested; nothing installed yet)

```powershell
wsl --install -d Ubuntu-22.04
```

Create `%USERPROFILE%\.wslconfig` before the first heavy run:

```ini
[wsl2]
memory=10GB
processors=8
networkingMode=nat
```

**Use `nat`, not `mirrored`.** Mirrored mode (WSL2 shares the PC's network addresses) was tried first and **broke local ROS 2 discovery on this PC**: two ROS processes in the same distro could not see each other with FastDDS (default, UDP-only and the repository profile) or CycloneDDS, even though plain UDP and raw multicast worked. Under `nat` the same test passed immediately. The cost is that the Jetson cannot open connections back into WSL2, so live DDS between the two is one-way; use the dashboard, SSH or recorded bags instead (methods 1, 2 and 4 below). Then inside Ubuntu: install ROS 2 Humble from the official apt repository, clone the repository **inside the Linux file system** (not under `/mnt/d`, which is slow for builds), and follow [build-and-run.md](../build-and-run.md). USB devices need `usbipd-win`; CAN, GPIO and anything real-time stay on the Jetson **[background]**.

### Status of the WSL2 environment (set up 2026-11-08)

| Item | State |
|---|---|
| Distribution | Ubuntu 22.04.5 LTS, stored on the D drive at `D:\WSL\Ubuntu-22.04` (disk image `ext4.vhdx`); swap file at `D:\WSL\swap.vhdx` |
| User | `dave`, member of `sudo` with passwordless sudo (convenient for a local dev VM; remove `/etc/sudoers.d/90-dave` to change) |
| WSL config | `C:\Users\canad\.wslconfig`: 10 GB memory, 8 processors, NAT networking (mirrored mode broke local DDS discovery, see above) |
| ROS | ROS 2 Humble `ros-base`, `ros-dev-tools`, colcon, rosdep, plus the packages the workspace uses (rosbridge, cv_bridge, xacro, robot_state_publisher, tf2, OpenCV, NumPy/SciPy/pyserial). Gazebo, RViz and Isaac ROS are **not** installed yet, on purpose |
| Repository | Cloned inside the Linux file system at `~/D.A.V.E` (not under `/mnt/d`); update it with `git pull` from the Windows repository path |
| Workspace build | `colcon build --symlink-install`: **all 16 packages build**. `navigation_system` needs `raylib`, installed from source (v5.0, static) into `~/opt/raylib` with no system-wide install; configure with `PKG_CONFIG_PATH=$HOME/opt/raylib/lib/pkgconfig` and `-Draylib_DIR=$HOME/opt/raylib/lib/cmake/raylib -Draylib_USE_STATIC_LIBS=ON` |
| Runtime test | `scripts/tests/navnode_nmea_smoke_test.sh` passes: `NAVnode` parses a simulated NMEA sentence from a virtual serial port, all expected services exist, `NAVDisplayInfo` is published, and after `ResetHome` at the same position the local position is within about 0.25 m of zero |
| Not done | Gazebo, RViz, `pykalman`/`Jetson.GPIO`/Adafruit libraries (runtime only), any hardware access, any other node run end to end |

The build also surfaced a real bug (`SerialPort::write` length), fixed and recorded in [known-issues.md](../known-issues.md).

### Reaching the Jetson from WSL2 (best to worst)

1. **Browser dashboard / rosbridge.** Needs no ROS on the laptop: open `http://<jetson>:8000`. Already works ([dashboard-and-network.md](../dashboard-and-network.md)).
2. **SSH plus the ROS command line.** `ssh` to the Jetson and run `ros2 topic hz`, `ros2 node list`, `ros2 bag record` there; copy bags to the laptop for analysis. Lowest risk.
3. **Direct DDS from WSL2** (not available under NAT; needs mirrored mode, which broke local discovery here, or a VPN-style bridge). Add the laptop's address to the Jetson's static peer list in [`fastdds_profiles.xml`](../../fastdds_profiles.xml) (the profile uses unicast peers, not multicast) and use the same `ROS_DOMAIN_ID`. Gives live RViz of the Jetson's topics. Large point clouds over Wi-Fi will lag; subscribe to downsampled or compressed topics.
4. **Record on the rover, analyse offline.** Best for SLAM work: [`record_bag.sh`](../../scripts/slam/record_bag.sh), copy the bag, replay in WSL2.

### Does a Raspberry Pi in the middle help?

**Not in the data path [I].** An intermediate computer between the Jetson and the laptop adds latency, another failure point, a second ARM Ubuntu/ROS version to keep in step, and can neither run LiDAR odometry nor keep up with point-cloud forwarding. WSL2 can reach the Jetson directly by the methods above.

A Pi is worth having for these roles **[I]**:

| Role | Why a Pi fits | Caveat |
|---|---|---|
| **Hardware-in-the-loop bench** for non-GPU nodes (serial, EKF, NAV logic, Arduino links) | Cheap and disposable; protects the Jetson during wiring experiments | GPIO libraries differ from `Jetson.GPIO`; the `RPi` stack is a separate code path |
| **Field gateway / ground station** (Wi-Fi access point, rosbag recorder, rosbridge relay) | Keeps the operator link alive when the laptop sleeps | One more device to power and configure |
| **LiDAR driver host** | The community L2 driver targets Raspberry Pi 5 **[S]** | Moves 64 000 points/s over a network to the Jetson; only sensible if the Jetson's CPU is the bottleneck, which must be measured first |

Decision: **no Pi until a measured need appears** (a bench for safe wiring tests is the most likely first need).

## 2. Which tool goes in which slot

"Initial pick" is the starting point; the *Revisit when* column names the measurement that would change it.

| Slot | Initial pick | Alternatives considered | Why | Revisit when |
|---|---|---|---|---|
| ROS distribution | **ROS 2 Humble** | Jazzy (needs Ubuntu 24.04) | Matches JetPack 6.x / Ubuntu 22.04 **[S]** | JetPack moves to Ubuntu 24.04 |
| Integration simulator | **Gazebo** (already in repo) | MuJoCo, Isaac Sim, Webots | Native ROS 2 sensors; runs in WSL2; enough fidelity for plumbing | Need photoreal data (Isaac Sim, desktop RTX only) |
| Contact/RL research | **Defer** | MuJoCo, GS-Playground | No learning task yet; GS-Playground is rigid-body, legged/arm, x86 **[P]** | A reinforcement-learning task is defined |
| Soft-terrain physics | **Defer** | PhysGaussian, GASP, MPM engines | Offline research tools **[S]** | Wheel encoders exist and slip data is recorded |
| LiDAR driver | **Unitree `unilidar_sdk2`**, ROS 2 wrapper ported to Humble | Community `l2lidar_node` (GPL-3.0, Jazzy) **[S]** | Vendor-supported SDK, BSD-3 licence; keep GPL code out of this MIT repository's tree **[I; confirm licensing]** | The port proves harder than a thin wrapper over the C++ SDK |
| LiDAR-inertial odometry | **Point-LIO Unitree ROS 2 port** first; **KISS-ICP** as LiDAR-only baseline | FAST-LIO2 ports, DLIO, GLIM | Unitree support exists for Point-LIO **[S]**; KISS-ICP isolates the value of the IMU | Bench-off E1 ([research-plan.md](research-plan.md#e1--odometry-bench-off)) |
| Global pose fusion | **Extend `rover_ekf`** | GTSAM factor graph | Already written, small, understood | EKF cannot hold 3-D slopes or LIO loops → factor graph |
| Visual odometry | **None initially** | cuVSLAM (needs synced stereo + 200–300 Hz IMU **[S]**) | A plain depth camera is not a stereo-IMU rig | A global-shutter stereo camera is added |
| Planner map (baseline) | **Elevation / traversability grid**, then **nvblox** if memory allows | nvblox first | Grid is transparent and cheap; nvblox gives Nav2 integration but its 8 GB footprint is unmeasured **[S]** | E3a memory/latency numbers |
| Navigation/control | **Nav2** (MPPI or regulated pure pursuit) with `NAVnode` as waypoint source | Extend `NAVnode` to drive | Planner, recovery and costmaps exist; Splatblox used Nav2 + MPPI **[M]** | Nav2 too heavy on the Nano |
| Safety stop | **Own corridor check**, independent | Rely on Nav2 | Must not share a failure with the planner | — |
| Gaussian mapping (rover tier) | **Own minimal LiDAR-seeded surfel map** using GS-LIVO's voxel/octree and sliding-window ideas **[M]** | Run GS-LIVO as a dependency | GS-LIVO's build status on JetPack 6.2 is unknown and its README is a demo page **[P]** | E3b shows GS-LIVO builds and fits |
| Gaussian training/rendering (research tier) | **gsplat**-style rasteriser on the workstation GPU | Original 3DGS, 2DGS code | GS-Playground builds on gsplat **[P]**; 2DGS losses for normals **[S]** | — |
| Semantic and foundation models (CLIPSeg, VGGT, SAM, PointGauss) | **Research tier only** | Onboard | Splatblox's CLIPSeg alone is 479 ms per frame **[M]**; VGGT too large for 8 GB shared memory **[I]** | A distilled student replaces them (below) |
| Trajectory evaluation | Scripts in `scripts/slam/` (planned `eval_traj.py`); the community `evo` tool is a candidate **[background]** | — | Reproducible metrics | — |
| Data format | **rosbag2** with zstd | MCAP | Default storage needs no extra plugin | Large-file performance problems |
| Visualisation | **RViz** + the existing web dashboard | Foxglove **[background]** | Already in use | Remote viewing needs improve |

## 3. The two-tier strategy: a full stack and a baby stack

The idea you proposed — build a full version, then a reduced version from it — is the right structure, and it is a standard pattern (a heavy "teacher" produces labels and reference maps, a light "student" runs on the robot) **[background]**. Here it maps cleanly onto the hardware.

```
                       recorded bags (real rover, real terrain)
                                   │
          ┌────────────────────────┴────────────────────────┐
          ▼                                                 ▼
 RESEARCH TIER  (WSL2 / workstation, GPU)           ROVER TIER  (Jetson Orin Nano Super)
  full Gaussian/2DGS map, high resolution            LiDAR-inertial odometry
  semantic labelling (CLIPSeg, PointGauss)           elevation/traversability baseline
  optional VGGT, MPM / slip studies                  capped surfel Gaussian layer (optional)
  heavy evaluation and plots                         geometry + intensity traversability (small model)
          │                                                 │
          │  reference maps, labels, training data          │  same topics, same message types
          └────────────► distillation ────────────────────► │
                                                            ▼
                                              /terrain/traversability → Nav2 / safety
```

### What each tier is for

| | Research tier ("full") | Rover tier ("baby") |
|---|---|---|
| Hardware | RTX 4060 Laptop GPU in WSL2, later any workstation | Orin Nano Super, 8 GB shared |
| Runs | Offline on bags; no latency target | Live; budgets fixed in [architecture.md](architecture.md#compute-and-memory-budget-targets-not-measurements) |
| Contents | Everything in [literature.md](literature.md) worth trying | Only what fits the budget and has passed a gate |
| Purpose | Produce the best possible map and labels; discover what matters; publishable results | Drive the rover reliably; the competition configuration |
| Failure cost | None | Safety |

Both tiers use the **same inputs** (the [sensor contract](architecture.md#sensor-contract-camera-agnostic)), the **same outputs** (`/terrain/traversability`, `/slam/odometry`) and the **same evaluation scripts**. That makes the research tier a drop-in *reference* for the rover tier and lets any improvement migrate by swapping an implementation, not an interface.

### How the full stack improves the baby stack (distillation)

| Step | Full tier provides | Baby tier gains | Within the hardware limit because |
|---|---|---|---|
| 1. Reference maps | High-resolution LiDAR + camera Gaussian/2DGS maps from repeated passes | A ground-truth-like target to measure the live map against | Runs offline, once per site |
| 2. Labels | Semantic traversability from heavy networks, plus human corrections, plus where the rover actually drove successfully | Training data for a small classifier | Labelling cost is paid on the workstation |
| 3. Student model | — | A tiny per-cell model (gradient-boosted trees or a small MLP) on geometry features: slope, roughness, step height, point density, LiDAR intensity, and colour when a camera is present | Runs on the CPU in milliseconds; no GPU, no network, no 479 ms CLIPSeg |
| 4. Parameter tuning | Large sweeps of voxel size, Gaussian cap, grid resolution | Settings that hold the memory budget | Sweeps use the workstation, not the rover |
| 5. Route priors | Full maps of a known course | Optional map prior for repeated routes (a rerun competition course) | Loads a static map instead of building one |
| 6. Regression tests | Reference results per bag | Pass/fail checks for every code change | Replay in WSL2 |

A practical bonus: the Unitree LiDARs return **grayscale (intensity) as a fourth value per point** **[P]**. That cheap per-point reflectivity is a material cue (rock vs sand vs vegetation, wet vs dry) that the student model can use with no camera at all **[I]**.

### Improvements available inside the hardware limits

| Idea | Gain | Cost | Label |
|---|---|---|---|
| Body and arm mask plus moving-point rejection before mapping | Stops the arm and people from becoming obstacles | Negligible | [I] |
| Ground segmentation ahead of terrain features | Cleaner slope/roughness | Small CPU | [I] |
| Confidence layer (point count, age) in the costmap | Planner distinguishes "free" from "unknown" | Memory per cell | [I] |
| Rolling-window map instead of a global map | Constant memory | Loses old areas; use a separate offline global map | [I] |
| Depth camera for near-field; LiDAR for far field | Covers the LiDAR blind zone and mount shadow | Second sensor stream | [I] |
| Self-supervised traversability from driving outcomes (slip, stall, tilt) | Labels without humans | Needs wheel-speed feedback the rover lacks today ([known-issues.md](../known-issues.md)) | [I] |
| INT8/TensorRT only for small networks | Speed on Tensor Cores | Splatting itself does not use them **[I]** | [I] |
| Power-mode profiles (competition vs research) | Predictable thermals | Lower peak speed | [I] |

### Two launch profiles

| Profile | Rover tier components | Extras |
|---|---|---|
| `competition` | Layers 1–2 and the student traversability model; budgets enforced; no heavy logging | None |
| `research` | `competition` + optional Gaussian layer + full-rate bag recording + load logging | [`record_bag.sh`](../../scripts/slam/record_bag.sh), [`bench_tegrastats.sh`](../../scripts/slam/bench_tegrastats.sh) |

Both are launched from the existing process manager ([code-map.md](code-map.md)); there is still no third start-up script.

## 4. Phased plan

| Phase | Where | Deliverable | Gate to continue |
|---|---|---|---|
| 0. Environment | Windows / WSL2 | Ubuntu 22.04 distro, Humble, repository builds, Gazebo launches | `colcon build` passes in WSL2 |
| 1. Simulated sensors | WSL2 | LiDAR + depth-camera model in `rover_description`; the sensor contract topics appear in Gazebo | Topics and TF correct in RViz |
| 2. Real sensors | Jetson | Driver running, bags recorded ([E0](research-plan.md#e0--bring-up-and-calibration)) | 10-minute replayable bag |
| 3. Rover tier baseline | Jetson | Odometry + fusion + elevation baseline ([E1–E3a](research-plan.md)) | Meets Layer 1–2 budgets |
| 4. Research tier | WSL2 | Reference maps, labels, comparison scripts | Reference maps for ≥ 2 sites |
| 5. Distillation | WSL2 → Jetson | Student traversability model in the rover tier | Student within an agreed margin of the teacher |
| 6. Gaussian layer | Both | Capped surfel layer ([E3b](research-plan.md#e3--planner-map-and-the-gaussian-layer)) | Beats the baseline within budget, or stop |
| 7. Closed loop | Jetson | Nav2 runs, safety stop, field trials ([E4](research-plan.md#e4--closed-loop-navigation)) | Success rate target set in advance |

## 5. Risks specific to these decisions

- **WSL2 is not the rover.** Timing, USB and CAN behaviour must be confirmed on the Jetson; do not treat a passing WSL2 run as a hardware result.
- **GPU memory on the laptop** (8 GB at best) limits the research tier too; large scenes will need chunking or a bigger machine.
- **Distillation can bake in the teacher's errors.** Keep human labels and driven-outcome labels in the training set and report student accuracy against them, not only against the teacher.
- **Licences.** The community L2 driver is GPL-3.0; keep third-party code in the overlay workspace and review before linking anything into the MIT repository.
- **Scope.** The research tier is where the deferred topics (MPM, 4D, VGGT) can live without touching the rover; resist moving them into the rover tier without a gate.
