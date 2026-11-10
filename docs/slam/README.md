# SLAM and perception program

Goal: give D.A.V.E. a pose estimate that does not depend on GNSS alone, a map it can plan against, and a way to avoid obstacles it has not been told about. The target hardware is an **NVIDIA Jetson Orin Nano Super Developer Kit** with a **Unitree 4D LiDAR (L2 preferred, L1 as the cheaper option)** and a camera.

This folder is the working record of that program: what was researched, what was decided, what is still a guess, and exactly where in the repository each piece lives or will live.

## Design assumptions (set by the team)

- **Sensors:** a 3-D LiDAR (a Unitree model; the L2 is preferred) and a depth camera. The camera model is deliberately **not** fixed; everything is designed against the [sensor contract](architecture.md#sensor-contract-camera-agnostic) so a Unitree depth camera or a general-purpose one can be dropped in.
- **Compute:** Jetson Orin Nano Super Developer Kit (8 GB shared memory).
- **Software:** ROS 2 Humble on Ubuntu 22.04 (JetPack 6.x). Why ROS 2, whether Linux is required and how it compares with MuJoCo and other simulators is in [../ros-and-simulation.md](../ros-and-simulation.md).

> **Competition context:** the five CIRC 2027 tasks (one of them with GPS banned, one at night) are summarised in [../competition.md](../competition.md); they set the priorities for this program.

## Read in this order

| # | Page | Read it to learn |
|---|---|---|
| 1 | [platform.md](platform.md) | What the Jetson and the Unitree sensors can and cannot do, and integration problems found on paper |
| 2 | [literature.md](literature.md) | Every topic on the original list, what it really is, and whether it belongs on a rover |
| 3 | [architecture.md](architecture.md) | The proposed three-layer design, frames, topics, compute budget |
| 3b | [decisions.md](decisions.md) | Development environment (WSL2, Jetson, optional Pi), the tool chosen for each slot, and the two-tier research/rover strategy |
| 4 | [research-plan.md](research-plan.md) | Experiments, metrics, decision gates, risks, and the novelty claim |
| 5 | [code-map.md](code-map.md) | Concept → file map: existing code to reuse, new code to write, scripts, config |
| 6 | [sources.md](sources.md) | Every link used, grouped, with how it was read |

Related pages outside this folder: [known-issues.md](../known-issues.md) (the pre-existing gaps this program fills), [state-estimation.md](../state-estimation.md) (the filters that exist today), [navigation.md](../navigation.md) (the obstacle interface), [hardware.md](../hardware.md) (pins, ports, network).

## Evidence labels

Every factual claim in this folder carries a label where the source quality matters:

- **[P]** read from a primary page (the vendor, the paper, the repository) during this research.
- **[S]** seen only in search-result text or a third-party summary. Treat as a lead.
- **[M]** model-extracted summary of a page; numbers should be re-checked against the paper before citing.
- **[I]** inference or recommendation by the author of these notes. Not measured.
- **[?]** could not be found; stated so nobody assumes it.

Nothing here has been run on the Jetson. Every performance figure is either a published claim on *different* hardware or a target.

## Where things stand

- **Done:** literature and hardware research; architecture proposal; research plan; repository scaffolding for data capture and benchmarking (see [code-map.md](code-map.md)).
- **Not done:** no sensor has been connected; no SLAM package is installed; the Unitree driver has not been built for JetPack 6 / ROS 2 Humble; nothing has been benchmarked on the Orin Nano.
- **Open decisions** (details in [research-plan.md](research-plan.md#decisions-needed)): L1 or L2, whether the Gaussian layer is a research contribution or an optional extra, and which odometry front end wins the bench-off. The camera model is no longer blocking; only the contract has to be met.

## The recommendation in six lines

1. **Do not make Gaussians do the tracking.** Run a LiDAR-inertial odometry front end for pose; it is cheap, well proven and the L2 already ships an IMU.
2. **Fuse that pose with GNSS and wheel speed** in the existing `rover_ekf` (or a factor graph) so there is one global pose.
3. **Build a baseline planner map first** (2.5-D elevation grid and/or nvblox ESDF → Nav2 costmap) so the rover can drive autonomously without any research risk.
4. **Add the Gaussian map as a separate, optional layer** seeded by the LiDAR points, with surfel-style (2DGS) normals, and score terrain traversability from it. This is the research contribution, and the closest prior work (Splatblox) was not run on a Jetson and has no released code.
5. **Gate everything on measurements** from the real board, with memory and latency budgets fixed in advance.
6. **Treat physics (MPM) and dynamic-scene (4D) topics as later work**, not as part of the first build.

## Corrections to the original topic list

The research found that some items on the original list are not what the name suggests. These matter before anyone builds on them:

- The link `arxiv.org/abs/2504.13153` is **not** 2D Gaussian splatting. It is *Training-Free Hierarchical Scene Understanding for Gaussian Splatting with Superpoint Graphs* (3DGS semantic segmentation). The 2DGS paper is `arxiv.org/abs/2403.17888`.
- **GS-Playground** is a rigid-body robot-learning simulator. It does **not** include MPM or granular terrain. MPM + Gaussians is **PhysGaussian** / **GASP**.
- "Unitree lidar/camera": Unitree sells the LiDARs; it does **not** sell a bundled LiDAR + camera. See [platform.md](platform.md#camera).

Full detail: [literature.md](literature.md).
