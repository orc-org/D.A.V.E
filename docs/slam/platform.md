# Platform: Jetson Orin Nano Super + Unitree sensors

Labels: **[P]** primary page, **[S]** search/third-party, **[M]** model-extracted summary, **[I]** inference, **[?]** not found. See the legend in [README.md](README.md#evidence-labels).

## Compute: Jetson Orin Nano Super Developer Kit

The official product page you linked was not fetched; the figures below come from retailer and press pages ([sources](sources.md#jetson)) and agree with each other. Confirm them against NVIDIA's datasheet before relying on a number.

| Item | Value | Label |
|---|---|---|
| AI performance | 67 TOPS (sparse INT8); about 33 TOPS dense | [S] |
| Memory | 8 GB 128-bit LPDDR5, about 102 GB/s (the original Orin Nano had about 68 GB/s) | [S] |
| CPU | 6-core Arm Cortex-A78AE | [S] |
| GPU | 1024-core Ampere, 32 Tensor Cores | [S] |
| Power modes | 7 W, 15 W, 25 W, and a MAXN_SUPER mode | [S] |
| Software | JetPack 6.2 (Jetson Linux 36.4.3, kernel 5.15, Ubuntu 22.04 root file system, CUDA 12.6, TensorRT 10.3, cuDNN 9.3, VPI 3.2); Humble is the matching ROS 2 release | [S] |
| Thermals | One user review reports throttling in Super mode and a quiet fan default. Anecdotal | [S] |

What this means for the work **[I]**:

- Memory is **shared** between CPU and GPU. "8 GB" is the budget for the operating system, ROS 2, every node, the lidar driver, the map and any GPU buffers together. Memory, not TOPS, is the likely limit for Gaussian methods.
- Six CPU cores are fewer than the boards used in most Jetson SLAM papers (see below). LiDAR-inertial odometry is mostly CPU-bound, so core count and the 25 W / MAXN_SUPER setting matter.
- The Tensor-core TOPS figure is irrelevant to splatting and classical SLAM; those use CUDA cores and memory bandwidth.
- The repository already targets Ubuntu 22.04 + Humble ([build-and-run.md](../build-and-run.md)), so JetPack 6.x is consistent with the existing workspace.

### How the published Jetson results compare to this board

| Result | Board | Note | Label |
|---|---|---|---|
| GS-LIVO: over 10 Hz indoors, about 3 Hz outdoors; tested on Jetson Orin NX 16 GB (8-core CPU, 1024 CUDA cores, 16 GB) | Orin NX 16 GB | Twice the memory and two more CPU cores than the Nano. Images were reduced to 256×216 on the Jetson | [P]/[M] |
| Splatblox: under 6 GB VRAM at about 2 Hz | RTX 3070 laptop, not a Jetson | The "edge" claim is not demonstrated on an edge device | [M] |
| Continual 3DGS training on Orin Nano (arXiv 2603.08499): first to train on the Nano, 19× lower per-frame latency than 3DGS | Orin Nano | Novel-view synthesis, not SLAM; exact FPS/memory not read | [S] |
| nvblox / Isaac ROS: Orin Nano 4 GB "not recommended"; 8 GB is the safer target | Orin Nano | Official docs | [S] |
| cuVSLAM: 113 fps (about 65 ms) at 720p on Orin Nano 8 GB | Orin Nano 8 GB | Older benchmark table | [S] |
| FAST-LIO2 at 100 Hz odometry, 60–70 % CPU | Orin NX with a Livox Mid-360 | Blog post, unverified; one other write-up thinned the cloud to 1/6 to cut CPU | [S] |
| Standard 3DGS training needs about 9.44 GB peak GPU memory (A5000) | desktop | Does not fit the Nano's 8 GB shared pool | [S] |

**No source was found that runs a LiDAR SLAM or Gaussian SLAM system on an Orin Nano with a Unitree LiDAR.** That is the first thing to measure ([research-plan.md](research-plan.md#e1--odometry-bench-off)).

## LiDAR: Unitree 4D LiDAR L2 (preferred) and L1

From Unitree's product pages and the `unilidar_sdk2` README. The user-manual PDF could not be read by the fetch tool, so manual-only facts are marked **[?]**.

| | L2 | L1 PM / L1 RM |
|---|---|---|
| Price (Unitree page) | $419 | from $249 / from $309 |
| Field of view | 360° × 96° (96° only in "negative angle mode"; 90° standard) | 360° × 90° |
| Scan type | Rotating mirror, non-repetitive pattern | Rotating mirror, non-repetitive pattern |
| Point rate | 128 000 pts/s sampled, **64 000 effective** | 43 200 sampled, **21 600 effective** |
| Rotation | 5.55 Hz circumferential (customisable), 216 Hz vertical | 11 Hz circumferential, 180 Hz vertical |
| Range | 30 m @ 90 % reflectivity (10 % figure not given) | PM 20 m / 10 m, RM 30 m / 15 m (@90 % / @10 %) |
| Accuracy | ≤ 2.0 cm; 4.5 mm distance resolution | ±2.0 cm |
| Blind zone | 0.05 m | 0.05 m |
| IMU | Built-in 3-axis accelerometer + gyroscope; model and rate **[?]** (the SDK sample shows about 2.6 ms between IMU stamps, suggesting roughly 380 Hz; an estimate **[I]**) | Built-in 3-axis accel + gyro |
| Interface | Ethernet UDP (default) or TTL UART | TTL UART |
| Power | 10 W typical, 13 W peak; self-heats below about 30 °C ambient and emits no cloud until warm | 6 W |
| Weight / size | 230 g, 75 × 75 × 65 mm | 230 g, 75 × 75 × 65 mm |
| Light tolerance | > 100 klux, "indoor/outdoor"; IP rating and operating temperature **[?]** | > 100 klux |
| Eye safety | Class 1 | Class 1 |

Why the L2 is the better fit **[I]**: three times the effective point rate (denser per sweep), a slower mirror (one user reports it is much more stable than the L1), and Ethernet instead of UART-only. The price difference is $110–$170.

### Software support

| Item | Finding | Label |
|---|---|---|
| Official SDK | `unilidar_sdk2`: C++ SDK, ROS 1 package, ROS 2 package; BSD-3-Clause. **Verified environments: Ubuntu 20.04 with ROS 1 Noetic or ROS 2 Foxy, PCL 1.10.** Humble on Ubuntu 22.04 is not listed | [P] |
| Topics (defaults) | `unilidar/cloud` (frame `unilidar_lidar`), `unilidar/imu` (frame `unilidar_imu`). The README does not state message types (assumed `PointCloud2` and `Imu`) | [P]/[I] |
| Frames | LiDAR frame origin at the centre of the bottom mounting surface, +X opposite the cable outlet, +Z up. IMU frame has the same orientation, origin offset **(−0.007698, −0.014655, 0.00667) m** in the LiDAR frame | [P] |
| Work-mode bits | Bit 0 standard vs wide FOV; bit 1 3-D vs 2-D; bit 2 IMU on/off; bit 3 Ethernet vs serial; bit 4 auto-start | [P] |
| Community ROS 2 driver | `markgol/l2lidar_node`: ROS 2 Jazzy, Ubuntu 24.04, x86_64 and Raspberry Pi 5, GPL-3.0, publishes `/points` and `/imu/data` | [S] |
| LiDAR-inertial odometry for Unitree | A ROS 2 port of Point-LIO with Unitree L1/L2 support exists, using Unitree's `unilidar_sdk` drivers; Unitree itself offers a Point-LIO build for the L1 | [S] |
| Reports of FAST-LIO2 on a Unitree-class platform | One write-up saw several metres of z drift and had to thin the cloud to 1/6 | [S] |

### Integration problems found on paper

1. **Driver port.** The only official ROS 2 build is verified on Foxy / Ubuntu 20.04. Expect to port or patch it for Humble / Ubuntu 22.04 / JetPack 6. The C++ SDK underneath is independent of ROS, so the port should be small, but it is unproven **[I]**.
2. **Network.** The SDK README configures the *host* network card to `192.168.1.2` for the default Ethernet mode. The rover's DDS configuration already uses `192.168.1.87`, `192.168.1.88` and `127.0.0.1` as peers on the same `/24` ([dashboard-and-network.md](../dashboard-and-network.md#dds--network-configuration-fastdds_profilesxml)). Put the LiDAR on a **separate interface or subnet**, or use UART mode (`/dev/ttyACM0` by default, work-mode bit 3).
3. **One Ethernet port.** The developer kit has a single gigabit Ethernet port (**[I]**, from the kit layout; not looked up). It is also the rover's link to the base station. Use a USB-Ethernet adapter for the LiDAR, a switch with VLANs, or UART.
4. **Serial port clash.** Several nodes already default to `/dev/ttyUSB0` ([hardware.md](../hardware.md#serial-devices)). The L2's serial default is `/dev/ttyACM0`, which the arm's Arduino driver also uses.
5. **Cold start.** Below about 30 °C ambient the L2 warms itself and outputs no cloud until a temperature threshold is met. Odometry start-up must tolerate a delay (a "wait for cloud" gate).
6. **Time sync.** No hardware time-synchronisation (PPS or PTP) feature was found for either LiDAR **[?]**. GS-LIVO emulates a PPS signal for its multi-sensor sync; any LiDAR–camera fusion here needs a measured time offset ([research-plan.md](research-plan.md#e0--bring-up-and-calibration)).
7. **Power.** 10–13 W for the L2 on top of the Jetson's 7–25 W changes the battery budget. The wiring and voltage were not found **[?]**.

## Camera

Unitree does **not** sell a bundled LiDAR + camera **[P]**. What its pages show:

- The L2 and L1 pages compare against an unnamed "A depth camera" at **$359**: infrared binocular, global shutter, 86° × 57° FOV, **4–5 m range**, 0.1 m blind zone, 5 W, 90 × 25 × 25 mm, no IMU. It is not named or linked, and it does not match the 0.3–10 m figure on the A1 robot page, so it cannot be tied to a specific product **[P]**.
- Unitree's G1 humanoid ships an **Intel RealSense D435i** depth camera **[S]**.
- The "A1" page is the A1 quadruped robot, not a camera **[P]**.

A 4–5 m depth range is useless for outdoor terrain at rover speeds, so a depth camera adds little over the LiDAR. What a camera adds is **colour and texture** (for the Gaussian layer and for classification) and **visual odometry** if it is stereo and synchronised.

**Design assumption (decided):** the rover will carry a 3-D LiDAR (a Unitree model) and a depth camera. The exact camera is not fixed and may be a Unitree depth camera or a general-purpose one, so the design depends only on the **sensor contract** below, not on a model. See [architecture.md](architecture.md#sensor-contract-camera-agnostic).

What the depth camera is for **[I]**:

- **Near field.** The LiDAR has a 0.05 m blind zone and, mounted high, may not see the ground close to the wheels or low obstacles in front of the rover. A depth camera covers roughly the first few metres.
- **Colour and texture** for the Gaussian layer and for any classification.
- **Redundancy.** If the LiDAR fails or is shadowed by the arm, the camera keeps a short-range map alive.

What it is *not* for: long-range mapping or odometry (infrared depth is short-range and degrades in sunlight; the unnamed Unitree depth camera quotes only 4–5 m).

If a camera is still to be chosen, the options are **[I]**:

| Option | Gives | Cost to the design |
|---|---|---|
| Existing CSI/USB cameras (already on the rover) | RGB for the Gaussian layer; no depth | Needs LiDAR–camera extrinsic and time offset; rolling shutter on CSI sensors may hurt photometric optimisation |
| Global-shutter stereo + IMU module | cuVSLAM visual odometry (needs about ±100 µs camera sync and 200–300 Hz IMU per NVIDIA) and RGB | New hardware; the L2 IMU can serve as the IMU if time-aligned |
| RealSense D435i | Depth to a few metres, RGB, built-in IMU | Short range outdoors; infrared depth degrades in sunlight |

## Open hardware questions

- Which camera exactly (model, interface, resolution, shutter type)?
- L2 or L1? (Recommendation: L2.)
- How will the LiDAR be mounted (height, tilt, field of view blocked by the mast or arm)? The arm column and mount ([arm-and-kinematics.md](../arm-and-kinematics.md#the-arm-as-configured-motion_coordinator-dimensions-from-rover_descriptionconfigphysical_paramsyaml)) can shadow part of a 360° scan.
- Can the Orin Nano Super hold MAXN_SUPER thermally inside the rover enclosure?
- Is there an RTK-capable GNSS to provide ground truth for evaluation?
