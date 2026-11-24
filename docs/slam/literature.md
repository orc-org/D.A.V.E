# Literature review

Every topic from the original list, checked against the papers and repositories. Labels: **[P]** primary page read, **[S]** search result or third-party summary, **[M]** model-extracted summary of a page (re-check numbers), **[I]** inference, **[?]** not found. Links are in [sources.md](sources.md).

## Summary table

| # | Topic (as listed) | What it actually is | Relevance to a wheeled rover on a Jetson | Priority |
|---|---|---|---|---|
| 1 | 3DGS with seeded LiDAR | A family: **GS-LIVO**, LIV-GaussMap, GS-LIVM, LVI-GS, Gaussian-LIC(2). Gaussians are initialised from LiDAR points and refined photometrically | **Core.** GS-LIVO is the only one shown on a Jetson (Orin NX 16 GB) | High |
| 2 | Gaussian primitives with point-cloud application | Same family as (1); also PointGauss (6) and Gaussians-to-point-cloud conversion used by Splatblox | Core, as the representation | High |
| 3 | Material point method (MPM) | A physics method for deformable/granular matter. With Gaussians: **PhysGaussian**, **GASP** | Possible *terrain physics* research; not an onboard mapping tool | Later |
| 4 | GS-Playground | A **rigid-body** robot-learning simulator with batch 3DGS rendering (RSS 2026) | Poor fit: no wheeled robots, no granular terrain, x86 CUDA 12.8 | Skip for now |
| 5 | Improved 3DGS integrating VGGT and LiDAR | A journal paper on UAV visual relocalisation for aircraft inspection; closest preprint is **LiDAR-VGGT** | Idea (metric-scale-from-LiDAR seeding) is useful; VGGT is too heavy for the Nano | Watch |
| 6 | PointGauss | Segmentation *of* a Gaussian scene using point-cloud networks | Labelling objects in a finished map; not needed to navigate | Low |
| 7 | Trilinear point splatting (TRIPS) | A point-based neural renderer (Eurographics 2024), 60 fps on desktop hardware | Rendering quality, not mapping | Low |
| 8 | 2D Gaussian splatting | **2DGS** (SIGGRAPH 2024): flat oriented disks with true surface normals | **Core**, for terrain surface and slope. *The linked arXiv number is a different paper* | High |
| 9 | TSDF | Truncated signed distance fields; **nvblox** is the GPU implementation for Jetson | **Baseline planner map** | High |
| 10 | Native 4D primitives + canonical deformation field | Two families of dynamic-scene Gaussians (4D Gaussian splatting; deformation-field methods) | Moving objects only; the rover's environment is mostly static | Later |
| 11 | Spatiotemporal occupation grids via neural SDFs | No specific paper found **[?]** | Unknown | Verify source |
| 12 | Dynamic rigid-body clustering (scaffolds) | No specific paper found **[?]** | Unknown | Verify source |

## 1–2. LiDAR-seeded Gaussian maps

**GS-LIVO** (Hong, Zheng, Shen, Li, Zhang, Qin, Shen; HKUST; arXiv 2501.08672, Jan 2025).

- LiDAR–inertial–visual odometry with a Gaussian map. Odometry is an iterated error-state Kalman filter (IESKF), modified from FAST-LIVO2, tightly coupling LiDAR, IMU and camera; the visual term compares the *rendered* Gaussian image with the camera image rather than warping patches **[M]**.
- Map: a global hash-indexed octree of voxels, plus a **sliding window** of Gaussians that is the only part optimised on the GPU. The full map stays in CPU RAM; only Gaussians in view are copied to the GPU **[M]**.
- Seeding: Gaussians are created per octree leaf voxel from LiDAR points; each is a flat (surfel-like) Gaussian whose rotation comes from the LiDAR-inertial surface normal, with colour sampled from the image. The point-selection rule inside a voxel is not clearly specified **[M]**.
- Embedded result: "over 10 Hz indoors on a single CPU thread and around 3 Hz outdoors", on a Jetson Orin NX with 16 GB **[P]**. A model summary gives a 48.3 ms total for a 20 000-Gaussian window with 256×216 images (15.3 ms optimisation, 18.9 ms map maintenance) **[M]**.
- Accuracy: comparable to, not better than, FAST-LIVO outdoors (for example HKisland03 RMSE 0.58 m vs 0.51 m) **[M]**; the summary itself flagged inconsistencies, so re-read Tables II–IV.
- Authors' stated limitations: slightly lower accuracy and higher cost than sparse visual maps; fixed octree level; merging similar Gaussians left as future work **[M]**.
- Code: repository `HKUST-Aerial-Robotics/GS-LIVO` is public (about 500 stars); the README is a demo page and says "Tested on Jetson Orin NX (16GB)". Whether `src/gs-livo` is a complete buildable system was not checked **[P]**.

**Related.** *LIV-GaussMap* (arXiv 2401.14857) builds a LiDAR–inertial–visual Gaussian map using surface-splatting Gaussians; *GS-LIVM* (arXiv 2410.17084) targets outdoor scenes with size-adaptive voxels; *LVI-GS* adds a LiDAR depth loss; *Gaussian-LIC* and *Gaussian-LIC2* combine Gaussian splatting with LiDAR–inertial–camera SLAM **[S]**. Detail beyond the abstracts was not read.

**Survey caution.** A 2025 comparison on a Jetson AGX Orin (arXiv 2505.12384) concludes NeRF- and Gaussian-based methods give rich semantic detail but demand substantial compute, limiting embedded use **[S]**.

**Verdict.** This is the right family and the right seeding strategy. The only embedded demonstration is on a larger board than ours and at reduced image size, and it runs at about 3 Hz in outdoor scenes. Expect the Orin Nano to be slower unless the Gaussian count and image size are capped **[I]**.

## 3. Material point method

MPM simulates continuum matter (sand, snow, jelly) by moving particles through a background grid.

- **PhysGaussian** (arXiv 2311.12198; code `XPandora/PhysGaussian`) attaches mechanical stress and deformation to 3D Gaussian kernels and simulates them with a custom MPM. Example materials include jelly, metal, sand, foam, snow and plasticine **[S]**.
- **GASP** (arXiv 2409.05819) combines flat Gaussians with MPM for physics-based simulation **[S]**.
- **PhysGS** (arXiv 2511.18570) estimates physical properties of a scene with Bayesian-inferred Gaussian splatting **[S]**; its terrain relevance was not confirmed.

Relevance **[I]**: these are simulation and offline analysis tools. They could support a research idea such as *predicting wheel sinkage or slip on soft ground from a Gaussian terrain map*, but nothing in the sources demonstrates that, and it needs wheel-ground interaction data the rover does not currently record (no encoders; see [known-issues.md](../known-issues.md)). Keep it as a stretch goal.

## 4. GS-Playground

*GS-Playground: A High-Throughput Photorealistic Simulator for Vision-Informed Robot Learning* (arXiv 2604.25459, RSS 2026) **[P]**:

- Couples a rigid-body physics engine (MotrixSim) with batch 3DGS rendering for reinforcement learning. About 10⁴ FPS aggregate over 2 048 environments at 640×480 on a desktop GPU (RTX 4090 per the project page).
- **Rigid bodies only.** "Deformable and soft-body objects are unsupported"; PBD or MPM are future plans. No sand, mud or soil.
- Robots evaluated: Unitree Go1/Go2, G1 humanoid, AIRBOT Play arm. **No wheeled robots, no outdoor scenes.**
- Real-to-sim: a single RGB image becomes a scene through Grounding DINO, SAM, LaMa, SAM-3D and AnySplat; under 5 min on an RTX 3090.
- Sensors include a batch LiDAR (ray casting) and height scan, but the repository lists the LiDAR module as planned.
- Code: partial preview (`discoverse-dev/gs_playground`, MIT); "the paper system is larger than this preview repository". Linux x86_64, NVIDIA GPU, CUDA 12.8.

Verdict: runs on a workstation, not the Jetson, and is not a rover simulator. The rover already has a Gazebo model ([rover_description](../../ros2_ws/src/rover_description/README.md)). Revisit only if a photorealistic sim-to-real learning pipeline becomes a goal.

## 5. VGGT with LiDAR

- *Improved 3D Gaussian splatting integrating VGGT and LiDAR: application in visual relocalization for UAV inspection of large aircraft* (journal paper, DOI 10.1063/5.0342109) registers VGGT point clouds to LiDAR clouds so the Gaussians start with real metric scale, then uses a coarse-to-fine image-matching strategy **[S]**. Full text not read.
- *LiDAR-VGGT* (arXiv 2511.01186) couples LiDAR-inertial odometry with VGGT because feed-forward VGGT is dense but has no metric scale **[S]**.
- VGGT is a feed-forward transformer. Its size and memory make it unlikely to run alongside everything else on 8 GB shared memory **[I]**; no Orin Nano figure was found.

Verdict: the *idea* (use LiDAR to give a learned or visual map metric scale) is already satisfied by using the LiDAR directly. Skip VGGT unless a camera-only fallback is wanted.

## 6. PointGauss

*PointGauss: Point Cloud-Guided Multi-Object Segmentation for Gaussian Splatting* (arXiv 2508.00259) applies point-cloud segmentation networks directly to Gaussian primitives, producing 3D instance masks "within 1 minute" plus a GPU 2D mask renderer; reports 1.89–31.78 % multi-view mIoU gains and a new benchmark, DesktopObjects-360 **[S]**.

Verdict: object-level labelling of a finished scene. Not on the navigation path.

## 7. TRIPS

*TRIPS: Trilinear Point Splatting for Real-Time Radiance Field Rendering* (arXiv 2401.06003, Eurographics 2024) rasterises points into a multi-layer image pyramid and refines with a small network; 60 fps on "readily available hardware" **[S]**. It improves image quality, not geometry or localisation.

## 8. 2D Gaussian splatting (and the wrong link)

- **2DGS**: *2D Gaussian Splatting for Geometrically Accurate Radiance Fields* (Huang, Yu, Chen, Geiger, Gao; arXiv 2403.17888; SIGGRAPH 2024) **[S]**. Flat oriented disks with explicit ray–splat intersection give view-consistent depth and **true normals**; a depth-distortion loss and a normal-consistency loss regularise geometry; meshes can be extracted. The normals are what make it useful here: **slope and roughness are direct functions of splat normals**.
- **The linked paper is different.** `arxiv.org/abs/2504.13153` is *Training-Free Hierarchical Scene Understanding for Gaussian Splatting with Superpoint Graphs* (Dai et al., 17 Apr 2025), about 3DGS open-vocabulary segmentation. It does not mention 2DGS **[P]**. Code promised at `Atrovast/THGS`.

## 9. TSDF and nvblox

- A TSDF stores signed distance to the nearest surface in voxels, truncated near the surface; an ESDF is derived from it for planning.
- **Isaac ROS nvblox** is NVIDIA's GPU implementation with ROS 2 packages for 3-D reconstruction and cost maps. It supports depth cameras and/or 3-D LiDAR, can use several sensors at once, and exposes a Nav2 costmap plugin (a static map slice added to the local costmap beside the inflation layer) **[S]**. Official docs say the Orin Nano **4 GB** may not have enough memory for many Isaac ROS packages and is not recommended; the 8 GB is the safer target **[S]**.
- Forum reports mention a single costmap layer and accuracy that depends on depth quality **[S]**.

Verdict: the planner-ready baseline, and the comparison point for the Gaussian layer.

## Closest prior art to the proposed contribution: Splatblox

*Splatblox: Traversability-Aware Gaussian Splatting for Outdoor Robot Navigation* (Chopra, Liang, Seneviratne, Lee, Choi, An, Cheng, Manocha; University of Maryland; arXiv 2511.18525, v2 Apr 2026) **[P]/[M]**:

- Segments RGB with CLIPSeg (terrain-word prompts) to a per-pixel traversability cost, attaches it to **LiDAR points**, turns the points into anisotropic Gaussians, builds a 3-D distance field, **rasterises it to a 2-D ESDF** in the camera's frontal region, and fuses it with an nvblox LiDAR ESDF for 360° coverage. The fused field drives Nav2 with the MPPI controller.
- Robots: a Ghost Vision 60 quadruped and a Clearpath Jackal wheeled robot; sensors: a wide-angle camera, Ouster OS1-32 LiDAR, IMU; poses from LIO-SAM.
- Compute: a **laptop (RTX 3070, Ryzen 7 5800H) tethered by Ethernet**, not an onboard edge device. Intro claims "less than 6 GB of VRAM at ≈2 Hz". Table III: adding 100 000 Gaussians costs 4 000 MB VRAM; Gaussian part 159 ms; CLIPSeg 479 ms per frame; end-to-end 2.09 Hz.
- Claims: over 50 % higher success rate, 40 % fewer freezing incidents, 5 % shorter paths, up to 13 % faster time to goal (the body text gives slightly different figures, so the abstract and body disagree).
- Sample sizes are small: 10 runs per scenario in four short-range scenarios, 3 runs per scenario in two long-range scenarios.
- **No code** ("Coming Soon"). Authors say the system is best suited to quasi-static environments and degrades under challenging lighting.

What this tells us **[I]**:

1. A Gaussian-to-costmap pipeline for outdoor navigation has been published, so "Gaussian traversability" alone is not novel.
2. It leaves open: running on a **Jetson-class device**, doing it **without a heavy semantic network**, replacing semantics with **geometry-derived traversability from surfel normals**, and comparing against a plain **elevation-map baseline** on rover-style terrain.
3. Its memory numbers (4 GB for 100 k Gaussians) show why the Nano needs a Gaussian cap.

## 10–12. Dynamic and 4D topics

- *Native 4D primitives*: 4D Gaussian splatting optimises Gaussians in space-time directly (Yang et al., arXiv 2310.10642). *Canonical deformation field*: a static set of 3D Gaussians plus a learned deformation per timestep. Hybrid methods use 4D Gaussians only in dynamic regions. Storage grows for long sequences **[S]**.
- *Spatiotemporal occupation grids via neural SDFs* and *Dynamic rigid-body clustering (scaffolds)*: searches returned nothing on these specific terms **[?]**. If they come from a particular paper or talk, supply the source and the item can be reviewed properly.

Verdict **[I]**: these address moving or deforming scenes. A rover mapping rocks and slopes is largely static; dynamic handling here means *rejecting moving points* so people and the rover's own arm do not corrupt the map. That is a filtering problem solvable in the odometry front end, not a 4D representation problem.

## Other relevant systems

| System | Why it matters | Label |
|---|---|---|
| Isaac ROS Visual SLAM (cuVSLAM, arXiv 2506.04359) | GPU stereo visual(-inertial) odometry; needs about ±100 µs camera sync and 200–300 Hz IMU; 113 fps at 720p on Orin Nano 8 GB in an older table | [S] |
| Point-LIO ROS 2 port with Unitree L1/L2 support | Ready-made odometry for the exact sensor | [S] |
| FAST-LIO2 ROS 2 ports, DLIO, KISS-ICP, GLIM | Candidates for the odometry bench-off. A public Mid-360 benchmark compares KISS-ICP, DLIO, FAST-LIO2 and GLIM on an Orin NX (timing not read) | [S] |
| Photo-SLAM, MemGS, RTGS, Pocket-SLAM | Visual and memory-aware Gaussian SLAM; real-time claims on larger Jetsons or simulated edge accelerators; none confirmed on an Orin Nano with LiDAR | [S] |
| Sparse Gaussian-process traversability (arXiv 2503.04134) | Geometry-only traversability from point clouds (curvature, gradient, elevation), no splatting | [S] |
| Motion-aware traversability (arXiv 2602.00877) | Traversability depends on how the robot moves, not only on terrain | [S] |

## What was not researched

- 2DGS implementation details beyond the abstract and loss description (training speed, memory).
- Whether any Gaussian SLAM code runs under JetPack 6.2's CUDA 12.6 without porting.
- Elevation-mapping packages (for example GPU elevation mapping) were not searched; they are mentioned in [architecture.md](architecture.md#layer-2--planner-map-baseline) from background knowledge and must be verified before use **[I]**.
- Competition rules (which autonomy tasks need which capability).
