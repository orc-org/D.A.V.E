# Sources

Every link used in this folder. "How read" matters: **fetched** means a page was opened and read (some model-summarised in the second half of long pages); **search** means only a search-result snippet or third-party summary was seen. Re-open anything that drives a purchase or a design decision.

Research was done on 2026-11-08. Pages may have changed since.

## Jetson

| Source | How read |
|---|---|
| [NVIDIA Jetson Orin Nano Super Developer Kit (official page, supplied by the user)](https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/nano-super-developer-kit/) | **not fetched** |
| [CNX Software: $249 Jetson Orin Nano Super targets generative AI at the edge](https://cnx-software.com/2024/12/18/249-nvidia-jetson-orin-nano-super-developer-kit-targets-generative-ai-applications-at-the-edge/) | search |
| [DroneBot Workshop: what you need to know about the Orin Nano Super specs](https://wiki.dronebotworkshop.com/news-dronebotworkshop/what-you-need-to-know-about-jetson-orin-nano-super-dev-kit-specs-361670) | search |
| [ThinkRobotics review of the Orin Nano Super](https://thinkrobotics.com/blogs/product-reviews-buying-guides/nvidia-jetson-orin-nano-super-developer-kit-review-is-it-the-best-edge-ai-board-in-2025) (thermal-throttling anecdote) | search |
| [NVIDIA JetPack SDK 6.2](https://developer.nvidia.com/embedded/jetpack-sdk-62) | search |
| [NVIDIA forum: JetPack 6.2 brings Super Mode](https://forums.developer.nvidia.com/t/jetpack-6-2-brings-super-mode-to-nvidia-jetson-orin-nano-and-jetson-orin-nx-modules/320343) | search |
| [JetsonHacks: JetPack 6.2.2 for Jetson Orin](https://jetsonhacks.com/2026/02/06/jetpack-6-2-2-for-jetson-orin/) | search |

## Unitree sensors

| Source | How read |
|---|---|
| [Unitree 4D LiDAR L2 product page](https://www.unitree.com/L2) | fetched |
| [Unitree 4D LiDAR L1 product page](https://www.unitree.com/LiDAR) | fetched |
| [Unitree A1 (quadruped robot) page](https://www.unitree.com/A1) | fetched |
| [`unilidar_sdk2` repository and README](https://github.com/unitreerobotics/unilidar_sdk2) | fetched |
| [Unitree 4D LiDAR L2 User Manual (PDF)](https://oss-global-cdn.unitree.com/static/Unitree%204D%20LiDAR%20L2%20User%20Manual.pdf) | **fetched but not machine-readable**; read it by hand |
| [`markgol/l2lidar_node` (community ROS 2 driver) listing](https://awesome.ecosyste.ms/projects/github.com%2Fmarkgol%2Fl2lidar_node) | search |
| [Point-LIO ROS 2 port (Unitree L1/L2 support)](https://gitee.com/csc105/point_lio_ros2) | search |
| [r/computervision: Unitree 4D LiDAR L2 review (mirror)](https://red.ks32.dev/r/computervision/comments/1jquey6/unitree_4d_lidar_l2_review/mlazoph/?context=3) | search |

## LiDAR-inertial odometry

| Source | How read |
|---|---|
| [SLAM benchmark on Livox Mid-360: KISS-ICP, DLIO, FAST-LIO2, GLIM](https://discourse.openrobotics.org/t/slam-benchmark-on-livox-mid-360-public-dataset-our-own-uav-flight-data-kiss-icp-dlio-fast-lio2-glim/57912) | search |
| [FAST-LIO GPU variant](https://github.com/OmerMersin/FAST_LIO_GPU) | search |
| [FAST-LIO2 ROS 2 port](https://github.com/CANGO2/FASTLIO2) | search |
| [spark-fast-lio (MIT SPARK)](https://github.com/MIT-SPARK/spark-fast-lio) | search |
| [Jetson Orin NX + Fast-LIO2 deployment write-up (Chinese)](https://zeeklog.com/jetson-orin-nx-fast-lio2zi-zhu-wu-ren-ji-wan-zheng-bu-shu-fang-an-3) | search |
| [Go2 × Nav2 outdoor autonomous driving notes (Japanese)](https://scrapbox.io/kenjiked/Go2%C3%97Nav2_%E5%B1%8B%E5%A4%96%E8%87%AA%E5%BE%8B%E8%B5%B0%E8%A1%8C%E3%83%87%E3%83%A2%E3%81%AE%E5%AD%A6%E3%81%B3_2026-07-06) | search |

## Gaussian mapping and SLAM

| Source | How read |
|---|---|
| [GS-LIVO, arXiv 2501.08672](https://arxiv.org/abs/2501.08672) · [HTML](https://arxiv.org/html/2501.08672v1) · [code](https://github.com/HKUST-Aerial-Robotics/GS-LIVO) | abstract and first half fetched verbatim; rest model-summarised |
| [LIV-GaussMap, arXiv 2401.14857](https://arxiv.deeppaper.ai/papers/2401.14857v2) | search |
| [GS-LIVM, arXiv 2410.17084 (similar-papers listing)](https://arxiv.deeppaper.ai/papers/2410.17084v1/similar) | search |
| [Splatblox, arXiv 2511.18525](https://arxiv.org/abs/2511.18525) · [HTML v2](https://arxiv.org/html/2511.18525v2) · [project page](https://splatblox.github.io) | abstract and first half fetched verbatim; rest model-summarised |
| [PhysGS, arXiv 2511.18570](https://arxiv.org/pdf/2511.18570) | search |
| [Semantic-SLAM-on-embedded survey (Jetson AGX Orin), arXiv 2505.12384](https://arxiv.org/html/2505.12384v1) | search |
| [Continual Gaussian splatting on Orin Nano, arXiv 2603.08499](https://arxiv.org/abs/2603.08499) | search |
| [Photo-SLAM, CVPR 2024](https://openaccess.thecvf.com/content/CVPR2024/papers/Huang_Photo-SLAM_Real-time_Simultaneous_Localization_and_Photorealistic_Mapping_for_Monocular_Stereo_CVPR_2024_paper.pdf) | search |
| [MemGS, arXiv 2509.13536](https://arxiv.org/html/2509.13536v1) | search |
| [RTGS review](https://www.themoonlight.io/review/rtgs-real-time-3d-gaussian-splatting-slam-via-multi-level-redundancy-reduction) | search |
| [Sparse Gaussian-process traversability, arXiv 2503.04134](https://arxiv.org/pdf/2503.04134) | search |
| [Motion-aware traversability, arXiv 2602.00877](https://arxiv.org/pdf/2602.00877) | search |

## Topics from the original list

| Topic | Source | How read |
|---|---|---|
| 2D Gaussian splatting | [2DGS, arXiv 2403.17888](https://arxiv.org/abs/2403.17888) · [project](https://surfsplatting.github.io) | search |
| The linked paper `2504.13153` | [Training-Free Hierarchical Scene Understanding for Gaussian Splatting with Superpoint Graphs](https://arxiv.org/abs/2504.13153) | fetched |
| PointGauss | [arXiv 2508.00259](https://arxiv.org/html/2508.00259v1) | search |
| TRIPS | [arXiv 2401.06003](https://huggingface.co/papers/2401.06003) · [project](https://lfranke.github.io/trips) | search |
| MPM + Gaussians | [PhysGaussian, arXiv 2311.12198](https://arxiv.org/pdf/2311.12198v1.pdf) · [code](https://github.com/XPandora/PhysGaussian) · [GASP, arXiv 2409.05819](https://arxiv.org/html/2409.05819v2) | search |
| GS-Playground | [arXiv 2604.25459](https://arxiv.org/abs/2604.25459) · [full text](https://arxiv.org/html/2604.25459v2) · [project](https://gsplayground.github.io) · [code](https://github.com/discoverse-dev/gs_playground) | abstract and first part fetched; rest model-summarised |
| VGGT with LiDAR | [Improved 3DGS integrating VGGT and LiDAR (DOI 10.1063/5.0342109)](https://www.citedrive.com/en/discovery/improved-3d-gaussian-splatting-integrating-vggt-and-lidar-application-in-visual-relocalization-for-uav-inspection-of-large-aircraft/) · [LiDAR-VGGT, arXiv 2511.01186](https://arxiv.org/abs/2511.01186) | search |
| 4D Gaussians / deformation fields | [4D Gaussian Splatting, arXiv 2310.10642](https://ar5iv.labs.arxiv.org/html/2310.10642) · [Swift4D, arXiv 2503.12307](https://arxiv.org/html/2503.12307v1) | search |
| TSDF / nvblox | [Isaac ROS nvblox](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/index.html) · [Nav2 + Isaac Perceptor tutorial](https://docs.nav2.org/jazzy/tutorials/general_tutorials/isaac_perceptor/) | search |
| Visual SLAM on Jetson | [Isaac ROS Visual SLAM](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_visual_slam/index.html) · [cuVSLAM, arXiv 2506.04359](https://arxiv.org/html/2506.04359v1) | search |
| Spatiotemporal occupation grids via neural SDFs; Dynamic rigid-body clustering (scaffolds) | no specific paper found | searched twice |

## Not consulted

The NVIDIA official Jetson page, the Unitree L2 manual contents, the Isaac ROS release notes for Orin Nano memory, any competition rule document, and elevation-mapping package documentation.
