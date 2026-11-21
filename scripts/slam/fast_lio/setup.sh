#!/usr/bin/env bash
# Build FAST-LIO2 (official hku-mars/FAST_LIO, ROS2 branch) in a separate overlay workspace for evaluation.
#
#   scripts/slam/fast_lio/setup.sh            # workspace ~/slam_ws (override with SLAM_WS=...)
#
# - pinned to an exact commit (FAST_LIO_COMMIT below); its ikd-Tree submodule is pinned by that commit
# - licence GPL-2.0: it runs as its own process; none of its code is copied into this repository
# - audited 2026-11-15 at this commit: no downloads or shell commands at build or run time; it writes
#   files only when pcd_save_en / runtime_pos_log_enable are true (into its own package folder), and it
#   offers one `map_save` service (Trigger) that writes a PCD there
# - livox_ros_driver2 is replaced by a message-only stub (livox_ros_driver2_stub/)
# Built at low priority with one package at a time, so it does not swamp a busy machine.
set -euo pipefail
FAST_LIO_COMMIT=cc3c9e8b1469b33b8072764d6db2348ed6dc597f   # ROS2 branch, 2026-10-25
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${SLAM_WS:-$HOME/slam_ws}"

mkdir -p "$WS/src"
cd "$WS/src"
if [ ! -d FAST_LIO ]; then
    git clone --branch ROS2 https://github.com/hku-mars/FAST_LIO.git
fi
git -C FAST_LIO fetch -q origin ROS2
git -C FAST_LIO checkout -q "$FAST_LIO_COMMIT"
git -C FAST_LIO submodule update --init --recursive
# drop unused pcl_ros CMake dependency (FAST_LIO only uses pcl_conversions + PCL common/io/filters + tf2_ros)
sed -i 's/find_package(pcl_ros REQUIRED)/find_package(tf2_ros REQUIRED)/; s/  pcl_ros/  tf2_ros/; s/find_package(PCL REQUIRED COMPONENTS common io)/find_package(PCL REQUIRED COMPONENTS common io filters kdtree)/' FAST_LIO/CMakeLists.txt
sed -i 's/<depend>pcl_ros<\/depend>/<depend>tf2_ros<\/depend>/' FAST_LIO/package.xml
rm -rf livox_ros_driver2_stub
cp -r "$HERE/livox_ros_driver2_stub" livox_ros_driver2_stub

set +u; source /opt/ros/humble/setup.bash; set -u
cd "$WS"
MAKEFLAGS="-j${JOBS:-2}" nice -n 10 colcon build --parallel-workers 1 \
    --packages-select livox_ros_driver2 fast_lio --cmake-args -DCMAKE_BUILD_TYPE=Release
echo "built; use:  source $WS/install/setup.bash"
