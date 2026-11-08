#!/usr/bin/env bash
# Record a SLAM dataset bag from the rover.
#
# Usage:   scripts/slam/record_bag.sh [run_name]
# Env:     BAG_DIR        output directory            (default: ~/bags)
#          LIDAR_TOPICS   space-separated lidar topics (default: /unilidar/cloud /unilidar/imu)
#          CAMERA_TOPICS  space-separated image topics (default: none; set e.g. "/image_raw /camera_info")
#          EXTRA_TOPICS   anything else to record
#
# Records the lidar, IMU, GNSS, wheel commands, TF and the existing rover estimates so
# that every odometry/mapping candidate can be replayed offline on identical data
# (see docs/slam/research-plan.md, experiments E1-E3).
#
# Point clouds are large (plan for gigabytes per hour); the bag is zstd-compressed.
# Not yet tested on the rover hardware.
set -euo pipefail

NAME="${1:-run_$(date +%Y%m%d_%H%M%S)}"
BAG_DIR="${BAG_DIR:-$HOME/bags}"
LIDAR_TOPICS="${LIDAR_TOPICS:-/unilidar/cloud /unilidar/imu}"
CAMERA_TOPICS="${CAMERA_TOPICS:-}"
EXTRA_TOPICS="${EXTRA_TOPICS:-}"

# Rover topics that already exist in this repository (see docs/architecture.md).
ROVER_TOPICS="/imu/data_raw /imu/euler /gps/fix /gps/nmea_raw \
/motor/front_left /motor/front_right /motor/rear_left /motor/rear_right \
/cmd_vel /ekf/odometry /ekf/filtered_euler /joint_states /tf /tf_static"

if ! command -v ros2 >/dev/null 2>&1; then
    echo "ros2 not found: source /opt/ros/humble/setup.bash and ros2_ws/install/setup.bash first" >&2
    exit 1
fi

mkdir -p "$BAG_DIR"
OUT="$BAG_DIR/$NAME"
if [ -e "$OUT" ]; then
    echo "refusing to overwrite existing bag: $OUT" >&2
    exit 1
fi

# Save run metadata next to the bag so the experiment log is reproducible.
mkdir -p "$OUT.meta"
{
    echo "date:      $(date -Is)"
    echo "git:       $(git -C "$(dirname "$0")/../.." rev-parse --short HEAD 2>/dev/null || echo unknown)"
    echo "host:      $(hostname)"
    echo "nvpmodel:  $(nvpmodel -q 2>/dev/null | tr '\n' ' ' || echo n/a)"
    echo "lidar:     $LIDAR_TOPICS"
    echo "camera:    ${CAMERA_TOPICS:-none}"
} > "$OUT.meta/info.txt"

echo "Recording to $OUT (Ctrl-C to stop)"
# shellcheck disable=SC2086
exec ros2 bag record -o "$OUT" \
    --compression-mode file --compression-format zstd \
    $LIDAR_TOPICS $CAMERA_TOPICS $ROVER_TOPICS $EXTRA_TOPICS
