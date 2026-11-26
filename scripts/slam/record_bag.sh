#!/usr/bin/env bash
# Record a SLAM dataset bag from the rover.
#
# Usage:   scripts/slam/record_bag.sh [run_name]
# Env:     BAG_DIR        output directory            (default: ~/bags)
#          LIDAR_TOPICS   space-separated lidar topics (default: /unilidar/cloud /unilidar/imu)
#          CAMERA_TOPICS  space-separated image topics (default: none; set e.g. "/image_raw /camera_info")
#          EXTRA_TOPICS   anything else to record
#          MIN_FREE_GB    refuse to start below this much free disk (default 20)
#          BAG_SPLIT_BYTES  split the bag at this size (default 4000000000)
#
# Records the lidar, IMU, GNSS, wheel commands, TF and the existing rover estimates so
# that every odometry/mapping candidate can be replayed offline on identical data
# (see docs/slam/research-plan.md, experiments E1-E3).
#
# Point clouds are large (plan for gigabytes per hour); the bag is zstd-compressed.
# Not yet tested on the rover hardware.
set -euo pipefail

NAME="${1:-run_$(date +%Y%m%d_%H%M%S)}"
NAME="${NAME%/}"   # "run1/" would otherwise put the metadata inside the bag folder as run1/.meta
case "$NAME" in
    ''|*/*) echo "run name must be a plain folder name (no '/'), got: ${1:-}" >&2; exit 2 ;;
esac
BAG_DIR="${BAG_DIR:-$HOME/bags}"
BAG_DIR="${BAG_DIR%/}"; BAG_DIR="${BAG_DIR:-/}"
LIDAR_TOPICS="${LIDAR_TOPICS:-/unilidar/cloud /unilidar/imu}"
CAMERA_TOPICS="${CAMERA_TOPICS:-}"
EXTRA_TOPICS="${EXTRA_TOPICS:-}"

# Rover topics that already exist in this repository (see docs/architecture.md).
ROVER_TOPICS="/imu/data_raw /imu/euler /gps/fix /gps/nmea_raw \
/motor/front_left /motor/front_right /motor/rear_left /motor/rear_right /wheel_states \
/cmd_vel /ekf/odometry /ekf/filtered_euler /joint_states /tf /tf_static"

if ! command -v ros2 >/dev/null 2>&1; then
    echo "ros2 not found: source /opt/ros/humble/setup.bash and ros2_ws/install/setup.bash first" >&2
    exit 1
fi

mkdir -p "$BAG_DIR"

# Failure point: point clouds fill a disk quickly and a full disk silently truncates the bag.
# Refuse to start with less than MIN_FREE_GB free (default 20).
MIN_FREE_GB="${MIN_FREE_GB:-20}"
FREE_GB=$(df -Pk "$BAG_DIR" | awk 'NR==2 {print int($4/1048576)}')
if [ "${FREE_GB:-0}" -lt "$MIN_FREE_GB" ]; then
    echo "only ${FREE_GB:-0} GB free in $BAG_DIR (need MIN_FREE_GB=$MIN_FREE_GB); free space or set BAG_DIR/MIN_FREE_GB" >&2
    exit 1
fi

# Failure point: `ros2 bag record` happily records a topic that nobody publishes, so a driver that
# failed to start gives an empty bag that is only noticed hours later. Warn about missing topics.
LIVE="$(timeout 10 ros2 topic list 2>/dev/null || true)"
for t in $LIDAR_TOPICS $CAMERA_TOPICS; do
    echo "$LIVE" | grep -qx "$t" || echo "WARNING: $t is not being published right now" >&2
done

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
# Split files (default 4 GB) so a crash or power loss only damages the last piece.
exec ros2 bag record -o "$OUT" --max-bag-size "${BAG_SPLIT_BYTES:-4000000000}" \
    --compression-mode file --compression-format zstd \
    $LIDAR_TOPICS $CAMERA_TOPICS $ROVER_TOPICS $EXTRA_TOPICS
