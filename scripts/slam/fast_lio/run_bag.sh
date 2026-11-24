#!/usr/bin/env bash
# Run FAST-LIO2 on a SIMULATION bag and score it against the simulator ground truth.
#
#   scripts/slam/fast_lio/run_bag.sh BAG [OUT_DIR]
#
# BAG must be uncompressed (see eval_trajectory.py) and contain /clock, /lidar/points, /lidar/imu and
# /sim/ground_truth_poses. Needs scripts/slam/fast_lio/setup.sh to have been run. Plays the bag at
# RATE x its recording speed (env, default 1). A simulation bag recorded slower than real time can be
# replayed faster, since everything runs on the bag's /clock, as long as FAST-LIO keeps up (check that
# "poses written" is close to the number of scans). Writes OUT_DIR/fastlio_tum.txt, the node logs and a
# CPU/RSS sample every 2 s (usage.log).
set -u
BAG="${1:?usage: run_bag.sh BAG [OUT_DIR]}"
OUT="${2:-/tmp/fastlio_run}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SLAM="$(cd "$HERE/.." && pwd)"
set +u; source /opt/ros/humble/setup.bash; source "${SLAM_WS:-$HOME/slam_ws}/install/setup.bash"; set -u
mkdir -p "$OUT"

python3 "$SLAM/lidar_point_time.py" --ros-args -p use_sim_time:=true > "$OUT/pointtime.log" 2>&1 &
PT=$!
python3 "$SLAM/odom_to_tum.py" "$OUT/fastlio_tum.txt" --ros-args -p use_sim_time:=true > "$OUT/tum.log" 2>&1 &
TU=$!
ros2 run fast_lio fastlio_mapping --ros-args --params-file "$HERE/sim_l2.yaml" -p use_sim_time:=true > "$OUT/fastlio.log" 2>&1 &
FL=$!
FLPID=""
cleanup_proc() {
    kill -INT ${FLPID:-} "$FL" "$TU" "$PT" 2>/dev/null || true
    for _ in $(seq 10); do
        alive=0
        for p in ${FLPID:-} "$FL" "$TU" "$PT"; do
            kill -0 "$p" 2>/dev/null && alive=1
        done
        [ "$alive" -eq 0 ] && break
        sleep 0.5
    done
    kill -TERM ${FLPID:-} "$FL" "$TU" "$PT" 2>/dev/null || true
}
trap cleanup_proc EXIT
sleep 4
FLPID=$(pgrep -f "lib/fast_lio/fastlio_mapping" | head -1 || true)
( while [ -n "$FLPID" ] && kill -0 "$FLPID" 2>/dev/null; do ps -o %cpu=,rss= -p "$FLPID"; sleep 2; done ) > "$OUT/usage.log" &

ros2 bag play "$BAG" --rate "${RATE:-1.0}" --topics /clock /lidar/points /lidar/imu > "$OUT/play.log" 2>&1
sleep 3
cleanup_proc
trap - EXIT

if [ ! -s "$OUT/fastlio_tum.txt" ]; then
    echo "ERROR: $OUT/fastlio_tum.txt is empty or missing; check $OUT/fastlio.log and $OUT/tum.log" >&2
    exit 1
fi
echo "poses written: $(wc -l < "$OUT/fastlio_tum.txt")"
grep -ciE "warn|error" "$OUT/fastlio.log" | xargs echo "warning/error lines in fastlio.log:"
awk '{c+=$1; if($2>r) r=$2; n++} END {if(n) printf "fastlio_mapping: CPU mean %.0f%% of one core, peak RSS %.0f MB\n", c/n, r/1024}' "$OUT/usage.log"
python3 "$SLAM/eval_trajectory.py" --est "$OUT/fastlio_tum.txt" --bag "$BAG" --est-clock sim
