#!/usr/bin/env bash
# Log Jetson load (CPU, GPU, RAM, temperature, power) while an experiment runs.
#
# Usage:   scripts/slam/bench_tegrastats.sh <label> [seconds]
#          e.g. scripts/slam/bench_tegrastats.sh pointlio_bagA 300
# Output:  $BENCH_DIR/<label>_<timestamp>.log   (default BENCH_DIR: ~/bench)
#          $BENCH_DIR/<label>_<timestamp>.meta  (power mode, git commit, kernel)
#
# Start it, then start the algorithm under test (for example `ros2 bag play`).
# Stops after [seconds] if given, otherwise on Ctrl-C.
# docs/slam/research-plan.md (E1, E3) describes how the logs are used.
# Not yet tested on the rover hardware.
set -euo pipefail

LABEL="${1:?usage: bench_tegrastats.sh <label> [seconds]}"
SECONDS_TO_RUN="${2:-0}"
BENCH_DIR="${BENCH_DIR:-$HOME/bench}"
INTERVAL_MS="${INTERVAL_MS:-1000}"

case "$SECONDS_TO_RUN" in
    ''|*[!0-9]*) echo "seconds must be a non-negative integer, got: $SECONDS_TO_RUN" >&2; exit 2 ;;
esac

if ! command -v tegrastats >/dev/null 2>&1; then
    echo "tegrastats not found: this script only runs on a Jetson" >&2
    exit 1
fi

mkdir -p "$BENCH_DIR"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG="$BENCH_DIR/${LABEL}_${STAMP}.log"
META="$BENCH_DIR/${LABEL}_${STAMP}.meta"

{
    echo "label:    $LABEL"
    echo "date:     $(date -Is)"
    echo "git:      $(git -C "$(dirname "$0")/../.." rev-parse --short HEAD 2>/dev/null || echo unknown)"
    echo "kernel:   $(uname -r)"
    echo "nvpmodel: $(nvpmodel -q 2>/dev/null | tr '\n' ' ' || echo n/a)"
} > "$META"

# tegrastats --logfile writes in the background; stop it when we exit.
tegrastats --interval "$INTERVAL_MS" --logfile "$LOG" &
TS_PID=$!
cleanup() {
    kill "$TS_PID" 2>/dev/null || true
    echo "log: $LOG"
    [ -s "$LOG" ] || echo "WARNING: log is empty (tegrastats may need sudo on this JetPack)" >&2
}
# Clean up once, on EXIT only; INT/TERM just exit (which fires EXIT) instead of running the cleanup twice.
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

# Failure point: tegrastats exits at once when it lacks permission, which would leave an
# empty log while the experiment runs unmonitored.
sleep 2
kill -0 "$TS_PID" 2>/dev/null || { echo "tegrastats exited immediately; try running this script with sudo" >&2; exit 1; }

echo "logging every ${INTERVAL_MS} ms to $LOG"
if [ "$SECONDS_TO_RUN" -gt 0 ]; then
    sleep "$SECONDS_TO_RUN"
else
    wait "$TS_PID"
fi
