#!/usr/bin/env bash
# Inspect (and optionally set) the Jetson power mode and clocks before benchmarking.
#
# Usage:   scripts/slam/jetson_perf_setup.sh                 # show current mode and the available modes
#          scripts/slam/jetson_perf_setup.sh --set <id>      # choose a power mode by id (needs sudo)
#          scripts/slam/jetson_perf_setup.sh --clocks        # lock clocks to maximum (needs sudo)
#
# The mode ids differ between JetPack releases and modules, so this script lists them
# from /etc/nvpmodel.conf instead of hard-coding any. Run it first and record the output
# with every benchmark (docs/slam/research-plan.md, experiment E0).
# Not yet tested on the rover hardware.
set -euo pipefail

if ! command -v nvpmodel >/dev/null 2>&1; then
    echo "nvpmodel not found: this script only runs on a Jetson" >&2
    exit 1
fi

case "${1:-}" in
    --set)
        ID="${2:?usage: --set <mode id>}"
        sudo nvpmodel -m "$ID"
        ;;
    --clocks)
        sudo jetson_clocks
        ;;
    "" )
        ;;
    *)
        echo "unknown option: $1" >&2
        exit 2
        ;;
esac

echo "--- current power mode ---"
nvpmodel -q || true
echo
echo "--- modes defined in /etc/nvpmodel.conf ---"
grep -E "POWER_MODEL" /etc/nvpmodel.conf 2>/dev/null || echo "(file not found)"
echo
echo "--- jetson_clocks state ---"
sudo jetson_clocks --show 2>/dev/null || echo "(jetson_clocks not available)"
