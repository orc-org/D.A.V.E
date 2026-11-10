#!/usr/bin/env bash
# End-to-end smoke test for NAVnode without any GNSS hardware.
#
# A virtual serial port pair (socat) feeds a fixed NMEA GGA sentence to NAVnode, which parses
# it, converts it to ECEF/local coordinates and publishes NAVDisplayInfo. The test then sets
# the home waypoint at the same position through the ResetHome service and checks that the
# local position becomes (almost) zero. It exercises the NMEA parser, the geodetic <-> ECEF <-> local
# maths and the service names (docs/navigation.md, docs/known-issues.md).
#
# Usage (after building ros2_ws, with ROS and the workspace sourced):
#     scripts/tests/navnode_nmea_smoke_test.sh
# Needs: socat (sudo apt install socat), a built navigation_system.
# Environment: TOL_M  allowed local error in metres after setting home (default 1.0)
set -u

TOL_M="${TOL_M:-1.0}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
WORK="$(mktemp -d)"
FAILS=0

pass() { echo "PASS: $1"; }
fail() { echo "FAIL: $1"; FAILS=$((FAILS + 1)); }

command -v socat >/dev/null 2>&1 || { echo "socat not found (sudo apt install socat)"; exit 2; }
command -v ros2 >/dev/null 2>&1 || { echo "ros2 not found: source ROS and ros2_ws/install/setup.bash first"; exit 2; }

# The ResetHome password is a constant in the source; read it rather than copying it here.
PW="$(grep -m1 'password = ' "$REPO/ros2_ws/src/navigation_system/include/navigation_system/Waypoint.hpp" | grep -oE '[0-9]{4,}' | head -1)"
[ -n "$PW" ] || { echo "could not read the home password from Waypoint.hpp"; exit 2; }

cleanup() {
    # SIGINT lets `ros2 run` forward the shutdown to NAVnode; SIGTERM could leave NAVnode orphaned
    # and a stale node would then answer the next run's service calls.
    kill -INT "${NAV_PID:-}" 2>/dev/null
    kill "${FEED_PID:-}" "${SOCAT_PID:-}" 2>/dev/null
    sleep 1
    pkill -f "navigation_system/NAVnode.*$WORK" 2>/dev/null
    rm -rf "$WORK"
}
trap cleanup EXIT

socat pty,raw,echo=0,link="$WORK/ttyA" pty,raw,echo=0,link="$WORK/ttyB" >/dev/null 2>&1 </dev/null &
SOCAT_PID=$!
for _ in $(seq 20); do [ -e "$WORK/ttyA" ] && [ -e "$WORK/ttyB" ] && break; sleep 0.1; done
[ -e "$WORK/ttyA" ] || { echo "socat did not create the virtual serial ports"; exit 2; }

# Run from the temp directory: NAVnode writes its track log to ./src/navigation_system/captainsLogs
# relative to the current directory and would otherwise litter wherever the test is started.
cd "$WORK" || exit 2
ros2 run navigation_system NAVnode --ros-args -p gnss_port:="$WORK/ttyA" >"$WORK/nav.log" 2>&1 </dev/null &
NAV_PID=$!

# Wait (up to 30 s) for the node to come up instead of sleeping a fixed time.
for _ in $(seq 30); do
    timeout 5 ros2 service list 2>/dev/null | grep -q '^/ResetHome$' && break
    sleep 1
done

# 51 deg 27.6 min N, 112 deg 42.0 min W  ==  51.46 N, 112.70 W
# (The checksum field is fake: NAVnode does not validate it, gps_bridge_node.py does.)
( while true; do printf '$GNGGA,123519.00,5127.60000,N,11242.00000,W,1,12,0.9,792.0,M,0.0,M,,*00\r\n' >"$WORK/ttyB"; sleep 0.3; done ) &
FEED_PID=$!
sleep 2

SERVICES="$(timeout 15 ros2 service list 2>/dev/null)"
for s in SelectRouteToFollow SelectRouteToEdit ResetHome AddLocalObstacle RemoveLocalWaypoint; do
    echo "$SERVICES" | grep -q "^/$s$" && pass "service /$s exists" || fail "service /$s missing"
done

INFO="$(timeout 15 ros2 topic echo --once /NAVDisplayInfo 2>/dev/null)"
[ -n "$INFO" ] && pass "NAVDisplayInfo is published" || fail "NAVDisplayInfo not published"

REPLY="$(timeout 15 ros2 service call /ResetHome navigation_interfaces/srv/ResetHome \
    "{longitude: -112.70, latitude: 51.46, password: $PW}" 2>&1)"
echo "$REPLY" | grep -q "success=True" && pass "ResetHome accepted" || fail "ResetHome rejected"
sleep 2

INFO="$(timeout 15 ros2 topic echo --once /NAVDisplayInfo 2>/dev/null)"
LX="$(echo "$INFO" | awk '/^local_x:/ {print $2}')"
LY="$(echo "$INFO" | awk '/^local_y:/ {print $2}')"
if [ -n "$LX" ] && [ -n "$LY" ] && awk -v x="$LX" -v y="$LY" -v t="$TOL_M" 'BEGIN { exit !(x*x + y*y <= t*t) }'; then
    pass "local position after setting home is ($LX, $LY) m, within $TOL_M m of zero"
else
    fail "local position after setting home is (${LX:-?}, ${LY:-?}) m, expected within $TOL_M m of zero"
fi

[ "$FAILS" -eq 0 ] && echo "all checks passed" || echo "$FAILS check(s) failed"
exit "$FAILS"
