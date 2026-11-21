#!/usr/bin/env bash
# End-to-end smoke test for NAVnode without any GNSS hardware.
#
# A virtual serial port pair (socat) feeds a fixed NMEA GGA sentence to NAVnode, which parses
# it, converts it to ECEF/local coordinates and publishes NAVDisplayInfo. It checks that a waypoint
# is refused before any fix, the RMC speed, waypoint targeting and FollowRoute feedback. The test sets
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

# Before any fix, a local waypoint must be refused (it used to set home silently to a meaningless point).
timeout 15 ros2 service call /AddLocalWaypoint navigation_interfaces/srv/AddLocalWaypoint "{x: 1.0, y: 1.0}" >/dev/null 2>&1
sleep 0.5
grep -q "no GNSS fix yet" "$WORK/nav.log" && pass "waypoint before any fix is refused" \
    || fail "waypoint before any fix was not refused (home may have been set silently)"

# Before any fix, corrupted NMEA checksum frames must be rejected (verify_nmea_checksum:=true by default).
printf '$GNRMC,123519.00,A,5127.60000,N,11242.00000,W,010.0,090.0,091126,,,A*FF\r\n$GNGGA,123519.00,5127.60000,N,11242.00000,W,1,12,0.9,792.0,M,0.0,M,,*FF\r\n' >"$WORK/ttyB"
sleep 0.6
INFO_BAD="$(timeout 15 ros2 topic echo --once /NAVDisplayInfo 2>/dev/null)"
LX_BAD="$(echo "$INFO_BAD" | awk '/^local_x:/ {print $2}')"
if [ -n "$LX_BAD" ] && awk -v x="$LX_BAD" 'BEGIN { exit !(x > -1.0 && x < 1.0) }'; then
    pass "bad NMEA checksum frame rejected (local_x remained $LX_BAD m)"
else
    fail "bad NMEA checksum frame was accepted (local_x jumped to ${LX_BAD:-?})"
fi

# 51 deg 27.6 min N, 112 deg 42.0 min W  ==  51.46 N, 112.70 W
# Compute valid XOR checksums and send RMC+GGA together. RMC carries 10 knots = 18.52 km/h.
nmea_sentence() {
    python3 -c 'import sys
body = sys.argv[1]
s = 0
for b in body.encode("ascii"):
    s ^= b
print(f"${body}*{s:02X}\r")' "$1"
}
RMC_FRAME="$(nmea_sentence "GNRMC,123519.00,A,5127.60000,N,11242.00000,W,010.0,090.0,091126,,,A")"
GGA_FRAME="$(nmea_sentence "GNGGA,123519.00,5127.60000,N,11242.00000,W,1,12,0.9,792.0,M,0.0,M,,")"
( while true; do printf '%s\n%s\n' "$RMC_FRAME" "$GGA_FRAME" >"$WORK/ttyB"; sleep 0.3; done ) &
FEED_PID=$!
sleep 2

SERVICES="$(timeout 15 ros2 service list 2>/dev/null)"
for s in SelectRouteToFollow SelectRouteToEdit ResetHome AddLocalObstacle RemoveLocalWaypoint; do
    echo "$SERVICES" | grep -q "^/$s$" && pass "service /$s exists" || fail "service /$s missing"
done

INFO="$(timeout 15 ros2 topic echo --once /NAVDisplayInfo 2>/dev/null)"
[ -n "$INFO" ] && pass "NAVDisplayInfo is published" || fail "NAVDisplayInfo not published"

# Before home is set the local frame is centred on the Earth's centre, so a parsed position is
# millions of metres from zero. Exactly (0, 0) means no position was parsed at all, which the
# "near zero after ResetHome" check below cannot tell apart from success.
LX0="$(echo "$INFO" | awk '/^local_x:/ {print $2}')"
if [ -n "$LX0" ] && awk -v x="$LX0" 'BEGIN { exit !(x > 100000 || x < -100000) }'; then
    pass "a position was parsed from the GNSS feed (local_x before home is $LX0 m)"
else
    fail "no position parsed from the GNSS feed (local_x before home is ${LX0:-?})"
fi

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

SPD="$(echo "$INFO" | awk '/^speed:/ {print $2}')"
if [ -n "$SPD" ] && awk -v v="$SPD" 'BEGIN { exit !(v > 18.0 && v < 19.0) }'; then
    pass "speed from the RMC sentence is $SPD km/h (expected 18.52)"
else
    fail "speed from the RMC sentence is ${SPD:-?} km/h, expected 18.52 (RMC not parsed)"
fi

# A waypoint added after start-up must become the target: 10 m east of home gives a range of about 10 m.
timeout 15 ros2 service call /AddLocalWaypoint navigation_interfaces/srv/AddLocalWaypoint "{x: 10.0, y: 0.0}" >/dev/null 2>&1
sleep 2
INFO="$(timeout 15 ros2 topic echo --once /NAVDisplayInfo 2>/dev/null)"
RNG="$(echo "$INFO" | awk '/^range2waypoint:/ {print $2}')"
if [ -n "$RNG" ] && awk -v v="$RNG" 'BEGIN { exit !(v > 9.0 && v < 11.0) }'; then
    pass "added waypoint is targeted: range2waypoint is $RNG m (expected about 10)"
else
    fail "added waypoint is not targeted: range2waypoint is ${RNG:-?} m, expected about 10"
fi

# FollowRoute is tracked by the node's timer (no thread): it must report progress towards that waypoint.
FB="$(timeout 6 ros2 action send_goal --feedback /FollowRoute navigation_interfaces/action/NavigationAction "{}" 2>&1)"
echo "$FB" | grep -q "Navigating to waypoint (10" && pass "FollowRoute reports progress to the target" \
    || fail "FollowRoute gave no progress feedback: $(echo "$FB" | tail -3 | tr '\n' ' ')"

[ "$FAILS" -eq 0 ] && echo "all checks passed" || echo "$FAILS check(s) failed"
exit "$FAILS"
