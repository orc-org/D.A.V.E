#!/bin/bash
# Script to launch all required backend systems for the D.A.V.E. Dashboard

# Resolve the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Re-enabled FastDDS profiles configuration (Fixed IPC bug by disabling SHM in XML)
export FASTRTPS_DEFAULT_PROFILES_FILE="$SCRIPT_DIR/fastdds_profiles.xml"

echo "Cleaning up any old background processes..."
pkill -f "python3.*dashboard_helper" 2>/dev/null
pkill -f rosbridge_websocket 2>/dev/null
pkill -f rosapi 2>/dev/null
pkill -f "python.*http.server" 2>/dev/null
sleep 1

echo "=== Starting D.A.V.E. Dashboard Backend System ==="
echo "Project Directory: $SCRIPT_DIR"

# Source ROS 2 environments
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "WARNING: /opt/ros/humble/setup.bash not found. Make sure ROS 2 is sourced."
fi

if [ -f "$SCRIPT_DIR/ros2_ws/install/setup.bash" ]; then
    source "$SCRIPT_DIR/ros2_ws/install/setup.bash"
else
    echo "ERROR: Workspace setup.bash not found at $SCRIPT_DIR/ros2_ws/install/setup.bash"
    echo "Please build the workspace first with 'colcon build' inside $SCRIPT_DIR/ros2_ws."
    exit 1
fi

# Failure point: a stale process (or another program) already holding a port makes the new
# server fail silently into its log while this script still prints "All services launched".
port_busy() { (exec 3<>"/dev/tcp/127.0.0.1/$1") 2>/dev/null; }
for p in 9090 8000; do
    if port_busy "$p"; then
        echo "ERROR: port $p is already in use; stop whatever holds it (ss -ltnp | grep $p) and retry."
        exit 1
    fi
done

# 1. Start Rosbridge Websocket Server on port 9090
echo "Starting Rosbridge Websocket Server..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090 > /tmp/rosbridge.log 2>&1 &
ROSBRIDGE_PID=$!

# Wait briefly for rosbridge to bind
sleep 1

# 2. Start Dashboard Helper Node
echo "Starting Dashboard Helper Node..."
ros2 run arm_controller dashboard_helper > /tmp/dashboard_helper.log 2>&1 &
HELPER_PID=$!

# 3. Start Python Web Server for Dashboard UI on port 8000
echo "Starting Dashboard Web Server on http://localhost:8000..."
python3 -m http.server 8000 --directory "$SCRIPT_DIR/dashboard" > /tmp/webserver.log 2>&1 &
WEB_PID=$!

# Failure point: report children that died during start-up instead of claiming success.
sleep 3
STARTUP_OK=1
for pair in "Rosbridge:$ROSBRIDGE_PID:/tmp/rosbridge.log" "Dashboard Helper:$HELPER_PID:/tmp/dashboard_helper.log" "Web Server:$WEB_PID:/tmp/webserver.log"; do
    IFS=: read -r pname ppid plog <<< "$pair"
    if ! kill -0 "$ppid" 2>/dev/null; then
        echo "ERROR: $pname exited during start-up; last log lines ($plog):"
        tail -n 5 "$plog" 2>/dev/null
        STARTUP_OK=0
    fi
done
if [ "$STARTUP_OK" -eq 0 ]; then
    kill $ROSBRIDGE_PID $HELPER_PID $WEB_PID 2>/dev/null
    exit 1
fi

echo "All services launched in the background!"
echo "- Rosbridge PID: $ROSBRIDGE_PID (Logs: /tmp/rosbridge.log)"
echo "- Dashboard Helper PID: $HELPER_PID (Logs: /tmp/dashboard_helper.log)"
echo "- Web Server PID: $WEB_PID (Logs: /tmp/webserver.log)"
echo ""
echo "Press Ctrl+C to stop all services..."

# Keep script running to allow clean shutdown on exit
cleanup() {
    echo ""
    echo "=== Shutting down services ==="
    echo "Stopping Web Server..."
    kill $WEB_PID 2>/dev/null
    echo "Stopping Dashboard Helper..."
    kill $HELPER_PID 2>/dev/null
    echo "Stopping Rosbridge..."
    kill $ROSBRIDGE_PID 2>/dev/null
    # Force clean up any leftover subprocesses
    pkill -f dashboard_helper 2>/dev/null
    pkill -f rosbridge_websocket 2>/dev/null
    pkill -f rosapi 2>/dev/null
    echo "Clean exit."
    exit 0
}

trap cleanup SIGINT SIGTERM

# Wait for background processes to keep shell open
wait
