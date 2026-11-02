#!/bin/bash
# script to launch modern gazebo simulation and web dashboard

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# Ensure FastDDS uses standard multicast/localhost discovery so all 8+ sim nodes discover each other
unset FASTRTPS_DEFAULT_PROFILES_FILE

echo "cleaning up old simulation and dashboard processes..."
pkill -f "python3.*dashboard_helper" 2>/dev/null
pkill -f "ros2.*sim.launch.py" 2>/dev/null
pkill -f "ign gazebo" 2>/dev/null
pkill -f "gz sim" 2>/dev/null
pkill -f "ruby.*gz" 2>/dev/null
pkill -f "parameter_bridge" 2>/dev/null
pkill -f "robot_state_publisher" 2>/dev/null
pkill -f "rviz2" 2>/dev/null
pkill -f "rosbridge_websocket" 2>/dev/null
pkill -f "python.*http.server" 2>/dev/null
pkill -f "sim_drive_bridge" 2>/dev/null
pkill -f "drive_node" 2>/dev/null
pkill -f "motion_coordinator" 2>/dev/null
pkill -f "vesc_can_driver_node" 2>/dev/null
sleep 1

echo "starting d.a.v.e. modern gazebo and dashboard system..."

# source ros 2 humble
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "warning: /opt/ros/humble/setup.bash not found"
fi

# source workspace
if [ -f "$SCRIPT_DIR/ros2_ws/install/setup.bash" ]; then
    source "$SCRIPT_DIR/ros2_ws/install/setup.bash"
else
    echo "error: workspace not built, run 'colcon build' inside $SCRIPT_DIR/ros2_ws first"
    exit 1
fi

# start rosbridge websocket on port 9090
echo "starting rosbridge websocket on port 9090..."
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090 > /tmp/rosbridge_sim.log 2>&1 &
ROSBRIDGE_PID=$!
sleep 1

# start dashboard helper node (process manager backend)
echo "starting dashboard helper node..."
ros2 run arm_controller dashboard_helper > /tmp/dashboard_helper.log 2>&1 &
HELPER_PID=$!

# start dashboard web server on port 8000
echo "starting dashboard web server on port 8000..."
python3 -m http.server 8000 --directory "$SCRIPT_DIR/dashboard" > /tmp/webserver_sim.log 2>&1 &
WEB_PID=$!

# launch modern gazebo, rviz, and drive subsystem
echo "launching modern gazebo (ros_gz), rviz, and drive subsystem..."
ros2 launch rover_description sim.launch.py > /tmp/gazebo_sim.log 2>&1 &
SIM_PID=$!

echo "all systems started"
echo "dashboard: http://localhost:8000"
echo "press ctrl+c to stop"

# clean shutdown trap
cleanup() {
    echo "shutting down..."
    kill $WEB_PID 2>/dev/null
    kill $HELPER_PID 2>/dev/null
    kill $ROSBRIDGE_PID 2>/dev/null
    kill $SIM_PID 2>/dev/null
    pkill -f "ign gazebo" 2>/dev/null
    pkill -f "gz sim" 2>/dev/null
    pkill -f "ruby.*gz" 2>/dev/null
    pkill -f "parameter_bridge" 2>/dev/null
    pkill -f "robot_state_publisher" 2>/dev/null
    pkill -f "rviz2" 2>/dev/null
    pkill -f "sim_drive_bridge" 2>/dev/null
    pkill -f "drive_node" 2>/dev/null
    pkill -f "motion_coordinator" 2>/dev/null
    pkill -f "vesc_can_driver_node" 2>/dev/null
    pkill -f "rosbridge_websocket" 2>/dev/null
    pkill -f "python3.*dashboard_helper" 2>/dev/null
    exit 0
}

trap cleanup SIGINT SIGTERM
wait
