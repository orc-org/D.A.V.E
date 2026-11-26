#!/usr/bin/env python3
"""Adversarial multi-node integration tests exercising cross-package seams under ROS 2 Humble."""

import math
import os
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter

from arm_interfaces.action import GripperCommand
from geometry_msgs.msg import Point, Twist, Vector3
from nav_msgs.msg import Odometry
from navigation_interfaces.msg import DisplayInfo
from navigation_interfaces.srv import AddLocalWaypoint, RemoveLocalWaypoint, ResetHome, VoidService
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, Joy, NavSatFix, NavSatStatus
from std_msgs.msg import Bool, Float32, String


def _spawn(cmd, log_path):
    f = open(log_path, 'w')
    # Run inside the temporary log folder: NAVnode writes its track log relative to the current
    # directory (src/navigation_system/captainsLogs), which would otherwise land in the caller's checkout.
    p = subprocess.Popen(cmd, stdout=f, stderr=f, preexec_fn=os.setsid, cwd=os.path.dirname(log_path))
    return p, f


def _stop(procs):
    for p, f in procs:
        if p.poll() is None:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGINT)
            except Exception:
                pass
    deadline = time.monotonic() + 3.0
    while time.monotonic() < deadline:
        if all(p.poll() is not None for p, _ in procs):
            break
        time.sleep(0.1)
    for p, f in procs:
        if p.poll() is None:
            try:
                os.killpg(os.getpgid(p.pid), signal.SIGKILL)
            except Exception:
                pass
        try:
            f.close()
        except Exception:
            pass


def _spin_until(node, pred, timeout_s=5.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if pred():
            return True
    return False


def test_drive_arbitration_and_lockout(log_dir):
    print('=== Scenario 1: Drive Arbitration, Lockout & Sensitivity Seam ===')
    procs = [
        _spawn(['ros2', 'run', 'drive_package', 'drive_node'], f'{log_dir}/drive_node.log'),
        _spawn(
            ['ros2', 'run', 'rover_description', 'sim_drive_bridge.py',
             '--ros-args', '-p', 'subscribe_cmd_vel:=false'],
            f'{log_dir}/sim_drive_bridge.log',
        ),
        _spawn(['ros2', 'run', 'drive_package', 'vesc_can_driver_node'], f'{log_dir}/vesc.log'),
    ]
    node = rclpy.create_node('test_drive_seam')
    try:
        pub_joy = node.create_publisher(Joy, '/joy', 10)
        pub_cmd = node.create_publisher(Twist, '/cmd_vel', 10)
        pub_mode = node.create_publisher(String, '/control_mode', 10)
        pub_sens = node.create_publisher(String, '/sensitivity', 10)
        samples = []
        node.create_subscription(Twist, '/cmd_vel_sim', lambda m: samples.append((m.linear.x, m.angular.z)), 20)
        motor_fl = []
        node.create_subscription(Float32, '/motor/front_left', lambda m: motor_fl.append(m.data), 20)

        assert _spin_until(
            node,
            lambda: pub_cmd.get_subscription_count() >= 1 and node.count_publishers('/cmd_vel_sim') >= 1,
            8.0,
        ), 'drive_node or sim_drive_bridge failed to register endpoints'
        time.sleep(0.3)

        # A: /control_mode = arm must block /cmd_vel on BOTH /motor/* and /cmd_vel_sim
        pub_mode.publish(String(data='arm'))
        _spin_until(node, lambda: False, 0.2)
        samples.clear()
        motor_fl.clear()
        for _ in range(8):
            pub_cmd.publish(Twist(linear=Vector3(x=0.8), angular=Vector3(z=0.4)))
            _spin_until(node, lambda: False, 0.04)
        assert not motor_fl or all(abs(v) < 1e-4 for v in motor_fl), f'/motor/front_left moved under arm mode: {motor_fl}'
        assert all(abs(lx) < 1e-4 and abs(az) < 1e-4 for lx, az in samples), f'Bypassed lockout: {samples}'
        print('PASS: /control_mode=arm blocks /cmd_vel end-to-end including /cmd_vel_sim')

        # B: /sensitivity=ultra scales /cmd_vel to 0.25x in Gazebo bridge
        pub_mode.publish(String(data='drive'))
        pub_sens.publish(String(data='ultra'))
        _spin_until(node, lambda: False, 0.2)
        samples.clear()
        for _ in range(12):
            pub_cmd.publish(Twist(linear=Vector3(x=0.8)))
            _spin_until(node, lambda: False, 0.04)
        assert samples and abs(samples[-1][0] - 0.20) < 0.03, f'Sensitivity failed: {samples}'
        print(f'PASS: /sensitivity=ultra scaled 0.8 m/s -> {samples[-1][0]:.3f} m/s')

        # C: Active /joy wins over simultaneous 0.0 /cmd_vel dashboard idle spam without chatter
        pub_sens.publish(String(data='coarse'))
        _spin_until(node, lambda: False, 0.1)
        samples.clear()
        joy_fwd = Joy(axes=[0.0, 0.0, 0.0, 0.0, 1.0, -1.0])
        for _ in range(15):
            pub_joy.publish(joy_fwd)
            pub_cmd.publish(Twist())
            _spin_until(node, lambda: False, 0.04)
        nonzero = [lx for lx, _ in samples if lx > 0.2]
        zero_drops = [lx for lx, _ in samples if abs(lx) < 1e-3]
        assert len(nonzero) >= 5 and len(zero_drops) == 0, f'Joy vs idle cmd_vel chattered: {samples}'
        print(f'PASS: Active /joy held {len(nonzero)} samples without zero chatter')

        # D: <6-axis controller fault does not crash drive_node and stops wheels
        samples.clear()
        for _ in range(6):
            pub_joy.publish(Joy(axes=[0.5, 0.5]))
            _spin_until(node, lambda: False, 0.05)
        assert procs[0][0].poll() is None, 'drive_node crashed on short /joy'
        assert samples and abs(samples[-1][0]) < 1e-3 and abs(samples[-1][1]) < 1e-3, f'No stop on short joy: {samples}'
        print('PASS: <6-axis /joy survived safely and zeroed /cmd_vel_sim')

        # E: Reverse left-turn keeps CCW yaw (+angular.z)
        samples.clear()
        joy_rev_ccw = Joy(axes=[0.5, 0.0, 0.0, 0.0, -1.0, 1.0])
        for _ in range(12):
            pub_joy.publish(joy_rev_ccw)
            _spin_until(node, lambda: False, 0.04)
        assert samples and samples[-1][0] < -0.5 and samples[-1][1] > 0.5, f'Reverse yaw bad: {samples[-1]}'
        print(f'PASS: Reverse CCW joy yields ({samples[-1][0]:.2f} m/s, {samples[-1][1]:.2f} rad/s)')
    finally:
        node.destroy_node()
        _stop(procs)


def test_ekf_and_nav_seam(log_dir):
    print('=== Scenario 2: Slow-Clock EKF + NAVnode /gps/fix + ResetHome Rebase ===')
    procs = [
        _spawn(
            ['ros2', 'run', 'rover_ekf', 'ekf_node', '--ros-args',
             '-p', 'use_sim_time:=true', '-p', 'wheel_radius:=0.15', '-p', 'imu_yaw_offset_deg:=0.0'],
            f'{log_dir}/ekf.log',
        ),
        _spawn(
            ['ros2', 'run', 'navigation_system', 'NAVnode', '--ros-args',
             '-p', 'use_gps_fix_topic:=true'],
            f'{log_dir}/nav.log',
        ),
    ]
    node = rclpy.create_node('test_ekf_nav_seam')
    try:
        pub_clk = node.create_publisher(Clock, '/clock', 10)
        pub_ws = node.create_publisher(JointState, '/wheel_states', 10)
        pub_imu = node.create_publisher(Vector3, '/imu/euler', 10)
        pub_gps = node.create_publisher(NavSatFix, '/gps/fix', 10)

        cli_add = node.create_client(AddLocalWaypoint, '/AddLocalWaypoint')
        cli_rem = node.create_client(RemoveLocalWaypoint, '/RemoveLocal')
        cli_tog_edit = node.create_client(VoidService, '/ToggleRouteToEdit')
        cli_home = node.create_client(ResetHome, '/ResetHome')
        cli_next = node.create_client(VoidService, '/TargetNextWaypoint')

        odom_msgs = []
        nav_msgs = []
        node.create_subscription(Odometry, '/ekf/odometry', lambda m: odom_msgs.append(m), 10)
        node.create_subscription(DisplayInfo, '/NAVDisplayInfo', lambda m: nav_msgs.append(m), 10)

        assert _spin_until(node, lambda: cli_home.service_is_ready() and pub_ws.get_subscription_count() >= 1, 8.0)

        def step_clock(t_s):
            sec = int(t_s)
            nsec = int((t_s - sec) * 1e9)
            c = Clock()
            c.clock.sec = sec
            c.clock.nanosec = nsec
            pub_clk.publish(c)
            return sec, nsec

        # A: Feed one-sided left wheels + STATUS_NO_FIX; EKF must ignore unbalanced wheels and NAV must refuse add
        for k in range(15):
            sec, nsec = step_clock(100.0 + k * 0.05)
            js = JointState()
            js.header.stamp.sec, js.header.stamp.nanosec = sec, nsec
            js.name = ['front_left_wheel_joint', 'rear_left_wheel_joint']
            js.velocity = [5.0, 5.0]
            pub_ws.publish(js)

            nf = NavSatFix()
            nf.status.status = NavSatStatus.STATUS_NO_FIX
            pub_gps.publish(nf)
            _spin_until(node, lambda: False, 0.05)

        fut_add0 = cli_add.call_async(AddLocalWaypoint.Request(x=0.0, y=30.0))
        _spin_until(node, lambda: fut_add0.done(), 2.0)
        assert odom_msgs and abs(odom_msgs[-1].twist.twist.linear.x) < 0.05, \
            f'One-sided wheels corrupted EKF speed: {odom_msgs[-1].twist.twist.linear.x}'
        with open(f'{log_dir}/nav.log') as lf:
            assert 'no GNSS fix yet' in lf.read(), 'NAVnode did not refuse waypoint under STATUS_NO_FIX'
        print('PASS: One-sided /wheel_states rejected by EKF & STATUS_NO_FIX kept waypoint add locked')

        # B: Feed balanced 4 wheels at 4.0 rad/s (0.60 m/s) over slowed sim clock + valid /gps/fix
        odom_msgs.clear()
        for k in range(30):
            sec, nsec = step_clock(101.0 + k * 0.05)
            js = JointState()
            js.header.stamp.sec, js.header.stamp.nanosec = sec, nsec
            js.name = ['front_left_wheel_joint', 'rear_left_wheel_joint',
                       'front_right_wheel_joint', 'rear_right_wheel_joint']
            js.velocity = [4.0, 4.0, 4.0, 4.0]
            pub_ws.publish(js)
            pub_imu.publish(Vector3(x=0.0, y=0.0, z=0.0))

            gf = NavSatFix()
            gf.header.stamp.sec, gf.header.stamp.nanosec = sec, nsec
            gf.status.status = NavSatStatus.STATUS_FIX
            gf.latitude = 51.46
            gf.longitude = -112.70
            gf.altitude = 792.0
            pub_gps.publish(gf)
            _spin_until(node, lambda: False, 0.05)

        vx = odom_msgs[-1].twist.twist.linear.x
        assert abs(vx - 0.60) < 0.05 and odom_msgs[-1].twist.covariance[0] > 0.0, f'EKF steady vx wrong: {vx}'
        print(f'PASS: EKF converged under sim /clock to {vx:.3f} m/s (target 0.600) with populated twist cov')

        # B2: ensureHome() auto-origin immediately rebases currentPosition so range2waypoint ~ 30 m before explicit ResetHome
        fut_eh = cli_add.call_async(AddLocalWaypoint.Request(x=0.0, y=30.0))
        assert _spin_until(node, lambda: fut_eh.done(), 3.0)
        nav_msgs.clear()
        assert _spin_until(node, lambda: len(nav_msgs) >= 3, 2.0)
        assert abs(nav_msgs[-1].range2waypoint - 30.0) < 1.0, \
            f'ensureHome() left stale ECEF/unrebased currentPosition: range={nav_msgs[-1].range2waypoint}'
        fut_rm_eh = cli_rem.call_async(RemoveLocalWaypoint.Request(x=0.0, y=30.0))
        assert _spin_until(node, lambda: fut_rm_eh.done(), 3.0)
        print('PASS: ensureHome() immediately rebased local pose without waiting for another /gps/fix')

        # C: ResetHome, AddLocalWaypoint (0, 30), ResetHome 20m North -> verify rebased waypoint_local_y ~ 10 m
        fut_h1 = cli_home.call_async(ResetHome.Request(latitude=51.46, longitude=-112.70, password=741021))
        assert _spin_until(node, lambda: fut_h1.done(), 3.0) and fut_h1.result().success

        fut_w1 = cli_add.call_async(AddLocalWaypoint.Request(x=0.0, y=30.0))
        assert _spin_until(node, lambda: fut_w1.done(), 3.0)
        _spin_until(node, lambda: False, 0.6)
        assert nav_msgs and abs(nav_msgs[-1].range2waypoint - 30.0) < 1.0, f'Initial range unexpected: {nav_msgs[-1]}'

        # Move home ~20 m north (~0.0001798 deg lat); rover stays at 51.46 -> y_local=-20, wp becomes y_local=+10
        fut_h2 = cli_home.call_async(ResetHome.Request(latitude=51.4601798, longitude=-112.70, password=741021))
        assert _spin_until(node, lambda: fut_h2.done(), 3.0) and fut_h2.result().success
        _spin_until(node, lambda: False, 0.6)
        assert abs(nav_msgs[-1].waypoint_local_y - 10.0) < 1.0 and abs(nav_msgs[-1].range2waypoint - 30.0) < 1.0, \
            f'ResetHome failed to rebase waypoint cleanly: y={nav_msgs[-1].waypoint_local_y}, r={nav_msgs[-1].range2waypoint}'
        print(f'PASS: ResetHome rebase moved waypoint_local_y to {nav_msgs[-1].waypoint_local_y:.2f} m (range preserved {nav_msgs[-1].range2waypoint:.2f} m)')

        # C2: Stationary /gps/fix after ResetHome must not compute old-home vs new-home speed spike
        nav_msgs.clear()
        gf_post = NavSatFix()
        gf_post.header.stamp.sec = 104
        gf_post.header.stamp.nanosec = 0
        gf_post.status.status = NavSatStatus.STATUS_FIX
        gf_post.latitude = 51.46
        gf_post.longitude = -112.70
        gf_post.altitude = 792.0
        pub_gps.publish(gf_post)
        assert _spin_until(node, lambda: len(nav_msgs) >= 3, 2.0)
        assert nav_msgs[-1].speed < 1.0, \
            f'Post-ResetHome /gps/fix caused speed spike: {nav_msgs[-1].speed}'
        print(f'PASS: Post-ResetHome stationary /gps/fix kept speed={nav_msgs[-1].speed:.2f} km/h (no frame-shift spike)')

        # D: Step past end of route via TargetNextWaypoint and confirm validateTarget doesn't wrap back
        fut_nx = cli_next.call_async(VoidService.Request())
        assert _spin_until(node, lambda: fut_nx.done(), 3.0)
        nav_msgs.clear()
        _spin_until(node, lambda: len(nav_msgs) >= 4, 3.0)
        assert nav_msgs and nav_msgs[-1].range2waypoint > 1e11, \
            f'TargetNextWaypoint snapped back to route head: {nav_msgs[-1].range2waypoint}'
        print('PASS: TargetNextWaypoint past route end remained finished without auto-restarting')

        # E: Toggle routeToEdit to Route2 while routeToFollow stays on Route1; add + remove on Route2
        assert _spin_until(node, lambda: cli_tog_edit.service_is_ready() and cli_rem.service_is_ready(), 3.0)
        fut_te = cli_tog_edit.call_async(VoidService.Request())
        assert _spin_until(node, lambda: fut_te.done(), 3.0)
        fut_w2 = cli_add.call_async(AddLocalWaypoint.Request(x=12.0, y=34.0))
        assert _spin_until(node, lambda: fut_w2.done(), 3.0)
        fut_rm = cli_rem.call_async(RemoveLocalWaypoint.Request(x=12.0, y=34.0))
        assert _spin_until(node, lambda: fut_rm.done(), 3.0)
        assert fut_rm.result().output == 'Waypoint removed successfully', \
            f'RemoveLocal failed to target routeToEdit: {fut_rm.result().output}'
        print('PASS: RemoveLocal removes edited waypoint on routeToEdit even when routeToFollow differs')
    finally:
        node.destroy_node()
        _stop(procs)


def test_arm_coordinator_and_stepper_seam(log_dir):
    print('=== Scenario 3: Arm Stepper Action + Coordinator Finger Limits & Base Lock ===')
    procs = [
        _spawn(
            ['ros2', 'run', 'arm_controller', 'arm_stepper_driver',
             '--ros-args', '-p', 'use_hardware_gpio:=false'],
            f'{log_dir}/arm_stepper.log',
        ),
        _spawn(
            ['ros2', 'run', 'arm_controller', 'motion_coordinator',
             '--ros-args', '-p', 'show_plot:=false', '-p', 'allow_base_motion:=false'],
            f'{log_dir}/coordinator.log',
        ),
    ]
    node = rclpy.create_node('test_arm_seam')
    try:
        ac = ActionClient(node, GripperCommand, 'gripper_command')
        pub_auto = node.create_publisher(Bool, '/auto_mode', 10)
        pub_tgt = node.create_publisher(Point, 'arm_target', 10)

        gripper_states = []
        joint_states = []
        cmd_vels = []
        node.create_subscription(Float32, '/gripper_state', lambda m: gripper_states.append(m.data), 10)
        node.create_subscription(JointState, 'joint_states', lambda m: joint_states.append(m), 10)
        node.create_subscription(Twist, 'cmd_vel', lambda m: cmd_vels.append(m), 10)

        assert ac.wait_for_server(timeout_sec=6.0), 'gripper_command action server unavailable'

        # A: GripperCommand action completes concurrently without deadlocking & coordinator stays inside URDF limits
        goal_msg = GripperCommand.Goal()
        goal_msg.position = 0.5
        send_fut = ac.send_goal_async(goal_msg)
        assert _spin_until(node, lambda: send_fut.done(), 3.0)
        gh = send_fut.result()
        assert gh.accepted
        res_fut = gh.get_result_async()
        assert _spin_until(node, lambda: res_fut.done(), 8.0), 'GripperCommand deadlocked or timed out'
        assert res_fut.result().result.success, 'GripperCommand returned failure'
        _spin_until(node, lambda: False, 0.3)

        assert gripper_states and abs(gripper_states[-1] - 20000.0) < 1000.0, f'Unexpected steps: {gripper_states[-1]}'
        idx_l = joint_states[-1].name.index('left_finger_joint')
        finger_val = joint_states[-1].position[idx_l]
        assert -0.021 <= finger_val <= 0.041, f'Finger joint out of URDF [-0.02, 0.04]: {finger_val}'
        print(f'PASS: GripperCommand(0.5) succeeded (steps={gripper_states[-1]:.0f}, URDF finger={finger_val:.4f} m)')

        # A2: Cancel mid-move GripperCommand(0.0) -> target must reset so stepper stops moving immediately
        goal_msg2 = GripperCommand.Goal()
        goal_msg2.position = 0.0
        send_fut2 = ac.send_goal_async(goal_msg2)
        assert _spin_until(node, lambda: send_fut2.done(), 3.0)
        gh2 = send_fut2.result()
        assert gh2.accepted
        assert _spin_until(node, lambda: gripper_states and gripper_states[-1] > 21500.0, 3.0), \
            f'Gripper did not begin moving toward 40000 steps: {gripper_states[-1]}'
        cancel_fut = gh2.cancel_goal_async()
        assert _spin_until(node, lambda: cancel_fut.done(), 3.0)
        res_fut2 = gh2.get_result_async()
        assert _spin_until(node, lambda: res_fut2.done(), 3.0)
        assert not res_fut2.result().result.success, 'Canceled GripperCommand reported success=True'
        stopped_steps = gripper_states[-1]
        _spin_until(node, lambda: False, 0.6)
        assert abs(gripper_states[-1] - stopped_steps) < 500.0 and gripper_states[-1] < 38000.0, \
            f'Gripper kept moving after cancel! stopped={stopped_steps}, later={gripper_states[-1]}'
        print(f'PASS: Canceled GripperCommand halted stepper motion cleanly at {gripper_states[-1]:.0f} steps')

        # B: Auto mode with allow_base_motion=false must not drive base wheels
        cmd_vels.clear()
        pub_auto.publish(Bool(data=True))
        pub_tgt.publish(Point(x=20.0, y=5.0, z=1.5))
        _spin_until(node, lambda: False, 0.6)
        assert all(abs(c.linear.x) < 1e-6 and abs(c.angular.z) < 1e-6 for c in cmd_vels), \
            f'motion_coordinator published nonzero cmd_vel despite allow_base_motion:=false'
        print('PASS: motion_coordinator kept base stationary in auto mode when allow_base_motion=false')
    finally:
        node.destroy_node()
        _stop(procs)


def main():
    rclpy.init()
    with tempfile.TemporaryDirectory() as log_dir:
        try:
            test_drive_arbitration_and_lockout(log_dir)
            test_ekf_and_nav_seam(log_dir)
            test_arm_coordinator_and_stepper_seam(log_dir)
        except AssertionError as e:
            print(f'INTEGRATION FAILURE: {e}')
            for fn in sorted(os.listdir(log_dir)):
                print(f'--- {fn} ---')
                with open(os.path.join(log_dir, fn)) as lf:
                    print(lf.read()[-800:])
            sys.exit(1)
        finally:
            if rclpy.ok():
                rclpy.shutdown()
    print('ALL INTEGRATION SCENARIOS PASSED')


if __name__ == '__main__':
    main()
