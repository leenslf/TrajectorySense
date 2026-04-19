#!/usr/bin/env python3
"""
experiment_common.py
====================
Shared constants, helpers, and the standard experiment runner used by all
scenario scripts.  Scenario scripts only need to define their name, world
file, motion sequence, and call ``run_experiment()``.
"""

import datetime
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# ─── SHARED CONSTANTS ────────────────────────────────────────────────────────

# Resolved relative to this file so paths work from any CWD.
_COMMON_DIR = os.path.dirname(os.path.abspath(__file__))
_REPO_ROOT   = os.path.abspath(os.path.join(_COMMON_DIR, '..', '..', '..', '..'))

# World files — scenarios reference these or build their own path with _REPO_ROOT.
WORLD_DIR = os.path.join(_REPO_ROOT, 'src', 'moborobo_robot', 'world')

# Topics captured in every bag.
TOPICS_TO_RECORD = [
    '/tf',
    '/tf_static',
    '/clock',
    '/cmd_vel',
    '/imu/data',
    '/lio/odometry',
    '/gps/odom',
    '/gps/fix',
    '/gazebo/model_states',   # ground-truth source
]

# Topics that must appear before bag recording and motion begin.
REQUIRED_TOPICS = [
    '/clock',
    '/gazebo/model_states',
    '/tf',
]

ROBOT_SPAWN_POSE = [0.0, 0.0, 0.0]   # x, y, yaw — must match launch file

BAG_OUTPUT_DIR = os.path.join(_COMMON_DIR, '..', '..', 'bags')

CMD_VEL_TOPIC   = '/cmd_vel'
CMD_VEL_RATE_HZ = 10

# Seconds to let the simulation, robot spawner, ig_lio, and EKF settle before
# any motion is sent.  Increased from 12 s to 30 s to give the system more
# time to initialise and avoid motion starting before localisation converges.
STABILIZATION_WAIT_S = 20.0

# ─── HELPERS ─────────────────────────────────────────────────────────────────

def wait_for_topics(required_topics=REQUIRED_TOPICS, timeout_s=120.0,
                    poll_interval_s=3.0):
    """Block until every topic in *required_topics* appears in ``ros2 topic list``."""
    print(f'[wait_for_topics] Waiting up to {timeout_s:.0f}s for: {required_topics}')
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        try:
            result = subprocess.run(
                ['ros2', 'topic', 'list'],
                capture_output=True, text=True, timeout=10.0,
            )
            available = set(result.stdout.splitlines())
            missing = [t for t in required_topics if t not in available]
            if not missing:
                print('[wait_for_topics] All required topics available.')
                return
            print(f'[wait_for_topics] Still waiting for: {missing}')
        except subprocess.TimeoutExpired:
            print('[wait_for_topics] ros2 topic list timed out, retrying...')
        time.sleep(poll_interval_s)
    raise RuntimeError(
        f'Required topics not available after {timeout_s:.0f}s: {required_topics}'
    )


def make_bag_path(scenario_name):
    """Return a timestamped bag path under BAG_OUTPUT_DIR (directory is created)."""
    ts      = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    bag_dir = os.path.abspath(BAG_OUTPUT_DIR)
    os.makedirs(bag_dir, exist_ok=True)
    return os.path.join(bag_dir, f'{scenario_name}_run_{ts}')


def start_bag_recording(bag_path, topics=TOPICS_TO_RECORD):
    """Launch ``ros2 bag record`` as a subprocess and return the Popen handle."""
    cmd = ['ros2', 'bag', 'record', '-o', bag_path] + topics
    print(f'[bag] Recording to : {bag_path}')
    print(f'[bag] Command      : {" ".join(cmd)}')
    return subprocess.Popen(cmd)


def stop_bag_recording(proc):
    """Send SIGINT to the bag-record process and wait for a clean exit."""
    if proc is None or proc.poll() is not None:
        return
    print('[bag] Stopping bag recording...')
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=10.0)
    except subprocess.TimeoutExpired:
        print('[bag] Force-killing bag record process.')
        proc.kill()
    print('[bag] Bag recording stopped.')


def _shutdown_proc(proc, tag, timeout_s=5.0):
    """
    Signal the entire process group of *proc* with SIGINT, then SIGKILL on
    timeout.  Requires the process to have been started with
    start_new_session=True so its group is isolated from this script.
    """
    if proc is None or proc.poll() is not None:
        return
    print(f'[{tag}] Shutting down...')
    try:
        pgid = os.getpgid(proc.pid)
        os.killpg(pgid, signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        proc.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(pgid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        try:
            proc.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            pass


def kill_stale_gazebo():
    """
    Kill any orphaned gzserver/gzclient processes left over from a previous run.
    If a stale Gazebo is still up when ros2 launch starts, the new robot spawner
    will connect to it and the robot will appear at whatever pose it had last —
    not at origin.
    """
    killed = False
    for name in ('gzserver', 'gzclient'):
        result = subprocess.run(['pgrep', '-x', name],
                                capture_output=True, text=True)
        pids = result.stdout.split()
        if pids:
            print(f'[sim] Stale {name} found (PIDs: {pids}) — killing...')
            subprocess.run(['pkill', '-SIGKILL', '-x', name], capture_output=True)
            killed = True
    if killed:
        time.sleep(2.0)   # give the OS time to reap the processes


def execute_motion(pub, motion_sequence, rate_hz=CMD_VEL_RATE_HZ):
    """Publish each ``(linear_x, angular_z, duration_s)`` step at *rate_hz*."""
    dt          = 1.0 / rate_hz
    total_steps = len(motion_sequence)
    for i, (linear_x, angular_z, duration_s) in enumerate(motion_sequence, start=1):
        msg           = Twist()
        msg.linear.x  = float(linear_x)
        msg.angular.z = float(angular_z)
        print(f'[motion] Step {i}/{total_steps}: '
              f'lin={linear_x:.2f} m/s  ang={angular_z:.2f} rad/s  '
              f'dur={duration_s:.1f}s')
        step_end = time.time() + duration_s
        while time.time() < step_end:
            pub.publish(msg)
            time.sleep(dt)


def stop_robot(pub, duration_s=2.0, rate_hz=CMD_VEL_RATE_HZ):
    """Publish zero velocity for *duration_s* seconds."""
    stop_msg = Twist()
    dt       = 1.0 / rate_hz
    end      = time.time() + duration_s
    while time.time() < end:
        pub.publish(stop_msg)
        time.sleep(dt)


# ─── STANDARD EXPERIMENT RUNNER ──────────────────────────────────────────────

def run_experiment(scenario_name, world_file, motion_sequence,
                   pre_motion_hook=None):
    """
    Standard experiment lifecycle used by scenarios 1–3 and 5.

    Parameters
    ----------
    scenario_name   : str   — used for node name and bag filename prefix.
    world_file      : str   — absolute path to the Gazebo .world file.
    motion_sequence : list  — list of (linear_x, angular_z, duration_s).
    pre_motion_hook : callable(pub) | None
                      Called just before the motion sequence starts.  Scenarios
                      that need extra setup (e.g. spin up a relay node) should
                      pass this in.  Receives the cmd_vel publisher.
    """
    bag_proc       = None
    launch_proc    = None
    lio_proc       = None
    state_est_proc = None
    node           = None
    bag_path       = None

    try:
        # ── 0. Ensure no stale Gazebo is running ──────────────────────────────
        kill_stale_gazebo()

        # ── 1. Launch Gazebo simulation ───────────────────────────────────────
        print(f'[sim] World file  : {world_file}')
        if not os.path.isfile(world_file):
            raise FileNotFoundError(f'World file not found: {world_file}')

        launch_proc = subprocess.Popen(
            ['ros2', 'launch', 'moborobo_robot', 'minimal_gazebo.launch.py',
             f'world_file:={world_file}'],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            start_new_session=True,
        )
        print(f'[sim] Launch PID  : {launch_proc.pid}')

        # ── 2. Wait for Gazebo topics ─────────────────────────────────────────
        wait_for_topics()

        # ── 2b. Launch ig_lio ─────────────────────────────────────────────────
        print('[lio] Launching ig_lio...')
        lio_proc = subprocess.Popen(
            ['ros2', 'launch', 'ig_lio', 'ig_lio_velodyne.launch.py'],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            start_new_session=True,
        )
        print(f'[lio] PID: {lio_proc.pid}')

        # ── 2c. Launch state estimation ───────────────────────────────────────
        print('[ekf] Launching state_estimation...')
        state_est_proc = subprocess.Popen(
            ['ros2', 'launch', 'state_estimation', 'state_estimation.launch.py'],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            start_new_session=True,
        )
        print(f'[ekf] PID: {state_est_proc.pid}')

        # ── 3. Init rclpy + publisher ─────────────────────────────────────────
        rclpy.init()
        node        = Node(scenario_name + '_runner')
        cmd_vel_pub = node.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        time.sleep(1.0)   # allow DDS publisher discovery

        # ── 4. Start bag recording ────────────────────────────────────────────
        bag_path = make_bag_path(scenario_name)
        bag_proc = start_bag_recording(bag_path)
        time.sleep(2.0)   # let the bag writer initialise

        # ── 5. Stabilisation wait ─────────────────────────────────────────────
        print(f'[main] Stabilisation wait: {STABILIZATION_WAIT_S:.0f}s')
        time.sleep(STABILIZATION_WAIT_S)

        # ── 6. Optional pre-motion setup ──────────────────────────────────────
        if pre_motion_hook is not None:
            pre_motion_hook(cmd_vel_pub)

        # ── 7. Motion sequence ────────────────────────────────────────────────
        print('[main] Starting motion sequence...')
        execute_motion(cmd_vel_pub, motion_sequence)

        # ── 8. Stop robot ─────────────────────────────────────────────────────
        print('[main] Halting robot...')
        stop_robot(cmd_vel_pub, duration_s=2.0)

        print('[main] Scenario complete.')
        print(f'[main] Bag saved to: {bag_path}')

    except KeyboardInterrupt:
        print('\n[main] Interrupted by user.')

    except Exception as exc:
        print(f'[main] ERROR: {exc}', file=sys.stderr)
        raise

    finally:
        stop_bag_recording(bag_proc)

        if node is not None:
            node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

        _shutdown_proc(state_est_proc, 'ekf')
        _shutdown_proc(lio_proc,       'lio')
        _shutdown_proc(launch_proc,    'sim')
        if launch_proc is not None:
            print('[sim] Simulation stopped.')
