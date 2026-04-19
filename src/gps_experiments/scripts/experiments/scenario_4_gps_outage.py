#!/usr/bin/env python3
"""
Scenario 4 — GPS Outage and Recovery
======================================
Drives a continuous gentle arc for ~120 s.  At t=30 s into motion the GPS
relay stops forwarding messages (simulating signal loss); at t=70 s it resumes.

GPS relay architecture
----------------------
The relay node subscribes to GPS_SOURCE_TOPIC and republishes to
GPS_RELAY_TOPIC.  During the outage window it simply withholds messages.

  TODO: Confirm the two topic names before running:
    • GPS_SOURCE_TOPIC  — topic published by the Gazebo GPS sensor plugin.
                         
    • GPS_RELAY_TOPIC   — topic that robot_localization's EKF subscribes to for
                          GPS data.  Set this in the EKF yaml and here so they
                          agree.  Example: '/gps/fix_relayed'

The relay runs in a daemon thread; the main thread handles timing and motion.
Outage start/end wall-clock timestamps are written to a sidecar .txt file
alongside the bag directory for post-analysis alignment.

Usage:
    python3 scenario_4_gps_outage.py
"""

import datetime
import os
import subprocess
import sys
import threading
import time

# Allow importing experiment_common from the same directory when running as a
# standalone script (i.e. without a proper package install).
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from experiment_common import (
    WORLD_DIR,
    STABILIZATION_WAIT_S,
    CMD_VEL_TOPIC,
    CMD_VEL_RATE_HZ,
    wait_for_topics,
    make_bag_path,
    start_bag_recording,
    stop_bag_recording,
    stop_robot,
    kill_stale_gazebo,
    _shutdown_proc,
)

# ─── CONFIG ──────────────────────────────────────────────────────────────────

SCENARIO_NAME = 'scenario_4_gps_outage'

WORLD_FILE = os.path.join(WORLD_DIR, 'gravel_experiment.world')

# GPS relay topics

GPS_SOURCE_TOPIC = '/gps/fix'          
GPS_RELAY_TOPIC  = '/gps/fix_relayed'  # TODO: must match EKF yaml input topic

# GPS outage timing (seconds *after motion starts*)
GPS_OUTAGE_START_S = 30.0   # suppress GPS at this offset
GPS_OUTAGE_END_S   = 70.0   # resume  GPS at this offset

# Motion sequence: one continuous gentle arc for the full duration.
# Radius ≈ 0.3 / 0.15 = 2 m; ~3 full circles over 120 s.
MOTION_SEQUENCE = [
    (0.3, 0.15, 120.0),   # continuous gentle arc — DO NOT STOP mid-run
]

# ─── GPS RELAY NODE ───────────────────────────────────────────────────────────

class GpsRelayNode(Node):
    """
    Subscribes to GPS_SOURCE_TOPIC; republishes to GPS_RELAY_TOPIC.
    Call set_active(False) to suppress forwarding, set_active(True) to resume.

    NOTE: This relay only affects EKF fusion if the EKF subscribes to
    GPS_RELAY_TOPIC (not to GPS_SOURCE_TOPIC directly).
    See module docstring for the required configuration steps.
    """

    def __init__(self):
        super().__init__('gps_relay')
        from sensor_msgs.msg import NavSatFix
        self._active = True
        self._lock   = threading.Lock()
        self._sub = self.create_subscription(
            NavSatFix, GPS_SOURCE_TOPIC, self._callback, 10)
        self._pub = self.create_publisher(NavSatFix, GPS_RELAY_TOPIC, 10)
        self.get_logger().info(
            f'GPS relay: {GPS_SOURCE_TOPIC} → {GPS_RELAY_TOPIC}  (active)')

    def _callback(self, msg):
        with self._lock:
            if self._active:
                self._pub.publish(msg)

    def set_active(self, active: bool):
        with self._lock:
            self._active = active
        self.get_logger().info(
            f'GPS relay state → {"ACTIVE" if active else "SUPPRESSED"}')

# ─── SCENARIO-SPECIFIC HELPERS ───────────────────────────────────────────────

def execute_motion_with_gps_outage(cmd_pub, relay_node, motion_sequence,
                                    rate_hz=CMD_VEL_RATE_HZ):
    """
    Run motion_sequence while managing the GPS relay outage window.
    Returns (outage_start_wall, outage_end_wall, outage_start_offset, outage_end_offset).
    """
    dt             = 1.0 / rate_hz
    outage_started = False
    outage_ended   = False
    outage_start_wall = outage_end_wall = outage_start_real = None
    motion_start   = time.time()
    total_steps    = len(motion_sequence)

    for i, (linear_x, angular_z, duration_s) in enumerate(motion_sequence, start=1):
        msg           = Twist()
        msg.linear.x  = float(linear_x)
        msg.angular.z = float(angular_z)
        print(f'[motion] Step {i}/{total_steps}: '
              f'lin={linear_x:.2f} m/s  ang={angular_z:.2f} rad/s  '
              f'dur={duration_s:.1f}s')
        step_end = time.time() + duration_s
        while time.time() < step_end:
            elapsed = time.time() - motion_start

            if not outage_started and elapsed >= GPS_OUTAGE_START_S:
                outage_started    = True
                outage_start_real = time.time()
                outage_start_wall = datetime.datetime.now().isoformat()
                relay_node.set_active(False)
                print(f'[gps]  OUTAGE START at motion t={elapsed:.2f}s  '
                      f'({outage_start_wall})')

            if outage_started and not outage_ended and elapsed >= GPS_OUTAGE_END_S:
                outage_ended    = True
                outage_end_wall = datetime.datetime.now().isoformat()
                relay_node.set_active(True)
                print(f'[gps]  OUTAGE END   at motion t={elapsed:.2f}s  '
                      f'({outage_end_wall})  '
                      f'outage duration={time.time() - outage_start_real:.1f}s')

            cmd_pub.publish(msg)
            time.sleep(dt)

    return outage_start_wall, outage_end_wall, GPS_OUTAGE_START_S, GPS_OUTAGE_END_S


def write_outage_sidecar(bag_path, outage_start_wall, outage_end_wall,
                          outage_start_offset, outage_end_offset):
    """Write outage timestamps to a sidecar .txt file for post-analysis."""
    sidecar = bag_path + '_gps_outage.txt'
    with open(sidecar, 'w') as fh:
        fh.write(f'scenario            : {SCENARIO_NAME}\n')
        fh.write(f'gps_source_topic    : {GPS_SOURCE_TOPIC}\n')
        fh.write(f'gps_relay_topic     : {GPS_RELAY_TOPIC}\n')
        fh.write(f'outage_start_wall   : {outage_start_wall}\n')
        fh.write(f'outage_end_wall     : {outage_end_wall}\n')
        fh.write(f'outage_start_offset : {outage_start_offset:.3f} s  '
                 f'(seconds after motion start)\n')
        fh.write(f'outage_end_offset   : {outage_end_offset:.3f} s\n')
    print(f'[gps]  Outage log written to: {sidecar}')

# ─── MAIN ────────────────────────────────────────────────────────────────────

def main():
    bag_proc       = None
    launch_proc    = None
    lio_proc       = None
    state_est_proc = None
    node           = None
    relay_node     = None
    spin_thread    = None
    bag_path       = None

    try:
        # ── 0. Ensure no stale Gazebo is running ──────────────────────────────
        kill_stale_gazebo()

        # ── 1. Launch Gazebo simulation ───────────────────────────────────────
        print(f'[sim] World file  : {WORLD_FILE}')
        if not os.path.isfile(WORLD_FILE):
            raise FileNotFoundError(f'World file not found: {WORLD_FILE}')

        launch_proc = subprocess.Popen(
            ['ros2', 'launch', 'moborobo_robot', 'minimal_gazebo.launch.py',
             f'world_file:={WORLD_FILE}'],
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

        # ── 3. Init rclpy, cmd_vel publisher, GPS relay node ─────────────────
        rclpy.init()
        node        = Node(SCENARIO_NAME + '_runner')
        relay_node  = GpsRelayNode()
        cmd_vel_pub = node.create_publisher(Twist, CMD_VEL_TOPIC, 10)

        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(relay_node)
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        time.sleep(1.0)   # allow DDS publisher discovery

        # ── 4. Start bag recording ────────────────────────────────────────────
        bag_path = make_bag_path(SCENARIO_NAME)
        bag_proc = start_bag_recording(bag_path)
        time.sleep(2.0)   # let the bag writer initialise

        # ── 5. Stabilisation wait ─────────────────────────────────────────────
        print(f'[main] Stabilisation wait: {STABILIZATION_WAIT_S:.0f}s')
        time.sleep(STABILIZATION_WAIT_S)

        # ── 6. Motion + GPS outage management ────────────────────────────────
        print('[main] Starting motion (GPS relay active)...')
        print(f'[main] GPS outage scheduled: '
              f't={GPS_OUTAGE_START_S:.0f}s → t={GPS_OUTAGE_END_S:.0f}s')
        outage_results = execute_motion_with_gps_outage(
            cmd_vel_pub, relay_node, MOTION_SEQUENCE)

        # ── 7. Stop robot ─────────────────────────────────────────────────────
        print('[main] Halting robot...')
        stop_robot(cmd_vel_pub, duration_s=2.0)

        # ── 8. Write sidecar log ──────────────────────────────────────────────
        write_outage_sidecar(bag_path, *outage_results)

        print('[main] Scenario complete.')
        print(f'[main] Bag saved to: {bag_path}')

    except KeyboardInterrupt:
        print('\n[main] Interrupted by user.')

    except Exception as exc:
        print(f'[main] ERROR: {exc}', file=sys.stderr)
        raise

    finally:
        stop_bag_recording(bag_proc)

        try:
            rclpy.shutdown()
        except Exception:
            pass
        if spin_thread is not None:
            spin_thread.join(timeout=3.0)
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass

        _shutdown_proc(state_est_proc, 'ekf')
        _shutdown_proc(lio_proc,       'lio')
        _shutdown_proc(launch_proc,    'sim')
        if launch_proc is not None:
            print('[sim] Simulation stopped.')


if __name__ == '__main__':
    main()
