#!/usr/bin/env python3
"""
evaluate.py
===========
Play a recorded bag and collect raw odometry data from three topics into CSV files.

Usage:
    python3 evaluate.py <path_to_bag_dir>
"""

import csv
import json
import os
import signal
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from gazebo_msgs.msg import ModelStates
from nav_msgs.msg import Odometry

TOPIC_GROUND_TRUTH = '/gazebo/model_states'
TOPIC_LIO_ODOM     = '/lio/odometry'
TOPIC_GPS_FUSED    = '/gps/odom'
ROBOT_NAME         = 'moborobo_robot'


class DataCollector(Node):
    def __init__(self):
        super().__init__('evaluate_collector')
        self.ground_truth_poses = []
        self.lio_poses          = []
        self.gps_fused_poses    = []

        self.create_subscription(
            ModelStates, TOPIC_GROUND_TRUTH, self._cb_ground_truth, 10)
        self.create_subscription(
            Odometry, TOPIC_LIO_ODOM, self._cb_lio, 10)
        self.create_subscription(
            Odometry, TOPIC_GPS_FUSED, self._cb_gps_fused, 10)

    def _cb_ground_truth(self, msg):
        ts = time.time()
        try:
            idx = msg.name.index(ROBOT_NAME)
        except ValueError:
            return
        p = msg.pose[idx].position
        o = msg.pose[idx].orientation
        self.ground_truth_poses.append(
            (ts, p.x, p.y, p.z, o.x, o.y, o.z, o.w))

    def _cb_lio(self, msg):
        ts = time.time()
        p  = msg.pose.pose.position
        o  = msg.pose.pose.orientation
        self.lio_poses.append(
            (ts, p.x, p.y, p.z, o.x, o.y, o.z, o.w))

    def _cb_gps_fused(self, msg):
        ts = time.time()
        p  = msg.pose.pose.position
        o  = msg.pose.pose.orientation
        self.gps_fused_poses.append(
            (ts, p.x, p.y, p.z, o.x, o.y, o.z, o.w))


def _write_csv(path, rows):
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['timestamp', 'x', 'y', 'z', 'qx', 'qy', 'qz', 'qw'])
        writer.writerows(rows)


def _stop_proc(proc, timeout_s=5.0):
    if proc is None or proc.poll() is not None:
        return
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()


def main():
    if len(sys.argv) != 2:
        print('Usage: python3 evaluate.py <path_to_bag_dir>', file=sys.stderr)
        sys.exit(1)

    bag_dir = os.path.abspath(sys.argv[1])
    if not os.path.isdir(bag_dir):
        print(f'ERROR: bag directory not found: {bag_dir}', file=sys.stderr)
        sys.exit(1)

    # Load metadata
    metadata_path = os.path.join(bag_dir, 'metadata.json')
    metadata = {}
    if os.path.isfile(metadata_path):
        with open(metadata_path) as f:
            metadata = json.load(f)

    # ── Step 1: Play the bag ─────────────────────────────────────────────────
    cmd = ['ros2', 'bag', 'play', '--wait-for-all-acked', '200', bag_dir]
    print(f'[bag] Playing: {" ".join(cmd)}')
    bag_proc = subprocess.Popen(cmd)

    print('[bag] Waiting 3 s before collecting data...')
    time.sleep(3.0)

    # ── Step 2: Collect data ─────────────────────────────────────────────────
    rclpy.init()
    node = DataCollector()

    print('[collect] Collecting until bag play exits...')
    try:
        while bag_proc.poll() is None:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        print('\n[collect] Interrupted.')
    finally:
        node.destroy_node()
        rclpy.shutdown()

    # ── Step 4: Cleanup bag play ─────────────────────────────────────────────
    _stop_proc(bag_proc)

    # ── Step 3: Save output ──────────────────────────────────────────────────
    gt_path  = os.path.join(bag_dir, 'ground_truth.csv')
    lio_path = os.path.join(bag_dir, 'lio_odom.csv')
    gps_path = os.path.join(bag_dir, 'gps_fused_odom.csv')

    _write_csv(gt_path,  node.ground_truth_poses)
    _write_csv(lio_path, node.lio_poses)
    _write_csv(gps_path, node.gps_fused_poses)

    if str(metadata.get('scenario_id')) == '4':
        outage_path = os.path.join(bag_dir, 'outage_window.txt')
        with open(outage_path, 'w') as f:
            f.write(f"gps_outage_start: {metadata['gps_outage_start']}\n")
            f.write(f"gps_outage_end: {metadata['gps_outage_end']}\n")
        print(f'[output] Outage window written to: {outage_path}')

    print(f'[output] ground_truth : {len(node.ground_truth_poses):5d} poses -> {gt_path}')
    print(f'[output] lio_odom     : {len(node.lio_poses):5d} poses -> {lio_path}')
    print(f'[output] gps_fused    : {len(node.gps_fused_poses):5d} poses -> {gps_path}')


if __name__ == '__main__':
    main()
