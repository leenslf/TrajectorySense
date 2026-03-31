#!/usr/bin/env python3
"""
Scenario 1 — Baseline Short Loop
=================================
Drives a gentle circular arc (~1.5 loops, radius ~1.2 m) for ~90 s.
Establishes the baseline odometry performance with GPS fusion in a
structured environment with static obstacles.

Usage:
    python3 scenario_1_baseline_loop.py
    # or, after colcon build with --symlink-install:
    ros2 run gps_experiments scenario_1_baseline_loop
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from experiment_common import WORLD_DIR, run_experiment

# ─── CONFIG ──────────────────────────────────────────────────────────────────

SCENARIO_NAME = 'scenario_1_baseline_loop'

WORLD_FILE = os.path.join(WORLD_DIR, 'gravel_large_loop.world')

# Motion sequence: list of (linear_x [m/s], angular_z [rad/s], duration [s])
MOTION_SEQUENCE = [
    (0.3, 0.25, 75.0),   # circular arc — ~1.5 loops at radius ≈ 1.2 m
]

# ─── MAIN ────────────────────────────────────────────────────────────────────

def main():
    run_experiment(SCENARIO_NAME, WORLD_FILE, MOTION_SEQUENCE)


if __name__ == '__main__':
    main()
