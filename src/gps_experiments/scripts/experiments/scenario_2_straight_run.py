#!/usr/bin/env python3
"""
Scenario 2 — Straight Run
==========================
Drives forward ~8 m, stops, turns 180°, then returns ~8 m to origin.
Tests odometry accuracy on a simple back-and-forth linear trajectory.

Usage:
    python3 scenario_2_straight_run.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from experiment_common import WORLD_DIR, run_experiment

# ─── CONFIG ──────────────────────────────────────────────────────────────────

SCENARIO_NAME = 'scenario_2_straight_run'

WORLD_FILE = os.path.join(WORLD_DIR, 'gravel_large_loop.world')

# Motion sequence: (linear_x [m/s], angular_z [rad/s], duration [s])
#
#   Forward  ~8 m  at 0.4 m/s  → 20 s
#   Stop           (0 m/s)     →  2 s
#   Turn 180°      at 0.5 r/s  →  6.3 s  (π / 0.5 ≈ 6.28 s)
#   Return  ~8 m  at 0.4 m/s  → 20 s
MOTION_SEQUENCE = [
    (0.4,  0.0,  20.0),   # forward ~8 m
    (0.0,  0.0,   2.0),   # stop
    (0.0,  0.5,   6.3),   # turn 180° in place
    (0.4,  0.0,  20.0),   # return ~8 m
]

# ─── MAIN ────────────────────────────────────────────────────────────────────

def main():
    run_experiment(SCENARIO_NAME, WORLD_FILE, MOTION_SEQUENCE)


if __name__ == '__main__':
    main()
