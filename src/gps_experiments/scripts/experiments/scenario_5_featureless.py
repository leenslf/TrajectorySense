#!/usr/bin/env python3
"""
Scenario 5 — Featureless Environment
======================================
Drives straight segments with 90° turns in an empty arena (no interior
obstacles).  Tests how odometry degrades when LiDAR has no geometric
features to lock onto, making GPS fusion the dominant correction source.

World: gravel_featureless.world — four boundary walls, no interior obstacles.

Motion path (approximate, all within the 25×25 m arena):
  fwd 10s → 90° turn → fwd 8s → 90° turn → fwd 10s
  Maximum extent from origin ≈ 4.5 m + 3.6 m = ~8 m — well within bounds.

Usage:
    python3 scenario_5_featureless.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from experiment_common import WORLD_DIR, run_experiment

# ─── CONFIG ──────────────────────────────────────────────────────────────────

SCENARIO_NAME = 'scenario_5_featureless'

WORLD_FILE = os.path.join(WORLD_DIR, 'gravel_featureless.world')

# Motion sequence: (linear_x [m/s], angular_z [rad/s], duration [s])
#
#   Forward   10 s × 0.45 m/s ≈ 4.5 m
#   Turn  90° at 0.5 rad/s    ≈ 3.1 s  (π/2 / 0.5 ≈ 3.14 s)
#   Forward    8 s × 0.45 m/s ≈ 3.6 m
#   Turn  90° at 0.5 rad/s    ≈ 3.1 s
#   Forward   10 s × 0.45 m/s ≈ 4.5 m
MOTION_SEQUENCE = [
    (0.45, 0.0,  10.0),   # forward ~4.5 m
    (0.0,  0.5,   3.1),   # turn 90° in place
    (0.45, 0.0,   8.0),   # forward ~3.6 m
    (0.0,  0.5,   3.1),   # turn 90° in place
    (0.45, 0.0,  10.0),   # forward ~4.5 m
]

# ─── MAIN ────────────────────────────────────────────────────────────────────

def main():
    run_experiment(SCENARIO_NAME, WORLD_FILE, MOTION_SEQUENCE)


if __name__ == '__main__':
    main()
