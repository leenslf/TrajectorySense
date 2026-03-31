#!/usr/bin/env python3
"""
Scenario 3 — Large Irregular Loop
===================================
Executes an irregular loop using alternating straight and turning segments,
returning near the origin after ~105 s of motion.  Tests odometry drift over
a longer, more complex trajectory.

Path sketch (all within the 25×25 m arena):
  fwd 8s → arc 4s → fwd 6s → arc 5s →
  fwd 8s → arc 4s → fwd 6s → arc 5s → (≈ back near start)

Usage:
    python3 scenario_3_large_loop.py
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from experiment_common import WORLD_DIR, run_experiment

# ─── CONFIG ──────────────────────────────────────────────────────────────────

SCENARIO_NAME = 'scenario_3_large_loop'

WORLD_FILE = os.path.join(WORLD_DIR, 'gravel_large_loop.world')

# Motion sequence: (linear_x [m/s], angular_z [rad/s], duration [s])
#
# 3 s stationary at start, then alternating forward/arc segments, then 3 s stop.
# Arc radii ≈ 0.35 / 0.4 = 0.875 m; heading changes: 1.6, 2.0, 1.6, 2.0 rad
# Total heading rotation ≈ 7.2 rad (≈ 412°) — returns close to origin heading.
MOTION_SEQUENCE = [
    (0.0,  0.0,  3.0),   # stationary — start
    (0.35, 0.0,  8.0),   # forward
    (0.35, 0.4,  4.0),   # gentle left arc (~1.6 rad turn)
    (0.35, 0.0,  6.0),   # forward
    (0.35, 0.4,  5.0),   # arc (~2.0 rad turn)
    (0.35, 0.0,  8.0),   # forward
    (0.35, 0.4,  4.0),   # arc (~1.6 rad turn)
    (0.35, 0.0,  6.0),   # forward
    (0.35, 0.4,  5.0),   # arc (~2.0 rad turn, returning toward start)
    (0.0,  0.0,  3.0),   # stationary — end
]

# ─── MAIN ────────────────────────────────────────────────────────────────────

def main():
    run_experiment(SCENARIO_NAME, WORLD_FILE, MOTION_SEQUENCE)


if __name__ == '__main__':
    main()
