#!/usr/bin/env python3
"""
analyze.py
==========
Analyze GPS-fusion evaluation results from CSV files produced by evaluate.py.

Usage:
    python3 analyze.py <path_to_bag_dir>
"""

import csv
import json
import math
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

SYNC_TOLERANCE = 0.05   # seconds
ALIGN_WINDOW   = 5.0    # seconds


# ── Utility helpers ───────────────────────────────────────────────────────────

def quat_to_yaw(qx, qy, qz, qw):
    return math.atan2(2 * (qw * qz + qx * qy), 1 - 2 * (qy**2 + qz**2))


def wrap_angle(a):
    """Wrap angle to [-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


def load_csv(path):
    """Return list of row dicts with float values."""
    poses = []
    with open(path, newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            poses.append({k: float(v) for k, v in row.items()})
    return poses


# ── Step 1: Temporal synchronization ─────────────────────────────────────────

def sync_trajectories(gt_poses, odom_poses):
    """
    For each odom pose find the nearest ground-truth pose by timestamp.
    Accept matches within SYNC_TOLERANCE seconds.
    Returns list of (gt_pose, odom_pose, timestamp).
    """
    matched = []
    gt_ts = [p['timestamp'] for p in gt_poses]
    n = len(gt_ts)

    for odom_pose in odom_poses:
        ts = odom_pose['timestamp']

        # Binary search for nearest gt timestamp
        lo, hi = 0, n - 1
        while lo < hi:
            mid = (lo + hi) // 2
            if gt_ts[mid] < ts:
                lo = mid + 1
            else:
                hi = mid

        # Check lo and its left neighbour
        best_idx = lo
        if lo > 0 and abs(gt_ts[lo - 1] - ts) < abs(gt_ts[lo] - ts):
            best_idx = lo - 1

        if abs(gt_ts[best_idx] - ts) <= SYNC_TOLERANCE:
            matched.append((gt_poses[best_idx], odom_pose, ts))

    return matched


# ── Step 2: Frame alignment ───────────────────────────────────────────────────

def compute_alignment_transform(gt_poses_xyyaw, odom_poses_xyyaw):
    """
    Compute 2D rigid transform (dx, dy, dyaw) that maps odom onto gt.
    Inputs: lists of (x, y, yaw).
    Uses SVD-based closed-form least-squares on the (x, y) components.
    Returns (dx, dy, dyaw).
    """
    gt_xy   = np.array([(p[0], p[1]) for p in gt_poses_xyyaw], dtype=float)
    odom_xy = np.array([(p[0], p[1]) for p in odom_poses_xyyaw], dtype=float)

    # Centroids
    gt_c   = gt_xy.mean(axis=0)
    odom_c = odom_xy.mean(axis=0)

    # Centred point sets
    gt_centered   = gt_xy   - gt_c
    odom_centered = odom_xy - odom_c

    # 2x2 cross-covariance
    H = odom_centered.T @ gt_centered

    U, _, Vt = np.linalg.svd(H)

    # Ensure proper rotation (det = +1, not reflection)
    d = np.linalg.det(Vt.T @ U.T)
    R = Vt.T @ np.diag([1.0, d]) @ U.T

    dyaw = math.atan2(R[1, 0], R[0, 0])

    t = gt_c - R @ odom_c
    dx, dy = float(t[0]), float(t[1])

    return dx, dy, dyaw


def apply_transform(x, y, yaw, dx, dy, dyaw):
    """Apply 2D rigid transform (dx, dy, dyaw) to a single pose."""
    c, s = math.cos(dyaw), math.sin(dyaw)
    return (
        c * x - s * y + dx,
        s * x + c * y + dy,
        wrap_angle(yaw + dyaw),
    )


def align_trajectory(matched_pairs):
    """
    Estimate an alignment transform from the first ALIGN_WINDOW seconds of
    matched pairs, then apply it to all poses in the trajectory.
    Returns list of (gt_pose_dict, (ax, ay, ayaw), timestamp).
    """
    if not matched_pairs:
        return []

    t0 = matched_pairs[0][2]

    # Collect poses inside the alignment window
    gt_win, odom_win = [], []
    for gt_pose, odom_pose, ts in matched_pairs:
        if ts - t0 > ALIGN_WINDOW:
            break
        gt_win.append((
            gt_pose['x'], gt_pose['y'],
            quat_to_yaw(gt_pose['qx'], gt_pose['qy'],
                        gt_pose['qz'], gt_pose['qw']),
        ))
        odom_win.append((
            odom_pose['x'], odom_pose['y'],
            quat_to_yaw(odom_pose['qx'], odom_pose['qy'],
                        odom_pose['qz'], odom_pose['qw']),
        ))

    if len(gt_win) >= 2:
        dx, dy, dyaw = compute_alignment_transform(gt_win, odom_win)
    else:
        dx, dy, dyaw = 0.0, 0.0, 0.0

    # Apply transform to every odom pose
    aligned = []
    for gt_pose, odom_pose, ts in matched_pairs:
        ox   = odom_pose['x']
        oy   = odom_pose['y']
        oyaw = quat_to_yaw(odom_pose['qx'], odom_pose['qy'],
                           odom_pose['qz'], odom_pose['qw'])
        aligned.append((gt_pose, apply_transform(ox, oy, oyaw, dx, dy, dyaw), ts))

    return aligned


# ── Step 3: Compute metrics ───────────────────────────────────────────────────

def _pos_error(gt_pose, aligned_odom):
    return math.sqrt((gt_pose['x'] - aligned_odom[0])**2 +
                     (gt_pose['y'] - aligned_odom[1])**2)


def _yaw_error_deg(gt_pose, aligned_odom):
    gt_yaw = quat_to_yaw(gt_pose['qx'], gt_pose['qy'],
                          gt_pose['qz'], gt_pose['qw'])
    diff = abs(wrap_angle(aligned_odom[2] - gt_yaw))
    # Wrap to [0, pi]
    if diff > math.pi:
        diff = 2 * math.pi - diff
    return math.degrees(diff)


def compute_metrics(aligned_pairs):
    """Return dict of position and yaw metrics for a list of aligned pairs."""
    zero = {'rmse_pos': 0.0, 'mean_pos': 0.0, 'max_pos': 0.0, 'final_pos': 0.0,
            'rmse_yaw': 0.0, 'mean_yaw': 0.0, 'max_yaw': 0.0}
    if not aligned_pairs:
        return zero

    pos_errs = [_pos_error(gt, odom) for gt, odom, _ in aligned_pairs]
    yaw_errs = [_yaw_error_deg(gt, odom) for gt, odom, _ in aligned_pairs]
    n = len(pos_errs)

    return {
        'rmse_pos':  round(math.sqrt(sum(e**2 for e in pos_errs) / n), 4),
        'mean_pos':  round(sum(pos_errs) / n, 4),
        'max_pos':   round(max(pos_errs), 4),
        'final_pos': round(pos_errs[-1], 4),
        'rmse_yaw':  round(math.sqrt(sum(e**2 for e in yaw_errs) / n), 4),
        'mean_yaw':  round(sum(yaw_errs) / n, 4),
        'max_yaw':   round(max(yaw_errs), 4),
    }


def compute_phase_metrics(lio_aligned, gps_aligned, outage_start, outage_end):
    """
    Split each trajectory into before/during/after the GPS outage window and
    compute metrics for each phase.
    Returns dict with keys: lio_before, lio_during, lio_after,
                             gps_before, gps_during, gps_after.
    """
    def split(pairs):
        before = [(g, o, t) for g, o, t in pairs if t < outage_start]
        during = [(g, o, t) for g, o, t in pairs if outage_start <= t <= outage_end]
        after  = [(g, o, t) for g, o, t in pairs if t > outage_end]
        return before, during, after

    lio_b, lio_d, lio_a = split(lio_aligned)
    gps_b, gps_d, gps_a = split(gps_aligned)

    return {
        'lio_before': compute_metrics(lio_b),
        'lio_during': compute_metrics(lio_d),
        'lio_after':  compute_metrics(lio_a),
        'gps_before': compute_metrics(gps_b),
        'gps_during': compute_metrics(gps_d),
        'gps_after':  compute_metrics(gps_a),
    }


# ── Step 5: Plots ─────────────────────────────────────────────────────────────

def make_trajectory_plot(bag_dir, scenario_id,
                         gt_poses, lio_aligned):
    fig, ax = plt.subplots(figsize=(8, 8))

    # Ground truth
    gt_x = [p['x'] for p in gt_poses]
    gt_y = [p['y'] for p in gt_poses]
    ax.plot(gt_x, gt_y, 'k-', label='Ground Truth', linewidth=1.5)
    ax.plot(gt_x[0], gt_y[0], 'ko', markersize=7)

    # LIO odometry
    if lio_aligned:
        lx = [odom[0] for _, odom, _ in lio_aligned]
        ly = [odom[1] for _, odom, _ in lio_aligned]
        ax.plot(lx, ly, 'r--', label='LIO Odometry', linewidth=1.2)
        ax.plot(lx[0], ly[0], 'ro', markersize=7)

    ax.set_aspect('equal')
    ax.grid(True)
    ax.legend()
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    ax.set_title(f'Scenario {scenario_id} \u2014 Trajectory Comparison')

    out_path = os.path.join(bag_dir, 'trajectory_plot.pdf')
    fig.savefig(out_path, bbox_inches='tight')
    plt.close(fig)
    print(f'[plot] Saved: {out_path}')


def make_error_plot(bag_dir, scenario_id, lio_aligned):
    fig, ax = plt.subplots(figsize=(10, 5))

    t0 = lio_aligned[0][2] if lio_aligned else 0.0

    if lio_aligned:
        lio_t  = [ts - t0 for _, _, ts in lio_aligned]
        lio_pe = [_pos_error(gt, odom) for gt, odom, _ in lio_aligned]
        ax.plot(lio_t, lio_pe, 'r-', label='LIO Position Error', linewidth=1.2)

    ax.set_xlabel('Time (s)')
    ax.set_ylabel('Position Error (m)')
    ax.set_title(f'Scenario {scenario_id} \u2014 Position Error Over Time')
    ax.grid(True)
    ax.legend()

    out_path = os.path.join(bag_dir, 'error_plot.pdf')
    fig.savefig(out_path, bbox_inches='tight')
    plt.close(fig)
    print(f'[plot] Saved: {out_path}')


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    if len(sys.argv) != 2:
        print('Usage: python3 analyze.py <path_to_bag_dir>', file=sys.stderr)
        sys.exit(1)

    bag_dir = os.path.abspath(sys.argv[1])
    if not os.path.isdir(bag_dir):
        print(f'ERROR: bag directory not found: {bag_dir}', file=sys.stderr)
        sys.exit(1)

    # Load metadata — prefer metadata.json, fall back to parsing the directory name
    metadata_path = os.path.join(bag_dir, 'metadata.json')
    if os.path.exists(metadata_path):
        with open(metadata_path) as f:
            metadata = json.load(f)
        scenario_id = int(metadata['scenario_id'])
    else:
        # Infer from directory name, e.g. "scenario_1_baseline_loop_run_..."
        import re
        m = re.search(r'scenario_(\d+)', os.path.basename(bag_dir))
        if not m:
            print('ERROR: cannot determine scenario_id (no metadata.json and directory name has no scenario_N pattern)', file=sys.stderr)
            sys.exit(1)
        scenario_id = int(m.group(1))
        metadata = {'scenario_id': scenario_id}

    # Load CSVs
    gt_poses  = load_csv(os.path.join(bag_dir, 'ground_truth.csv'))
    lio_poses = load_csv(os.path.join(bag_dir, 'lio_odom.csv'))
    gps_poses = load_csv(os.path.join(bag_dir, 'gps_fused_odom.csv'))

    print(f'[load] ground_truth: {len(gt_poses)} poses')
    print(f'[load] lio_odom    : {len(lio_poses)} poses')
    print(f'[load] gps_fused   : {len(gps_poses)} poses')

    # Step 1: Temporal synchronization
    lio_matched = sync_trajectories(gt_poses, lio_poses)
    gps_matched = sync_trajectories(gt_poses, gps_poses)
    print(f'[sync] lio matched : {len(lio_matched)} pairs')
    print(f'[sync] gps matched : {len(gps_matched)} pairs')

    # Step 2: Frame alignment
    lio_aligned = align_trajectory(lio_matched)
    gps_aligned = align_trajectory(gps_matched)

    # Step 3: Compute overall metrics
    lio_metrics = compute_metrics(lio_aligned)
    gps_metrics = compute_metrics(gps_aligned)

    # Step 4: Build and write results.json
    results = {
        'scenario_id':   scenario_id,
        'n_samples_lio': len(lio_aligned),
        'n_samples_gps': len(gps_aligned),
        'lio':           lio_metrics,
        'gps_fused':     gps_metrics,
    }

    outage_start = outage_end = None
    if scenario_id == 4:
        outage_start = float(metadata['gps_outage_start'])
        outage_end   = float(metadata['gps_outage_end'])
        results['scenario_4_phases'] = compute_phase_metrics(
            lio_aligned, gps_aligned, outage_start, outage_end)

    results_path = os.path.join(bag_dir, 'results.json')
    with open(results_path, 'w') as f:
        json.dump(results, f, indent=2)
    print(f'[output] Written: {results_path}')

    # Step 5: Generate plots
    make_trajectory_plot(bag_dir, scenario_id, gt_poses, lio_aligned)
    make_error_plot(bag_dir, scenario_id, lio_aligned)

    # Stdout summary
    print()
    print(f'=== Scenario {scenario_id} Summary ===')
    print(f'  LIO  samples : {len(lio_aligned)}')
    print(f'  GPS  samples : {len(gps_aligned)}')
    print(f'  LIO  RMSE pos: {lio_metrics["rmse_pos"]:.4f} m   '
          f'RMSE yaw: {lio_metrics["rmse_yaw"]:.4f} deg')
    print(f'  GPS  RMSE pos: {gps_metrics["rmse_pos"]:.4f} m   '
          f'RMSE yaw: {gps_metrics["rmse_yaw"]:.4f} deg')


if __name__ == '__main__':
    main()
