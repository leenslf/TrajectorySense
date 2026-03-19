# GPS Experiments

Evaluation pipeline for GPS-fused LiDAR-inertial odometry.

## Pipeline order

1. `scripts/experiments/scenario_N_*.py` — record experiment bags into `bags/`
2. `scripts/evaluate.py <bag_dir>`      — replay bag, collect raw CSVs
3. `scripts/analyze.py <bag_dir>`       — align, compute metrics, generate per-scenario plots



## Bags

Bags are not tracked by git (see .gitignore).
