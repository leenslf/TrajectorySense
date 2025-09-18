#!/usr/bin/env python3

import csv
import os
import glob
import math
from datetime import datetime

def load_csv(filename):
    """Load CSV and return data as list of dicts"""
    data = []
    with open(filename, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                data.append({
                    'pos_error': float(row['pos_error']),
                    'orient_error': float(row['orient_error']),
                    'timestamp': float(row['timestamp'])
                })
            except (ValueError, KeyError):
                continue
    return data

def calc_stats(values):
    """Calculate basic stats"""
    if not values:
        return {'mean': 0, 'std': 0, 'rmse': 0, 'max': 0, 'count': 0}
    
    n = len(values)
    mean = sum(values) / n
    variance = sum((x - mean) ** 2 for x in values) / n
    std = math.sqrt(variance)
    rmse = math.sqrt(sum(x**2 for x in values) / n)
    
    return {
        'mean': mean,
        'std': std,
        'rmse': rmse,
        'max': max(values),
        'count': n
    }

def generate_report(data_dir="test_results"):
    """Generate simple report from test data"""
    
    print("="*60)
    print("SLAM EVALUATION REPORT")
    print("="*60)
    print(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print()
    
    # Find all CSV files
    csv_files = glob.glob(os.path.join(data_dir, "*.csv"))
    
    if not csv_files:
        print(f"No CSV files found in {data_dir}")
        return
    
    # Group by test pattern
    patterns = {}
    for csv_file in csv_files:
        filename = os.path.basename(csv_file)
        
        # Extract pattern name from filename
        if 'straight' in filename:
            pattern = 'straight'
        elif 'rotate' in filename:
            pattern = 'rotate'
        elif 'square' in filename:
            pattern = 'square'
        elif 'circle' in filename:
            pattern = 'circle'
        else:
            pattern = 'unknown'
        
        if pattern not in patterns:
            patterns[pattern] = []
        
        data = load_csv(csv_file)
        if data:
            patterns[pattern].append({
                'filename': filename,
                'data': data
            })
            print(f"Loaded {len(data)} points from {filename}")
    
    print()
    
    # Analyze each pattern
    all_results = {}
    
    for pattern, runs in patterns.items():
        print(f"{pattern.upper()} RESULTS:")
        print("-" * 30)
        
        # Combine all runs for this pattern
        all_pos_errors = []
        all_orient_errors = []
        
        for run in runs:
            data = run['data']
            pos_errors = [d['pos_error'] for d in data]
            orient_errors = [d['orient_error'] for d in data]
            
            all_pos_errors.extend(pos_errors)
            all_orient_errors.extend(orient_errors)
            
            # Individual run stats
            pos_stats = calc_stats(pos_errors)
            orient_stats = calc_stats(orient_errors)
            
            duration = data[-1]['timestamp'] - data[0]['timestamp'] if len(data) > 1 else 0
            
            print(f"  Run: {run['filename']}")
            print(f"    Samples: {len(data)}, Duration: {duration:.1f}s")
            print(f"    Position RMSE: {pos_stats['rmse']*1000:.1f} mm")
            print(f"    Orientation RMSE: {math.degrees(orient_stats['rmse']):.2f}°")
            print(f"    Max pos error: {pos_stats['max']*1000:.1f} mm")
            print()
        
        # Pattern summary
        if all_pos_errors:
            pos_stats = calc_stats(all_pos_errors)
            orient_stats = calc_stats(all_orient_errors)
            
            all_results[pattern] = {
                'pos_rmse': pos_stats['rmse'],
                'orient_rmse': orient_stats['rmse'],
                'samples': len(all_pos_errors),
                'runs': len(runs)
            }
            
            print(f"  PATTERN SUMMARY ({len(runs)} runs, {len(all_pos_errors)} total samples):")
            print(f"    Position RMSE: {pos_stats['rmse']*1000:.1f} ± {pos_stats['std']*1000:.1f} mm")
            print(f"    Orientation RMSE: {math.degrees(orient_stats['rmse']):.2f} ± {math.degrees(orient_stats['std']):.2f}°")
            print(f"    Max error: {pos_stats['max']*1000:.1f} mm")
            print()
    
    # Overall summary
    if all_results:
        print("OVERALL SUMMARY:")
        print("-" * 30)
        
        best_pattern = min(all_results.keys(), key=lambda x: all_results[x]['pos_rmse'])
        worst_pattern = max(all_results.keys(), key=lambda x: all_results[x]['pos_rmse'])
        
        total_samples = sum(r['samples'] for r in all_results.values())
        total_runs = sum(r['runs'] for r in all_results.values())
        
        print(f"Total: {len(all_results)} patterns, {total_runs} runs, {total_samples} samples")
        print(f"Best pattern: {best_pattern} ({all_results[best_pattern]['pos_rmse']*1000:.1f} mm RMSE)")
        print(f"Worst pattern: {worst_pattern} ({all_results[worst_pattern]['pos_rmse']*1000:.1f} mm RMSE)")
        
        # System-wide RMSE
        all_system_pos = []
        all_system_orient = []
        for runs in patterns.values():
            for run in runs:
                for d in run['data']:
                    all_system_pos.append(d['pos_error'])
                    all_system_orient.append(d['orient_error'])
        
        system_pos_rmse = math.sqrt(sum(x**2 for x in all_system_pos) / len(all_system_pos))
        system_orient_rmse = math.sqrt(sum(x**2 for x in all_system_orient) / len(all_system_orient))
        
        print(f"System-wide position RMSE: {system_pos_rmse*1000:.1f} mm")
        print(f"System-wide orientation RMSE: {math.degrees(system_orient_rmse):.2f}°")
        
        # Performance assessment
        if system_pos_rmse < 0.05:
            rating = "EXCELLENT"
        elif system_pos_rmse < 0.10:
            rating = "GOOD"
        elif system_pos_rmse < 0.20:
            rating = "ACCEPTABLE"
        else:
            rating = "NEEDS IMPROVEMENT"
        
        print(f"Performance rating: {rating}")

def save_latex_table(data_dir="test_results", output_file="results_table.tex"):
    """Generate simple LaTeX table"""
    csv_files = glob.glob(os.path.join(data_dir, "*.csv"))
    
    patterns = {}
    for csv_file in csv_files:
        filename = os.path.basename(csv_file)
        
        if 'straight' in filename:
            pattern = 'straight'
        elif 'rotate' in filename:
            pattern = 'rotate'
        elif 'square' in filename:
            pattern = 'square'
        elif 'circle' in filename:
            pattern = 'circle'
        else:
            continue
            
        if pattern not in patterns:
            patterns[pattern] = []
        
        data = load_csv(csv_file)
        if data:
            patterns[pattern].extend(data)
    
    latex = r"""\begin{table}[h]
\centering
\caption{SLAM Performance Results}
\begin{tabular}{lccc}
\hline
\textbf{Pattern} & \textbf{Samples} & \textbf{Pos RMSE (mm)} & \textbf{Orient RMSE (°)} \\
\hline
"""
    
    for pattern, data in patterns.items():
        if data:
            pos_errors = [d['pos_error'] for d in data]
            orient_errors = [d['orient_error'] for d in data]
            
            pos_rmse = math.sqrt(sum(x**2 for x in pos_errors) / len(pos_errors))
            orient_rmse = math.sqrt(sum(x**2 for x in orient_errors) / len(orient_errors))
            
            latex += f"{pattern.capitalize()} & {len(data)} & {pos_rmse*1000:.1f} & {math.degrees(orient_rmse):.2f} \\\\\n"
    
    latex += r"""\hline
\end{tabular}
\end{table}
"""
    
    with open(output_file, 'w') as f:
        f.write(latex)
    
    print(f"LaTeX table saved to {output_file}")

if __name__ == "__main__":
    import sys
    
    data_dir = sys.argv[1] if len(sys.argv) > 1 else "test_results"
    
    generate_report(data_dir)
    
    if "--latex" in sys.argv:
        save_latex_table(data_dir)