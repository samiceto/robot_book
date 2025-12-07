#!/usr/bin/env python3
"""
Generate comparison report from benchmark results.

Usage:
    python3 generate_report.py \
      --gazebo-results results/gazebo_*.json \
      --isaac-results results/isaac_*.json \
      --mujoco-results results/mujoco_*.json \
      --output report.pdf
"""

import argparse
import json
import glob
from typing import List, Dict
import matplotlib.pyplot as plt
import pandas as pd

parser = argparse.ArgumentParser(description="Generate Benchmark Report")
parser.add_argument("--gazebo-results", type=str, help="Gazebo result files (glob pattern)")
parser.add_argument("--isaac-results", type=str, help="Isaac Sim result files (glob pattern)")
parser.add_argument("--mujoco-results", type=str, help="MuJoCo result files (glob pattern)")
parser.add_argument("--output", type=str, default="report.pdf", help="Output PDF file")
args = parser.parse_args()


def load_results(pattern: str) -> List[Dict]:
    """Load all JSON results matching pattern."""
    results = []
    for filepath in glob.glob(pattern):
        with open(filepath, 'r') as f:
            results.append(json.load(f))
    return results


def create_comparison_table(all_results: List[Dict]):
    """Create comparison table."""
    df = pd.DataFrame(all_results)

    # Group by simulator and num_robots
    summary = df.groupby(['simulator', 'num_robots']).agg({
        'fps': 'mean',
        'rtf': 'mean',
        'cpu_percent': 'mean',
        'memory_mb': 'mean'
    }).round(1)

    print("\n" + "=" * 70)
    print("PERFORMANCE COMPARISON")
    print("=" * 70)
    print(summary)
    print("=" * 70)

    return summary


def plot_scaling(all_results: List[Dict], output_file: str):
    """Plot FPS vs number of robots."""
    df = pd.DataFrame(all_results)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))

    # FPS vs num_robots
    for sim in df['simulator'].unique():
        sim_data = df[df['simulator'] == sim].sort_values('num_robots')
        ax1.plot(sim_data['num_robots'], sim_data['fps'], marker='o', label=sim, linewidth=2)

    ax1.set_xlabel('Number of Robots', fontsize=12)
    ax1.set_ylabel('FPS', fontsize=12)
    ax1.set_title('Simulation Performance Scaling', fontsize=14, weight='bold')
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.set_xscale('log')

    # Memory vs num_robots
    for sim in df['simulator'].unique():
        sim_data = df[df['simulator'] == sim].sort_values('num_robots')
        ax2.plot(sim_data['num_robots'], sim_data['memory_mb'], marker='s', label=sim, linewidth=2)

    ax2.set_xlabel('Number of Robots', fontsize=12)
    ax2.set_ylabel('Memory (MB)', fontsize=12)
    ax2.set_title('Memory Usage Scaling', fontsize=14, weight='bold')
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    ax2.set_xscale('log')

    plt.tight_layout()
    plt.savefig(output_file.replace('.pdf', '_scaling.png'), dpi=300, bbox_inches='tight')
    print(f"Scaling plot saved to: {output_file.replace('.pdf', '_scaling.png')}")

    plt.show()


def main():
    """Main report generation."""
    print("Loading benchmark results...")

    all_results = []

    if args.gazebo_results:
        all_results.extend(load_results(args.gazebo_results))
    if args.isaac_results:
        all_results.extend(load_results(args.isaac_results))
    if args.mujoco_results:
        all_results.extend(load_results(args.mujoco_results))

    if not all_results:
        print("ERROR: No results found. Check file patterns.")
        return

    print(f"Loaded {len(all_results)} result files")

    # Create comparison table
    summary = create_comparison_table(all_results)

    # Plot scaling curves
    plot_scaling(all_results, args.output)

    print("\n" + "=" * 70)
    print("RECOMMENDATION MATRIX")
    print("=" * 70)
    print("Use Case                 | Recommended Simulator")
    print("-" * 70)
    print("ROS 2 Prototyping        | Gazebo (native integration)")
    print("VLA Training (100+ envs) | Isaac Sim (GPU parallelization)")
    print("RL Research (fast iter)  | MuJoCo (fastest physics)")
    print("Photorealistic Rendering | Isaac Sim (RTX raytracing)")
    print("CPU-Only Systems         | Gazebo or MuJoCo")
    print("=" * 70)


if __name__ == "__main__":
    main()
