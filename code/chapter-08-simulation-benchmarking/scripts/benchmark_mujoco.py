#!/usr/bin/env python3
"""
MuJoCo benchmark script.

Usage:
    python3 benchmark_mujoco.py --num-robots 1 --duration 60
    python3 benchmark_mujoco.py --num-robots 100 --duration 60 --output results/mujoco_100.json
"""

import argparse
import time
import numpy as np
import mujoco
from common import BenchmarkResults, get_cpu_usage, get_memory_usage_mb, get_gpu_usage, save_results, print_results

parser = argparse.ArgumentParser(description="MuJoCo Benchmark")
parser.add_argument("--num-robots", type=int, default=1, help="Number of robots")
parser.add_argument("--duration", type=float, default=60.0, help="Simulation duration (seconds)")
parser.add_argument("--output", type=str, default=None, help="Output JSON file")
args = parser.parse_args()


def create_humanoid_xml(num_robots: int) -> str:
    """
    Generate MuJoCo XML with multiple humanoid robots.

    Args:
        num_robots: Number of robots to spawn

    Returns:
        XML string
    """
    xml = """
<mujoco model="humanoid_benchmark">
  <option timestep="0.001" integrator="RK4"/>
  <asset>
    <mesh name="torso" file="torso.stl" scale="0.1 0.1 0.1"/>
  </asset>

  <worldbody>
    <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
    <geom name="floor" type="plane" size="10 10 0.1" rgba="0.8 0.9 0.8 1"/>
"""

    grid_size = int(np.ceil(np.sqrt(num_robots)))
    for i in range(num_robots):
        row = i // grid_size
        col = i % grid_size
        x = col * 2.0 - (grid_size - 1)
        y = row * 2.0 - (grid_size - 1)

        xml += f"""
    <body name="humanoid_{i}" pos="{x} {y} 0.5">
      <freejoint/>
      <geom name="torso_{i}" type="box" size="0.1 0.05 0.2" rgba="0.8 0.3 0.3 1" mass="5"/>
      <geom name="head_{i}" type="sphere" size="0.08" pos="0 0 0.25" rgba="0.9 0.7 0.6 1" mass="1"/>

      <body name="left_arm_{i}" pos="0 0.12 0.15">
        <joint name="l_shoulder_{i}" type="hinge" axis="0 1 0" range="-90 90"/>
        <geom type="capsule" size="0.025" fromto="0 0 0 0 0.15 -0.15" rgba="0.7 0.5 0.5 1" mass="0.5"/>
      </body>

      <body name="right_arm_{i}" pos="0 -0.12 0.15">
        <joint name="r_shoulder_{i}" type="hinge" axis="0 1 0" range="-90 90"/>
        <geom type="capsule" size="0.025" fromto="0 0 0 0 -0.15 -0.15" rgba="0.7 0.5 0.5 1" mass="0.5"/>
      </body>

      <body name="left_leg_{i}" pos="0 0.06 -0.2">
        <joint name="l_hip_{i}" type="hinge" axis="0 1 0" range="-90 90"/>
        <geom type="capsule" size="0.035" fromto="0 0 0 0 0 -0.3" rgba="0.6 0.6 0.8 1" mass="1"/>
      </body>

      <body name="right_leg_{i}" pos="0 -0.06 -0.2">
        <joint name="r_hip_{i}" type="hinge" axis="0 1 0" range="-90 90"/>
        <geom type="capsule" size="0.035" fromto="0 0 0 0 0 -0.3" rgba="0.6 0.6 0.8 1" mass="1"/>
      </body>
    </body>
"""

    xml += """
  </worldbody>
</mujoco>
"""
    return xml


def main():
    """Main benchmark function."""
    print(f"MuJoCo Benchmark: {args.num_robots} robot(s) for {args.duration}s")

    # Create model
    xml_string = create_humanoid_xml(args.num_robots)
    model = mujoco.MjModel.from_xml_string(xml_string)
    data = mujoco.MjData(model)

    # Warmup
    print("Warming up...")
    for _ in range(100):
        mujoco.mj_step(model, data)

    # Benchmark
    print("Running benchmark...")
    start_time = time.time()
    start_sim_time = data.time
    start_mem = get_memory_usage_mb()

    frames = 0
    while data.time - start_sim_time < args.duration:
        mujoco.mj_step(model, data)
        frames += 1

    elapsed = time.time() - start_time
    end_mem = get_memory_usage_mb()

    # Get GPU usage (MuJoCo is CPU-only by default)
    gpu_percent, vram_mb = get_gpu_usage()

    # Calculate metrics
    fps = frames / elapsed
    rtf = args.duration / elapsed
    cpu = get_cpu_usage()
    memory = end_mem - start_mem

    # Create results
    results = BenchmarkResults(
        simulator="MuJoCo",
        num_robots=args.num_robots,
        duration_seconds=args.duration,
        total_frames=frames,
        fps=fps,
        rtf=rtf,
        cpu_percent=cpu,
        gpu_percent=gpu_percent,
        memory_mb=memory,
        vram_mb=vram_mb
    )

    # Print and save
    print_results(results)

    if args.output:
        save_results(results, args.output)


if __name__ == "__main__":
    main()
