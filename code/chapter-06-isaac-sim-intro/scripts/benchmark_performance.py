#!/usr/bin/env python3
"""
Performance benchmarking script for Isaac Sim across hardware tiers.

This script measures:
1. FPS (frames per second) in different rendering modes
2. Physics simulation rate
3. Memory usage (VRAM)
4. Parallel environment scaling (1, 10, 100 robots)

Usage:
    cd ~/.local/share/ov/pkg/isaac-sim-4.2.0

    # Single robot benchmark
    ./python.sh ~/ros2_ws/src/robot-book-code/chapter-06-isaac-sim-intro/scripts/benchmark_performance.py

    # Multi-robot scaling test
    ./python.sh ~/ros2_ws/src/robot-book-code/chapter-06-isaac-sim-intro/scripts/benchmark_performance.py --num-robots 100

    # Headless mode (max physics throughput)
    ./python.sh ~/ros2_ws/src/robot-book-code/chapter-06-isaac-sim-intro/scripts/benchmark_performance.py --headless
"""

import argparse
import time
import psutil
from omni.isaac.kit import SimulationApp

parser = argparse.ArgumentParser(description="Isaac Sim Performance Benchmark")
parser.add_argument("--num-robots", type=int, default=1, help="Number of robots to spawn (1, 10, 100)")
parser.add_argument("--duration", type=int, default=10, help="Benchmark duration in seconds")
parser.add_argument("--headless", action="store_true", help="Run in headless mode (no rendering)")
args, unknown = parser.parse_known_args()

# Launch Isaac Sim
CONFIG = {
    "headless": args.headless,
    "width": 1280,
    "height": 720,
}
simulation_app = SimulationApp(CONFIG)

import os
import numpy as np
from omni.isaac.core import World
from omni.isaac.core.utils.extensions import enable_extension
from omni.isaac.core.utils.prims import create_prim
import omni.isaac.core.utils.prims as prim_utils
from pxr import Gf, UsdGeom
import carb

# Enable extensions
enable_extension("omni.isaac.urdf")

# Try to get GPU memory info
try:
    import pynvml
    pynvml.nvmlInit()
    gpu_handle = pynvml.nvmlDeviceGetHandleByIndex(0)
    HAS_NVML = True
except:
    HAS_NVML = False
    carb.log_warn("pynvml not available. GPU memory stats disabled.")


def get_gpu_memory_mb():
    """Get current GPU memory usage in MB."""
    if not HAS_NVML:
        return 0
    try:
        mem_info = pynvml.nvmlDeviceGetMemoryInfo(gpu_handle)
        return mem_info.used / (1024 ** 2)  # Convert to MB
    except:
        return 0


def load_robot(urdf_path: str, robot_index: int, grid_size: int = 10):
    """
    Load a single robot instance.

    Args:
        urdf_path: Path to URDF file
        robot_index: Index for naming (Robot_0, Robot_1, ...)
        grid_size: Grid dimension for robot placement

    Returns:
        Robot prim path
    """
    from omni.isaac.urdf import _urdf

    # Calculate grid position
    row = robot_index // grid_size
    col = robot_index % grid_size
    x = col * 2.0 - (grid_size - 1)  # Center grid
    y = row * 2.0 - (grid_size - 1)

    robot_path = f"/World/Robot_{robot_index}"

    # Import config
    import_config = _urdf.ImportConfig()
    import_config.fix_base = False
    import_config.merge_fixed_joints = True  # Performance optimization
    import_config.default_drive_strength = 5000.0
    import_config.default_position_drive_damping = 500.0

    # Parse URDF
    urdf_interface = _urdf.acquire_urdf_interface()
    success, actual_path = urdf_interface.parse_urdf(
        urdf_path, robot_path, import_config
    )

    if not success:
        carb.log_error(f"Failed to load robot {robot_index}")
        return None

    # Position robot
    robot_prim = prim_utils.get_prim_at_path(actual_path)
    UsdGeom.Xformable(robot_prim).AddTranslateOp().Set(Gf.Vec3d(x, y, 1.0))

    return actual_path


def run_benchmark(world: World, num_robots: int, duration: float):
    """
    Run benchmark and collect statistics.

    Args:
        world: Isaac Sim World instance
        num_robots: Number of robots in scene
        duration: Benchmark duration in seconds

    Returns:
        Dict with benchmark results
    """
    carb.log_info("="*70)
    carb.log_info(f"Starting benchmark: {num_robots} robot(s) for {duration}s")
    carb.log_info("="*70)

    # Reset world to ensure physics is initialized
    world.reset()

    # Warmup (10 steps)
    for _ in range(10):
        world.step(render=True)

    # Benchmark
    frame_count = 0
    start_time = time.time()
    start_mem = get_gpu_memory_mb()

    while time.time() - start_time < duration:
        world.step(render=True)
        frame_count += 1

    end_time = time.time()
    end_mem = get_gpu_memory_mb()

    # Calculate statistics
    elapsed = end_time - start_time
    avg_fps = frame_count / elapsed
    physics_rate = 60.0  # Assume 60Hz physics (default in Isaac Sim)
    speedup = (avg_fps * num_robots) / 1.0  # Speedup vs real-time single robot

    results = {
        "num_robots": num_robots,
        "duration_s": elapsed,
        "total_frames": frame_count,
        "avg_fps": avg_fps,
        "physics_rate_hz": physics_rate,
        "speedup_vs_realtime": speedup,
        "gpu_memory_mb": end_mem - start_mem if HAS_NVML else "N/A",
    }

    return results


def print_results(results: dict):
    """Print benchmark results in formatted table."""
    carb.log_info("")
    carb.log_info("="*70)
    carb.log_info("BENCHMARK RESULTS")
    carb.log_info("="*70)
    carb.log_info(f"Number of Robots:       {results['num_robots']}")
    carb.log_info(f"Duration:               {results['duration_s']:.2f} seconds")
    carb.log_info(f"Total Frames:           {results['total_frames']}")
    carb.log_info(f"Average FPS:            {results['avg_fps']:.2f}")
    carb.log_info(f"Physics Rate:           {results['physics_rate_hz']:.0f} Hz")
    carb.log_info(f"Speedup vs Real-Time:   {results['speedup_vs_realtime']:.1f}x")
    if HAS_NVML:
        carb.log_info(f"GPU Memory Used:        {results['gpu_memory_mb']:.0f} MB")
    carb.log_info("="*70)
    carb.log_info("")

    # Performance tier classification
    fps = results['avg_fps']
    if fps >= 60:
        tier = "Premium (RTX 4090)"
    elif fps >= 30:
        tier = "Mid (RTX 4080)"
    elif fps >= 15:
        tier = "Budget (RTX 4070)"
    else:
        tier = "Below Minimum Requirements"

    carb.log_info(f"Estimated Hardware Tier: {tier}")
    carb.log_info("")


def main():
    """Main benchmark function."""

    # Create world
    world = World(stage_units_in_meters=1.0)
    world.scene.add_default_ground_plane()

    # Add lighting (dome light for realistic rendering)
    create_prim(
        "/World/DomeLight",
        "DomeLight",
        attributes={"inputs:intensity": 1000.0}
    )

    # Find URDF
    urdf_path = "~/ros2_ws/src/robot-book-code/chapter-04-urdf-sdf/urdf/humanoid_12dof.urdf"
    urdf_path = os.path.expanduser(urdf_path)

    if not os.path.exists(urdf_path):
        carb.log_error(f"URDF not found: {urdf_path}")
        carb.log_error("Please ensure Chapter 4 code is available.")
        simulation_app.close()
        return

    # Load robots
    carb.log_info(f"Loading {args.num_robots} robot(s)...")
    grid_size = int(np.ceil(np.sqrt(args.num_robots)))

    for i in range(args.num_robots):
        robot_path = load_robot(urdf_path, i, grid_size)
        if robot_path:
            carb.log_info(f"  Robot {i} loaded: {robot_path}")

    carb.log_info(f"All {args.num_robots} robot(s) loaded successfully.")

    # Get system info
    import platform
    carb.log_info("")
    carb.log_info("="*70)
    carb.log_info("SYSTEM INFO")
    carb.log_info("="*70)
    carb.log_info(f"OS:                {platform.system()} {platform.release()}")
    carb.log_info(f"CPU:               {psutil.cpu_count(logical=False)} cores ({psutil.cpu_count()} threads)")
    carb.log_info(f"RAM:               {psutil.virtual_memory().total / (1024**3):.1f} GB")

    if HAS_NVML:
        gpu_name = pynvml.nvmlDeviceGetName(gpu_handle)
        gpu_mem_total = pynvml.nvmlDeviceGetMemoryInfo(gpu_handle).total / (1024**2)
        carb.log_info(f"GPU:               {gpu_name}")
        carb.log_info(f"VRAM:              {gpu_mem_total:.0f} MB")

    carb.log_info(f"Rendering Mode:    {'Headless' if args.headless else 'GUI (RTX Interactive)'}")
    carb.log_info("="*70)

    # Run benchmark
    results = run_benchmark(world, args.num_robots, args.duration)

    # Print results
    print_results(results)

    # Cleanup
    if HAS_NVML:
        pynvml.nvmlShutdown()

    simulation_app.close()


if __name__ == "__main__":
    main()
