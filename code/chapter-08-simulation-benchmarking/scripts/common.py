#!/usr/bin/env python3
"""
Common utilities for simulation benchmarking.
"""

import json
import os
import time
import psutil
from dataclasses import dataclass, asdict
from typing import Optional


@dataclass
class BenchmarkResults:
    """Container for benchmark results."""
    simulator: str
    num_robots: int
    duration_seconds: float
    total_frames: int
    fps: float
    rtf: float  # Real-time factor
    cpu_percent: float
    gpu_percent: Optional[float]
    memory_mb: float
    vram_mb: Optional[float]


def get_cpu_usage() -> float:
    """Get current CPU usage percentage."""
    return psutil.cpu_percent(interval=0.1)


def get_memory_usage_mb() -> float:
    """Get current process memory usage in MB."""
    process = psutil.Process(os.getpid())
    return process.memory_info().rss / (1024 ** 2)


def get_gpu_usage():
    """
    Get GPU usage percentage and VRAM usage in MB.

    Returns:
        Tuple of (gpu_percent, vram_mb) or (None, None) if unavailable
    """
    try:
        import pynvml
        pynvml.nvmlInit()
        handle = pynvml.nvmlDeviceGetHandleByIndex(0)

        # GPU utilization
        util = pynvml.nvmlDeviceGetUtilizationRates(handle)
        gpu_percent = util.gpu

        # VRAM usage
        mem_info = pynvml.nvmlDeviceGetMemoryInfo(handle)
        vram_mb = mem_info.used / (1024 ** 2)

        pynvml.nvmlShutdown()
        return gpu_percent, vram_mb
    except:
        return None, None


def save_results(results: BenchmarkResults, output_file: str):
    """Save benchmark results to JSON file."""
    os.makedirs(os.path.dirname(output_file), exist_ok=True)
    with open(output_file, 'w') as f:
        json.dump(asdict(results), f, indent=2)
    print(f"Results saved to: {output_file}")


def print_results(results: BenchmarkResults):
    """Print benchmark results in formatted table."""
    print("")
    print("=" * 70)
    print("BENCHMARK RESULTS")
    print("=" * 70)
    print(f"Simulator:        {results.simulator}")
    print(f"Robots:           {results.num_robots}")
    print(f"Duration:         {results.duration_seconds:.1f} seconds")
    print(f"Total Frames:     {results.total_frames}")
    print(f"Average FPS:      {results.fps:.1f}")
    print(f"Real-Time Factor: {results.rtf:.1f}x")
    print(f"CPU Usage:        {results.cpu_percent:.1f}%")
    if results.gpu_percent is not None:
        print(f"GPU Usage:        {results.gpu_percent:.1f}%")
    print(f"Memory:           {results.memory_mb:.0f} MB")
    if results.vram_mb is not None:
        print(f"VRAM:             {results.vram_mb:.0f} MB")
    print("=" * 70)
