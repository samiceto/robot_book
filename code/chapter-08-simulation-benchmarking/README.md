# Chapter 8: Simulation Benchmarking

Benchmark suite comparing Gazebo, Isaac Sim, and MuJoCo across performance and fidelity metrics.

## Overview

This repository contains automated benchmark scripts for comparing three major robotics simulators:
- **Gazebo Harmonic** (CPU-based, ROS 2 native)
- **Isaac Sim 2024.2** (GPU-accelerated, PhysX)
- **MuJoCo 3.0+** (Optimized contact physics)

## Repository Structure

```
chapter-08-simulation-benchmarking/
├── scripts/
│   ├── benchmark_gazebo.py       # Gazebo benchmark runner
│   ├── benchmark_isaac.py        # Isaac Sim benchmark runner
│   ├── benchmark_mujoco.py       # MuJoCo benchmark runner
│   ├── generate_report.py        # Aggregate results and create PDF report
│   └── common.py                 # Shared utilities
├── models/
│   └── humanoid_12dof.xml        # MuJoCo model file
└── README.md                      # This file
```

## Prerequisites

### Software

- Ubuntu 22.04 or 24.04 LTS
- Gazebo Harmonic (Chapter 5)
- Isaac Sim 2024.2 (Chapter 6)
- MuJoCo 3.0+
- Python 3.10+

### Install MuJoCo

```bash
pip install mujoco numpy
python3 -c "import mujoco; print(mujoco.__version__)"
# Expected: 3.0.0 or higher
```

### Install Report Generation Dependencies

```bash
pip install matplotlib pandas seaborn reportlab
```

## Usage

### 1. Benchmark Gazebo

```bash
# Single robot
python3 scripts/benchmark_gazebo.py --num-robots 1 --duration 60

# 10 robots
python3 scripts/benchmark_gazebo.py --num-robots 10 --duration 60

# Save results to JSON
python3 scripts/benchmark_gazebo.py --num-robots 10 --duration 60 \
  --output results/gazebo_10robots.json
```

### 2. Benchmark Isaac Sim

```bash
cd ~/.local/share/ov/pkg/isaac-sim-4.2.0

# Single robot
./python.sh ~/ros2_ws/src/robot-book-code/chapter-08-simulation-benchmarking/scripts/benchmark_isaac.py \
  --num-robots 1 --duration 60

# 100 robots (headless for max performance)
./python.sh ~/ros2_ws/src/robot-book-code/chapter-08-simulation-benchmarking/scripts/benchmark_isaac.py \
  --num-robots 100 --duration 60 --headless \
  --output ~/results/isaac_100robots.json
```

### 3. Benchmark MuJoCo

```bash
# Single robot
python3 scripts/benchmark_mujoco.py --num-robots 1 --duration 60

# 100 robots
python3 scripts/benchmark_mujoco.py --num-robots 100 --duration 60 \
  --output results/mujoco_100robots.json
```

### 4. Generate Comparison Report

```bash
python3 scripts/generate_report.py \
  --gazebo-results results/gazebo_*.json \
  --isaac-results results/isaac_*.json \
  --mujoco-results results/mujoco_*.json \
  --output report.pdf
```

## Expected Results

### Performance Comparison (Mid Tier - RTX 4080)

| Simulator | 1 Robot FPS | 10 Robots FPS | 100 Robots FPS | Memory (100 robots) |
|-----------|-------------|---------------|----------------|---------------------|
| Gazebo    | 55-65       | 40-50         | 8-12           | 800 MB              |
| Isaac Sim | 90-110      | 75-85         | 40-50          | 4500 MB             |
| MuJoCo    | 180-220     | 160-180       | 120-140        | 200 MB              |

### Use Case Recommendations

- **Gazebo**: ROS 2 prototyping, no GPU, < 10 robots
- **Isaac Sim**: VLA training, synthetic data, 100+ robots, photorealism
- **MuJoCo**: RL research, fast iteration, contact-rich tasks

## Troubleshooting

### Gazebo: Low FPS

**Cause**: CPU bottleneck or complex collision geometry

**Fix**:
```bash
# Use DART physics (fastest for humanoids)
# Already configured in benchmark script
```

### Isaac Sim: CUDA Out of Memory

**Cause**: Too many robots for available VRAM

**Fix**:
```bash
# Reduce number of robots or use instancing
./python.sh scripts/benchmark_isaac.py --num-robots 50  # Instead of 100
```

### MuJoCo: "Model file not found"

**Cause**: humanoid_12dof.xml not in correct location

**Fix**:
```bash
# Ensure models/humanoid_12dof.xml exists
ls models/humanoid_12dof.xml
```

## License

MIT License - See LICENSE file in repository root.

## Support

- GitHub Issues: https://github.com/yourusername/robot-book-code/issues
- Book Website: https://physicalai-book.com
