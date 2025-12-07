# Chapter 6: Isaac Sim Introduction

This repository contains code examples for Chapter 6 of "Physical AI & Humanoid Robotics with ROS 2, Isaac Sim, and VLAs".

## Overview

Learn to use NVIDIA Isaac Sim for GPU-accelerated robotics simulation, photorealistic rendering, and ROS 2 integration. This chapter covers:

- Isaac Sim vs. Gazebo comparison and when to use each
- Installing Isaac Sim 2024.2 via Omniverse Launcher
- USD (Universal Scene Description) fundamentals
- URDF import and PhysX configuration
- ROS 2 bridge using ActionGraph
- Performance benchmarking across hardware tiers

## Repository Structure

```
chapter-06-isaac-sim-intro/
├── scripts/
│   ├── load_urdf.py              # Load URDF into Isaac Sim with PhysX config
│   ├── ros2_bridge.py            # Create ROS 2 bridge programmatically
│   ├── benchmark_performance.py  # Performance benchmarking script
│   └── multi_robot_scene.py      # Multi-robot scene with domain randomization
└── README.md                      # This file
```

## Prerequisites

### Hardware Requirements

**Minimum** (Budget Tier):
- GPU: NVIDIA RTX 4070 (12GB VRAM)
- CPU: 8-core
- RAM: 32GB
- Storage: 100GB SSD

**Recommended** (Mid Tier):
- GPU: NVIDIA RTX 4080 (16GB VRAM)
- CPU: 12-core
- RAM: 64GB
- Storage: 500GB NVMe SSD

### Software Requirements

- Ubuntu 22.04 LTS or 24.04 LTS
- NVIDIA Driver 550+ (check with `nvidia-smi`)
- Isaac Sim 2024.2 installed via Omniverse Launcher
- ROS 2 Iron or Jazzy
- Python 3.10+

### Install Isaac Sim

1. **Download Omniverse Launcher**:
   ```bash
   wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage
   chmod +x omniverse-launcher-linux.AppImage
   ./omniverse-launcher-linux.AppImage
   ```

2. **Install Isaac Sim 2024.2**:
   - In Launcher, go to "Exchange" tab
   - Search for "Isaac Sim"
   - Click "Install" on "Isaac Sim 2024.2.0"
   - Default install location: `~/.local/share/ov/pkg/isaac-sim-4.2.0/`

3. **Verify Installation**:
   ```bash
   cd ~/.local/share/ov/pkg/isaac-sim-4.2.0
   ./isaac-sim.sh
   # Isaac Sim GUI should launch (first launch takes 2-5 minutes for shader compilation)
   ```

### Install Dependencies

```bash
# ROS 2 Isaac integration (optional, for ros2_bridge.py)
sudo apt install ros-${ROS_DISTRO}-isaac-ros-nitros

# Python packages (optional, for performance stats)
pip install pynvml psutil
```

## Usage

### 1. Load URDF into Isaac Sim

**Script**: `load_urdf.py`

Loads Chapter 4 humanoid URDF into Isaac Sim and configures PhysX parameters.

```bash
cd ~/.local/share/ov/pkg/isaac-sim-4.2.0

# Load humanoid URDF
./python.sh ~/ros2_ws/src/robot-book-code/chapter-06-isaac-sim-intro/scripts/load_urdf.py

# Custom URDF path
./python.sh ~/ros2_ws/src/robot-book-code/chapter-06-isaac-sim-intro/scripts/load_urdf.py \
  --urdf /path/to/your/robot.urdf

# Headless mode (no GUI)
./python.sh ~/ros2_ws/src/robot-book-code/chapter-06-isaac-sim-intro/scripts/load_urdf.py --headless
```

**Expected Output**:
- Isaac Sim GUI opens (unless --headless)
- Humanoid robot appears 1m above ground plane
- Press **Space** to start simulation
- Robot should fall and land stably

**Controls**:
- **Space**: Play/Pause simulation
- **Backspace**: Reset to initial state
- **Middle Mouse**: Orbit camera
- **Scroll Wheel**: Zoom

### 2. ROS 2 Bridge

**Script**: `ros2_bridge.py`

Creates ActionGraph programmatically to publish sensor data to ROS 2.

**Terminal 1**: Launch Isaac Sim with ROS 2 bridge

```bash
cd ~/.local/share/ov/pkg/isaac-sim-4.2.0
./python.sh ~/ros2_ws/src/robot-book-code/chapter-06-isaac-sim-intro/scripts/ros2_bridge.py
# Press Space to start simulation
```

**Terminal 2**: Verify ROS 2 topics

```bash
source /opt/ros/iron/setup.bash

# List topics
ros2 topic list
# Expected:
# /joint_states
# /camera/image_raw
# /camera/camera_info

# Check joint states
ros2 topic hz /joint_states
# Expected: ~60 Hz

# Echo joint states
ros2 topic echo /joint_states --once
# Should show all joint names and positions

# View camera
ros2 run rqt_image_view rqt_image_view
# Select topic: /camera/image_raw
```

### 3. Performance Benchmarking

**Script**: `benchmark_performance.py`

Measures FPS, physics rate, and GPU memory usage across different configurations.

**Single Robot Benchmark**:

```bash
cd ~/.local/share/ov/pkg/isaac-sim-4.2.0
./python.sh ~/ros2_ws/src/robot-book-code/chapter-06-isaac-sim-intro/scripts/benchmark_performance.py \
  --duration 10
```

**Expected Output**:
```
======================================================================
BENCHMARK RESULTS
======================================================================
Number of Robots:       1
Duration:               10.00 seconds
Total Frames:           550
Average FPS:            55.0
Physics Rate:           60 Hz
Speedup vs Real-Time:   55.0x
GPU Memory Used:        1200 MB
======================================================================

Estimated Hardware Tier: Mid (RTX 4080)
```

**Multi-Robot Scaling Test**:

```bash
# 10 robots
./python.sh ~/ros2_ws/src/robot-book-code/chapter-06-isaac-sim-intro/scripts/benchmark_performance.py \
  --num-robots 10 --duration 10

# 100 robots (requires RTX 4080+)
./python.sh ~/ros2_ws/src/robot-book-code/chapter-06-isaac-sim-intro/scripts/benchmark_performance.py \
  --num-robots 100 --duration 10

# Headless mode (max physics throughput)
./python.sh ~/ros2_ws/src/robot-book-code/chapter-06-isaac-sim-intro/scripts/benchmark_performance.py \
  --num-robots 100 --headless --duration 10
```

**Performance Targets**:

| Hardware Tier | GPU | 1 Robot FPS | 10 Robots FPS | 100 Robots FPS |
|---------------|-----|-------------|---------------|----------------|
| Budget | RTX 4070 | 45-55 | 30-40 | 15-20 |
| Mid | RTX 4080 | 90-110 | 75-85 | 30-40 |
| Premium | RTX 4090 | 180-200 | 150-170 | 80-100 |

## Troubleshooting

### Issue 1: "Failed to load libcuda.so"

**Cause**: NVIDIA driver not installed or incompatible version

**Fix**:
```bash
# Check driver
nvidia-smi

# If missing or < 550, install:
sudo apt install nvidia-driver-550
sudo reboot

# Verify
nvidia-smi
# Driver Version should be 550.xx.xx or higher
```

### Issue 2: Robot Explodes on Play

**Cause**: Inertia values too low or joint stiffness too high

**Fix**:
1. Check URDF inertia: `<mass>` should be > 0.1 kg per link
2. Reduce joint stiffness in `load_urdf.py`:
   ```python
   import_config.default_drive_strength = 1000.0  # Instead of 5000.0
   import_config.default_position_drive_damping = 100.0  # Instead of 500.0
   ```
3. Increase PhysX solver iterations:
   ```python
   physx_articulation_api.CreateSolverPositionIterationCountAttr(16)  # Was 8
   ```

### Issue 3: Very Low FPS (< 10)

**Cause**: Complex collision geometry or high-resolution rendering

**Fix**:
1. **Run headless**: `--headless` flag (disables rendering)
2. **Reduce viewport resolution**:
   - In Isaac Sim: Viewport → Render Settings → Resolution: 1280×720
3. **Simplify collision meshes**:
   - In URDF importer: Enable "Merge Fixed Joints"
4. **Disable RTX**:
   - Viewport → Rendering → Mode: Lit (disables ray tracing)
5. **Check GPU utilization**:
   ```bash
   watch -n 0.5 nvidia-smi
   # GPU utilization should be > 80%
   # If < 50%, CPU bottleneck (increase num_robots)
   ```

### Issue 4: ROS 2 Topics Not Published

**Cause**: ROS 2 bridge not initialized or ActionGraph errors

**Fix**:
1. **Check ActionGraph**:
   - In Isaac Sim: Window → Visual Scripting → Action Graph
   - Verify nodes are connected (green arrows)
   - Check Console for errors (red text)

2. **Verify ROS 2 installation**:
   ```bash
   ros2 pkg list | grep isaac_ros
   # Should show isaac_ros packages
   ```

3. **Check domain ID**:
   - Ensure Isaac Sim and ROS 2 use same domain ID (default: 0)
   - In `ros2_bridge.py`: `ROS2Context.inputs:domain_id = 0`

4. **Restart**:
   - Close Isaac Sim
   - Run `ros2 daemon stop && ros2 daemon start`
   - Relaunch Isaac Sim script

### Issue 5: GPU Out of Memory (OOM)

**Cause**: Too many robots or high-resolution textures

**Symptoms**:
```
[Error] CUDA out of memory
```

**Fix**:
1. **Reduce number of robots**:
   ```bash
   # Instead of 100 robots, try 50
   --num-robots 50
   ```

2. **Enable instancing** (share meshes):
   ```python
   from omni.isaac.core.utils.extensions import enable_extension
   enable_extension("omni.physx.flatcache")
   ```

3. **Reduce camera resolution**:
   ```python
   resolution=(320, 240)  # Instead of (640, 480)
   ```

4. **Monitor VRAM usage**:
   ```bash
   watch -n 0.5 nvidia-smi
   # Check "Memory-Usage" column
   ```

## Key Concepts

### USD vs URDF

| Aspect | URDF | USD |
|--------|------|-----|
| **Format** | XML | Binary (.usdc) or ASCII (.usda) |
| **Purpose** | Robot description | General 3D scene |
| **Hierarchy** | Tree (links/joints) | Arbitrary graph (references, layers) |
| **Editing** | Destructive | Non-destructive (layers) |
| **File Size** | Small (< 100 KB) | Large (1-100 MB with meshes) |

### PhysX Articulations

Isaac Sim uses **PhysX articulations** (reduced-coordinate formulation) for robots:
- Faster than maximal-coordinate (Featherstone algorithm)
- GPU-parallelizable (1000+ robots on single GPU)
- Requires careful mass/inertia tuning (same as URDF)

**Key Parameters**:
- `SolverPositionIterationCount`: 8 (default), increase for stability
- `StabilizationThreshold`: 0.001 (stop simulating if stable)
- `SleepThreshold`: 0.005 (performance optimization)

### ActionGraph vs Python API

| Approach | Pros | Cons |
|----------|------|------|
| **ActionGraph (GUI)** | Visual, no coding, real-time preview | Hard to version control, manual setup |
| **Python API** | Scriptable, reproducible, git-friendly | Requires Python knowledge, no live preview |

**Recommendation**: Use Python API for reproducibility (as in `ros2_bridge.py`)

## Further Reading

- **Isaac Sim Docs**: https://docs.omniverse.nvidia.com/isaacsim/latest/
- **USD Tutorial**: https://graphics.pixar.com/usd/docs/index.html
- **PhysX Manual**: https://nvidia-omniverse.github.io/PhysX/physx/5.4.0/
- **Isaac ROS**: https://github.com/NVIDIA-ISAAC-ROS

## License

MIT License - See LICENSE file in repository root.

## Support

For issues, questions, or contributions:
- GitHub Issues: https://github.com/yourusername/robot-book-code/issues
- Book Website: https://physicalai-book.com
- NVIDIA Isaac Forum: https://forums.developer.nvidia.com/c/agx-autonomous-machines/isaac/
