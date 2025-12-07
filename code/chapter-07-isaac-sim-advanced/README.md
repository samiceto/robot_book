# Chapter 7: Isaac Sim Advanced - Domain Randomization & Synthetic Data

This repository contains code examples for Chapter 7 of "Physical AI & Humanoid Robotics with ROS 2, Isaac Sim, and VLAs".

## Overview

Learn advanced Isaac Sim techniques for sim-to-real transfer, synthetic data generation, and performance optimization. This chapter covers:

- Domain randomization for visual and physics parameters
- Synthetic data generation with automatic annotations
- Isaac Sim Replicator API for procedural scenes
- Performance profiling with Nsight Graphics
- Scene optimization techniques (LOD, instancing, convex decomposition)

## Repository Structure

```
chapter-07-isaac-sim-advanced/
├── scripts/
│   ├── domain_randomization.py          # Visual and physics randomization examples
│   ├── replicator_pipeline.py           # Complete Replicator pipeline
│   ├── generate_grasping_dataset.py     # 1000-image grasping dataset generator
│   ├── visualize_dataset.py             # Dataset visualization with bounding boxes
│   └── convert_to_coco.py               # Convert BasicWriter output to COCO format
└── README.md                             # This file
```

## Prerequisites

### Hardware Requirements

**Minimum** (Budget Tier):
- GPU: NVIDIA RTX 4070 (12GB VRAM)
- CPU: 8-core
- RAM: 32GB
- Storage: 200GB SSD (100GB for Isaac Sim, 100GB for datasets)

**Recommended** (Mid Tier):
- GPU: NVIDIA RTX 4080 (16GB VRAM)
- CPU: 12-core
- RAM: 64GB
- Storage: 500GB NVMe SSD

### Software Requirements

- Ubuntu 22.04 LTS or 24.04 LTS
- NVIDIA Driver 550+ (check with `nvidia-smi`)
- Isaac Sim 2024.2 installed via Omniverse Launcher
- Python 3.10+
- Optional: ROS 2 Iron or Jazzy

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
   # Isaac Sim GUI should launch (first launch takes 2-5 minutes)
   ```

### Install Dependencies

```bash
# Python packages for visualization and dataset conversion
pip install opencv-python pillow matplotlib pycocotools

# Optional: Install Nsight Graphics for profiling
# Download from: https://developer.nvidia.com/nsight-graphics
```

## Usage

### 1. Domain Randomization

**Script**: `domain_randomization.py`

Demonstrates visual (textures, colors, lighting) and physics (mass, friction, damping) randomization.

```bash
cd ~/.local/share/ov/pkg/isaac-sim-4.2.0

# Run domain randomization demo
./python.sh ~/ros2_ws/src/robot-book-code/chapter-07-isaac-sim-advanced/scripts/domain_randomization.py

# Options:
--headless              # Run without GUI
--num-episodes 100      # Number of randomization episodes
--visual-only           # Only randomize visual parameters
--physics-only          # Only randomize physics parameters
```

**Expected Output**:
- Isaac Sim GUI opens with a simple scene (robot + cubes)
- Every 2 seconds, scene randomizes:
  - Cube colors change
  - Lighting intensity changes
  - Physics parameters change (if enabled)
- Console logs show randomized parameter values

**What You'll Learn**:
- How to randomize textures and materials using USD APIs
- How to randomize physics properties (mass, friction, damping)
- How to randomize lighting (intensity, color temperature, rotation)
- Best practices for variance ranges

### 2. Replicator Pipeline

**Script**: `replicator_pipeline.py`

Complete Replicator pipeline with randomization and annotation.

```bash
cd ~/.local/share/ov/pkg/isaac-sim-4.2.0

# Run basic Replicator pipeline
./python.sh ~/ros2_ws/src/robot-book-code/chapter-07-isaac-sim-advanced/scripts/replicator_pipeline.py

# Generate specific number of frames
./python.sh ~/ros2_ws/src/robot-book-code/chapter-07-isaac-sim-advanced/scripts/replicator_pipeline.py \
  --num-frames 100

# Custom output directory
./python.sh ~/ros2_ws/src/robot-book-code/chapter-07-isaac-sim-advanced/scripts/replicator_pipeline.py \
  --output-dir ~/datasets/my_dataset

# Headless mode (faster)
./python.sh ~/ros2_ws/src/robot-book-code/chapter-07-isaac-sim-advanced/scripts/replicator_pipeline.py \
  --headless --num-frames 1000
```

**Expected Output**:
```
Output directory: ~/datasets/replicator_output/
├── rgb/
│   ├── rgb_0000.png
│   ├── rgb_0001.png
│   └── ...
├── semantic_segmentation/
│   ├── semantic_segmentation_0000.png
│   └── ...
├── bounding_box_2d_tight/
│   ├── bounding_box_2d_tight_0000.npy
│   └── ...
└── bounding_box_2d_tight.npy  # Aggregated bounding boxes
```

**Performance Targets**:
| Hardware Tier | Headless FPS | GUI FPS | 1000 Frames Time |
|---------------|--------------|---------|------------------|
| Budget (RTX 4070) | 30-40 | 15-20 | ~30 seconds |
| Mid (RTX 4080) | 80-100 | 40-50 | ~12 seconds |
| Premium (RTX 4090) | 150-200 | 80-100 | ~6 seconds |

### 3. Grasping Dataset Generation

**Script**: `generate_grasping_dataset.py`

Generates a 1000-image dataset for humanoid grasping with automatic annotations (Chapter 7 Hands-On Lab).

```bash
cd ~/.local/share/ov/pkg/isaac-sim-4.2.0

# Generate 1000-image grasping dataset
./python.sh ~/ros2_ws/src/robot-book-code/chapter-07-isaac-sim-advanced/scripts/generate_grasping_dataset.py \
  --num-images 1000 \
  --output-dir ~/datasets/grasping_dataset

# Quick test with 10 images
./python.sh ~/ros2_ws/src/robot-book-code/chapter-07-isaac-sim-advanced/scripts/generate_grasping_dataset.py \
  --num-images 10

# Headless mode for maximum throughput
./python.sh ~/ros2_ws/src/robot-book-code/chapter-07-isaac-sim-advanced/scripts/generate_grasping_dataset.py \
  --num-images 1000 \
  --headless
```

**Expected Output**:
```
~/datasets/grasping_dataset/
├── rgb/                    # 1000 RGB images (640×480)
├── depth/                  # 1000 depth maps (640×480, 16-bit PNG)
├── semantic_segmentation/  # 1000 segmentation masks
├── bounding_box_2d_tight/  # Per-frame bounding boxes (.npy)
├── bounding_box_2d_tight.npy  # Aggregated bounding boxes
└── metadata.json           # Dataset statistics and object classes
```

**Dataset Statistics** (logged to console):
```
======================================================================
DATASET GENERATION COMPLETE
======================================================================
Total Images:           1000
Output Directory:       ~/datasets/grasping_dataset
Generation Time:        42.3 seconds
Average FPS:            23.6
======================================================================
Object Classes:
  - cube (ID: 1): 1000 instances
  - cylinder (ID: 2): 1000 instances
  - sphere (ID: 3): 1000 instances
======================================================================
```

### 4. Dataset Visualization

**Script**: `visualize_dataset.py`

Visualizes generated datasets with bounding boxes overlaid.

```bash
# Visualize first 10 images from Replicator output
python3 ~/ros2_ws/src/robot-book-code/chapter-07-isaac-sim-advanced/scripts/visualize_dataset.py \
  --dataset-dir ~/datasets/replicator_output \
  --num-images 10

# Visualize grasping dataset
python3 ~/ros2_ws/src/robot-book-code/chapter-07-isaac-sim-advanced/scripts/visualize_dataset.py \
  --dataset-dir ~/datasets/grasping_dataset \
  --num-images 20

# Save visualizations to file instead of displaying
python3 ~/ros2_ws/src/robot-book-code/chapter-07-isaac-sim-advanced/scripts/visualize_dataset.py \
  --dataset-dir ~/datasets/grasping_dataset \
  --output-dir ~/visualizations \
  --num-images 100
```

**Expected Output**:
- Matplotlib window showing RGB images with bounding boxes overlaid
- Each bounding box labeled with class name
- Different colors for different object classes
- If `--output-dir` specified, saves images to disk

### 5. COCO Format Conversion

**Script**: `convert_to_coco.py`

Converts Isaac Sim Replicator output to COCO format for training object detectors.

```bash
# Convert Replicator output to COCO format
python3 ~/ros2_ws/src/robot-book-code/chapter-07-isaac-sim-advanced/scripts/convert_to_coco.py \
  --input-dir ~/datasets/replicator_output \
  --output-file ~/datasets/coco_annotations.json

# Convert grasping dataset
python3 ~/ros2_ws/src/robot-book-code/chapter-07-isaac-sim-advanced/scripts/convert_to_coco.py \
  --input-dir ~/datasets/grasping_dataset \
  --output-file ~/datasets/grasping_coco.json

# Specify custom category mapping
python3 ~/ros2_ws/src/robot-book-code/chapter-07-isaac-sim-advanced/scripts/convert_to_coco.py \
  --input-dir ~/datasets/grasping_dataset \
  --output-file ~/datasets/grasping_coco.json \
  --categories "cube,cylinder,sphere,humanoid"
```

**Expected Output**:
```
COCO dataset written to: ~/datasets/coco_annotations.json
Statistics:
  - Images: 1000
  - Annotations: 3000 (3 objects per image)
  - Categories: 3 (cube, cylinder, sphere)
```

**COCO Format Structure**:
```json
{
  "images": [
    {
      "id": 0,
      "file_name": "rgb/rgb_0000.png",
      "width": 640,
      "height": 480
    }
  ],
  "annotations": [
    {
      "id": 0,
      "image_id": 0,
      "category_id": 1,
      "bbox": [120, 200, 150, 180],
      "area": 27000,
      "iscrowd": 0
    }
  ],
  "categories": [
    {"id": 1, "name": "cube"},
    {"id": 2, "name": "cylinder"},
    {"id": 3, "name": "sphere"}
  ]
}
```

## Troubleshooting

### Issue 1: Replicator Import Error

**Symptom**:
```
ModuleNotFoundError: No module named 'omni.replicator.core'
```

**Cause**: Replicator extension not enabled

**Fix**:
```python
from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.replicator.core")
```

Ensure this is called **before** importing `omni.replicator.core`.

### Issue 2: Black/Empty Images in Output

**Cause**: Rendering not initialized or camera misconfigured

**Fix**:
1. **Ensure warmup frames**:
   ```python
   for _ in range(10):
       rep.orchestrator.step()  # Warmup before capturing
   ```

2. **Check camera is active**:
   ```python
   camera = rep.create.camera(position=(5, 5, 5))
   render_product = rep.create.render_product(camera, (640, 480))
   # Verify render_product is not None
   ```

3. **Verify lighting**:
   ```python
   rep.create.light(light_type="Dome", intensity=1000.0)
   ```

### Issue 3: Slow Generation Speed (< 5 FPS)

**Cause**: Complex scene or CPU bottleneck

**Fix**:
1. **Run headless**: `--headless` flag (disables GUI rendering)
2. **Reduce resolution**:
   ```python
   render_product = rep.create.render_product(camera, (320, 240))  # Instead of (640, 480)
   ```
3. **Simplify scene**: Reduce number of objects or polygon count
4. **Enable instancing**:
   ```python
   enable_extension("omni.physx.flatcache")
   ```

### Issue 4: COCO Conversion Errors

**Symptom**:
```
KeyError: 'data' in bounding_box_2d_tight.npy
```

**Cause**: Incompatible BasicWriter output format

**Fix**:
1. **Ensure BasicWriter is configured correctly**:
   ```python
   writer = rep.WriterRegistry.get("BasicWriter")
   writer.initialize(
       output_dir=output_dir,
       rgb=True,
       bounding_box_2d_tight=True,
       semantic_segmentation=True
   )
   ```

2. **Check .npy file structure**:
   ```python
   data = np.load("bounding_box_2d_tight.npy", allow_pickle=True)
   print(data.dtype.names)  # Should include 'data'
   ```

### Issue 5: GPU Out of Memory (OOM)

**Symptom**:
```
[Error] CUDA out of memory
```

**Cause**: Too many objects or high-resolution rendering

**Fix**:
1. **Reduce batch size**:
   ```python
   # Generate in smaller batches
   for batch in range(10):
       generate_batch(100)  # 10 batches of 100 instead of 1000 at once
   ```

2. **Lower resolution**:
   ```python
   resolution = (320, 240)  # Instead of (640, 480)
   ```

3. **Enable instancing** (share meshes):
   ```python
   enable_extension("omni.physx.flatcache")
   ```

4. **Monitor VRAM**:
   ```bash
   watch -n 0.5 nvidia-smi
   ```

## Performance Profiling

### Nsight Graphics Profiling

1. **Install Nsight Graphics**:
   - Download from: https://developer.nvidia.com/nsight-graphics
   - Extract and run: `./nv-nsight-gfx`

2. **Profile Isaac Sim**:
   - In Nsight: File → Connect → Add Activity → Frame Debugger
   - Executable: `~/.local/share/ov/pkg/isaac-sim-4.2.0/isaac-sim.sh`
   - Launch and capture frame (press `Ctrl+Z` in Isaac Sim)

3. **Analyze Hotspots**:
   - Look for expensive draw calls (> 5ms)
   - Check shader complexity
   - Identify overdraw (multiple layers rendered)

### Python Profiling

```bash
# Profile Replicator script with cProfile
cd ~/.local/share/ov/pkg/isaac-sim-4.2.0
./python.sh -m cProfile -o profile.stats \
  ~/ros2_ws/src/robot-book-code/chapter-07-isaac-sim-advanced/scripts/replicator_pipeline.py

# Analyze with snakeviz
pip install snakeviz
snakeviz profile.stats
```

## Key Concepts

### Domain Randomization

**Visual Randomization**:
- **Textures**: Random patterns/materials on objects
- **Colors**: RGB values within physically plausible ranges
- **Lighting**: Intensity (500-2000 lux), color temperature (2700-6500K), rotation

**Physics Randomization**:
- **Mass**: ±20% variance (e.g., 1.0 kg → 0.8-1.2 kg)
- **Friction**: 0.3-0.9 (wood, rubber, metal)
- **Restitution**: 0.0-0.8 (elastic vs inelastic collisions)
- **Damping**: 0.1-10.0 (air resistance, joint friction)

**Best Practices**:
- Randomize every parameter that varies in real world
- Use physically plausible ranges (don't randomize mass to 0.001 kg)
- Test on real robot to validate transfer

### Replicator API

**Core Components**:
1. **Randomizers**: Functions that modify scene (e.g., `rep.randomizer.register`)
2. **Render Products**: Camera outputs (RGB, depth, segmentation)
3. **Writers**: Save data to disk (BasicWriter, COCO format)
4. **Orchestrator**: Manages randomization and capture loop

**Typical Pipeline**:
```python
# 1. Setup scene
rep.create.light()
camera = rep.create.camera()

# 2. Register randomizers
with rep.trigger.on_frame():
    randomize_objects()
    randomize_lighting()

# 3. Configure output
writer = rep.WriterRegistry.get("BasicWriter")
render_product = rep.create.render_product(camera, (640, 480))
writer.attach(render_product)

# 4. Generate
rep.orchestrator.run()
```

### Synthetic Data Annotations

**Annotation Types**:
- **Bounding Boxes (2D)**: `[x_min, y_min, x_max, y_max]` in pixels
- **Segmentation Masks**: Per-pixel class labels (0 = background, 1 = class 1, ...)
- **Depth Maps**: Per-pixel distance in meters (16-bit PNG or float32 .npy)
- **Normals**: Surface orientation vectors

**COCO Format**:
- Industry-standard format for object detection
- Supported by PyTorch, TensorFlow, Detectron2
- Required fields: `images`, `annotations`, `categories`

## Further Reading

- **Isaac Sim Replicator Docs**: https://docs.omniverse.nvidia.com/isaacsim/latest/replicator_tutorials/index.html
- **USD Tutorial**: https://graphics.pixar.com/usd/docs/index.html
- **COCO Dataset Format**: https://cocodataset.org/#format-data
- **Nsight Graphics**: https://developer.nvidia.com/nsight-graphics
- **Domain Randomization Paper**: Tobin et al. (2017) "Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World"

## License

MIT License - See LICENSE file in repository root.

## Support

For issues, questions, or contributions:
- GitHub Issues: https://github.com/yourusername/robot-book-code/issues
- Book Website: https://physicalai-book.com
- NVIDIA Isaac Forum: https://forums.developer.nvidia.com/c/agx-autonomous-machines/isaac/
