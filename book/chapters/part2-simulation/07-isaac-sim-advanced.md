# Chapter 7: Isaac Sim Advanced - Domain Randomization, Synthetic Data, Replicator

**Learning Objectives**

By the end of this chapter, you will be able to:

1. Explain domain randomization and its role in closing the sim-to-real gap
2. Randomize visual properties (textures, colors, lighting) using USD and Python API
3. Randomize physics parameters (mass, friction, joint damping) for robust policies
4. Generate synthetic datasets with automatic annotations (bounding boxes, segmentation, depth)
5. Use Isaac Sim Replicator API to create procedural scenes at scale
6. Profile Isaac Sim performance to identify bottlenecks (rendering vs. physics vs. I/O)
7. Optimize scenes for training (LOD, culling, physics simplification)
8. Generate 1000+ labeled images for vision-based manipulation tasks

**Prerequisites**: Chapter 6 (Isaac Sim basics), Python programming, basic computer vision

**Estimated Time**: 12-14 hours (6 hours reading, 6-8 hours hands-on lab)

---

## 1. Domain Randomization for Sim-to-Real

### 1.1 What is Domain Randomization?

**Domain randomization** is a technique to train robust robot policies by exposing them to diverse simulated environments, reducing overfitting to specific simulation artifacts.

**The Sim-to-Real Gap Problem**:
- **Simulation**: Perfect lighting, no sensor noise, known object poses
- **Real World**: Variable lighting, sensor noise, uncertain object detection
- **Result**: Policies trained in simulation fail on real hardware

**Domain Randomization Solution**:
Instead of trying to make simulation perfectly match reality (impossible), randomize simulation to cover the space of possible realities.

**Randomization Dimensions**:

| Category | Parameters | Example Range |
|----------|-----------|---------------|
| **Visual** | Texture, color, lighting | 10 textures × 5 light intensities = 50 variations |
| **Physics** | Mass, friction, damping | Mass ± 20%, friction 0.5-1.5 |
| **Sensor** | Camera noise, IMU drift | Gaussian noise σ = 0.01-0.05 |
| **Geometry** | Object size, robot dimensions | Scale 0.9-1.1× |
| **Dynamics** | External forces, delays | Wind 0-5 N, latency 10-50ms |

**Example**: Training a grasping policy with domain randomization
- **Without DR**: 95% success in sim, 30% on real robot (overfits to sim lighting)
- **With DR**: 85% success in sim, 75% on real robot (robust to lighting variations)

**Key Papers**:
- **OpenAI (2018)**: Trained Dactyl hand to solve Rubik's cube using DR in simulation, deployed to real robot
- **NVIDIA (2021)**: Factory digital twin with DR achieves 90%+ sim-to-real transfer

### 1.2 Randomizing Visual Appearance: Textures, Colors, Lighting

**Texture Randomization**:

Isaac Sim supports dynamic texture replacement via USD Material API.

**Example** (Python script):

```python
from pxr import UsdShade, Sdf
import omni.isaac.core.utils.prims as prim_utils
import random

def randomize_texture(prim_path: str, texture_paths: list):
    """
    Randomize texture on a prim.

    Args:
        prim_path: USD path to prim (e.g., "/World/Table")
        texture_paths: List of texture file paths
    """
    prim = prim_utils.get_prim_at_path(prim_path)

    # Get or create material
    material_path = f"{prim_path}/Material"
    material = UsdShade.Material.Get(stage, material_path)
    if not material:
        material = UsdShade.Material.Define(stage, material_path)

    # Get shader
    shader = UsdShade.Shader.Define(stage, f"{material_path}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")

    # Randomize diffuse texture
    texture_path = random.choice(texture_paths)
    diffuse_texture = UsdShade.Shader.Define(stage, f"{material_path}/DiffuseTexture")
    diffuse_texture.CreateIdAttr("UsdUVTexture")
    diffuse_texture.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(texture_path)

    # Connect texture to shader
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        diffuse_texture.ConnectableAPI(), "rgb"
    )

    # Bind material to prim
    UsdShade.MaterialBindingAPI(prim).Bind(material)

# Usage
texture_library = [
    "omniverse://localhost/NVIDIA/Assets/Materials/Wood/WoodFloor_01.mdl",
    "omniverse://localhost/NVIDIA/Assets/Materials/Metal/BrushedAluminum.mdl",
    "omniverse://localhost/NVIDIA/Assets/Materials/Plastic/PlasticMatte.mdl",
]

randomize_texture("/World/Table", texture_library)
```

**Color Randomization** (simpler, faster):

```python
from pxr import UsdGeom, Gf
import numpy as np

def randomize_color(prim_path: str):
    """Randomize RGB color of a prim."""
    geom = UsdGeom.Mesh.Get(stage, prim_path)

    # Random RGB (0.2-0.9 to avoid pure black/white)
    color = Gf.Vec3f(*np.random.uniform(0.2, 0.9, 3))
    geom.GetDisplayColorAttr().Set([color])

# Usage
randomize_color("/World/Cube")
```

**Lighting Randomization**:

```python
def randomize_lighting(light_path: str):
    """Randomize intensity and color of light."""
    from pxr import UsdLux

    light = UsdLux.DistantLight.Get(stage, light_path)

    # Randomize intensity (500-2000)
    intensity = random.uniform(500, 2000)
    light.GetIntensityAttr().Set(intensity)

    # Randomize color temperature (3000K-6500K: warm to cool)
    temp = random.uniform(3000, 6500)
    light.GetColorTemperatureAttr().Set(temp)

    # Randomize direction (sun angle)
    angle_x = random.uniform(0, 90)
    angle_z = random.uniform(0, 360)
    # Apply rotation to light prim...

randomize_lighting("/World/Sun")
```

### 1.3 Randomizing Physics: Mass, Friction, Joint Damping

**Mass Randomization**:

```python
from pxr import UsdPhysics

def randomize_mass(prim_path: str, variance: float = 0.2):
    """
    Randomize mass of a rigid body by ±variance.

    Args:
        prim_path: USD path to rigid body
        variance: Fractional variance (0.2 = ±20%)
    """
    prim = prim_utils.get_prim_at_path(prim_path)
    mass_api = UsdPhysics.MassAPI(prim)

    # Get original mass
    original_mass = mass_api.GetMassAttr().Get()

    # Randomize (e.g., ±20%)
    scale = random.uniform(1 - variance, 1 + variance)
    new_mass = original_mass * scale

    mass_api.GetMassAttr().Set(new_mass)

# Usage
randomize_mass("/World/Humanoid/base_link", variance=0.15)
```

**Friction Randomization**:

```python
def randomize_friction(prim_path: str, mu_range: tuple = (0.5, 1.5)):
    """Randomize Coulomb friction coefficient."""
    prim = prim_utils.get_prim_at_path(prim_path)

    # Get or create physics material
    material_path = f"{prim_path}/PhysicsMaterial"
    physics_material = UsdPhysics.MaterialAPI.Get(stage, material_path)
    if not physics_material:
        physics_material = UsdPhysics.MaterialAPI.Apply(prim)

    # Randomize friction
    mu = random.uniform(*mu_range)
    physics_material.CreateStaticFrictionAttr(mu)
    physics_material.CreateDynamicFrictionAttr(mu * 0.95)  # Dynamic < static

randomize_friction("/World/groundPlane/Plane", mu_range=(0.8, 1.2))
```

**Joint Damping Randomization**:

```python
from pxr import PhysxSchema

def randomize_joint_damping(joint_path: str, damping_range: tuple = (100, 1000)):
    """Randomize joint damping coefficient."""
    joint_prim = prim_utils.get_prim_at_path(joint_path)

    # PhysX joint drive API
    drive_api = UsdPhysics.DriveAPI.Get(joint_prim, "angular")
    if not drive_api:
        drive_api = UsdPhysics.DriveAPI.Apply(joint_prim, "angular")

    # Randomize damping
    damping = random.uniform(*damping_range)
    drive_api.GetDampingAttr().Set(damping)

# Usage (apply to all robot joints)
for joint_name in ["left_hip_joint", "left_knee_joint", "right_hip_joint", "right_knee_joint"]:
    joint_path = f"/World/Humanoid/{joint_name}"
    randomize_joint_damping(joint_path, damping_range=(300, 700))
```

### 1.4 Randomizing Sensor Noise

**Camera Noise** (post-processing):

```python
import numpy as np
from PIL import Image

def add_camera_noise(image: np.ndarray, noise_std: float = 0.01):
    """
    Add Gaussian noise to RGB image.

    Args:
        image: HxWx3 numpy array (0-255 uint8)
        noise_std: Standard deviation (fraction of 255)

    Returns:
        Noisy image
    """
    noise = np.random.normal(0, noise_std * 255, image.shape)
    noisy_image = np.clip(image + noise, 0, 255).astype(np.uint8)
    return noisy_image

# Apply after capturing from Isaac Sim camera
rgb = get_camera_image()  # Returns numpy array
noisy_rgb = add_camera_noise(rgb, noise_std=0.02)
```

**IMU Noise** (in ActionGraph or Python):

```python
def add_imu_noise(accel: np.ndarray, gyro: np.ndarray):
    """
    Add realistic IMU noise (white noise + bias).

    Args:
        accel: 3D acceleration vector (m/s²)
        gyro: 3D angular velocity vector (rad/s)

    Returns:
        Noisy accel, noisy gyro
    """
    # Noise parameters (based on MPU-6050 datasheet)
    accel_noise_std = 0.017  # m/s²
    gyro_noise_std = 0.009  # rad/s (0.5 deg/s)
    accel_bias = np.random.normal(0, 0.05, 3)
    gyro_bias = np.random.normal(0, 0.001, 3)

    # Add noise
    noisy_accel = accel + np.random.normal(0, accel_noise_std, 3) + accel_bias
    noisy_gyro = gyro + np.random.normal(0, gyro_noise_std, 3) + gyro_bias

    return noisy_accel, noisy_gyro
```

---

## 2. Synthetic Data Generation

### 2.1 Why Synthetic Data for Robotics?

**Challenges with Real-World Data**:
- **Expensive**: $1000+ per hour for human annotators
- **Slow**: Weeks to label 10k images
- **Limited Diversity**: Hard to capture rare scenarios (failures, edge cases)
- **Privacy**: Cannot collect data in private spaces

**Synthetic Data Benefits**:
- **Free Annotations**: Segmentation, depth, normals auto-generated
- **Scalable**: Generate 1M images overnight
- **Diverse**: Procedural randomization covers infinite variations
- **Safe**: Simulate dangerous scenarios (collisions, drops)

**Use Cases**:
- **VLA Training**: OpenVLA, RT-2, Octo require 100k-1M diverse demonstrations
- **Object Detection**: Train YOLOv8 on synthetic grasping scenes
- **Depth Estimation**: Paired RGB-D data for monocular depth networks
- **Segmentation**: Instance masks for part-based manipulation

**Challenges**:
- **Domain Gap**: Synthetic images look "fake" (too perfect, wrong noise)
- **Solution**: Domain randomization + GAN-based refinement

### 2.2 Annotating Images: Bounding Boxes, Segmentation Masks, Depth Maps

Isaac Sim provides **automatic annotations** via Synthetic Data Sensors (SyntheticData extension).

**Annotation Types**:

| Annotation | Description | Format | Use Case |
|------------|-------------|--------|----------|
| **Bounding Box (2D)** | [x, y, w, h] rectangle | COCO JSON | Object detection |
| **Semantic Segmentation** | Per-pixel class ID | PNG (8-bit indexed) | Scene parsing |
| **Instance Segmentation** | Per-pixel instance ID | PNG (16-bit) | Part-based manipulation |
| **Depth** | Per-pixel distance (m) | NPY (float32) | 3D reconstruction |
| **Normal** | Per-pixel surface normal | PNG (RGB as XYZ) | Surface estimation |
| **Pose (3D)** | 6-DOF object pose | JSON (position + quaternion) | Grasping, tracking |

### 2.3 Isaac Sim's Built-In Annotation Tools

**Enable Synthetic Data Extension**:

```python
from omni.isaac.core.utils.extensions import enable_extension
enable_extension("omni.syntheticdata")
```

**Capture Annotated Data** (Python API):

```python
import omni.replicator.core as rep

# Create render product (camera view)
render_product = rep.create.render_product("/World/Camera", (640, 480))

# Attach annotators
rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
bbox_annotator = rep.AnnotatorRegistry.get_annotator("bounding_box_2d_tight")
sem_seg_annotator = rep.AnnotatorRegistry.get_annotator("semantic_segmentation")
depth_annotator = rep.AnnotatorRegistry.get_annotator("distance_to_camera")

# Attach to render product
rgb_annotator.attach([render_product])
bbox_annotator.attach([render_product])
sem_seg_annotator.attach([render_product])
depth_annotator.attach([render_product])

# Capture frame
rep.orchestrator.step()

# Get data
rgb = rgb_annotator.get_data()  # (H, W, 4) RGBA uint8
bboxes = bbox_annotator.get_data()  # List of {"semanticId": int, "x_min": float, ...}
sem_seg = sem_seg_annotator.get_data()  # (H, W, 4) class IDs in R channel
depth = depth_annotator.get_data()  # (H, W) float32 depth in meters

print(f"RGB shape: {rgb['data'].shape}")
print(f"Num bounding boxes: {len(bboxes['data'])}")
print(f"Depth range: {depth['data'].min():.2f}m to {depth['data'].max():.2f}m")
```

**Semantic Segmentation Setup** (assign class IDs):

```python
from pxr import Sdf
import omni.isaac.core.utils.semantics as semantics_utils

# Assign semantic class to objects
semantics_utils.add_update_semantics(
    prim=prim_utils.get_prim_at_path("/World/Table"),
    semantic_label="table",
    type_label="class"
)

semantics_utils.add_update_semantics(
    prim=prim_utils.get_prim_at_path("/World/Cube"),
    semantic_label="object",
    type_label="class"
)

# Isaac Sim will auto-assign class IDs:
# - "table" → ID 1 (red in visualization)
# - "object" → ID 2 (green in visualization)
```

---

## 3. Isaac Sim Replicator API

### 3.1 Scripting Procedural Scenes with Python

**Replicator** is Isaac Sim's framework for procedural scene generation at scale.

**Core Concepts**:

1. **Randomizers**: Functions that modify scene (e.g., `randomize_pose`)
2. **Triggers**: When to randomize (e.g., every frame, every N frames)
3. **Writers**: How to export data (e.g., COCO JSON, custom format)

**Basic Replicator Script**:

```python
import omni.replicator.core as rep

# Define randomization function
with rep.new_layer():
    # Create camera
    camera = rep.create.camera(position=(2, 2, 1), look_at=(0, 0, 0.5))

    # Create render product
    render_product = rep.create.render_product(camera, (640, 480))

    # Define what to randomize
    def randomize_scene():
        # Randomize cube position
        cube = rep.get.prims(path_pattern="/World/Cube")
        with cube:
            rep.modify.pose(
                position=rep.distribution.uniform((-0.5, -0.5, 0.5), (0.5, 0.5, 1.5)),
                rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360))
            )

        # Randomize lighting
        light = rep.get.prims(path_pattern="/World/Sun")
        with light:
            rep.modify.attribute(
                "inputs:intensity",
                rep.distribution.uniform(500, 2000)
            )

        return True  # Trigger successful

    # Register randomizer
    rep.randomizer.register(randomize_scene)

    # Trigger: randomize every frame
    with rep.trigger.on_frame():
        rep.randomizer.randomize_scene()

    # Writer: save to disk
    writer = rep.WriterRegistry.get("BasicWriter")
    writer.initialize(
        output_dir="_output_replicator",
        rgb=True,
        bounding_box_2d_tight=True,
        semantic_segmentation=True,
        distance_to_camera=True
    )
    writer.attach([render_product])

# Run replicator for 100 frames (generates 100 images)
rep.orchestrator.run()
for i in range(100):
    rep.orchestrator.step()
    print(f"Generated frame {i+1}/100")

rep.orchestrator.stop()
```

### 3.2 Randomizing Object Placement and Poses

**Scatter Objects on Table Surface**:

```python
import omni.replicator.core as rep

# Define table surface bounds
table_min = (-0.5, -0.3, 0.75)  # (x, y, z) min
table_max = (0.5, 0.3, 0.75)    # (x, y, z) max (same Z = tabletop)

# Get all graspable objects (assumed to have semantic label "object")
objects = rep.get.prims(semantics=[("class", "object")])

def scatter_objects():
    """Randomly place objects on table surface."""
    with objects:
        # Random XY position on table, fixed Z
        rep.modify.pose(
            position=rep.distribution.uniform(table_min, table_max),
            rotation=rep.distribution.uniform((0, 0, 0), (0, 0, 360))  # Only rotate around Z
        )
    return True

rep.randomizer.register(scatter_objects)
```

**Ensure No Collisions** (advanced, physics-based):

```python
def scatter_objects_physics():
    """
    Scatter objects and let them settle with physics.
    Ensures no interpenetration.
    """
    from omni.isaac.core import World

    # Randomize positions (above table)
    with objects:
        rep.modify.pose(
            position=rep.distribution.uniform(
                (table_min[0], table_min[1], 1.0),  # 0.25m above table
                (table_max[0], table_max[1], 1.5)
            ),
            rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360))
        )

    # Run physics for 60 frames (1 second at 60Hz) to let objects settle
    world = World.instance()
    for _ in range(60):
        world.step(render=False)  # Physics only, no rendering

    return True
```

### 3.3 Generating Large-Scale Datasets (1000+ images)

**Production Script** (generate 10k images):

```python
import omni.replicator.core as rep
from omni.isaac.kit import SimulationApp

# Headless mode for max throughput
simulation_app = SimulationApp({"headless": True})

# ... (scene setup, randomizer definitions) ...

# Writer configuration
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(
    output_dir="/data/grasping_dataset",
    rgb=True,
    bounding_box_2d_tight=True,
    instance_segmentation=True,
    semantic_segmentation=True,
    distance_to_camera=True,
    normals=True,
    file_name_prefix="grasp"
)

# Generate 10,000 images
NUM_FRAMES = 10000
rep.orchestrator.run()

for i in range(NUM_FRAMES):
    rep.orchestrator.step()

    if (i + 1) % 100 == 0:
        print(f"Progress: {i+1}/{NUM_FRAMES} ({(i+1)/NUM_FRAMES*100:.1f}%)")

rep.orchestrator.stop()
simulation_app.close()

print(f"Dataset generated: {NUM_FRAMES} images in /data/grasping_dataset")
```

**Performance**: RTX 4090 generates ~100 images/second (headless, 640×480 resolution).

### 3.4 Exporting Data in COCO, Pascal VOC, or Custom Formats

**COCO Format** (for object detection):

Isaac Sim's BasicWriter outputs bounding boxes as JSON. Convert to COCO:

```python
import json
import os
from pathlib import Path

def convert_to_coco(output_dir: str):
    """Convert BasicWriter output to COCO format."""

    # COCO structure
    coco_dataset = {
        "images": [],
        "annotations": [],
        "categories": [
            {"id": 1, "name": "cube"},
            {"id": 2, "name": "cylinder"},
            {"id": 3, "name": "sphere"}
        ]
    }

    annotation_id = 0

    # Read BasicWriter bbox JSON files
    bbox_dir = Path(output_dir) / "bounding_box_2d_tight"
    for i, bbox_file in enumerate(sorted(bbox_dir.glob("*.json"))):
        with open(bbox_file) as f:
            data = json.load(f)

        # Add image entry
        image_id = i
        coco_dataset["images"].append({
            "id": image_id,
            "file_name": f"rgb_{i:04d}.png",
            "width": 640,
            "height": 480
        })

        # Add annotations
        for bbox in data["data"]:
            semantic_id = bbox["semanticId"]
            x_min = bbox["x_min"]
            y_min = bbox["y_min"]
            x_max = bbox["x_max"]
            y_max = bbox["y_max"]

            width = x_max - x_min
            height = y_max - y_min
            area = width * height

            coco_dataset["annotations"].append({
                "id": annotation_id,
                "image_id": image_id,
                "category_id": semantic_id,
                "bbox": [x_min, y_min, width, height],  # COCO format: [x, y, w, h]
                "area": area,
                "iscrowd": 0
            })
            annotation_id += 1

    # Save COCO JSON
    with open(os.path.join(output_dir, "annotations.json"), "w") as f:
        json.dump(coco_dataset, f, indent=2)

    print(f"COCO dataset created: {len(coco_dataset['images'])} images, {len(coco_dataset['annotations'])} annotations")

# Usage
convert_to_coco("/data/grasping_dataset")
```

**Custom Writer** (for specialized formats):

```python
import omni.replicator.core as rep

class CustomWriter(rep.Writer):
    """Custom writer for specific data format."""

    def __init__(self, output_dir: str):
        self.output_dir = output_dir
        self.frame_id = 0

    def write(self, data: dict):
        """
        Called for each frame.

        Args:
            data: Dict with keys: "rgb", "bounding_box_2d_tight", etc.
        """
        # Extract RGB
        rgb = data["rgb"]["data"]  # (H, W, 4) RGBA

        # Extract bounding boxes
        bboxes = data["bounding_box_2d_tight"]["data"]

        # Custom format: save RGB as PNG, bboxes as TXT
        from PIL import Image
        Image.fromarray(rgb[:, :, :3]).save(f"{self.output_dir}/frame_{self.frame_id:05d}.png")

        with open(f"{self.output_dir}/frame_{self.frame_id:05d}.txt", "w") as f:
            for bbox in bboxes:
                # Format: class_id x_center y_center width height (YOLO format)
                class_id = bbox["semanticId"]
                x_center = (bbox["x_min"] + bbox["x_max"]) / 2 / 640  # Normalize
                y_center = (bbox["y_min"] + bbox["y_max"]) / 2 / 480
                width = (bbox["x_max"] - bbox["x_min"]) / 640
                height = (bbox["y_max"] - bbox["y_min"]) / 480
                f.write(f"{class_id} {x_center:.6f} {y_center:.6f} {width:.6f} {height:.6f}\n")

        self.frame_id += 1

# Register and use custom writer
rep.WriterRegistry.register(CustomWriter)
writer = CustomWriter(output_dir="/data/custom_dataset")
writer.attach([render_product])
```

---

## 4. Performance Profiling

### 4.1 Identifying Bottlenecks: Rendering vs. Physics vs. I/O

Isaac Sim's profiler shows time breakdown per frame.

**Enable Profiler**:

Menu: `Window` → `Profiler`

**Key Metrics**:

| Metric | Description | Target | Bottleneck If... |
|--------|-------------|--------|------------------|
| **Total Frame Time** | ms per frame | < 16.7ms (60 FPS) | > 33ms (< 30 FPS) |
| **Physics Time** | PhysX solve + collision | < 5ms | > 10ms → Simplify physics |
| **Render Time** | RTX ray tracing | < 10ms | > 20ms → Reduce resolution |
| **Python Time** | Script execution | < 1ms | > 5ms → Optimize Python code |
| **I/O Time** | Disk writes | < 1ms | > 10ms → Use SSD, batch writes |

**Interpret Profiler**:

```
Frame Time: 45ms (22 FPS) ← TOO SLOW
├─ Physics: 8ms (18%)
├─ Rendering: 30ms (67%) ← BOTTLENECK!
├─ Python: 5ms (11%)
└─ I/O: 2ms (4%)
```

**Solution**: Reduce rendering load (lower resolution, disable RTX, headless mode).

### 4.2 Using Nsight Graphics for GPU Profiling

**Nsight Graphics** is NVIDIA's GPU profiler for graphics applications.

**Install**:

```bash
# Download from NVIDIA website
wget https://developer.nvidia.com/downloads/assets/tools/secure/nsight-graphics/2024_1/nsight-graphics-linux-x64-2024.1.0.24068.deb

sudo dpkg -i nsight-graphics-linux-x64-2024.1.0.24068.deb
```

**Profile Isaac Sim**:

1. Launch Nsight Graphics: `nsight-gfx`
2. File → Connect
3. Application: `~/.local/share/ov/pkg/isaac-sim-4.2.0/isaac-sim.sh`
4. Start profiling
5. In Isaac Sim: Play simulation for 10 seconds
6. Nsight Graphics: Capture frame

**Analysis**:

- **GPU Timeline**: See which shaders take longest
- **Memory Usage**: Check VRAM allocation
- **Bottleneck**: Identify fragment shader vs. ray tracing vs. compute

**Common GPU Bottlenecks**:
- **Fragment Shader**: Too many pixels (reduce resolution)
- **Ray Tracing**: Complex materials (simplify shaders)
- **Compute**: PhysX on GPU (reduce num_robots or simplify collisions)

### 4.3 Memory Usage Analysis

**Monitor VRAM** (command line):

```bash
watch -n 0.5 nvidia-smi

# Output:
# GPU  Name        Temp  Memory-Usage
# 0    RTX 4080    65C   12GB / 16GB  ← 75% usage, still OK
```

**Python API** (in Isaac Sim script):

```python
import pynvml

pynvml.nvmlInit()
handle = pynvml.nvmlDeviceGetHandleByIndex(0)
mem_info = pynvml.nvmlDeviceGetMemoryInfo(handle)

print(f"VRAM Used: {mem_info.used / 1e9:.2f} GB")
print(f"VRAM Total: {mem_info.total / 1e9:.2f} GB")
print(f"VRAM Utilization: {mem_info.used / mem_info.total * 100:.1f}%")
```

**Reduce VRAM Usage**:

1. **Enable Instancing**: Share meshes across robots
   ```python
   enable_extension("omni.physx.flatcache")
   ```

2. **Reduce Texture Resolution**:
   ```python
   # In material, set maxTextureSize
   material.CreateInput("maxTextureSize", Sdf.ValueTypeNames.Int).Set(512)  # Was 2048
   ```

3. **Unload Unused Assets**:
   ```python
   from pxr import Usd
   stage = omni.usd.get_context().get_stage()
   stage.Unload("/World/UnusedModel")  # Free VRAM
   ```

---

## 5. Scene Optimization

### 5.1 Level of Detail (LOD) for Meshes

**LOD** reduces polygon count for distant objects.

**Create LOD Manually** (Blender workflow):

1. Export high-poly mesh: `humanoid_high.obj` (100k triangles)
2. Decimate in Blender:
   - Select mesh → Modifiers → Decimate
   - Ratio: 0.5 (50k triangles) → Export as `humanoid_mid.obj`
   - Ratio: 0.1 (10k triangles) → Export as `humanoid_low.obj`

3. In Isaac Sim, swap meshes based on distance:
   ```python
   def update_lod(camera_pos, object_pos):
       """Swap mesh based on distance to camera."""
       distance = np.linalg.norm(camera_pos - object_pos)

       if distance < 5:
           mesh_path = "humanoid_high.obj"
       elif distance < 20:
           mesh_path = "humanoid_mid.obj"
       else:
           mesh_path = "humanoid_low.obj"

       # Update mesh reference
       # (code omitted for brevity, use USD API to swap mesh)
   ```

### 5.2 Occlusion Culling and Frustum Culling

**Frustum Culling** (automatic in Isaac Sim):
- Objects outside camera view are not rendered
- No action needed (enabled by default)

**Occlusion Culling** (objects hidden behind others):
- Not automatic in Isaac Sim (ray tracing handles this)
- For rasterization: Manually disable rendering for hidden objects

**Example** (disable robots not in camera view):

```python
from pxr import Usd

def cull_out_of_view_robots(camera_prim_path: str, robot_paths: list):
    """Disable rendering for robots outside camera frustum."""
    camera_prim = prim_utils.get_prim_at_path(camera_prim_path)
    # ... (compute camera frustum, test bounding boxes)

    for robot_path in robot_paths:
        if not in_frustum(robot_path):
            robot_prim = prim_utils.get_prim_at_path(robot_path)
            imageable = UsdGeom.Imageable(robot_prim)
            imageable.MakeInvisible()  # Skip rendering
```

### 5.3 Shader Complexity Reduction

**Disable Expensive Effects**:

```python
# Disable ambient occlusion (expensive)
import carb.settings
settings = carb.settings.get_settings()
settings.set("/rtx/post/aa/op", 0)  # Disable anti-aliasing
settings.set("/rtx/ambientOcclusion/enabled", False)
settings.set("/rtx/reflections/enabled", False)  # Disable reflections
settings.set("/rtx/translucency/enabled", False)
```

**Use Simpler Materials**:

```python
# Replace PBR material with basic diffuse
from pxr import UsdShade, Sdf

def simplify_material(prim_path: str):
    """Replace complex material with simple diffuse."""
    material = UsdShade.Material.Get(stage, f"{prim_path}/Material")
    shader = UsdShade.Shader.Get(stage, f"{prim_path}/Material/Shader")

    # Clear all inputs (removes metallic, roughness, etc.)
    for input_name in shader.GetInputs():
        shader.GetInput(input_name).ClearConnections()

    # Keep only diffuse color
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set((0.8, 0.8, 0.8))
```

### 5.4 Physics Simplification: Convex Decomposition

**Problem**: Complex concave meshes require many collision primitives (slow).

**Solution**: Approximate with convex hulls.

**URDF Importer** (automatic convex decomposition):

```python
from omni.isaac.urdf import _urdf

import_config = _urdf.ImportConfig()
import_config.convex_decomp = True  # Enable V-HACD decomposition
import_config.convex_decomp_params = {
    "max_hulls": 16,  # Max convex hulls per mesh (fewer = faster)
    "resolution": 100000  # Voxel resolution (lower = faster, less accurate)
}

urdf_interface.parse_urdf(urdf_path, "/World/Robot", import_config)
```

**Manual Replacement** (use primitives):

```python
# Replace complex foot mesh with simple box
foot_collision_path = "/World/Humanoid/left_foot/collisions"
prim_utils.delete_prim(foot_collision_path)

# Create box collision
from pxr import UsdGeom, UsdPhysics
collision_prim = UsdGeom.Cube.Define(stage, foot_collision_path)
collision_prim.GetSizeAttr().Set(0.15)  # 15cm box
UsdPhysics.CollisionAPI.Apply(collision_prim.GetPrim())
```

---

## 6. Hands-On Lab: Randomized Grasping Dataset

**Objective**: Generate 1000 labeled images of humanoid grasping random objects.

**Time**: 6-8 hours

### Lab Setup

**Prerequisites**:
- Isaac Sim 2024.2 installed
- Chapter 6 humanoid scene
- 10-20 graspable object meshes (cubes, cylinders, spheres)

### Step 1: Scene Creation (2 hours)

1. Launch Isaac Sim
2. Create scene:
   - Ground plane
   - Table (0.75m height, 1m × 0.6m surface)
   - Humanoid positioned 0.5m from table
   - 20 small objects (0.05-0.15m size) on table
3. Add camera:
   - Position: (1.5, 0, 1.2)
   - Look at: table center
   - Resolution: 640 × 480
4. Assign semantic labels:
   - Table: "table"
   - Objects: "cube", "cylinder", "sphere" (by geometry)
   - Humanoid: "robot"

### Step 2: Replicator Script (2 hours)

**File**: `generate_grasping_dataset.py`

```python
import omni.replicator.core as rep
from omni.isaac.kit import SimulationApp

# Launch headless
simulation_app = SimulationApp({"headless": True})

import numpy as np

# ... (load scene from Step 1) ...

# Camera and render product
camera = rep.create.camera(position=(1.5, 0, 1.2), look_at=(0, 0, 0.8))
render_product = rep.create.render_product(camera, (640, 480))

# Get objects to randomize
objects = rep.get.prims(semantics=[("class", "cube"), ("class", "cylinder"), ("class", "sphere")])
table = rep.get.prims(path_pattern="/World/Table")
lights = rep.get.prims(semantics=[("class", "light")])

def randomize_grasping_scene():
    """Randomize object poses, textures, and lighting."""

    # Scatter objects on table (physics-based)
    with objects:
        rep.modify.pose(
            position=rep.distribution.uniform((-0.4, -0.25, 0.9), (0.4, 0.25, 1.2)),
            rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360))
        )

    # Let objects settle with physics
    from omni.isaac.core import World
    world = World.instance()
    for _ in range(120):  # 2 seconds at 60Hz
        world.step(render=False)

    # Randomize table texture
    with table:
        rep.randomizer.texture(
            textures=[
                "omniverse://localhost/NVIDIA/Assets/Materials/Wood/WoodFloor_01.mdl",
                "omniverse://localhost/NVIDIA/Assets/Materials/Metal/BrushedAluminum.mdl",
            ]
        )

    # Randomize lighting
    with lights:
        rep.modify.attribute("inputs:intensity", rep.distribution.uniform(500, 2500))
        rep.modify.attribute("inputs:color", rep.distribution.uniform((0.8, 0.8, 0.8), (1.0, 1.0, 1.0)))

    return True

rep.randomizer.register(randomize_grasping_scene)

# Trigger
with rep.trigger.on_frame():
    rep.randomizer.randomize_grasping_scene()

# Writer
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(
    output_dir="/data/grasping_1k",
    rgb=True,
    bounding_box_2d_tight=True,
    instance_segmentation=True,
    semantic_segmentation=True,
    distance_to_camera=True
)
writer.attach([render_product])

# Generate 1000 frames
NUM_FRAMES = 1000
rep.orchestrator.run()

for i in range(NUM_FRAMES):
    rep.orchestrator.step()
    if (i + 1) % 100 == 0:
        print(f"Generated {i+1}/{NUM_FRAMES} images")

rep.orchestrator.stop()
simulation_app.close()
```

### Step 3: Run and Verify (30 minutes)

```bash
cd ~/.local/share/ov/pkg/isaac-sim-4.2.0
./python.sh ~/path/to/generate_grasping_dataset.py

# Wait ~30-60 minutes (RTX 4080: ~30 images/second)
# Output: /data/grasping_1k/ with 1000 images + annotations
```

**Verify Dataset**:

```bash
ls /data/grasping_1k/
# rgb/               # 1000 PNG images
# bounding_box_2d_tight/  # 1000 JSON files
# semantic_segmentation/  # 1000 PNG masks
# distance_to_camera/     # 1000 NPY depth maps
```

### Step 4: Convert to COCO (1 hour)

Run conversion script from Section 3.4.

### Step 5: Visualize Dataset (30 minutes)

**Script**: `visualize_dataset.py`

```python
import json
import cv2
import numpy as np
from pathlib import Path

dataset_dir = Path("/data/grasping_1k")

# Load COCO annotations
with open(dataset_dir / "annotations.json") as f:
    coco = json.load(f)

# Visualize first 10 images
for i in range(10):
    # Load RGB
    img_path = dataset_dir / "rgb" / f"rgb_{i:04d}.png"
    img = cv2.imread(str(img_path))

    # Find annotations for this image
    img_id = coco["images"][i]["id"]
    annotations = [ann for ann in coco["annotations"] if ann["image_id"] == img_id]

    # Draw bounding boxes
    for ann in annotations:
        x, y, w, h = ann["bbox"]
        cat_id = ann["category_id"]
        cat_name = next(cat["name"] for cat in coco["categories"] if cat["id"] == cat_id)

        cv2.rectangle(img, (int(x), int(y)), (int(x+w), int(y+h)), (0, 255, 0), 2)
        cv2.putText(img, cat_name, (int(x), int(y)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Display
    cv2.imshow(f"Image {i}", img)
    cv2.waitKey(1000)  # 1 second per image

cv2.destroyAllWindows()
```

---

## 7. End-of-Chapter Project: Multi-Robot Assembly Dataset

**Objective**: Generate dataset of 2 humanoids collaboratively assembling objects (sim-to-real transfer for multi-agent manipulation).

**Requirements**:

1. **Scene**:
   - 2 humanoids facing each other across table
   - 20 assembly parts (blocks, connectors)
   - Random part placements

2. **Randomization**:
   - Robot arm joint positions (±20°)
   - Part colors (10 color variations)
   - Lighting (day/night simulation)
   - Camera viewpoint (5 viewpoints around table)

3. **Annotations**:
   - 2D bounding boxes for all parts
   - 6-DOF poses for parts (for grasping)
   - Robot joint states (for imitation learning)

4. **Dataset Size**: 5000 images (1 hour generation on RTX 4090)

5. **Output**: COCO JSON + custom robot state JSON

**Starter Code**: Extend `generate_grasping_dataset.py` with second humanoid and 6-DOF pose annotations.

---

## Summary

In this chapter, you learned:

1. **Domain Randomization**: Randomize visual, physics, and sensor properties to close sim-to-real gap
2. **Synthetic Data**: Generate labeled datasets (bboxes, segmentation, depth) automatically
3. **Replicator API**: Procedural scene generation at scale (1000+ images in minutes)
4. **Performance Profiling**: Identify bottlenecks (rendering vs. physics vs. I/O)
5. **Scene Optimization**: LOD, culling, shader simplification, physics approximation
6. **Hands-On Skills**: Generated 1000-image grasping dataset with full annotations

**Key Takeaways**:

- Domain randomization is **essential** for sim-to-real transfer (OpenAI Dactyl, NVIDIA factory twins)
- Isaac Sim's Replicator API enables **industrial-scale** synthetic data generation
- Performance tuning is **critical**: 10× speedup possible with proper optimization
- Always profile **before** optimizing: Don't guess bottlenecks

**Next Chapter Preview**: Chapter 8 covers Simulation Benchmarking—comparing Gazebo, Isaac Sim, MuJoCo across fidelity, performance, and sim-to-real metrics.

---

## Further Reading

**Domain Randomization**:
- OpenAI Dactyl Paper: https://arxiv.org/abs/1808.00177
- Domain Randomization Survey: https://arxiv.org/abs/1910.07113

**Synthetic Data**:
- NVIDIA Synthetic Data Blog: https://developer.nvidia.com/blog/synthetic-data-generation/
- COCO Dataset Format: https://cocodataset.org/#format-data

**Replicator API**:
- Isaac Sim Replicator Docs: https://docs.omniverse.nvidia.com/extensions/latest/ext_replicator.html
- Replicator Examples: https://github.com/NVIDIA-Omniverse/synthetic-data-examples

**Performance Optimization**:
- Nsight Graphics: https://developer.nvidia.com/nsight-graphics
- USD Performance Best Practices: https://graphics.pixar.com/usd/docs/USD-Glossary.html

**Research Papers**:
- RT-2 (Google): https://robotics-transformer2.github.io/
- OpenVLA (Stanford): https://openvla.github.io/
- Isaac Gym (NVIDIA): https://arxiv.org/abs/2108.10470

---

**End of Chapter 7**
