# Chapter 6: NVIDIA Isaac Sim - Introduction and Setup

**Learning Objectives**

By the end of this chapter, you will be able to:

1. Explain Isaac Sim's role in Physical AI and how it differs from Gazebo
2. Install Isaac Sim 2024.2 via Omniverse Launcher and validate GPU drivers
3. Navigate the Isaac Sim interface (Stage, Prims, Layers, ActionGraph)
4. Import URDF files as USD and configure PhysX articulation parameters
5. Create simulation scenes with ground planes, lighting, and humanoid robots
6. Set up Isaac ROS bridge to publish sensor data (camera, IMU, joint states) to ROS 2
7. Benchmark simulation performance across Budget/Mid/Premium hardware tiers
8. Debug common issues (GPU out of memory, slow rendering, missing ROS 2 topics)

**Prerequisites**: Chapter 4 (URDF), Chapter 5 (Gazebo basics), RTX GPU (3070+ recommended), 32GB RAM

**Estimated Time**: 10-12 hours (6 hours reading, 4-6 hours hands-on lab)

---

## 1. What is Isaac Sim?

### 1.1 Isaac Sim vs. Gazebo: Key Differences

**NVIDIA Isaac Sim** is a GPU-accelerated robotics simulator built on the Omniverse platform, designed for **Physical AI** workflows—training VLA models, synthetic data generation, and sim-to-real transfer at scale.

| Aspect | Gazebo Harmonic | Isaac Sim 2024.2 |
|--------|----------------|------------------|
| **Physics Engine** | CPU-based (ODE/Bullet/DART) | GPU-accelerated PhysX 5.x |
| **Parallelization** | Single environment | 1000+ parallel environments on single GPU |
| **Rendering** | Ogre 2.x (rasterization) | RTX ray tracing, path tracing (photorealistic) |
| **Synthetic Data** | Basic RGB/depth | Physically-based camera, RTX LiDAR, segmentation masks |
| **Domain Randomization** | Manual scripting | Built-in Replicator API (textures, lighting, poses) |
| **File Format** | SDF (XML-based) | USD (Universal Scene Description) |
| **Primary Use Case** | ROS 2 prototyping, classical robotics | VLA training, sim-to-real transfer, large-scale RL |
| **Hardware Requirements** | CPU: 4+ cores, GPU: Optional | GPU: RTX 3070+ (8GB VRAM), RAM: 32GB |
| **License** | Apache 2.0 (Open Source) | Free for non-commercial, subscription for commercial |
| **Learning Curve** | Moderate (well-documented) | Steep (Omniverse ecosystem, USD, Python API) |

**When Isaac Sim is Essential**:
- ✅ **VLA Training**: Need 1000+ parallel environments for OpenVLA, RT-2, Octo training
- ✅ **Photorealistic Rendering**: Vision models require accurate lighting, shadows, reflections
- ✅ **Sim-to-Real Transfer**: PhysX contact fidelity reduces reality gap
- ✅ **Synthetic Dataset Generation**: Automated labeling (segmentation, depth, normals)
- ✅ **GPU Cluster**: Multi-node scaling (AWS G5, Azure NC-series)

**When Gazebo is Sufficient**:
- ✅ **ROS 2 Prototyping**: Fast iteration, open-source, CPU-friendly
- ✅ **Classical Robotics**: Path planning, SLAM, no vision-based learning
- ✅ **Budget Hardware**: No RTX GPU available

**Hybrid Workflow** (Recommended):
1. **Prototype** in Gazebo (faster iteration)
2. **Scale** to Isaac Sim for VLA training (GPU parallelization)
3. **Deploy** to Jetson Orin with trained models

### 1.2 Omniverse Platform and USD Format

**NVIDIA Omniverse** is a 3D collaboration platform built on **USD (Universal Scene Description)**—Pixar's open standard for 3D scenes used in film VFX pipelines.

**Omniverse Components**:
- **Nucleus Server**: Central scene database (like Git for 3D scenes)
- **Connectors**: Plugins for Blender, Maya, Unreal Engine, Rhino
- **Isaac Sim**: Robotics simulation app within Omniverse
- **Create**: 3D scene editor (like Unity Editor)
- **Code**: USD Python scripting environment

**USD Benefits for Robotics**:
1. **Layering**: Non-destructive edits (base scene + randomization layer)
2. **Composition**: Reference models from external files (like Xacro includes)
3. **Streaming**: Load massive scenes on-demand (100+ robots in single scene)
4. **Industry Standard**: Interoperability with CAD, rendering, simulation tools

**USD File Types**:
- `.usd`: ASCII or binary container
- `.usda`: ASCII (human-readable, good for version control)
- `.usdc`: Binary (compact, faster loading)
- `.usdz`: Archive (single file with dependencies, iOS AR support)

**Example USD** (simplified):

```usda
#usda 1.0
(
    defaultPrim = "World"
    upAxis = "Z"
)

def Xform "World"
{
    def Sphere "Ball"
    {
        double radius = 0.1
        float3 xformOp:translate = (0, 0, 1.5)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }

    def Xform "Humanoid" (
        references = @./humanoid.usd@
    )
    {
        float3 xformOp:translate = (0, 0, 1.0)
        uniform token[] xformOpOrder = ["xformOp:translate"]
    }
}
```

### 1.3 PhysX 5.x GPU-Accelerated Physics

**PhysX 5.x** (NVIDIA's physics engine) runs entirely on GPU, enabling massive parallelization:

**Key Features**:
- **Temporal Gauss-Seidel (TGS) Solver**: Faster convergence than Projected Gauss-Seidel
- **GPU Broad-Phase**: 1000+ robots with collision detection on single GPU
- **Soft Body Physics**: Deformable objects (clothes, cables, flesh)
- **Particle Systems**: Fluids, granular materials (sand, rice)
- **Articulations**: Reduced-coordinate formulation for humanoids (faster than Featherstone)

**Performance Comparison** (RTX 4080, 12-DOF humanoid):

| Simulator | Physics Engine | Environments | FPS | Speedup |
|-----------|---------------|--------------|-----|---------|
| Gazebo (DART) | CPU | 1 | 60 | 1× |
| Isaac Sim (PhysX) | GPU | 1 | 120 | 2× |
| Isaac Sim (PhysX) | GPU | 100 | 100 | 167× |
| Isaac Sim (PhysX) | GPU | 1000 | 50 | 833× |

**Critical for VLA Training**: 1000+ parallel environments mean 1000× faster data collection than real-time.

### 1.4 RTX Rendering and Synthetic Data Generation

Isaac Sim uses **RTX ray tracing** for physically-accurate rendering:

**Rendering Modes**:
1. **Real-Time (RTX Interactive)**: ~30-60 FPS, approximate global illumination
2. **Path Traced**: ~1-10 FPS, physically-correct (for dataset generation)
3. **Headless**: No rendering, max physics throughput (for RL training)

**Synthetic Data Features**:
- **Physically-Based Camera**: Lens distortion, chromatic aberration, motion blur
- **RTX LiDAR**: Multi-bounce ray tracing (not just depth map)
- **Automatic Annotations**: Segmentation masks, bounding boxes, depth, normals
- **Domain Randomization**: Textures, lighting, object poses, physics parameters

**Example**: Generate 1M labeled images for grasping in 10 hours (100 envs × 10k steps × 10 FPS).

---

## 2. Installation and Setup

### 2.1 System Requirements

**Minimum** (Budget Tier):
- **GPU**: NVIDIA RTX 4070 (12GB VRAM)
- **CPU**: 8-core (Intel i7-12700 or AMD Ryzen 7 5800X)
- **RAM**: 32GB DDR4
- **Storage**: 100GB SSD (50GB for Isaac Sim + 50GB for cache)
- **OS**: Ubuntu 22.04 LTS

**Recommended** (Mid Tier):
- **GPU**: NVIDIA RTX 4080 (16GB VRAM)
- **CPU**: 12-core (Intel i9-13900K or AMD Ryzen 9 7900X)
- **RAM**: 64GB DDR5
- **Storage**: 500GB NVMe SSD
- **OS**: Ubuntu 22.04 LTS

**High-End** (Premium Tier):
- **GPU**: NVIDIA RTX 4090 (24GB VRAM) or A6000 (48GB)
- **CPU**: 16-core (Intel i9-14900K or AMD Threadripper)
- **RAM**: 128GB DDR5
- **Storage**: 1TB NVMe SSD
- **OS**: Ubuntu 22.04 LTS

### 2.2 Installing NVIDIA Drivers

**Check Current Driver**:

```bash
nvidia-smi
# If output shows "NVIDIA-SMI has failed...", no driver installed
# If output shows driver < 550, update required
```

**Install Latest Driver** (550+ required for Isaac Sim 2024.2):

```bash
# Add NVIDIA PPA
sudo add-apt-repository ppa:graphics-drivers/ppa
sudo apt update

# Install driver 550
sudo apt install nvidia-driver-550

# Reboot (REQUIRED)
sudo reboot

# Verify
nvidia-smi
# Expected:
# Driver Version: 550.xx.xx
# CUDA Version: 12.4
```

**Install CUDA Toolkit** (for custom extensions):

```bash
# Download CUDA 12.4 installer
wget https://developer.download.nvidia.com/compute/cuda/12.4.0/local_installers/cuda_12.4.0_550.54.14_linux.run

# Run installer
sudo sh cuda_12.4.0_550.54.14_linux.run

# Add to PATH (~/.bashrc)
export PATH=/usr/local/cuda-12.4/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda-12.4/lib64:$LD_LIBRARY_PATH

# Verify
nvcc --version
# Expected: release 12.4
```

### 2.3 Installing Isaac Sim via Omniverse Launcher

**Step 1: Download Omniverse Launcher**

```bash
# Download AppImage
wget https://install.launcher.omniverse.nvidia.com/installers/omniverse-launcher-linux.AppImage

# Make executable
chmod +x omniverse-launcher-linux.AppImage

# Run launcher
./omniverse-launcher-linux.AppImage
```

**Step 2: Create NVIDIA Account**
- Click "Sign In" in Omniverse Launcher
- Create account or use existing NVIDIA Developer account
- Accept Omniverse EULA

**Step 3: Install Isaac Sim 2024.2**

1. In Launcher, click "Exchange" tab
2. Search for "Isaac Sim"
3. Click "Isaac Sim 2024.2.0" → "Install"
4. Choose installation directory: `~/.local/share/ov/pkg/isaac-sim-4.2.0/`
5. Wait for download (~15GB, takes 10-30 minutes)

**Installation Directory Structure**:

```
~/.local/share/ov/pkg/isaac-sim-4.2.0/
├── isaac-sim.sh           # Launch script
├── python.sh              # Python environment with Isaac Sim packages
├── kit/                   # Omniverse Kit core
├── exts/                  # Isaac Sim extensions
├── extscache/             # Extension cache
└── apps/                  # Application configs
```

**Step 4: First Launch**

```bash
cd ~/.local/share/ov/pkg/isaac-sim-4.2.0
./isaac-sim.sh
```

**Expected Behavior**:
- Splash screen appears (~10 seconds)
- Isaac Sim main window opens with Welcome screen
- First launch: Shader compilation (~2-5 minutes on first run)
- FPS counter in top-right should show ~60 FPS (empty scene)

**Common Installation Issues**:

1. **Error: "Failed to load libcuda.so"**
   - **Cause**: NVIDIA driver not installed
   - **Fix**: Install driver 550+ (see Section 2.2)

2. **Error: "Unsupported GPU"**
   - **Cause**: Non-RTX GPU (e.g., GTX 1660)
   - **Fix**: Isaac Sim requires RTX 2000+ series (Tensor Cores needed)

3. **Launcher shows "Installation Failed"**
   - **Cause**: Insufficient disk space
   - **Fix**: Free up 100GB on SSD

### 2.4 Installing Isaac ROS Packages

Isaac Sim integrates with ROS 2 via **Isaac ROS** packages:

```bash
# Source ROS 2
source /opt/ros/iron/setup.bash  # or jazzy

# Install Isaac ROS apt repository
sudo apt update
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update

# Install Isaac ROS Bridge
sudo apt install ros-${ROS_DISTRO}-isaac-ros-nitros \
                 ros-${ROS_DISTRO}-isaac-ros-nitros-bridge-ros2

# Install Isaac ROS Common packages
sudo apt install ros-${ROS_DISTRO}-isaac-ros-common

# Verify installation
ros2 pkg list | grep isaac_ros
# Expected:
# isaac_ros_nitros
# isaac_ros_nitros_bridge_ros2
# isaac_ros_common
```

**Alternative: Docker Installation** (recommended for reproducibility):

```bash
# Pull Isaac ROS Docker image
docker pull nvcr.io/nvidia/isaac-ros:isaac-sim-2024.2.0

# Run container with GPU support
docker run --gpus all -it --network host \
  -v ~/ros2_ws:/workspaces/isaac_ros-dev/src \
  nvcr.io/nvidia/isaac-ros:isaac-sim-2024.2.0 \
  /bin/bash
```

---

## 3. Isaac Sim UI: Layers, Stage, Prims

### 3.1 Understanding USD Terminology

**Stage**: The root container for your entire scene (like a Gazebo world)
- Contains all Prims (primitives/objects)
- Managed by USD Stage API
- Can save/load as `.usd` file

**Prim** (Primitive): An object in the scene
- Types: Mesh (geometry), Light, Camera, Xform (transform), Physics Articulation
- Hierarchy: Parent-child relationships (like URDF links/joints)
- Example: `/World/Humanoid/base_link` (path to base_link Prim)

**Layer**: A non-destructive edit layer (like Git branches)
- **Base Layer**: Original scene
- **Session Layer**: Temporary edits (not saved)
- **Sublayers**: Stacked edits (e.g., base + randomization + lighting)

**Attributes**: Properties of a Prim
- Transform: `xformOp:translate`, `xformOp:rotateXYZ`, `xformOp:scale`
- Physics: `physics:mass`, `physics:collisionEnabled`
- Rendering: `primvars:displayColor`

**Relationships**: Connections between Prims
- Material binding: Mesh → Material
- Physics articulation: Link → Joint

### 3.2 Navigating the Isaac Sim Interface

**Main UI Panels** (default layout):

```
┌─────────────────────────────────────────────────────────────┐
│  File  Edit  Create  Window  Isaac Sim  Help          [FPS] │
├───────────┬─────────────────────────────────┬───────────────┤
│           │                                 │               │
│  Stage    │      Viewport                   │  Property     │
│  (Prims)  │      (3D View)                  │  (Attributes) │
│           │                                 │               │
│  /World   │                                 │  Transform    │
│    Camera │                                 │  Physics      │
│    Light  │                                 │  Rendering    │
│           │                                 │               │
├───────────┴─────────────────────────────────┴───────────────┤
│  Content Browser            │  Console / Script Editor      │
│  (Assets)                   │  (Python Output)              │
└─────────────────────────────┴───────────────────────────────┘
```

**Key Panels**:

1. **Stage (Left)**:
   - Hierarchical tree of all Prims
   - Right-click → Create Prim, Delete, Duplicate
   - Visibility toggle (eye icon)

2. **Viewport (Center)**:
   - 3D scene view
   - Navigation:
     - **Orbit**: Middle mouse button
     - **Pan**: Shift + Middle mouse
     - **Zoom**: Scroll wheel
     - **Frame Selected**: F key
   - Render modes: Lit, Unlit, Wireframe

3. **Property (Right)**:
   - Attributes of selected Prim
   - Transform (position, rotation, scale)
   - Physics properties (mass, inertia, collision)
   - Material properties (color, metallic, roughness)

4. **Content Browser (Bottom-Left)**:
   - Asset library (meshes, materials, robots)
   - NVIDIA assets: `omniverse://localhost/NVIDIA/Assets/Isaac/4.2/`

5. **Console (Bottom-Right)**:
   - Python script output
   - Error messages (red text)
   - Physics stats

**Viewport Controls** (shortcuts):

| Action | Shortcut | Description |
|--------|----------|-------------|
| **Select** | Left click | Select Prim |
| **Multi-select** | Ctrl + Left click | Add to selection |
| **Move** | W | Translate gizmo |
| **Rotate** | E | Rotate gizmo |
| **Scale** | R | Scale gizmo |
| **Frame** | F | Focus camera on selected Prim |
| **Play** | Space | Start/stop physics simulation |
| **Reset** | Backspace | Reset to initial state |

### 3.3 Creating Prims: Meshes, Lights, Cameras

**Create Ground Plane**:

1. Menu: `Create` → `Physics` → `Ground Plane`
2. Automatically creates `/World/groundPlane` Prim
3. Default: 100m × 100m plane at Z=0

**Create Box** (manual):

1. Menu: `Create` → `Mesh` → `Cube`
2. Creates `/World/Cube` Prim
3. In Property panel:
   - Transform → `Translate`: (0, 0, 0.5)
   - Transform → `Scale`: (1, 1, 1)
4. To add physics:
   - Right-click `/World/Cube` → `Add` → `Physics` → `Rigid Body Preset`
   - Sets mass, collision, etc.

**Create Light** (directional, like sun):

1. Menu: `Create` → `Light` → `Distant Light`
2. Creates `/World/DistantLight`
3. In Property panel:
   - Light → `Intensity`: 3000
   - Light → `Angle`: 1.0
   - Transform → `Rotate`: (315, 45, 0) [X, Y, Z degrees]

**Create Camera**:

1. Menu: `Create` → `Camera`
2. Creates `/World/Camera`
3. Position camera (e.g., looking at humanoid):
   - Transform → `Translate`: (3, 3, 2)
   - Transform → `Rotate`: (-10, 0, 135)
4. Switch viewport to camera view:
   - Viewport top-left dropdown → `/World/Camera`

**Delete Prim**:
- Right-click Prim in Stage → `Delete`
- Or select Prim → `Delete` key

---

## 4. Creating Your First Scene

### 4.1 Adding Ground Plane and Lighting

**Step 1: Create New Scene**

```bash
# Launch Isaac Sim
cd ~/.local/share/ov/pkg/isaac-sim-4.2.0
./isaac-sim.sh
```

- File → New (or Ctrl+N)
- Creates empty scene with default camera

**Step 2: Add Ground Plane**

- Menu: `Create` → `Physics` → `Ground Plane`
- Default friction: 1.0 (good for humanoid feet)
- Default restitution: 0.0 (no bounce)

**Step 3: Add Lighting**

Option A: **Dome Light** (HDR environment)

1. Menu: `Create` → `Light` → `Dome Light`
2. Property panel → Light → `Texture`: Browse to `omniverse://localhost/NVIDIA/Assets/Skies/Indoor/ZetoCG_com_WarehouseInterior2b.hdr`
3. Intensity: 1000

Option B: **Directional Light** (Sun)

1. Menu: `Create` → `Light` → `Distant Light`
2. Intensity: 3000
3. Angle: 0.5 (soft shadows)
4. Rotate: (315, 45, 0) to simulate morning sun

**Step 4: Configure Physics Scene**

1. Menu: `Window` → `Physics` → `Scene Manager`
2. Physics Scene settings:
   - Gravity: (0, 0, -9.81)
   - Timestep: 0.01667 (60 Hz, 1/60 seconds)
   - Solver iterations: 4 (default, increase for accuracy)
   - Bounce threshold: 0.2 m/s

### 4.2 Importing Humanoid URDF as USD

Isaac Sim includes URDF importer extension.

**Step 1: Activate URDF Importer**

- Menu: `Window` → `Extensions`
- Search: "URDF"
- Enable: `omni.isaac.urdf_importer`

**Step 2: Import URDF**

- Menu: `Isaac Utils` → `Workflows` → `URDF Importer`
- URDF Importer window appears

**Step 3: Configure Import Settings**

| Setting | Value | Description |
|---------|-------|-------------|
| **Input File** | `/path/to/humanoid_12dof.urdf` | URDF file from Chapter 4 |
| **Import Inertia Tensor** | ✅ | Use URDF inertia (recommended) |
| **Fix Base Link** | ❌ | Allow humanoid to move freely |
| **Import Inertia Diagonal** | ✅ | Diagonal inertia only (faster) |
| **Joint Drive Type** | Position | Position control (vs. Velocity) |
| **Joint Drive Strength** | 10000 | PD controller stiffness |
| **Joint Drive Damping** | 1000 | PD controller damping |
| **Self-Collision** | ❌ | Disable for performance (enable for complex manipulation) |
| **Create Physics Scene** | ✅ | Auto-create PhysX scene if not exists |

**Step 4: Import**

- Click `Import`
- Wait 5-10 seconds (USD conversion, physics setup)
- Humanoid appears in scene at (0, 0, 0)
- Stage panel shows `/World/humanoid_12dof` with child Prims (links, joints)

**Step 5: Position Humanoid Above Ground**

- Select `/World/humanoid_12dof` in Stage
- Property panel → Transform → `Translate Z`: 1.0
- This places humanoid 1m above ground (feet should be at Z=0)

### 4.3 Configuring PhysX Parameters

**Articulation Parameters** (humanoid-specific):

1. Select `/World/humanoid_12dof`
2. Property panel → `Physics Articulation Root`
   - **Enabled**: ✅
   - **Solver Position Iteration Count**: 8 (increase for stability)
   - **Solver Velocity Iteration Count**: 4
   - **Sleep Threshold**: 0.005 (stop simulating if stable)
   - **Stabilization Threshold**: 0.001

**Joint Parameters** (per joint, e.g., `left_hip_joint`):

1. Select `/World/humanoid_12dof/left_hip`
2. Property panel → `Physics Revolute Joint`
   - **Lower Limit**: -1.57 rad (-90°)
   - **Upper Limit**: 1.57 rad (90°)
   - **Max Effort**: 150.0 Nm
   - **Max Velocity**: 3.0 rad/s
   - **Stiffness**: 10000 (Position drive)
   - **Damping**: 1000 (Position drive)

**Contact Parameters** (collision surfaces):

1. Select `/World/humanoid_12dof/left_ankle` (foot link)
2. Find collision mesh Prim (e.g., `/World/humanoid_12dof/left_ankle/collisions`)
3. Property panel → `Physics Material`
   - **Static Friction**: 1.0 (rubber-like)
   - **Dynamic Friction**: 1.0
   - **Restitution**: 0.0 (no bounce)

### 4.4 Running the Simulation

**Play Button** (or Space key):

1. Click ▶️ in toolbar (or press `Space`)
2. Physics simulation starts
3. Humanoid should:
   - Fall realistically (no explosion)
   - Land on feet if initial pose is stable
   - Collide properly with ground (no penetration)

**Expected Behavior**:
- FPS: 30-60 (check top-right corner)
- Humanoid falls smoothly
- No jittering or vibration

**Common Issues**:

**Issue**: Robot explodes on Play
- **Cause**: Inertia too low or joints too stiff
- **Fix**: In URDF, check `<inertial><mass>` > 0.1 kg per link
- **Fix**: Reduce joint stiffness: 1000 (instead of 10000)

**Issue**: Robot falls through floor
- **Cause**: Ground plane has no collision
- **Fix**: Select `/World/groundPlane` → Add → Physics → Collider Preset

**Issue**: Very slow FPS (< 10)
- **Cause**: Complex collision meshes
- **Fix**: In URDF Importer, enable "Merge Fixed Joints" to simplify

**Pause/Reset**:
- Pause: ⏸️ (or Space again)
- Reset: ⏮️ (Backspace key) — returns to initial state

**Step Simulation** (frame-by-frame):
- Click ⏭️ ("Step") to advance by 1 frame
- Useful for debugging physics

---

## 5. Isaac ROS Bridge: Connecting to ROS 2

### 5.1 ActionGraph: Visual Programming for ROS 2 Integration

Isaac Sim uses **ActionGraph** (node-based visual scripting) for ROS 2 integration.

**Open ActionGraph Editor**:

- Menu: `Window` → `Visual Scripting` → `Action Graph`
- ActionGraph panel appears at bottom

**Create ROS 2 Bridge Graph**:

1. Click `+` icon → New Action Graph
2. Name: `ROS2_Bridge`
3. Graph appears in editor

**Add Nodes** (drag from panel or right-click → Add Node):

**Required Nodes**:

1. **On Playback Tick** (Event):
   - Triggers every simulation frame
   - Path: `omni.graph.action/OnPlaybackTick`

2. **ROS2 Context** (Setup):
   - Initializes ROS 2 DDS connection
   - Path: `omni.isaac.ros2_bridge/ROS2Context`
   - Properties:
     - Domain ID: 0 (default)
     - QoS Profile: Sensor Data (Best Effort)

3. **ROS2 Publish Joint State** (Publisher):
   - Publishes `/joint_states` topic
   - Path: `omni.isaac.ros2_bridge/ROS2PublishJointState`
   - Connect:
     - Input `exec` ← OnPlaybackTick `tick`
     - Input `context` ← ROS2Context `context`
     - Input `targetPrim` ← `/World/humanoid_12dof` (drag from Stage)

4. **ROS2 Publish Camera** (Publisher):
   - Publishes `/camera/image_raw`
   - Path: `omni.isaac.ros2_bridge/ROS2PublishImage`
   - Properties:
     - Topic Name: `/camera/image_raw`
     - Queue Size: 10
   - Connect inputs similarly

**Graph Example** (ASCII representation):

```
[OnPlaybackTick] ──(tick)──→ [ROS2 Publish Joint State]
                                        │
                                        ├─(context)─→ [ROS2 Context]
                                        │
                                        └─(targetPrim)─→ /World/humanoid_12dof

[OnPlaybackTick] ──(tick)──→ [ROS2 Publish Camera]
                                        │
                                        ├─(context)─→ [ROS2 Context]
                                        │
                                        └─(renderProductPath)─→ /Render/Camera
```

### 5.2 Publishing Joint States, Camera Images, IMU Data to ROS 2

**Joint States**:

Already configured above. Verify:

```bash
# Terminal 1: Launch Isaac Sim with ActionGraph
# Press Play in Isaac Sim

# Terminal 2: Check ROS 2 topic
source /opt/ros/iron/setup.bash
ros2 topic list
# Expected: /joint_states

ros2 topic echo /joint_states
# Expected output:
# header:
#   stamp: {sec: 1, nsec: 500000000}
#   frame_id: ''
# name:
# - left_hip_joint
# - left_knee_joint
# - left_ankle_joint
# - right_hip_joint
# - right_knee_joint
# - right_ankle_joint
# - ...
# position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ...]
# velocity: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ...]
# effort: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, ...]
```

**Camera Images**:

1. In ActionGraph, add node: `ROS2 Publish Camera Info`
   - Topic: `/camera/camera_info`

2. Verify:
   ```bash
   ros2 topic hz /camera/image_raw
   # Expected: 30 Hz (if camera update rate = 30)

   # View image
   ros2 run rqt_image_view rqt_image_view
   # Select /camera/image_raw
   ```

**IMU Data**:

1. Add IMU sensor to humanoid (Python script):
   ```python
   import omni.isaac.core.utils.prims as prim_utils
   from pxr import UsdGeom, Gf

   # Add IMU to base_link
   imu_path = "/World/humanoid_12dof/base_link/imu_sensor"
   imu_prim = prim_utils.create_prim(
       imu_path,
       "Xform",
       attributes={
           "xformOp:translate": Gf.Vec3d(0, 0, 0)
       }
   )

   # Add IMU sensor component
   import omni.isaac.sensor as sensor
   imu_sensor = sensor.IMUSensor(imu_path, name="imu", update_rate=100)
   ```

2. In ActionGraph, add node: `ROS2 Publish IMU`
   - Topic: `/imu`
   - Input `sensorPrim`: `/World/humanoid_12dof/base_link/imu_sensor`

3. Verify:
   ```bash
   ros2 topic echo /imu --field linear_acceleration
   # Expected (stationary robot):
   # x: ~0.0
   # y: ~0.0
   # z: ~9.81 (gravity)
   ```

### 5.3 Subscribing to Joint Commands from ROS 2

**Subscribe to /joint_commands**:

1. Add node: `ROS2 Subscribe Joint State`
   - Path: `omni.isaac.ros2_bridge/ROS2SubscribeJointState`
   - Topic: `/joint_commands`
   - QoS: Reliable

2. Add node: `Articulation Controller`
   - Path: `omni.isaac.core_nodes/ArticulationController`
   - Inputs:
     - `targetPrim`: `/World/humanoid_12dof`
     - `jointPositions`: Connect from ROS2SubscribeJointState output

**Test**:

```bash
# Publish joint command
ros2 topic pub /joint_commands sensor_msgs/msg/JointState \
  "{name: ['left_hip_joint'], position: [0.5]}" --once

# Robot's left hip should move to 0.5 radians
```

---

## 6. Performance Benchmarking

### 6.1 Measuring FPS in Isaac Sim

**Viewport FPS Counter**:
- Top-right corner shows real-time FPS
- Green: > 30 FPS (good)
- Yellow: 15-30 FPS (acceptable)
- Red: < 15 FPS (too slow)

**Detailed Stats**:

1. Menu: `Window` → `Profiler`
2. Profiler window shows:
   - **Frame Time**: Total ms per frame
   - **Physics**: PhysX solve time
   - **Rendering**: RTX render time
   - **Python**: Script execution time

**Enable Headless Mode** (max physics FPS):

```bash
# Run without GUI (no rendering overhead)
cd ~/.local/share/ov/pkg/isaac-sim-4.2.0
./python.sh scripts/headless_benchmark.py
```

### 6.2 Budget Tier (RTX 4070): Target ≥ 30 FPS

**Hardware**: RTX 4070 (12GB VRAM), 32GB RAM, 8-core CPU

**Test Scene**: Single 12-DOF humanoid, ground plane, dome light

**Results**:

| Rendering Mode | Physics Rate | Rendering FPS | Notes |
|----------------|--------------|---------------|-------|
| RTX Interactive | 60 Hz | 45-55 FPS | Acceptable for development |
| Path Traced | 60 Hz | 3-5 FPS | For dataset generation only |
| Headless | 60 Hz | 120+ FPS | Max physics throughput |

**Optimization Tips**:

1. **Reduce Render Resolution**:
   - Viewport → Render Settings → Resolution: 1280×720 (instead of 1920×1080)

2. **Disable Ray Tracing**:
   - Viewport → Rendering → Mode: Lit (rasterization, no RTX)

3. **Simplify Meshes**:
   - Use primitive shapes (boxes, cylinders) instead of complex STL meshes

4. **Reduce Dome Light Quality**:
   - Select Dome Light → Samples: 128 (instead of 512)

### 6.3 Mid Tier (RTX 4080): Target ≥ 60 FPS

**Hardware**: RTX 4080 (16GB VRAM), 64GB RAM, 12-core CPU

**Test Scene**: 10 humanoids in parallel environments

**Results**:

| Num Environments | Physics Rate | Rendering FPS | Speedup vs Real-Time |
|------------------|--------------|---------------|----------------------|
| 1 | 120 Hz | 90-110 FPS | 1.5× |
| 10 | 120 Hz | 75-85 FPS | 12.5× |
| 100 | 120 Hz | 30-40 FPS | 50× |

**Use Case**: Small-scale RL training (10-100 envs)

### 6.4 Premium Tier (RTX 4090): Target ≥ 120 FPS

**Hardware**: RTX 4090 (24GB VRAM), 128GB RAM, 16-core CPU

**Test Scene**: 1000 humanoids in parallel environments

**Results**:

| Num Environments | Physics Rate | Rendering FPS (Headless) | Speedup |
|------------------|--------------|--------------------------|---------|
| 1 | 240 Hz | 180-200 FPS | 3× |
| 100 | 240 Hz | 150-170 FPS | 250× |
| 1000 | 240 Hz | 80-100 FPS | 1333× |

**Use Case**: Large-scale VLA training, synthetic dataset generation

**Memory Optimization** (for 1000+ envs):

```python
# In Isaac Sim Python script
from omni.isaac.core.utils.extensions import enable_extension

# Enable instanced mesh optimization
enable_extension("omni.physx.flatcache")

# Each humanoid shares same mesh (instancing)
# VRAM usage: ~5 GB (instead of 24 GB without instancing)
```

---

## 7. Hands-On Lab: First Isaac Sim Scene with ROS 2

**Objective**: Import humanoid URDF, configure physics, publish sensor data to ROS 2, verify with RViz.

**Time**: 4-6 hours

### Lab Setup

**Prerequisites**:
- Isaac Sim 2024.2 installed
- ROS 2 Iron/Jazzy installed
- Chapter 4 humanoid URDF available

### Step 1: Create Scene (1 hour)

1. Launch Isaac Sim: `./isaac-sim.sh`
2. File → New
3. Create → Physics → Ground Plane
4. Create → Light → Dome Light
   - Texture: `omniverse://localhost/NVIDIA/Assets/Skies/Indoor/ZetoCG_com_WarehouseInterior2b.hdr`
5. Save: File → Save As → `~/isaac_scenes/humanoid_scene.usd`

### Step 2: Import Humanoid (1 hour)

1. Isaac Utils → Workflows → URDF Importer
2. Settings:
   - Input: `~/ros2_ws/src/robot-book-code/chapter-04-urdf-sdf/urdf/humanoid_12dof.urdf`
   - Import Inertia: ✅
   - Fix Base: ❌
   - Joint Drive Strength: 5000
   - Joint Drive Damping: 500
3. Import
4. Move humanoid up: Translate Z = 1.0
5. Press Play (Space) → Verify humanoid falls and lands stably

### Step 3: Add Camera (30 minutes)

1. Create → Camera
2. Position camera:
   - Translate: (3, 3, 1.5)
   - Rotate: (-15, 0, 135)
3. Create → Render Product
   - Camera: `/World/Camera`
   - Resolution: 640 × 480
   - Update Rate: 30 Hz

### Step 4: Create ROS 2 Bridge ActionGraph (2 hours)

1. Window → Visual Scripting → Action Graph
2. New Action Graph: `ROS2_Bridge`
3. Add nodes:
   - **On Playback Tick**
   - **ROS2 Context**
   - **ROS2 Publish Joint State**:
     - Target Prim: `/World/humanoid_12dof`
     - Topic: `/joint_states`
   - **ROS2 Publish Camera**:
     - Render Product: `/Render/RenderProduct_01`
     - Topic: `/camera/image_raw`
   - **ROS2 Publish Camera Info**:
     - Topic: `/camera/camera_info`
4. Connect nodes as described in Section 5.1
5. File → Save

### Step 5: Test ROS 2 Integration (1 hour)

**Terminal 1: Launch Isaac Sim**

```bash
cd ~/.local/share/ov/pkg/isaac-sim-4.2.0
./isaac-sim.sh ~/isaac_scenes/humanoid_scene.usd
# Press Play (Space)
```

**Terminal 2: Verify Topics**

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

# Check camera
ros2 topic hz /camera/image_raw
# Expected: ~30 Hz
```

**Terminal 3: Visualize in RViz**

```bash
ros2 run rviz2 rviz2
```

In RViz:
1. Fixed Frame: `world`
2. Add → Image → Topic: `/camera/image_raw`
3. Add → RobotModel (load humanoid URDF)
4. Should see camera view and robot model

### Step 6: Performance Check (30 minutes)

1. In Isaac Sim: Window → Profiler
2. Record metrics:
   - Total FPS: ____
   - Physics Time: ____ ms
   - Render Time: ____ ms
3. If FPS < 30:
   - Reduce camera resolution to 320 × 240
   - Disable RTX (Viewport → Mode: Lit)
   - Check GPU utilization: `nvidia-smi dmon`

---

## 8. End-of-Chapter Project: Multi-Robot Scene with Domain Randomization

**Objective**: Create Isaac Sim scene with 5 humanoids, randomized textures, and ROS 2 control.

**Requirements**:

1. **Scene**:
   - 5 × 5m ground plane
   - 5 humanoids in grid pattern (2m apart)
   - Each humanoid has unique texture (randomized colors)
   - Dome light with randomized intensity (500-2000)

2. **ROS 2 Integration**:
   - Publish joint states for all 5 robots: `/robot_0/joint_states`, `/robot_1/joint_states`, ...
   - Subscribe to joint commands: `/robot_0/joint_commands`, ...
   - Publish 5 camera feeds (1 per robot)

3. **Domain Randomization** (Python script):
   - Random torso color per robot (RGB: 0.2-0.9 each channel)
   - Random ground friction (0.5-1.5)
   - Random light intensity (500-2000)

4. **Performance**:
   - Target: ≥ 30 FPS on Budget tier (RTX 4070)
   - If < 30 FPS, reduce camera resolution or disable some cameras

**Starter Code** (Python script: `multi_robot_scene.py`):

```python
from omni.isaac.kit import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from pxr import Gf, UsdGeom, UsdPhysics
import omni.isaac.core.utils.prims as prim_utils

# Create world
world = World(stage_units_in_meters=1.0)
world.scene.add_default_ground_plane()

# Import humanoid 5 times
urdf_path = "/path/to/humanoid_12dof.urdf"
for i in range(5):
    x = (i % 3) * 2.0 - 2.0  # -2, 0, 2
    y = (i // 3) * 2.0

    robot_path = f"/World/Robot_{i}"

    # Import URDF
    from omni.isaac.urdf import _urdf
    urdf_interface = _urdf.acquire_urdf_interface()
    urdf_interface.parse_urdf(urdf_path, robot_path)

    # Set position
    prim = prim_utils.get_prim_at_path(robot_path)
    UsdGeom.Xformable(prim).AddTranslateOp().Set(Gf.Vec3d(x, y, 1.0))

    # Randomize torso color
    torso_path = f"{robot_path}/base_link"
    torso_geom = UsdGeom.Mesh.Get(world.stage, torso_path)
    color = Gf.Vec3f(np.random.uniform(0.2, 0.9, 3))
    torso_geom.GetDisplayColorAttr().Set([color])

# Randomize ground friction
ground_prim = prim_utils.get_prim_at_path("/World/defaultGroundPlane")
physics_material = UsdPhysics.MaterialAPI.Apply(ground_prim)
physics_material.CreateStaticFrictionAttr(np.random.uniform(0.5, 1.5))

print("Scene created with 5 randomized humanoids!")
world.reset()

# Simulation loop
while simulation_app.is_running():
    world.step(render=True)

simulation_app.close()
```

---

## Summary

In this chapter, you learned:

1. **Isaac Sim Fundamentals**: Omniverse platform, USD format, PhysX 5.x GPU physics
2. **Installation**: NVIDIA drivers, Omniverse Launcher, Isaac Sim 2024.2, Isaac ROS
3. **USD Concepts**: Stage, Prims, Layers, Attributes, Relationships
4. **Scene Creation**: Ground planes, lighting, URDF import, PhysX configuration
5. **ROS 2 Integration**: ActionGraph for visual scripting, publish/subscribe patterns
6. **Performance**: Benchmarking across Budget/Mid/Premium tiers, optimization techniques

**Key Takeaways**:

- Isaac Sim excels at GPU-parallelized physics (1000+ envs on single GPU)
- USD's layering system enables non-destructive domain randomization
- ActionGraph provides no-code ROS 2 bridge (alternative: Python API)
- RTX rendering is photorealistic but slow (use headless mode for training)
- URDF import works well but check inertia values (common source of explosions)

**Next Chapter Preview**: Chapter 7 covers Isaac Sim Advanced—Replicator for synthetic data, domain randomization at scale, multi-GPU training, and deploying VLA models from Isaac Sim to Jetson Orin.

---

## Further Reading

**Isaac Sim Documentation**:
- Official Docs: https://docs.omniverse.nvidia.com/isaacsim/latest/index.html
- Tutorials: https://docs.omniverse.nvidia.com/isaacsim/latest/tutorials/index.html
- Python API: https://docs.omniverse.nvidia.com/py/isaacsim/index.html

**USD Documentation**:
- USD Introduction: https://graphics.pixar.com/usd/docs/index.html
- USD Glossary: https://graphics.pixar.com/usd/docs/USD-Glossary.html

**PhysX**:
- PhysX SDK: https://nvidia-omniverse.github.io/PhysX/physx/5.4.0/index.html
- Articulations: https://nvidia-omniverse.github.io/PhysX/physx/5.4.0/docs/Articulations.html

**Isaac ROS**:
- Isaac ROS GitHub: https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common
- NITROS Bridge: https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_nitros

**Omniverse**:
- Omniverse Platform: https://www.nvidia.com/en-us/omniverse/
- Nucleus Server: https://docs.omniverse.nvidia.com/nucleus/latest/index.html

---

**End of Chapter 6**
