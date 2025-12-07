# Chapter 5: Gazebo Basics

**Learning Objectives**

By the end of this chapter, you will be able to:

1. Compare Gazebo and Isaac Sim and choose the right simulator for your project
2. Understand Gazebo's client-server architecture and plugin system
3. Install Gazebo Harmonic and configure the ROS 2 bridge (`ros_gz_bridge`)
4. Spawn humanoid robots in Gazebo worlds with proper physics configuration
5. Configure physics parameters (gravity, time step, contact properties) for realistic simulation
6. Add sensor plugins (camera, IMU, LiDAR, contact sensors) to robot models
7. Debug common Gazebo issues (real-time factor, unstable physics, missing sensor data)
8. Build custom Gazebo worlds with terrain, obstacles, and lighting

**Prerequisites**: Chapter 4 (URDF/SDF), basic Linux command line, ROS 2 fundamentals

**Estimated Time**: 8-10 hours (4 hours reading, 4-6 hours hands-on lab)

---

## 1. Gazebo vs. Isaac Sim: When to Use Which

### 1.1 Open-Source vs. Proprietary Tradeoffs

Both Gazebo and NVIDIA Isaac Sim are physics-based robot simulators, but they serve different use cases:

| Feature | Gazebo Harmonic | NVIDIA Isaac Sim 2024.2 |
|---------|----------------|------------------------|
| **License** | Apache 2.0 (Open Source) | Free for non-commercial, subscription for commercial |
| **Primary Use Case** | General robotics, ROS 2 integration | Physical AI, VLA training, sim-to-real transfer |
| **Physics Engine** | ODE, Bullet, DART (choice) | PhysX 5.x (GPU-accelerated) |
| **Rendering** | Ogre 2.x (traditional rasterization) | RTX ray tracing, path tracing |
| **Performance** | CPU-based, ~30-60 FPS for single robot | GPU-accelerated, 1000+ parallel envs on single GPU |
| **Sensor Fidelity** | Good (RGB, depth, LiDAR) | Excellent (physically-based camera, RTX LiDAR) |
| **ROS 2 Integration** | Native (`ros_gz_bridge`) | Via Isaac ROS packages |
| **Learning Curve** | Moderate (well-documented) | Steep (Omniverse ecosystem) |
| **Hardware Requirements** | CPU: 4+ cores, RAM: 8GB, GPU: Optional | GPU: RTX 3070+ (8GB VRAM), RAM: 32GB |
| **Commercial Support** | Community-driven | NVIDIA Enterprise support |

**When to Use Gazebo**:
- ✅ Open-source projects, academic research, teaching
- ✅ ROS 2-native workflows (tight integration)
- ✅ CPU-only or low-end GPU hardware
- ✅ Rapid prototyping without GPU cluster
- ✅ Traditional robotics (arms, mobile robots, basic humanoids)

**When to Use Isaac Sim**:
- ✅ Physical AI and VLA model training (need 1000+ parallel envs)
- ✅ Photorealistic rendering for vision models (RTX ray tracing)
- ✅ High-fidelity contact dynamics (grasping, manipulation)
- ✅ Sim-to-real transfer (NVIDIA Jetson deployment)
- ✅ Access to RTX-capable GPUs (3070+)

**Hybrid Approach** (Recommended):
- **Prototyping**: Gazebo (fast iteration, free, easy setup)
- **VLA Training**: Isaac Sim (GPU parallelization, synthetic data generation)
- **Hardware Testing**: Real robot (Jetson Orin + cameras)

In this chapter, we focus on **Gazebo** for rapid prototyping and ROS 2 integration. Chapter 6-7 cover Isaac Sim for advanced use cases.

### 1.2 Physics Engines: ODE, Bullet, DART vs. PhysX

Gazebo supports multiple physics engines via plugins:

**ODE (Open Dynamics Engine)**:
- **Pros**: Stable, well-tested, default in Gazebo Classic
- **Cons**: Slow for complex scenes, inaccurate contact for small objects
- **Use Case**: Simple mobile robots, basic manipulation

**Bullet**:
- **Pros**: Fast, good for rigid body dynamics, multithreading support
- **Cons**: Less stable than ODE for high-DOF systems (humanoids)
- **Use Case**: Fast prototyping, multi-robot simulations

**DART (Dynamic Animation and Robotics Toolkit)**:
- **Pros**: Excellent for humanoids (accurate contact, muscle models)
- **Cons**: Slower than Bullet, requires careful tuning
- **Use Case**: Bipedal locomotion, manipulation with compliant contact

**PhysX 5.x (Isaac Sim)**:
- **Pros**: GPU-accelerated, highly parallel, accurate soft body physics
- **Cons**: NVIDIA GPUs only, closed-source
- **Use Case**: Large-scale RL training, sim-to-real transfer

**Recommendation for Humanoids**:
- **Gazebo**: Use **DART** for bipedal walking (better contact stability)
- **Isaac Sim**: Use **PhysX** for VLA training (GPU parallelization)

### 1.3 Rendering: Ogre vs. RTX Ray Tracing

**Gazebo (Ogre 2.x)**:
- Traditional rasterization (forward/deferred shading)
- Phong/Blinn-Phong materials
- Real-time shadows (shadow maps)
- ~30-60 FPS for typical scenes
- **Limitation**: Not physically accurate for vision (poor reflections, global illumination)

**Isaac Sim (RTX)**:
- Hardware ray tracing (RTX cores)
- Physically-based rendering (PBR materials)
- Path tracing for global illumination
- ~10-30 FPS for single env, scales to 1000+ envs
- **Advantage**: Photorealistic images for VLA training (reduces sim-to-real gap)

**Impact on Physical AI**:
- **Gazebo**: Good for testing algorithms, not for training vision models
- **Isaac Sim**: Required for training VLAs on synthetic data (OpenVLA, RT-2)

---

## 2. Gazebo Architecture

### 2.1 Client-Server Model

Gazebo uses a **distributed architecture** with separate processes:

```
┌─────────────────────────────────────────────────────────────┐
│                    Gazebo Ecosystem                         │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌──────────────┐      ┌──────────────┐                   │
│  │  gz sim      │◄────►│  Transport   │◄──────────────┐   │
│  │  (Server)    │      │  (Ignition   │               │   │
│  │              │      │  Transport)  │               │   │
│  └──────────────┘      └──────────────┘               │   │
│         │                      ▲                       │   │
│         │ Physics/Sensors      │ Topics/Services      │   │
│         ▼                      │                       │   │
│  ┌──────────────┐              │                       │   │
│  │  Plugins     │──────────────┘                       │   │
│  │  (Sensors,   │                                      │   │
│  │   Physics)   │                                      │   │
│  └──────────────┘                                      │   │
│                                                         │   │
│  ┌──────────────┐      ┌──────────────┐               │   │
│  │  gz sim -g   │◄────►│  ROS 2 (DDS) │◄──────────────┤   │
│  │  (GUI Client)│      │              │               │   │
│  └──────────────┘      └──────────────┘               │   │
│                               ▲                        │   │
│                               │                        │   │
│                        ┌──────────────┐               │   │
│                        │ ros_gz_bridge│               │   │
│                        └──────────────┘               │   │
│                                                         │   │
└─────────────────────────────────────────────────────────────┘
```

**Components**:

1. **`gz sim` (Server)**:
   - Runs physics simulation (ODE/Bullet/DART)
   - Manages world state (models, actors, lights)
   - Publishes sensor data via Ignition Transport
   - Headless mode: `gz sim -r world.sdf` (no GUI, for CI/cluster)

2. **`gz sim -g` (GUI Client)**:
   - Visualizes simulation (Ogre 2.x rendering)
   - Provides entity inspector, topic viewer, transform control
   - Can run on separate machine (network transparency)

3. **Ignition Transport**:
   - Custom pub-sub middleware (not DDS)
   - Topics: `/model/robot/joint_states`, `/camera/image`
   - Services: `/world/default/create`, `/world/default/set_pose`

4. **`ros_gz_bridge`**:
   - Bidirectional bridge between Ignition Transport ↔ ROS 2 (DDS)
   - Maps Gazebo topics to ROS 2 topics (e.g., `/camera/image` → `/camera/image_raw`)

**Key Difference from Gazebo Classic**:
- **Classic**: Monolithic server with embedded GUI (single process)
- **Harmonic**: Separate server/client (scalable, can run headless on cluster)

### 2.2 Worlds, Models, Actors, Sensors

Gazebo organizes entities hierarchically:

**World** (`.sdf` file):
- Top-level container
- Physics settings (gravity, time step)
- Lights, ground plane, atmosphere

```xml
<sdf version="1.9">
  <world name="humanoid_world">
    <physics name="default_physics" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1.0 1.0 1.0 1.0</diffuse>
      <specular>0.5 0.5 0.5 1.0</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane><normal>0 0 1</normal></plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane><normal>0 0 1</normal><size>100 100</size></plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <include>
      <uri>model://humanoid_12dof</uri>
      <pose>0 0 1.0 0 0 0</pose>
    </include>
  </world>
</sdf>
```

**Model**:
- Composed of links (rigid bodies) and joints (constraints)
- Can be included from model database: `<include><uri>model://name</uri></include>`
- SDF format (same as URDF structure, but with additional features)

**Actor**:
- Scripted entity with pre-defined trajectory (e.g., walking human for obstacle avoidance)
- Uses skeletal animation (Collada `.dae` files)

**Sensor**:
- Attached to links via `<sensor>` tag
- Types: camera, depth_camera, imu, lidar, contact, force_torque
- Publishes data via Ignition Transport topics

### 2.3 Plugin System

Gazebo extensibility comes from **plugins** (shared libraries loaded at runtime):

**Plugin Types**:

1. **World Plugin**: Global behavior (e.g., wind, custom physics)
2. **Model Plugin**: Robot-specific (e.g., joint controller, ROS 2 bridge)
3. **Sensor Plugin**: Sensor data processing (e.g., image compression, noise injection)
4. **System Plugin**: Core Gazebo features (e.g., physics engine, rendering)
5. **GUI Plugin**: Custom UI panels

**Example**: ROS 2 Joint State Publisher Plugin (model plugin)

```xml
<model name="humanoid">
  <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher">
    <joint_name>left_hip_joint</joint_name>
    <joint_name>left_knee_joint</joint_name>
    <joint_name>right_hip_joint</joint_name>
    <joint_name>right_knee_joint</joint_name>
    <update_rate>50</update_rate>
  </plugin>
</model>
```

**Custom Plugin** (C++):

```cpp
#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>

namespace my_plugins {
class CustomController : public gz::sim::System,
                          public gz::sim::ISystemConfigure,
                          public gz::sim::ISystemPreUpdate {
  public:
    void Configure(const gz::sim::Entity &entity,
                   const std::shared_ptr<const sdf::Element> &sdf,
                   gz::sim::EntityComponentManager &ecm,
                   gz::sim::EventManager &eventMgr) override {
      // Initialize plugin
    }

    void PreUpdate(const gz::sim::UpdateInfo &info,
                   gz::sim::EntityComponentManager &ecm) override {
      // Called every simulation step
    }
};
}

GZ_ADD_PLUGIN(my_plugins::CustomController,
              gz::sim::System,
              my_plugins::CustomController::ISystemConfigure,
              my_plugins::CustomController::ISystemPreUpdate)
```

---

## 3. Launching Gazebo with ROS 2

### 3.1 Installing Gazebo Harmonic

**Prerequisites**:
- Ubuntu 22.04 LTS (ROS 2 Iron) or Ubuntu 24.04 LTS (ROS 2 Jazzy)

**Installation** (Binary packages):

```bash
# Add Gazebo repository
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# Update and install Gazebo Harmonic
sudo apt update
sudo apt install gz-harmonic

# Verify installation
gz sim --version
# Expected output: Gazebo Sim, version 8.x.x
```

**Install ROS 2 Integration Packages**:

```bash
# ROS 2 Iron (Ubuntu 22.04)
sudo apt install ros-iron-ros-gz

# ROS 2 Jazzy (Ubuntu 24.04)
sudo apt install ros-jazzy-ros-gz

# This installs:
# - ros_gz_bridge: ROS 2 ↔ Gazebo topic bridge
# - ros_gz_sim: Launch file integration
# - ros_gz_image: Image transport bridge
```

**Test Installation**:

```bash
# Terminal 1: Launch Gazebo with empty world
gz sim empty.sdf

# Terminal 2: List Gazebo topics
gz topic -l
# Expected:
# /clock
# /stats
# /world/empty/clock
# /world/empty/stats
```

### 3.2 `ros_gz_bridge`: Connecting ROS 2 and Gazebo

The bridge maps Gazebo topics (Ignition Transport) to ROS 2 topics (DDS):

**Launch Bridge** (manual):

```bash
# Bridge single topic: Gazebo → ROS 2
ros2 run ros_gz_bridge parameter_bridge /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock

# Bridge bidirectional: ROS 2 ↔ Gazebo
ros2 run ros_gz_bridge parameter_bridge \
  /cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist \
  /odom@nav_msgs/msg/Odometry[gz.msgs.Odometry
```

**Bridge Configuration** (YAML file):

```yaml
# bridge_config.yaml
- topic_name: "/camera/image_raw"
  ros_type_name: "sensor_msgs/msg/Image"
  gz_type_name: "gz.msgs.Image"
  direction: GZ_TO_ROS

- topic_name: "/imu"
  ros_type_name: "sensor_msgs/msg/Imu"
  gz_type_name: "gz.msgs.IMU"
  direction: GZ_TO_ROS

- topic_name: "/joint_states"
  ros_type_name: "sensor_msgs/msg/JointState"
  gz_type_name: "gz.msgs.Model"
  direction: GZ_TO_ROS

- topic_name: "/cmd_vel"
  ros_type_name: "geometry_msgs/msg/Twist"
  gz_type_name: "gz.msgs.Twist"
  direction: ROS_TO_GZ
```

**Launch with YAML**:

```bash
ros2 run ros_gz_bridge parameter_bridge --ros-args \
  -p config_file:=path/to/bridge_config.yaml
```

**Common Topic Mappings**:

| ROS 2 Topic | ROS 2 Type | Gazebo Topic | Gazebo Type |
|-------------|-----------|--------------|-------------|
| `/camera/image_raw` | `sensor_msgs/msg/Image` | `/camera/image` | `gz.msgs.Image` |
| `/imu` | `sensor_msgs/msg/Imu` | `/imu` | `gz.msgs.IMU` |
| `/joint_states` | `sensor_msgs/msg/JointState` | `/world/default/model/robot/joint_state` | `gz.msgs.Model` |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | `/cmd_vel` | `gz.msgs.Twist` |
| `/scan` | `sensor_msgs/msg/LaserScan` | `/lidar` | `gz.msgs.LaserScan` |

### 3.3 Spawning Humanoid URDF in Gazebo World

**Method 1: Convert URDF to SDF, Include in World**

```bash
# Convert URDF to SDF
gz sdf -p humanoid_12dof.urdf > humanoid.sdf

# Edit world file to include model
# <include><uri>file://path/to/humanoid.sdf</uri></include>

# Launch Gazebo
gz sim my_world.sdf
```

**Method 2: Spawn via ROS 2 Service** (dynamic spawning)

```bash
# Terminal 1: Launch Gazebo with empty world
gz sim empty.sdf

# Terminal 2: Spawn robot using ros_gz_sim spawn service
ros2 run ros_gz_sim create -file humanoid.sdf \
  -name humanoid -x 0 -y 0 -z 1.0
```

**Method 3: Launch File Integration** (recommended)

```python
# launch/gazebo_humanoid.launch.py
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # Gazebo server (headless)
    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r -v 4 empty.sdf'}.items(),
    )

    # Gazebo GUI client
    gazebo_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v 4'}.items(),
    )

    # Spawn robot
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', '/path/to/humanoid.sdf',
            '-name', 'humanoid',
            '-x', '0', '-y', '0', '-z', '1.0'
        ],
        output='screen',
    )

    # ROS 2 ↔ Gazebo bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        output='screen',
    )

    return LaunchDescription([
        gazebo_server,
        gazebo_gui,
        spawn_robot,
        bridge,
    ])
```

**Launch**:

```bash
ros2 launch my_package gazebo_humanoid.launch.py
```

---

## 4. Physics Configuration

### 4.1 Gravity, Time Step, Solver Iterations

**World Physics Tag**:

```xml
<world name="default">
  <physics name="default_physics" type="dart">
    <!-- Gravity (m/s²) -->
    <gravity>0 0 -9.81</gravity>

    <!-- Simulation time step (seconds) -->
    <max_step_size>0.001</max_step_size>

    <!-- Real-time factor (1.0 = real-time, 0.5 = half-speed) -->
    <real_time_factor>1.0</real_time_factor>

    <!-- Solver iterations (higher = more accurate, slower) -->
    <dart>
      <solver>
        <solver_type>dantzig</solver_type>
        <constraint_cfm>0.0</constraint_cfm>
        <constraint_erp>0.2</constraint_erp>
      </solver>
      <collision_detector>bullet</collision_detector>
    </dart>
  </physics>
</world>
```

**Key Parameters**:

1. **`max_step_size`** (default: 0.001s = 1ms):
   - Smaller = more accurate, slower
   - For humanoids: 0.001s (1 kHz) recommended for bipedal stability
   - For wheeled robots: 0.01s (100 Hz) often sufficient

2. **`real_time_factor`**:
   - `1.0`: Simulation runs at real-time speed
   - `< 1.0`: Simulation slower than real-time (complex scenes)
   - `> 1.0`: Simulation faster than real-time (simple scenes)
   - **Check RTF in Gazebo GUI**: Bottom-right corner shows "RTF: 0.95"

3. **Solver Iterations** (DART-specific):
   - `constraint_cfm` (Constraint Force Mixing): Softness (0 = hard, 1e-5 = soft)
   - `constraint_erp` (Error Reduction Parameter): Constraint stabilization (0.2 typical)

**Tuning for Humanoids**:

```xml
<physics name="humanoid_physics" type="dart">
  <max_step_size>0.0005</max_step_size>  <!-- 2 kHz for fast contacts -->
  <real_time_factor>1.0</real_time_factor>
  <dart>
    <solver>
      <solver_type>pgs</solver_type>  <!-- Projected Gauss-Seidel (faster) -->
      <constraint_cfm>0.00001</constraint_cfm>  <!-- Slightly soft constraints -->
      <constraint_erp>0.2</constraint_erp>
    </solver>
    <collision_detector>bullet</collision_detector>  <!-- Fast broad-phase -->
  </dart>
</physics>
```

### 4.2 Contact Parameters: Friction, Restitution, Damping

Contact properties are defined in `<collision>` tags:

```xml
<link name="left_foot">
  <collision name="collision">
    <geometry>
      <box><size>0.15 0.08 0.05</size></box>
    </geometry>
    <surface>
      <friction>
        <ode>
          <mu>1.0</mu>    <!-- Coulomb friction (μ) -->
          <mu2>1.0</mu2>  <!-- Secondary friction direction -->
          <fdir1>0 0 0</fdir1>  <!-- Friction direction (0 = omnidirectional) -->
          <slip1>0.0</slip1>
          <slip2>0.0</slip2>
        </ode>
        <bullet>
          <friction>1.0</friction>
          <rolling_friction>0.01</rolling_friction>
        </bullet>
      </friction>
      <contact>
        <ode>
          <kp>1e6</kp>  <!-- Contact stiffness (N/m) -->
          <kd>1e3</kd>  <!-- Contact damping (N·s/m) -->
          <max_vel>0.01</max_vel>  <!-- Max penetration correction velocity -->
          <min_depth>0.001</min_depth>  <!-- Min penetration before contact -->
        </ode>
      </contact>
      <bounce>
        <restitution_coefficient>0.0</restitution_coefficient>  <!-- 0 = inelastic, 1 = elastic -->
        <threshold>0.01</threshold>
      </bounce>
    </surface>
  </collision>
</link>
```

**Material Presets**:

| Material | `mu` (Friction) | `restitution` | Use Case |
|----------|----------------|---------------|----------|
| **Rubber (foot)** | 1.0 - 1.5 | 0.0 | Bipedal locomotion (high friction, no bounce) |
| **Metal (gripper)** | 0.5 - 0.8 | 0.3 | Manipulation (moderate friction, slight bounce) |
| **Ice** | 0.02 - 0.05 | 0.1 | Slippery terrain testing |
| **Wood (table)** | 0.4 - 0.6 | 0.5 | Typical indoor surface |

**Common Issue**: Robot sinking into floor
- **Cause**: `kp` (stiffness) too low
- **Fix**: Increase to `1e7` or higher

### 4.3 Performance Tuning: Real-Time Factor

**Monitoring RTF**:

```bash
# Check simulation stats
gz topic -e -t /stats

# Expected output:
# sim_time: 10.523
# real_time: 10.500
# iterations: 10523
# real_time_factor: 1.002  # Good! Close to 1.0
```

**Improving RTF** (if < 0.9):

1. **Reduce Physics Complexity**:
   - Simplify collision geometry (use boxes instead of meshes)
   - Reduce `max_step_size` only if needed (try 0.002s instead of 0.001s)

2. **Optimize Rendering**:
   - Run headless: `gz sim -r -s world.sdf` (no GUI)
   - Disable shadows in GUI: View → Shadows → Off
   - Reduce camera resolution in SDF: `<width>320</width><height>240</height>`

3. **Use Faster Physics Engine**:
   - Switch from DART to Bullet: `<physics type="bullet">`
   - Trade accuracy for speed (Bullet faster, DART more accurate)

4. **Parallelize** (advanced):
   - Use Gazebo Island Threading (automatic for disconnected models)
   - Run multiple Gazebo instances with different world portions

---

## 5. Sensor Plugins

### 5.1 Camera Sensor (RGB, Depth)

**RGB Camera**:

```xml
<link name="camera_link">
  <sensor name="camera" type="camera">
    <pose>0.1 0 0 0 0 0</pose>
    <update_rate>30</update_rate>
    <camera name="head_camera">
      <horizontal_fov>1.3962634</horizontal_fov>  <!-- 80° FOV -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>  <!-- RGB 24-bit -->
      </image>
      <clip>
        <near>0.02</near>  <!-- Near clipping plane (m) -->
        <far>10.0</far>    <!-- Far clipping plane (m) -->
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>  <!-- ~2/255 noise per pixel -->
      </noise>
    </camera>
    <always_on>true</always_on>
    <visualize>true</visualize>
    <topic>/camera/image_raw</topic>
  </sensor>
</link>
```

**Depth Camera** (RGB-D):

```xml
<sensor name="rgbd_camera" type="depth_camera">
  <update_rate>30</update_rate>
  <camera name="depth_camera">
    <horizontal_fov>1.3962634</horizontal_fov>
    <image>
      <width>640</width>
      <height>480</height>
      <format>R_FLOAT32</format>  <!-- Depth in meters -->
    </image>
    <clip>
      <near>0.1</near>
      <far>8.0</far>
    </clip>
  </camera>
  <topic>/camera/depth</topic>
</sensor>
```

**Verify Camera in ROS 2**:

```bash
# Bridge camera topic
ros2 run ros_gz_bridge parameter_bridge \
  /camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image

# View image in RViz
ros2 run rviz2 rviz2

# Or use rqt_image_view
ros2 run rqt_image_view rqt_image_view
```

### 5.2 IMU Sensor

**IMU Configuration**:

```xml
<link name="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>  <!-- 100 Hz typical for control -->
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.009</stddev>  <!-- 0.5 deg/s noise -->
            <bias_mean>0.00008</bias_mean>
            <bias_stddev>0.00004</bias_stddev>
          </noise>
        </x>
        <!-- Repeat for y, z -->
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>  <!-- 0.017 m/s² noise (typical IMU) -->
            <bias_mean>0.1</bias_mean>
            <bias_stddev>0.008</bias_stddev>
          </noise>
        </x>
        <!-- Repeat for y, z -->
      </linear_acceleration>
    </imu>
    <topic>/imu</topic>
  </sensor>
</link>
```

**Key Parameters**:
- `stddev`: White noise (random at each timestep)
- `bias_mean`: Constant offset (simulates calibration error)
- `bias_stddev`: Drift over time (random walk)

**Realistic IMU Specs** (from datasheets):

| IMU Type | Gyro Noise (deg/s) | Accel Noise (m/s²) | Update Rate |
|----------|-------------------|-------------------|-------------|
| **MPU-6050 (Budget)** | 0.5 | 0.03 | 100 Hz |
| **BMI088 (Mid-range)** | 0.1 | 0.015 | 400 Hz |
| **ADIS16505 (Premium)** | 0.005 | 0.005 | 2000 Hz |

### 5.3 LiDAR Sensor

**2D LiDAR (Laser Scanner)**:

```xml
<sensor name="lidar" type="gpu_lidar">
  <update_rate>10</update_rate>
  <lidar>
    <scan>
      <horizontal>
        <samples>360</samples>  <!-- 1° angular resolution -->
        <resolution>1.0</resolution>
        <min_angle>-3.14159</min_angle>  <!-- -180° -->
        <max_angle>3.14159</max_angle>   <!-- +180° -->
      </horizontal>
    </scan>
    <range>
      <min>0.08</min>
      <max>10.0</max>
      <resolution>0.01</resolution>
    </range>
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>  <!-- 1 cm noise -->
    </noise>
  </lidar>
  <topic>/scan</topic>
  <visualize>true</visualize>
</sensor>
```

**3D LiDAR** (Velodyne-like):

```xml
<sensor name="lidar_3d" type="gpu_lidar">
  <update_rate>10</update_rate>
  <lidar>
    <scan>
      <horizontal>
        <samples>1800</samples>  <!-- 0.2° resolution -->
        <resolution>1.0</resolution>
        <min_angle>-3.14159</min_angle>
        <max_angle>3.14159</max_angle>
      </horizontal>
      <vertical>
        <samples>16</samples>  <!-- 16 laser beams -->
        <resolution>1.0</resolution>
        <min_angle>-0.2618</min_angle>  <!-- -15° -->
        <max_angle>0.2618</max_angle>   <!-- +15° -->
      </vertical>
    </scan>
    <range>
      <min>0.1</min>
      <max>100.0</max>
    </range>
  </lidar>
  <topic>/lidar/points</topic>
</sensor>
```

### 5.4 Contact Sensors

**Force/Torque Sensor** (for feet):

```xml
<link name="left_foot">
  <sensor name="left_foot_contact" type="contact">
    <contact>
      <collision>left_foot_collision</collision>
    </contact>
    <update_rate>100</update_rate>
    <topic>/left_foot/contact</topic>
  </sensor>
</link>
```

**Contact State** (boolean contact detection):

```xml
<sensor name="bumper" type="contact">
  <contact>
    <collision>torso_collision</collision>
  </contact>
  <plugin name="gazebo_ros_bumper" filename="libgazebo_ros_bumper.so">
    <ros>
      <namespace>/humanoid</namespace>
      <remapping>~/out:=bumper_states</remapping>
    </ros>
    <frame_name>world</frame_name>
  </plugin>
</sensor>
```

**Verify Contact in ROS 2**:

```bash
ros2 topic echo /left_foot/contact

# Expected output (when foot touches ground):
# contacts:
# - collision1: left_foot_collision
#   collision2: ground_plane::link::collision
#   position: {x: 0.0, y: 0.1, z: 0.0}
#   normal: {x: 0.0, y: 0.0, z: 1.0}
#   depth: 0.001
#   wrench:
#     force: {x: 0.0, y: 0.0, z: 150.5}  # ~15 kg * 9.81 m/s²
#     torque: {x: 0.0, y: 0.0, z: 0.0}
```

---

## 6. Hands-On Lab: Humanoid in Gazebo with Camera and IMU

**Objective**: Spawn the Chapter 4 humanoid in Gazebo, add camera and IMU sensors, verify sensor data in ROS 2.

**Time**: 4-6 hours

### Lab Setup

```bash
# Create workspace
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake gazebo_humanoid_lab \
  --dependencies ros_gz_sim ros_gz_bridge sensor_msgs

cd gazebo_humanoid_lab
mkdir worlds models launch config
```

### Step 1: Create Gazebo World (1 hour)

**File**: `worlds/humanoid_world.sdf`

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <world name="humanoid_world">

    <!-- Physics -->
    <physics name="dart_physics" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <dart>
        <solver>
          <solver_type>pgs</solver_type>
        </solver>
        <collision_detector>bullet</collision_detector>
      </dart>
    </physics>

    <!-- Lighting -->
    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1.0 1.0 1.0 1.0</diffuse>
      <specular>0.5 0.5 0.5 1.0</specular>
      <direction>-0.5 0.1 -0.9</direction>
      <cast_shadows>true</cast_shadows>
    </light>

    <!-- Ground Plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane><normal>0 0 1</normal></plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>1.0</mu>
                <mu2>1.0</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane><normal>0 0 1</normal><size>100 100</size></plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- Include Humanoid Model -->
    <include>
      <uri>model://humanoid_with_sensors</uri>
      <pose>0 0 1.0 0 0 0</pose>
    </include>

  </world>
</sdf>
```

### Step 2: Add Sensors to Humanoid SDF (2 hours)

**File**: `models/humanoid_with_sensors/model.sdf`

Start with Chapter 4's `humanoid_12dof.urdf`, convert to SDF, then add sensors:

```bash
# Convert URDF to SDF
cd ~/ros2_ws/src/robot-book-code/chapter-04-urdf-sdf
gz sdf -p urdf/humanoid_12dof.urdf > ~/ros2_ws/src/gazebo_humanoid_lab/models/humanoid_with_sensors/model.sdf
```

**Edit** `model.sdf` to add IMU to torso:

```xml
<!-- Inside <model name="humanoid_12dof"> -->
<!-- After base_link definition -->

<link name="imu_link">
  <pose relative_to="base_link">0 0 0 0 0 0</pose>
  <inertial>
    <mass>0.01</mass>
    <inertia><ixx>0.00001</ixx><iyy>0.00001</iyy><izz>0.00001</izz></inertia>
  </inertial>

  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.009</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.009</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.009</stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
    <topic>/imu</topic>
  </sensor>
</link>

<joint name="imu_joint" type="fixed">
  <parent>base_link</parent>
  <child>imu_link</child>
  <pose>0 0 0 0 0 0</pose>
</joint>
```

**Add Camera to Head**:

```xml
<link name="camera_link">
  <pose relative_to="head">0.13 0 0 0 0 0</pose>
  <inertial>
    <mass>0.1</mass>
    <inertia><ixx>0.00001</ixx><iyy>0.00001</iyy><izz>0.00001</izz></inertia>
  </inertial>

  <visual name="visual">
    <geometry>
      <box><size>0.02 0.08 0.02</size></box>
    </geometry>
    <material>
      <ambient>0.1 0.1 0.1 1</ambient>
      <diffuse>0.1 0.1 0.1 1</diffuse>
    </material>
  </visual>

  <sensor name="camera" type="camera">
    <update_rate>30</update_rate>
    <camera name="head_camera">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.02</near>
        <far>10.0</far>
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>
      </noise>
    </camera>
    <always_on>true</always_on>
    <visualize>true</visualize>
    <topic>/camera/image_raw</topic>
  </sensor>
</link>

<joint name="camera_joint" type="fixed">
  <parent>head</parent>
  <child>camera_link</child>
  <pose>0.13 0 0 0 0 0</pose>
</joint>
```

### Step 3: Create Model Configuration (15 minutes)

**File**: `models/humanoid_with_sensors/model.config`

```xml
<?xml version="1.0"?>
<model>
  <name>Humanoid with Sensors</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>

  <author>
    <name>Your Name</name>
    <email>your.email@example.com</email>
  </author>

  <description>
    12-DOF humanoid robot with RGB camera and IMU sensor for Gazebo simulation.
  </description>
</model>
```

### Step 4: Create Launch File (30 minutes)

**File**: `launch/gazebo_humanoid.launch.py`

```python
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('gazebo_humanoid_lab')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_file = os.path.join(pkg_share, 'worlds', 'humanoid_world.sdf')

    # Set Gazebo model path
    model_path = os.path.join(pkg_share, 'models')
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] += os.pathsep + model_path
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = model_path

    # Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # ROS 2 ↔ Gazebo Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
        ],
        output='screen',
    )

    # Robot State Publisher (for TF frames)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': open(os.path.join(
                get_package_share_directory('robot_book_code'),
                'chapter-04-urdf-sdf', 'urdf', 'humanoid_12dof.urdf'
            )).read()
        }]
    )

    return LaunchDescription([
        gazebo,
        bridge,
        robot_state_publisher,
    ])
```

### Step 5: Build and Run (30 minutes)

```bash
# Build workspace
cd ~/ros2_ws
colcon build --packages-select gazebo_humanoid_lab
source install/setup.bash

# Launch Gazebo with humanoid
ros2 launch gazebo_humanoid_lab gazebo_humanoid.launch.py
```

**Expected Result**:
- Gazebo GUI opens with humanoid standing on ground plane
- Robot should be stable (not falling or vibrating)
- Camera view visible in Gazebo (green frustum)

### Step 6: Verify Sensor Data (1 hour)

**Terminal 1**: Check ROS 2 topics

```bash
ros2 topic list
# Expected:
# /camera/image_raw
# /imu
# /clock
# /joint_states
```

**Terminal 2**: Monitor IMU data

```bash
ros2 topic echo /imu --field linear_acceleration
# Expected (robot standing still):
# x: ~0.0 (small noise)
# y: ~0.0
# z: ~9.81 (gravity)
```

**Terminal 3**: View camera image

```bash
ros2 run rqt_image_view rqt_image_view
# Select topic: /camera/image_raw
# Should see camera view (ground plane, sky)
```

**Terminal 4**: Check topic rates

```bash
ros2 topic hz /camera/image_raw
# Expected: average rate: 30.0 (±0.5 Hz)

ros2 topic hz /imu
# Expected: average rate: 100.0 (±1.0 Hz)
```

### Step 7: Debugging (1-2 hours)

**Issue 1**: Robot falls through floor
- **Diagnosis**: Run `gz topic -e -t /stats` → Check `real_time_factor < 0.5`
- **Fix**: Reduce physics complexity
  ```xml
  <max_step_size>0.002</max_step_size>  <!-- Increase timestep -->
  ```

**Issue 2**: No sensor data on ROS 2 topics
- **Diagnosis**: Bridge not running or wrong topic names
- **Fix**: Check Gazebo topic names
  ```bash
  gz topic -l | grep camera
  # If output is /world/humanoid_world/model/humanoid/link/camera_link/sensor/camera/image
  # Update bridge arguments to match full path
  ```

**Issue 3**: Robot vibrating/unstable
- **Diagnosis**: Contact stiffness too low
- **Fix**: Increase contact parameters in collision surfaces
  ```xml
  <contact>
    <ode>
      <kp>1e7</kp>  <!-- Increase from 1e6 -->
      <kd>1e4</kd>  <!-- Increase from 1e3 -->
    </ode>
  </contact>
  ```

---

## 7. End-of-Chapter Project: Custom Gazebo World with Obstacles

**Objective**: Create a Gazebo world with randomized obstacles (boxes, cylinders) for humanoid navigation testing.

**Requirements**:

1. **World Layout**:
   - 10m × 10m ground plane
   - 5-10 randomly placed obstacles (boxes: 0.5m × 0.5m × 1.0m)
   - Directional lighting with shadows
   - Textured ground (not plain gray)

2. **Humanoid**:
   - Spawn at origin (0, 0, 1.0)
   - Camera and IMU sensors
   - Contact sensors on both feet

3. **ROS 2 Integration**:
   - All sensor data bridged to ROS 2
   - Launch file starts Gazebo + bridge + RViz
   - RViz shows: robot model, camera view, TF frames, laser scan (if added)

4. **Validation**:
   - Simulation runs at RTF > 0.9
   - All sensors publish at expected rates
   - Contact sensors detect when feet touch ground

**Starter Code** (procedural obstacle generation):

```python
# scripts/generate_world.py
import random
import sys

def generate_world(num_obstacles=8):
    sdf = """<?xml version="1.0"?>
<sdf version="1.9">
  <world name="obstacle_world">
    <physics name="default_physics" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>1.0 1.0 1.0 1.0</diffuse>
      <direction>-0.5 0.1 -0.9</direction>
      <cast_shadows>true</cast_shadows>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal></plane></geometry>
          <surface>
            <friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>10 10</size></plane></geometry>
          <material>
            <ambient>0.5 0.5 0.5 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

"""

    # Generate random obstacles
    for i in range(num_obstacles):
        x = random.uniform(-4.0, 4.0)
        y = random.uniform(-4.0, 4.0)
        height = random.uniform(0.5, 2.0)

        sdf += f"""
    <model name="obstacle_{i}">
      <pose>{x} {y} {height/2} 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>0.5 0.5 {height}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>0.5 0.5 {height}</size></box></geometry>
          <material>
            <ambient>0.8 0.3 0.1 1</ambient>
            <diffuse>0.9 0.4 0.2 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
"""

    sdf += """
    <include>
      <uri>model://humanoid_with_sensors</uri>
      <pose>0 0 1.0 0 0 0</pose>
    </include>
  </world>
</sdf>
"""

    return sdf

if __name__ == '__main__':
    num_obstacles = int(sys.argv[1]) if len(sys.argv) > 1 else 8
    world_sdf = generate_world(num_obstacles)

    with open('worlds/obstacle_world.sdf', 'w') as f:
        f.write(world_sdf)

    print(f"Generated world with {num_obstacles} obstacles: worlds/obstacle_world.sdf")
```

**Run**:

```bash
python3 scripts/generate_world.py 10
ros2 launch gazebo_humanoid_lab gazebo_humanoid.launch.py world:=obstacle_world.sdf
```

---

## Summary

In this chapter, you learned:

1. **Simulator Selection**: Gazebo for ROS 2 integration, Isaac Sim for VLA training
2. **Gazebo Architecture**: Client-server model, plugin system, Ignition Transport
3. **ROS 2 Integration**: `ros_gz_bridge` for topic mapping, launch file setup
4. **Physics Tuning**: Time step, solver iterations, contact properties for stability
5. **Sensor Simulation**: Camera, IMU, LiDAR, contact sensors with realistic noise models
6. **Debugging**: RTF monitoring, collision geometry optimization, sensor data verification

**Key Takeaways**:

- Always check real-time factor (RTF > 0.9 for real-time control)
- Use DART physics for humanoids (better contact stability than ODE/Bullet)
- Add realistic sensor noise (IMU: 0.5 deg/s gyro, 0.017 m/s² accel)
- Start with simple collision geometry (boxes), add meshes only if needed
- Bridge only necessary topics (reduces network overhead)

**Next Chapter Preview**: Chapter 6 covers NVIDIA Isaac Sim—installation, URDF import, PhysX configuration, and your first GPU-parallelized simulation with 100+ humanoid robots.

---

## Further Reading

**Gazebo Documentation**:
- Gazebo Harmonic Docs: https://gazebosim.org/docs/harmonic
- SDF Specification: http://sdformat.org/spec
- Gazebo Tutorials: https://gazebosim.org/docs/harmonic/tutorials

**ROS 2 Integration**:
- ros_gz Repository: https://github.com/gazebosim/ros_gz
- ros_gz_bridge Tutorial: https://gazebosim.org/docs/harmonic/ros2_integration

**Physics Engines**:
- ODE Manual: https://ode.org/wiki/index.php/Manual
- Bullet Physics: https://pybullet.org/wordpress/
- DART: https://dartsim.github.io/

**Sensor Simulation**:
- Gazebo Sensors: https://gazebosim.org/api/sensors/8/tutorials.html
- IMU Noise Models: https://www.nxp.com/docs/en/application-note/AN5087.pdf

**Example Worlds**:
- Gazebo Fuel (Model Database): https://app.gazebosim.org/fuel/models
- ROS 2 Example Worlds: https://github.com/gazebosim/gz_sim_demos

---

**End of Chapter 5**
