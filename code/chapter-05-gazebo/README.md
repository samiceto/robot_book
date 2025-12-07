# Chapter 5: Gazebo Basics

This repository contains code examples for Chapter 5 of "Physical AI & Humanoid Robotics with ROS 2, Isaac Sim, and VLAs".

## Overview

Learn to simulate humanoid robots in Gazebo Harmonic with realistic physics, sensors, and ROS 2 integration. This chapter covers:

- Gazebo vs. Isaac Sim comparison and when to use each
- Gazebo client-server architecture and plugin system
- Physics configuration (DART/Bullet/ODE engines, contact parameters)
- Sensor plugins (camera, IMU, LiDAR, contact sensors)
- ROS 2 bridge (`ros_gz_bridge`) for seamless integration
- Custom world generation with randomized obstacles

## Repository Structure

```
chapter-05-gazebo/
├── worlds/
│   ├── humanoid_world.sdf         # Basic world with ground plane and lighting
│   └── obstacle_world.sdf         # Generated world with random obstacles (see scripts/)
├── launch/
│   └── gazebo_humanoid.launch.py  # Launch Gazebo + ROS 2 bridge
├── config/
│   └── bridge_config.yaml         # ROS 2 ↔ Gazebo topic mappings
├── scripts/
│   └── generate_world.py          # Generate random obstacle worlds
└── README.md                       # This file
```

## Prerequisites

### System Requirements

- Ubuntu 22.04 LTS (ROS 2 Iron) or Ubuntu 24.04 LTS (ROS 2 Jazzy)
- Gazebo Harmonic (gz-harmonic)
- ROS 2 (Iron or Jazzy) installed
- Python 3.10+

### Install Dependencies

```bash
# Install Gazebo Harmonic
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update
sudo apt install gz-harmonic

# Install ROS 2 Gazebo integration
sudo apt install ros-${ROS_DISTRO}-ros-gz \
                 ros-${ROS_DISTRO}-ros-gz-bridge \
                 ros-${ROS_DISTRO}-ros-gz-sim \
                 ros-${ROS_DISTRO}-robot-state-publisher \
                 ros-${ROS_DISTRO}-rviz2

# Verify installation
gz sim --version
# Expected: Gazebo Sim, version 8.x.x (Harmonic)
```

## Usage

### 1. Generate Random Obstacle World

```bash
cd ~/ros2_ws/src/robot-book-code/chapter-05-gazebo

# Generate world with 10 random obstacles
python3 scripts/generate_world.py 10

# Output: worlds/obstacle_world.sdf
```

### 2. Launch Gazebo with Humanoid (Manual)

```bash
# Launch Gazebo with basic world
gz sim worlds/humanoid_world.sdf

# Or with obstacle world
gz sim worlds/obstacle_world.sdf
```

### 3. Launch with ROS 2 Integration

```bash
# Build workspace
cd ~/ros2_ws
colcon build --packages-select chapter-05-gazebo
source install/setup.bash

# Launch Gazebo + ROS 2 bridge
ros2 launch chapter-05-gazebo gazebo_humanoid.launch.py

# Launch with obstacle world
ros2 launch chapter-05-gazebo gazebo_humanoid.launch.py world:=obstacle_world.sdf
```

**Expected Result**:
- Gazebo GUI opens with humanoid standing on ground plane
- Robot should be stable (not falling or vibrating)
- Real-time factor (RTF) should be > 0.9 (check bottom-right corner)

### 4. Verify ROS 2 Topics

```bash
# List available topics
ros2 topic list
# Expected:
# /clock
# /humanoid/camera/image_raw
# /humanoid/camera/camera_info
# /humanoid/imu

# Check topic rates
ros2 topic hz /humanoid/camera/image_raw
# Expected: average rate: 30.0 Hz

ros2 topic hz /humanoid/imu
# Expected: average rate: 100.0 Hz

# Echo IMU data (should show gravity when robot is stationary)
ros2 topic echo /humanoid/imu --field linear_acceleration
# Expected:
# x: ~0.0
# y: ~0.0
# z: ~9.81 (gravity)
```

### 5. View Camera Feed

```bash
# Using rqt_image_view
ros2 run rqt_image_view rqt_image_view

# Select topic: /humanoid/camera/image_raw
# Should see camera view (ground plane, obstacles if present)
```

### 6. Monitor Simulation Performance

```bash
# Check simulation statistics
gz topic -e -t /stats

# Expected output:
# sim_time: {sec: 10, nsec: 523000000}
# real_time: {sec: 10, nsec: 500000000}
# iterations: 10523
# real_time_factor: 1.002  # Good! Close to 1.0
```

## Configuration

### Physics Tuning

Edit `worlds/humanoid_world.sdf` to adjust physics parameters:

```xml
<physics name="dart_physics" type="dart">
  <!-- Time step (smaller = more accurate, slower) -->
  <max_step_size>0.001</max_step_size>

  <!-- Real-time factor (1.0 = real-time) -->
  <real_time_factor>1.0</real_time_factor>

  <dart>
    <solver>
      <!-- Solver type: pgs (fast), dantzig (accurate) -->
      <solver_type>pgs</solver_type>

      <!-- Constraint parameters -->
      <constraint_cfm>0.00001</constraint_cfm>  <!-- Softness -->
      <constraint_erp>0.2</constraint_erp>      <!-- Stabilization -->
    </solver>

    <!-- Collision detector: bullet (fast), ode (stable) -->
    <collision_detector>bullet</collision_detector>
  </dart>
</physics>
```

**Recommended Settings for Humanoids**:
- **Physics Engine**: DART (best contact stability for bipedal locomotion)
- **Time Step**: 0.001s (1 kHz) for fast contacts
- **Solver**: PGS (Projected Gauss-Seidel) for speed, Dantzig for accuracy

### Contact Properties

Adjust friction and stiffness in collision surfaces:

```xml
<collision name="foot_collision">
  <surface>
    <friction>
      <ode>
        <mu>1.0</mu>    <!-- Coulomb friction (rubber = 1.0-1.5) -->
        <mu2>1.0</mu2>
      </ode>
    </friction>
    <contact>
      <ode>
        <kp>1e7</kp>  <!-- Stiffness (higher = less penetration) -->
        <kd>1e4</kd>  <!-- Damping -->
      </ode>
    </contact>
  </surface>
</collision>
```

**Material Presets**:
| Material | μ (Friction) | kp (Stiffness) | Use Case |
|----------|-------------|----------------|----------|
| Rubber (foot) | 1.0-1.5 | 1e7 | Bipedal walking |
| Metal | 0.5-0.8 | 1e6 | Manipulation |
| Ice | 0.02-0.05 | 1e6 | Slippery terrain |

### Bridge Configuration

Modify `config/bridge_config.yaml` to add/remove bridged topics:

```yaml
# Add custom topic
- topic_name: "/my_sensor/data"
  ros_type_name: "std_msgs/msg/Float64"
  gz_type_name: "gz.msgs.Double"
  direction: GZ_TO_ROS
```

**Launch with YAML Config**:

```bash
ros2 run ros_gz_bridge parameter_bridge --ros-args \
  -p config_file:=$(ros2 pkg prefix chapter_05_gazebo)/share/chapter_05_gazebo/config/bridge_config.yaml
```

## Troubleshooting

### Issue 1: Robot Falls Through Floor

**Symptoms**: Robot sinks into ground plane, falls infinitely

**Diagnosis**:
```bash
gz topic -e -t /stats
# Check real_time_factor < 0.5 (simulation too slow)
```

**Fix**:
1. Increase time step (trade accuracy for speed):
   ```xml
   <max_step_size>0.002</max_step_size>  <!-- Was 0.001 -->
   ```

2. Increase contact stiffness:
   ```xml
   <contact><ode><kp>1e8</kp></ode></contact>  <!-- Was 1e7 -->
   ```

3. Simplify collision geometry (use boxes instead of meshes)

### Issue 2: No Sensor Data on ROS 2 Topics

**Symptoms**: Topics exist but no messages received

**Diagnosis**:
```bash
# Check Gazebo topic names
gz topic -l | grep camera
# Example output: /world/humanoid_world/model/humanoid/link/camera_link/sensor/camera/image

# Check if bridge is running
ros2 node list | grep bridge
```

**Fix**:
1. Update bridge arguments to match **full Gazebo topic path**:
   ```bash
   ros2 run ros_gz_bridge parameter_bridge \
     /world/humanoid_world/model/humanoid/link/camera_link/sensor/camera/image@sensor_msgs/msg/Image[gz.msgs.Image
   ```

2. Verify sensor `<topic>` tag in SDF matches bridge configuration

### Issue 3: Robot Vibrating/Unstable

**Symptoms**: Robot shakes violently, joints oscillate

**Diagnosis**: Contact damping too low or time step too large

**Fix**:
1. Increase joint damping in URDF/SDF:
   ```xml
   <joint name="hip_joint">
     <dynamics damping="1.0" friction="0.2"/>  <!-- Increase damping -->
   </joint>
   ```

2. Reduce time step:
   ```xml
   <max_step_size>0.0005</max_step_size>  <!-- Was 0.001 -->
   ```

3. Increase contact damping:
   ```xml
   <contact><ode><kd>1e5</kd></ode></contact>  <!-- Was 1e4 -->
   ```

### Issue 4: Low Real-Time Factor (RTF < 0.9)

**Symptoms**: Simulation runs slower than real-time

**Diagnosis**:
```bash
gz topic -e -t /stats
# real_time_factor: 0.65  # Too slow!
```

**Optimization Steps**:

1. **Run Headless** (no GUI rendering):
   ```bash
   gz sim -r -s worlds/humanoid_world.sdf
   ```

2. **Reduce Camera Resolution**:
   ```xml
   <camera>
     <image>
       <width>320</width>  <!-- Was 640 -->
       <height>240</height>  <!-- Was 480 -->
     </image>
   </camera>
   ```

3. **Disable Shadows**:
   - In Gazebo GUI: View → Shadows → Off

4. **Switch to Faster Physics Engine**:
   ```xml
   <physics type="bullet">  <!-- Was "dart" -->
   ```
   Note: Bullet is faster but less accurate for humanoids

5. **Simplify Collision Meshes**:
   - Replace complex meshes with primitive shapes (boxes, cylinders)

### Issue 5: Gazebo Crashes on Startup

**Symptoms**: `gz sim` crashes with segmentation fault

**Common Causes**:
1. **Invalid SDF**: Run validation first
   ```bash
   gz sdf --check worlds/humanoid_world.sdf
   ```

2. **Missing Model**: Ensure model paths are correct
   ```bash
   export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:$HOME/ros2_ws/src/robot-book-code/chapter-05-gazebo/models
   ```

3. **GPU Driver Issues**: Update NVIDIA drivers
   ```bash
   nvidia-smi  # Check driver version (should be 550+)
   ```

## Exercises

### Exercise 1: Add LiDAR Sensor

Add a 2D LiDAR sensor to the humanoid's head:

1. Edit humanoid model SDF, add to `<link name="head">`:
   ```xml
   <sensor name="lidar" type="gpu_lidar">
     <update_rate>10</update_rate>
     <lidar>
       <scan>
         <horizontal>
           <samples>360</samples>
           <min_angle>-3.14159</min_angle>
           <max_angle>3.14159</max_angle>
         </horizontal>
       </scan>
       <range>
         <min>0.08</min>
         <max>10.0</max>
       </range>
     </lidar>
     <topic>/scan</topic>
   </sensor>
   ```

2. Add bridge mapping:
   ```bash
   ros2 run ros_gz_bridge parameter_bridge \
     /scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan
   ```

3. Verify in RViz:
   ```bash
   ros2 run rviz2 rviz2
   # Add display: LaserScan, topic: /scan
   ```

### Exercise 2: Tune Physics for Faster Simulation

Goal: Achieve RTF > 1.5 (faster than real-time) while maintaining stability

Steps:
1. Increase time step to 0.002s
2. Switch to Bullet physics
3. Reduce camera resolution to 160x120
4. Run headless: `gz sim -r -s world.sdf`
5. Measure RTF: `gz topic -e -t /stats | grep real_time_factor`

### Exercise 3: Create Multi-Room World

Generate a world with walls creating separate rooms:

1. Extend `generate_world.py` to add wall models
2. Walls should be 2m high, 0.1m thick
3. Create 3 rooms: 3m × 3m each
4. Place humanoid in center room
5. Add obstacles in each room

## Further Reading

- **Gazebo Documentation**: https://gazebosim.org/docs/harmonic
- **ROS 2 Integration**: https://github.com/gazebosim/ros_gz
- **SDF Specification**: http://sdformat.org/spec
- **Physics Tuning Guide**: https://gazebosim.org/docs/harmonic/physics
- **Sensor Plugins**: https://gazebosim.org/api/sensors/8/tutorials.html

## License

MIT License - See LICENSE file in repository root.

## Support

For issues, questions, or contributions:
- GitHub Issues: https://github.com/yourusername/robot-book-code/issues
- Book Website: https://physicalai-book.com
- Discussions: https://github.com/yourusername/robot-book-code/discussions
