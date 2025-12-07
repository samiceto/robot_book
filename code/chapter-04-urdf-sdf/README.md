# Chapter 4: Robot Description - URDF, SDF, and Xacro

This repository contains code examples for Chapter 4 of "Physical AI & Humanoid Robotics with ROS 2, Isaac Sim, and VLAs".

## Overview

Learn to create robot descriptions using URDF, Xacro, and SDF formats. This chapter covers:

- URDF fundamentals (links, joints, transforms, inertia)
- Building complete humanoid robot descriptions
- Xacro macros for modular, parameterized robots
- URDF→SDF conversion for Gazebo simulation
- Validation tools: `check_urdf`, `urdf_to_graphviz`, RViz, Gazebo, Isaac Sim

## Repository Structure

```
chapter-04-urdf-sdf/
├── urdf/
│   ├── humanoid_12dof.urdf      # Complete 12-DOF humanoid (raw URDF)
│   ├── humanoid.xacro            # Same robot using Xacro macros
│   └── macros/
│       ├── inertia.xacro         # Inertia calculation macros
│       └── leg.xacro             # Parameterized leg macro
├── config/
│   └── robot_dimensions.yaml    # Robot parameters (masses, lengths)
├── scripts/
│   └── validate.sh               # Automated validation script
└── README.md                     # This file
```

## Prerequisites

### System Requirements

- Ubuntu 22.04 LTS or 24.04 LTS
- ROS 2 Iron or Jazzy installed (see Chapter 2)
- Python 3.10+

### Install Dependencies

```bash
# URDF validation tools
sudo apt install liburdfdom-tools

# ROS 2 packages
sudo apt install ros-${ROS_DISTRO}-xacro \
                 ros-${ROS_DISTRO}-robot-state-publisher \
                 ros-${ROS_DISTRO}-joint-state-publisher-gui \
                 ros-${ROS_DISTRO}-rviz2

# Gazebo (optional, for SDF validation)
sudo apt install gz-harmonic  # or gz-garden
```

## Usage

### 1. Validate URDF Files

```bash
# Navigate to repository
cd ~/ros2_ws/src/robot-book-code/chapter-04-urdf-sdf

# Run validation script
chmod +x scripts/validate.sh
./scripts/validate.sh
```

**Expected Output:**

```
========================================
URDF/SDF Validation Script
========================================

Checking dependencies...
[PASS] check_urdf found
[PASS] ROS 2 found
[PASS] Gazebo found

========================================
Validating URDF Files
========================================

Testing: humanoid_12dof.urdf
[PASS] humanoid_12dof.urdf is valid
[PASS]   Root link is base_link
[PASS]   Found 9 child links

========================================
Testing Xacro Conversion
========================================

Testing: humanoid.xacro
[PASS] Xacro conversion successful
[PASS] Converted URDF is valid

========================================
Validation Summary
========================================
Tests Passed: 8
Tests Failed: 0

All validations passed!
```

### 2. Manual Validation with check_urdf

```bash
# Check URDF structure
check_urdf urdf/humanoid_12dof.urdf

# Expected output:
# robot name is: humanoid_12dof
# ---------- Successfully Parsed XML ---------------
# root Link: base_link has 3 child(ren)
#     child(1):  head
#     child(2):  left_hip
#         child(1):  left_knee
#             child(1):  left_ankle
#     child(3):  right_hip
#         child(1):  right_knee
#             child(1):  right_ankle
```

### 3. Convert Xacro to URDF

```bash
# Convert humanoid.xacro to URDF
ros2 run xacro xacro urdf/humanoid.xacro > /tmp/humanoid_expanded.urdf

# Validate converted file
check_urdf /tmp/humanoid_expanded.urdf
```

### 4. Visualize Kinematic Tree

```bash
# Generate PDF visualization of kinematic tree
urdf_to_graphviz urdf/humanoid_12dof.urdf

# Output: humanoid_12dof.pdf
# Open with your PDF viewer
evince humanoid_12dof.pdf  # or xdg-open humanoid_12dof.pdf
```

### 5. Visualize in RViz

```bash
# Terminal 1: Publish robot description and joint states
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(xacro urdf/humanoid.xacro)"

# Terminal 2: Launch joint state publisher GUI
ros2 run joint_state_publisher_gui joint_state_publisher_gui \
  --ros-args -p robot_description:="$(xacro urdf/humanoid.xacro)"

# Terminal 3: Launch RViz
ros2 run rviz2 rviz2
```

**In RViz:**

1. Set "Fixed Frame" to `base_link`
2. Add display: "RobotModel"
3. Use joint sliders in GUI to move robot

### 6. Convert to SDF for Gazebo

```bash
# Convert URDF to SDF
gz sdf -p urdf/humanoid_12dof.urdf > /tmp/humanoid.sdf

# Validate SDF
gz sdf --check /tmp/humanoid.sdf

# Launch Gazebo with SDF
gz sim /tmp/humanoid.sdf
```

### 7. Load in Isaac Sim

**Steps:**

1. Launch Isaac Sim:
   ```bash
   ~/.local/share/ov/pkg/isaac-sim-4.2.0/isaac-sim.sh
   ```

2. Import URDF:
   - Menu: "Isaac Utils" → "URDF Importer"
   - Browse to: `~/ros2_ws/src/robot-book-code/chapter-04-urdf-sdf/urdf/humanoid_12dof.urdf`
   - Settings:
     - **Import Inertia Tensor**: ✅
     - **Fix Base Link**: ❌ (allow free motion)
     - **Joint Drive Type**: Position
     - **Joint Drive Strength**: 10000

3. Click "Import"

4. Click "Play" to start physics simulation

## Robot Specifications

### Humanoid_12dof

**Degrees of Freedom**: 12
- **Legs**: 6 DOF (3 per leg: hip, knee, ankle)
- **Arms**: 4 DOF (2 per arm: shoulder, elbow)
- **Head**: 1 DOF (neck yaw)
- **Torso**: Fixed

**Total Mass**: ~30 kg
- Torso: 15.0 kg
- Head: 2.0 kg
- Arms: 2 × (1.5 + 1.0) = 5.0 kg
- Legs: 2 × (3.0 + 2.0 + 1.0) = 12.0 kg

**Dimensions**:
- Height: ~1.2 m (standing)
- Torso: 0.3m × 0.2m × 0.4m
- Leg length: 0.6 m (hip + knee)
- Arm length: 0.44 m (shoulder + elbow)

**Joint Limits**:
- Hip: ±90° (±1.57 rad)
- Knee: 0° to 135° (0 to 2.356 rad)
- Ankle: ±45° (±0.785 rad)
- Shoulder: ±90°
- Elbow: 0° to 135°
- Neck: ±45°

## Key Concepts

### URDF Structure

```xml
<robot name="...">
  <link name="...">
    <visual>...</visual>
    <collision>...</collision>
    <inertial>
      <mass value="..."/>
      <inertia ixx="..." iyy="..." izz="..."/>
    </inertial>
  </link>

  <joint name="..." type="revolute">
    <parent link="..."/>
    <child link="..."/>
    <origin xyz="..." rpy="..."/>
    <axis xyz="..."/>
    <limit lower="..." upper="..." effort="..." velocity="..."/>
    <dynamics damping="..." friction="..."/>
  </joint>
</robot>
```

### Xacro Macros

```xml
<xacro:property name="leg_length" value="0.6"/>

<xacro:macro name="leg" params="prefix reflect">
  <link name="${prefix}_hip">
    <origin xyz="0 ${reflect * 0.1} -0.2"/>
    ...
  </link>
</xacro:macro>

<xacro:leg prefix="left" reflect="1"/>
<xacro:leg prefix="right" reflect="-1"/>
```

### Inertia Calculation

**Box** (width `w`, depth `d`, height `h`):
- `ixx = m/12 * (d² + h²)`
- `iyy = m/12 * (w² + h²)`
- `izz = m/12 * (w² + d²)`

**Cylinder** (radius `r`, length `l`, axis along Z):
- `ixx = m/12 * (3r² + l²)`
- `iyy = m/12 * (3r² + l²)`
- `izz = m/2 * r²`

**Sphere** (radius `r`):
- `ixx = iyy = izz = 2mr²/5`

## Troubleshooting

### Error: "Link has no inertia tag"

**Cause**: Missing `<inertial>` tag in link definition.

**Fix**: Add inertia to every link (even visual-only links need minimal mass):

```xml
<link name="camera_link">
  <visual>...</visual>
  <inertial>
    <mass value="0.01"/>
    <inertia ixx="0.00001" iyy="0.00001" izz="0.00001"/>
  </inertial>
</link>
```

### Error: "Joint references unknown link"

**Cause**: Typo in `<parent>` or `<child>` link name, or links defined after joint.

**Fix**: Ensure links are defined before the joint that references them:

```xml
<link name="base_link">...</link>
<link name="child_link">...</link>
<joint name="my_joint" type="revolute">
  <parent link="base_link"/>
  <child link="child_link"/>
</joint>
```

### Error: "Cannot find package"

**Cause**: Xacro file uses `$(find package_name)` but package not in workspace.

**Fix**: Replace with absolute path or ensure package is built:

```bash
cd ~/ros2_ws
colcon build --packages-select humanoid_description
source install/setup.bash
```

### Robot Explodes in Gazebo/Isaac Sim

**Cause**: Inertia values too low or missing damping.

**Fix**:
1. Increase joint damping: `<dynamics damping="1.0"/>`
2. Check inertia is realistic (not 0.0001 for a 5kg link)
3. Verify COM matches visual geometry center

### Robot Falls Through Floor

**Cause**: Missing collision geometry.

**Fix**: Ensure every link has `<collision>` tags:

```xml
<link name="foot">
  <visual>...</visual>
  <collision>
    <geometry>
      <box size="0.15 0.08 0.05"/>
    </geometry>
  </collision>
</link>
```

## Exercises

### Exercise 1: Modify Robot Dimensions

Edit `config/robot_dimensions.yaml` to create a taller robot (1.5m height):

```yaml
leg:
  hip_length: 0.4   # was 0.3
  knee_length: 0.4  # was 0.3
```

Rebuild and validate:

```bash
ros2 run xacro xacro urdf/humanoid.xacro > /tmp/tall_humanoid.urdf
check_urdf /tmp/tall_humanoid.urdf
```

### Exercise 2: Add Camera Sensor

Add RGB camera to head in `urdf/humanoid.xacro`:

```xml
<link name="camera_link">
  <visual>
    <geometry><box size="0.02 0.08 0.02"/></geometry>
  </visual>
  <inertial>
    <mass value="0.1"/>
    <inertia ixx="0.00001" iyy="0.00001" izz="0.00001"/>
  </inertial>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="head"/>
  <child link="camera_link"/>
  <origin xyz="0.13 0 0" rpy="0 0 0"/>
</joint>
```

### Exercise 3: Create Arm Macro

Extract arm definitions into `urdf/macros/arm.xacro` similar to `leg.xacro`. Parameters: `prefix` (left/right), `reflect` (±1).

## Further Reading

- **URDF Specification**: http://wiki.ros.org/urdf/XML
- **Xacro Tutorial**: http://wiki.ros.org/urdf/Tutorials/Using%20Xacro%20to%20Clean%20Up%20a%20URDF%20File
- **SDF Specification**: http://sdformat.org/spec
- **Isaac Sim URDF Importer**: https://docs.omniverse.nvidia.com/isaacsim/latest/features/environment_setup/ext_omni_isaac_urdf.html
- **Inertia Formulas**: https://en.wikipedia.org/wiki/List_of_moments_of_inertia

## License

MIT License - See LICENSE file in repository root.

## Support

For issues, questions, or contributions:
- GitHub Issues: https://github.com/yourusername/robot-book-code/issues
- Book Website: https://physicalai-book.com
- Discussions: https://github.com/yourusername/robot-book-code/discussions
