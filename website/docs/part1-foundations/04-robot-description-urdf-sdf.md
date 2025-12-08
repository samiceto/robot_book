# Chapter 4: Robot Description - URDF, SDF, and Xacro for Humanoids

**Learning Objectives**

By the end of this chapter, you will be able to:

1. Explain the purpose and structure of robot description formats (URDF, SDF, Xacro)
2. Create URDF files defining links, joints, visual/collision geometry, and inertial properties
3. Build a complete humanoid robot description with realistic kinematics and dynamics
4. Use Xacro macros to create modular, parameterized robot descriptions
5. Convert between URDF and SDF formats and understand their differences
6. Validate robot descriptions using check_urdf, urdf_to_graphviz, and Gazebo tools
7. Debug common URDF/SDF errors (missing masses, broken kinematic chains, invalid joint limits)
8. Load and visualize robot descriptions in RViz and Isaac Sim

**Prerequisites**: Chapter 2 (ROS 2 installation), basic XML syntax, linear algebra (transforms, rotation matrices)

**Estimated Time**: 10-12 hours (5 hours reading, 5-7 hours hands-on lab)

---

## 1. Why Robot Descriptions Matter

### 1.1 The Role of Robot Descriptions in Physical AI

Every robot simulation, motion planner, and control system needs to answer fundamental questions:

- **Kinematics**: Where is my end-effector when my shoulder is at 45° and elbow at 90°?
- **Dynamics**: How much torque do I need to lift this 2 kg object with a 0.5m arm?
- **Collision Detection**: Will my robot's elbow hit the table if I execute this trajectory?
- **Sensor Placement**: Where is my camera's optical frame relative to my torso?

Robot description files provide the **single source of truth** for these questions. They define:

1. **Kinematic Structure**: Parent-child relationships between rigid bodies (links) connected by joints
2. **Physical Properties**: Mass, center of mass, moment of inertia for dynamics simulation
3. **Geometric Models**: Visual meshes for rendering and collision meshes for physics
4. **Sensor Specifications**: Camera intrinsics, LiDAR scan ranges, IMU noise parameters

For humanoid robots, accurate robot descriptions are **critical** because:

- **High DOF Complexity**: 30+ DOF requires precise joint limits and coupling constraints
- **Dynamic Stability**: Small inertia errors cause bipedal controllers to fail catastrophically
- **Sim-to-Real Transfer**: VLA models trained in simulation with wrong mass distribution won't transfer to real hardware
- **Safety**: Collision checking prevents expensive hardware damage during autonomous operation

### 1.2 URDF vs. SDF vs. USD: Format Comparison

The robotics ecosystem has three major robot description formats:

| Feature | URDF (Unified Robot Description Format) | SDF (Simulation Description Format) | USD (Universal Scene Description) |
|---------|----------------------------------------|-------------------------------------|-----------------------------------|
| **Primary Use Case** | ROS 2 ecosystem, simple robots | Gazebo simulation, multi-robot scenes | NVIDIA Omniverse, Isaac Sim |
| **Joint Types** | Revolute, Prismatic, Fixed, Continuous, Planar, Floating | All URDF types + Ball, Universal, Screw | All joint types + custom constraints |
| **Multi-Robot Support** | No (single robot only) | Yes (multiple `<model>` tags) | Yes (USD composition) |
| **Closed Loops** | No (strict tree topology) | Yes (with explicit constraints) | Yes (PhysX articulations) |
| **Plugin System** | Gazebo Classic plugins | Gazebo plugins + SDF extensions | Omniverse extensions + Python scripting |
| **Units** | SI (meters, kg, rad) | SI (explicit in schema) | Configurable (cm/m toggle in Isaac Sim) |
| **File Format** | XML | XML | Binary (`.usdc`) or ASCII (`.usda`) |

**Key Differences**:

- **URDF Limitations**:
  - Cannot define closed kinematic loops (e.g., parallel linkages, four-bar mechanisms)
  - No native support for multiple robots in one file
  - Joint friction/damping requires Gazebo-specific `<gazebo>` tags

- **SDF Advantages**:
  - Native support for multiple models, nested models, and model composition
  - More expressive physics (joint friction, spring-damper systems, contact properties)
  - Sensor definitions are first-class citizens (LiDAR, depth cameras with noise models)

- **USD for Isaac Sim**:
  - Industry-standard format (Pixar, used in film VFX pipelines)
  - Real-time ray tracing, physically-based rendering (PBR materials)
  - Tight integration with PhysX 5.x for accurate contact dynamics
  - Python scripting via Omniverse API for procedural generation

**Practical Workflow**:

1. **Author in URDF + Xacro**: Easiest to write, best ROS 2 tool support
2. **Convert to SDF**: Use `gz sdf -p robot.urdf > robot.sdf` for Gazebo simulation
3. **Import to USD**: Use Isaac Sim's URDF Importer for Omniverse workflows

In this chapter, we focus on **URDF + Xacro** as the authoring format, then show conversion to SDF and USD.

---

## 2. URDF Fundamentals

### 2.1 Anatomy of a URDF File

A URDF file is an XML document with three core elements:

1. **`<robot>`**: Root element containing the robot's name
2. **`<link>`**: Rigid body with visual, collision, and inertial properties
3. **`<joint>`**: Connection between two links with motion constraints

**Minimal Example** (2-link planar arm):

```xml
<?xml version="1.0"?>
<robot name="simple_arm">

  <!-- Link 1: Fixed Base -->
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
      <material name="gray">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.1" radius="0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.001" ixy="0.0" ixz="0.0"
               iyy="0.001" iyz="0.0" izz="0.002"/>
    </inertial>
  </link>

  <!-- Link 2: Upper Arm -->
  <link name="upper_arm">
    <visual>
      <origin xyz="0.25 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.5 0.08 0.08"/>
      </geometry>
      <material name="blue">
        <color rgba="0.2 0.2 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0.25 0 0" rpy="0 0 0"/>
      <geometry>
        <box size="0.5 0.08 0.08"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0.25 0 0" rpy="0 0 0"/>
      <mass value="2.0"/>
      <inertia ixx="0.02" ixy="0.0" ixz="0.0"
               iyy="0.17" iyz="0.0" izz="0.17"/>
    </inertial>
  </link>

  <!-- Joint 1: Shoulder (Revolute) -->
  <joint name="shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="upper_arm"/>
    <origin xyz="0 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100.0" velocity="2.0"/>
    <dynamics damping="0.5" friction="0.1"/>
  </joint>

</robot>
```

**Key Concepts**:

1. **`<origin>`**: Homogeneous transform (translation `xyz`, rotation `rpy` in radians)
   - For `<visual>` and `<collision>`: offset from link frame
   - For `<joint>`: transform from parent link to joint frame

2. **`<geometry>`**: Primitive shapes or mesh files
   - Primitives: `<box>`, `<cylinder>`, `<sphere>`
   - Meshes: `<mesh filename="package://my_robot/meshes/arm.stl" scale="1 1 1"/>`

3. **`<inertial>`**: Required for dynamic simulation
   - `<mass>`: kilograms
   - `<inertia>`: 3×3 inertia tensor (only upper triangle needed due to symmetry)

4. **`<joint>` types**:
   - `revolute`: Rotating hinge with joint limits
   - `prismatic`: Sliding joint (e.g., linear actuator)
   - `continuous`: Rotating hinge without limits (e.g., wheel)
   - `fixed`: Rigidly attached (0 DOF)
   - `floating`: 6-DOF free motion (rarely used, for mobile bases)
   - `planar`: 2D motion in a plane

### 2.2 Coordinate Frames and Transforms

URDF uses **right-handed coordinate systems** with transforms defined by:

```math
T = \begin{bmatrix}
R_{3 \times 3} & t_{3 \times 1} \\
0_{1 \times 3} & 1
\end{bmatrix}
```

Where:
- $R$ is the rotation matrix (computed from roll-pitch-yaw Euler angles)
- $t$ is the translation vector `[x, y, z]`

**Roll-Pitch-Yaw Convention** (`rpy="r p y"` in radians):

1. **Roll** (rotation around X-axis): Right-hand rule, thumb along +X
2. **Pitch** (rotation around Y-axis): Right-hand rule, thumb along +Y
3. **Yaw** (rotation around Z-axis): Right-hand rule, thumb along +Z

**Applied in order**: Yaw → Pitch → Roll (intrinsic rotations) or Roll → Pitch → Yaw (extrinsic rotations).

**Example**: Joint at `<origin xyz="0 0 0.5" rpy="0 1.57 0"/>` means:
- Translate 0.5m along parent's Z-axis
- Rotate 90° (π/2 radians) around Y-axis

**Common Mistake**: Confusing link frame vs. visual/collision frame:
```xml
<!-- WRONG: Inertial origin should match center of mass, not link origin -->
<link name="arm">
  <visual>
    <origin xyz="0.25 0 0" rpy="0 0 0"/>  <!-- Box center at +0.25m -->
    <geometry><box size="0.5 0.1 0.1"/></geometry>
  </visual>
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>  <!-- WRONG! Should be xyz="0.25 0 0" -->
    <mass value="2.0"/>
    ...
  </inertial>
</link>
```

### 2.3 Calculating Inertia Tensors

For rigid bodies, the inertia tensor $I$ is:

```math
I = \begin{bmatrix}
I_{xx} & I_{xy} & I_{xz} \\
I_{xy} & I_{yy} & I_{yz} \\
I_{xz} & I_{yz} & I_{zz}
\end{bmatrix}
```

**Moments of Inertia** (diagonal elements):
- $I_\{xx\} = \int (y^2 + z^2) \, dm$
- $I_\{yy\} = \int (x^2 + z^2) \, dm$
- $I_\{zz\} = \int (x^2 + y^2) \, dm$

**Products of Inertia** (off-diagonal):
- $I_\{xy\} = -\int xy \, dm$
- $I_\{xz\} = -\int xz \, dm$
- $I_\{yz\} = -\int yz \, dm$

**Common Shapes** (at center of mass):

| Shape | Dimensions | $I_\{xx\}$ | $I_\{yy\}$ | $I_\{zz\}$ |
|-------|-----------|---------|---------|---------|
| **Box** | width $w$, depth $d$, height $h$ | $\frac\{m\}\{12\}(d^2 + h^2)$ | $\frac\{m\}\{12\}(w^2 + h^2)$ | $\frac\{m\}\{12\}(w^2 + d^2)$ |
| **Cylinder** (Z-axis) | radius $r$, length $l$ | $\frac\{m\}\{12\}(3r^2 + l^2)$ | $\frac\{m\}\{12\}(3r^2 + l^2)$ | $\frac\{m r^2\}\{2\}$ |
| **Sphere** | radius $r$ | $\frac\{2mr^2\}\{5\}$ | $\frac\{2mr^2\}\{5\}$ | $\frac\{2mr^2\}\{5\}$ |

**Example**: 2kg box (0.5m × 0.1m × 0.1m):
- $I_\{xx\} = \frac\{2\}\{12\}(0.1^2 + 0.1^2) = 0.00333$ kg·m²
- $I_\{yy\} = \frac\{2\}\{12\}(0.5^2 + 0.1^2) = 0.0433$ kg·m²
- $I_\{zz\} = \frac\{2\}\{12\}(0.5^2 + 0.1^2) = 0.0433$ kg·m²

**Parallel Axis Theorem**: If inertia at COM is $I_\{\text\{COM\}\}$, inertia at offset $d$ is:

```math
I = I_\{\text\{COM\}\} + m d^2
```

**Tool**: Use MeshLab or Blender to compute inertia from STL meshes with assumed density.

---

## 3. Building a Humanoid URDF

### 3.1 Kinematic Chain Design

Humanoid robots have a **tree topology** with the torso as the root:

```
                    torso (base_link)
                      |
         +------------+------------+
         |            |            |
      left_leg    right_leg     head
         |            |
    +----+----+  +----+----+
    |    |    |  |    |    |
   hip  knee ankle hip knee ankle
    |         |   |         |
  foot      foot foot      foot
```

**Standard DOF Allocation** (30 DOF humanoid):

- **Legs (12 DOF)**: 6 per leg
  - Hip: 3 DOF (roll, pitch, yaw)
  - Knee: 1 DOF (pitch)
  - Ankle: 2 DOF (pitch, roll)

- **Arms (14 DOF)**: 7 per arm
  - Shoulder: 3 DOF (roll, pitch, yaw)
  - Elbow: 1 DOF (pitch)
  - Wrist: 2 DOF (pitch, yaw)
  - Gripper: 1 DOF (parallel jaw or underactuated hand)

- **Torso (2 DOF)**: Waist pitch, waist yaw

- **Head (2 DOF)**: Neck pitch, neck yaw

**Simplified 12-DOF Version** (Chapter 2 quickstart):
- Legs: 6 DOF (3 per leg: hip pitch, knee pitch, ankle pitch)
- Arms: 4 DOF (2 per leg: shoulder pitch, elbow pitch)
- Torso/Head: 2 DOF (waist pitch, neck pitch)

### 3.2 Complete Humanoid URDF Structure

Let's build a 12-DOF humanoid with realistic inertial properties.

**File**: `urdf/humanoid_12dof.urdf`

```xml
<?xml version="1.0"?>
<robot name="humanoid_12dof">

  <!-- ========== TORSO (BASE LINK) ========== -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
      <material name="torso_blue">
        <color rgba="0.2 0.2 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="15.0"/>
      <inertia ixx="0.45" ixy="0.0" ixz="0.0"
               iyy="0.55" iyz="0.0" izz="0.25"/>
    </inertial>
  </link>

  <!-- ========== HEAD ========== -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.12"/>
      </geometry>
      <material name="head_gray">
        <color rgba="0.7 0.7 0.7 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.12"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2.0"/>
      <inertia ixx="0.00576" ixy="0.0" ixz="0.0"
               iyy="0.00576" iyz="0.0" izz="0.00576"/>
    </inertial>
  </link>

  <joint name="neck_joint" type="revolute">
    <parent link="base_link"/>
    <child link="head"/>
    <origin xyz="0 0 0.25" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-0.785" upper="0.785" effort="10.0" velocity="2.0"/>
    <dynamics damping="0.5" friction="0.1"/>
  </joint>

  <!-- ========== LEFT LEG ========== -->

  <!-- Left Hip Link -->
  <link name="left_hip">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <material name="leg_gray">
        <color rgba="0.5 0.5 0.5 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <mass value="3.0"/>
      <inertia ixx="0.023" ixy="0.0" ixz="0.0"
               iyy="0.023" iyz="0.0" izz="0.0024"/>
    </inertial>
  </link>

  <joint name="left_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_hip"/>
    <origin xyz="0 0.1 -0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="150.0" velocity="3.0"/>
    <dynamics damping="1.0" friction="0.2"/>
  </joint>

  <!-- Left Knee Link -->
  <link name="left_knee">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.035"/>
      </geometry>
      <material name="leg_gray"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.035"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <mass value="2.0"/>
      <inertia ixx="0.015" ixy="0.0" ixz="0.0"
               iyy="0.015" iyz="0.0" izz="0.0012"/>
    </inertial>
  </link>

  <joint name="left_knee_joint" type="revolute">
    <parent link="left_hip"/>
    <child link="left_knee"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0.0" upper="2.356" effort="150.0" velocity="3.0"/>
    <dynamics damping="1.0" friction="0.2"/>
  </joint>

  <!-- Left Ankle Link -->
  <link name="left_ankle">
    <visual>
      <origin xyz="0.05 0 -0.025" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
      <material name="foot_black">
        <color rgba="0.1 0.1 0.1 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0.05 0 -0.025" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0.05 0 -0.025" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.00067" ixy="0.0" ixz="0.0"
               iyy="0.00208" iyz="0.0" izz="0.00242"/>
    </inertial>
  </link>

  <joint name="left_ankle_joint" type="revolute">
    <parent link="left_knee"/>
    <child link="left_ankle"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.785" upper="0.785" effort="100.0" velocity="2.0"/>
    <dynamics damping="0.5" friction="0.1"/>
  </joint>

  <!-- ========== RIGHT LEG (SYMMETRIC) ========== -->

  <link name="right_hip">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
      <material name="leg_gray"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.04"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <mass value="3.0"/>
      <inertia ixx="0.023" ixy="0.0" ixz="0.0"
               iyy="0.023" iyz="0.0" izz="0.0024"/>
    </inertial>
  </link>

  <joint name="right_hip_joint" type="revolute">
    <parent link="base_link"/>
    <child link="right_hip"/>
    <origin xyz="0 -0.1 -0.2" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="150.0" velocity="3.0"/>
    <dynamics damping="1.0" friction="0.2"/>
  </joint>

  <link name="right_knee">
    <visual>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.035"/>
      </geometry>
      <material name="leg_gray"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.3" radius="0.035"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.15" rpy="0 0 0"/>
      <mass value="2.0"/>
      <inertia ixx="0.015" ixy="0.0" ixz="0.0"
               iyy="0.015" iyz="0.0" izz="0.0012"/>
    </inertial>
  </link>

  <joint name="right_knee_joint" type="revolute">
    <parent link="right_hip"/>
    <child link="right_knee"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0.0" upper="2.356" effort="150.0" velocity="3.0"/>
    <dynamics damping="1.0" friction="0.2"/>
  </joint>

  <link name="right_ankle">
    <visual>
      <origin xyz="0.05 0 -0.025" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
      <material name="foot_black"/>
    </visual>
    <collision>
      <origin xyz="0.05 0 -0.025" rpy="0 0 0"/>
      <geometry>
        <box size="0.15 0.08 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0.05 0 -0.025" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.00067" ixy="0.0" ixz="0.0"
               iyy="0.00208" iyz="0.0" izz="0.00242"/>
    </inertial>
  </link>

  <joint name="right_ankle_joint" type="revolute">
    <parent link="right_knee"/>
    <child link="right_ankle"/>
    <origin xyz="0 0 -0.3" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-0.785" upper="0.785" effort="100.0" velocity="2.0"/>
    <dynamics damping="0.5" friction="0.1"/>
  </joint>

  <!-- ========== LEFT ARM ========== -->

  <link name="left_shoulder">
    <visual>
      <origin xyz="0 0 -0.12" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.24" radius="0.03"/>
      </geometry>
      <material name="arm_gray">
        <color rgba="0.6 0.6 0.6 1.0"/>
      </material>
    </visual>
    <collision>
      <origin xyz="0 0 -0.12" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.24" radius="0.03"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.12" rpy="0 0 0"/>
      <mass value="1.5"/>
      <inertia ixx="0.0072" ixy="0.0" ixz="0.0"
               iyy="0.0072" iyz="0.0" izz="0.00068"/>
    </inertial>
  </link>

  <joint name="left_shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="left_shoulder"/>
    <origin xyz="0 0.15 0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="80.0" velocity="2.5"/>
    <dynamics damping="0.5" friction="0.1"/>
  </joint>

  <link name="left_elbow">
    <visual>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.2" radius="0.025"/>
      </geometry>
      <material name="arm_gray"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.2" radius="0.025"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.0033" ixy="0.0" ixz="0.0"
               iyy="0.0033" iyz="0.0" izz="0.00031"/>
    </inertial>
  </link>

  <joint name="left_elbow_joint" type="revolute">
    <parent link="left_shoulder"/>
    <child link="left_elbow"/>
    <origin xyz="0 0 -0.24" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0.0" upper="2.356" effort="60.0" velocity="2.5"/>
    <dynamics damping="0.3" friction="0.05"/>
  </joint>

  <!-- ========== RIGHT ARM (SYMMETRIC) ========== -->

  <link name="right_shoulder">
    <visual>
      <origin xyz="0 0 -0.12" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.24" radius="0.03"/>
      </geometry>
      <material name="arm_gray"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.12" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.24" radius="0.03"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.12" rpy="0 0 0"/>
      <mass value="1.5"/>
      <inertia ixx="0.0072" ixy="0.0" ixz="0.0"
               iyy="0.0072" iyz="0.0" izz="0.00068"/>
    </inertial>
  </link>

  <joint name="right_shoulder_joint" type="revolute">
    <parent link="base_link"/>
    <child link="right_shoulder"/>
    <origin xyz="0 -0.15 0.15" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="-1.57" upper="1.57" effort="80.0" velocity="2.5"/>
    <dynamics damping="0.5" friction="0.1"/>
  </joint>

  <link name="right_elbow">
    <visual>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.2" radius="0.025"/>
      </geometry>
      <material name="arm_gray"/>
    </visual>
    <collision>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <geometry>
        <cylinder length="0.2" radius="0.025"/>
      </geometry>
    </collision>
    <inertial>
      <origin xyz="0 0 -0.1" rpy="0 0 0"/>
      <mass value="1.0"/>
      <inertia ixx="0.0033" ixy="0.0" ixz="0.0"
               iyy="0.0033" iyz="0.0" izz="0.00031"/>
    </inertial>
  </link>

  <joint name="right_elbow_joint" type="revolute">
    <parent link="right_shoulder"/>
    <child link="right_elbow"/>
    <origin xyz="0 0 -0.24" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0.0" upper="2.356" effort="60.0" velocity="2.5"/>
    <dynamics damping="0.3" friction="0.05"/>
  </joint>

</robot>
```

**Key Features**:

1. **Realistic Masses**: Total mass ~30 kg (15kg torso + limbs)
2. **Accurate Inertia**: Computed using cylinder/box formulas at correct COM
3. **Joint Limits**: Physiologically realistic (knee: 0-135°, elbow: 0-135°)
4. **Effort Limits**: Hip/knee 150 Nm (high torque for locomotion), arms 60-80 Nm
5. **Damping/Friction**: Higher for legs (stability), lower for arms (dexterity)

### 3.3 Adding Sensors (Cameras, IMUs, LiDAR)

Sensors are attached as **fixed joints** to links with proper frame conventions.

**Example**: RGB-D camera on head:

```xml
<!-- Camera Link -->
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.02 0.08 0.02"/>
    </geometry>
    <material name="camera_black">
      <color rgba="0.1 0.1 0.1 1.0"/>
    </material>
  </visual>
  <inertial>
    <mass value="0.1"/>
    <inertia ixx="0.00001" ixy="0.0" ixz="0.0"
             iyy="0.00001" iyz="0.0" izz="0.00001"/>
  </inertial>
</link>

<joint name="camera_joint" type="fixed">
  <parent link="head"/>
  <child link="camera_link"/>
  <origin xyz="0.13 0 0" rpy="0 0 0"/>
</joint>

<!-- Optical Frame (ROS convention: Z forward, X right, Y down) -->
<link name="camera_optical_frame"/>

<joint name="camera_optical_joint" type="fixed">
  <parent link="camera_link"/>
  <child link="camera_optical_frame"/>
  <origin xyz="0 0 0" rpy="-1.57 0 -1.57"/>
</joint>

<!-- Gazebo Plugin (for simulation) -->
<gazebo reference="camera_link">
  <sensor type="camera" name="head_camera">
    <update_rate>30.0</update_rate>
    <camera name="head">
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
    </camera>
    <plugin name="camera_controller" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/humanoid</namespace>
        <remapping>~/image_raw:=camera/image_raw</remapping>
        <remapping>~/camera_info:=camera/camera_info</remapping>
      </ros>
      <frame_name>camera_optical_frame</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

**IMU in Torso** (standard for balance control):

```xml
<link name="imu_link"/>

<joint name="imu_joint" type="fixed">
  <parent link="base_link"/>
  <child link="imu_link"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
</joint>

<gazebo reference="imu_link">
  <sensor name="imu_sensor" type="imu">
    <always_on>true</always_on>
    <update_rate>100.0</update_rate>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <namespace>/humanoid</namespace>
        <remapping>~/out:=imu</remapping>
      </ros>
      <frame_name>imu_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

---

## 4. Xacro for Modular Descriptions

### 4.1 Why Xacro?

**Problems with Raw URDF**:

1. **Repetition**: Left/right symmetric limbs copy-pasted with tiny differences
2. **Hard-coded Values**: Changing leg length requires editing 10+ files
3. **No Reusability**: Can't share arm definitions between robot variants

**Xacro (XML Macros)** solves this with:

- **Properties**: Variables like `${leg_length}`
- **Macros**: Parameterized templates for limbs
- **Includes**: Compose robot from modular files
- **Math**: Compute inertia from parameters

**Example**: Parameterized leg macro:

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="humanoid_xacro">

  <!-- ========== PROPERTIES ========== -->
  <xacro:property name="leg_length" value="0.6"/>
  <xacro:property name="hip_length" value="${leg_length * 0.5}"/>
  <xacro:property name="knee_length" value="${leg_length * 0.5}"/>
  <xacro:property name="leg_mass" value="5.0"/>
  <xacro:property name="leg_radius" value="0.04"/>

  <!-- ========== MACROS ========== -->
  <xacro:macro name="leg" params="prefix reflect">

    <!-- Hip Link -->
    <link name="${prefix}_hip">
      <visual>
        <origin xyz="0 0 ${-hip_length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder length="${hip_length}" radius="${leg_radius}"/>
        </geometry>
        <material name="leg_gray">
          <color rgba="0.5 0.5 0.5 1.0"/>
        </material>
      </visual>
      <collision>
        <origin xyz="0 0 ${-hip_length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder length="${hip_length}" radius="${leg_radius}"/>
        </geometry>
      </collision>
      <inertial>
        <origin xyz="0 0 ${-hip_length/2}" rpy="0 0 0"/>
        <mass value="${leg_mass/2}"/>
        <inertia ixx="${(leg_mass/2) * (3*leg_radius*leg_radius + hip_length*hip_length) / 12}"
                 ixy="0.0" ixz="0.0"
                 iyy="${(leg_mass/2) * (3*leg_radius*leg_radius + hip_length*hip_length) / 12}"
                 iyz="0.0"
                 izz="${(leg_mass/2) * leg_radius * leg_radius / 2}"/>
      </inertial>
    </link>

    <joint name="${prefix}_hip_joint" type="revolute">
      <parent link="base_link"/>
      <child link="${prefix}_hip"/>
      <origin xyz="0 ${reflect * 0.1} -0.2" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="-1.57" upper="1.57" effort="150.0" velocity="3.0"/>
      <dynamics damping="1.0" friction="0.2"/>
    </joint>

    <!-- Knee Link -->
    <link name="${prefix}_knee">
      <visual>
        <origin xyz="0 0 ${-knee_length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder length="${knee_length}" radius="${leg_radius * 0.9}"/>
        </geometry>
        <material name="leg_gray"/>
      </visual>
      <collision>
        <origin xyz="0 0 ${-knee_length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder length="${knee_length}" radius="${leg_radius * 0.9}"/>
        </geometry>
      </collision>
      <inertial>
        <origin xyz="0 0 ${-knee_length/2}" rpy="0 0 0"/>
        <mass value="${leg_mass/2}"/>
        <inertia ixx="${(leg_mass/2) * (3*(leg_radius*0.9)*(leg_radius*0.9) + knee_length*knee_length) / 12}"
                 ixy="0.0" ixz="0.0"
                 iyy="${(leg_mass/2) * (3*(leg_radius*0.9)*(leg_radius*0.9) + knee_length*knee_length) / 12}"
                 iyz="0.0"
                 izz="${(leg_mass/2) * (leg_radius*0.9) * (leg_radius*0.9) / 2}"/>
      </inertial>
    </link>

    <joint name="${prefix}_knee_joint" type="revolute">
      <parent link="${prefix}_hip"/>
      <child link="${prefix}_knee"/>
      <origin xyz="0 0 ${-hip_length}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="0.0" upper="2.356" effort="150.0" velocity="3.0"/>
      <dynamics damping="1.0" friction="0.2"/>
    </joint>

    <!-- Ankle/Foot -->
    <link name="${prefix}_ankle">
      <visual>
        <origin xyz="0.05 0 -0.025" rpy="0 0 0"/>
        <geometry>
          <box size="0.15 0.08 0.05"/>
        </geometry>
        <material name="foot_black">
          <color rgba="0.1 0.1 0.1 1.0"/>
        </material>
      </visual>
      <collision>
        <origin xyz="0.05 0 -0.025" rpy="0 0 0"/>
        <geometry>
          <box size="0.15 0.08 0.05"/>
        </geometry>
      </collision>
      <inertial>
        <origin xyz="0.05 0 -0.025" rpy="0 0 0"/>
        <mass value="1.0"/>
        <inertia ixx="0.00067" ixy="0.0" ixz="0.0"
                 iyy="0.00208" iyz="0.0" izz="0.00242"/>
      </inertial>
    </link>

    <joint name="${prefix}_ankle_joint" type="revolute">
      <parent link="${prefix}_knee"/>
      <child link="${prefix}_ankle"/>
      <origin xyz="0 0 ${-knee_length}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="-0.785" upper="0.785" effort="100.0" velocity="2.0"/>
      <dynamics damping="0.5" friction="0.1"/>
    </joint>

  </xacro:macro>

  <!-- ========== TORSO ========== -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
      <material name="torso_blue">
        <color rgba="0.2 0.2 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.3 0.2 0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="15.0"/>
      <inertia ixx="0.45" ixy="0.0" ixz="0.0"
               iyy="0.55" iyz="0.0" izz="0.25"/>
    </inertial>
  </link>

  <!-- ========== INSTANTIATE LEGS ========== -->
  <xacro:leg prefix="left" reflect="1"/>
  <xacro:leg prefix="right" reflect="-1"/>

</robot>
```

**Key Features**:

1. **`${property}`**: Substitutes variable values
2. **`${expression}`**: Evaluates math (e.g., `${leg_length * 0.5}`)
3. **`params="prefix reflect"`**: Macro parameters
4. **`reflect`**: Multiplies Y-offset by ±1 for left/right symmetry

**Convert Xacro to URDF**:

```bash
ros2 run xacro xacro humanoid.xacro > humanoid.urdf
```

### 4.2 Advanced Xacro Patterns

**Include External Files** (modular design):

```xml
<!-- main.xacro -->
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_robot">
  <xacro:include filename="$(find my_robot_description)/urdf/leg.xacro"/>
  <xacro:include filename="$(find my_robot_description)/urdf/arm.xacro"/>
  <xacro:include filename="$(find my_robot_description)/urdf/sensors.xacro"/>

  <link name="base_link">...</link>

  <xacro:leg prefix="left" reflect="1" length="0.8"/>
  <xacro:leg prefix="right" reflect="-1" length="0.8"/>
  <xacro:arm prefix="left" reflect="1"/>
  <xacro:arm prefix="right" reflect="-1"/>
  <xacro:camera_sensor parent="head" xyz="0.12 0 0"/>
</robot>
```

**Conditional Macros** (optional sensors):

```xml
<xacro:property name="use_lidar" value="true"/>

<xacro:if value="${use_lidar}">
  <link name="lidar_link">
    <!-- LiDAR definition -->
  </link>
  <joint name="lidar_joint" type="fixed">
    <parent link="head"/>
    <child link="lidar_link"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
  </joint>
</xacro:if>
```

**External YAML Properties** (load from file):

```yaml
# config/robot_params.yaml
leg_length: 0.75
arm_length: 0.6
torso_mass: 18.0
```

```xml
<!-- Load YAML -->
<xacro:property name="config" value="${load_yaml('$(find my_robot)/config/robot_params.yaml')}"/>

<xacro:property name="leg_length" value="${config['leg_length']}"/>
```

---

## 5. SDF Format and URDF→SDF Conversion

### 5.1 SDF Advantages Over URDF

**Simulation Description Format (SDF)** is Gazebo's native format with key improvements:

1. **Multi-Model Support**: One file can define multiple robots and static objects
2. **Native Sensor Definitions**: Cameras, LiDAR, contact sensors without `<gazebo>` tags
3. **Explicit Physics**: Friction coefficients, contact stiffness, solver parameters
4. **Nested Models**: Hierarchical composition (e.g., gripper as sub-model of arm)

**SDF Example** (simplified humanoid):

```xml
<?xml version="1.0"?>
<sdf version="1.9">
  <model name="humanoid_sdf">

    <!-- Base Link -->
    <link name="base_link">
      <pose>0 0 1.0 0 0 0</pose>
      <inertial>
        <mass>15.0</mass>
        <inertia>
          <ixx>0.45</ixx>
          <iyy>0.55</iyy>
          <izz>0.25</izz>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyz>0</iyz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry>
          <box>
            <size>0.3 0.2 0.4</size>
          </box>
        </geometry>
        <material>
          <ambient>0.2 0.2 0.8 1</ambient>
          <diffuse>0.2 0.2 0.8 1</diffuse>
        </material>
      </visual>
      <collision name="collision">
        <geometry>
          <box>
            <size>0.3 0.2 0.4</size>
          </box>
        </geometry>
        <surface>
          <friction>
            <ode>
              <mu>0.8</mu>
              <mu2>0.8</mu2>
            </ode>
          </friction>
          <contact>
            <ode>
              <kp>1e6</kp>
              <kd>1e3</kd>
            </ode>
          </contact>
        </surface>
      </collision>

      <!-- IMU Sensor -->
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
            <!-- Y and Z similar -->
          </angular_velocity>
          <linear_acceleration>
            <x>
              <noise type="gaussian">
                <mean>0.0</mean>
                <stddev>0.017</stddev>
              </noise>
            </x>
            <!-- Y and Z similar -->
          </linear_acceleration>
        </imu>
      </sensor>
    </link>

    <!-- Hip Joint -->
    <joint name="left_hip_joint" type="revolute">
      <parent>base_link</parent>
      <child>left_hip</child>
      <pose>0 0.1 -0.2 0 0 0</pose>
      <axis>
        <xyz>0 1 0</xyz>
        <limit>
          <lower>-1.57</lower>
          <upper>1.57</upper>
          <effort>150</effort>
          <velocity>3.0</velocity>
        </limit>
        <dynamics>
          <damping>1.0</damping>
          <friction>0.2</friction>
        </dynamics>
      </axis>
    </joint>

    <!-- Left Hip Link (simplified) -->
    <link name="left_hip">
      <pose relative_to="left_hip_joint">0 0 -0.15 0 0 0</pose>
      <inertial>
        <mass>3.0</mass>
        <inertia>
          <ixx>0.023</ixx>
          <iyy>0.023</iyy>
          <izz>0.0024</izz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>0.04</radius>
            <length>0.3</length>
          </cylinder>
        </geometry>
      </visual>
      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>0.04</radius>
            <length>0.3</length>
          </cylinder>
        </geometry>
      </collision>
    </link>

    <!-- Plugins -->
    <plugin name="gazebo_ros_joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
      <ros>
        <namespace>/humanoid</namespace>
        <remapping>~/out:=joint_states</remapping>
      </ros>
      <update_rate>50</update_rate>
    </plugin>

  </model>
</sdf>
```

**Key Differences from URDF**:

1. **`<pose>` instead of `<origin>`**: 6-DOF transform (x y z roll pitch yaw)
2. **`relative_to` attribute**: Explicit frame references (URDF infers from joint tree)
3. **`<surface>` tags**: Friction (`mu`, `mu2`), contact stiffness (`kp`, `kd`)
4. **Sensor noise**: Gaussian noise models directly in sensor definition

### 5.2 URDF→SDF Conversion

**Using Gazebo Tools**:

```bash
# Convert URDF to SDF (prints to stdout)
gz sdf -p humanoid.urdf > humanoid.sdf

# Validate SDF
gz sdf --check humanoid.sdf

# Convert with verbose error messages
gz sdf -p humanoid.urdf -v 3 > humanoid.sdf
```

**Common Conversion Issues**:

1. **Missing `<gazebo>` tags**: Friction/damping not in URDF → zero friction in SDF
   - **Fix**: Add `<gazebo reference="link_name"><mu1>0.8</mu1><mu2>0.8</mu2></gazebo>`

2. **Material colors**: URDF `<material>` doesn't convert to SDF `<ambient>/<diffuse>`
   - **Fix**: Manually add `<material><ambient>R G B A</ambient></material>` in SDF

3. **Plugin parameters**: URDF `<gazebo><plugin>` tags may need adjustment for SDF syntax

4. **Frame semantics**: URDF joint origins are parent-to-child, SDF `<pose>` is child-in-parent
   - **Usually handled correctly** by `gz sdf`, but check complex kinematic chains

**Editing Converted SDF** (add sensor noise):

```xml
<!-- After conversion, manually add noise to IMU -->
<sensor name="imu_sensor" type="imu">
  <imu>
    <angular_velocity>
      <x>
        <noise type="gaussian">
          <mean>0.0</mean>
          <stddev>0.009</stddev>  <!-- 0.5 deg/s noise -->
        </noise>
      </x>
      <!-- Repeat for y, z -->
    </angular_velocity>
  </imu>
</sensor>
```

---

## 6. Validation and Debugging Tools

### 6.1 check_urdf

**Purpose**: Parse URDF and verify XML structure, joint tree topology, and required tags.

**Usage**:

```bash
# Basic check
check_urdf humanoid.urdf

# Output on success:
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

**Common Errors**:

1. **Missing inertia**:
   ```
   Error: link 'left_hip' has no inertia tag
   ```
   **Fix**: Add `<inertial><mass/><inertia/></inertial>` to every link

2. **Broken parent-child chain**:
   ```
   Error: joint 'left_knee_joint' references unknown link 'left_thigh'
   ```
   **Fix**: Typo in `<parent>` or `<child>` tag

3. **Invalid XML**:
   ```
   Error: Tag 'origin' not closed
   ```
   **Fix**: Ensure `<origin ... />` has closing `/>`

### 6.2 urdf_to_graphviz

**Purpose**: Visualize kinematic tree as a graph (helpful for debugging complex robots).

**Usage**:

```bash
# Generate PDF visualization
urdf_to_graphviz humanoid.urdf

# Output: humanoid_12dof.pdf (tree graph)
# Open with: evince humanoid_12dof.pdf
```

**Interpreting Output**:

- **Nodes**: Links (boxes)
- **Edges**: Joints (arrows from parent to child)
- **Colors**: Different colors for joint types (revolute, fixed, prismatic)

**Example** (simplified text representation):

```
base_link
  ├── [neck_joint] → head
  ├── [left_hip_joint] → left_hip
  │     └── [left_knee_joint] → left_knee
  │           └── [left_ankle_joint] → left_ankle
  └── [right_hip_joint] → right_hip
        └── [right_knee_joint] → right_knee
              └── [right_ankle_joint] → right_ankle
```

### 6.3 RViz Visualization

**Purpose**: Render robot in 3D with interactive joint sliders.

**Steps**:

1. **Launch RViz**:
   ```bash
   ros2 run rviz2 rviz2
   ```

2. **Add Robot Model Display**:
   - Click "Add" → "RobotModel"
   - Set "Fixed Frame" to `base_link`
   - Set "Robot Description" topic to `/robot_description`

3. **Publish URDF to /robot_description**:
   ```bash
   ros2 run robot_state_publisher robot_state_publisher \
     --ros-args -p robot_description:="$(xacro humanoid.xacro)"
   ```

4. **Add Joint State Publisher GUI** (interactive sliders):
   ```bash
   ros2 run joint_state_publisher_gui joint_state_publisher_gui \
     --ros-args -p robot_description:="$(xacro humanoid.xacro)"
   ```

**Common Issues**:

- **"No transform from X to Y"**: Missing `robot_state_publisher` node
- **Red links**: Missing mesh files or incorrect `package://` path
- **Flipped joints**: Check `<axis xyz="..."/>` direction

### 6.4 Gazebo Simulation Validation

**Purpose**: Test physics, collisions, and sensor plugins.

**Steps**:

1. **Launch Gazebo with URDF**:
   ```bash
   gz sim -v 4
   # In Gazebo GUI: File → Insert → Browse to humanoid.sdf
   ```

2. **Check for Errors**:
   ```bash
   # Terminal will show warnings like:
   # [Wrn] Link 'left_hip' has zero mass
   # [Err] Joint 'left_knee_joint' exceeds effort limit
   ```

3. **Validate Inertia** (run simulation and check for):
   - **Unstable behavior**: Legs vibrating → inertia too low
   - **Slow response**: Limbs sluggish → inertia too high
   - **Collapsing**: Robot falls through floor → missing collision tags

4. **Test Sensors**:
   ```bash
   # Check IMU publishing
   ros2 topic hz /humanoid/imu  # Expected: 100 Hz

   # Check camera
   ros2 topic hz /humanoid/camera/image_raw  # Expected: 30 Hz
   ```

### 6.5 Isaac Sim URDF Importer

**Purpose**: Import URDF into NVIDIA Isaac Sim for high-fidelity rendering and PhysX dynamics.

**Steps**:

1. **Launch Isaac Sim**:
   ```bash
   ~/.local/share/ov/pkg/isaac-sim-4.2.0/isaac-sim.sh
   ```

2. **Import URDF**:
   - Menu: "Isaac Utils" → "URDF Importer"
   - Settings:
     - **Import Inertia Tensor**: ✅ (use URDF inertia)
     - **Fix Base Link**: ❌ (allow free motion for humanoids)
     - **Joint Drive Type**: Position (for position control)
     - **Joint Drive Strength**: 10000 (high stiffness for accurate tracking)

3. **Validate Import**:
   - Check "Stage" panel: All links and joints present
   - Click "Play" → Robot should remain stable (not explode or fall through floor)
   - Select joint → Modify "targetPosition" → Verify motion

**Common Issues**:

- **Exploding robot**: Inertia too low or joint damping zero
  - **Fix**: Increase `<dynamics damping="1.0"/>`

- **Frozen joints**: Joint drive strength too low
  - **Fix**: Increase to 5000-20000 in importer settings

- **Missing visuals**: Mesh file paths incorrect
  - **Fix**: Use absolute paths or `package://` with `ROS_PACKAGE_PATH` set

---

## 7. Hands-On Lab: Build Your Own Humanoid URDF

**Objective**: Create a custom 12-DOF humanoid URDF from scratch, validate it, and simulate in RViz and Gazebo.

**Time**: 5-7 hours

### Lab Setup

```bash
# Create workspace
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_humanoid_description
cd my_humanoid_description
mkdir urdf launch meshes

# Install dependencies
sudo apt install ros-${ROS_DISTRO}-xacro \
                 ros-${ROS_DISTRO}-joint-state-publisher-gui \
                 ros-${ROS_DISTRO}-robot-state-publisher \
                 liburdfdom-tools  # provides check_urdf
```

### Step 1: Define Robot Parameters (30 minutes)

Create `config/robot_dimensions.yaml`:

```yaml
# Torso
torso:
  width: 0.3
  depth: 0.2
  height: 0.4
  mass: 15.0

# Legs
leg:
  hip_length: 0.3
  knee_length: 0.3
  foot_length: 0.15
  foot_width: 0.08
  hip_mass: 3.0
  knee_mass: 2.0
  foot_mass: 1.0
  radius: 0.04

# Arms
arm:
  shoulder_length: 0.24
  elbow_length: 0.2
  shoulder_mass: 1.5
  elbow_mass: 1.0
  radius: 0.03

# Head
head:
  radius: 0.12
  mass: 2.0
```

### Step 2: Create Xacro Macros (2 hours)

**File**: `urdf/macros/leg.xacro`

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:macro name="leg" params="prefix reflect *origin">

    <!-- Hip Link -->
    <link name="${prefix}_hip">
      <visual>
        <origin xyz="0 0 ${-leg_hip_length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder length="${leg_hip_length}" radius="${leg_radius}"/>
        </geometry>
        <material name="leg_gray">
          <color rgba="0.5 0.5 0.5 1.0"/>
        </material>
      </visual>
      <collision>
        <origin xyz="0 0 ${-leg_hip_length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder length="${leg_hip_length}" radius="${leg_radius}"/>
        </geometry>
      </collision>
      <xacro:cylinder_inertia mass="${leg_hip_mass}"
                              radius="${leg_radius}"
                              length="${leg_hip_length}"
                              xyz="0 0 ${-leg_hip_length/2}"/>
    </link>

    <joint name="${prefix}_hip_joint" type="revolute">
      <xacro:insert_block name="origin"/>
      <parent link="base_link"/>
      <child link="${prefix}_hip"/>
      <axis xyz="0 1 0"/>
      <limit lower="-1.57" upper="1.57" effort="150.0" velocity="3.0"/>
      <dynamics damping="1.0" friction="0.2"/>
    </joint>

    <!-- Knee Link -->
    <link name="${prefix}_knee">
      <visual>
        <origin xyz="0 0 ${-leg_knee_length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder length="${leg_knee_length}" radius="${leg_radius * 0.9}"/>
        </geometry>
        <material name="leg_gray"/>
      </visual>
      <collision>
        <origin xyz="0 0 ${-leg_knee_length/2}" rpy="0 0 0"/>
        <geometry>
          <cylinder length="${leg_knee_length}" radius="${leg_radius * 0.9}"/>
        </geometry>
      </collision>
      <xacro:cylinder_inertia mass="${leg_knee_mass}"
                              radius="${leg_radius * 0.9}"
                              length="${leg_knee_length}"
                              xyz="0 0 ${-leg_knee_length/2}"/>
    </link>

    <joint name="${prefix}_knee_joint" type="revolute">
      <parent link="${prefix}_hip"/>
      <child link="${prefix}_knee"/>
      <origin xyz="0 0 ${-leg_hip_length}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="0.0" upper="2.356" effort="150.0" velocity="3.0"/>
      <dynamics damping="1.0" friction="0.2"/>
    </joint>

    <!-- Ankle/Foot -->
    <link name="${prefix}_foot">
      <visual>
        <origin xyz="${leg_foot_length/2} 0 -0.025" rpy="0 0 0"/>
        <geometry>
          <box size="${leg_foot_length} ${leg_foot_width} 0.05"/>
        </geometry>
        <material name="foot_black">
          <color rgba="0.1 0.1 0.1 1.0"/>
        </material>
      </visual>
      <collision>
        <origin xyz="${leg_foot_length/2} 0 -0.025" rpy="0 0 0"/>
        <geometry>
          <box size="${leg_foot_length} ${leg_foot_width} 0.05"/>
        </geometry>
      </collision>
      <xacro:box_inertia mass="${leg_foot_mass}"
                         x="${leg_foot_length}" y="${leg_foot_width}" z="0.05"
                         xyz="${leg_foot_length/2} 0 -0.025"/>
    </link>

    <joint name="${prefix}_ankle_joint" type="revolute">
      <parent link="${prefix}_knee"/>
      <child link="${prefix}_foot"/>
      <origin xyz="0 0 ${-leg_knee_length}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="-0.785" upper="0.785" effort="100.0" velocity="2.0"/>
      <dynamics damping="0.5" friction="0.1"/>
    </joint>

  </xacro:macro>

</robot>
```

**File**: `urdf/macros/inertia.xacro` (helper macros):

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <!-- Cylinder Inertia -->
  <xacro:macro name="cylinder_inertia" params="mass radius length xyz">
    <inertial>
      <origin xyz="${xyz}" rpy="0 0 0"/>
      <mass value="${mass}"/>
      <inertia ixx="${mass * (3*radius*radius + length*length) / 12}"
               ixy="0.0" ixz="0.0"
               iyy="${mass * (3*radius*radius + length*length) / 12}"
               iyz="0.0"
               izz="${mass * radius * radius / 2}"/>
    </inertial>
  </xacro:macro>

  <!-- Box Inertia -->
  <xacro:macro name="box_inertia" params="mass x y z xyz">
    <inertial>
      <origin xyz="${xyz}" rpy="0 0 0"/>
      <mass value="${mass}"/>
      <inertia ixx="${mass * (y*y + z*z) / 12}"
               ixy="0.0" ixz="0.0"
               iyy="${mass * (x*x + z*z) / 12}"
               iyz="0.0"
               izz="${mass * (x*x + y*y) / 12}"/>
    </inertial>
  </xacro:macro>

  <!-- Sphere Inertia -->
  <xacro:macro name="sphere_inertia" params="mass radius">
    <inertial>
      <mass value="${mass}"/>
      <inertia ixx="${2 * mass * radius * radius / 5}"
               ixy="0.0" ixz="0.0"
               iyy="${2 * mass * radius * radius / 5}"
               iyz="0.0"
               izz="${2 * mass * radius * radius / 5}"/>
    </inertial>
  </xacro:macro>

</robot>
```

### Step 3: Assemble Main URDF (1 hour)

**File**: `urdf/humanoid.xacro`

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="my_humanoid">

  <!-- Load macros -->
  <xacro:include filename="$(find my_humanoid_description)/urdf/macros/inertia.xacro"/>
  <xacro:include filename="$(find my_humanoid_description)/urdf/macros/leg.xacro"/>

  <!-- Load parameters -->
  <xacro:property name="config" value="${load_yaml('$(find my_humanoid_description)/config/robot_dimensions.yaml')}"/>

  <xacro:property name="torso_width" value="${config['torso']['width']}"/>
  <xacro:property name="torso_depth" value="${config['torso']['depth']}"/>
  <xacro:property name="torso_height" value="${config['torso']['height']}"/>
  <xacro:property name="torso_mass" value="${config['torso']['mass']}"/>

  <xacro:property name="leg_hip_length" value="${config['leg']['hip_length']}"/>
  <xacro:property name="leg_knee_length" value="${config['leg']['knee_length']}"/>
  <xacro:property name="leg_foot_length" value="${config['leg']['foot_length']}"/>
  <xacro:property name="leg_foot_width" value="${config['leg']['foot_width']}"/>
  <xacro:property name="leg_hip_mass" value="${config['leg']['hip_mass']}"/>
  <xacro:property name="leg_knee_mass" value="${config['leg']['knee_mass']}"/>
  <xacro:property name="leg_foot_mass" value="${config['leg']['foot_mass']}"/>
  <xacro:property name="leg_radius" value="${config['leg']['radius']}"/>

  <!-- Torso (Base Link) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="${torso_width} ${torso_depth} ${torso_height}"/>
      </geometry>
      <material name="torso_blue">
        <color rgba="0.2 0.2 0.8 1.0"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="${torso_width} ${torso_depth} ${torso_height}"/>
      </geometry>
    </collision>
    <xacro:box_inertia mass="${torso_mass}"
                       x="${torso_width}" y="${torso_depth}" z="${torso_height}"
                       xyz="0 0 0"/>
  </link>

  <!-- Instantiate Legs -->
  <xacro:leg prefix="left" reflect="1">
    <origin xyz="0 0.1 ${-torso_height/2}" rpy="0 0 0"/>
  </xacro:leg>

  <xacro:leg prefix="right" reflect="-1">
    <origin xyz="0 -0.1 ${-torso_height/2}" rpy="0 0 0"/>
  </xacro:leg>

  <!-- TODO: Add arms and head (left as exercise) -->

</robot>
```

### Step 4: Validation (1 hour)

```bash
# Convert Xacro to URDF
cd ~/ros2_ws/src/my_humanoid_description
ros2 run xacro xacro urdf/humanoid.xacro > urdf/humanoid.urdf

# Validate with check_urdf
check_urdf urdf/humanoid.urdf

# Generate kinematic tree visualization
urdf_to_graphviz urdf/humanoid.urdf
evince my_humanoid.pdf
```

**Expected Output**:

```
robot name is: my_humanoid
---------- Successfully Parsed XML ---------------
root Link: base_link has 2 child(ren)
    child(1):  left_hip
        child(1):  left_knee
            child(1):  left_foot
    child(2):  right_hip
        child(1):  right_knee
            child(1):  right_foot
```

### Step 5: RViz Visualization (1 hour)

**File**: `launch/view_robot.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('my_humanoid_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'humanoid.xacro')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_file])
            }]
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'view_robot.rviz')]
        ),
    ])
```

**Run**:

```bash
# Build workspace
cd ~/ros2_ws
colcon build --packages-select my_humanoid_description
source install/setup.bash

# Launch
ros2 launch my_humanoid_description view_robot.launch.py
```

**Tasks**:

1. Use joint sliders to move legs
2. Verify no collisions between links
3. Check COM stays above support polygon (feet)
4. Screenshot stable standing pose

### Step 6: Gazebo Simulation (2 hours)

**Convert to SDF**:

```bash
gz sdf -p urdf/humanoid.urdf > urdf/humanoid.sdf
```

**Add Gazebo plugins** to `humanoid.xacro`:

```xml
<!-- Joint State Publisher Plugin -->
<gazebo>
  <plugin name="gazebo_ros_joint_state_publisher" filename="libgazebo_ros_joint_state_publisher.so">
    <ros>
      <namespace>/my_humanoid</namespace>
      <remapping>~/out:=joint_states</remapping>
    </ros>
    <update_rate>50</update_rate>
  </plugin>
</gazebo>

<!-- Joint Position Controller Plugin -->
<gazebo>
  <plugin name="gazebo_ros_joint_pose_trajectory" filename="libgazebo_ros_joint_pose_trajectory.so">
    <ros>
      <namespace>/my_humanoid</namespace>
      <remapping>~/set_joint_trajectory:=joint_trajectory</remapping>
    </ros>
    <update_rate>50</update_rate>
  </plugin>
</gazebo>
```

**Launch Gazebo**:

```bash
gz sim humanoid.sdf
```

**Tasks**:

1. Drop robot from 0.1m height → Should land stably without tipping
2. Apply impulse to torso → Should recover balance (if controller active)
3. Check `/my_humanoid/joint_states` publishes at 50 Hz

---

## 8. End-of-Chapter Project: Modular Sensor Platform

**Objective**: Design a humanoid URDF with **swappable end-effectors** (parallel gripper, suction cup, tool holder) using Xacro macros and YAML configuration.

**Requirements**:

1. **Base Robot**: 12-DOF humanoid from lab
2. **Three End-Effectors**:
   - Parallel gripper (2 prismatic fingers)
   - Suction cup (vacuum gripper for flat objects)
   - Tool holder (fixed mount for screwdriver/wrench)
3. **Xacro Parameter**: `end_effector_type` selects which to attach
4. **Validation**:
   - All three variants pass `check_urdf`
   - RViz visualization shows correct end-effector
   - Gazebo simulation with object grasping (bonus)

**Starter Code** (`urdf/end_effectors.xacro`):

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

  <xacro:macro name="parallel_gripper" params="parent_link">
    <!-- Left Finger -->
    <link name="left_finger">
      <visual>
        <geometry>
          <box size="0.02 0.01 0.05"/>
        </geometry>
        <material name="gripper_gray">
          <color rgba="0.6 0.6 0.6 1.0"/>
        </material>
      </visual>
      <collision>
        <geometry>
          <box size="0.02 0.01 0.05"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.05"/>
        <inertia ixx="0.00001" ixy="0" ixz="0" iyy="0.00001" iyz="0" izz="0.00001"/>
      </inertial>
    </link>

    <joint name="left_finger_joint" type="prismatic">
      <parent link="${parent_link}"/>
      <child link="left_finger"/>
      <origin xyz="0.02 0.015 0" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <limit lower="0.0" upper="0.03" effort="20.0" velocity="0.1"/>
    </joint>

    <!-- Right Finger (symmetric) -->
    <link name="right_finger">
      <visual>
        <geometry>
          <box size="0.02 0.01 0.05"/>
        </geometry>
        <material name="gripper_gray"/>
      </visual>
      <collision>
        <geometry>
          <box size="0.02 0.01 0.05"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.05"/>
        <inertia ixx="0.00001" ixy="0" ixz="0" iyy="0.00001" iyz="0" izz="0.00001"/>
      </inertial>
    </link>

    <joint name="right_finger_joint" type="prismatic">
      <parent link="${parent_link}"/>
      <child link="right_finger"/>
      <origin xyz="0.02 -0.015 0" rpy="0 0 0"/>
      <axis xyz="0 -1 0"/>
      <limit lower="0.0" upper="0.03" effort="20.0" velocity="0.1"/>
    </joint>
  </xacro:macro>

  <xacro:macro name="suction_cup" params="parent_link">
    <link name="suction_cup">
      <visual>
        <geometry>
          <cylinder radius="0.025" length="0.04"/>
        </geometry>
        <material name="suction_orange">
          <color rgba="1.0 0.5 0.0 1.0"/>
        </material>
      </visual>
      <collision>
        <geometry>
          <cylinder radius="0.025" length="0.04"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="0.1"/>
        <inertia ixx="0.00002" ixy="0" ixz="0" iyy="0.00002" iyz="0" izz="0.00003"/>
      </inertial>
    </link>

    <joint name="suction_joint" type="fixed">
      <parent link="${parent_link}"/>
      <child link="suction_cup"/>
      <origin xyz="0.02 0 0" rpy="0 0 0"/>
    </joint>
  </xacro:macro>

  <!-- TODO: Add tool_holder macro -->

</robot>
```

**Main URDF** (`humanoid_with_tools.xacro`):

```xml
<xacro:include filename="$(find my_humanoid_description)/urdf/end_effectors.xacro"/>

<xacro:property name="end_effector_type" value="parallel_gripper"/>

<xacro:if value="${end_effector_type == 'parallel_gripper'}">
  <xacro:parallel_gripper parent_link="left_elbow"/>
  <xacro:parallel_gripper parent_link="right_elbow"/>
</xacro:if>

<xacro:if value="${end_effector_type == 'suction_cup'}">
  <xacro:suction_cup parent_link="left_elbow"/>
  <xacro:suction_cup parent_link="right_elbow"/>
</xacro:if>
```

**Evaluation Criteria**:

- **Correctness (40%)**: All three variants pass `check_urdf` and `gz sdf --check`
- **Modularity (30%)**: Single parameter changes end-effector, no code duplication
- **Documentation (20%)**: README with usage, parameter descriptions, screenshots
- **Bonus (10%)**: Gazebo grasping demo with vacuum plugin or mimic joints

---

## Summary

In this chapter, you learned:

1. **Robot Description Formats**: URDF for ROS 2, SDF for Gazebo, USD for Isaac Sim
2. **URDF Structure**: Links (geometry, inertia), joints (types, limits), and coordinate frames
3. **Humanoid Kinematics**: 12-30 DOF configurations, kinematic tree topology
4. **Xacro Macros**: Parameterized templates for modular robot descriptions
5. **Validation Tools**: `check_urdf`, `urdf_to_graphviz`, RViz, Gazebo, Isaac Sim
6. **SDF Conversion**: URDF→SDF workflow and common issues
7. **Hands-On Skills**: Built a complete 12-DOF humanoid from scratch, validated in simulation

**Key Takeaways**:

- Always define inertial properties (mass, COM, inertia tensor) for dynamic simulation
- Use Xacro for maintainability (DRY principle)
- Validate early and often (check_urdf after every major change)
- Match inertial frame to COM, not link origin
- Test in RViz (kinematics) before Gazebo (dynamics)

**Next Chapter Preview**: Chapter 5 covers Gazebo simulation fundamentals—worlds, physics engines, sensor plugins, and your first bipedal walking controller.

---

## Further Reading

**URDF Specification**:
- ROS Wiki URDF/XML: http://wiki.ros.org/urdf/XML
- Official URDF Spec (GitHub): https://github.com/ros/urdfdom/blob/master/urdf_parser/urdf.bnf

**SDF Specification**:
- SDFormat Specification: http://sdformat.org/spec
- Gazebo SDF Tutorials: https://gazebosim.org/docs/latest/sdf_worlds

**Xacro Documentation**:
- ROS 2 Xacro: https://github.com/ros/xacro
- Xacro Tutorial: http://wiki.ros.org/urdf/Tutorials/Using%20Xacro%20to%20Clean%20Up%20a%20URDF%20File

**Isaac Sim USD**:
- NVIDIA Isaac Sim URDF Importer: https://docs.omniverse.nvidia.com/isaacsim/latest/features/environment_setup/ext_omni_isaac_urdf.html
- USD Specification (Pixar): https://openusd.org/release/spec.html

**Inertia Calculation**:
- List of Moments of Inertia (Wikipedia): https://en.wikipedia.org/wiki/List_of_moments_of_inertia
- MeshLab (Geometric Measures): http://www.meshlab.net/

**Example Humanoid URDFs**:
- PR2 Robot (Willow Garage): https://github.com/pr2/pr2_common/tree/melodic-devel/pr2_description
- Atlas Robot (Boston Dynamics/OSRF): https://github.com/RobotLocomotion/drake/tree/master/examples/atlas
- Valkyrie (NASA JSC): https://github.com/NASA-JSC/val_description

---

**End of Chapter 4**
