# Chapter 2 Quickstart - ROS 2 + Isaac Sim Demo

Companion code for Chapter 2: Development Environment Setup

## Contents

- `urdf/simple_humanoid.urdf` - 12-DOF humanoid robot description
- `scripts/` - Python helper scripts (future)

## Usage

### 1. Clone Repository

```bash
cd ~/ros2_ws/src
git clone https://github.com/your-username/robot-book-code.git
cd robot-book-code/chapter-02-quickstart/
```

### 2. Load URDF in Isaac Sim

1. Launch Isaac Sim:
   ```bash
   cd ~/.local/share/ov/pkg/isaac_sim-2024.2.0
   ./isaac-sim.sh
   ```

2. Import URDF:
   - Top menu: "Isaac Utils" → "URDF Importer"
   - Browse to `~/ros2_ws/src/robot-book-code/chapter-02-quickstart/urdf/simple_humanoid.urdf`
   - Click "Import"

### 3. Verify Joint States in ROS 2

```bash
source /opt/ros/iron/setup.bash
ros2 topic list | grep joint_states
ros2 topic echo /joint_states --once
```

### 4. Visualize in RViz2

```bash
# Terminal 1: Publish robot description
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(cat urdf/simple_humanoid.urdf)"

# Terminal 2: Launch RViz2
rviz2
```

In RViz2:
- Add → RobotModel
- Fixed Frame: "base_link"

## Robot Specifications

- **Total Mass**: 30 kg
- **Height**: ~1.2 m (standing)
- **DOF**: 12
  - 6 leg joints (3 per leg: hip pitch, knee, ankle)
  - 4 arm joints (2 per arm: shoulder, elbow)
- **Joint Limits**:
  - Hip pitch: -90° to +90°
  - Knee: 0° to +135°
  - Ankle: -45° to +45°
  - Shoulder: -180° to +180°
  - Elbow: 0° to +135°

## Troubleshooting

**URDF import fails**:
- Ensure Isaac Sim 2024.2 is installed
- Check URDF syntax: `check_urdf urdf/simple_humanoid.urdf`

**No `/joint_states` topic**:
- Verify ROS 2 Bridge extension is enabled in Isaac Sim
- Check ROS_DOMAIN_ID matches: `echo $ROS_DOMAIN_ID`

**Robot not visible in RViz2**:
- Ensure `robot_state_publisher` is running
- Check Fixed Frame is set to "base_link"

## License

MIT License - See main repository LICENSE file
