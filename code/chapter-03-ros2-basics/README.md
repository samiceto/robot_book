# Chapter 3: ROS 2 Fundamentals - Code Examples

Companion code for Chapter 3: ROS 2 Fundamentals - Nodes, Topics, Services, Actions

## Contents

- `examples/sensor_simulator.py` - IMU sensor simulator (publishes to `/imu` at 50 Hz)
- `examples/filter_node.py` - Moving average filter (subscribes to `/imu`, publishes to `/imu_filtered`)
- `examples/calibration_service.py` - Calibration service server (`/calibrate_imu`)
- `examples/long_computation_action.py` - Action server for long-running tasks (`/compute_trajectory`)
- `launch/multi_node.launch.py` - Launch file to start all nodes
- `config/params.yaml` - Parameter configuration file (optional)

## Setup

### 1. Create ROS 2 Workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/your-username/robot-book-code.git
```

### 2. Make Scripts Executable

```bash
cd robot-book-code/chapter-03-ros2-basics/examples
chmod +x *.py
```

### 3. Build Workspace

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## Usage

### Option 1: Run Nodes Individually

**Terminal 1: Sensor Simulator**
```bash
source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/src/robot-book-code/chapter-03-ros2-basics/examples/sensor_simulator.py
```

**Terminal 2: Filter Node**
```bash
source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/src/robot-book-code/chapter-03-ros2-basics/examples/filter_node.py
```

**Terminal 3: Calibration Service**
```bash
source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/src/robot-book-code/chapter-03-ros2-basics/examples/calibration_service.py
```

**Terminal 4: Action Server**
```bash
source ~/ros2_ws/install/setup.bash
python3 ~/ros2_ws/src/robot-book-code/chapter-03-ros2-basics/examples/long_computation_action.py
```

### Option 2: Use Launch File (Recommended)

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch chapter-03-ros2-basics multi_node.launch.py
```

## Testing

### Verify Topics

```bash
# List topics
ros2 topic list

# Check topic rates
ros2 topic hz /imu
ros2 topic hz /imu_filtered

# Echo topic data
ros2 topic echo /imu --once
ros2 topic echo /imu_filtered --once
```

**Expected Output**:
- `/imu` publishes at ~50 Hz
- `/imu_filtered` publishes at ~50 Hz (same rate, filtered data)

### Test Service

```bash
# Calibrate IMU
ros2 service call /calibrate_imu example_interfaces/srv/SetBool "{data: true}"

# Expected response:
# response: example_interfaces.srv.SetBool_Response(success=True, message='Calibration complete. Bias: x=0.05, y=-0.03, z=0.02')

# Reset calibration
ros2 service call /calibrate_imu example_interfaces/srv/SetBool "{data: false}"

# Expected response:
# response: example_interfaces.srv.SetBool_Response(success=True, message='Bias reset to zero')
```

### Test Action

```bash
# Send action goal with feedback
ros2 action send_goal /compute_trajectory example_interfaces/action/Fibonacci "{order: 10}" --feedback

# Expected output:
# Waiting for an action server to become available...
# Sending goal:
#      order: 10
#
# Goal accepted with ID: ...
#
# Feedback:
#     sequence: [0, 1]
#
# Feedback:
#     sequence: [0, 1, 1]
# ...
# Result:
#     sequence: [0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55]
#
# Goal finished with status: SUCCEEDED
```

## Visualize Node Graph

```bash
# Install rqt_graph if not already installed
sudo apt install ros-iron-rqt-graph

# Launch rqt_graph
ros2 run rqt_graph rqt_graph
```

**Expected Graph**:
- `sensor_simulator` → `/imu` → `filter_node` → `/imu_filtered`
- `calibration_service` provides `/calibrate_imu` service
- `long_computation_action_server` provides `/compute_trajectory` action

## Record and Replay Data

### Record Topics

```bash
# Record all topics for 30 seconds
ros2 bag record -a --duration 30 -o chapter3_demo

# Or record specific topics
ros2 bag record /imu /imu_filtered -o chapter3_imu_data
```

### Replay Bag File

```bash
# Play recorded bag
ros2 bag play chapter3_demo

# Play at 2x speed
ros2 bag play chapter3_demo --rate 2.0

# Play in loop
ros2 bag play chapter3_demo --loop
```

## Troubleshooting

**Issue: Topics not visible**
```bash
# Check ROS_DOMAIN_ID matches (default: 0)
echo $ROS_DOMAIN_ID

# List all topics to verify nodes are running
ros2 topic list
```

**Issue: Filter node not receiving data**
```bash
# Check QoS compatibility
ros2 topic info /imu --verbose

# Both publisher and subscriber should have matching QoS policies
```

**Issue: Service call times out**
```bash
# Verify service exists
ros2 service list

# Check if service server is running
ros2 node list
# Should show: /calibration_service
```

**Issue: Action goal rejected**
```bash
# Check if action server is running
ros2 action list

# Send action goal to correct action name
ros2 action info /compute_trajectory
```

## Parameters

### Filter Node Parameters

- `window_size` (int, default: 5) - Moving average window size
  ```bash
  ros2 param set /filter_node window_size 10
  ```

### Sensor Simulator Parameters

- `publish_rate` (float, default: 50.0) - IMU publish rate in Hz
  ```bash
  ros2 param set /sensor_simulator publish_rate 100.0
  ```

## Performance Benchmarks

**Expected Performance** (Intel i7-12700K, 32 GB RAM):
- Topic latency: <1 ms (average)
- Topic rate: 49.5-50.5 Hz (within 1% of target)
- Service response time: <10 ms
- Action feedback rate: ~10 Hz

## License

MIT License - See main repository LICENSE file

## Further Reading

- ROS 2 Documentation: https://docs.ros.org/en/iron/
- ROS 2 Tutorials: https://docs.ros.org/en/iron/Tutorials.html
- Python rclpy API: https://docs.ros2.org/latest/api/rclpy/
