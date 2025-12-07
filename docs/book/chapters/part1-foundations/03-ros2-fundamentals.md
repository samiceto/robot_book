# Chapter 3: ROS 2 Fundamentals - Nodes, Topics, Services, Actions

**Learning Objectives**

By the end of this chapter, you will be able to:

1. Explain the architectural differences between ROS 1 and ROS 2 (DDS middleware, real-time support, security)
2. Create ROS 2 nodes in Python and C++ that publish and subscribe to topics
3. Understand and configure Quality of Service (QoS) policies for reliable communication
4. Implement service servers and clients for request-reply patterns
5. Build action servers with goal, feedback, and result handling for long-running tasks
6. Use parameters and launch files to configure multi-node systems
7. Debug ROS 2 systems using command-line tools (`ros2 topic`, `ros2 service`, `rqt_graph`, `ros2 bag`)
8. Build a complete multi-node sensor processing system with topics, services, and actions

**Prerequisites**: Completed Chapter 2 (ROS 2 Iron installed, development environment configured)

**Estimated Time**: 12-16 hours (including hands-on lab and project)

---

## 1. ROS 2 Conceptual Overview

### 1.1 Why ROS 2 vs. ROS 1 (DDS, Real-Time, Security)

ROS 1 (Robot Operating System 1) revolutionized robotics research from 2007-2020, enabling rapid prototyping through a rich ecosystem of packages. However, industrial adoption revealed critical limitations that ROS 2 addresses.

**ROS 1 Limitations**:

1. **No Real-Time Support**: ROS 1 uses TCP for communication, which has unpredictable latency due to packet buffering and retransmission. Critical control loops (motor controllers running at 1 kHz) cannot tolerate 100+ ms latency spikes.

2. **Single Master Architecture**: `roscore` is a single point of failure. If the ROS master crashes, the entire system fails. Multi-robot systems require complex master synchronization.

3. **No Native Security**: All ROS 1 communication is unencrypted. Any node on the network can subscribe to any topic, publish malicious data, or eavesdrop on sensor streams. This is unacceptable for commercial robots handling sensitive data.

4. **Poor Windows Support**: ROS 1 was designed for Ubuntu. Windows and macOS support was an afterthought, limiting adoption in mixed-OS environments (e.g., Windows developer laptops + Linux robots).

**ROS 2 Solutions**:

**DDS Middleware (Data Distribution Service)**:
- ROS 2 uses DDS, an OMG (Object Management Group) standard middleware designed for real-time, distributed systems
- DDS implementations: Fast DDS (default), CycloneDDS, Connext DDS
- **Key Advantage**: DDS provides Quality of Service (QoS) policies—you can specify reliability (guaranteed delivery vs. best-effort), durability (late-joining subscribers get historical data), and deadline (detect when publishers go silent)

**No Single Point of Failure**:
- ROS 2 uses DDS discovery—nodes find each other via multicast without a central master
- Each node is autonomous; if one crashes, others continue operating
- Multi-robot systems work out-of-the-box (set different `ROS_DOMAIN_ID` values)

**DDS-Security (SROS2)**:
- Encrypt all communication with AES-256
- Authenticate nodes with X.509 certificates (PKI infrastructure)
- Fine-grained access control (node A can publish to `/cmd_vel` but not subscribe to `/camera`)
- **Use Case**: Warehouse robots handling proprietary inventory data; hospital robots processing patient information

**Cross-Platform Support**:
- ROS 2 officially supports Ubuntu, Windows 10/11, macOS (Intel + Apple Silicon)
- Real-time operating systems: VxWorks, QNX, FreeRTOS (for embedded systems)

**Migration Path**:
- ROS 1 Bridge package allows ROS 1 and ROS 2 nodes to communicate during transition
- Most popular ROS 1 packages (MoveIt, Nav2, Gazebo) have ROS 2 equivalents

**Performance Comparison** (benchmarks on Intel i7-12700K):

| Metric | ROS 1 (Noetic) | ROS 2 (Iron) |
|--------|----------------|--------------|
| **Topic Latency (avg)** | 1.2 ms | 0.8 ms |
| **Topic Latency (p99)** | 15 ms | 3 ms |
| **Throughput (10 KB msgs)** | 450 MB/s | 680 MB/s |
| **Discovery Time (50 nodes)** | 8 seconds | 2 seconds |

**Source**: ROS 2 Performance Benchmarks (Open Robotics, 2023)

### 1.2 ROS 2 Architecture: Nodes, Topics, Services, Actions, Parameters

ROS 2 systems are composed of **nodes**—independent processes that communicate via well-defined interfaces. This modular architecture enables code reuse, parallel development, and graceful degradation.

**Node**: A single-purpose executable (e.g., `camera_driver`, `object_detector`, `motor_controller`)
- Written in Python, C++, or other languages (Rust, Java support via client libraries)
- Runs as an OS process (one node = one process)
- Can be composed into a single process for performance (composition)

**Communication Patterns**:

**1. Topics (Publish-Subscribe)**
- **Use Case**: Streaming sensor data (camera images at 30 Hz, IMU at 100 Hz)
- **Pattern**: One-to-many (1 publisher, N subscribers) or many-to-many
- **Example**: `/camera/rgb` topic carries `sensor_msgs/Image` messages

**2. Services (Request-Reply)**
- **Use Case**: Trigger an action and wait for result (reset odometry, switch camera mode)
- **Pattern**: One-to-one (client sends request, server sends response)
- **Example**: `/reset_odometry` service (Empty request → Empty response)

**3. Actions (Goal-Feedback-Result)**
- **Use Case**: Long-running tasks with progress updates (navigate to waypoint, grasp object)
- **Pattern**: One-to-one with streaming feedback
- **Example**: `/navigate_to_pose` action sends periodic feedback (distance remaining, ETA)

**4. Parameters**
- **Use Case**: Configure node behavior without recompiling (set PID gains, camera resolution)
- **Pattern**: Key-value store per node
- **Example**: `camera.exposure_time` parameter (integer, microseconds)

**Analogy to Software Design Patterns**:
- Topics ≈ Observer pattern (subscribers observe publisher state changes)
- Services ≈ Remote Procedure Call (RPC)
- Actions ≈ Futures/Promises with progress callbacks
- Parameters ≈ Configuration files

### 1.3 Quality of Service (QoS) Policies

QoS policies control the reliability and performance of topic communication. Mismatched QoS between publisher and subscriber prevents communication—this is a common pitfall.

**Key QoS Policies**:

**1. Reliability**
- **`RELIABLE`**: Guaranteed delivery (TCP-like). DDS retransmits lost packets. Use for critical data (joint commands, goal poses).
- **`BEST_EFFORT`**: No retransmission (UDP-like). Accepts packet loss. Use for high-frequency sensor data (camera images, LiDAR scans).

**2. Durability**
- **`VOLATILE`**: Late-joining subscribers only receive messages published after subscription.
- **`TRANSIENT_LOCAL`**: Late-joining subscribers receive the last N messages (determined by History depth). Use for configuration topics (e.g., robot state).

**3. History**
- **`KEEP_LAST(N)`**: Store last N messages. If queue fills, oldest message is dropped.
- **`KEEP_ALL`**: Store all messages (unbounded queue—risk of memory exhaustion).

**4. Deadline**
- Detect when publisher stops publishing. If no message received within deadline period, subscriber is notified.
- **Use Case**: Motor safety—if `/cmd_vel` topic goes silent for >500 ms, emergency stop.

**5. Lifespan**
- Messages expire after a time duration (e.g., 1 second). Stale data is discarded.
- **Use Case**: GPS waypoints that are only valid for 10 seconds.

**Common QoS Presets** (defined in `rclpy.qos` and `rclcpp`):

```python
# Python example
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Sensor data preset (best-effort, volatile, keep last 10)
sensor_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

# System default preset (reliable, volatile, keep last 10)
default_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

# Parameter events preset (reliable, transient local, keep all)
parameter_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_ALL
)
```

**QoS Compatibility**:
- Publishers and subscribers must have compatible QoS
- **Compatible**: Reliable publisher + Reliable subscriber ✅
- **Compatible**: Best-effort publisher + Best-effort subscriber ✅
- **Incompatible**: Reliable subscriber + Best-effort publisher ❌ (subscriber won't receive messages)
- **Rule**: Subscribers can request stricter reliability than publishers offer, but not vice versa

**Debugging QoS Mismatches**:
```bash
# Check QoS settings for a topic
ros2 topic info /camera/rgb --verbose

# Output shows publisher and subscriber QoS policies
Publishers:
  * /camera_driver (sensor_msgs/msg/Image)
    QoS: Reliability=BEST_EFFORT, Durability=VOLATILE, Depth=10
Subscribers:
  * /object_detector (sensor_msgs/msg/Image)
    QoS: Reliability=RELIABLE, Durability=VOLATILE, Depth=10
    WARNING: QoS mismatch - subscriber won't receive messages!
```

---

## 2. Nodes and Topics

### 2.1 Creating a Simple Publisher (Python)

Let's create a publisher that publishes IMU (Inertial Measurement Unit) data at 50 Hz.

**File**: `sensor_simulator.py`

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import numpy as np

class SensorSimulator(Node):
    def __init__(self):
        super().__init__('sensor_simulator')

        # Create publisher for /imu topic
        self.publisher_ = self.create_publisher(Imu, '/imu', 10)

        # Create timer (20 ms = 50 Hz)
        self.timer = self.create_timer(0.02, self.timer_callback)

        # Simulation state
        self.count = 0

        self.get_logger().info('Sensor simulator started, publishing to /imu at 50 Hz')

    def timer_callback(self):
        msg = Imu()

        # Header with timestamp
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # Simulate accelerometer (gravity + noise)
        msg.linear_acceleration.x = np.random.normal(0.0, 0.1)
        msg.linear_acceleration.y = np.random.normal(0.0, 0.1)
        msg.linear_acceleration.z = np.random.normal(9.81, 0.1)

        # Simulate gyroscope (random noise)
        msg.angular_velocity.x = np.random.normal(0.0, 0.05)
        msg.angular_velocity.y = np.random.normal(0.0, 0.05)
        msg.angular_velocity.z = np.random.normal(0.0, 0.05)

        # Orientation (set to identity quaternion)
        msg.orientation.w = 1.0
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0

        # Publish message
        self.publisher_.publish(msg)

        self.count += 1
        if self.count % 100 == 0:  # Log every 2 seconds
            self.get_logger().info(f'Published {self.count} IMU messages')

def main(args=None):
    rclpy.init(args=args)
    node = SensorSimulator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Running the Publisher**:
```bash
chmod +x sensor_simulator.py
python3 sensor_simulator.py

# Output:
# [INFO] [sensor_simulator]: Sensor simulator started, publishing to /imu at 50 Hz
# [INFO] [sensor_simulator]: Published 100 IMU messages
# [INFO] [sensor_simulator]: Published 200 IMU messages
```

**Code Breakdown**:
- `super().__init__('sensor_simulator')`: Initialize node with name `sensor_simulator`
- `self.create_publisher(Imu, '/imu', 10)`: Create publisher for topic `/imu` with message type `Imu` and queue size 10
- `self.create_timer(0.02, self.timer_callback)`: Call `timer_callback()` every 20 ms (50 Hz)
- `self.get_clock().now().to_msg()`: Get current ROS time (not system time—important for simulation and bag playback)

### 2.2 Creating a Simple Subscriber (Python)

Now create a subscriber that listens to `/imu` and applies a simple moving average filter.

**File**: `filter_node.py`

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from collections import deque

class FilterNode(Node):
    def __init__(self):
        super().__init__('filter_node')

        # Create subscriber
        self.subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            10
        )

        # Create publisher for filtered data
        self.publisher_ = self.create_publisher(Imu, '/imu_filtered', 10)

        # Moving average filter (window size = 5)
        self.window_size = 5
        self.accel_x_buffer = deque(maxlen=self.window_size)
        self.accel_y_buffer = deque(maxlen=self.window_size)
        self.accel_z_buffer = deque(maxlen=self.window_size)

        self.get_logger().info('Filter node started, subscribing to /imu')

    def imu_callback(self, msg):
        # Add to buffers
        self.accel_x_buffer.append(msg.linear_acceleration.x)
        self.accel_y_buffer.append(msg.linear_acceleration.y)
        self.accel_z_buffer.append(msg.linear_acceleration.z)

        # Compute moving average
        filtered_msg = Imu()
        filtered_msg.header = msg.header
        filtered_msg.header.frame_id = 'imu_link_filtered'

        filtered_msg.linear_acceleration.x = sum(self.accel_x_buffer) / len(self.accel_x_buffer)
        filtered_msg.linear_acceleration.y = sum(self.accel_y_buffer) / len(self.accel_y_buffer)
        filtered_msg.linear_acceleration.z = sum(self.accel_z_buffer) / len(self.accel_z_buffer)

        # Copy gyroscope (no filtering)
        filtered_msg.angular_velocity = msg.angular_velocity
        filtered_msg.orientation = msg.orientation

        # Publish filtered data
        self.publisher_.publish(filtered_msg)

def main(args=None):
    rclpy.init(args=args)
    node = FilterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Testing Publisher + Subscriber**:
```bash
# Terminal 1: Run publisher
python3 sensor_simulator.py

# Terminal 2: Run subscriber
python3 filter_node.py

# Terminal 3: Check topics
ros2 topic list
# Output:
# /imu
# /imu_filtered
# /parameter_events
# /rosout

# Terminal 3: Verify rates
ros2 topic hz /imu
# average rate: 50.023 Hz

ros2 topic hz /imu_filtered
# average rate: 50.018 Hz
```

### 2.3 Understanding Message Types (std_msgs, sensor_msgs, geometry_msgs)

ROS 2 messages are defined in `.msg` files and compiled to Python/C++ classes.

**Common Message Packages**:

**1. std_msgs** (standard messages):
```
std_msgs/String - string data
std_msgs/Int32 - int32 data
std_msgs/Float64 - float64 data
std_msgs/Bool - bool data
std_msgs/Header - timestamp + frame_id
```

**2. sensor_msgs** (sensor data):
```
sensor_msgs/Image - camera images
sensor_msgs/Imu - IMU (accelerometer + gyroscope)
sensor_msgs/LaserScan - 2D LiDAR scans
sensor_msgs/PointCloud2 - 3D point clouds
sensor_msgs/JointState - robot joint positions/velocities/efforts
```

**3. geometry_msgs** (geometric primitives):
```
geometry_msgs/Point - (x, y, z)
geometry_msgs/Pose - position + orientation (quaternion)
geometry_msgs/Twist - linear + angular velocity
geometry_msgs/Transform - translation + rotation
```

**Inspecting Message Definitions**:
```bash
# Show Imu message structure
ros2 interface show sensor_msgs/msg/Imu

# Output:
# std_msgs/Header header
#   builtin_interfaces/Time stamp
#   string frame_id
# geometry_msgs/Quaternion orientation
# float64[9] orientation_covariance
# geometry_msgs/Vector3 angular_velocity
# float64[9] angular_velocity_covariance
# geometry_msgs/Vector3 linear_acceleration
# float64[9] linear_acceleration_covariance
```

### 2.4 QoS Policies for Topics: Reliability, Durability, History

Let's modify the publisher to use best-effort QoS (appropriate for high-frequency sensor data):

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class SensorSimulator(Node):
    def __init__(self):
        super().__init__('sensor_simulator')

        # Define QoS profile (best-effort for sensor data)
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Create publisher with custom QoS
        self.publisher_ = self.create_publisher(Imu, '/imu', qos_profile)
        # ... rest of code
```

**Subscriber with Matching QoS**:
```python
class FilterNode(Node):
    def __init__(self):
        super().__init__('filter_node')

        # Subscriber must use compatible QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
            qos_profile
        )
        # ... rest of code
```

---

## 3. Services for Request-Reply

### 3.1 Defining a Custom Service (.srv Files)

Services use `.srv` files with request and response fields separated by `---`.

**File**: `my_interfaces/srv/CalibrateImu.srv`

```
# Request (empty - just trigger calibration)
---
# Response
bool success
string message
```

**Building Custom Messages** (requires ROS 2 package):
```bash
# Create package for custom interfaces
cd ~/ros2_ws/src
ros2 pkg create my_interfaces --build-type ament_cmake

# Add srv file
mkdir -p my_interfaces/srv
# Copy CalibrateImu.srv to my_interfaces/srv/

# Edit CMakeLists.txt to add:
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CalibrateImu.srv"
)

# Build
cd ~/ros2_ws
colcon build --packages-select my_interfaces
source install/setup.bash
```

### 3.2 Implementing a Service Server (Python)

**File**: `calibration_service.py`

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool  # Using built-in service for simplicity

class CalibrationService(Node):
    def __init__(self):
        super().__init__('calibration_service')

        # Create service server
        self.srv = self.create_service(
            SetBool,
            '/calibrate_imu',
            self.calibrate_callback
        )

        self.bias_x = 0.0
        self.bias_y = 0.0
        self.bias_z = 0.0

        self.get_logger().info('Calibration service ready at /calibrate_imu')

    def calibrate_callback(self, request, response):
        """
        Request: bool data (True = calibrate, False = reset)
        Response: bool success, string message
        """
        if request.data:  # Calibrate
            # Simulate calibration (in reality, would collect samples)
            self.bias_x = 0.05
            self.bias_y = -0.03
            self.bias_z = 0.02

            response.success = True
            response.message = f'Calibration complete. Bias: x={self.bias_x}, y={self.bias_y}, z={self.bias_z}'

            self.get_logger().info(response.message)
        else:  # Reset
            self.bias_x = 0.0
            self.bias_y = 0.0
            self.bias_z = 0.0

            response.success = True
            response.message = 'Bias reset to zero'

            self.get_logger().info(response.message)

        return response

def main(args=None):
    rclpy.init(args=args)
    node = CalibrationService()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3.3 Calling a Service from a Client (Python)

**Command-Line Client**:
```bash
# Call service from terminal
ros2 service call /calibrate_imu example_interfaces/srv/SetBool "{data: true}"

# Output:
# requester: making request: example_interfaces.srv.SetBool_Request(data=True)
# response: example_interfaces.srv.SetBool_Response(success=True, message='Calibration complete. Bias: x=0.05, y=-0.03, z=0.02')
```

**Programmatic Client**:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool

class CalibrationClient(Node):
    def __init__(self):
        super().__init__('calibration_client')

        # Create client
        self.client = self.create_client(SetBool, '/calibrate_imu')

        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /calibrate_imu service...')

        self.get_logger().info('Service available')

    def send_request(self, calibrate=True):
        request = SetBool.Request()
        request.data = calibrate

        # Call service asynchronously
        future = self.client.call_async(request)
        return future

def main(args=None):
    rclpy.init(args=args)
    client = CalibrationClient()

    # Send request
    future = client.send_request(calibrate=True)

    # Wait for response
    rclpy.spin_until_future_complete(client, future)

    if future.result() is not None:
        response = future.result()
        client.get_logger().info(f'Service response: {response.message}')
    else:
        client.get_logger().error('Service call failed')

    client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3.4 Synchronous vs. Asynchronous Service Calls

**Synchronous Call** (blocks until response):
```python
# WARNING: Blocks the executor - don't use in timer callbacks
request = SetBool.Request()
request.data = True
response = self.client.call(request)  # Blocks here
print(response.message)
```

**Asynchronous Call** (non-blocking):
```python
# Preferred method - doesn't block
future = self.client.call_async(request)

# Option 1: Spin until complete
rclpy.spin_until_future_complete(node, future)
response = future.result()

# Option 2: Add callback
def response_callback(future):
    response = future.result()
    node.get_logger().info(f'Got response: {response.message}')

future.add_done_callback(response_callback)
```

---

## 4. Actions for Long-Running Tasks

### 4.1 Action Definition (.action Files): Goal, Feedback, Result

Actions have three components: Goal (what to do), Feedback (progress updates), Result (final outcome).

**File**: `my_interfaces/action/ComputeTrajectory.action`

```
# Goal
float64 computation_time  # How long to compute (seconds)
---
# Result
bool success
string message
float64[] trajectory  # Computed trajectory (dummy data)
---
# Feedback
float64 percent_complete
float64 time_remaining
```

**Build Action**:
```bash
# Edit CMakeLists.txt to add action file
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/CalibrateImu.srv"
  "action/ComputeTrajectory.action"
  DEPENDENCIES builtin_interfaces
)

# Rebuild
cd ~/ros2_ws
colcon build --packages-select my_interfaces
source install/setup.bash
```

### 4.2 Implementing an Action Server (Python)

**File**: `long_computation_action.py`

```python
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci  # Using built-in action for simplicity
import time

class LongComputationActionServer(Node):
    def __init__(self):
        super().__init__('long_computation_action_server')

        self._action_server = ActionServer(
            self,
            Fibonacci,
            '/compute_trajectory',
            self.execute_callback
        )

        self.get_logger().info('Action server ready at /compute_trajectory')

    def execute_callback(self, goal_handle):
        """
        Execute the goal and publish feedback.

        Args:
            goal_handle: Handle to manage goal execution

        Returns:
            Result message
        """
        self.get_logger().info(f'Executing goal: order={goal_handle.request.order}')

        # Simulate 5-second computation
        total_duration = 5.0
        update_rate = 10  # Hz
        num_updates = int(total_duration * update_rate)

        # Initialize feedback
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        # Simulate computation with progress feedback
        for i in range(num_updates):
            # Check if goal was canceled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                result = Fibonacci.Result()
                result.sequence = feedback_msg.sequence
                return result

            # Compute next Fibonacci number (limited to goal.order)
            if len(feedback_msg.sequence) < goal_handle.request.order:
                feedback_msg.sequence.append(
                    feedback_msg.sequence[-1] + feedback_msg.sequence[-2]
                )

            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)

            percent_complete = (i + 1) / num_updates * 100
            self.get_logger().info(f'Progress: {percent_complete:.1f}%')

            time.sleep(1.0 / update_rate)

        # Mark goal as succeeded
        goal_handle.succeed()

        # Return result
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence

        self.get_logger().info(f'Goal succeeded with result: {result.sequence}')
        return result

def main(args=None):
    rclpy.init(args=args)
    node = LongComputationActionServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 4.3 Implementing an Action Client with Feedback Handling (Python)

**File**: `action_client.py`

```python
#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from example_interfaces.action import Fibonacci

class ComputeTrajectoryClient(Node):
    def __init__(self):
        super().__init__('compute_trajectory_client')

        self._action_client = ActionClient(self, Fibonacci, '/compute_trajectory')

        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Action server available')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self.get_logger().info(f'Sending goal: order={order}')

        # Send goal with callbacks
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        self.get_logger().info('Goal accepted')

        # Wait for result
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Feedback: {feedback.sequence}')

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.sequence}')

        # Shutdown after receiving result
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    client = ComputeTrajectoryClient()

    # Send goal
    client.send_goal(order=10)

    # Spin until result received
    rclpy.spin(client)

if __name__ == '__main__':
    main()
```

**Testing Action Server + Client**:
```bash
# Terminal 1: Run action server
python3 long_computation_action.py

# Terminal 2: Run action client
python3 action_client.py

# Terminal 1 Output:
# [INFO] [long_computation_action_server]: Executing goal: order=10
# [INFO] [long_computation_action_server]: Progress: 10.0%
# [INFO] [long_computation_action_server]: Progress: 20.0%
# ...
# [INFO] [long_computation_action_server]: Goal succeeded with result: [0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55]

# Terminal 2 Output:
# [INFO] [compute_trajectory_client]: Sending goal: order=10
# [INFO] [compute_trajectory_client]: Goal accepted
# [INFO] [compute_trajectory_client]: Feedback: [0, 1]
# [INFO] [compute_trajectory_client]: Feedback: [0, 1, 1]
# ...
# [INFO] [compute_trajectory_client]: Result: [0, 1, 1, 2, 3, 5, 8, 13, 21, 34, 55]
```

### 4.4 Use Case: Navigation Goal with Progress Feedback

Actions are ideal for navigation tasks where the robot moves to a goal pose and provides periodic feedback on distance remaining.

**Pseudocode for Navigation Action**:
```python
# Goal: target pose (x, y, theta)
# Feedback: current pose, distance remaining, ETA
# Result: final pose, success/failure

class NavigateToP oseActionServer:
    def execute_callback(self, goal_handle):
        target_pose = goal_handle.request.pose

        while not_at_target(target_pose):
            current_pose = get_current_pose()
            distance = compute_distance(current_pose, target_pose)

            # Publish feedback
            feedback = NavigateToPose.Feedback()
            feedback.current_pose = current_pose
            feedback.distance_remaining = distance
            feedback.eta = distance / velocity
            goal_handle.publish_feedback(feedback)

            # Move toward target
            cmd_vel = compute_velocity(current_pose, target_pose)
            publish_cmd_vel(cmd_vel)

            sleep(0.1)  # 10 Hz control loop

        # Goal reached
        result = NavigateToPose.Result()
        result.final_pose = get_current_pose()
        result.success = True
        return result
```

---

## 5. Parameters and Launch Files

### 5.1 Declaring and Using Parameters in Nodes (Python)

Parameters allow runtime configuration without recompiling code.

**File**: `configurable_filter.py`

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from collections import deque

class ConfigurableFilterNode(Node):
    def __init__(self):
        super().__init__('configurable_filter')

        # Declare parameters with default values
        self.declare_parameter('window_size', 5)
        self.declare_parameter('output_topic', '/imu_filtered')
        self.declare_parameter('publish_rate', 50.0)

        # Get parameter values
        window_size = self.get_parameter('window_size').value
        output_topic = self.get_parameter('output_topic').value
        publish_rate = self.get_parameter('publish_rate').value

        self.get_logger().info(f'Parameters: window_size={window_size}, output_topic={output_topic}, publish_rate={publish_rate}')

        # Create subscriber and publisher
        self.subscription = self.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.publisher_ = self.create_publisher(Imu, output_topic, 10)

        # Initialize buffers
        self.accel_x_buffer = deque(maxlen=window_size)
        self.accel_y_buffer = deque(maxlen=window_size)
        self.accel_z_buffer = deque(maxlen=window_size)

    def imu_callback(self, msg):
        # ... same filtering logic as before ...
        pass

def main(args=None):
    rclpy.init(args=args)
    node = ConfigurableFilterNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

**Setting Parameters at Runtime**:
```bash
# Set parameters via command line
ros2 run my_package configurable_filter.py --ros-args \
  -p window_size:=10 \
  -p output_topic:=/imu_smoothed \
  -p publish_rate:=100.0

# List parameters of running node
ros2 param list /configurable_filter
# Output:
# publish_rate
# output_topic
# use_sim_time
# window_size

# Get parameter value
ros2 param get /configurable_filter window_size
# Integer value is: 10

# Set parameter on running node
ros2 param set /configurable_filter window_size 20
# Set parameter successful
```

### 5.2 Launch Files: Orchestrating Multi-Node Systems (Python)

Launch files start multiple nodes with specified parameters and remappings.

**File**: `multi_node.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Sensor simulator node
        Node(
            package='my_package',
            executable='sensor_simulator.py',
            name='sensor_simulator',
            output='screen',
            parameters=[{
                'publish_rate': 50.0
            }]
        ),

        # Filter node
        Node(
            package='my_package',
            executable='configurable_filter.py',
            name='filter_node',
            output='screen',
            parameters=[{
                'window_size': 10,
                'output_topic': '/imu_filtered',
                'publish_rate': 50.0
            }]
        ),

        # Calibration service
        Node(
            package='my_package',
            executable='calibration_service.py',
            name='calibration_service',
            output='screen'
        ),

        # Action server
        Node(
            package='my_package',
            executable='long_computation_action.py',
            name='long_computation_action_server',
            output='screen'
        ),
    ])
```

**Running Launch File**:
```bash
ros2 launch my_package multi_node.launch.py

# Output shows all nodes starting:
# [sensor_simulator-1] [INFO] [sensor_simulator]: Sensor simulator started
# [filter_node-2] [INFO] [configurable_filter]: Parameters: window_size=10, output_topic=/imu_filtered
# [calibration_service-3] [INFO] [calibration_service]: Calibration service ready
# [long_computation_action_server-4] [INFO] [long_computation_action_server]: Action server ready
```

### 5.3 YAML Parameter Files for Configuration

**File**: `config/params.yaml`

```yaml
sensor_simulator:
  ros__parameters:
    publish_rate: 100.0
    add_noise: true
    noise_stddev: 0.05

filter_node:
  ros__parameters:
    window_size: 20
    output_topic: '/imu_smoothed'
    publish_rate: 100.0
```

**Launch File with YAML Parameters**:

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('my_package'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='my_package',
            executable='sensor_simulator.py',
            name='sensor_simulator',
            parameters=[config]
        ),
        Node(
            package='my_package',
            executable='configurable_filter.py',
            name='filter_node',
            parameters=[config]
        ),
    ])
```

---

## 6. Debugging and Introspection Tools

### 6.1 `ros2 topic` Commands

**List all topics**:
```bash
ros2 topic list
```

**Echo topic data**:
```bash
# Print all messages
ros2 topic echo /imu

# Print only once
ros2 topic echo /imu --once

# Print without arrays (for large data)
ros2 topic echo /imu --no-arr
```

**Check topic rate**:
```bash
ros2 topic hz /imu
# average rate: 50.023 Hz
#   min: 0.019s max: 0.021s std dev: 0.00012s window: 51
```

**Topic info**:
```bash
ros2 topic info /imu --verbose
# Type: sensor_msgs/msg/Imu
# Publisher count: 1
# Subscription count: 1
#
# Publishers:
#   * /sensor_simulator (rmw_fastrtps_cpp)
#     QoS: Reliability=BEST_EFFORT, Durability=VOLATILE, Depth=10
#
# Subscriptions:
#   * /filter_node (rmw_fastrtps_cpp)
#     QoS: Reliability=BEST_EFFORT, Durability=VOLATILE, Depth=10
```

**Publish to topic from command line**:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.1}}" --rate 10
```

### 6.2 `ros2 service` Commands

**List services**:
```bash
ros2 service list
```

**Call service**:
```bash
ros2 service call /calibrate_imu example_interfaces/srv/SetBool "{data: true}"
```

**Service type**:
```bash
ros2 service type /calibrate_imu
# example_interfaces/srv/SetBool
```

### 6.3 `rqt_graph` for Visualizing Node Connections

`rqt_graph` provides a visual graph of nodes, topics, services, and actions.

**Launch rqt_graph**:
```bash
ros2 run rqt_graph rqt_graph
```

**Graph shows**:
- Ovals: Nodes
- Boxes: Topics
- Arrows: Publisher → Topic → Subscriber relationships

**Use Case**: Debug why a subscriber isn't receiving messages (check if topic names match, if QoS is compatible).

### 6.4 `ros2 bag` for Recording and Replaying Data

ROS 2 bags record all topic data to a database file (SQLite3) for later replay.

**Record topics**:
```bash
# Record specific topics
ros2 bag record /imu /imu_filtered -o my_bag

# Record all topics
ros2 bag record -a -o full_system_bag

# Output:
# [INFO] [rosbag2_storage]: Opened database 'my_bag/my_bag_0.db3'.
# [INFO] [rosbag2_transport]: Listening for topics...
# [INFO] [rosbag2_transport]: Subscribed to topic '/imu'
# [INFO] [rosbag2_transport]: Subscribed to topic '/imu_filtered'
# Press Ctrl+C to stop recording
```

**Inspect bag**:
```bash
ros2 bag info my_bag

# Output:
# Files:             my_bag_0.db3
# Bag size:          2.3 MB
# Storage id:        sqlite3
# Duration:          30.2s
# Start:             Jan 15 2025 10:30:15.123
# End:               Jan 15 2025 10:30:45.321
# Messages:          3020
# Topic information:
#   Topic: /imu | Type: sensor_msgs/msg/Imu | Count: 1510 | Serialization Format: cdr
#   Topic: /imu_filtered | Type: sensor_msgs/msg/Imu | Count: 1510 | Serialization Format: cdr
```

**Replay bag**:
```bash
ros2 bag play my_bag

# Replay at 2x speed
ros2 bag play my_bag --rate 2.0

# Replay in loop
ros2 bag play my_bag --loop
```

**Use Cases**:
- **Offline Development**: Record sensor data in field, develop algorithms offline
- **Regression Testing**: Record ground-truth data, replay to test algorithm changes
- **Debugging**: Record data when bug occurs, replay multiple times to debug

---

## 7. Hands-On Lab: Building a Multi-Node System

**Objective**: Build a complete sensor processing system with 4 nodes communicating via topics, services, and actions.

**Time Estimate**: 8 hours

**System Architecture**:
```
sensor_simulator → /imu (50 Hz) → filter_node → /imu_filtered (50 Hz)
                                         ↓
                               calibration_service (/calibrate_imu service)
                               long_computation_action (/compute_trajectory action)
```

**Tasks**:

1. ✅ Create `sensor_simulator.py` (already completed in Section 2.1)
2. ✅ Create `filter_node.py` (already completed in Section 2.2)
3. ✅ Create `calibration_service.py` (already completed in Section 3.2)
4. ✅ Create `long_computation_action.py` (already completed in Section 4.2)
5. Create launch file `multi_node.launch.py` (see Section 5.2)

**Step 6: Test the Complete System**

**Terminal 1: Launch all nodes**:
```bash
ros2 launch my_package multi_node.launch.py
```

**Terminal 2: Verify topics**:
```bash
ros2 topic hz /imu
ros2 topic hz /imu_filtered

# Both should show ~50 Hz
```

**Terminal 3: Call calibration service**:
```bash
ros2 service call /calibrate_imu example_interfaces/srv/SetBool "{data: true}"

# Expected: success=True, message='Calibration complete...'
```

**Terminal 4: Send action goal**:
```bash
ros2 action send_goal /compute_trajectory example_interfaces/action/Fibonacci "{order: 15}" --feedback

# Expected: Periodic feedback with Fibonacci sequence, then result
```

**Validation Criteria**:
- ✅ Launch file starts all 4 nodes without errors
- ✅ `/imu` topic publishes at 49-51 Hz
- ✅ `/imu_filtered` topic publishes at 49-51 Hz
- ✅ Calibration service responds within 100 ms
- ✅ Action server provides feedback every ~500 ms
- ✅ Action completes after ~5 seconds

**Common Issues**:

**Issue 1: Topic rate drops below 40 Hz**
- **Cause**: System CPU overload
- **Solution**: Reduce publish rate to 30 Hz or run on more powerful hardware

**Issue 2: Service call times out**
- **Cause**: Service server not running or different namespace
- **Solution**: `ros2 service list` to verify service exists

**Issue 3: Action feedback not received**
- **Cause**: Action server not publishing feedback
- **Solution**: Check action server code has `goal_handle.publish_feedback()`

---

## 8. End-of-Chapter Project: Humanoid Joint Controller Action Server

**Objective**: Create a ROS 2 action server that accepts a joint trajectory goal and executes it on the Isaac Sim humanoid from Chapter 2.

**Requirements**:

1. **Action Definition** (`JointTrajectory.action`):
```
# Goal
string[] joint_names
float64[] target_positions
float64 duration
---
# Result
bool success
string message
float64 final_error  # Sum of squared errors
---
# Feedback
float64 percent_complete
float64 time_remaining
float64[] current_positions
```

2. **Action Server** (`humanoid_joint_controller.py`):
- Subscribe to `/joint_states` to get current joint positions
- Interpolate from current positions to target positions over `duration` seconds
- Publish joint commands to Isaac Sim (via `/joint_command` topic)
- Provide feedback every 100 ms (10 Hz)
- Validate final positions are within 5% of target

3. **Launch File** (`humanoid_controller.launch.py`):
- Start Isaac Sim ROS 2 bridge
- Start action server
- Load humanoid URDF from Chapter 2

**Example Usage**:
```bash
# Terminal 1: Launch Isaac Sim + action server
ros2 launch my_package humanoid_controller.launch.py

# Terminal 2: Send goal (move arms to 90° shoulder flexion)
ros2 action send_goal /joint_trajectory my_interfaces/action/JointTrajectory \
  "{joint_names: ['left_shoulder', 'right_shoulder'], target_positions: [1.57, 1.57], duration: 3.0}" \
  --feedback

# Expected feedback:
# Feedback: percent_complete=10.0, time_remaining=2.7, current_positions=[0.17, 0.17]
# Feedback: percent_complete=20.0, time_remaining=2.4, current_positions=[0.31, 0.31]
# ...
# Feedback: percent_complete=100.0, time_remaining=0.0, current_positions=[1.57, 1.57]
#
# Result: success=True, message='Trajectory executed successfully', final_error=0.002
```

**Deliverables**:
1. Action definition file (`JointTrajectory.action`)
2. Action server code (`humanoid_joint_controller.py`)
3. Launch file (`humanoid_controller.launch.py`)
4. Video demonstration (30-60 seconds) showing Isaac Sim humanoid executing smooth arm motion

**Validation Criteria**:
- ✅ Trajectory completes within 10% of specified duration (e.g., 3.0 ± 0.3 seconds)
- ✅ Final joint positions within 5% of target (e.g., 1.57 ± 0.08 radians)
- ✅ Feedback published at 10 Hz (±1 Hz)
- ✅ No joint limit violations (checked via URDF limits)
- ✅ Smooth motion (no abrupt velocity changes)

---

## 9. Further Reading

**Official Documentation**:
- ROS 2 Design Documentation: https://design.ros2.org/
- DDS Specification: https://www.omg.org/spec/DDS/
- ROS 2 QoS Policies: https://docs.ros.org/en/iron/Concepts/About-Quality-of-Service-Settings.html

**Recommended Books**:
- *Programming Robots with ROS 2* (Ayala & Koubâa, 2023)
- *ROS 2 Robotics Developer Guide* (Ibrahim, 2024)

**Community Resources**:
- ROS Discourse: https://discourse.ros.org/
- ROS Answers: https://answers.ros.org/
- GitHub Discussions: https://github.com/ros2/ros2/discussions

---

## Chapter Summary

This chapter introduced ROS 2 fundamentals through hands-on examples. You learned the architectural advantages of ROS 2 over ROS 1 (DDS middleware, no single point of failure, security), created publishers and subscribers with QoS policies, implemented services for request-reply patterns, and built action servers for long-running tasks with feedback.

You configured nodes with parameters, orchestrated multi-node systems with launch files, and debugged using `ros2 topic`, `ros2 service`, `rqt_graph`, and `ros2 bag`. Finally, you built a complete sensor processing system with 4 nodes and an end-of-chapter project controlling a humanoid's joints via action server.

**Key Takeaways**:
1. ROS 2 uses DDS middleware for real-time, decentralized, secure communication
2. QoS policies must be compatible between publishers and subscribers (e.g., reliable vs. best-effort)
3. Services are synchronous request-reply (block until response); actions are asynchronous with feedback
4. Parameters enable runtime configuration without recompiling
5. Launch files simplify starting multi-node systems with specified parameters
6. `ros2 topic hz` and `ros2 bag` are essential debugging tools

**Next Steps**: In Chapter 4, we'll dive into robot description formats—URDF, SDF, and Xacro for humanoids. You'll learn to define links (visual, collision, inertial properties), joints (revolute, prismatic, fixed), and create modular humanoid models with Xacro macros.

---

## References

Ayala, A., & Koubâa, A. (2023). *Programming robots with ROS 2: Develop robotics applications with the most powerful open-source framework*. Packt Publishing.

Ibrahim, A. (2024). *ROS 2 robotics developer guide*. Packt Publishing.

Macenski, S., Foote, T., Gerkey, B., Lalancette, C., & Woodall, W. (2022). Robot Operating System 2: Design, architecture, and uses in the wild. *Science Robotics*, 7(66), eabm6074.

Object Management Group. (2015). *Data Distribution Service (DDS) Version 1.4*. OMG Document formal/2015-04-10.

Open Robotics. (2024). *ROS 2 Iron Irwini documentation*. https://docs.ros.org/en/iron/

Quigley, M., Gerkey, B., & Smart, W. D. (2015). *Programming robots with ROS: A practical introduction to the Robot Operating System*. O'Reilly Media.

ROS 2 Design Team. (2024). *ROS 2 design documentation*. https://design.ros2.org/

Thomas, D., & Woodall, W. (2023). *ROS 2 performance benchmarks*. Open Robotics Technical Report TR-2023-001.
