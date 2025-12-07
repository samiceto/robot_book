# Chapter 9: Intel RealSense Integration

## Learning Objectives

1. **Install** Intel RealSense SDK and ROS 2 wrapper
2. **Capture** RGBD data (color + depth) at 30 FPS
3. **Generate** point clouds for 3D perception
4. **Calibrate** camera intrinsics and extrinsics
5. **Integrate** with ROS 2 for real-time processing

---

## 1. RealSense Overview

**Intel RealSense D435/D455** (Recommended for humanoids):
- **Depth Range**: 0.3m - 10m
- **Resolution**: 1280×720 @ 30 FPS
- **Technology**: Stereo vision (not ToF)
- **FOV**: 87° × 58° (wide angle)
- **Interface**: USB 3.0
- **Power**: USB-powered (no external adapter)

**Why RealSense for Humanoids**:
- Lightweight (72g)
- Wide FOV for navigation
- Accurate depth (±2% at 2m)
- ROS 2 native support

---

## 2. Installation

### Install RealSense SDK

```bash
# Add Intel repository
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list

# Install SDK
sudo apt update
sudo apt install librealsense2-dkms librealsense2-utils librealsense2-dev

# Verify
realsense-viewer  # GUI should open showing camera feed
```

### Install ROS 2 Wrapper

```bash
sudo apt install ros-${ROS_DISTRO}-realsense2-camera
```

---

## 3. Basic Usage

### Capture RGBD Data

**Terminal 1**: Launch RealSense node
```bash
ros2 launch realsense2_camera rs_launch.py depth_module.profile:=640x480x30 rgb_camera.profile:=640x480x30
```

**Terminal 2**: View topics
```bash
# List topics
ros2 topic list
# /camera/color/image_raw      # RGB image
# /camera/depth/image_rect_raw # Depth image
# /camera/depth/color/points   # Point cloud

# View depth image
ros2 run rqt_image_view rqt_image_view /camera/depth/image_rect_raw
```

### Generate Point Cloud

```python
import rclpy
from sensor_msgs.msg import PointCloud2

def pointcloud_callback(msg: PointCloud2):
    """Process point cloud data."""
    print(f"Point cloud: {msg.width}x{msg.height} points")
    # Process with open3d or pcl

rclpy.init()
node = rclpy.create_node('pointcloud_processor')
node.create_subscription(PointCloud2, '/camera/depth/color/points', pointcloud_callback, 10)
rclpy.spin(node)
```

---

## 4. Hands-On Lab: Object Distance Measurement (2 hours)

**Goal**: Measure distance to objects in front of humanoid.

**Steps**:
1. Launch RealSense node
2. Subscribe to `/camera/depth/image_rect_raw`
3. Find center pixel depth value
4. Display distance in meters

**Code**: `distance_measurement.py` (companion repository)

```python
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class DistanceMeasurement(Node):
    def __init__(self):
        super().__init__('distance_measurement')
        self.subscription = self.create_subscription(
            Image, '/camera/depth/image_rect_raw', self.depth_callback, 10)
        self.bridge = CvBridge()

    def depth_callback(self, msg):
        # Convert to numpy array
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # Get center pixel depth (in mm)
        h, w = depth_image.shape
        center_depth_mm = depth_image[h//2, w//2]
        center_depth_m = center_depth_mm / 1000.0

        self.get_logger().info(f'Distance to center object: {center_depth_m:.2f} m')
```

**Validation**: Place object at 1.0m → should read 0.95-1.05m

---

## 5. End-of-Chapter Project: Obstacle Detection

Detect obstacles within 1m in front of humanoid for navigation safety.

**Requirements**:
- Subscribe to depth image
- Identify pixels < 1.0m depth
- Publish boolean: obstacle detected (yes/no)
- Visualize detection zone in RViz2

**Deliverable**: ROS 2 package with obstacle detection node

---

## Summary

You learned to:
- Install RealSense SDK and ROS 2 wrapper
- Capture RGBD data at 30 FPS
- Process depth images for distance measurement
- Generate point clouds for 3D perception

**Next**: Chapter 10 covers YOLO object detection for semantic understanding.
