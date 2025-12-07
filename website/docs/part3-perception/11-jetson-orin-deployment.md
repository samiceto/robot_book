# Chapter 11: Jetson Orin Deployment

## Learning Objectives

1. **Install** JetPack 6.0 and ROS 2 on Jetson Orin
2. **Quantize** models to INT8 for 3-5x speedup
3. **Profile** GPU/DLA utilization with Nsight Systems
4. **Optimize** power modes for battery life vs performance
5. **Deploy** real-time perception pipeline (RealSense + YOLO)

---

## 1. Jetson Orin Overview

**NVIDIA Jetson Orin Nano** (Recommended for humanoids):
- **GPU**: 1024 CUDA cores, 32 Tensor cores
- **CPU**: 6-core Arm Cortex-A78
- **RAM**: 8GB LPDDR5
- **Power**: 7W - 25W (configurable)
- **Price**: ~$499
- **Performance**: 40 TOPS INT8

**Why Jetson for Humanoids**:
- Low power (runs on battery)
- Compact (70mm × 45mm)
- Real-time AI inference
- ROS 2 compatible

**Alternatives**:
- **Jetson Orin NX** ($599, 16GB RAM, 100 TOPS) - more headroom
- **Raspberry Pi 5** (CPU-only, $80) - budget option, no GPU acceleration

---

## 2. Installation

### Flash JetPack 6.0

```bash
# On host PC (Ubuntu 22.04)
# Download SDK Manager: https://developer.nvidia.com/sdk-manager

# Install SDK Manager
sudo dpkg -i sdkmanager_*.deb

# Launch and follow GUI to flash Jetson
sdkmanager
```

**JetPack 6.0 includes**:
- Ubuntu 22.04
- CUDA 12.2
- cuDNN 8.9
- TensorRT 8.6

### Install ROS 2

```bash
# On Jetson (after first boot)
sudo apt update
sudo apt install ros-iron-desktop python3-colcon-common-extensions

# Verify
ros2 topic list
```

### Install Perception Stack

```bash
# RealSense
sudo apt install ros-iron-realsense2-camera

# YOLOv8 (with TensorRT support)
pip3 install ultralytics onnx onnxruntime-gpu
```

---

## 3. Model Quantization (FP32 → INT8)

### Why Quantization?

| Precision | Model Size | Inference Speed | Accuracy Loss |
|-----------|------------|-----------------|---------------|
| FP32      | 100%       | 1x              | 0%            |
| FP16      | 50%        | 2x              | <1%           |
| INT8      | 25%        | 3-5x            | 1-3%          |

### Quantize YOLOv8

```python
from ultralytics import YOLO

# Load model
model = YOLO('yolov8n.pt')

# Export to TensorRT INT8 (requires calibration data)
model.export(
    format='engine',
    imgsz=640,
    half=False,       # Set to True for FP16
    int8=True,        # INT8 quantization
    data='config.yaml',  # Calibration dataset
    batch=1,
    device=0
)

# Result: yolov8n.engine (INT8 optimized)
```

**Performance Gain** (Jetson Orin Nano):
- FP32: 12 FPS
- FP16: 25 FPS (2x speedup)
- INT8: 50 FPS (4x speedup)

---

## 4. Power Modes

Jetson supports multiple power modes trading performance for battery life:

```bash
# List available modes
sudo /usr/sbin/nvpmodel -q

# Modes (Jetson Orin Nano):
# Mode 0: 25W (max performance)
# Mode 1: 15W (balanced)
# Mode 2: 7W (power saver)

# Set to 15W mode
sudo /usr/sbin/nvpmodel -m 1

# Monitor power consumption
sudo tegrastats
# RAM 2048/8192MB CPU [15%@1420,10%@1420,8%@1420] GPU 45%@625MHz ...
```

**Battery Life Estimates** (5000mAh battery):
- 25W mode: ~45 minutes
- 15W mode: ~1.5 hours
- 7W mode: ~3 hours

---

## 5. Profiling with Nsight Systems

```bash
# Profile perception pipeline
nsys profile -o perception_profile \
  ros2 run my_package perception_node

# View results (on host PC with GUI)
nsys-ui perception_profile.qdrep
```

**Key Metrics**:
- **GPU Utilization**: Target >70% (indicates GPU-bound, good)
- **CUDA Kernel Duration**: Identify slow layers
- **Memory Transfers**: Minimize CPU↔GPU copies

**Optimization Tips**:
- Use `pinned_memory=True` for faster CPU→GPU transfer
- Batch inference (process 2-4 frames at once)
- Async execution (overlap CPU/GPU work)

---

## 6. Real-Time Perception Pipeline

Deploy RealSense + YOLO on Jetson:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
from cv_bridge import CvBridge
from ultralytics import YOLO

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        # Load TensorRT INT8 model
        self.model = YOLO('yolov8n.engine')  # Optimized for Jetson
        self.bridge = CvBridge()

        # Subscribe to RealSense
        self.subscription = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10)

        # Publish detections
        self.publisher = self.create_publisher(Detection2DArray, '/detections', 10)

        self.get_logger().info('Perception node started on Jetson Orin')

    def image_callback(self, msg):
        # Convert to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Run YOLO (INT8 TensorRT)
        results = self.model.predict(cv_image, verbose=False)

        # Publish detections
        detections_msg = Detection2DArray()
        # ... (populate detections) ...
        self.publisher.publish(detections_msg)

def main():
    rclpy.init()
    node = PerceptionNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

**Launch**:
```bash
# Set to 15W mode (balanced)
sudo nvpmodel -m 1

# Run perception
ros2 run my_package perception_node

# Expected: 30-40 FPS at 15W
```

---

## 7. Hands-On Lab: Jetson Deployment (4 hours)

**Goal**: Deploy complete perception pipeline to Jetson Orin.

**Steps**:
1. Flash JetPack 6.0 to Jetson
2. Install ROS 2 + RealSense + YOLOv8
3. Quantize YOLOv8n to INT8
4. Deploy perception node
5. Profile with Nsight Systems
6. Test at 15W power mode

**Validation**:
- [ ] Runs at ≥30 FPS
- [ ] GPU utilization ≥60%
- [ ] Power consumption ≤15W
- [ ] Detects objects with ≥45 mAP

---

## 8. End-of-Chapter Project: Autonomous Navigation Vision

Build vision system for humanoid autonomous navigation.

**Requirements**:
- Detect obstacles (people, furniture) using YOLO
- Estimate distance using RealSense depth
- Publish obstacle map to `/obstacle_map`
- Run on Jetson at ≥20 FPS, ≤15W

**Deliverables**:
- ROS 2 package deployable to Jetson
- TensorRT INT8 model
- Power profiling report
- Demo video (robot navigating around obstacles)

---

## Summary

You learned to:
- Install JetPack and ROS 2 on Jetson Orin
- Quantize models to INT8 for 3-5x speedup
- Profile GPU utilization with Nsight
- Optimize power modes for battery operation
- Deploy real-time perception at 30+ FPS on edge

**Next**: Chapter 12 covers containerization with Docker and CI/CD pipelines.
