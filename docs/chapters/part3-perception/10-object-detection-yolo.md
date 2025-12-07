# Chapter 10: Object Detection with YOLO

## Learning Objectives

1. **Install** YOLOv8 and integrate with ROS 2
2. **Detect** objects in real-time (30 FPS on GPU)
3. **Train** custom models for humanoid-specific objects
4. **Optimize** for edge deployment (TensorRT)
5. **Combine** with RealSense depth for 3D object localization

---

## 1. YOLO Overview

**YOLOv8** (Ultralytics, 2023):
- **Speed**: 30-60 FPS on RTX 4070
- **Accuracy**: 50+ mAP on COCO dataset
- **Classes**: 80 pre-trained (person, cup, bottle, etc.)
- **Variants**: YOLOv8n (nano), YOLOv8s (small), YOLOv8m (medium)

**Why YOLO for Humanoids**:
- Real-time performance
- Single-stage detector (fast)
- Easy fine-tuning for custom objects
- TensorRT support for Jetson

---

## 2. Installation

```bash
# Install Ultralytics
pip install ultralytics

# Verify
yolo predict model=yolov8n.pt source='https://ultralytics.com/images/bus.jpg'
# Should download model and detect objects in bus.jpg
```

---

## 3. Basic Object Detection

### Python API

```python
from ultralytics import YOLO

# Load pre-trained model
model = YOLO('yolov8n.pt')  # Nano (fastest)

# Run inference
results = model.predict(source='image.jpg', conf=0.5)

# Process results
for result in results:
    boxes = result.boxes
    for box in boxes:
        x1, y1, x2, y2 = box.xyxy[0]  # Bounding box
        conf = box.conf[0]              # Confidence
        cls = box.cls[0]                # Class ID
        print(f"Detected {model.names[int(cls)]} at ({x1}, {y1}) conf={conf:.2f}")
```

### ROS 2 Integration

```python
import rclpy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D
from cv_bridge import CvBridge
from ultralytics import YOLO

class YOLODetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.model = YOLO('yolov8n.pt')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10)
        self.publisher = self.create_publisher(Detection2DArray, '/detections', 10)

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Run YOLO
        results = self.model.predict(cv_image, verbose=False)

        # Publish detections
        detections_msg = Detection2DArray()
        for result in results:
            for box in result.boxes:
                det = Detection2D()
                det.bbox.center.x = float((box.xyxy[0][0] + box.xyxy[0][2]) / 2)
                det.bbox.center.y = float((box.xyxy[0][1] + box.xyxy[0][3]) / 2)
                det.bbox.size_x = float(box.xyxy[0][2] - box.xyxy[0][0])
                det.bbox.size_y = float(box.xyxy[0][3] - box.xyxy[0][1])
                detections_msg.detections.append(det)

        self.publisher.publish(detections_msg)
```

---

## 4. Training Custom Models

### Prepare Dataset (COCO Format)

Use Chapter 7 synthetic data generator:
```bash
# Generate 1000 images
./python.sh generate_grasping_dataset.py --num-images 1000

# Convert to COCO format
python3 convert_to_coco.py --input-dir datasets/grasping_dataset \
  --output-file annotations.json
```

### Train YOLOv8

```python
from ultralytics import YOLO

# Load base model
model = YOLO('yolov8n.pt')

# Train on custom data
model.train(
    data='config.yaml',  # Dataset config
    epochs=50,
    imgsz=640,
    batch=16,
    device=0  # GPU 0
)

# Validate
metrics = model.val()
print(f"mAP50: {metrics.box.map50}")

# Export to TensorRT (for Jetson)
model.export(format='engine', device=0)
```

**config.yaml**:
```yaml
path: /path/to/dataset
train: images/train
val: images/val

names:
  0: cube
  1: cylinder
  2: sphere
```

---

## 5. TensorRT Optimization

**For Jetson Orin**:
```bash
# Export to TensorRT engine
yolo export model=yolov8n.pt format=engine device=0

# Inference with TensorRT
model = YOLO('yolov8n.engine')
results = model.predict('image.jpg')
# 2-3x faster than PyTorch on Jetson
```

**Performance** (Jetson Orin Nano):
- PyTorch FP32: 12 FPS
- TensorRT FP16: 35 FPS
- TensorRT INT8: 50 FPS (with calibration)

---

## 6. 3D Object Localization (RGBD Fusion)

Combine YOLO (2D) with RealSense depth (3D):

```python
def get_3d_position(bbox, depth_image, camera_intrinsics):
    """
    Convert 2D bounding box + depth to 3D position.

    Args:
        bbox: (x1, y1, x2, y2) in pixels
        depth_image: Depth map (H, W) in mm
        camera_intrinsics: (fx, fy, cx, cy)

    Returns:
        (x, y, z) in meters (camera frame)
    """
    # Get bbox center
    cx_bbox = int((bbox[0] + bbox[2]) / 2)
    cy_bbox = int((bbox[1] + bbox[3]) / 2)

    # Get depth at center
    depth_mm = depth_image[cy_bbox, cx_bbox]
    z = depth_mm / 1000.0  # Convert to meters

    # Unproject to 3D
    fx, fy, cx, cy = camera_intrinsics
    x = (cx_bbox - cx) * z / fx
    y = (cy_bbox - cy) * z / fy

    return (x, y, z)

# Example usage
bbox = [100, 150, 200, 250]  # Detected cup
position_3d = get_3d_position(bbox, depth_image, (615, 615, 320, 240))
print(f"Cup position: {position_3d} meters from camera")
```

---

## 7. Hands-On Lab: Tabletop Object Detection (3 hours)

**Goal**: Detect and localize objects on table for grasping.

**Steps**:
1. Launch RealSense + YOLO detector
2. Detect objects (cups, bottles)
3. Get 3D positions using depth
4. Publish to `/objects_3d` topic
5. Visualize in RViz2

**Validation**:
- Detects ≥3 objects on table
- 3D positions accurate to ±5cm
- Runs at ≥20 FPS

---

## 8. End-of-Chapter Project: Pick-and-Place Vision

Build vision system for humanoid pick-and-place task.

**Requirements**:
- Detect graspable objects (cups, bottles, bowls)
- Filter objects within reach (0.3-0.8m)
- Rank by grasp confidence (size, orientation)
- Publish top candidate to `/grasp_target`

**Deliverables**:
- ROS 2 package with YOLO + depth fusion
- Launch file
- 2-minute demo video

---

## Summary

You learned to:
- Install YOLOv8 and integrate with ROS 2
- Detect objects in real-time at 30+ FPS
- Train custom models on synthetic data
- Optimize with TensorRT for Jetson
- Fuse 2D detections with 3D depth data

**Next**: Chapter 11 covers deploying to Jetson Orin with model quantization.
