# Chapter 11: Jetson Orin Deployment

## Quick Start

```bash
# Flash JetPack 6.0 (on host PC)
sdkmanager

# On Jetson: Install ROS 2
sudo apt install ros-iron-desktop

# Quantize model to INT8
python3 quantize_yolo.py --model yolov8n.pt --output yolov8n.engine

# Deploy perception
ros2 run chapter_11 perception_node
```

## Scripts

- `quantize_yolo.py` - Export YOLOv8 to TensorRT INT8
- `perception_node.py` - Real-time perception pipeline
- `profile.sh` - Profile with Nsight Systems

## Performance (Jetson Orin Nano)

| Model | FPS | Power |
|-------|-----|-------|
| FP32 | 12 | 18W |
| FP16 | 25 | 16W |
| INT8 | 50 | 15W |
