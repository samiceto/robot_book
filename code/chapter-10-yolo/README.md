# Chapter 10: Object Detection with YOLO

## Quick Start

```bash
# Install YOLOv8
pip install ultralytics

# Run YOLO detector
ros2 run chapter_10 yolo_detector

# Train custom model
python3 train_custom.py --data config.yaml --epochs 50
```

## Scripts

- `yolo_detector.py` - ROS 2 YOLO detection node
- `train_custom.py` - Train YOLOv8 on custom dataset
- `rgbd_fusion.py` - Combine YOLO + RealSense depth for 3D localization

## Performance

- PyTorch: 30 FPS (RTX 4070)
- TensorRT INT8: 50 FPS (Jetson Orin)
