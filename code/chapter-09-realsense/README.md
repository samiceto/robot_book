# Chapter 9: Intel RealSense Integration

## Quick Start

```bash
# Install RealSense
sudo apt install ros-${ROS_DISTRO}-realsense2-camera

# Launch camera
ros2 launch realsense2_camera rs_launch.py

# Run distance measurement
ros2 run chapter_09 distance_measurement
```

## Scripts

- `distance_measurement.py` - Measure distance to center object
- `obstacle_detection.py` - Detect obstacles within 1m

## Validation

Place object at 1.0m → should read 0.95-1.05m
