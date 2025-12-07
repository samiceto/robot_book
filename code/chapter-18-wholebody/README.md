# Chapter 18: Whole-Body Control

## Quick Start

```bash
# Run whole-body IK solver
python3 wholebody_ik.py --targets config/targets.yaml

# Walk-and-reach demo
ros2 run chapter_18 walk_and_reach --target-object cup
```

## Features

- Hierarchical task prioritization
- Null-space projection
- Real-time IK solving
- ROS 2 integration

## Performance

- IK solve time: <10ms
- Success rate: >80% for mobile manipulation
