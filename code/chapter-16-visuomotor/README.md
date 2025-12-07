# Chapter 16: End-to-End Visuomotor Control

## Quick Start

```bash
# Launch ROS 2 visuomotor controller
ros2 run chapter_16 visuomotor_controller \
  --model humanoid_vla_lora \
  --instruction "Pick up the cup"

# Benchmark latency
python3 benchmark_latency.py --model humanoid_vla_lora

# Real robot deployment
ros2 launch chapter_16 deploy_real.launch.py
```

## Performance Targets

- Inference latency: <100 ms
- Control frequency: ≥10 Hz
- Success rate: >60%

## Optimization

- FP32 → INT8: 2x speedup
- 7B → 1B distillation: 3x speedup
