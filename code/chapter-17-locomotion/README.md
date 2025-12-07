# Chapter 17: Bipedal Locomotion

## Quick Start

```bash
# Install dependencies
pip install numpy scipy

# Run walking simulation in Isaac Sim
cd ~/.local/share/ov/pkg/isaac-sim-4.2.0
./python.sh ~/ros2_ws/src/robot-book-code/chapter-17-locomotion/walking_controller.py

# Tune gait parameters
python3 tune_gait.py --step-length 0.2 --frequency 1.0
```

## Controllers

- `walking_controller.py` - ZMP-based walking with MPC
- `balance_controller.py` - Balance maintenance
- `trajectory_generator.py` - Foot trajectory generation

## Performance

- Step length: 0.1-0.3m
- Walking speed: 0.2-0.5 m/s
- Stability: 10+ consecutive steps
