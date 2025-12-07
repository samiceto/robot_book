# Chapter 13: VLA Architecture

## Quick Start

```bash
# Install OpenVLA
git clone https://github.com/openvla/openvla.git
pip install -e openvla/

# Download model
huggingface-cli download openvla/openvla-7b --local-dir models/

# Run inference
python3 vla_inference.py --model models/openvla-7b --image test.jpg --instruction "Pick up the cup"
```

## Scripts

- `vla_inference.py` - Run VLA inference on images
- `visualize_actions.py` - Visualize predicted actions in RViz2

## Requirements

- GPU: RTX 4070+ (12GB VRAM)
- Python 3.10+
- PyTorch 2.0+
