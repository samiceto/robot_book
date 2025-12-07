# Chapter 13: VLA Architecture Fundamentals

## Learning Objectives

1. **Understand** Vision-Language-Action (VLA) model architecture
2. **Compare** VLA vs traditional RL approaches
3. **Identify** key components: vision encoder, language model, action decoder
4. **Install** OpenVLA framework
5. **Run** pre-trained VLA models for inference

---

## 1. What are VLAs?

**Vision-Language-Action Models** combine:
- **Vision**: Process camera images (RGB/RGBD)
- **Language**: Understand natural language instructions
- **Action**: Output robot control commands

**Example Task**:
- Input: Image + "Pick up the red cup"
- Output: 7-DOF arm joint positions

**Key Insight**: VLAs learn generalizable policies from large-scale robot data, unlike traditional RL which requires task-specific training.

---

## 2. VLA vs Traditional Approaches

| Approach | Training Data | Generalization | Example |
|----------|---------------|----------------|---------|
| **Traditional RL** | Task-specific (1K demos) | Limited | Pick red cup only |
| **VLA** | Multi-task (1M demos) | Broad | Pick any object by name |

**VLA Advantages**:
- Zero-shot transfer to new objects
- Language-conditioned behavior
- Scales with data (like LLMs)

---

## 3. Architecture Components

### Vision Encoder (CLIP/DINOv2)
```
Input: 224×224 RGB image
↓
Pretrained Vision Transformer (ViT)
↓
Output: 768-dim embedding
```

### Language Model (LLaMA/GPT)
```
Input: "Pick up the cup"
↓
Transformer decoder
↓
Output: Text embeddings aligned with vision
```

### Action Decoder
```
Input: Vision + language embeddings
↓
MLP layers
↓
Output: Robot actions (joint positions or velocities)
```

**End-to-End Training**: All components fine-tuned jointly on robot data.

---

## 4. OpenVLA Installation

```bash
# Clone OpenVLA
git clone https://github.com/openvla/openvla.git
cd openvla

# Install dependencies
pip install -e .
pip install torch torchvision transformers

# Download pre-trained model (7B params)
huggingface-cli download openvla/openvla-7b --local-dir models/openvla-7b
```

---

## 5. Inference Example

```python
from openvla import OpenVLA
import torch
from PIL import Image

# Load model
model = OpenVLA.from_pretrained("openvla/openvla-7b")
model = model.to("cuda")

# Load image
image = Image.open("robot_view.jpg")

# Language instruction
instruction = "Pick up the red mug"

# Get action
with torch.no_grad():
    action = model.predict_action(
        image=image,
        instruction=instruction
    )

print(f"Action: {action}")  # 7-DOF joint positions
```

---

## 6. Training Data Format

VLAs require large-scale datasets:

**Open X-Embodiment Dataset** (1M+ trajectories):
- 22 robot platforms
- 527 skills
- 160,000 tasks

**Data Format** (HDF5):
```python
{
    "observations": {
        "image": [224, 224, 3],
        "state": [7],  # Joint positions
    },
    "actions": [7],  # Next joint positions
    "language": "Pick up the cup"
}
```

---

## 7. Hands-On Lab: VLA Inference (2 hours)

**Goal**: Run OpenVLA on sample images and instructions.

**Steps**:
1. Install OpenVLA framework
2. Download pre-trained 7B model
3. Run inference on test images
4. Visualize predicted actions in RViz2

**Validation**: Model outputs 7-DOF actions for manipulation tasks

---

## 8. End-of-Chapter Project

Run VLA on humanoid grasping dataset from Chapter 7.

**Requirements**:
- Load OpenVLA model
- Process 100 test images
- Compare predicted vs ground-truth actions
- Calculate success rate (±10° joint tolerance)

---

## Summary

VLAs enable language-conditioned robot control through vision-language-action alignment. Unlike traditional RL, VLAs generalize across tasks by training on massive multi-robot datasets.

**Next**: Chapter 14 covers fine-tuning OpenVLA on custom robot data.
