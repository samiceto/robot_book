# Chapter 14: OpenVLA Fine-Tuning

## Learning Objectives

1. **Prepare** custom robot datasets for VLA training
2. **Fine-tune** OpenVLA on humanoid-specific tasks
3. **Optimize** training with LoRA for efficiency
4. **Evaluate** model performance on held-out tasks
5. **Deploy** fine-tuned models to real robots

---

## 1. Why Fine-Tune?

**Pre-trained VLA Limitations**:
- Trained on arm manipulators (not humanoids)
- Limited to tabletop tasks
- May not understand your robot's morphology

**Fine-Tuning Benefits**:
- Adapt to humanoid kinematics (12-DOF)
- Learn domain-specific skills (bipedal grasping)
- Improve success rate from 40% → 80%

---

## 2. Dataset Preparation

### Data Collection

**Option 1: Simulation** (Chapter 7 datasets)
- 1000+ demonstrations
- Perfect labels
- Fast iteration

**Option 2: Real Robot** (teleoperation)
- 100-500 demonstrations
- Higher fidelity
- Slower but more realistic

### Data Format (RLDS)

```python
import tensorflow_datasets as tfds

# Convert Chapter 7 data to RLDS
dataset = {
    "steps": [
        {
            "observation": {
                "image": np.array(...),  # 224x224x3
                "state": np.array(...),   # 12-DOF joints
            },
            "action": np.array(...),      # 12-DOF targets
            "language_instruction": "Pick up the cup",
            "is_terminal": False,
        },
        # ... more steps
    ]
}
```

---

## 3. Fine-Tuning with LoRA

**LoRA** (Low-Rank Adaptation): Efficient fine-tuning method
- Freezes base model (7B params)
- Trains small adapter (8M params)
- 100x less memory, 10x faster

### Training Script

```python
from openvla import OpenVLA
from peft import LoraConfig, get_peft_model

# Load base model
model = OpenVLA.from_pretrained("openvla/openvla-7b")

# Configure LoRA
lora_config = LoraConfig(
    r=16,              # Rank
    lora_alpha=32,
    target_modules=["q_proj", "v_proj"],
    lora_dropout=0.1,
)

# Apply LoRA
model = get_peft_model(model, lora_config)

# Train
trainer = VLATrainer(
    model=model,
    train_dataset=train_dataset,
    eval_dataset=eval_dataset,
    batch_size=8,
    learning_rate=1e-4,
    num_epochs=10,
)

trainer.train()
model.save_pretrained("humanoid_vla_lora")
```

---

## 4. Training Configuration

**Hardware Requirements**:
- GPU: RTX 4090 (24GB VRAM) or A100
- RAM: 64GB
- Storage: 500GB for datasets

**Training Time** (1000 demos):
- RTX 4090: 4-6 hours
- A100: 2-3 hours

**Hyperparameters**:
```yaml
batch_size: 8
learning_rate: 1e-4
lora_rank: 16
epochs: 10
gradient_accumulation: 4
mixed_precision: fp16
```

---

## 5. Evaluation Metrics

### Success Rate
```python
success_count = 0
for episode in test_set:
    predicted_action = model.predict(image, instruction)
    success = execute_action(predicted_action)
    if success:
        success_count += 1

success_rate = success_count / len(test_set)
print(f"Success Rate: {success_rate:.1%}")
```

**Target**: >70% success on held-out tasks

### Action Error
```python
mse = np.mean((predicted_actions - ground_truth) ** 2)
print(f"Action MSE: {mse:.4f} radians")
```

---

## 6. Deployment

```python
# Load fine-tuned model
model = OpenVLA.from_pretrained(
    "openvla/openvla-7b",
    peft_model="humanoid_vla_lora"
)

# ROS 2 integration
class VLAControllerNode(Node):
    def __init__(self):
        self.model = model.to("cuda")
        self.image_sub = self.create_subscription(...)
        self.action_pub = self.create_publisher(...)

    def image_callback(self, msg):
        action = self.model.predict_action(
            image=msg,
            instruction=self.current_instruction
        )
        self.action_pub.publish(action)
```

---

## 7. Hands-On Lab: Fine-Tune on Grasping (4 hours)

**Goal**: Fine-tune OpenVLA on Chapter 7 grasping dataset.

**Steps**:
1. Convert synthetic data to RLDS format
2. Configure LoRA training
3. Train for 10 epochs (~2 hours on RTX 4090)
4. Evaluate on 100 held-out images
5. Compare pre-trained vs fine-tuned success rates

**Validation**: Fine-tuned model achieves >70% success

---

## 8. End-of-Chapter Project

Fine-tune VLA for humanoid dish-loading task.

**Requirements**:
- Collect 500 demonstrations (sim or real)
- Fine-tune with LoRA
- Achieve >60% success rate
- Deploy to ROS 2 for real-time control

**Deliverables**:
- Fine-tuned model checkpoint
- Evaluation report with metrics
- ROS 2 deployment package
- Demo video

---

## Summary

Fine-tuning VLAs with LoRA enables efficient adaptation to custom robots and tasks. With 500-1000 demonstrations, you can achieve 70%+ success on humanoid manipulation tasks.

**Next**: Chapter 15 covers multimodal reasoning for complex decision-making.
