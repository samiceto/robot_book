# Chapter 16: End-to-End Visuomotor Control

## Learning Objectives

1. **Deploy** VLA models on real humanoid robots
2. **Implement** closed-loop visuomotor control at 10+ Hz
3. **Handle** sim-to-real transfer challenges
4. **Optimize** inference latency for real-time control
5. **Evaluate** performance on physical manipulation tasks

---

## 1. What is Visuomotor Control?

**Definition**: Direct mapping from visual observations to motor commands without explicit perception or planning stages.

```
Raw Pixels → Neural Network → Robot Actions
```

**Benefits**:
- End-to-end learning
- No manual feature engineering
- Handles uncertainty implicitly

---

## 2. Real-Time Requirements

**Control Loop**:
```
1. Capture image (30 ms)
2. VLA inference (50-100 ms)
3. Execute action (30 ms)
Total: ~100-160 ms → 6-10 Hz
```

**Target**: ≥10 Hz for stable manipulation

---

## 3. Inference Optimization

### Quantization (FP32 → INT8)

```python
from openvla import OpenVLA
import torch

# Load model
model = OpenVLA.from_pretrained("openvla/openvla-7b")

# Quantize to INT8
model_int8 = torch.quantization.quantize_dynamic(
    model,
    {torch.nn.Linear},
    dtype=torch.qint8
)

# Benchmark
import time
start = time.time()
action = model_int8.predict_action(image, instruction)
latency = time.time() - start
print(f"Latency: {latency*1000:.1f} ms")  # 50-80 ms on RTX 4090
```

### Model Distillation (7B → 1B)

```python
# Train smaller model to mimic large model
teacher = OpenVLA.from_pretrained("openvla/openvla-7b")
student = OpenVLA.from_pretrained("openvla/openvla-1b")

# Distillation loss
def distillation_loss(student_logits, teacher_logits, temperature=2.0):
    soft_targets = F.softmax(teacher_logits / temperature, dim=-1)
    soft_student = F.log_softmax(student_logits / temperature, dim=-1)
    return F.kl_div(soft_student, soft_targets, reduction='batchmean')

# Train student to match teacher
# Result: 3x faster inference, ~5% accuracy drop
```

---

## 4. Sim-to-Real Transfer

**Domain Gap**:
| Aspect | Simulation | Reality |
|--------|------------|---------|
| Lighting | Perfect | Variable |
| Physics | Exact | Noisy |
| Sensing | Ideal | Delays, noise |

**Mitigation Strategies**:

### 1. Domain Randomization (Chapter 7)
```python
# Randomize in sim during training
randomize_lighting(intensity=(500, 2000))
randomize_textures()
randomize_physics(friction=(0.3, 0.9))
```

### 2. Real-World Fine-Tuning
```python
# Collect 50-100 real demos
real_data = collect_teleoperation_data(num_episodes=100)

# Fine-tune sim-trained model
model.fine_tune(
    real_data,
    learning_rate=1e-5,
    epochs=5
)
# Improves success: 45% → 75%
```

---

## 5. ROS 2 Integration

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge
from openvla import OpenVLA

class VisuomotorController(Node):
    def __init__(self):
        super().__init__('visuomotor_controller')

        # Load VLA model
        self.model = OpenVLA.from_pretrained("humanoid_vla_lora")
        self.model = self.model.to("cuda")

        # Current instruction
        self.instruction = "Pick up the cup"

        # Subscribers
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10)

        # Publishers
        self.joint_pub = self.create_publisher(
            JointState, '/joint_commands', 10)

        self.bridge = CvBridge()
        self.get_logger().info('Visuomotor controller ready')

    def image_callback(self, msg):
        # Convert to numpy
        image = self.bridge.imgmsg_to_cv2(msg, 'rgb8')

        # VLA inference
        action = self.model.predict_action(
            image=image,
            instruction=self.instruction
        )

        # Publish joint commands
        joint_msg = JointState()
        joint_msg.position = action.tolist()
        self.joint_pub.publish(joint_msg)

def main():
    rclpy.init()
    node = VisuomotorController()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

---

## 6. Safety and Monitoring

### Collision Detection
```python
def safe_execute(action, current_joints):
    # Check joint limits
    if not within_limits(action):
        return None

    # Predict next state
    next_state = forward_kinematics(action)

    # Check self-collision
    if self_collision(next_state):
        return None

    # Check workspace bounds
    if out_of_bounds(next_state):
        return None

    return action
```

### Emergency Stop
```python
class SafetyMonitor:
    def __init__(self):
        self.force_threshold = 50.0  # Newtons
        self.velocity_threshold = 1.0  # m/s

    def check_safety(self, force, velocity):
        if force > self.force_threshold:
            robot.emergency_stop()
            return False

        if velocity > self.velocity_threshold:
            robot.slow_down(factor=0.5)

        return True
```

---

## 7. Hands-On Lab: Real Robot Deployment (4 hours)

**Goal**: Deploy VLA to physical humanoid for grasping.

**Steps**:
1. Setup RealSense + humanoid arm
2. Load fine-tuned VLA model
3. Implement visuomotor control node
4. Test on 20 grasping scenarios
5. Measure success rate and latency

**Validation**:
- Inference: <100 ms
- Control frequency: >10 Hz
- Success rate: >60%

---

## 8. Evaluation Metrics

### Success Rate
```python
successes = 0
for trial in range(100):
    success = execute_task(object, instruction)
    if success:
        successes += 1

success_rate = successes / 100
print(f"Success Rate: {success_rate:.1%}")
```

### Execution Time
```python
times = []
for trial in range(100):
    start = time.time()
    execute_task(object, instruction)
    times.append(time.time() - start)

print(f"Mean: {np.mean(times):.2f}s")
print(f"Std: {np.std(times):.2f}s")
```

---

## 9. End-of-Chapter Project

Deploy complete visuomotor control system on humanoid.

**Requirements**:
- VLA model (fine-tuned from Chapter 14)
- ROS 2 integration with RealSense
- Real-time control at ≥10 Hz
- Safety monitoring (collision, limits)
- Test on 50 real-world manipulation tasks
- Success rate >55%

**Deliverables**:
- Deployment code (ROS 2 package)
- Performance report (success rate, latency)
- Safety analysis
- 5-minute demo video

---

## Summary

End-to-end visuomotor control enables direct pixel-to-action policies for robot manipulation. With proper optimization (quantization, distillation) and sim-to-real transfer (domain randomization, real fine-tuning), VLAs achieve 60-80% success on real humanoid tasks.

---

## Part 4 Complete!

You've learned:
- VLA architecture and components
- Fine-tuning with LoRA
- Multimodal reasoning (VLM + VLA)
- Real-time visuomotor control

**Overall Progress**: 16 of 21 chapters (76%)

**Next**: Part 5 (Advanced Topics) covers locomotion, whole-body control, human-robot interaction, and deployment at scale.
