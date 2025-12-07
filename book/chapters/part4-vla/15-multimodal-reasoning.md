# Chapter 15: Multimodal Reasoning

## Learning Objectives

1. **Integrate** VLMs (Vision-Language Models) for scene understanding
2. **Implement** chain-of-thought reasoning for complex tasks
3. **Combine** VLM planning with VLA execution
4. **Handle** failure recovery with language feedback
5. **Build** hierarchical task decomposition systems

---

## 1. VLM vs VLA

| Model Type | Purpose | Example |
|------------|---------|---------|
| **VLM** (GPT-4V, LLaVA) | Scene understanding, planning | "The cup is on the left side of the table" |
| **VLA** (OpenVLA) | Action execution | Output: [0.2, 0.5, ...] joint positions |

**Key Insight**: Use VLM for "what" and "why", VLA for "how"

---

## 2. Multimodal Pipeline

```
Camera Image + "Clean the table"
        ↓
    VLM (GPT-4V)
        ↓
Task Decomposition:
1. "Locate dishes on table"
2. "Pick up each dish"
3. "Place in dishwasher"
        ↓
    VLA (OpenVLA)
        ↓
Execute each subtask
```

---

## 3. GPT-4V Integration

```python
import openai
from PIL import Image
import base64

# Encode image
with open("scene.jpg", "rb") as f:
    image_b64 = base64.b64encode(f.read()).decode()

# Query GPT-4V
response = openai.ChatCompletion.create(
    model="gpt-4-vision-preview",
    messages=[
        {
            "role": "user",
            "content": [
                {"type": "text", "text": "Describe objects on the table and suggest grasping order."},
                {"type": "image_url", "image_url": f"data:image/jpeg;base64,{image_b64}"}
            ]
        }
    ]
)

plan = response.choices[0].message.content
print(plan)
# "I see 3 objects: red cup (left), blue plate (center), green bottle (right).
#  Suggested order: cup → plate → bottle (left to right)"
```

---

## 4. Chain-of-Thought Reasoning

**Prompt Engineering** for step-by-step reasoning:

```python
prompt = """
You are a robot planning assistant. Given the image:

1. List all objects visible
2. Identify graspable objects
3. Determine optimal grasp order
4. Provide action sequence

Think step-by-step before answering.
"""

response = query_gpt4v(image, prompt)
```

**Example Output**:
```
Step 1: Objects visible
- Red ceramic mug (left side, handle facing right)
- White plate with residue (center)
- Green water bottle (right side, upright)

Step 2: Graspable analysis
- Mug: Yes (stable handle grasp)
- Plate: Yes (edge grasp, but fragile)
- Bottle: Yes (cylinder grasp)

Step 3: Optimal order
1. Mug (easiest, stable)
2. Bottle (medium difficulty)
3. Plate (hardest, requires delicate grasp)

Step 4: Action sequence
FOR mug: "Grasp red mug by handle"
FOR bottle: "Grasp green bottle at center"
FOR plate: "Grasp white plate at edge with two hands"
```

---

## 5. VLM → VLA Pipeline

```python
class MultimodalController:
    def __init__(self):
        self.vlm = GPT4V()        # Planning
        self.vla = OpenVLA()      # Execution

    def execute_task(self, image, task):
        # 1. VLM: Generate plan
        plan = self.vlm.plan(image, task)
        steps = self.parse_plan(plan)

        # 2. VLA: Execute each step
        for step in steps:
            action = self.vla.predict_action(
                image=image,
                instruction=step
            )
            success = self.robot.execute(action)

            # 3. Failure recovery
            if not success:
                new_plan = self.vlm.replan(image, f"Failed: {step}. Suggest alternative.")
                # ... retry

        return success

# Usage
controller = MultimodalController()
success = controller.execute_task(
    image=camera_image,
    task="Clear the table"
)
```

---

## 6. Failure Recovery

**Feedback Loop**:
```python
def execute_with_recovery(image, instruction, max_retries=3):
    for attempt in range(max_retries):
        # Execute
        action = vla.predict_action(image, instruction)
        success = robot.execute(action)

        if success:
            return True

        # Get new image after failure
        new_image = robot.get_camera_image()

        # Ask VLM for recovery strategy
        recovery_plan = vlm.query(
            new_image,
            f"Execution failed for '{instruction}'. What went wrong? Suggest fix."
        )

        # Update instruction based on feedback
        instruction = recovery_plan

    return False
```

---

## 7. Hierarchical Task Decomposition

```python
# High-level task
task = "Prepare breakfast"

# VLM decomposes into subtasks
subtasks = vlm.decompose(task)
# ["Locate cereal box", "Grasp cereal box", "Pour cereal into bowl",
#  "Locate milk", "Grasp milk", "Pour milk into bowl"]

# Execute each subtask with VLA
for subtask in subtasks:
    image = robot.get_camera_image()
    action = vla.predict_action(image, subtask)
    robot.execute(action)
```

---

## 8. Hands-On Lab: VLM-VLA Integration (3 hours)

**Goal**: Build multimodal system for tabletop clearing.

**Steps**:
1. Setup GPT-4V API access
2. Implement VLM → VLA pipeline
3. Test on 10 cluttered table scenes
4. Measure success rate with/without VLM planning

**Validation**: VLM planning improves success by 20%+

---

## 9. End-of-Chapter Project

Build autonomous kitchen assistant with multimodal reasoning.

**Requirements**:
- VLM: Identify dishes, utensils, food items
- VLA: Execute grasping and placement
- Hierarchical decomposition for multi-step tasks
- Failure recovery with replanning
- Success rate >65% on 20 test scenarios

**Deliverables**:
- Multimodal pipeline code
- Evaluation report
- Demo video (breakfast preparation or table clearing)

---

## Summary

Multimodal reasoning combines VLM planning (semantic understanding) with VLA execution (precise control). This enables complex, long-horizon tasks beyond single-step manipulation.

**Next**: Chapter 16 covers end-to-end visuomotor control and real-world deployment.
