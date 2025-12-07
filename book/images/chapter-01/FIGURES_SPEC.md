# Chapter 1 Figures Specification

This document describes the figures that need to be created for Chapter 1: Introduction to Physical AI and Humanoid Robotics.

## Figure 1-1: Physical AI Technology Stack and Virtuous Cycle

**Location in Text**: Section 1.3 (Humanoids as the Ultimate Embodied AI Challenge)
**File Name**: `fig-1-1-physical-ai-stack.webp` (or .png, .svg)
**Type**: System architecture diagram with feedback loop
**Size**: 800×600 pixels (or vector format)

**Description**:
A diagram showing the three layers of the Physical AI stack with a virtuous cycle:

**Layer 1 - Perception** (Bottom):
- Icons: Camera, depth sensor, IMU
- Labels: "RGB Cameras", "Depth Sensors (RealSense)", "Proprioception (IMU, Joint Encoders)"
- Outputs: "Object Poses, Point Clouds, Obstacle Maps"

**Layer 2 - Cognition** (Middle):
- Central box: "VLA Models (OpenVLA, RT-2)"
- Labels: "Vision Encoders (CLIP, DINOv2)", "Language Understanding", "Action Prediction"
- Outputs: "Joint Velocities, Grasps, Trajectories"

**Layer 3 - Action** (Top):
- Icons: Robot arm, humanoid legs
- Labels: "Motion Planning (MoveIt 2)", "Low-Level Control (MPC, PID)", "Actuators (Motors, Servos)"
- Outputs: "Physical Manipulation, Bipedal Locomotion"

**Virtuous Cycle** (Arrows connecting components):
1. "GPU-Accelerated Simulation (Isaac Sim)" → generates training data → VLA Models
2. "VLA Models" → learn policies → "Edge Deployment (Jetson Orin)"
3. "Edge Deployment" → collects real-world data → "Improved Simulation Models"
4. "Improved Simulation Models" → closes loop back to Simulation

**Style**: Clean, modern diagram with icons. Use blue for perception layer, green for cognition, orange for action. Use circular arrows to indicate the virtuous cycle.

**Tool Suggestions**: draw.io, Figma, Adobe Illustrator, or Python (Matplotlib/Graphviz)

---

## Figure 1-2: Commercial Humanoid Platform Comparison Table

**Location in Text**: Section 2.1 (Commercial Humanoids)
**File Name**: `fig-1-2-humanoid-comparison-table.webp` (or .png)
**Type**: Comparison table/infographic
**Size**: 1200×800 pixels

**Description**:
A table comparing three commercial humanoid platforms:

| Feature | Figure 02 | Unitree G1 | Tesla Optimus Gen 2 |
|---------|-----------|------------|---------------------|
| **Manufacturer** | Figure AI (USA) | Unitree Robotics (China) | Tesla (USA) |
| **Height** | 167 cm | 127-170 cm (adjustable) | 173 cm |
| **Weight** | 60 kg | 47 kg | 73 kg |
| **DOF** | 16 (legs) + 6 (arms) = 22 | 23 (12 legs, 6 arms, 3 torso, 2 head) | 28 |
| **Actuators** | Electric | Electric (torque-controlled) | Custom electric + FT sensing |
| **Walking Speed** | ~1.0 m/s | 1.5 m/s | Not disclosed |
| **Key Innovation** | End-to-end VLA with GPT-4V | Ultra-low-cost actuators ($200/unit) | FSD vision transformers |
| **Deployment** | BMW, Amazon pilots | Open ecosystem (ROS 2, GitHub) | Tesla factories (internal) |
| **Cost** | $150K-$250K (estimated) | ~$16,000 | Projected <$20K at scale |
| **Open Source** | No | Yes (URDF, ROS 2 drivers) | No |

**Additional Visual Elements**:
- Include small product photos of each humanoid (3 images across the top)
- Use color coding: Green for "Yes/Available", Red for "No/Proprietary", Yellow for "Estimated"
- Highlight cost row (most important differentiator)

**Style**: Professional table with alternating row colors for readability. Include company logos.

**Tool Suggestions**: Excel → export as image, Canva, Adobe Illustrator, or Python (Pandas + Matplotlib)

---

## Optional Figure 1-3: Embodiment Hypothesis Diagram

**Location in Text**: Section 1.3 (could be added)
**File Name**: `fig-1-3-embodiment-hypothesis.webp`
**Type**: Conceptual diagram
**Size**: 600×400 pixels

**Description**:
A diagram illustrating the embodiment hypothesis: Intelligence emerges from Brain ↔ Body ↔ Environment interactions.

**Components**:
- **Brain (Cognition)**: Neural network icon, labeled "VLA Models, Planning, Learning"
- **Body (Morphology)**: Humanoid silhouette, labeled "Actuators, Sensors, Kinematics"
- **Environment**: World icon, labeled "Physics, Objects, Humans"

**Interaction Arrows**:
- Brain → Body: "Motor Commands (Joint Velocities)"
- Body → Brain: "Sensory Feedback (Vision, Proprioception)"
- Body ↔ Environment: "Physical Interactions (Contact, Forces)"
- Environment → Brain: "Perception (Object Poses, Affordances)"

**Caption**: "The Embodiment Hypothesis: Intelligence emerges from continuous sensorimotor loops between cognition, morphology, and environment."

**Style**: Circular layout with three nodes (Brain, Body, Environment) and bidirectional arrows. Use icons instead of text where possible.

---

## Implementation Notes

**Format Recommendations**:
- **WebP**: Recommended for web version (10-30% smaller than PNG)
- **PNG**: Good for print version (lossless)
- **SVG**: Best for diagrams that need to scale (vector format)

**Color Palette** (for consistency across all figures):
- Primary Blue: #2E86DE (perception, technology)
- Primary Green: #10AC84 (cognition, intelligence)
- Primary Orange: #FF6348 (action, control)
- Neutral Gray: #57606F (text, backgrounds)
- Accent Yellow: #FFC312 (highlights, warnings)

**Accessibility**:
- Ensure 4.5:1 contrast ratio for all text on backgrounds (WCAG 2.1 AA)
- Include alt text in Markdown: `![Alt text describing diagram](path/to/image.webp)`
- Avoid red-green color combinations (colorblind-friendly palette)

**File Naming Convention**:
- `fig-{chapter}-{number}-{short-description}.webp`
- Example: `fig-1-1-physical-ai-stack.webp`

---

## Task Status

- [ ] Figure 1-1: Physical AI Stack and Virtuous Cycle
- [ ] Figure 1-2: Commercial Humanoid Comparison Table
- [ ] Figure 1-3: Embodiment Hypothesis Diagram (optional)

**Tools Needed**: Graphic design software (Figma, Adobe Illustrator, draw.io) or Python (Matplotlib, Graphviz)
**Estimated Time**: 2-3 hours for all figures

**Note**: These figures require graphic design tools that are outside the scope of text-based content generation. The specifications above provide detailed requirements for a graphic designer or the author to create these figures.
