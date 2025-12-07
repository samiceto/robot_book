# Instructor Resources Plan

**Date**: 2025-12-06
**Status**: Phase 2 Complete
**Related Tasks**: T028
**Scope**: Lecture slides, assignments, grading rubrics, quiz/exam questions, syllabus templates

## Purpose

This document defines the comprehensive instructor resources package that accompanies the textbook. It enables university instructors to adopt the book for courses with minimal preparation time, providing ready-to-use materials for lectures, assignments, assessments, and course planning.

---

## Instructor Resources Overview

### Resource Categories

| Category | Deliverables | Format | Location |
|----------|--------------|--------|----------|
| **Lecture Slides** | 21 slide decks (one per chapter) | PowerPoint (.pptx) + PDF | `instructor-resources/slides/` |
| **Assignments** | 21 assignment prompts + solutions | Markdown + PDF | `instructor-resources/assignments/` |
| **Grading Rubrics** | 3 rubric templates (labs, projects, capstone) | Markdown + PDF | `instructor-resources/rubrics/` |
| **Quiz Questions** | 6 quiz sets (one per part) | Markdown + Moodle XML | `instructor-resources/quizzes/` |
| **Exam Questions** | 50+ question bank with solutions | Markdown + LaTeX | `instructor-resources/exam-questions/` |
| **Syllabus Templates** | 2 syllabi (13-week semester, 10-week quarter) | Markdown + PDF | `instructor-resources/syllabus-templates/` |

**Total Deliverables**: ~100 files (21 slide decks + 21 assignments + 21 solutions + 6 quizzes + 1 exam bank + 2 syllabi + 3 rubrics)

---

## Lecture Slides (21 Slide Decks)

### Structure Template

Each chapter has a corresponding slide deck with the following structure:

#### Slide Deck Template (40-60 slides per chapter)

**Section 1: Chapter Overview** (Slides 1-5)
- Title slide (chapter number, title, learning objectives)
- Prerequisites review (2-3 slides)
- Chapter roadmap (outline of sections)

**Section 2: Theory and Concepts** (Slides 6-30)
- Key concepts (1 concept per slide, visual diagrams)
- Mathematical derivations (step-by-step, progressive reveal)
- Algorithms (pseudocode + flowcharts)
- Example problems (worked solutions)

**Section 3: Hands-On Lab Overview** (Slides 31-40)
- Lab objectives (what students will build)
- Setup checklist (hardware, software, dependencies)
- Step-by-step demo (screenshots or screen recordings)
- Troubleshooting tips (common errors)

**Section 4: End-of-Chapter Project** (Slides 41-50)
- Project requirements (deliverables, validation criteria)
- Grading rubric overview (how project will be graded)
- Example project showcase (demo video or screenshots)

**Section 5: Further Reading and Q&A** (Slides 51-60)
- Recommended papers, blog posts, videos
- Q&A slide (placeholder for instructor notes)

---

### Example: Chapter 3 Slide Deck (ROS 2 Fundamentals)

**File**: `instructor-resources/slides/chapter-03-ros2-fundamentals.pptx`
**Slide Count**: 55 slides

**Sample Slides**:
1. **Slide 1**: Title - "Chapter 3: ROS 2 Fundamentals - Nodes, Topics, Services, Actions"
2. **Slide 2**: Learning Objectives (5 bullet points)
3. **Slide 3**: Prerequisites Review (ROS 1 knowledge optional, Linux CLI required)
4. **Slide 4-6**: ROS 2 Conceptual Overview (diagram: nodes, topics, services, actions)
5. **Slide 7-12**: Topics (publisher/subscriber pattern, QoS policies)
6. **Slide 13-18**: Services (request-reply pattern, custom .srv files)
7. **Slide 19-24**: Actions (long-running tasks, goal/feedback/result pattern)
8. **Slide 25-30**: Parameters and Launch Files
9. **Slide 31-35**: Debugging Tools (`ros2 topic echo`, `rqt_graph`, `ros2 bag`)
10. **Slide 36-45**: Hands-On Lab Demo (building multi-node system)
11. **Slide 46-50**: End-of-Chapter Project (humanoid joint controller)
12. **Slide 51-55**: Further Reading (ROS 2 Design Docs, DDS Specification)

**Design Guidelines**:
- **Visuals**: Use diagrams, flowcharts, and screenshots (not text-heavy slides)
- **Code Snippets**: Max 10-15 lines per slide, syntax-highlighted
- **Animations**: Progressive reveal for complex concepts (e.g., build up architecture diagram layer by layer)
- **Accessibility**: High-contrast colors (WCAG 2.1 AA), avoid red/green for colorblind accessibility

---

### Slide Deck Creation Timeline

**Phase 3 (Writing)**: Create slide decks concurrently with chapter writing
- After completing first draft of each chapter, create corresponding slide deck (2-3 hours per deck)
- Total: 21 chapters × 2.5 hours = **52.5 hours** (spread across Phase 3)

**Phase 4 (Review)**: Academic reviewers provide feedback on slide deck clarity
- Update slide decks based on feedback (1 hour per deck)
- Total: 21 hours

**Deliverable**: 21 PowerPoint (.pptx) + 21 PDF versions (for non-PowerPoint users)

---

## Assignments (21 Prompts + Solutions)

### Assignment Template

Each chapter has a corresponding assignment based on the end-of-chapter project, with clear prompts, deliverables, and solution sketches.

#### Assignment Structure

**File**: `instructor-resources/assignments/assignment-XX-chapter-name.md`

**Sections**:
1. **Assignment Title**: (e.g., "Assignment 3: Multi-Node ROS 2 System")
2. **Learning Objectives**: (3-5 objectives aligned with chapter)
3. **Prerequisites**: (prior chapters, skills required)
4. **Assignment Prompt**: (task description, 200-300 words)
5. **Deliverables**:
   - Code repository (GitHub link)
   - Demo video (2-3 minutes, screen recording)
   - Written report (2-3 pages, answers to reflection questions)
6. **Validation Criteria**: (how students can verify their solution works)
7. **Grading Rubric Reference**: (link to rubric in `rubrics/` directory)
8. **Estimated Time**: (e.g., "10-12 hours")
9. **Due Date**: (placeholder for instructor to fill in)
10. **Reflection Questions** (3-5 questions):
    - E.g., "What QoS policy did you use for the `/imu` topic and why?"
    - E.g., "How would you modify your system to handle sensor failures?"

---

### Example: Assignment 3 (ROS 2 Fundamentals)

**File**: `instructor-resources/assignments/assignment-03-ros2-fundamentals.md`

```markdown
# Assignment 3: Multi-Node ROS 2 System

## Learning Objectives

1. Implement publishers and subscribers in ROS 2
2. Create custom services and action servers
3. Orchestrate multi-node systems with launch files
4. Debug ROS 2 systems using introspection tools

## Prerequisites

- Completed Chapter 2 (ROS 2 environment set up)
- Python proficiency (OOP, async)
- Basic understanding of sensors (IMU, cameras)

## Assignment Prompt

Create a multi-node ROS 2 system that simulates a sensor processing pipeline for a humanoid robot. Your system must include:

1. **sensor_simulator** node: Publishes simulated IMU data (`/imu` topic, sensor_msgs/Imu, 50 Hz)
2. **filter** node: Subscribes to `/imu`, applies moving average filter, publishes `/imu_filtered` (50 Hz)
3. **calibration_service** node: Offers `/calibrate_imu` service to reset IMU bias
4. **long_computation_action** node: Offers `/compute_trajectory` action (simulates 5-second computation with progress feedback)

All nodes must be started via a single launch file.

## Deliverables

1. **Code Repository**:
   - GitHub repository with all 4 nodes
   - README.md with setup instructions
   - launch/system.launch.py file
   - requirements.txt (if any Python dependencies)

2. **Demo Video** (2-3 minutes):
   - Launch all nodes with launch file
   - Use `ros2 topic hz /imu_filtered` to verify 50 Hz
   - Call `/calibrate_imu` service with `ros2 service call`
   - Start action client and show progress feedback

3. **Written Report** (2-3 pages PDF):
   - Answer reflection questions (see below)
   - Include screenshots of `rqt_graph` showing node connections

## Validation Criteria

- [ ] All 4 nodes launch without errors
- [ ] `/imu` and `/imu_filtered` topics publish at 50 Hz (±5 Hz)
- [ ] `/calibrate_imu` service responds correctly
- [ ] `/compute_trajectory` action provides progress feedback
- [ ] `rqt_graph` shows correct node connections

## Grading Rubric

See `instructor-resources/rubrics/lab-assignment-rubric.md`

**Breakdown**:
- Code Quality (30%): Follows ROS 2 style guide, Black formatting, no flake8 violations
- Functionality (40%): All validation criteria met
- Documentation (20%): README clear, code commented
- Video Demonstration (10%): Demo video shows all functionality

## Estimated Time

10-12 hours

## Due Date

[Instructor to fill in]

## Reflection Questions

1. **QoS Policies**: What QoS policy did you use for the `/imu` and `/imu_filtered` topics? Why?
2. **Failure Handling**: How would you modify your system to handle sensor failures (e.g., `/imu` stops publishing)?
3. **Performance**: What is the maximum rate you can publish `/imu` before the filter node starts dropping messages? (Test with `ros2 topic hz`)
4. **Scalability**: How would you extend this system to support multiple sensors (e.g., 5 IMUs on different body parts)?
5. **Real-World Deployment**: What challenges would you face deploying this system on a real humanoid robot (e.g., Jetson Orin)?
```

---

### Solution Sketch Template

**File**: `instructor-resources/assignments/assignment-03-solution.md`

```markdown
# Assignment 3 Solution Sketch

## Code Structure

```
assignment-03/
├── src/
│   ├── sensor_simulator.py       # Publishes /imu at 50 Hz
│   ├── filter_node.py             # Subscribes to /imu, publishes /imu_filtered
│   ├── calibration_service.py     # Service server for /calibrate_imu
│   └── long_computation_action.py # Action server for /compute_trajectory
├── launch/
│   └── system.launch.py           # Launch all 4 nodes
├── README.md
└── requirements.txt
```

## Key Implementation Details

**sensor_simulator.py**:
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import random

class SensorSimulator(Node):
    def __init__(self):
        super().__init__('sensor_simulator')
        self.publisher = self.create_publisher(Imu, '/imu', 10)
        self.timer = self.create_timer(1.0 / 50.0, self.publish_imu)  # 50 Hz

    def publish_imu(self):
        msg = Imu()
        msg.linear_acceleration.x = random.gauss(0, 0.1)
        # ... (full implementation in code repo)
        self.publisher.publish(msg)
```

**filter_node.py** (Moving Average Filter):
```python
class FilterNode(Node):
    def __init__(self):
        super().__init__('filter_node')
        self.subscription = self.create_subscription(Imu, '/imu', self.filter_callback, 10)
        self.publisher = self.create_publisher(Imu, '/imu_filtered', 10)
        self.window = []  # Moving average window

    def filter_callback(self, msg):
        self.window.append(msg.linear_acceleration.x)
        if len(self.window) > 5:  # Window size = 5
            self.window.pop(0)
        avg = sum(self.window) / len(self.window)
        # ... (publish filtered message)
```

## Expected Output

**rqt_graph**:
```
sensor_simulator → /imu → filter_node → /imu_filtered
```

**ros2 topic hz /imu_filtered**:
```
average rate: 50.123 Hz
min: 0.019s max: 0.021s std dev: 0.00043s window: 100
```

## Reflection Answers

1. **QoS Policies**: Used RELIABLE durability for critical sensor data (ensures no message loss)
2. **Failure Handling**: Add heartbeat monitoring, switch to degraded mode if `/imu` timeout
3. **Performance**: Max ~500 Hz before filter node drops messages (tested with `ros2 topic pub` at high rate)
4. **Scalability**: Use topic namespaces (`/imu_left_foot`, `/imu_right_foot`) and dynamic node creation
5. **Real-World**: Challenges include sensor noise (needs adaptive filtering), clock synchronization (NTP), power management (Jetson thermal throttling)
```

---

### Assignment Creation Timeline

**Phase 3 (Writing)**: Create assignments concurrently with chapters
- After completing chapter draft, create assignment prompt (1 hour) + solution sketch (2 hours)
- Total: 21 chapters × 3 hours = **63 hours**

**Phase 4 (Review)**: Student testers validate assignments (10-15 testers complete 4-5 assignments each)
- Update assignments based on feedback (1 hour per assignment)
- Total: 21 hours

---

## Grading Rubrics (3 Templates)

### Rubric 1: Lab Assignment Rubric

**File**: `instructor-resources/rubrics/lab-assignment-rubric.md`

**Grading Categories** (100 points total):

| Category | Weight | Criteria | Points |
|----------|--------|----------|--------|
| **Code Quality** | 30% | - Follows ROS 2 style guide<br>- Black formatting (0 violations)<br>- flake8 linting (0 violations)<br>- Clear variable names, comments | 30 |
| **Functionality** | 40% | - All validation criteria met<br>- No runtime errors<br>- Performance targets met (e.g., 50 Hz topic rate) | 40 |
| **Documentation** | 20% | - README clear and complete<br>- Code comments explain logic<br>- Reflection questions answered thoroughly | 20 |
| **Demo Video** | 10% | - Shows all functionality<br>- Clear screen recording (2-3 min)<br>- Narration or captions explaining steps | 10 |

**Detailed Rubric** (see Appendix D in book for full version)

---

### Rubric 2: End-of-Chapter Project Rubric

**File**: `instructor-resources/rubrics/project-rubric.md`

**Grading Categories** (100 points total):

| Category | Weight | Criteria | Points |
|----------|--------|----------|--------|
| **Technical Correctness** | 40% | - Project meets all requirements<br>- Validation criteria pass<br>- Performance targets met | 40 |
| **Creativity / Innovation** | 20% | - Goes beyond basic requirements<br>- Novel approach or extension<br>- Demonstrates understanding | 20 |
| **Performance Benchmarks** | 20% | - Benchmarks documented<br>- Targets met (e.g., ≥30 FPS Isaac Sim)<br>- Results reproducible | 20 |
| **Report Quality** | 20% | - Clear methodology<br>- Results analysis (graphs, tables)<br>- Reflection on challenges | 20 |

---

### Rubric 3: Capstone Project Rubric (Chapter 20)

**File**: `instructor-resources/rubrics/capstone-rubric.md`

**Grading Categories** (100 points total):

| Category | Weight | Criteria | Points |
|----------|--------|----------|--------|
| **System Integration** | 30% | - All components integrated (Whisper, LLM, Nav2, VLA)<br>- ROS 2 topics/services/actions correct<br>- End-to-end pipeline works | 30 |
| **Performance Targets** | 30% | - ≥12 Hz on Jetson Orin Nano 8GB<br>- <2 GB RAM usage<br>- Task success rate ≥80% | 30 |
| **Code Quality & Documentation** | 20% | - Modular code structure<br>- Clear README with setup instructions<br>- Docker deployment (optional bonus) | 20 |
| **Demo Video** | 20% | - Shows end-to-end demo<br>- Voice command → robot action<br>- Performance metrics displayed | 20 |

---

## Quiz Questions (6 Quiz Sets)

### Quiz Structure

Each Part (Parts 1-6) has a corresponding quiz with 10-15 multiple-choice and short-answer questions.

#### Quiz Template

**File**: `instructor-resources/quizzes/quiz-part1-foundations.md`

**Format**: Markdown + Moodle XML export

**Question Types**:
1. **Multiple Choice** (60%): 4 options, 1 correct answer
2. **Short Answer** (30%): 1-2 sentence answers
3. **Code Completion** (10%): Fill in missing code snippets

---

### Example: Quiz for Part 1 (Foundations & ROS 2)

**File**: `instructor-resources/quizzes/quiz-part1-foundations.md`

```markdown
# Quiz: Part 1 - Foundations & ROS 2

**Total Points**: 100
**Time Limit**: 45 minutes
**Questions**: 12

---

## Question 1 (Multiple Choice, 8 points)

What is the primary difference between ROS 1 and ROS 2?

A) ROS 2 uses Python 3, ROS 1 uses Python 2
B) ROS 2 is built on DDS for real-time communication, ROS 1 uses custom middleware
C) ROS 2 only supports Ubuntu 22.04, ROS 1 supports multiple OS
D) ROS 2 does not support services or actions

**Correct Answer**: B

**Explanation**: ROS 2 uses Data Distribution Service (DDS) for middleware, enabling real-time communication and improved security.

---

## Question 2 (Multiple Choice, 8 points)

Which ROS 2 Quality of Service (QoS) policy should you use for critical sensor data that must not be lost?

A) BEST_EFFORT
B) RELIABLE
C) TRANSIENT_LOCAL
D) VOLATILE

**Correct Answer**: B

**Explanation**: RELIABLE durability ensures messages are not lost (retransmitted if dropped), critical for sensor data.

---

## Question 3 (Short Answer, 10 points)

Explain the difference between a ROS 2 service and a ROS 2 action. Give one example use case for each. (2-3 sentences)

**Sample Answer**:
A ROS 2 service is synchronous request-reply (client waits for server response), suitable for short operations like `/calibrate_imu`. A ROS 2 action is asynchronous with goal/feedback/result, suitable for long-running tasks like `/navigate_to_pose` (navigation goal with progress feedback).

---

## Question 4 (Code Completion, 10 points)

Complete the following ROS 2 publisher code to publish an IMU message at 50 Hz:

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

class ImuPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher = self.create_publisher(______, '/imu', 10)  # Fill in blank 1
        self.timer = self.create_timer(______, self.publish_imu)    # Fill in blank 2

    def publish_imu(self):
        msg = Imu()
        self.publisher.publish(msg)
```

**Blank 1**: `Imu` (message type)
**Blank 2**: `1.0 / 50.0` or `0.02` (50 Hz = 1/50 seconds)

---

[... 8 more questions covering URDF, Isaac Sim, Gazebo]
```

**Moodle XML Export**: Provide Moodle XML file for direct import into Learning Management Systems (LMS)

---

## Exam Question Bank (50+ Questions)

### Question Bank Structure

**File**: `instructor-resources/exam-questions/question-bank.md`

**Organization**:
- 50+ questions covering all 6 parts
- Sorted by difficulty (Easy, Medium, Hard)
- Sorted by Bloom's Taxonomy level (Remember, Understand, Apply, Analyze, Evaluate, Create)
- Tagged by chapter and topic

**Question Types**:
1. **Multiple Choice** (30 questions)
2. **Short Answer** (15 questions)
3. **Long Answer / Essay** (5 questions)
4. **Code Analysis** (5 questions)
5. **Problem Solving** (5 questions)

---

### Example Questions

**Easy (Remember/Understand)**:
```markdown
**Q1**: What does URDF stand for?
- A) Universal Robot Description Format
- B) Unified Robot Data Framework
- C) Universal Robotics Development Framework
- D) Unified Robot Description Format

**Answer**: A

**Bloom's Level**: Remember
**Topic**: Chapter 4 - URDF
```

**Medium (Apply/Analyze)**:
```markdown
**Q15**: Given a humanoid with 23 DOF (6 per leg, 7 per arm, 3 torso), how many actuators are required to achieve full-body control?

**Answer**: 23 actuators (one per DOF)

**Bloom's Level**: Apply
**Topic**: Chapter 4 - URDF, Chapter 17 - Locomotion
```

**Hard (Evaluate/Create)**:
```markdown
**Q35**: You are deploying a VLA model on Jetson Orin Nano 8GB and achieving only 5 Hz inference (target: ≥10 Hz). Describe three optimization strategies you would apply, in order of expected impact. (150-200 words)

**Sample Answer**:
1. **INT8 Quantization** (Expected: 2-3× speedup): Convert FP32 model to INT8 using TensorRT, reducing memory footprint from ~14 GB to ~7 GB and leveraging Tensor Cores.
2. **TensorRT Engine Conversion** (Expected: 1.5-2× additional speedup): Convert PyTorch model to TensorRT engine for optimized inference (layer fusion, kernel optimization).
3. **Pipeline Parallelization** (Expected: 1.2-1.5× speedup): Overlap vision preprocessing (on CPU) with model inference (on GPU) using CUDA streams.

Combined, these should achieve ~6-9× speedup, bringing 5 Hz → 30-45 Hz (well above 10 Hz target).

**Bloom's Level**: Evaluate, Create
**Topic**: Chapter 12 - Jetson Deployment, Chapter 14 - OpenVLA
```

---

## Syllabus Templates (2 Templates)

### Template 1: 13-Week Semester

**File**: `instructor-resources/syllabus-templates/13-week-semester.md`

**Course Title**: CS/ECE 598 - Physical AI & Humanoid Robotics
**Credits**: 3 semester hours
**Prerequisites**: CS 101 (Python), MATH 241 (Linear Algebra), ECE 202 (Signals & Systems) or equivalent

**Course Description**:
This course provides a comprehensive introduction to humanoid robotics and embodied AI. Students will learn to program humanoid robots using ROS 2, simulate physics-based interactions in Isaac Sim, deploy vision-language-action (VLA) models on edge hardware, and build end-to-end autonomous systems. The course culminates in a capstone project: a voice-controlled autonomous humanoid.

**Learning Outcomes**:
1. Implement multi-node robotic systems using ROS 2 (nodes, topics, services, actions)
2. Create and simulate humanoid robots in Isaac Sim with realistic physics
3. Deploy GPU-accelerated perception pipelines (Isaac ROS, Nav2)
4. Integrate vision-language-action (VLA) models for manipulation
5. Optimize models for edge deployment (Jetson Orin, TensorRT, quantization)
6. Build end-to-end autonomous systems integrating multiple AI components

**Grading**:
- Lab Assignments (21 assignments, 40%): Weekly hands-on labs
- End-of-Chapter Projects (4 projects, 30%): Larger projects every 3-4 weeks
- Capstone Project (20%): Final voice-controlled humanoid
- Midterm Exam (5%): Week 7
- Final Exam (5%): Week 13

---

#### Weekly Schedule

| Week | Topics (Chapters) | Lab Due | Project Due |
|------|------------------|---------|-------------|
| 1 | Introduction to Physical AI (Ch 1) | - | - |
| 2 | Environment Setup (Ch 2) + ROS 2 Basics (Ch 3) | Lab 2 | - |
| 3 | URDF & Robot Modeling (Ch 4) | Lab 3 | Project 1 (Ch 2-4) |
| 4 | Gazebo Basics (Ch 5) + Isaac Sim Intro (Ch 6) | Lab 5 | - |
| 5 | Isaac Sim Advanced (Ch 7) | Lab 7 | - |
| 6 | Simulation Benchmarking (Ch 8) | Lab 8 | Project 2 (Ch 5-8) |
| 7 | **Midterm Exam** (Parts 1-2) | - | - |
| 8 | Computer Vision (Ch 9) + Isaac ROS (Ch 10) | Lab 10 | - |
| 9 | Nav2 Navigation (Ch 11) | Lab 11 | - |
| 10 | Jetson Deployment (Ch 12) | Lab 12 | Project 3 (Ch 9-12) |
| 11 | VLA Models (Ch 13-14) | Lab 14 | - |
| 12 | LLM Planning (Ch 15) + Multimodal Perception (Ch 16) | Lab 16 | Project 4 (Ch 13-16) |
| 13 | **Capstone Project Presentations** (Ch 20) | - | **Capstone** |

**Note**: Chapters 17-19 (Bipedal Locomotion, Whole-Body Control, Sim-to-Real) optional for advanced students or graduate seminar follow-up course.

---

### Template 2: 10-Week Quarter

**File**: `instructor-resources/syllabus-templates/10-week-quarter.md`

**Accelerated Pacing**: 2 chapters per week (vs. 1.5 chapters per week in semester)

| Week | Topics (Chapters) | Lab Due | Project Due |
|------|------------------|---------|-------------|
| 1 | Ch 1-2: Introduction + Environment Setup | Lab 2 | - |
| 2 | Ch 3-4: ROS 2 Fundamentals + URDF | Lab 4 | Project 1 (Ch 2-4) |
| 3 | Ch 5-6: Gazebo + Isaac Sim Intro | Lab 6 | - |
| 4 | Ch 7-8: Isaac Sim Advanced + Benchmarking | Lab 8 | Project 2 (Ch 5-8) |
| 5 | Ch 9-10: Computer Vision + Isaac ROS | Lab 10 | - |
| 6 | Ch 11-12: Nav2 + Jetson Deployment | Lab 12 | Project 3 (Ch 9-12) |
| 7 | Ch 13-14: VLA Models + OpenVLA Integration | Lab 14 | - |
| 8 | Ch 15-16: LLM Planning + Multimodal Perception | Lab 16 | Project 4 (Ch 13-16) |
| 9 | Ch 20: Capstone Project Development | - | - |
| 10 | **Capstone Presentations + Final Exam** | - | **Capstone** |

---

## Instructor Access and Distribution

### Access Methods

**Option 1: Public GitHub Repository** (Free, Open-Source)
- All instructor resources hosted in public repo: `github.com/your-org/robot-book-instructor-resources`
- Instructors clone repo to access all materials
- No authentication required (MIT License, same as code repos)

**Option 2: Instructor-Only Portal** (If Traditional Publisher)
- Publisher hosts instructor resources behind login (e.g., O'Reilly Instructor Resources, Manning's MEAP)
- Instructors request access via email (verify instructor status with .edu email or syllabus)
- Access to slide decks, solutions, exam questions

**Option 3: Hybrid Approach** (Recommended)
- **Public**: Slide decks, assignment prompts, rubrics (help students understand expectations)
- **Private**: Solutions, exam question bank (prevent students from accessing answers)

---

### License and Usage Terms

**Instructor Resources License**: Creative Commons BY-NC-SA 4.0
- **BY** (Attribution): Instructors must credit book when using materials
- **NC** (Non-Commercial): Cannot sell slide decks or assignments commercially
- **SA** (Share-Alike): Derivative works must use same license

**Permitted Uses**:
- ✅ Use slide decks in university courses
- ✅ Modify assignments to fit course schedule
- ✅ Create custom quizzes based on question bank
- ✅ Share materials with TAs and co-instructors

**Prohibited Uses**:
- ❌ Sell slide decks or assignments on third-party platforms
- ❌ Post solution sketches publicly (students could access)
- ❌ Use materials in commercial bootcamps without author permission

---

## Creation Timeline

### Phase 3 (Writing, Concurrent with Chapters)

**Weekly Effort**: 5-8 hours per chapter for instructor resources

| Resource | Time Per Chapter | Total Time (21 Chapters) |
|----------|------------------|--------------------------|
| Slide Deck | 2.5 hours | 52.5 hours |
| Assignment Prompt | 1 hour | 21 hours |
| Solution Sketch | 2 hours | 42 hours |
| Quiz Questions (6 quizzes ÷ 21 chapters) | 0.5 hours | 10.5 hours |
| **Total** | **6 hours/chapter** | **126 hours** |

**Timeline**: Spread across Phase 3 (Months 5-12, 32 weeks)
- Average: 4 hours per week on instructor resources (concurrent with 25-30 hours chapter writing)

---

### Phase 4 (Review and Finalization)

**Tasks**:
1. Academic reviewers validate slide decks (clarity, accuracy) - 2 weeks
2. Student testers validate assignments (completeness, time estimates) - 4 weeks
3. Create exam question bank (50+ questions) - 1 week
4. Finalize syllabus templates (2 syllabi) - 1 week

**Total**: 8 weeks (concurrent with Phase 4 technical review)

---

## Success Criteria

**Primary Metric**: ≥2 university instructors pilot the book in their courses and provide structured feedback

**Secondary Metrics**:
1. **Slide Deck Quality**: Instructors rate slide decks as "clear and useful" (≥4/5 Likert scale)
2. **Assignment Completeness**: ≤10% of students request additional clarification on assignment prompts
3. **Exam Question Bank**: ≥50 questions covering all 6 parts with solutions
4. **Syllabus Templates**: 2 templates (13-week semester, 10-week quarter) provided

---

## Document Status

**Phase**: Phase 2 In Progress (Foundational Research)
**Next Milestone**: Begin Chapter 1 writing (Phase 3, Week 17)
**Last Updated**: 2025-12-06
**Review Frequency**: Quarterly during Phase 3 (update as chapters completed)
