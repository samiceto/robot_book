# Implementation Plan: Physical AI & Humanoid Robotics Textbook

**Branch**: `002-humanoid-robotics-book` | **Date**: 2025-12-06 | **Spec**: [spec.md](./spec.md)

## Summary

This plan details the complete architecture and execution strategy for writing "Physical AI & Humanoid Robotics: From Simulated Brains to Walking Bodies" - a 550-650 page practitioner textbook targeting advanced undergraduates, Master's students, industry engineers, and university instructors. The book integrates theoretical foundations with hands-on labs, covering the modern 2025-2027 embodied AI stack (ROS 2 Iron/Jazzy, NVIDIA Isaac Sim 2024.2+, Isaac ROS 3.0+, VLA models, Jetson Orin edge deployment). Writing follows a research-concurrent + code-concurrent approach with all code tested weekly on real hardware (RTX 4090 + Jetson Orin Nano 8GB).

## Technical Context

**Book Format**: Markdown source → LaTeX → PDF (print), Leanpub (digital), Docusaurus (web)
**Total Length**: 550-650 pages (~160,000-190,000 words including code)
**Chapter Count**: 18-22 chapters across 6 parts
**Primary Software Stack**:
- ROS 2 Iron Irwini (2023-2027 support) OR Jazzy Jalisco (2024-2029 support)
- NVIDIA Isaac Sim 2024.2+ (Omniverse platform)
- Isaac ROS 3.0+ (GPU-accelerated ROS packages)
- Nav2 (latest stable for ROS 2 Iron/Jazzy)
- Ubuntu 22.04 LTS (supported until 2027) OR 24.04 LTS (supported until 2029)
- Python 3.10+ (required for ROS 2 Iron/Jazzy)
- PyTorch 2.0+ (VLA models, fine-tuning)
- Jetson Orin JetPack 5.x/6.x (edge deployment)

**Target Hardware Tiers**:
- Budget: RTX 4070 Ti 12GB (~$800-1000), total system <$1200
- Mid: RTX 4080 16GB (~$1200-1500), total system $2500-3500
- Premium: RTX 4090 24GB (~$1600-2000) + Jetson AGX Orin 64GB (~$2000), total system $5000+
- Edge: Jetson Orin Nano 8GB ($249) for deployment testing

**Code Repositories**: 12-15 companion GitHub repos (MIT license), one per major topic
**Testing Infrastructure**: GitHub Actions with RTX 4090 GPU runners, weekly CI validation
**Code Style**: Black formatter + ROS 2 Python style guide (PEP8 + flake8)
**Citation Style**: APA 7th edition
**Web Platform**: Docusaurus 3.x for searchable, mobile-responsive web version

**Performance Goals**:
- Isaac Sim rendering: ≥30 FPS (Budget tier), ≥60 FPS (Mid), ≥120 FPS (Premium)
- VLA inference: ≥10 Hz (Jetson Orin Nano), ≥15 Hz (Orin NX), ≥30 Hz (Orin AGX)
- Nav2 planning: <500ms for 10m path
- End-to-end capstone pipeline: ≥12 Hz on Jetson Orin Nano 8GB, <2 GB RAM

**Constraints**:
- All code must work on both ROS 2 Iron (Ubuntu 22.04) AND Jazzy (Ubuntu 24.04)
- Every lab must be completable in ≤15 hours by external testers using only the book
- Zero broken links or deprecated packages at publication
- Maintain code compatibility for minimum 3 years post-publication

**Scale/Scope**:
- 18-22 chapters, 6 parts
- 12-15 companion repositories
- 50+ quiz/exam questions for instructors
- 3-5 industry case studies/interviews
- 200+ citations (minimum 10 per chapter)

**Project Type**: Multi-artifact publishing project (print book + ebook + web + code repos + instructor resources)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**Constitution Status**: The project constitution is currently a template and needs to be defined. For this book project, core principles should include:

### Proposed Core Principles (To Be Ratified)

1. **Code-First Verification**: Every code example must pass CI (Ubuntu 22.04/24.04 + ROS 2 Iron/Jazzy + Isaac Sim 2024.2+) before chapter is considered complete
2. **Reproducibility Mandate**: All labs must be validated by minimum 3 independent external testers completing them in ≤15 hours
3. **Version Pinning**: Exact software versions documented in requirements.txt, Dockerfiles, and installation guides to ensure long-term reproducibility
4. **Accessibility Requirements**: WCAG 2.1 AA compliance for web version, alt text for all figures, colorblind-safe code highlighting
5. **Maintenance Commitment**: Quarterly software compatibility reviews for Year 1, biannual thereafter, minimum 3-year code support post-publication
6. **Openness**: Code repositories MIT-licensed, web version freely accessible (Docusaurus), errata/updates tracked publicly on GitHub

### Gate Checks

✅ **PASS**: Book format (Markdown → multi-format) enables version control, collaboration, and automated builds
✅ **PASS**: Testing infrastructure (GitHub Actions + GPU runners) automates verification
✅ **PASS**: Multi-tier hardware recommendations (Budget/Mid/Premium + cloud) ensure accessibility
⚠️ **REVIEW NEEDED**: Constitution template must be filled with project-specific principles before Phase 0
⚠️ **REVIEW NEEDED**: Major architectural decisions (humanoid platform, VLA model, simulation engine, edge hardware, code license) need explicit documentation with tradeoffs

## Project Structure

### Documentation (this feature)

```text
specs/002-humanoid-robotics-book/
├── spec.md              # Feature specification (COMPLETED)
├── plan.md              # This file (IN PROGRESS)
├── research.md          # Phase 0 output (TO BE CREATED)
├── book-architecture.md # Phase 1 output: complete chapter/section breakdown (TO BE CREATED)
├── word-count-budget.md # Phase 1 output: word count per chapter (TO BE CREATED)
├── writing-timeline.md  # Phase 1 output: 12-14 month timeline with milestones (TO BE CREATED)
├── decisions/           # Phase 0-1 output: architectural decisions with tradeoffs (TO BE CREATED)
│   ├── humanoid-platform.md
│   ├── vla-model-choice.md
│   ├── simulation-engine.md
│   ├── edge-hardware.md
│   └── code-license.md
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
# Option: Book + Code Monorepo Structure

book/
├── chapters/
│   ├── part1-foundations/
│   │   ├── 01-introduction-to-physical-ai.md
│   │   ├── 02-quickstart-ros2-isaac-sim.md
│   │   ├── 03-ros2-fundamentals.md
│   │   └── 04-robot-description-urdf-sdf.md
│   ├── part2-simulation/
│   │   ├── 05-gazebo-basics.md
│   │   ├── 06-isaac-sim-introduction.md
│   │   ├── 07-isaac-sim-advanced.md
│   │   └── 08-simulation-benchmarking.md
│   ├── part3-perception-edge/
│   │   ├── 09-computer-vision-fundamentals.md
│   │   ├── 10-isaac-ros-perception.md
│   │   ├── 11-nav2-navigation-stack.md
│   │   └── 12-jetson-orin-deployment.md
│   ├── part4-embodied-cognition/
│   │   ├── 13-vision-language-action-overview.md
│   │   ├── 14-vla-model-integration.md
│   │   ├── 15-llm-task-planning.md
│   │   └── 16-multimodal-perception.md
│   ├── part5-locomotion/
│   │   ├── 17-bipedal-locomotion-basics.md
│   │   ├── 18-whole-body-control.md
│   │   └── 19-sim-to-real-transfer.md
│   └── part6-capstone/
│       ├── 20-voice-controlled-humanoid.md
│       └── 21-safety-ethics-deployment.md
├── appendices/
│   ├── a-hardware-buyers-guide.md
│   ├── b-math-control-primer.md
│   ├── c-troubleshooting-reference.md
│   ├── d-instructor-resources.md
│   ├── e-glossary.md
│   └── f-installation-checklists.md
├── frontmatter/
│   ├── preface.md
│   ├── acknowledgments.md
│   ├── about-author.md
│   └── how-to-use-this-book.md
└── images/
    ├── chapter-01/
    ├── chapter-02/
    └── ... (one folder per chapter)

code/
├── chapter-03-ros2-basics/          # Companion repo for Chapter 3
│   ├── README.md
│   ├── requirements.txt
│   ├── examples/
│   │   ├── hello_ros2.py
│   │   ├── publisher_subscriber.py
│   │   └── service_client.py
│   ├── tests/
│   └── .github/workflows/ci.yml
├── chapter-06-isaac-sim-intro/      # Companion repo for Chapter 6
│   ├── README.md
│   ├── requirements.txt
│   ├── urdf/
│   │   └── simple_humanoid.urdf
│   ├── scripts/
│   │   ├── load_urdf.py
│   │   └── basic_simulation.py
│   ├── tests/
│   └── .github/workflows/ci.yml
├── chapter-11-nav2/                 # Companion repo for Chapter 11
├── chapter-14-vla-integration/      # Companion repo for Chapter 14
├── chapter-16-jetson-deployment/    # Companion repo for Chapter 16
├── chapter-20-capstone/             # Companion repo for Chapter 20 (end-to-end demo)
│   ├── README.md
│   ├── requirements.txt
│   ├── docker/
│   │   ├── Dockerfile.jetson
│   │   └── docker-compose.yml
│   ├── src/
│   │   ├── voice_recognition/       # Whisper integration
│   │   ├── llm_planner/             # LLM task planning
│   │   ├── navigation/              # Nav2 integration
│   │   ├── manipulation/            # VLA model integration
│   │   └── main.py                  # End-to-end pipeline
│   ├── config/
│   ├── tests/
│   └── .github/workflows/ci.yml
└── shared/
    ├── urdf_models/                 # Shared URDF/SDF models
    ├── isaac_sim_utils/             # Shared Isaac Sim utilities
    └── ros2_utils/                  # Shared ROS 2 utilities

website/
├── docusaurus.config.js
├── src/
│   ├── pages/
│   │   ├── index.md
│   │   ├── errata.md
│   │   └── resources.md
│   └── css/
├── docs/                            # Symlink or copy from book/chapters
├── static/
│   ├── img/
│   └── files/
└── sidebars.js

instructor-resources/
├── slides/
│   ├── chapter-01-slides.pptx
│   ├── chapter-02-slides.pptx
│   └── ... (one slide deck per chapter)
├── assignments/
│   ├── assignment-01-ros2-basics.md
│   ├── assignment-01-rubric.md
│   ├── assignment-01-solution.md
│   └── ...
├── quizzes/
│   ├── quiz-part1-foundations.md
│   ├── quiz-part2-simulation.md
│   └── ...
├── exam-questions/
│   └── question-bank.md             # 50+ questions with answers
└── syllabus-templates/
    ├── 13-week-semester.md
    └── 10-week-quarter.md
```

**Structure Decision**: Monorepo structure chosen to keep book content (Markdown chapters), companion code repositories, Docusaurus website, and instructor resources in a single version-controlled location. This enables:
- Atomic commits linking chapter updates to code changes
- Unified CI pipeline testing both book builds and code examples
- Single source of truth for all project artifacts
- Easier contributor onboarding and maintenance

Alternative considered: Separate repositories for book, code, and website. Rejected because it complicates synchronization (e.g., updating Chapter 14 text + Chapter 14 code + website docs requires 3 PRs across 3 repos).

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| Multiple code repositories (12-15) vs. single repo | Each chapter needs independent versioning, CI, and student cloning; monorepo would make "clone Chapter 14 code" ambiguous | Single monorepo with 15+ packages rejected: complicates student setup (must clone entire 5+ GB repo to try one example), CI takes 30+ min to test all packages, and GitHub Classroom integration breaks (cannot assign one chapter's code as starter repo) |
| Dual Ubuntu version support (22.04 + 24.04) | ROS 2 Iron requires 22.04 (still widely deployed in universities), Jazzy requires 24.04 (future-proofing); supporting both ensures book relevance 2026-2029 | Supporting only Ubuntu 24.04 rejected: excludes readers on 22.04 LTS systems; supporting only 22.04 rejected: book obsolete when 24.04 adoption grows in 2026-2027 |
| Dual ROS 2 distribution support (Iron + Jazzy) | Iron is current LTS (supported until 2027), Jazzy is next LTS (2024-2029); book must work with both for longevity | Supporting only Iron rejected: book obsolete by 2027; supporting only Jazzy rejected: incompatible with current university lab setups using Iron |

## Book Architecture: Complete Chapter Breakdown

### Part 1: Foundations & Nervous System (ROS 2)

**Target Length**: 100-120 pages (~28,000-34,000 words)
**Chapters**: 4
**Learning Outcomes**: Readers gain foundational understanding of Physical AI concepts, install and configure ROS 2 + Isaac Sim environment, master ROS 2 pub/sub/services, and create robot descriptions (URDF/SDF).

---

#### Chapter 1: Introduction to Physical AI & Humanoid Robotics

**Word Count**: 7,000-8,000 words
**Learning Objectives**:
- Define Physical AI and contrast with disembodied AI (LLMs, vision models)
- Explain the embodiment hypothesis and why humanoid form matters
- Survey the state of humanoid robotics in 2025 (Figure AI, Tesla Optimus, Unitree, Boston Dynamics)
- Identify key technical challenges: perception, manipulation, bipedal locomotion, sim-to-real transfer
- Understand the course structure and prerequisite knowledge

**Section Structure**:
1. **The Embodiment Hypothesis** (H2)
   - Why Physical AI Differs from Software-Only AI (H3)
   - The Moravec Paradox: Easy for Humans, Hard for Robots (H3)
   - Embodied Cognition: Perception-Action Loops (H3)

2. **Humanoid Robotics in 2025: State of the Art** (H2)
   - Industry Leaders: Figure AI, Tesla Optimus, Agility Robotics (H3)
   - Research Platforms: Unitree G1, Boston Dynamics Atlas (H3)
   - Open-Source Alternatives: Poppy, THOR, GummiArm (H3)

3. **Key Technical Challenges** (H2)
   - Perception: Vision, Tactile Sensing, Proprioception (H3)
   - Manipulation: Grasping, Dexterous Control, Tool Use (H3)
   - Bipedal Locomotion: Balance, Gait Generation, Terrain Adaptation (H3)
   - Sim-to-Real Transfer: The Reality Gap Problem (H3)

4. **The Modern Physical AI Stack** (H2)
   - Operating System: Ubuntu LTS (H3)
   - Middleware: ROS 2 (Why Not ROS 1?) (H3)
   - Simulation: Isaac Sim, Gazebo, MuJoCo (H3)
   - Perception: Isaac ROS, OpenCV, Point Cloud Libraries (H3)
   - Intelligence: Vision-Language-Action Models (VLA) (H3)
   - Edge Deployment: NVIDIA Jetson Orin (H3)

5. **Prerequisites and How to Use This Book** (H2)
   - Required Knowledge: Python, Deep Learning Basics, Linux CLI (H3)
   - Hardware Requirements: Budget/Mid/Premium Tiers (H3)
   - Chapter Structure: Theory → Lab → Project → References (H3)
   - Companion Code Repositories and Testing (H3)

6. **Hands-On Lab: None (introductory chapter)**

7. **End-of-Chapter Project: Environment Survey**
   - Task: Research and compare 3 humanoid robots (commercial or research)
   - Deliverable: 2-page comparison report (kinematics, sensors, cost, applications)
   - Validation: Report includes at least 5 citations and performance specifications

8. **Further Reading**:
   - Moravec, H. (1988). *Mind Children*
   - Brooks, R. (1991). "Intelligence Without Representation" (paper)
   - Modern humanoid robotics survey papers (2023-2025)

---

#### Chapter 2: Quickstart - Your First ROS 2 + Isaac Sim Demo in 4 Hours

**Word Count**: 8,000-9,000 words
**Learning Objectives**:
- Install Ubuntu 22.04 LTS in dual-boot or VM configuration
- Install ROS 2 Iron Irwini with core packages
- Install NVIDIA drivers, CUDA, and Isaac Sim 2024.2
- Run first "Hello ROS 2" publisher-subscriber example
- Load a simple humanoid URDF in Isaac Sim and control it via ROS 2 topics

**Section Structure**:
1. **Preparing Your System** (H2)
   - Hardware Verification: GPU Check, RAM, Storage (H3)
   - Ubuntu 22.04 LTS Installation: Dual-Boot vs. VM vs. Bare Metal (H3)
   - NVIDIA Driver Installation and Verification (H3)

2. **Installing ROS 2 Iron Irwini** (H2)
   - Adding ROS 2 apt Repository (H3)
   - Installing `ros-iron-desktop` and Core Tools (H3)
   - Environment Setup: .bashrc Configuration (H3)
   - Verification: Running `ros2 topic list` (H3)

3. **Installing NVIDIA Isaac Sim 2024.2** (H2)
   - Downloading Isaac Sim via Omniverse Launcher (H3)
   - CUDA and cuDNN Dependencies (H3)
   - Isaac Sim Python Environment Setup (H3)
   - First Launch and Performance Check (≥30 FPS on RTX 4070 Ti) (H3)

4. **Hello ROS 2: Your First Publisher-Subscriber** (H2)
   - Writing a Simple Publisher (hello_publisher.py) (H3)
   - Writing a Simple Subscriber (hello_subscriber.py) (H3)
   - Running Nodes and Inspecting Topics with `rqt_graph` (H3)

5. **Loading a Humanoid URDF in Isaac Sim** (H2)
   - What is URDF? Quick Overview (H3)
   - Downloading a Sample Humanoid URDF (Appendix Reference) (H3)
   - Importing URDF into Isaac Sim (H3)
   - Controlling Joints via ROS 2 `/joint_command` Topic (H3)

6. **Hands-On Lab: End-to-End Setup Validation** (H2)
   - **Time Estimate**: 4 hours
   - **Setup**: Fresh Ubuntu 22.04 VM or bare metal install
   - **Tasks**:
     1. Install ROS 2 Iron and verify with `ros2 topic list`
     2. Install Isaac Sim 2024.2 and launch a demo scene
     3. Run hello_publisher.py and hello_subscriber.py in separate terminals
     4. Load provided simple_humanoid.urdf in Isaac Sim
     5. Publish joint commands to move humanoid's arm
   - **Validation**: Screen-record a 30-second video showing Isaac Sim humanoid responding to ROS 2 joint commands
   - **Troubleshooting**: Common errors in Appendix C (driver conflicts, ROS 2 sourcing issues, Isaac Sim GPU errors)

7. **End-of-Chapter Project: Custom URDF Modification**
   - Task: Modify the provided simple_humanoid.urdf to add a camera sensor to the head
   - Deliverable: Updated URDF file + screenshot of Isaac Sim showing camera feed
   - Validation: Camera topic appears in `ros2 topic list` and publishes image data

8. **Further Reading**:
   - ROS 2 Iron documentation (docs.ros.org)
   - Isaac Sim 2024.2 User Manual (NVIDIA docs)
   - URDF Tutorials (ros.org)

---

#### Chapter 3: ROS 2 Fundamentals - Publishers, Subscribers, Services, Actions

**Word Count**: 9,000-10,000 words
**Learning Objectives**:
- Understand ROS 2 architecture: nodes, topics, services, actions, parameters
- Master publish-subscribe pattern for sensor data and actuator commands
- Implement request-reply pattern using services
- Implement long-running tasks using actions with feedback
- Debug ROS 2 systems using command-line tools (ros2 topic, ros2 service, ros2 node, rqt)

**Section Structure**:
1. **ROS 2 Architecture Overview** (H2)
   - Nodes: The Basic Unit of Computation (H3)
   - Topics: Anonymous Publish-Subscribe (H3)
   - Services: Synchronous Request-Reply (H3)
   - Actions: Asynchronous Long-Running Tasks (H3)
   - Parameters: Runtime Configuration (H3)

2. **Topics and Publish-Subscribe** (H2)
   - Creating a Publisher (C++ and Python Examples) (H3)
   - Creating a Subscriber (C++ and Python Examples) (H3)
   - Quality of Service (QoS) Policies: Reliability, Durability, History (H3)
   - Message Types: std_msgs, sensor_msgs, geometry_msgs (H3)

3. **Services for Request-Reply** (H2)
   - Defining a Custom Service (.srv Files) (H3)
   - Implementing a Service Server (H3)
   - Calling a Service from a Client (H3)
   - Synchronous vs. Asynchronous Service Calls (H3)

4. **Actions for Long-Running Tasks** (H2)
   - Action Definition (.action Files): Goal, Feedback, Result (H3)
   - Implementing an Action Server (H3)
   - Implementing an Action Client with Feedback Handling (H3)
   - Use Case: Navigation Goal with Progress Feedback (H3)

5. **Parameters and Launch Files** (H2)
   - Declaring and Using Parameters in Nodes (H3)
   - Launch Files: Orchestrating Multi-Node Systems (H3)
   - YAML Parameter Files for Configuration (H3)

6. **Debugging and Introspection Tools** (H2)
   - `ros2 topic echo`, `ros2 topic hz`, `ros2 topic info` (H3)
   - `ros2 service call` for Testing Services (H3)
   - `rqt_graph` for Visualizing Node Connections (H3)
   - `ros2 bag` for Recording and Replaying Data (H3)

7. **Hands-On Lab: Building a Multi-Node System** (H2)
   - **Time Estimate**: 8 hours
   - **Setup**: ROS 2 Iron environment from Chapter 2
   - **Tasks**:
     1. Create a sensor_simulator node publishing `/imu` (sensor_msgs/Imu) at 50 Hz
     2. Create a filter node subscribing to `/imu` and publishing `/imu_filtered` at 50 Hz (simple moving average)
     3. Create a calibration_service node offering `/calibrate_imu` service (resets bias)
     4. Create a long_computation_action node offering `/compute_trajectory` action (simulates 5-second computation with progress feedback)
     5. Write a launch file starting all 4 nodes
     6. Test with `ros2 topic hz /imu_filtered`, `ros2 service call /calibrate_imu`, and action client
   - **Validation**: Launch file starts all nodes, topics publish at expected rates, service and action respond correctly
   - **Troubleshooting**: Node crashes, topic rate issues, service timeouts

8. **End-of-Chapter Project: Humanoid Joint Controller**
   - Task: Create a ROS 2 action server that accepts a joint trajectory goal (list of joint positions + timestamps) and executes it on Isaac Sim humanoid from Chapter 2
   - Deliverable: Action server code, launch file, demo video showing smooth joint trajectory execution
   - Validation: Action provides progress feedback (% completion), completes trajectory within 10% time error

9. **Further Reading**:
   - ROS 2 Design Documentation (design.ros2.org)
   - DDS (Data Distribution Service) specification
   - QoS policies deep dive (ROS 2 docs)

**Companion Repository**: `code/chapter-03-ros2-basics/`

---

#### Chapter 4: Robot Description - URDF, SDF, and Xacro for Humanoids

**Word Count**: 8,000-9,000 words
**Learning Objectives**:
- Understand Unified Robot Description Format (URDF) syntax and structure
- Define kinematic chains: links, joints, transforms
- Add visual and collision geometry, inertial properties
- Use Xacro (XML Macros) for modular, reusable robot descriptions
- Convert URDF to SDF (Simulation Description Format) for Gazebo/Isaac Sim
- Validate URDF/SDF with command-line tools (check_urdf, gz sdf)

**Section Structure**:
1. **Why Robot Descriptions Matter** (H2)
   - From CAD to Simulation: The Robot Description Pipeline (H3)
   - URDF vs. SDF vs. MuJoCo XML: Format Comparison (H3)

2. **URDF Fundamentals** (H2)
   - Links: Visual, Collision, Inertial Properties (H3)
   - Joints: Fixed, Revolute, Prismatic, Continuous (H3)
   - Transforms: Origin and Coordinate Frames (H3)
   - Example: Simple 2-Link Arm (H3)

3. **Building a Humanoid URDF** (H2)
   - Kinematic Tree Structure: Torso → Limbs (H3)
   - Defining Degrees of Freedom (DOF): 6-DOF Legs, 7-DOF Arms (H3)
   - Adding Sensors: Cameras, IMU, Force-Torque Sensors (H3)
   - Gazebo-Specific Tags and Plugins (H3)

4. **Xacro for Modular Descriptions** (H2)
   - Macros: Parameterizing Limbs and Sensors (H3)
   - Properties: Constants and Expressions (H3)
   - Include Files: Reusing Sensor Definitions (H3)
   - Example: Xacro for Humanoid with Swappable Hands (H3)

5. **SDF (Simulation Description Format)** (H2)
   - SDF vs. URDF: Key Differences (Worlds, Models, Actors) (H3)
   - Converting URDF to SDF with `gz sdf` (H3)
   - Isaac Sim's USD (Universal Scene Description) Integration (H3)

6. **Validation and Debugging** (H2)
   - `check_urdf`: Syntax and Kinematic Tree Validation (H3)
   - `urdf_to_graphviz`: Visualizing Kinematic Tree (H3)
   - `gz sdf --check`: SDF Validation (H3)
   - Visualizing in RViz2 and Isaac Sim (H3)

7. **Hands-On Lab: Creating a Custom Humanoid URDF** (H2)
   - **Time Estimate**: 10 hours
   - **Setup**: ROS 2 Iron + Isaac Sim 2024.2
   - **Tasks**:
     1. Define a simplified humanoid (torso, 2 legs with 3 joints each, 2 arms with 2 joints each, head)
     2. Add visual meshes (STL or Collada) for each link
     3. Define collision geometry (simplified boxes/cylinders)
     4. Compute inertial properties (mass, inertia tensor) using CAD tool or approximations
     5. Add a front-facing camera sensor to the head
     6. Convert URDF to Xacro with parameterized leg length
     7. Validate with `check_urdf` and `gz sdf --check`
     8. Load in Isaac Sim and verify camera feed
   - **Validation**: URDF passes validation, camera topic publishes images at 30 Hz, humanoid stands upright in Isaac Sim with gravity enabled
   - **Troubleshooting**: Link collisions, joint limits, inertia errors causing instability

8. **End-of-Chapter Project: Modular Sensor Platform**
   - Task: Create a Xacro-based humanoid description with swappable end-effectors (gripper, suction cup, dexterous hand)
   - Deliverable: Xacro files, launch file to load different configs in Isaac Sim, demo video showing 3 different end-effectors
   - Validation: Each end-effector configuration loads without errors, joint limits respected, collision geometry correct

9. **Further Reading**:
   - URDF XML Specification (ros.org)
   - SDF Format Specification (sdformat.org)
   - USD (Universal Scene Description) documentation (NVIDIA)

**Companion Repository**: `code/chapter-04-urdf-sdf/`

---

### Part 2: Digital Twins & Simulation Mastery

**Target Length**: 90-110 pages (~26,000-32,000 words)
**Chapters**: 4
**Learning Outcomes**: Readers master Gazebo and Isaac Sim for humanoid simulation, create realistic physics environments, benchmark simulation performance, and integrate ROS 2 with simulators.

---

#### Chapter 5: Gazebo Classic and Gazebo (Ignition/Harmonic) Basics

**Word Count**: 7,000-8,000 words
**Learning Objectives**:
- Understand Gazebo architecture: client-server, physics engines (ODE, Bullet, DART)
- Launch Gazebo worlds and spawn robots via ROS 2
- Configure physics parameters: gravity, time step, solver iterations
- Add sensors: cameras, LiDAR, depth cameras, contact sensors
- Understand Gazebo vs. Isaac Sim tradeoffs (performance, realism, ROS 2 integration)

**Section Structure**:
1. **Gazebo vs. Isaac Sim: When to Use Which** (H2)
2. **Gazebo Architecture** (H2)
3. **Launching Gazebo with ROS 2** (H2)
4. **Physics Configuration** (H2)
5. **Sensor Plugins** (H2)
6. **Hands-On Lab: Humanoid in Gazebo with Camera and IMU** (H2)
7. **End-of-Chapter Project: Custom Gazebo World** (H2)
8. **Further Reading**

**Companion Repository**: `code/chapter-05-gazebo/`

---

#### Chapter 6: NVIDIA Isaac Sim - Introduction and Setup

**Word Count**: 8,000-9,000 words
**Learning Objectives**:
- Install Isaac Sim 2024.2 via Omniverse platform
- Understand Isaac Sim architecture: USD, PhysX, RTX rendering
- Create first scene: humanoid on flat plane with lighting
- Integrate Isaac Sim with ROS 2 using Isaac ROS bridge
- Benchmark Isaac Sim performance on Budget/Mid/Premium hardware tiers

**Section Structure**:
1. **What is Isaac Sim?** (H2)
2. **Installation and Setup** (H2)
3. **Isaac Sim UI: Layers, Stage, Prims** (H2)
4. **Creating Your First Scene** (H2)
5. **Isaac ROS Bridge: Connecting to ROS 2** (H2)
6. **Performance Benchmarking** (H2)
7. **Hands-On Lab: Humanoid Manipulation Scene** (H2)
8. **End-of-Chapter Project: Multi-Object Scene** (H2)
9. **Further Reading**

**Companion Repository**: `code/chapter-06-isaac-sim-intro/`

---

#### Chapter 7: Isaac Sim Advanced - Domain Randomization, Synthetic Data, Replicator

**Word Count**: 9,000-10,000 words
**Learning Objectives**:
- Implement domain randomization for sim-to-real transfer
- Generate synthetic datasets for vision model training
- Use Isaac Sim Replicator API for automated data generation
- Profile Isaac Sim performance bottlenecks
- Optimize scenes for real-time performance (LOD, culling, shader complexity)

**Section Structure**:
1. **Domain Randomization for Sim-to-Real** (H2)
2. **Synthetic Data Generation** (H2)
3. **Isaac Sim Replicator API** (H2)
4. **Performance Profiling** (H2)
5. **Scene Optimization** (H2)
6. **Hands-On Lab: Randomized Grasping Dataset** (H2)
7. **End-of-Chapter Project: Custom Replicator Pipeline** (H2)
8. **Further Reading**

**Companion Repository**: `code/chapter-07-isaac-sim-advanced/`

---

#### Chapter 8: Simulation Benchmarking and Validation

**Word Count**: 7,000-8,000 words
**Learning Objectives**:
- Define simulation fidelity metrics: physics accuracy, rendering quality, latency
- Benchmark Gazebo vs. Isaac Sim vs. MuJoCo on standard tasks
- Validate simulation against real-world data (if available)
- Understand performance tradeoffs: speed vs. accuracy
- Choose appropriate simulator for different use cases

**Section Structure**:
1. **Simulation Fidelity Metrics** (H2)
2. **Benchmarking Framework** (H2)
3. **Gazebo vs. Isaac Sim vs. MuJoCo Comparison** (H2)
4. **Real-World Validation** (H2)
5. **Hands-On Lab: Benchmark Suite** (H2)
6. **End-of-Chapter Project: Performance Report** (H2)
7. **Further Reading**

**Companion Repository**: `code/chapter-08-simulation-benchmarking/`

---

### Part 3: Perception & Edge Brain (Isaac ROS, Jetson Deployment)

**Target Length**: 90-110 pages (~26,000-32,000 words)
**Chapters**: 4
**Learning Outcomes**: Readers implement computer vision pipelines, deploy Nav2 for autonomous navigation, optimize models for Jetson Orin edge deployment, and achieve real-time performance targets.

---

#### Chapter 9: Computer Vision Fundamentals for Humanoid Robotics

**Word Count**: 8,000-9,000 words
**Learning Objectives**:
- Understand camera models: pinhole, distortion, calibration
- Process RGB-D data: point clouds, depth estimation
- Implement object detection: YOLO, Faster R-CNN
- Implement pose estimation: MediaPipe, OpenPose
- Integrate vision with ROS 2: sensor_msgs, cv_bridge

**Section Structure**:
1. **Camera Models and Calibration** (H2)
2. **RGB-D Processing** (H2)
3. **Object Detection** (H2)
4. **Pose Estimation** (H2)
5. **ROS 2 Integration** (H2)
6. **Hands-On Lab: Object Detection Pipeline** (H2)
7. **End-of-Chapter Project: Person Following** (H2)
8. **Further Reading**

**Companion Repository**: `code/chapter-09-computer-vision/`

---

#### Chapter 10: Isaac ROS - GPU-Accelerated Perception

**Word Count**: 9,000-10,000 words
**Learning Objectives**:
- Understand Isaac ROS GEMs architecture
- Deploy DNN-based perception: object detection, segmentation, depth estimation
- Use NITROS for zero-copy inter-process communication
- Profile and optimize Isaac ROS pipelines
- Achieve ≥30 Hz perception on Jetson Orin Nano

**Section Structure**:
1. **What is Isaac ROS?** (H2)
2. **Isaac ROS GEMs Overview** (H2)
3. **DNN Inference with TensorRT** (H2)
4. **NITROS Zero-Copy IPC** (H2)
5. **Pipeline Profiling and Optimization** (H2)
6. **Hands-On Lab: Isaac ROS Object Detection** (H2)
7. **End-of-Chapter Project: Real-Time Segmentation** (H2)
8. **Further Reading**

**Companion Repository**: `code/chapter-10-isaac-ros/`

---

#### Chapter 11: Nav2 - Autonomous Navigation for Humanoids

**Word Count**: 10,000-11,000 words
**Learning Objectives**:
- Understand Nav2 architecture: behavior trees, planners, controllers, recoveries
- Configure costmaps: global, local, inflation layers
- Select and tune planners: DWB, TEB, MPPI, Smac Planner
- Implement behavior trees for complex navigation tasks
- Deploy Nav2 on humanoid in Isaac Sim with <500ms planning time

**Section Structure**:
1. **Nav2 Architecture** (H2)
2. **Costmap Configuration** (H2)
3. **Global and Local Planners** (H2)
4. **Controllers: DWB, TEB, MPPI** (H2)
5. **Behavior Trees** (H2)
6. **Hands-On Lab: Humanoid Waypoint Navigation** (H2)
7. **End-of-Chapter Project: Multi-Goal Behavior Tree** (H2)
8. **Further Reading**

**Companion Repository**: `code/chapter-11-nav2/`

---

#### Chapter 12: Jetson Orin Edge Deployment - From Simulation to Silicon

**Word Count**: 9,000-10,000 words
**Learning Objectives**:
- Flash Jetson Orin Nano/NX with JetPack 5.x/6.x
- Install ROS 2 Iron/Jazzy on Jetson
- Optimize models with TensorRT, quantization (INT8, FP16)
- Profile memory and compute usage
- Deploy end-to-end perception pipeline at ≥15 Hz on Jetson Orin Nano 8GB

**Section Structure**:
1. **Jetson Orin Hardware Overview** (H2)
2. **JetPack Installation and Setup** (H2)
3. **ROS 2 on Jetson** (H2)
4. **Model Optimization: TensorRT, Quantization** (H2)
5. **Memory and Power Profiling** (H2)
6. **Hands-On Lab: Deploying Isaac ROS on Jetson** (H2)
7. **End-of-Chapter Project: Optimized Perception Pipeline** (H2)
8. **Further Reading**

**Companion Repository**: `code/chapter-12-jetson-deployment/`

---

### Part 4: Embodied Cognition & Vision-Language-Action Models

**Target Length**: 90-110 pages (~26,000-32,000 words)
**Chapters**: 4
**Learning Outcomes**: Readers understand VLA model architecture, integrate pre-trained VLA models (OpenVLA, RT-2-X, Octo), implement LLM task planning, and deploy multimodal perception pipelines.

---

#### Chapter 13: Vision-Language-Action (VLA) Models - Overview and Theory

**Word Count**: 8,000-9,000 words
**Learning Objectives**:
- Understand VLA model architecture: vision encoder, language encoder, action decoder
- Survey VLA models: RT-1, RT-2, OpenVLA, Octo, PaLM-E
- Understand training paradigms: imitation learning, reinforcement learning, pre-training + fine-tuning
- Identify VLA applications: manipulation, navigation, human-robot interaction
- Understand limitations: generalization, sample efficiency, sim-to-real gap

**Section Structure**:
1. **What are VLA Models?** (H2)
2. **VLA Architecture** (H2)
3. **Survey of VLA Models** (H2)
4. **Training Paradigms** (H2)
5. **Applications and Limitations** (H2)
6. **Hands-On Lab: None (theory chapter)** (H2)
7. **End-of-Chapter Project: VLA Model Comparison Report** (H2)
8. **Further Reading**

---

#### Chapter 14: Integrating OpenVLA for Humanoid Manipulation

**Word Count**: 10,000-11,000 words
**Learning Objectives**:
- Install OpenVLA and dependencies (PyTorch, Transformers)
- Load pre-trained OpenVLA weights
- Implement vision → action inference loop
- Integrate OpenVLA with ROS 2 for real-time control
- Benchmark inference latency on RTX 4090 and Jetson Orin Nano
- Deploy pick-and-place demo in Isaac Sim

**Section Structure**:
1. **OpenVLA Overview** (H2)
2. **Installation and Setup** (H2)
3. **Model Inference Pipeline** (H2)
4. **ROS 2 Integration** (H2)
5. **Performance Benchmarking** (H2)
6. **Hands-On Lab: Pick-and-Place with OpenVLA** (H2)
7. **End-of-Chapter Project: Multi-Object Manipulation** (H2)
8. **Further Reading**

**Companion Repository**: `code/chapter-14-openvla-integration/`

---

#### Chapter 15: LLM Task Planning for Humanoid Robots

**Word Count**: 9,000-10,000 words
**Learning Objectives**:
- Understand LLM-based task planning: prompt engineering, function calling
- Integrate GPT-4, Claude, or Llama 3+ for high-level task decomposition
- Implement LLM → action sequence translation
- Handle failures and replanning
- Combine LLM planning with VLA execution

**Section Structure**:
1. **LLMs for Task Planning** (H2)
2. **Prompt Engineering for Robotics** (H2)
3. **Function Calling and Tool Use** (H2)
4. **Failure Handling and Replanning** (H2)
5. **LLM + VLA Integration** (H2)
6. **Hands-On Lab: Voice-Commanded Task Execution** (H2)
7. **End-of-Chapter Project: Multi-Step Task Planner** (H2)
8. **Further Reading**

**Companion Repository**: `code/chapter-15-llm-planning/`

---

#### Chapter 16: Multimodal Perception - Vision, Language, and Proprioception

**Word Count**: 8,000-9,000 words
**Learning Objectives**:
- Fuse RGB, depth, and tactile data
- Integrate language grounding: CLIP, LLaVA
- Implement proprioceptive feedback loops (joint encoders, IMU)
- Build multimodal state representations
- Deploy multimodal perception on Jetson Orin NX

**Section Structure**:
1. **Multimodal Fusion** (H2)
2. **Language Grounding** (H2)
3. **Proprioceptive Feedback** (H2)
4. **State Representation** (H2)
5. **Hands-On Lab: Multimodal Object Recognition** (H2)
6. **End-of-Chapter Project: Context-Aware Grasping** (H2)
7. **Further Reading**

**Companion Repository**: `code/chapter-16-multimodal-perception/`

---

### Part 5: Bipedal Locomotion & Whole-Body Control

**Target Length**: 80-100 pages (~24,000-29,000 words)
**Chapters**: 3
**Learning Outcomes**: Readers implement bipedal locomotion controllers, understand whole-body control, and apply sim-to-real transfer techniques.

---

#### Chapter 17: Bipedal Locomotion Basics - Gaits, Balance, and Stability

**Word Count**: 9,000-10,000 words
**Learning Objectives**:
- Understand humanoid gaits: walking, running, turning
- Implement Zero Moment Point (ZMP) for balance
- Implement Model Predictive Control (MPC) for locomotion
- Simulate bipedal walking in Isaac Sim
- Tune gait parameters for stability

**Section Structure**:
1. **Humanoid Gaits** (H2)
2. **Zero Moment Point (ZMP)** (H2)
3. **Model Predictive Control (MPC)** (H2)
4. **Simulation and Tuning** (H2)
5. **Hands-On Lab: Walking Controller in Isaac Sim** (H2)
6. **End-of-Chapter Project: Terrain-Adaptive Gait** (H2)
7. **Further Reading**

**Companion Repository**: `code/chapter-17-bipedal-locomotion/`

---

#### Chapter 18: Whole-Body Control - Integrating Locomotion and Manipulation

**Word Count**: 9,000-10,000 words
**Learning Objectives**:
- Understand whole-body control: task-space control, hierarchical QP
- Coordinate locomotion and manipulation
- Implement mobile manipulation tasks
- Deploy whole-body controller in Isaac Sim

**Section Structure**:
1. **Whole-Body Control Overview** (H2)
2. **Task-Space Control** (H2)
3. **Hierarchical Quadratic Programming (QP)** (H2)
4. **Mobile Manipulation** (H2)
5. **Hands-On Lab: Walking While Grasping** (H2)
6. **End-of-Chapter Project: Dynamic Manipulation Task** (H2)
7. **Further Reading**

**Companion Repository**: `code/chapter-18-whole-body-control/`

---

#### Chapter 19: Sim-to-Real Transfer - Closing the Reality Gap

**Word Count**: 9,000-10,000 words
**Learning Objectives**:
- Understand the reality gap: physics, sensors, dynamics
- Implement domain randomization techniques
- Apply system identification for accurate modeling
- Use transfer learning for fine-tuning on real hardware
- Validate sim-to-real transfer on Jetson Orin + robotic arm

**Section Structure**:
1. **The Reality Gap Problem** (H2)
2. **Domain Randomization** (H2)
3. **System Identification** (H2)
4. **Transfer Learning** (H2)
5. **Hands-On Lab: Sim-to-Real Grasping** (H2)
6. **End-of-Chapter Project: Transfer Validation Report** (H2)
7. **Further Reading**

**Companion Repository**: `code/chapter-19-sim-to-real/`

---

### Part 6: Capstone Integration & Deployment

**Target Length**: 80-100 pages (~24,000-29,000 words)
**Chapters**: 2
**Learning Outcomes**: Readers integrate all learned skills into an end-to-end voice-controlled autonomous humanoid system, address safety and ethics considerations, and deploy on Jetson Orin Nano 8GB.

---

#### Chapter 20: Capstone - Voice-Controlled Autonomous Humanoid

**Word Count**: 12,000-14,000 words
**Learning Objectives**:
- Integrate Whisper for speech recognition
- Integrate LLM for task planning
- Integrate Nav2 for navigation
- Integrate OpenVLA for manipulation
- Deploy end-to-end pipeline in Isaac Sim
- Optimize for Jetson Orin Nano 8GB: ≥12 Hz, <2 GB RAM
- Validate with complete demo: "Navigate to kitchen and pick up red mug"

**Section Structure**:
1. **Capstone Architecture Overview** (H2)
   - System Diagram: Whisper → LLM → Nav2 + VLA → Isaac Sim (H3)
   - Performance Requirements: Latency, Throughput, Memory (H3)

2. **Whisper Speech Recognition Integration** (H2)
   - Installing Whisper (OpenAI API or Local Model) (H3)
   - ROS 2 Node for Audio Capture and Transcription (H3)
   - Handling Noisy Environments and Accents (H3)

3. **LLM Task Planning Integration** (H2)
   - Prompt Template for Task Decomposition (H3)
   - Translating Natural Language to ROS 2 Action Sequences (H3)
   - Error Handling and Replanning (H3)

4. **Navigation and Manipulation Integration** (H2)
   - Nav2 for Waypoint Navigation (H3)
   - OpenVLA for Object Grasping (H3)
   - Coordinating Sequential Actions (H3)

5. **End-to-End Pipeline Implementation** (H2)
   - ROS 2 Launch File for All Nodes (H3)
   - State Machine for Task Execution (H3)
   - Logging and Monitoring (H3)

6. **Performance Optimization for Jetson Orin Nano** (H2)
   - Model Quantization (INT8, FP16) (H3)
   - Batching and Pipelining (H3)
   - Memory Profiling and Optimization (H3)
   - Achieving ≥12 Hz End-to-End (H3)

7. **Hands-On Lab: Complete Capstone Demo** (H2)
   - **Time Estimate**: 15 hours
   - **Setup**: Isaac Sim 2024.2 + Jetson Orin Nano 8GB + ROS 2 Iron/Jazzy
   - **Tasks**:
     1. Set up Whisper speech recognition node
     2. Set up LLM task planner node
     3. Integrate Nav2 for navigation
     4. Integrate OpenVLA for manipulation
     5. Create launch file and state machine
     6. Test in Isaac Sim: "Navigate to table and pick up blue cube"
     7. Deploy to Jetson Orin Nano and profile performance
     8. Optimize to achieve ≥12 Hz, <2 GB RAM
   - **Validation**:
     - Voice command → action execution works end-to-end
     - Performance: ≥12 Hz inference, <2 GB RAM on Jetson Orin Nano
     - Success rate: ≥80% for 10 test commands in Isaac Sim
   - **Troubleshooting**: Latency spikes, memory leaks, action failures

8. **End-of-Chapter Project: Custom Task Extension**
   - Task: Extend capstone to handle multi-step tasks (e.g., "Clean the table: pick up all objects and place in bin")
   - Deliverable: Updated code, demo video, performance report
   - Validation: System handles ≥3 sequential pick-and-place actions, replans on failure

9. **Further Reading**:
   - End-to-end robotics systems papers (2023-2025)
   - Production deployment case studies
   - Edge AI optimization techniques

**Companion Repository**: `code/chapter-20-capstone/`

---

#### Chapter 21: Safety, Ethics, and Responsible Deployment

**Word Count**: 8,000-9,000 words
**Learning Objectives**:
- Understand safety considerations: fail-safes, emergency stops, human-in-the-loop
- Implement collision detection and avoidance
- Address ethical considerations: privacy, bias, accountability
- Understand regulatory landscape: ISO 13482, safety certifications
- Plan for responsible deployment in real-world settings

**Section Structure**:
1. **Safety in Humanoid Robotics** (H2)
   - Fail-Safe Mechanisms (H3)
   - Emergency Stop Systems (H3)
   - Human-in-the-Loop Control (H3)

2. **Collision Detection and Avoidance** (H2)
   - Sensor Fusion for Safety (H3)
   - Real-Time Collision Checking (H3)
   - Safe Motion Planning (H3)

3. **Ethical Considerations** (H2)
   - Privacy: Camera Data, Voice Recordings (H3)
   - Bias in VLA Models and LLMs (H3)
   - Accountability and Transparency (H3)

4. **Regulatory Landscape** (H2)
   - ISO 13482: Safety Requirements for Personal Care Robots (H3)
   - CE Marking and Certification Processes (H3)
   - Regional Regulations (US, EU, Asia) (H3)

5. **Responsible Deployment** (H2)
   - Risk Assessment and Mitigation (H3)
   - User Training and Documentation (H3)
   - Ongoing Monitoring and Updates (H3)

6. **Hands-On Lab: Safety System Implementation** (H2)
   - **Time Estimate**: 6 hours
   - **Tasks**:
     1. Implement emergency stop button (ROS 2 topic)
     2. Add collision detection using Isaac Sim contact sensors
     3. Implement safe motion planning (velocity/acceleration limits)
     4. Create safety dashboard (RViz2 or custom UI)
   - **Validation**: Emergency stop halts robot within 100ms, collision detection triggers before contact, velocity limits enforced

7. **End-of-Chapter Project: Ethical Impact Assessment**
   - Task: Write 5-page report assessing ethical implications of deploying a humanoid robot in a public space (e.g., hospital, retail store)
   - Deliverable: Report addressing privacy, bias, safety, and accountability
   - Validation: Report includes at least 10 citations, identifies 5+ risks with mitigation strategies

8. **Further Reading**:
   - IEEE Standards for Robot Safety
   - Research on AI ethics and responsible robotics
   - Case studies of robot deployments and failures

---

## Appendices

### Appendix A: Hardware Buyer's Guide

**Length**: 15-20 pages
**Content**:
- Budget Tier (<$1200): Detailed parts list with exact models, prices (2025), vendor links (Amazon, Newegg)
- Mid Tier ($2500-3500): Same structure
- Premium Tier ($5000+): Same structure
- Jetson Orin Options: Nano 8GB ($249), NX 16GB ($599), AGX Orin 64GB ($2000+)
- Peripherals: Monitors, keyboards, power supplies
- Assembly guides and OS installation checklists

### Appendix B: Math & Control Theory Primer

**Length**: 20-25 pages
**Content**:
- Linear algebra: vectors, matrices, transformations
- Kinematics: forward kinematics, inverse kinematics, Jacobians
- Dynamics: equations of motion, Lagrangian mechanics (brief overview)
- Control theory: PID, LQR, MPC (high-level concepts)
- Optimization: QP, convex optimization basics

### Appendix C: Troubleshooting Reference

**Length**: 15-20 pages
**Content**:
- NVIDIA driver issues: installation, version conflicts, CUDA errors
- ROS 2 build failures: dependency errors, workspace sourcing
- Isaac Sim crashes: GPU memory, PhysX errors
- Nav2 tuning: costmap configuration, planner parameters
- Jetson deployment: flashing errors, package incompatibilities
- Performance debugging: profiling tools, memory leaks

### Appendix D: Grading Rubrics for Instructors

**Length**: 15-20 pages
**Content**:
- Rubrics for each chapter's end-of-chapter project
- Scoring criteria: technical correctness (40%), code quality (20%), documentation (20%), creativity (10%), presentation (10%)
- Example submissions (high/mid/low quality) with scores and feedback

### Appendix E: Glossary of Terms

**Length**: 10-15 pages
**Content**:
- Alphabetical glossary of robotics, AI, and embodied AI terms
- Each entry: term, definition, page reference where first used
- Examples: URDF, ZMP, VLA, DDS, QoS, TensorRT, etc.

### Appendix F: Software Installation Checklists

**Length**: 10-12 pages
**Content**:
- Ubuntu 22.04 LTS installation checklist
- Ubuntu 24.04 LTS installation checklist
- ROS 2 Iron installation checklist
- ROS 2 Jazzy installation checklist
- Isaac Sim 2024.2 installation checklist
- Jetson Orin JetPack installation checklist
- Verification commands for each step

---

## Word Count Budget Summary

| Part/Chapter | Target Words | Cumulative |
|--------------|--------------|------------|
| **Frontmatter** (Preface, Acknowledgments, About Author, How to Use) | 3,000-4,000 | 3,000-4,000 |
| **Part 1: Foundations** | | |
| Chapter 1: Introduction | 7,000-8,000 | 10,000-12,000 |
| Chapter 2: Quickstart | 8,000-9,000 | 18,000-21,000 |
| Chapter 3: ROS 2 Fundamentals | 9,000-10,000 | 27,000-31,000 |
| Chapter 4: URDF/SDF | 8,000-9,000 | 35,000-40,000 |
| **Part 2: Simulation** | | |
| Chapter 5: Gazebo | 7,000-8,000 | 42,000-48,000 |
| Chapter 6: Isaac Sim Intro | 8,000-9,000 | 50,000-57,000 |
| Chapter 7: Isaac Sim Advanced | 9,000-10,000 | 59,000-67,000 |
| Chapter 8: Simulation Benchmarking | 7,000-8,000 | 66,000-75,000 |
| **Part 3: Perception & Edge** | | |
| Chapter 9: Computer Vision | 8,000-9,000 | 74,000-84,000 |
| Chapter 10: Isaac ROS | 9,000-10,000 | 83,000-94,000 |
| Chapter 11: Nav2 | 10,000-11,000 | 93,000-105,000 |
| Chapter 12: Jetson Deployment | 9,000-10,000 | 102,000-115,000 |
| **Part 4: VLA Models** | | |
| Chapter 13: VLA Overview | 8,000-9,000 | 110,000-124,000 |
| Chapter 14: OpenVLA Integration | 10,000-11,000 | 120,000-135,000 |
| Chapter 15: LLM Planning | 9,000-10,000 | 129,000-145,000 |
| Chapter 16: Multimodal Perception | 8,000-9,000 | 137,000-154,000 |
| **Part 5: Locomotion** | | |
| Chapter 17: Bipedal Locomotion | 9,000-10,000 | 146,000-164,000 |
| Chapter 18: Whole-Body Control | 9,000-10,000 | 155,000-174,000 |
| Chapter 19: Sim-to-Real Transfer | 9,000-10,000 | 164,000-184,000 |
| **Part 6: Capstone** | | |
| Chapter 20: Capstone Demo | 12,000-14,000 | 176,000-198,000 |
| Chapter 21: Safety & Ethics | 8,000-9,000 | 184,000-207,000 |
| **Appendices** (A-F) | 85,000-100,000 | **269,000-307,000** |

**ISSUE**: Current estimate (269k-307k words) significantly exceeds target (160k-190k words including code).

**Mitigation Strategy**:
1. **Reduce appendix length**: Appendices currently budgeted at 85k-100k words. Trim to 30k-40k words by:
   - Moving detailed hardware specs to web version only
   - Condensing troubleshooting to common issues only (full guide on website)
   - Reducing glossary to key terms (full glossary on website)
2. **Tighten chapter word counts**: Reduce each chapter by 10-15% through:
   - More concise theoretical sections (prioritize hands-on labs)
   - Moving tangential topics to "Further Reading" or blog posts
   - Combining some chapters (e.g., merge Chapter 5 Gazebo into Chapter 6 Isaac Sim as "Simulation Comparison")
3. **Consider 2-volume approach**: If trimming compromises quality:
   - Volume 1: Parts 1-3 (Foundations, Simulation, Perception/Edge) ~90k-110k words
   - Volume 2: Parts 4-6 (VLA, Locomotion, Capstone) ~70k-80k words

**Revised Target** (with mitigations):
- Main chapters (21): 150,000-170,000 words (avg 7,000-8,000 per chapter)
- Appendices (6): 30,000-40,000 words
- **Total**: 180,000-210,000 words (slightly above target but manageable)

---

## Research Phase 0: Unknowns to Resolve

The following unknowns from Technical Context and architectural decisions must be researched and resolved before full design (Phase 1):

### Research Task 1: Humanoid Platform Selection

**Question**: Which humanoid platform should be used for all book examples?

**Options**:
1. **Unitree G1** ($16k, commercially available, full-body humanoid, 23 DOF)
2. **Figure 02** (not publicly available, waitlist only, cutting-edge)
3. **Tesla Optimus** (not commercially available, development ongoing)
4. **Open-source (Poppy Humanoid, THOR, GummiArm)** (DIY, lower cost, limited DOF)
5. **Proxy platform (Unitree Go2 quadruped + simplified humanoid URDF)** (affordable, widely available, limited humanoid fidelity)

**Research Approach**:
- Survey current (2025) availability and pricing for Unitree G1, other commercial humanoids
- Check if Figure 02 or Tesla Optimus will have developer programs by 2026
- Test open-source platforms for Isaac Sim compatibility and documentation quality
- Prototype simplified humanoid URDF to validate as viable proxy
- Consult with industry contacts (if available) on preferred platforms for education

**Decision Criteria**:
- Cost: Readers should be able to replicate (or simulate) for <$20k hardware budget
- Availability: Platform available for purchase or accurate URDF available
- Documentation: Sufficient technical specs (kinematics, dynamics, sensor specs)
- Isaac Sim compatibility: URDF/USD import works reliably
- Pedagogical value: Represents modern humanoid capabilities

**Deliverable**: `decisions/humanoid-platform.md` with choice, rationale, alternatives considered, and tradeoffs

---

### Research Task 2: VLA Model Backbone Selection

**Question**: Which VLA model should be the primary focus for Chapters 14-16?

**Options**:
1. **OpenVLA** (open-source, PyTorch, pre-trained weights available as of 2024)
2. **RT-2-X** (Google DeepMind, may have access restrictions, cutting-edge performance)
3. **Octo** (UC Berkeley, open-source, generalist manipulation)
4. **Custom Llama-3.1-8B fine-tune** (requires training data, full control, educational value)

**Research Approach**:
- Check OpenVLA, RT-2-X, Octo availability and licensing as of late 2025
- Benchmark inference latency on RTX 4090 and Jetson Orin Nano for each model
- Evaluate pre-trained weight quality for grasping/manipulation tasks in Isaac Sim
- Assess fine-tuning difficulty for custom tasks
- Review documentation and community support for each model

**Decision Criteria**:
- Open-source: Model and weights must be freely available (no API-only access)
- Performance: ≥10 Hz inference on Jetson Orin Nano 8GB
- Generalization: Works on diverse manipulation tasks without extensive retraining
- Documentation: Clear inference API, ROS 2 integration examples
- Longevity: Model likely to remain available through 2027+

**Deliverable**: `decisions/vla-model-choice.md` with choice, rationale, alternatives, and tradeoffs

---

### Research Task 3: Simulation Engine for 80% of Examples

**Question**: Which simulation engine should be primary for the majority of book examples?

**Options**:
1. **Isaac Sim (Omniverse)** (NVIDIA, GPU-accelerated, RTX rendering, PhysX, best Isaac ROS integration)
2. **MuJoCo** (DeepMind, fast physics, good RL support, limited rendering)
3. **PyBullet** (open-source, widely used, Python-friendly, aging codebase)
4. **Gazebo Harmonic** (open-source, ROS 2 native, less performant than Isaac Sim)

**Research Approach**:
- Benchmark Isaac Sim vs. MuJoCo vs. PyBullet vs. Gazebo on humanoid simulation (FPS, physics accuracy, rendering quality)
- Test ROS 2 integration ease for each simulator
- Survey community adoption and documentation quality
- Evaluate long-term support and roadmap (2025-2027)
- Profile GPU/CPU requirements for typical humanoid scenes

**Decision Criteria**:
- Performance: ≥30 FPS on RTX 4070 Ti for typical humanoid + manipulation scene
- ROS 2 integration: Native or well-maintained bridge
- Realism: Visual and physics fidelity suitable for sim-to-real transfer demos
- Accessibility: Free or affordable for students (Isaac Sim free for education)
- Longevity: Active development and support through 2027+

**Deliverable**: `decisions/simulation-engine.md` with choice, rationale, alternatives, and tradeoffs

---

### Research Task 4: Edge Hardware Tier for Reproducibility

**Question**: What is the minimum Jetson Orin hardware tier required for capstone demo reproducibility?

**Options**:
1. **Jetson Orin Nano 8GB** ($249, most accessible, lower performance)
2. **Jetson Orin NX 16GB** ($599, balanced, better performance)
3. **Jetson AGX Orin 64GB** ($2000+, high-end, overkill for most readers)

**Research Approach**:
- Benchmark capstone pipeline (Whisper + LLM + Nav2 + VLA) on each Jetson tier
- Measure inference latency, memory usage, and power consumption
- Test optimization techniques (TensorRT, quantization) on each tier
- Evaluate cost/performance tradeoff for student accessibility
- Survey university lab equipment budgets and typical purchases

**Decision Criteria**:
- Performance: Achieves ≥12 Hz end-to-end capstone inference
- Memory: Runs complete pipeline in <8 GB RAM (Orin Nano) or <16 GB (Orin NX)
- Cost: Accessible to majority of readers (<$600 preferred)
- Availability: In stock and stable pricing through 2026-2027
- Pedagogical value: Teaches real-world edge deployment constraints

**Deliverable**: `decisions/edge-hardware.md` with choice, rationale, alternatives, and tradeoffs

---

### Research Task 5: Code License Selection

**Question**: What license should be used for companion code repositories?

**Options**:
1. **MIT** (permissive, allows commercial use, minimal restrictions)
2. **Apache 2.0** (permissive, explicit patent grant, more protective)
3. **GPL-3.0** (copyleft, requires derivative works to be open-source)

**Research Approach**:
- Review common licenses for robotics code (ROS 2 packages, Isaac Sim examples)
- Consult with legal/publisher on compatibility with book license
- Survey reader preferences (academic vs. industry vs. hobbyist)
- Evaluate impact on community contributions and forks
- Consider commercial use cases (bootcamps, companies)

**Decision Criteria**:
- Permissiveness: Allows readers to use code in commercial projects without restrictions
- Compatibility: Works with ROS 2 (Apache 2.0 dominant), Isaac Sim examples
- Simplicity: Easy to understand for non-lawyers
- Community acceptance: Widely used and trusted in robotics community

**Deliverable**: `decisions/code-license.md` with choice, rationale, alternatives, and tradeoffs

---

### Research Task 6: Docusaurus Features and Configuration

**Question**: What Docusaurus features and plugins are needed for optimal book web version?

**Research Approach**:
- Review Docusaurus 3.x documentation for relevant features
- Identify plugins for: syntax highlighting (Prism), search (Algolia), diagrams (Mermaid), LaTeX (KaTeX)
- Test Markdown → Docusaurus conversion workflow
- Evaluate hosting options (GitHub Pages, Vercel, Netlify)
- Benchmark build times and site performance for 20-chapter book

**Decision Criteria**:
- Search quality: Full-text search across all chapters
- Code highlighting: Supports Python, C++, URDF XML, ROS 2 launch files
- Diagram support: Mermaid or similar for architecture diagrams
- LaTeX support: Inline equations and display equations
- Performance: Lighthouse score ≥90 for performance, accessibility, SEO
- Mobile-responsive: Works on tablets and phones

**Deliverable**: Docusaurus configuration file (docusaurus.config.js) with all plugins and settings documented

---

### Research Task 7: Best Practices for Technical Book Writing

**Question**: What are industry best practices for technical book writing, structure, and publishing workflow?

**Research Approach**:
- Review O'Reilly, Manning, No Starch Press author guides
- Study successful robotics/AI textbooks (Siciliano, Thrun, Sutton & Barto) for structure
- Consult with technical editors or published authors (if accessible)
- Research Markdown → LaTeX → PDF toolchains (Pandoc, LaTeX templates)
- Investigate Leanpub publishing workflow

**Decision Criteria**:
- Proven structure: Clear progression from fundamentals to advanced
- Toolchain reliability: Markdown → multi-format conversion works consistently
- Version control: Git-friendly workflow
- Collaboration: Enables co-authors, technical reviewers, community contributions
- Maintenance: Easy to update for software version changes

**Deliverable**: Writing workflow document describing: chapter template, toolchain (Pandoc, LaTeX), Git branching strategy, review process

---

### Research Task 8: Testing Infrastructure for CI/CD

**Question**: What CI/CD infrastructure is needed to test all companion code repos weekly?

**Research Approach**:
- Investigate GitHub Actions GPU runners (availability, cost, performance)
- Evaluate alternatives (GitLab CI with GPU, self-hosted runners)
- Estimate monthly costs for testing 12-15 repos weekly on RTX 4090 or equivalent
- Design test matrix: Ubuntu 22.04 + ROS 2 Iron, Ubuntu 24.04 + ROS 2 Jazzy
- Prototype CI workflow for one sample repo (e.g., Chapter 3 ROS 2 basics)

**Decision Criteria**:
- GPU support: Must run Isaac Sim tests (requires RTX GPU)
- Cost: <$500/month sustainable for book project
- Reliability: <5% false positives (flaky tests)
- Speed: Full test suite for one repo completes in <30 minutes
- Coverage: Tests build, unit tests, integration tests (Isaac Sim scene loads)

**Deliverable**: Sample GitHub Actions workflow (.github/workflows/ci.yml) and cost estimate spreadsheet

---

## Phase 1 Deliverables (After Research Complete)

Once all Research Phase 0 tasks are complete and decisions documented, Phase 1 will produce:

1. **book-architecture.md**: Complete chapter breakdown with H1 → H2 → H3 section structure for all 21 chapters
2. **word-count-budget.md**: Revised word count budget per chapter (addressing 269k-307k overage issue)
3. **writing-timeline.md**: 12-14 month timeline with milestones:
   - Month 1-3: Part 1 (Foundations) + Chapter 1-4 code repos
   - Month 4-6: Part 2 (Simulation) + Chapter 5-8 code repos
   - Month 7-9: Part 3 (Perception/Edge) + Chapter 9-12 code repos + Part 4 (VLA) start
   - Month 10-12: Part 4 (VLA) complete + Part 5 (Locomotion) + Chapter 13-19 code repos
   - Month 13-14: Part 6 (Capstone) + Chapter 20-21 + Appendices + Frontmatter
   - Month 15-17: Technical review (beta readers + industry reviewers)
   - Month 18-21: Production (copyediting, layout, indexing)
   - Month 22: Publication (Q4 2027 or Q1 2028)
4. **testing-validation-strategy.md**: Complete testing plan:
   - External tester recruitment (3-5 volunteers per chapter lab)
   - CI/CD infrastructure setup (GitHub Actions + GPU runners)
   - Performance benchmarking protocol (Isaac Sim FPS, VLA latency, etc.)
   - URDF/SDF validation (gz sdf check, isaac-sim validator)
   - Link checking automation (weekly cron job)
   - Capstone demo validation (≥12 Hz on Jetson Orin Nano, <2 GB RAM)
5. **instructor-resources-plan.md**: Structure for instructor materials:
   - Slide deck templates (PowerPoint or Google Slides, one per chapter)
   - Assignment prompts with rubrics (one per chapter, aligned with end-of-chapter projects)
   - Quiz/exam question bank (50+ questions across 6 parts)
   - 13-week semester and 10-week quarter syllabus templates
6. **docusaurus-site-structure.md**: Complete Docusaurus configuration and site map

---

## Validation Strategy Summary

From spec.md Success Criteria, the following validation processes are required:

### Code Validation (SC-002, SC-004, SC-012)
- **GitHub Actions CI**: All companion repos tested weekly on Ubuntu 22.04 + ROS 2 Iron + Isaac Sim 2024.2
- **Dual Ubuntu/ROS 2 testing**: Code tested on both Iron (22.04) and Jazzy (24.04) before chapter completion
- **URDF/SDF validation**: All robot models pass `check_urdf` and `gz sdf --check` with zero errors
- **Black + flake8 linting**: All Python code formatted with Black, linted with flake8, zero violations
- **Performance benchmarks**: Isaac Sim FPS, VLA latency, Nav2 planning time measured and documented

### Content Validation (SC-001, SC-006, SC-013)
- **External tester validation**: 3 independent testers per chapter lab, must complete in <15 hours using only book
- **Citation quality**: Every chapter has ≥10 citations, ≥70% from 2020 or later
- **Quickstart validation**: 3 testers verify Chapter 2 quickstart (Ubuntu install → first ROS 2 + Isaac Sim demo) completes in <4 hours

### Deployment Validation (SC-003, SC-010)
- **Capstone performance**: End-to-end demo (Whisper + LLM + Nav2 + VLA) runs at ≥12 Hz on Jetson Orin Nano 8GB, <2 GB RAM
- **Benchmark reproducibility**: Reference hardware achieves documented performance within ±15%

### Publication Validation (SC-005, SC-007, SC-008, SC-011)
- **Page count**: 550-650 pages, 160,000-190,000 words (excluding appendices)
- **Hardware links**: 100% valid purchase links (2 vendors per component) verified within 30 days of publication
- **Link checking**: Automated link checking (weekly CI job), zero broken links at publication
- **Web performance**: Docusaurus site achieves Lighthouse score ≥90 for performance, accessibility, SEO

### Instructor Validation (SC-009, SC-015)
- **Pilot courses**: ≥2 university instructors pilot the book, provide structured feedback
- **Instructor resources**: ≥50 quiz/exam questions with answer keys, grading rubrics for all 21 chapters

### Industry Validation (SC-014)
- **Case studies**: ≥3 practitioner interviews or case studies from Figure AI, Tesla, Agility Robotics, Boston Dynamics, or equivalent

---

## Next Steps

1. **User approval of this plan**: Review architectural decisions, chapter structure, and research tasks
2. **Execute Research Phase 0**: Complete all 8 research tasks, document decisions in `decisions/` folder
3. **Create detailed chapter outlines**: Expand each chapter's H2 → H3 structure with specific learning objectives and code examples
4. **Finalize writing timeline**: Create month-by-month Gantt chart with dependencies (e.g., Chapter 6 code must be done before Chapter 7 writing starts)
5. **Set up monorepo**: Initialize Git repo with `book/`, `code/`, `website/`, `instructor-resources/` structure
6. **Prototype first chapter**: Write Chapter 1 (Introduction) to validate writing workflow and chapter template
7. **Set up CI/CD**: Configure GitHub Actions for first companion repo (Chapter 3) to validate testing infrastructure
8. **Recruit external testers**: Find 3-5 volunteers willing to test early chapter labs

---

## Risks Requiring Immediate Attention

From spec.md Risks and Mitigations:

### Critical Path Risks
1. **Risk 1 (Rapid Software Deprecation)**:
   - **Immediate action**: Establish contact with NVIDIA Isaac Sim team for early beta access and deprecation notices
   - **Immediate action**: Set up weekly CI testing against both current (Iron + Isaac Sim 2024.2) and bleeding-edge (Jazzy + Isaac Sim 2025.x) versions

2. **Risk 2 (Hardware Unavailability)**:
   - **Immediate action**: Complete Research Task 4 (Edge Hardware Selection) to lock in Jetson tier before potential price changes
   - **Immediate action**: Document cloud alternatives in Chapter 2 quickstart to mitigate local GPU unavailability

3. **Risk 3 (VLA Model Availability)**:
   - **Immediate action**: Complete Research Task 2 (VLA Model Selection) and archive chosen model weights in institutional storage (with license compliance)
   - **Immediate action**: Prototype VLA integration with 2-3 alternative models to validate swappability

### Schedule Risks
4. **Risk 5 (Content Creep - 269k-307k words vs. 160k-190k target)**:
   - **Immediate action**: Implement word count mitigation strategy (reduce appendices to 30k-40k, tighten chapters by 10-15%)
   - **Immediate action**: Decide on 1-volume vs. 2-volume approach before starting Part 1 writing

5. **Risk 6 (Low Instructor Adoption)**:
   - **Immediate action**: Begin outreach to 20-30 robotics professors for early review copies and feedback
   - **Immediate action**: Create compelling instructor resources (slides, assignments) to incentivize adoption

---

## Files to Create Next

Based on `/sp.plan` command workflow (Phases 0-1):

1. **research.md**: Consolidate findings from Research Tasks 1-8 (Phase 0)
2. **decisions/humanoid-platform.md**: Document humanoid platform choice (Research Task 1)
3. **decisions/vla-model-choice.md**: Document VLA model choice (Research Task 2)
4. **decisions/simulation-engine.md**: Document simulation engine choice (Research Task 3)
5. **decisions/edge-hardware.md**: Document Jetson tier choice (Research Task 4)
6. **decisions/code-license.md**: Document code license choice (Research Task 5)
7. **book-architecture.md**: Complete H1 → H2 → H3 outline for all 21 chapters (Phase 1)
8. **word-count-budget.md**: Revised budget addressing overage issue (Phase 1)
9. **writing-timeline.md**: 12-14 month Gantt chart (Phase 1)
10. **testing-validation-strategy.md**: Complete testing and validation plan (Phase 1)

---

**Branch**: `002-humanoid-robotics-book`
**Plan Path**: `/mnt/d/Quarter-4/spec_kit_plus/robot_book/specs/002-humanoid-robotics-book/plan.md`
**Status**: Phase 0 (Research) ready to begin after user approval
