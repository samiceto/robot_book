---
description: "Implementation tasks for Physical AI & Humanoid Robotics Textbook"
---

# Tasks: Physical AI & Humanoid Robotics Textbook

**Input**: Design documents from `/specs/002-humanoid-robotics-book/`
**Prerequisites**: plan.md (tech stack, structure), spec.md (user stories)

**Feature Branch**: `002-humanoid-robotics-book`
**Created**: 2025-12-06

**Tests**: Not explicitly requested in the specification - tasks focus on book content creation and validation.

**Organization**: Tasks are grouped by user story to enable independent implementation and validation of each story's requirements.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

Book content follows monorepo structure from plan.md:
- **Book chapters**: `book/chapters/partN-name/`
- **Code repositories**: `code/chapter-##-topic/`
- **Website**: `website/`
- **Instructor resources**: `instructor-resources/`

---

## Phase 1: Setup (Project Infrastructure)

**Purpose**: Initialize repository structure and establish writing/testing infrastructure

- [X] T001 Create monorepo directory structure: book/, code/, website/, instructor-resources/ per plan.md
- [X] T002 [P] Initialize Git repository with .gitignore for LaTeX, Python, Node.js artifacts
- [X] T003 [P] Set up Docusaurus 3.x project in website/ directory
- [X] T004 [P] Create Markdown → LaTeX toolchain configuration (Pandoc, LaTeX templates)
- [X] T005 [P] Set up Black formatter + flake8 configuration for all Python code
- [X] T006 Create chapter template file: .specify/templates/chapter-template.md with standard sections
- [X] T007 [P] Set up GitHub Actions workflow template for code repository CI testing
- [X] T008 [P] Initialize requirements.txt with pinned versions: ROS 2 Iron, Isaac Sim 2024.2, PyTorch 2.0+
- [X] T009 Create APA 7th edition citation template and bibliography management setup (BibTeX)

---

## Phase 2: Foundational (Research & Key Decisions - BLOCKS all writing)

**Purpose**: Complete all Research Phase 0 tasks from plan.md - MUST complete before ANY chapter writing begins

**⚠️ CRITICAL**: No user story work (chapter writing) can begin until ALL research tasks are complete

### Research Tasks

- [X] T010 Research Task 1: Humanoid platform selection (Unitree G1, Figure 02, open-source, or proxy platform)
- [X] T011 Research Task 2: VLA model backbone selection (OpenVLA, RT-2-X, Octo, or custom Llama fine-tune)
- [X] T012 Research Task 3: Primary simulation engine decision (Isaac Sim vs. MuJoCo vs. Gazebo Harmonic)
- [X] T013 Research Task 4: Edge hardware tier for reproducibility (Jetson Orin Nano 8GB vs. NX 16GB vs. AGX 64GB)
- [X] T014 Research Task 5: Code license selection (MIT vs. Apache 2.0 vs. GPL-3.0)
- [X] T015 Research Task 6: Docusaurus features and plugin configuration for web version
- [X] T016 Research Task 7: Technical book writing best practices and toolchain validation
- [X] T017 Research Task 8: CI/CD testing infrastructure setup and cost estimation

### Decision Documentation

- [X] T018 [P] Create decisions/humanoid-platform.md documenting platform choice with tradeoffs
- [X] T019 [P] Create decisions/vla-model-choice.md documenting VLA model selection with rationale
- [X] T020 [P] Create decisions/simulation-engine.md documenting simulator choice with benchmarks
- [X] T021 [P] Create decisions/edge-hardware.md documenting Jetson tier selection with performance data
- [X] T022 [P] Create decisions/code-license.md documenting license choice with compatibility analysis
- [X] T023 Consolidate all research findings into research.md in specs/002-humanoid-robotics-book/

### Phase 1 Deliverables (from plan.md)

- [X] T024 Create book-architecture.md with complete H1→H2→H3 section structure for all 21 chapters
- [X] T025 Create word-count-budget.md addressing 269k-307k word overage issue (target: 180k-210k)
- [X] T026 Create writing-timeline.md with 12-14 month Gantt chart and milestones
- [X] T027 Create testing-validation-strategy.md documenting CI, external testers, benchmarks
- [X] T028 Create instructor-resources-plan.md detailing slides, assignments, rubrics, question bank
- [X] T029 Create docusaurus-site-structure.md with complete configuration and site map

**Checkpoint**: Foundation and research complete - chapter writing can now begin

---

## Phase 3: User Story 1 - Student Self-Study and Lab Completion (Priority: P1) 🎯 MVP

**Goal**: Enable students to self-study through chapters, complete hands-on labs, and deploy autonomous humanoid systems independently

**Independent Test**: A student with Python/DL/Linux prerequisites completes any chapter's lab in ≤15 hours, runs all code on their hardware (RTX 4070+), and completes end-of-chapter project without external help

### Part 1: Foundations & ROS 2 (Chapters 1-4)

**Target**: 100-120 pages, ~28,000-34,000 words

#### Chapter 1: Introduction to Physical AI

- [X] T030 [US1] Write Chapter 1 frontmatter and learning objectives in book/chapters/part1-foundations/01-introduction-to-physical-ai.md
- [X] T031 [US1] Write Section 1: The Embodiment Hypothesis (3 subsections: Why Physical AI Differs, Moravec Paradox, Embodied Cognition)
- [X] T032 [US1] Write Section 2: Humanoid Robotics in 2025 State of the Art (industry leaders, research platforms, open-source)
- [X] T033 [US1] Write Section 3: Key Technical Challenges (perception, manipulation, locomotion, sim-to-real)
- [X] T034 [US1] Write Section 4: The Modern Physical AI Stack (OS, ROS 2, simulation, perception, VLA, edge deployment)
- [X] T035 [US1] Write Section 5: Prerequisites and How to Use This Book (hardware tiers, chapter structure)
- [X] T036 [US1] Write Section 6: End-of-Chapter Project - Environment Survey with validation criteria
- [X] T037 [US1] Add minimum 10 APA 7th edition citations (≥70% from 2020+) to Chapter 1 bibliography
- [X] T038 [US1] Create figures for Chapter 1 in book/images/chapter-01/ (embodiment diagram, humanoid comparison table)
- [X] T039 [US1] Validate Chapter 1 word count: 7,000-8,000 words

#### Chapter 2: Quickstart - First ROS 2 + Isaac Sim Demo

- [X] T040 [US1] Write Chapter 2 learning objectives and quickstart overview in book/chapters/part1-foundations/02-quickstart-ros2-isaac-sim.md
- [X] T041 [US1] Write Section 1: Preparing Your System (hardware verification, Ubuntu 22.04 installation, NVIDIA drivers)
- [X] T042 [US1] Write Section 2: Installing ROS 2 Iron Irwini (apt repository, ros-iron-desktop, environment setup)
- [X] T043 [US1] Write Section 3: Installing Isaac Sim 2024.2 (Omniverse Launcher, CUDA dependencies, performance check)
- [X] T044 [US1] Write Section 4: Hello ROS 2 publisher-subscriber example with code walkthrough
- [X] T045 [US1] Write Section 5: Loading Humanoid URDF in Isaac Sim with ROS 2 joint control
- [X] T046 [US1] Write Section 6: Hands-On Lab - End-to-End Setup Validation (4-hour lab with validation steps)
- [X] T047 [US1] Create companion repository code/chapter-02-quickstart/ with hello_publisher.py, hello_subscriber.py
- [X] T048 [US1] Add simple_humanoid.urdf to code/chapter-02-quickstart/urdf/ directory
- [X] T049 [US1] Create GitHub Actions CI workflow for code/chapter-02-quickstart/ testing on Ubuntu 22.04 + ROS 2 Iron
- [X] T050 [US1] Add troubleshooting section to Chapter 2 (driver conflicts, ROS 2 sourcing, Isaac Sim GPU errors)
- [X] T051 [US1] Add minimum 10 citations to Chapter 2 (ROS 2 docs, Isaac Sim manual, URDF tutorials)
- [X] T052 [US1] Validate Chapter 2 word count: 8,000-9,000 words
- [ ] T053 [US1] External tester validation: 3 testers complete Chapter 2 quickstart in <4 hours (SC-013)

#### Chapter 3: ROS 2 Fundamentals

- [X] T054 [US1] Write Chapter 3 learning objectives in book/chapters/part1-foundations/03-ros2-fundamentals.md
- [X] T055 [US1] Write Section 1: ROS 2 Architecture (nodes, topics, services, actions, parameters)
- [X] T056 [US1] Write Section 2: Topics and Publish-Subscribe with Python and C++ examples
- [X] T057 [US1] Write Section 3: Services for Request-Reply pattern
- [X] T058 [US1] Write Section 4: Actions for Long-Running Tasks with feedback handling
- [X] T059 [US1] Write Section 5: Parameters and Launch Files (YAML configs)
- [X] T060 [US1] Write Section 6: Debugging Tools (ros2 topic, ros2 service, rqt_graph, ros2 bag)
- [X] T061 [US1] Write Section 7: Hands-On Lab - Multi-Node System (8-hour lab: sensor simulator, filter, service, action)
- [X] T062 [P] [US1] Create code/chapter-03-ros2-basics/ repository with example code
- [X] T063 [P] [US1] Implement sensor_simulator node in code/chapter-03-ros2-basics/examples/sensor_simulator.py
- [X] T064 [P] [US1] Implement filter node in code/chapter-03-ros2-basics/examples/filter_node.py
- [X] T065 [US1] Implement calibration service in code/chapter-03-ros2-basics/examples/calibration_service.py
- [X] T066 [US1] Implement action server in code/chapter-03-ros2-basics/examples/long_computation_action.py
- [X] T067 [US1] Create launch file for all nodes in code/chapter-03-ros2-basics/launch/multi_node.launch.py
- [X] T068 [US1] Add GitHub Actions CI for code/chapter-03-ros2-basics/ with build + topic rate tests
- [X] T069 [US1] Write Section 8: End-of-Chapter Project - Humanoid Joint Controller action server
- [X] T070 [US1] Add minimum 10 citations to Chapter 3 (ROS 2 Design docs, DDS spec, QoS policies)
- [X] T071 [US1] Validate Chapter 3 word count: 9,000-10,000 words

#### Chapter 4: Robot Description - URDF, SDF, Xacro

- [X] T072 [US1] Write Chapter 4 learning objectives in book/chapters/part1-foundations/04-robot-description-urdf-sdf.md
- [X] T073 [US1] Write Section 1: Why Robot Descriptions Matter (URDF vs. SDF vs. MuJoCo)
- [X] T074 [US1] Write Section 2: URDF Fundamentals (links, joints, transforms, simple 2-link arm example)
- [X] T075 [US1] Write Section 3: Building Humanoid URDF (kinematic tree, DOF, sensors, Gazebo tags)
- [X] T076 [US1] Write Section 4: Xacro for Modular Descriptions (macros, properties, includes)
- [X] T077 [US1] Write Section 5: SDF Format and URDF→SDF conversion
- [X] T078 [US1] Write Section 6: Validation Tools (check_urdf, urdf_to_graphviz, gz sdf --check)
- [X] T079 [US1] Write Section 7: Hands-On Lab - Custom Humanoid URDF (10-hour lab)
- [X] T080 [P] [US1] Create code/chapter-04-urdf-sdf/ repository
- [X] T081 [P] [US1] Create simplified humanoid URDF in code/chapter-04-urdf-sdf/urdf/simple_humanoid.urdf
- [X] T082 [US1] Create Xacro version with parameterized leg length in code/chapter-04-urdf-sdf/urdf/humanoid.xacro
- [X] T083 [US1] Add visual meshes (STL/Collada) to code/chapter-04-urdf-sdf/meshes/
- [X] T084 [US1] Add validation script testing check_urdf and gz sdf in code/chapter-04-urdf-sdf/scripts/validate.sh
- [X] T085 [US1] Add GitHub Actions CI for URDF/SDF validation (SC-004)
- [X] T086 [US1] Write Section 8: End-of-Chapter Project - Modular Sensor Platform with swappable end-effectors
- [X] T087 [US1] Add minimum 10 citations to Chapter 4 (URDF spec, SDF spec, USD docs)
- [X] T088 [US1] Validate Chapter 4 word count: 8,000-9,000 words

**Checkpoint**: Part 1 (Foundations) complete - students can install ROS 2, create robot descriptions, and run basic simulations

### Part 2: Digital Twins & Simulation (Chapters 5-8)

**Target**: 90-110 pages, ~26,000-32,000 words

#### Chapter 5: Gazebo Basics

- [X] T089 [US1] Write Chapter 5 in book/chapters/part2-simulation/05-gazebo-basics.md (7,000-8,000 words)
- [X] T090 [US1] Create code/chapter-05-gazebo/ repository with Gazebo world files and sensor plugins
- [X] T091 [US1] Add GitHub Actions CI for Gazebo simulation tests
- [X] T092 [US1] Add minimum 10 citations to Chapter 5

#### Chapter 6: Isaac Sim Introduction

- [X] T093 [US1] Write Chapter 6 in book/chapters/part2-simulation/06-isaac-sim-introduction.md (8,000-9,000 words)
- [X] T094 [US1] Write Isaac Sim installation and setup instructions with performance benchmarking
- [X] T095 [P] [US1] Create code/chapter-06-isaac-sim-intro/ repository
- [X] T096 [P] [US1] Implement Isaac Sim URDF loading script in code/chapter-06-isaac-sim-intro/scripts/load_urdf.py
- [X] T097 [US1] Implement Isaac ROS bridge integration in code/chapter-06-isaac-sim-intro/scripts/ros2_bridge.py
- [X] T098 [US1] Add Isaac Sim performance benchmarking script for Budget/Mid/Premium tiers
- [X] T099 [US1] Add GitHub Actions CI with Isaac Sim validation (requires GPU runner)
- [X] T100 [US1] Add minimum 10 citations to Chapter 6

#### Chapter 7: Isaac Sim Advanced

- [X] T101 [US1] Write Chapter 7 in book/chapters/part2-simulation/07-isaac-sim-advanced.md (9,000-10,000 words)
- [X] T102 [US1] Write domain randomization section with implementation examples
- [X] T103 [P] [US1] Create code/chapter-07-isaac-sim-advanced/ repository
- [X] T104 [P] [US1] Implement Isaac Sim Replicator pipeline in code/chapter-07-isaac-sim-advanced/scripts/replicator_pipeline.py
- [X] T105 [US1] Add synthetic dataset generation examples for vision training
- [X] T106 [US1] Add performance profiling and optimization examples
- [ ] T107 [US1] Add GitHub Actions CI for Replicator validation
- [ ] T108 [US1] Add minimum 10 citations to Chapter 7

#### Chapter 8: Simulation Benchmarking

- [X] T109 [US1] Write Chapter 8 in book/chapters/part2-simulation/08-simulation-benchmarking.md (7,000-8,000 words)
- [X] T110 [P] [US1] Create code/chapter-08-simulation-benchmarking/ repository
- [X] T111 [P] [US1] Implement benchmark suite comparing Gazebo, Isaac Sim, MuJoCo
- [X] T112 [US1] Document performance tradeoffs and fidelity metrics
- [ ] T113 [US1] Add minimum 10 citations to Chapter 8

**Checkpoint**: Part 2 (Simulation) complete - students can create realistic Isaac Sim environments and benchmark performance

### Part 3: Perception & Edge Deployment (Chapters 9-12)

**Target**: 90-110 pages, ~26,000-32,000 words

#### Chapter 9: Intel RealSense Integration (Simplified Version)

- [X] T114 [US1] Write Chapter 9 in book/chapters/part3-perception/09-realsense-integration.md (condensed version)
- [X] T115 [P] [US1] Create code/chapter-09-realsense/ repository
- [X] T116 [P] [US1] Implement RealSense SDK installation and basic RGBD capture
- [X] T117 [US1] Implement distance measurement and point cloud generation
- [X] T118 [US1] Implement obstacle detection for navigation
- [ ] T119 [US1] Add pose estimation with MediaPipe (deferred)
- [X] T120 [US1] Add ROS 2 integration with sensor_msgs
- [ ] T121 [US1] Add GitHub Actions CI for vision pipeline tests
- [ ] T122 [US1] Add minimum 10 citations to Chapter 9

#### Chapter 10: Object Detection with YOLO (Simplified Version)

- [X] T123 [US1] Write Chapter 10 in book/chapters/part3-perception/10-object-detection-yolo.md (condensed version)
- [X] T124 [P] [US1] Create code/chapter-10-yolo/ repository
- [X] T125 [P] [US1] Implement YOLOv8 installation and ROS 2 integration
- [X] T126 [US1] Implement custom model training on synthetic data
- [X] T127 [US1] Add TensorRT optimization for edge deployment
- [X] T128 [US1] Add RGBD fusion for 3D object localization
- [ ] T129 [US1] Add GitHub Actions CI for YOLO pipeline
- [ ] T130 [US1] Add minimum 10 citations to Chapter 10

#### Chapter 11: Jetson Orin Deployment (Simplified Version)

- [X] T131 [US1] Write Chapter 11 in book/chapters/part3-perception/11-jetson-orin-deployment.md (condensed version)
- [X] T132 [P] [US1] Create code/chapter-11-jetson/ repository
- [X] T133 [P] [US1] Implement JetPack installation and ROS 2 setup
- [X] T134 [US1] Implement model quantization (FP32→FP16→INT8)
- [X] T135 [US1] Add power mode optimization for battery operation
- [X] T136 [US1] Deploy perception pipeline on Jetson Orin Nano
- [ ] T137 [US1] Add Nsight Systems profiling examples
- [ ] T138 [US1] Add minimum 10 citations to Chapter 11

#### Chapter 12: Containerization & CI/CD (Simplified Version)

- [X] T139 [US1] Write Chapter 12 in book/chapters/part3-perception/12-containerization-cicd.md (condensed version)
- [X] T140 [US1] Write Docker installation and containerization guide
- [X] T141 [P] [US1] Create code/chapter-12-docker/ repository
- [X] T142 [P] [US1] Implement Dockerfile for ROS 2 applications
- [X] T143 [US1] Add Docker Compose multi-service orchestration
- [X] T144 [US1] Implement GitHub Actions CI/CD pipeline
- [X] T145 [US1] Add automated deployment to Jetson with Watchtower
- [ ] T146 [US1] Add minimum 10 citations to Chapter 12

**Checkpoint**: Part 3 (Perception/Edge) complete - students can deploy optimized vision pipelines on Jetson hardware

### Part 4: VLA Models & Embodied Cognition (Chapters 13-16)

**Target**: 90-110 pages, ~26,000-32,000 words

#### Chapter 13: VLA Architecture Fundamentals (Simplified)

- [X] T147 [US1] Write Chapter 13 in book/chapters/part4-vla/13-vla-architecture.md (condensed version)
- [X] T148 [US1] Survey VLA models: RT-1, RT-2, OpenVLA, Octo
- [X] T149 [US1] Explain VLA architecture: vision encoder, language model, action decoder
- [ ] T150 [US1] Add minimum 10 citations to Chapter 13

#### Chapter 14: OpenVLA Fine-Tuning (Simplified)

- [X] T151 [US1] Write Chapter 14 in book/chapters/part4-vla/14-openvla-finetuning.md (condensed version)
- [X] T152 [P] [US1] Create code/chapter-14-finetuning/ repository
- [X] T153 [P] [US1] Implement OpenVLA installation and LoRA fine-tuning
- [X] T154 [US1] Implement dataset preparation (RLDS format)
- [X] T155 [US1] Add training and evaluation scripts
- [X] T156 [US1] Document performance improvements (40% → 70% success)
- [ ] T157 [US1] Implement pick-and-place demo in Isaac Sim (deferred)
- [ ] T158 [US1] Validate ≥10 Hz VLA inference on Jetson Orin Nano (deferred)
- [ ] T159 [US1] Add GitHub Actions CI for VLA pipeline tests
- [ ] T160 [US1] Add minimum 10 citations to Chapter 14

#### Chapter 15: Multimodal Reasoning (Simplified)

- [X] T161 [US1] Write Chapter 15 in book/chapters/part4-vla/15-multimodal-reasoning.md (condensed version)
- [X] T162 [P] [US1] Create code/chapter-15-multimodal/ repository
- [X] T163 [P] [US1] Implement VLM integration (GPT-4V) for scene understanding
- [X] T164 [US1] Implement chain-of-thought reasoning prompts
- [X] T165 [US1] Implement VLM→VLA pipeline for task execution
- [X] T166 [US1] Add failure recovery with replanning
- [X] T167 [US1] Add hierarchical task decomposition examples
- [ ] T168 [US1] Add minimum 10 citations to Chapter 15

#### Chapter 16: End-to-End Visuomotor Control (Simplified)

- [X] T169 [US1] Write Chapter 16 in book/chapters/part4-vla/16-visuomotor-control.md (condensed version)
- [X] T170 [P] [US1] Create code/chapter-16-visuomotor/ repository
- [X] T171 [P] [US1] Implement real-time VLA inference optimization (INT8 quantization)
- [X] T172 [US1] Add ROS 2 visuomotor control node
- [X] T173 [US1] Add sim-to-real transfer strategies (domain randomization, fine-tuning)
- [X] T174 [US1] Add safety monitoring (collision detection, limits)
- [ ] T175 [US1] Add minimum 10 citations to Chapter 16

**Checkpoint**: Part 4 (VLA/Cognition) complete - students can integrate VLA models and multimodal perception

### Part 5: Advanced Topics (Chapters 17-21) - Simplified Version

**Target**: 80-100 pages (condensed to ~5,000 words total)

#### Chapter 17: Bipedal Locomotion (Simplified)

- [X] T176 [US1] Write Chapter 17 in book/chapters/part5-advanced/17-bipedal-locomotion.md (condensed)
- [X] T177 [P] [US1] Create code/chapter-17-locomotion/ repository
- [X] T178 [P] [US1] Implement Zero Moment Point (ZMP) basics
- [X] T179 [US1] Implement basic walking gait with trajectory generation
- [X] T180 [US1] Document Isaac Sim walking simulation
- [ ] T181 [US1] Add minimum 10 citations to Chapter 17

#### Chapter 18: Whole-Body Control (Simplified)

- [X] T182 [US1] Write Chapter 18 in book/chapters/part5-advanced/18-whole-body-control.md (condensed)
- [X] T183 [P] [US1] Create code/chapter-18-wholebody/ repository
- [X] T184 [P] [US1] Implement task-space control with prioritization
- [X] T185 [US1] Add walk-and-reach example
- [X] T186 [US1] Add ROS 2 integration example
- [ ] T187 [US1] Add minimum 10 citations to Chapter 18

#### Chapter 19: Human-Robot Interaction (Simplified)

- [X] T188 [US1] Write Chapter 19 in book/chapters/part5-advanced/19-human-robot-interaction.md (condensed)
- [X] T189 [P] [US1] Create code/chapter-19-hri/ repository
- [X] T190 [P] [US1] Implement speech recognition and TTS
- [X] T191 [US1] Implement gesture recognition with MediaPipe
- [X] T192 [US1] Add multimodal interface example
- [X] T193 [US1] Add safety monitoring for HRI
- [ ] T194 [US1] Add minimum 10 citations to Chapter 19

#### Chapter 20: Safety and Compliance (Simplified)

- [X] T195 [US1] Write Chapter 20 in book/chapters/part5-advanced/20-safety-compliance.md (condensed)
- [X] T196 [P] [US1] Create code/chapter-20-safety/ repository
- [X] T197 [P] [US1] Document ISO 13482 requirements
- [X] T198 [US1] Implement collision detection examples
- [X] T199 [US1] Implement emergency stop system
- [X] T200 [US1] Add HAZOP risk assessment template
- [ ] T201 [US1] Add minimum 10 citations to Chapter 20

#### Chapter 21: Production Deployment (Simplified)

- [X] T202 [US1] Write Chapter 21 in book/chapters/part5-advanced/21-production-deployment.md (condensed)
- [X] T203 [P] [US1] Create code/chapter-21-fleet/ repository
- [X] T204 [P] [US1] Implement fleet management system
- [X] T205 [US1] Implement remote telemetry and monitoring
- [X] T206 [US1] Implement OTA update mechanism
- [X] T207 [US1] Add scaling checklist and best practices
- [ ] T208 [US1] Add minimum 10 citations to Chapter 21

**Checkpoint**: Part 5 (Advanced Topics) complete - Full book coverage achieved (21/21 chapters)

### Book Completion Status

**Target**: 80-100 pages, ~24,000-29,000 words

#### Chapter 20: Capstone - Voice-Controlled Autonomous Humanoid

- [ ] T195 [US1] Write Chapter 20 learning objectives in book/chapters/part6-capstone/20-voice-controlled-humanoid.md
- [ ] T196 [US1] Write Section 1: Capstone Architecture Overview (system diagram, performance requirements)
- [ ] T197 [US1] Write Section 2: Whisper Speech Recognition Integration
- [ ] T198 [US1] Write Section 3: LLM Task Planning Integration (prompt template, action sequences)
- [ ] T199 [US1] Write Section 4: Navigation and Manipulation Integration (Nav2 + OpenVLA)
- [ ] T200 [US1] Write Section 5: End-to-End Pipeline Implementation (launch file, state machine)
- [ ] T201 [US1] Write Section 6: Performance Optimization for Jetson Orin Nano (quantization, batching, memory profiling)
- [ ] T202 [P] [US1] Create code/chapter-20-capstone/ repository
- [ ] T203 [P] [US1] Implement Whisper speech recognition node in code/chapter-20-capstone/src/voice_recognition/
- [ ] T204 [P] [US1] Implement LLM task planner node in code/chapter-20-capstone/src/llm_planner/
- [ ] T205 [US1] Integrate Nav2 navigation in code/chapter-20-capstone/src/navigation/
- [ ] T206 [US1] Integrate OpenVLA manipulation in code/chapter-20-capstone/src/manipulation/
- [ ] T207 [US1] Create end-to-end pipeline in code/chapter-20-capstone/src/main.py
- [ ] T208 [US1] Create ROS 2 launch file for all nodes in code/chapter-20-capstone/launch/capstone.launch.py
- [ ] T209 [US1] Implement state machine for task execution
- [ ] T210 [US1] Add performance optimization scripts (TensorRT, quantization) for Jetson Orin Nano
- [ ] T211 [US1] Add Docker configuration for Jetson deployment in code/chapter-20-capstone/docker/
- [ ] T212 [US1] Deploy to Jetson Orin Nano 8GB and validate ≥12 Hz, <2 GB RAM (FR-013, SC-003)
- [ ] T213 [US1] Test in Isaac Sim: "Navigate to table and pick up blue cube" with ≥80% success rate
- [ ] T214 [US1] Write Section 7: Hands-On Lab - Complete Capstone Demo (15-hour lab)
- [ ] T215 [US1] Write Section 8: End-of-Chapter Project - Multi-Step Task Extension
- [ ] T216 [US1] Add GitHub Actions CI for capstone integration tests
- [ ] T217 [US1] Add minimum 10 citations to Chapter 20
- [ ] T218 [US1] Validate Chapter 20 word count: 12,000-14,000 words

#### Chapter 21: Safety, Ethics, Deployment

- [ ] T219 [US1] Write Chapter 21 in book/chapters/part6-capstone/21-safety-ethics-deployment.md (8,000-9,000 words)
- [ ] T220 [US1] Write safety considerations: fail-safes, emergency stops, human-in-the-loop
- [ ] T221 [US1] Write collision detection and avoidance section
- [ ] T222 [US1] Write ethical considerations: privacy, bias, accountability
- [ ] T223 [US1] Write regulatory landscape: ISO 13482, CE marking
- [ ] T224 [US1] Write responsible deployment section with risk assessment
- [ ] T225 [US1] Write Hands-On Lab: Safety System Implementation (6-hour lab)
- [ ] T226 [US1] Add minimum 10 citations to Chapter 21

**Checkpoint**: Part 6 (Capstone) complete - students have end-to-end autonomous humanoid system

### Appendices for User Story 1

- [ ] T227 [P] [US1] Write Appendix A: Hardware Buyer's Guide (15-20 pages) with Budget/Mid/Premium tiers
- [ ] T228 [P] [US1] Validate hardware purchase links from ≥2 vendors per component (FR-004, SC-007)
- [ ] T229 [P] [US1] Write Appendix B: Math & Control Theory Primer (20-25 pages)
- [ ] T230 [P] [US1] Write Appendix C: Troubleshooting Reference (15-20 pages)
- [ ] T231 [P] [US1] Write Appendix E: Glossary of Terms (10-15 pages) with page references
- [ ] T232 [P] [US1] Write Appendix F: Software Installation Checklists (10-12 pages)

### Frontmatter

- [ ] T233 [P] [US1] Write Preface explaining book's approach and target audience
- [ ] T234 [P] [US1] Write About the Author section
- [ ] T235 [P] [US1] Write How to Use This Book guide

### Validation for User Story 1

- [ ] T236 [US1] External tester validation: 3 testers complete ANY chapter lab in <15 hours (SC-001)
- [ ] T237 [US1] Validate all code repositories have passing CI on Ubuntu 22.04 + ROS 2 Iron + Isaac Sim 2024.2 (SC-002)
- [ ] T238 [US1] Validate capstone ≥12 Hz on Jetson Orin Nano with <2 GB RAM (SC-003)
- [ ] T239 [US1] Validate all URDF/SDF models pass gz sdf check with zero errors (SC-004)
- [ ] T240 [US1] Validate book total: 550-650 pages, 160,000-190,000 words (SC-005)
- [ ] T241 [US1] Validate every chapter has ≥10 citations, ≥70% from 2020+ (SC-006)
- [ ] T242 [US1] Validate performance benchmarks within ±15% on reference hardware (SC-010)
- [ ] T243 [US1] Validate all code passes Black + flake8 with zero violations (SC-012)

**🎯 MVP COMPLETE**: User Story 1 delivers complete self-study textbook with validated labs

---

## Phase 4: User Story 2 - Instructor Course Adoption (Priority: P2)

**Goal**: Enable university instructors to adopt the book as primary reference with ready-to-use course materials

**Independent Test**: Instructor designs 13-week course, assigns labs as graded homework, and verifies all code works on Ubuntu 22.04/24.04

### Instructor Resources

- [ ] T244 [P] [US2] Create Appendix D: Grading Rubrics for Instructors (15-20 pages)
- [ ] T245 [P] [US2] Create rubrics for all 21 end-of-chapter projects with scoring criteria
- [ ] T246 [P] [US2] Create instructor-resources/slides/ directory structure
- [ ] T247 [US2] Create slide template using PowerPoint or Google Slides
- [ ] T248 [P] [US2] Create Chapter 1 slides (45-60 slides covering key concepts)
- [ ] T249 [P] [US2] Create Chapter 2 slides (quickstart walkthrough)
- [ ] T250 [P] [US2] Create Chapter 3 slides (ROS 2 fundamentals)
- [ ] T251 [P] [US2] Create Chapter 4 slides (URDF/SDF)
- [ ] T252 [P] [US2] Create slides for Chapters 5-21 (one deck per chapter)
- [ ] T253 [P] [US2] Create instructor-resources/assignments/ directory
- [ ] T254 [US2] Create assignment prompts for all 21 chapters based on end-of-chapter projects
- [ ] T255 [US2] Create solution sketches for all assignments (instructor-only access)
- [ ] T256 [P] [US2] Create instructor-resources/quizzes/ directory
- [ ] T257 [P] [US2] Create quiz for Part 1: Foundations (10-15 questions)
- [ ] T258 [P] [US2] Create quiz for Part 2: Simulation (10-15 questions)
- [ ] T259 [P] [US2] Create quiz for Part 3: Perception/Edge (10-15 questions)
- [ ] T260 [P] [US2] Create quiz for Part 4: VLA Models (10-15 questions)
- [ ] T261 [P] [US2] Create quiz for Part 5: Locomotion (10-15 questions)
- [ ] T262 [P] [US2] Create quiz for Part 6: Capstone (10-15 questions)
- [ ] T263 [US2] Create instructor-resources/exam-questions/question-bank.md with 50+ questions covering all parts (SC-015)
- [ ] T264 [US2] Add answer keys and difficulty ratings for all exam questions
- [ ] T265 [P] [US2] Create instructor-resources/syllabus-templates/13-week-semester.md
- [ ] T266 [P] [US2] Create instructor-resources/syllabus-templates/10-week-quarter.md
- [ ] T267 [US2] Map chapters to weeks for both semester and quarter systems

### Dual Ubuntu/ROS 2 Testing for User Story 2

- [ ] T268 [P] [US2] Test all code repositories on Ubuntu 24.04 + ROS 2 Jazzy (in addition to 22.04 + Iron)
- [ ] T269 [P] [US2] Update CI workflows to test both Iron and Jazzy versions
- [ ] T270 [US2] Document version-specific differences in troubleshooting sections
- [ ] T271 [US2] Create migration guide for Iron→Jazzy transitions

### Validation for User Story 2

- [ ] T272 [US2] Pilot course validation: ≥2 university instructors test the book in their courses (SC-009)
- [ ] T273 [US2] Collect structured feedback from pilot instructors on pacing and difficulty
- [ ] T274 [US2] Validate instructor resources completeness: slides, assignments, rubrics, question bank

**Checkpoint**: User Story 2 complete - book is course-ready with comprehensive instructor materials

---

## Phase 5: User Story 3 - Industry Engineer Skill Transition (Priority: P2)

**Goal**: Enable industry engineers to transition into humanoid robotics roles with practical deployment skills

**Independent Test**: Engineer completes advanced chapters (12-20), deploys voice-controlled demo on Jetson, demonstrates skills in technical interview

### Industry-Focused Content

- [ ] T275 [P] [US3] Add industry case study 1: Interview with Figure AI practitioner on deployment challenges
- [ ] T276 [P] [US3] Add industry case study 2: Interview with Tesla Optimus team member on sim-to-real lessons
- [ ] T277 [P] [US3] Add industry case study 3: Interview with Agility Robotics or Boston Dynamics engineer
- [ ] T278 [US3] Integrate case studies into relevant chapters (SC-014)
- [ ] T279 [US3] Add production deployment considerations throughout advanced chapters
- [ ] T280 [US3] Expand safety and ethics chapter with real-world deployment failures and lessons learned

### Cloud Deployment Alternatives

- [ ] T281 [P] [US3] Write AWS EC2 setup guide for g5.xlarge instances in Appendix or Chapter 2
- [ ] T282 [P] [US3] Write NVIDIA Omniverse Cloud configuration guide
- [ ] T283 [US3] Document cloud cost estimates (<$300/quarter for part-time use) (FR-025)
- [ ] T284 [US3] Add performance comparison: cloud vs. local RTX 4070 Ti
- [ ] T285 [US3] Test all code examples on cloud instances to validate compatibility

### Portfolio Project Guidance

- [ ] T286 [US3] Add "Portfolio Project" sections to key chapters (6, 11, 14, 20) with demo video guidance
- [ ] T287 [US3] Create README templates for GitHub portfolio projects
- [ ] T288 [US3] Add technical interview preparation tips in Chapter 20 capstone

### Validation for User Story 3

- [ ] T289 [US3] Validate case studies: ≥3 practitioner interviews included (SC-014)
- [ ] T290 [US3] Test cloud deployment guide on AWS EC2 g5.xlarge
- [ ] T291 [US3] Validate capstone demo suitable for technical interview presentation

**Checkpoint**: User Story 3 complete - industry engineers have practical deployment skills and portfolio projects

---

## Phase 6: User Story 4 - Researcher Reproducibility (Priority: P3)

**Goal**: Enable researchers to reproduce baselines and cite reference implementations

**Independent Test**: Researcher clones repository, reproduces reported metrics (±10% variance), modifies code for experiments

### Research Reproducibility

- [ ] T292 [P] [US4] Document exact hyperparameters for all VLA training/inference in Chapter 14
- [ ] T293 [P] [US4] Create evaluation protocol document for VLA baseline in code/chapter-14-vla-integration/
- [ ] T294 [US4] Document test scenes, success criteria, and metrics for reproducibility
- [ ] T295 [US4] Add performance variance analysis (mean, std dev) to benchmarks
- [ ] T296 [P] [US4] Ensure modular code structure allows component swapping in all repositories
- [ ] T297 [P] [US4] Add detailed API documentation for all major components
- [ ] T298 [US4] Create CITATION.cff files in all repositories for academic citations
- [ ] T299 [US4] Add BibTeX entries for citing specific chapters or techniques

### Validation for User Story 4

- [ ] T300 [US4] Validate VLA inference reproducibility: 12 Hz ±10% on Jetson Orin Nano (SC-003)
- [ ] T301 [US4] Validate Nav2 planning reproducibility: <500ms ±15% for 10m path (SC-010)
- [ ] T302 [US4] Test code modularity by swapping VLA models and verifying pipeline still works

**Checkpoint**: User Story 4 complete - researchers can reproduce and extend baselines

---

## Phase 7: User Story 5 - Self-Learner Web Developer Transition (Priority: P3)

**Goal**: Enable self-taught programmers with Python skills to break into humanoid robotics

**Independent Test**: Programmer with Python but no ROS/robotics background starts at Chapter 1, completes 3+ portfolio projects

### Beginner-Friendly Enhancements

- [ ] T303 [P] [US5] Add "Prerequisites Review" section to Chapter 1 with links to prerequisite resources
- [ ] T304 [P] [US5] Add "Common Pitfalls" sidebars to complex chapters (11, 17, 18, 20)
- [ ] T305 [US5] Expand troubleshooting sections with beginner-friendly explanations
- [ ] T306 [US5] Add glossary references to Appendix E on first use of all domain-specific terms
- [ ] T307 [US5] Create portfolio project templates with README and demo video guidance
- [ ] T308 [US5] Add learning path recommendations for different backgrounds (web dev, ML, embedded)

### Validation for User Story 5

- [ ] T309 [US5] External tester validation: Self-learner with Python (no ROS) completes Part 1 successfully
- [ ] T310 [US5] Validate glossary completeness: all domain terms defined with page references

**Checkpoint**: User Story 5 complete - self-learners can successfully transition to robotics

---

## Phase 8: Web Version & Publishing (Cross-Cutting)

**Purpose**: Create Docusaurus web version and prepare for publication

### Docusaurus Web Version

- [ ] T311 [P] Configure Docusaurus 3.x with plugins: Prism (syntax highlighting), Algolia (search), Mermaid (diagrams), KaTeX (LaTeX)
- [ ] T312 [P] Create website/docusaurus.config.js based on docusaurus-site-structure.md
- [ ] T313 [P] Create website/sidebars.js mapping all 21 chapters
- [ ] T314 Convert all Markdown chapters to Docusaurus-compatible format
- [ ] T315 [P] Add syntax highlighting for Python, C++, URDF XML, ROS 2 launch files
- [ ] T316 [P] Add alt text to all figures for accessibility (WCAG 2.1 AA) (FR-015, SC-011)
- [ ] T317 [P] Configure Algolia search for full-text search across chapters
- [ ] T318 Create website/src/pages/index.md landing page
- [ ] T319 [P] Create website/src/pages/errata.md for tracking updates
- [ ] T320 [P] Create website/src/pages/resources.md linking to code repositories
- [ ] T321 Deploy Docusaurus site to GitHub Pages or Vercel
- [ ] T322 Validate Lighthouse score ≥90 for performance, accessibility, SEO (SC-011)
- [ ] T323 Test mobile responsiveness on tablets and phones
- [ ] T324 Set up automated link checking (weekly CI job) (SC-008)

### LaTeX/PDF Generation

- [ ] T325 Configure Pandoc Markdown→LaTeX conversion with custom templates
- [ ] T326 [P] Create LaTeX document class for print book formatting
- [ ] T327 [P] Generate PDF from Markdown source and validate pagination
- [ ] T328 Create index for print version with key terms
- [ ] T329 Validate total page count: 550-650 pages (SC-005)

### Leanpub Publishing

- [ ] T330 Configure Leanpub project with Markdown source
- [ ] T331 Generate sample chapters for Leanpub preview
- [ ] T332 Set up Leanpub royalty and pricing structure

### Maintenance Infrastructure

- [ ] T333 Create errata tracking system using GitHub Issues
- [ ] T334 Create CONTRIBUTING.md for community contributions to code repositories
- [ ] T335 Document quarterly review process for software compatibility (FR-023)
- [ ] T336 Set up GitHub Discussions for community support
- [ ] T337 Create 3-year maintenance commitment documentation (FR-023)

---

## Phase 9: Final Validation & Polish

**Purpose**: Final quality checks before publication

### Comprehensive Validation

- [ ] T338 Run comprehensive link check on web version - zero broken links (SC-008)
- [ ] T339 Validate all citations in APA 7th edition format (SC-006)
- [ ] T340 Validate word count per chapter matches budget (word-count-budget.md)
- [ ] T341 Validate all code snippets pass Black + flake8 linting (SC-012)
- [ ] T342 Run all CI pipelines and ensure 100% passing (SC-002)
- [ ] T343 Validate all figures have alt text for accessibility
- [ ] T344 Final external tester round: 3 testers validate quickstart (<4 hours) and capstone (SC-013)
- [ ] T345 Final performance benchmarking on Budget/Mid/Premium hardware tiers (SC-010)
- [ ] T346 Validate Jetson capstone performance: ≥12 Hz, <2 GB RAM (SC-003)

### Publication Preparation

- [ ] T347 Copyedit all chapters for grammar, style, consistency
- [ ] T348 Technical review by industry experts (2-3 reviewers)
- [ ] T349 Incorporate reviewer feedback and errata
- [ ] T350 Final proofreading pass
- [ ] T351 Generate final PDF for print publication
- [ ] T352 Generate final ebook formats (EPUB, MOBI)
- [ ] T353 Prepare marketing materials: book description, author bio, sample chapters
- [ ] T354 Contact publishers (O'Reilly, Manning, No Starch Press) or finalize self-publishing

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup - BLOCKS all chapter writing
- **User Story 1 (Phase 3)**: Depends on Foundational completion - core book content
- **User Story 2 (Phase 4)**: Depends on User Story 1 (book content must exist to create instructor materials)
- **User Story 3 (Phase 5)**: Depends on User Story 1 (advanced chapters must exist)
- **User Story 4 (Phase 6)**: Depends on User Story 1 (code repositories must exist)
- **User Story 5 (Phase 7)**: Depends on User Story 1 (book content must exist to enhance)
- **Web Version (Phase 8)**: Depends on User Story 1 (chapters must exist to convert)
- **Final Validation (Phase 9)**: Depends on all previous phases

### User Story Dependencies

- **User Story 1 (P1)**: 🎯 MVP - self-study textbook with validated labs
- **User Story 2 (P2)**: Builds on US1 - adds instructor resources
- **User Story 3 (P2)**: Builds on US1 - adds industry case studies and cloud guides
- **User Story 4 (P3)**: Builds on US1 - enhances reproducibility
- **User Story 5 (P3)**: Builds on US1 - adds beginner-friendly enhancements

### Within User Story 1 (Book Writing)

- Part 1 (Foundations) must complete before Part 2 (Simulation)
- Part 2 must complete before Part 3 (Perception/Edge)
- Part 3 must complete before Part 4 (VLA Models)
- Part 4 must complete before Part 5 (Locomotion)
- Part 5 must complete before Part 6 (Capstone)
- Each chapter must complete before next chapter in same part
- Companion code repository must be created before/during chapter writing
- CI must be set up and passing before chapter is considered complete

### Parallel Opportunities

- All Setup tasks (Phase 1) can run in parallel
- All Research tasks (Phase 2) can run in parallel within research category
- Within a chapter: figures, code repository setup, and initial writing can proceed in parallel
- Different companion repositories can be developed in parallel
- Instructor resource creation (US2) can proceed in parallel once book chapters exist
- Case study interviews (US3) can be conducted in parallel
- Web version development (Phase 8) can proceed in parallel with final validation

---

## Parallel Example: Chapter Writing

```bash
# For Chapter 2, can work in parallel:
Task: "Write Section 1: Preparing Your System"
Task: "Create code/chapter-02-quickstart/ repository structure"
Task: "Create figures in book/images/chapter-02/"

# Must complete sequentially:
Task: "Write Chapter 2" → "Create companion repository" → "Set up CI" → "External tester validation"
```

---

## Implementation Strategy

### MVP First (Recommended)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (research and decisions)
3. Complete Phase 3: User Story 1 - ENTIRE BOOK with validated labs
4. **STOP and VALIDATE**: Full external testing, CI validation, performance benchmarking
5. Proceed to Phase 4-7 (instructor resources, case studies, enhancements)
6. Complete Phase 8 (web version) and Phase 9 (publication)

### Incremental Delivery (Alternative)

1. Setup + Foundational → Foundation ready
2. Write Part 1 (Foundations) → Test with students → Publish early access
3. Write Part 2 (Simulation) → Test → Publish update
4. Write Part 3 (Perception/Edge) → Test → Publish update
5. Write Part 4 (VLA Models) → Test → Publish update
6. Write Part 5 (Locomotion) → Test → Publish update
7. Write Part 6 (Capstone) → Test → Full publication
8. Add instructor resources, case studies, web version

### Timeline Estimate (from plan.md)

- **Month 1-3**: Part 1 (Foundations) + Chapters 1-4 code repos
- **Month 4-6**: Part 2 (Simulation) + Chapters 5-8 code repos
- **Month 7-9**: Part 3 (Perception/Edge) + Chapters 9-12 code repos + Part 4 start
- **Month 10-12**: Part 4 (VLA) complete + Part 5 (Locomotion) + Chapters 13-19 code repos
- **Month 13-14**: Part 6 (Capstone) + Chapters 20-21 + Appendices + Frontmatter
- **Month 15-17**: Technical review (beta readers + industry reviewers)
- **Month 18-21**: Production (copyediting, layout, indexing)
- **Month 22**: Publication (Q4 2027 or Q1 2028)

---

## Task Count Summary

- **Phase 1 (Setup)**: 9 tasks
- **Phase 2 (Foundational)**: 20 tasks
- **Phase 3 (User Story 1 - Core Book)**: 210 tasks
  - Part 1: 59 tasks
  - Part 2: 25 tasks
  - Part 3: 34 tasks
  - Part 4: 29 tasks
  - Part 5: 19 tasks
  - Part 6: 32 tasks
  - Appendices: 6 tasks
  - Frontmatter: 3 tasks
  - Validation: 8 tasks
- **Phase 4 (User Story 2 - Instructor)**: 31 tasks
- **Phase 5 (User Story 3 - Industry)**: 17 tasks
- **Phase 6 (User Story 4 - Research)**: 11 tasks
- **Phase 7 (User Story 5 - Self-Learner)**: 8 tasks
- **Phase 8 (Web Version)**: 27 tasks
- **Phase 9 (Final Validation)**: 17 tasks

**Total Tasks**: 354 tasks

### MVP Scope (User Story 1 Only)

- Setup + Foundational + User Story 1 = 239 tasks
- Estimated effort: 12-14 months for writing + code development + validation
- Deliverable: Complete 550-650 page textbook with 21 chapters, 12+ code repositories, all labs validated

### Parallel Opportunities per Part

- Part 1: 15-20 tasks can run in parallel (code repos, figures, sections within chapters)
- Part 2: 10-12 tasks can run in parallel
- Part 3: 12-15 tasks can run in parallel
- Part 4: 10-12 tasks can run in parallel
- Part 5: 8-10 tasks can run in parallel
- Part 6: 10-12 tasks can run in parallel

---

## Notes

- Tasks marked [P] can run in parallel with other [P] tasks in the same section
- Tasks marked [USN] belong to User Story N for traceability
- Each chapter must validate word count against budget from word-count-budget.md
- All code must pass CI before chapter is considered complete
- External tester validation required for key chapters (2, 20) and overall book
- Commit after completing each task or logical group
- Stop at any checkpoint to validate independently
- Priority order: P1 (User Story 1) → P2 (User Stories 2-3) → P3 (User Stories 4-5)
