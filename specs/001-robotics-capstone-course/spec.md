# Feature Specification: Physical AI & Humanoid Robotics Capstone Course

**Feature Branch**: `001-robotics-capstone-course`
**Created**: 2025-12-06
**Status**: Draft
**Input**: User description: "Complete Specification for the Physical AI & Humanoid Robotics Capstone Course - Target audience: University department heads, lab directors, and bootcamp founders who want to launch a cutting-edge, industry-aligned Physical AI / Humanoid Robotics course in 2026. Focus: Deliver a fully executable 13-week capstone course that takes students from zero robotics experience to deploying a voice-controlled autonomous humanoid (simulated + optional real hardware). Heavy emphasis on the modern 2025–2026 embodied-AI stack: ROS 2 Humble/Iron, NVIDIA Isaac Sim + Isaac ROS, Nav2, Vision-Language-Action models, Jetson Orin edge deployment."

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Course Launch and Setup (Priority: P1)

An instructor at a university or bootcamp receives the course specification package and needs to launch the Physical AI & Humanoid Robotics capstone course in Q1 2026. They must set up the course infrastructure, understand the curriculum structure, prepare materials, and ensure students can access required tools within the first week.

**Why this priority**: Without a complete, ready-to-execute package and clear setup instructions, the course cannot launch. This is the foundational requirement that enables all other user journeys.

**Independent Test**: Can be fully tested by providing the specification package to an instructor who has never taught robotics and verifying they can successfully launch the course, configure the environment options (Budget/Mid/Premium hardware tiers or cloud fallback), and conduct Week 1 activities without external support.

**Acceptance Scenarios**:

1. **Given** an instructor downloads the course specification package, **When** they review the documentation, **Then** they find complete week-by-week syllabus, slide deck outlines, assignment prompts, grading rubrics, and setup tutorials with no missing materials
2. **Given** an instructor needs to set up lab infrastructure, **When** they review the hardware specifications, **Then** they find three fully costed lab tiers (Budget <$1k, Mid $3–5k, Premium $15k+) with exact part numbers, purchase links valid in 2025, and compatibility verified for Ubuntu 22.04 LTS
3. **Given** students have limited budgets, **When** the instructor reviews cloud options, **Then** they find a complete cloud-native fallback path (AWS/NVIDIA Omniverse) with total student cost under $300/quarter and setup instructions
4. **Given** an instructor is setting up Week 1, **When** they prepare environment setup materials, **Then** they can guide students through ROS 2 Humble/Iron installation on Ubuntu 22.04 LTS with verification steps

---

### User Story 2 - Student Progression Through Core Curriculum (Priority: P2)

A student with Python and basic deep learning knowledge but no prior ROS or robotics experience enrolls in the capstone course. They need to progress through 13 weeks of structured learning, completing weekly assignments (12–15 hours/week), building skills incrementally from ROS 2 basics to deploying vision-language-action models on simulated humanoids in NVIDIA Isaac Sim.

**Why this priority**: The curriculum structure and progressive skill-building is the core educational value. Students must successfully move from zero robotics knowledge to autonomous humanoid deployment capability.

**Independent Test**: Can be tested by enrolling a student matching the prerequisite profile and tracking their weekly progress through assignments, verifying they can complete each week's deliverables within the time budget, and validating skill acquisition through graded rubrics.

**Acceptance Scenarios**:

1. **Given** a student completes Week 1 environment setup, **When** they attempt Week 2 ROS 2 fundamentals assignment, **Then** they successfully create and run basic ROS 2 nodes, publish/subscribe to topics, and complete the assignment within 12–15 hours
2. **Given** a student reaches Week 5 Navigation and Nav2 module, **When** they work on the navigation assignment, **Then** they successfully configure Nav2 for a simulated humanoid in Isaac Sim and demonstrate autonomous waypoint navigation
3. **Given** a student completes Week 8 Vision-Language-Action models module, **When** they integrate a VLA model, **Then** they successfully process visual input and generate action sequences for manipulation tasks in simulation
4. **Given** a student finishes all 13 weeks, **When** they review their skill progression, **Then** they can demonstrate competency in ROS 2, Isaac Sim, Nav2, VLA integration, and edge deployment concepts

---

### User Story 3 - Capstone Project End-to-End Deployment (Priority: P3)

A student in Week 12–13 needs to complete the capstone demonstration: deploying a voice-controlled autonomous humanoid system that takes a spoken command, processes it through Whisper speech recognition, uses an LLM planner to generate a task sequence, executes navigation and manipulation actions via ROS 2, and runs the complete pipeline in NVIDIA Isaac Sim. Advanced students may optionally deploy to physical Jetson Orin hardware.

**Why this priority**: This demonstrates the culmination of all learned skills and validates the course's success criteria. It proves students can build production-ready embodied AI systems.

**Independent Test**: Can be tested by evaluating the student's final capstone submission against the grading rubric, verifying the system runs end-to-end (voice input → LLM planning → ROS 2 execution → sim/hardware action), meets performance requirements (≥15 Hz inference on Jetson Orin Nano/NX 8–16 GB if hardware deployed), and completes assigned manipulation and navigation tasks.

**Acceptance Scenarios**:

1. **Given** a student has completed Weeks 1–11, **When** they begin the capstone project in Week 12, **Then** they receive clear project specifications, grading rubrics, example workflows, and support resources for integration
2. **Given** a student implements voice command processing, **When** they speak a command like "Navigate to the kitchen and pick up the red mug", **Then** Whisper transcribes the audio, the LLM planner generates a valid ROS 2 action sequence, and the system begins execution in Isaac Sim
3. **Given** the LLM planner generates navigation and manipulation actions, **When** the ROS 2 system executes them, **Then** the simulated humanoid successfully navigates to the target location and performs the manipulation task without collision
4. **Given** a student completes simulation testing, **When** they attempt optional Jetson Orin deployment, **Then** they successfully flash the Jetson Orin Nano/NX, deploy the inference pipeline, and achieve ≥15 Hz real-time inference with provided optimization guides
5. **Given** the instructor evaluates capstone submissions, **When** they apply the grading rubric, **Then** they can objectively score projects on completeness, performance, code quality, documentation, and demo video clarity

---

### User Story 4 - Instructor Continuous Support and Assessment (Priority: P2)

An instructor teaching the course needs to deliver weekly lectures, support students during lab sessions, grade 13+ assignments using provided rubrics, track student progress, identify struggling students early, and adjust pacing or provide additional resources as needed throughout the 13-week term.

**Why this priority**: Effective instruction and assessment are critical to student success. Clear grading rubrics, slide decks, and support materials reduce instructor burden and ensure consistency.

**Independent Test**: Can be tested by having an instructor conduct the course for one term, use provided slide decks and rubrics to grade assignments, and verify they can identify struggling students by Week 4, provide targeted support, and complete all assessments within reasonable time.

**Acceptance Scenarios**:

1. **Given** an instructor prepares for Week 1 lecture, **When** they review the provided slide deck outline, **Then** they find complete talking points, code examples, and visual aids for ROS 2 environment setup and core concepts
2. **Given** students submit Week 3 assignment (Isaac Sim basics), **When** the instructor applies the grading rubric, **Then** they can objectively evaluate submissions on setup completion, basic simulation tasks, and documentation quality within 15 minutes per student
3. **Given** an instructor reviews student progress at Week 7 midpoint, **When** they analyze assignment scores and submission patterns, **Then** they can identify students struggling with Nav2 or VLA concepts and provide targeted resources or office hours
4. **Given** an instructor grades the final capstone projects, **When** they use the comprehensive capstone rubric, **Then** they consistently score projects across technical implementation (40%), system integration (25%), performance (20%), and presentation (15%)

---

### User Story 5 - Hardware Procurement and Lab Setup (Priority: P3)

A lab director or bootcamp founder needs to procure hardware for the course, choosing between Budget (<$1k), Mid ($3–5k), or Premium ($15k+) lab tiers based on cohort size and budget. They must order parts, verify compatibility, set up lab workstations with Ubuntu 22.04 LTS and required GPU capabilities, and ensure students can access hardware by Week 2.

**Why this priority**: Physical infrastructure is necessary but follows initial course setup. Schools can defer hardware procurement if using cloud-only fallback, making this lower priority than core curriculum.

**Independent Test**: Can be tested by providing procurement specifications to a lab director, having them order parts from provided links, assemble workstations, and verify students can run Isaac Sim at acceptable frame rates (30+ FPS for Budget tier with RTX 4070 Ti, 60+ FPS for Mid/Premium tiers).

**Acceptance Scenarios**:

1. **Given** a lab director reviews Budget tier specifications, **When** they follow the procurement guide, **Then** they can order all parts (RTX 4070 Ti GPU, compatible motherboard, RAM, storage, peripherals) from valid 2025 links and stay under $1k total per workstation
2. **Given** a bootcamp founder has $20k budget for 4 workstations, **When** they select the Premium tier, **Then** they can procure RTX 4090-based systems with Jetson Orin NX dev kits and optional physical humanoid components (e.g., robotic arms) within budget
3. **Given** a lab technician assembles workstations, **When** they install Ubuntu 22.04 LTS, NVIDIA drivers, ROS 2 Humble, and Isaac Sim following setup guides, **Then** all systems boot correctly, pass GPU verification tests, and can run demo Isaac Sim scenes
4. **Given** students access lab workstations in Week 2, **When** they launch Isaac Sim for the first assignment, **Then** they achieve the required performance (≥30 FPS for Budget, ≥60 FPS for Mid/Premium tiers) without crashes or driver issues

---

### Edge Cases

- What happens when a student's personal hardware does not meet GPU requirements (RTX 4070 Ti minimum) and cloud budget exceeds $300/quarter?
- How does the course handle students who fall behind by Week 5 or later and need to catch up on foundational ROS 2 or Isaac Sim skills?
- What happens if NVIDIA discontinues Isaac Sim support or changes licensing terms mid-course?
- How does the instructor adapt if a critical dependency (e.g., Nav2, Isaac ROS packages) has breaking changes between course preparation and delivery?
- What happens when students attempt Jetson Orin deployment but encounter hardware failures or compatibility issues with specific sensors/actuators?
- How does the course scale if enrollment exceeds lab capacity (e.g., 50+ students but only 10 workstations)?
- What happens if a student completes the capstone early and wants advanced challenges or research project extensions?
- What happens if a student has never used Git before and cannot complete the Week 1 Git tutorial within 2-3 hours?
- How does the instructor handle GitHub Classroom technical issues (organization access denied, autograding failures, repository fork limits)?
- What happens if an institution cannot use GitHub Classroom due to data privacy policies or firewall restrictions?
- How does the course handle students who accidentally commit large files (50+ GB Isaac Sim cache) to their repositories and exceed GitHub storage limits?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: The course specification package MUST include a complete 13-week syllabus with week-by-week learning objectives, topics, lecture hours, lab hours, and expected student workload (12–15 hours/week total)
- **FR-002**: The package MUST provide detailed slide deck outlines for all 13 weeks with 5-10 structured bullet points per slide covering key concepts, talking points provided as separate instructor notes, code examples provided in companion files (not embedded in slides), visual aids specified by type and source (diagrams, architecture charts, demo screenshots), and demo scripts aligned with weekly topics. Instructors should expect 4-6 hours of preparation time per week to create presentation slides from these outlines
- **FR-003**: The package MUST include 13+ graded assignment prompts (minimum one per week) with clear instructions, starter code/templates where applicable, submission requirements via GitHub Classroom (students receive private repository forks, submit via Git push, graded via pull requests or automated GitHub Actions), and deadlines. Submission format includes ROS 2 packages, Python scripts, Jupyter notebooks for analysis, and demo videos for capstone milestones
- **FR-004**: The package MUST provide detailed grading rubrics for every assignment, including criteria, point allocation, example submissions (high/medium/low quality), and common grading pitfalls
- **FR-005**: The package MUST include complete setup tutorials for Ubuntu 22.04 LTS environment preparation, ROS 2 Humble/Iron installation, NVIDIA Isaac Sim installation, and verification steps
- **FR-006**: The package MUST specify three hardware lab tiers with exact part numbers, supplier links valid in 2025, and total cost breakdowns: Budget (<$1k), Mid ($3–5k), Premium ($15k+)
- **FR-007**: Each hardware tier specification MUST list GPU model, CPU, RAM, storage, motherboard compatibility, power supply requirements, cooling solutions, and peripherals
- **FR-008**: The package MUST provide a cloud-native fallback path using AWS EC2 GPU instances or NVIDIA Omniverse Cloud with setup instructions and cost estimation (target <$300/quarter per student)
- **FR-009**: Cloud setup instructions MUST include instance selection, OS configuration, remote desktop access, ROS 2 and Isaac Sim installation, and cost optimization strategies (spot instances, auto-shutdown)
- **FR-010**: The package MUST include capstone project specifications with end-to-end requirements: voice command input (Whisper), LLM planning, ROS 2 action execution, Isaac Sim simulation, and optional Jetson Orin deployment
- **FR-011**: Capstone specifications MUST define acceptance criteria: successful voice-to-action pipeline, autonomous navigation to target location, manipulation task completion, and performance metrics (≥15 Hz inference on Jetson Orin)
- **FR-012**: The package MUST provide comprehensive capstone grading rubric with scoring categories: technical implementation (40%), system integration (25%), performance/optimization (20%), documentation (10%), demo presentation (5%)
- **FR-013**: The package MUST include Jetson Orin deployment guides for Jetson Orin Nano and Jetson Orin NX (8 GB and 16 GB variants) with flashing instructions, ROS 2 setup, Isaac ROS installation, and optimization for ≥15 Hz real-time inference
- **FR-014**: All recommended software tools, libraries, and frameworks MUST be officially supported in 2026 with active community maintenance, stable package repositories, and documented upgrade paths
- **FR-015**: The package MUST verify that ROS 2 Humble or Iron, NVIDIA Isaac Sim, Isaac ROS, Nav2, and recommended VLA models have confirmed support through at least Q4 2026
- **FR-016**: The package MUST include sample VLA (Vision-Language-Action) model integration examples using open-source models (e.g., RT-1, RT-2, OpenVLA, or equivalent 2025–2026 alternatives) with inference code and ROS 2 bridges
- **FR-017**: The package MUST provide Nav2 configuration templates for humanoid navigation, including costmap parameters, planner selection (DWB, TEB, MPPI), recovery behaviors, and Isaac Sim integration
- **FR-018**: The package MUST include starter code repositories configured for GitHub Classroom distribution with templates for key assignments: ROS 2 pub/sub basics, Isaac Sim scene creation, Nav2 waypoint navigation, VLA model inference, Whisper integration, LLM planner, and full capstone pipeline. Each repository MUST include README with setup instructions, .gitignore templates (excluding Isaac Sim cache, compiled ROS binaries, large model weights >100MB), and automated testing workflows (GitHub Actions) for basic validation where applicable (ROS 2 package builds, Python linting). Students are expected to commit after each milestone with descriptive commit messages
- **FR-019**: The package MUST define prerequisite knowledge requirements: Python proficiency (data structures, OOP, async programming), basic deep learning (neural networks, training/inference, PyTorch or TensorFlow), and Linux command line familiarity
- **FR-020**: The package MUST provide a week-by-week topic progression from foundational to advanced: Week 1–2 ROS 2 basics, Week 3–4 Isaac Sim, Week 5–6 Nav2, Week 7–8 VLA models, Week 9–10 voice/LLM integration, Week 11 Jetson deployment, Week 12–13 capstone
- **FR-021**: The package MUST include troubleshooting guides for common issues: NVIDIA driver conflicts, ROS 2 package build failures, Isaac Sim GPU compatibility, Nav2 tuning, Jetson Orin deployment errors, and performance bottlenecks
- **FR-022**: The package MUST provide example capstone demo videos showing complete end-to-end execution: voice command → Whisper transcription → LLM planning → ROS 2 action sequence → Isaac Sim navigation and manipulation
- **FR-023**: The package MUST include instructor notes for each week with common student questions, difficult concepts that need extra emphasis, recommended pacing adjustments, and suggested office hour topics
- **FR-024**: The package MUST specify minimum GPU requirements for simulation: RTX 4070 Ti, RTX 4080, RTX 4090, or equivalent NVIDIA GPU with 12+ GB VRAM, verified to run Isaac Sim at ≥30 FPS for typical humanoid simulations
- **FR-025**: The package MUST include performance benchmarks for each hardware tier and cloud instance type, measuring Isaac Sim FPS, ROS 2 message throughput, VLA inference latency, and end-to-end pipeline latency
- **FR-026**: The package MUST provide safety and ethics guidelines for autonomous robotics development, including fail-safe behaviors, human-in-the-loop controls, simulation-to-reality gap awareness, and responsible AI deployment principles
- **FR-027**: The package MUST include assessment checkpoints at Week 4 (ROS 2 competency), Week 8 (VLA integration), and Week 13 (capstone) to identify struggling students early and provide intervention resources
- **FR-028**: The package MUST define learning objectives for each week using measurable outcomes (e.g., "Students will create ROS 2 packages with custom message types and services", "Students will configure Nav2 to navigate humanoid to specified waypoints")
- **FR-029**: The package MUST include optional extension projects for advanced students, such as multi-agent coordination, dynamic obstacle avoidance, advanced manipulation (dexterous hands), or sim-to-real transfer research
- **FR-030**: The package MUST provide TA (Teaching Assistant) guides with answer keys for assignments, common grading questions, student support escalation procedures, and lab session facilitation tips
- **FR-031**: The package MUST include GitHub Classroom setup instructions for instructors covering: organization creation, classroom roster import, starter repository configuration, autograding workflow setup, and student onboarding (Git installation, SSH key generation, first commit). Week 1 materials MUST include a 2-3 hour Git tutorial covering: basic workflow (clone, commit, push), branching concepts, .gitignore usage, and troubleshooting common Git errors
- **FR-032**: The package MUST provide grading rubric calibration materials including: 5-10 sample assignment submissions per assignment type (spanning quality levels from failing to exceptional), norming session guide for instructors/TAs to discuss divergent scores and establish grading standards, and rubric refinement feedback templates to capture improvements after grading the first cohort. Calibration exercises are required to achieve SC-012 inter-rater reliability ≥0.85

### Key Entities

- **Course Package**: The complete deliverable specification containing syllabus, slide decks, assignments, rubrics, setup guides, hardware specs, and all supporting materials required to launch the course
- **Weekly Module**: A single week's curriculum unit including learning objectives, lecture content outline, lab assignment, grading rubric, and estimated time budget (3–4 hours lecture, 8–11 hours lab/project work)
- **Hardware Lab Tier**: One of three costed hardware configurations (Budget <$1k, Mid $3–5k, Premium $15k+) with specific part numbers, performance benchmarks, and capability levels
- **Assignment**: A graded deliverable with prompt, starter code, submission requirements, grading rubric, and due date (13+ total assignments across the course)
- **Capstone Project**: The culminating Week 12–13 project requiring end-to-end voice-controlled autonomous humanoid system with Whisper, LLM planner, ROS 2, Nav2, VLA model, and Isaac Sim integration
- **Student Environment**: The technical setup required for students, either local hardware (GPU workstation with Ubuntu 22.04, ROS 2, Isaac Sim) or cloud instance (AWS/Omniverse) meeting performance requirements
- **Grading Rubric**: Scoring criteria for assignments and capstone, defining point allocation, quality levels, and evaluation guidelines for instructors
- **Setup Tutorial**: Step-by-step installation and configuration guide for software components (ROS 2, Isaac Sim, Isaac ROS, Nav2, Whisper, LLM integration) or hardware (Jetson Orin flashing, sensor setup)
- **Slide Deck Outline**: Lecture content structure for a weekly module, including key topics, code examples, visuals, demos, and talking points for instructors
- **Tool Specification**: Documentation for each recommended software tool/framework, including version requirements, support timeline through 2026, installation instructions, and compatibility notes

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: An instructor with no prior robotics teaching experience can launch the course in Q1 2026 within 2 weeks of receiving the specification package, without requiring external consultation or missing materials
- **SC-002**: 90% of students meeting prerequisites (Python, basic deep learning, Linux CLI) successfully complete all Week 1–11 assignments within the 12–15 hour/week time budget
- **SC-003**: 85% of students complete the capstone project by Week 13, demonstrating end-to-end voice-to-action pipeline (Whisper → LLM → ROS 2 → Isaac Sim navigation and manipulation)
- **SC-004**: 100% of recommended tools and frameworks (ROS 2 Humble/Iron, Isaac Sim, Isaac ROS, Nav2, Whisper, example VLA models) are officially supported and maintained through Q4 2026 as verified by vendor/community roadmaps
- **SC-005**: Students using Budget tier hardware (RTX 4070 Ti class) achieve ≥30 FPS in Isaac Sim for typical humanoid simulation scenes with navigation and manipulation tasks
- **SC-006**: Students using cloud fallback path (AWS/Omniverse) maintain total quarterly costs under $300 per student while meeting performance requirements (≥30 FPS Isaac Sim, <200ms VLA inference)
- **SC-007**: Students deploying to optional Jetson Orin hardware (Orin Nano/NX 8–16 GB) achieve ≥15 Hz real-time inference for the complete capstone pipeline (Whisper + LLM + Nav2 + VLA)
- **SC-008**: Instructors complete assignment grading for a 30-student cohort within 1 week of submission deadline using provided rubrics, averaging ≤20 minutes per assignment
- **SC-009**: Hardware procurement for any of the three lab tiers can be completed using provided part numbers and supplier links that remain valid and in-stock as of Q1 2025
- **SC-010**: 80% of students report high confidence (4/5 or 5/5 on end-of-course survey) in their ability to design and deploy autonomous humanoid systems after completing the course
- **SC-011**: Lab directors can assemble and configure workstations for any hardware tier within 2 days using provided setup tutorials, with all systems passing GPU and Isaac Sim verification tests
- **SC-012**: The capstone grading rubric demonstrates inter-rater reliability ≥0.85 when multiple TAs/instructors independently score the same capstone submission
- **SC-013**: Zero critical setup blockers reported by students in Week 1–2 when following environment setup tutorials (ROS 2, Isaac Sim, driver installation)
- **SC-014**: Average student capstone pipeline achieves <500ms end-to-end latency from voice command input to ROS 2 action initiation on recommended hardware
- **SC-015**: Troubleshooting guides resolve 90% of common technical issues (driver conflicts, build failures, Nav2 tuning, Jetson errors) without requiring instructor intervention

### Non-Functional Success Criteria

- **Usability**: Instructors with robotics domain expertise but no prior teaching experience can deliver effective lectures using provided slide deck outlines without extensive preparation
- **Accessibility**: The course specification package is provided in accessible Markdown format with clear headings, code blocks, tables, and navigation structure
- **Maintainability**: All software version numbers, package repositories, and installation commands are documented with upgrade paths for future ROS 2 and Isaac Sim releases
- **Scalability**: The course design supports cohorts ranging from 10 students (small bootcamp) to 100+ students (large university) with appropriate TA staffing recommendations
- **Vendor Neutrality**: Hardware specifications avoid mandatory proprietary platforms and provide open-source alternatives where possible (e.g., multiple VLA model options, cloud provider alternatives)

## Assumptions

1. **Student Hardware Access**: Students either have personal hardware meeting GPU requirements, access to provided lab workstations, or budget for cloud instances (<$300/quarter)
2. **Instructor Technical Background**: Instructors have domain expertise in robotics, AI, or related fields and are comfortable with Python, ROS concepts, and Linux environments
3. **Ubuntu 22.04 LTS Adoption**: Educational institutions can provision Ubuntu 22.04 LTS for lab workstations or guide students through dual-boot/VM setup
4. **NVIDIA GPU Availability**: RTX 4070 Ti / 4080 / 4090 class GPUs remain available for purchase in 2025–2026, or equivalent next-generation NVIDIA GPUs provide backward compatibility with Isaac Sim
5. **Open-Source VLA Stability**: At least one major open-source VLA model (RT-1, RT-2, OpenVLA, or 2025–2026 successors) remains publicly available with inference code and pre-trained weights
6. **Cloud Provider Pricing**: AWS EC2 GPU instance pricing (g5.xlarge or similar) and NVIDIA Omniverse Cloud pricing remain within $300/quarter student budget with cost optimization (spot instances, limited hours)
7. **ROS 2 LTS Support**: ROS 2 Humble (LTS until 2027) or Iron remains the recommended distribution with stable Isaac ROS packages
8. **Jetson Orin Availability**: Jetson Orin Nano and Orin NX dev kits remain available for purchase in 2025–2026 at stable pricing (~$500–$800)
9. **Whisper API Access**: OpenAI Whisper API or open-source Whisper models remain freely available for speech recognition integration
10. **LLM Planning Models**: Open-source or API-accessible LLMs (GPT-4, Claude, Llama 3+, or equivalents) can be used for task planning with acceptable latency (<1s planning time)
11. **Network Bandwidth**: Students and lab environments have sufficient internet bandwidth for cloud instance access (if used) and large package downloads (Isaac Sim ~50 GB, ROS packages ~5 GB)
12. **Course Delivery Mode**: The course is designed for in-person or hybrid delivery with synchronous lectures and lab access; fully asynchronous online delivery may require additional adaptations
13. **Assessment Integrity**: Students complete assignments and capstone projects individually or in small teams (2–3 students) with academic integrity policies enforced by the institution
14. **Physical Humanoid Availability**: Physical humanoid platforms for optional deployment are not provided by the course package; institutions procure separately if desired (e.g., TurtleBot 4, custom humanoid builds)
15. **Instructor Availability**: Instructors allocate sufficient time for weekly lectures (3–4 hours), office hours (2–3 hours), grading (5–10 hours for 30-student cohort), and student support throughout the 13-week term
16. **Git and GitHub Access**: Students and instructors have GitHub accounts and institutional approval to use GitHub Classroom (free for educational institutions). Students can install Git on their local systems or lab workstations
17. **Slide Deck Creation Capacity**: Instructors have access to presentation software (PowerPoint, Google Slides, Keynote, or LaTeX Beamer) and can dedicate 4-6 hours per week to convert detailed outlines into lecture-ready slides
18. **Language**: Course materials are provided in English. Institutions may translate to other languages with attribution, maintaining technical terminology consistency (ROS 2 terms remain in English even in translations)

## Out of Scope

- **Beginner Robotics Content**: The course does not cover basic robotics concepts (kinematics, dynamics, control theory) as these are assumed prerequisites or covered in earlier coursework
- **Custom LLM Training**: The course uses existing pre-trained LLMs for planning; training custom language models from scratch is explicitly excluded
- **Non-Humanoid Platforms**: Primary focus is humanoid robots; drones, autonomous vehicles, fixed-base robotic arms, or quadrupeds are not the core target platforms
- **Vendor-Specific Product Marketing**: The course avoids mandatory use of specific commercial humanoid products and maintains vendor neutrality with open-source alternatives
- **Comprehensive Humanoid Research Survey**: The course is not an academic survey of all humanoid robots ever built; it focuses on practical modern deployment skills
- **Real-Time Operating Systems**: Advanced RTOS topics, hard real-time guarantees, and low-level embedded programming are excluded; the course uses Ubuntu and standard ROS 2
- **Custom Hardware Design**: Students do not design or fabricate custom humanoid hardware (actuators, sensors, chassis); they use commercial off-the-shelf or provided kits
- **Regulatory Compliance and Certification**: The course does not cover robotics safety certifications (ISO 13482, CE marking) or regulatory approval processes for commercial deployment
- **Advanced Manipulation Skills**: Fine motor control, dexterous manipulation with multi-fingered hands, and tactile sensing are optional extensions, not core requirements
- **Multi-Robot Coordination**: Swarm robotics, multi-agent systems, and distributed coordination are optional advanced topics, not part of the base curriculum
- **Simulation-to-Reality Transfer Research**: While sim-to-real concepts are introduced, in-depth domain randomization, transfer learning research, and closing the reality gap are not core learning objectives
- **Production Deployment and DevOps**: CI/CD pipelines for robotics, container orchestration (Kubernetes), cloud robotics infrastructure, and production monitoring are excluded
- **Business and Entrepreneurship**: Commercialization strategies, business models for robotics startups, and entrepreneurship skills are not covered
- **Accessibility Features for Robots**: Designing robots for elderly care, disability assistance, or accessibility compliance is excluded from the core curriculum

## Dependencies

- **ROS 2 Humble/Iron LTS**: Course requires stable ROS 2 distribution with LTS support through 2026–2027
- **NVIDIA Isaac Sim**: Version compatible with ROS 2 Humble/Iron and Ubuntu 22.04 LTS with verified support through 2026
- **NVIDIA Isaac ROS**: GEMs (GPU-accelerated ROS packages) for perception, navigation, and manipulation with active maintenance
- **Nav2 Navigation Stack**: Stable Nav2 release compatible with ROS 2 Humble/Iron for autonomous navigation
- **NVIDIA GPU Drivers**: Latest stable drivers for RTX 4070 Ti / 4080 / 4090 supporting Isaac Sim and CUDA 11.8+ on Ubuntu 22.04
- **Jetson Orin Jetpack**: JetPack 5.x or 6.x supporting ROS 2 Humble/Iron and Isaac ROS on Jetson Orin Nano/NX
- **Whisper API or Open-Source Model**: OpenAI Whisper API access or locally runnable Whisper models for speech recognition
- **LLM API Access**: OpenAI GPT-4, Anthropic Claude, or open-source LLM (Llama 3+, Mistral) with API or local inference capability
- **Open-Source VLA Models**: At least one publicly available VLA model (RT-1, RT-2, OpenVLA, or 2025–2026 alternatives) with pre-trained weights and inference code
- **Cloud Provider (Optional)**: AWS EC2 with GPU instance availability (g5.xlarge or equivalent) or NVIDIA Omniverse Cloud with student pricing
- **Ubuntu 22.04 LTS**: Supported through 2027 with stable package repositories for ROS 2 and NVIDIA tooling
- **Python 3.10+**: Required for ROS 2 Humble/Iron, Isaac Sim Python API, and VLA model inference frameworks
- **Git and Version Control**: For distributing starter code, assignment templates, and student submission workflows

## Risks and Mitigations

### Risk 1: NVIDIA Isaac Sim Licensing or Support Changes

**Impact**: If NVIDIA discontinues free educational access to Isaac Sim or changes licensing terms mid-course, students and institutions may face unexpected costs or need alternative simulation platforms.

**Likelihood**: Low (NVIDIA has strong educational commitment and Omniverse ecosystem investment)

**Mitigation**:
- Verify Isaac Sim educational licensing terms and academic program eligibility before course launch
- Identify backup simulation platforms (Gazebo + gz_ros2_control, MuJoCo, PyBullet) with ROS 2 integration as fallback options
- Monitor NVIDIA announcements and maintain contact with NVIDIA academic programs team for early notice of changes

### Risk 2: Breaking Changes in ROS 2 or Nav2 Packages

**Impact**: If ROS 2 Humble/Iron or Nav2 packages introduce breaking API changes between course preparation and delivery, assignments and starter code may fail, requiring emergency updates.

**Likelihood**: Medium (ROS 2 LTS distributions are stable, but dependency updates can introduce breakage)

**Mitigation**:
- Pin exact package versions in installation tutorials and Docker containers for reproducibility
- Test all assignments on fresh Ubuntu 22.04 LTS installs 2 weeks before course launch
- Maintain contact with ROS 2 and Nav2 maintainer community for advance notice of deprecations
- Prepare version-specific documentation branches for ROS 2 Humble vs. Iron if divergence occurs

### Risk 3: Open-Source VLA Models Become Unavailable

**Impact**: If recommended VLA models (RT-1, RT-2, OpenVLA) are taken offline, change licenses to restrict commercial/educational use, or pre-trained weights are removed, students cannot complete VLA integration assignments.

**Likelihood**: Low-Medium (research institutions occasionally remove models; corporate acquisitions can restrict access)

**Mitigation**:
- Archive pre-trained VLA model weights in institutional storage (with license compliance) as backup
- Provide 2–3 alternative VLA model options in curriculum (if one becomes unavailable, switch to backup)
- Design VLA assignments to be model-agnostic (standardized input/output interfaces) so models are swappable
- Monitor VLA research community and Hugging Face repositories for new open-source releases

### Risk 4: GPU Hardware Availability and Pricing Volatility

**Impact**: If RTX 4070 Ti / 4080 / 4090 GPUs become unavailable due to supply chain issues or next-generation GPU launches, hardware procurement for lab tiers may fail or exceed budget.

**Likelihood**: Medium (GPU markets are volatile; generational transitions can create shortages)

**Mitigation**:
- Update hardware specifications quarterly to include latest NVIDIA GPU generation equivalents (RTX 50-series if released)
- Provide performance benchmarks and compatibility notes for multiple GPU generations
- Emphasize cloud fallback path to reduce dependency on local GPU procurement
- Establish relationships with educational hardware suppliers (NVIDIA, system integrators) for priority allocation

### Risk 5: Students Struggle with Prerequisite Skills

**Impact**: If students enroll without sufficient Python, deep learning, or Linux background, they may fall behind by Week 3–4 and require extensive remedial support, straining instructor/TA resources.

**Likelihood**: Medium-High (prerequisites are often self-assessed; students may overestimate readiness)

**Mitigation**:
- Provide Week 0 prerequisite self-assessment quiz with recommended resources for skill gaps
- Include optional "bootcamp" pre-course module (1 week) covering Python async, PyTorch basics, and Linux CLI
- Offer recorded prerequisite refresher videos and office hours during Week 1–2
- Implement early assessment checkpoint at Week 4 to identify struggling students and provide intervention resources

### Risk 6: Capstone Complexity Exceeds Student Capabilities

**Impact**: If the end-to-end voice-controlled autonomous humanoid capstone is too ambitious, students may fail to integrate all components (Whisper, LLM, Nav2, VLA) and become demoralized, leading to high failure rates.

**Likelihood**: Medium (capstone is complex multi-system integration; debugging distributed ROS 2 systems is challenging)

**Mitigation**:
- Provide incremental capstone milestones in Week 11–12 (voice transcription working → LLM planning working → navigation working → manipulation working → full integration)
- Offer "capstone-lite" fallback option with reduced scope (e.g., scripted commands instead of LLM planning, or navigation-only without manipulation)
- Create detailed debugging guides for common integration issues (ROS 2 topic mismatches, timing synchronization, Isaac Sim crashes)
- Allow team-based capstones (2–3 students) to distribute workload and enable peer support

### Risk 7: Cloud Costs Exceed $300/Quarter Budget

**Impact**: If students using cloud fallback misconfigure auto-shutdown, use expensive instance types, or run simulations inefficiently, costs may spiral beyond the $300 target, creating financial barriers.

**Likelihood**: Medium-High (students unfamiliar with cloud billing can easily overspend)

**Mitigation**:
- Provide auto-shutdown scripts and cost monitoring dashboards as part of cloud setup tutorials
- Recommend AWS Budget Alerts or billing notifications set at $50, $100, $200 thresholds
- Use spot instances for non-critical workloads to reduce costs by 60–70%
- Create cost estimation calculator in the course package with usage scenarios (hours/week, instance type → monthly cost)
- Offer "cloud office hours" in Week 1–2 to help students configure cost controls

### Risk 8: Jetson Orin Deployment Failures

**Impact**: If students attempting optional Jetson Orin deployment encounter hardware failures, incompatible sensors, or driver issues, they may waste significant time troubleshooting with limited instructor expertise.

**Likelihood**: Medium (embedded deployment is notoriously challenging; hardware variability introduces unknowns)

**Mitigation**:
- Clearly mark Jetson Orin deployment as **optional** and not required for course completion
- Provide tested, validated Jetson Orin hardware kits with verified sensor compatibility (if institutions procure)
- Create comprehensive Jetson troubleshooting guide with common issues (thermal throttling, power supply, I2C conflicts)
- Recommend students complete full capstone in Isaac Sim first, then attempt Jetson deployment as time permits
- Partner with NVIDIA Embedded Developer Relations for Jetson-specific support resources

### Risk 9: GitHub Classroom Dependency and Access Issues

**Impact**: If institutions cannot use GitHub Classroom due to data privacy policies, firewall restrictions, or GitHub service outages during assignment deadlines, the submission workflow breaks down and instructors cannot collect/grade assignments efficiently.

**Likelihood**: Low-Medium (most educational institutions allow GitHub; outages are rare but impactful)

**Mitigation**:
- Document alternative submission pathway using institutional LMS (Canvas, Moodle, Blackboard) with ZIP file uploads as fallback
- Provide scripts to bulk-download GitHub Classroom submissions for offline grading in case of GitHub unavailability
- Include Git LFS (Large File Storage) setup instructions in Week 1 to prevent repository bloat from large files
- Create troubleshooting guide for common GitHub Classroom issues (SSH key setup, repository access permissions, autograding failures)
- Recommend instructors test GitHub Classroom setup 2 weeks before course launch to identify institutional firewall/policy blockers early

## Notes

- This specification defines the **content and structure** of the course package, not the implementation of teaching the course itself
- Instructors are expected to have domain expertise in robotics/AI or related fields; the package provides curriculum structure, not pedagogy training
- The course is designed to be modular: institutions can substitute alternative tools (e.g., different VLA models, alternative LLM providers) if they maintain equivalent capabilities
- Weekly time budget (12–15 hours) assumes students have strong Python and deep learning foundations; students with weaker backgrounds may require 18–20 hours
- Hardware procurement lead times (4–8 weeks) must be factored into course launch planning; cloud fallback mitigates this risk
- The capstone project can be scaled for team-based work (2–3 students) or individual completion depending on cohort size and instructor preference
- Grading rubrics are designed for objective assessment but allow instructor discretion for edge cases and exceptional work
- **Distribution Model**: The course package uses a hybrid distribution approach: (1) Public GitHub repository containing open-source components (code templates, setup scripts, ROS 2 packages, .gitignore templates, GitHub Actions workflows, troubleshooting guides), and (2) Private institutional distribution for premium materials (complete slide deck outlines, assignment solutions, detailed grading rubrics with answer keys, TA guides). Public repository is versioned with semantic releases (v1.0.0, v1.1.0) and institutions can fork/clone to receive updates via Git pull
- **Licensing**: All code (starter templates, ROS 2 packages, scripts) is MIT-licensed to allow institutional customization and redistribution. Slide deck outlines, assignment prompts, and grading rubrics are licensed under Creative Commons Attribution 4.0 (CC BY 4.0), permitting modification and commercial use with attribution. Institutions may create derivative works and charge tuition for courses using these materials. Attribution requirement: "Based on Physical AI & Humanoid Robotics Capstone Course by [Original Authors], available at [GitHub URL]"
- The specification assumes continuous internet access for package downloads, cloud instances, and API calls (Whisper, LLM); fully air-gapped environments require additional preparation
