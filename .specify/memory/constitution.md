# Physical AI & Humanoid Robotics Constitution

<!--
Sync Impact Report - Constitution v1.1.0
────────────────────────────────────────────────────────────────────
Version Change: 1.0.0 → 1.1.0 (MINOR)
Modified Principles:
  - Principle III (Sim-to-Real Pipeline): Clarified applies to Weeks 3-13, not Weeks 1-2
Modified Standards:
  - Graded Hands-On Assignments: Updated enforcement language for "runs on hardware"
  - Capstone End-to-End Autonomy: Clarified "minimal manual intervention" boundaries
Added Sections:
  - Code Quality & ROS Package Standards (new Key Standard)
  - Equitable Hardware Access (new Key Standard)
Modified Success Criteria:
  - SC-001: Relaxed to ≤10% by Week 3 (from <5% immediately)
  - SC-002: Clarified "minimal manual intervention" vs "zero"
  - SC-003: Relaxed to ≥95% (from 100%) with theory exception

Removed Sections: None

Templates Sync Status:
  ✅ plan-template.md - Constitution Check section compatible
  ✅ spec-template.md - Requirements and success criteria align
  ✅ tasks-template.md - Task categorization supports hands-on deliverables
  ✅ Code quality checks can be added to CI/CD workflows
  ⚠️  Command files (.claude/commands/*.md) - May need updates for robotics-specific workflows

Follow-up TODOs:
  - Update assignment templates to include ROS package structure checklist
  - Create hardship request form for hardware access equity
  - Add linting/testing CI workflow examples
  - Update grading rubrics to include code quality criteria
────────────────────────────────────────────────────────────────────
-->

## Core Principles

### I. Real-World Readiness

Every learning objective MUST include ≥1 hands-on task demonstrating deployment capability (simulation→Jetson OR simulation→cloud edge device). All course content, assignments, and assessments MUST prepare students for immediate application on physical robots or edge devices used in industry.

**Rationale**: Students graduate into an industry where simulation-only knowledge is insufficient. Employers require engineers who can handle hardware constraints, sensor noise, latency, safety protocols, and physical debugging—skills that cannot be learned from theory or simulation alone.

**Testability**: Every module's learning objectives checklist includes at least one task marked "Deployment: [Sim→Jetson | Sim→Cloud]" with graded submission artifact.

### II. Hands-On Over Theory

Students MUST build, simulate, and (when feasible) deploy on physical hardware. Lectures and readings are supplementary; the primary learning mode is active construction and debugging of robotic systems. Every module artifact MUST pass automated checks (ROS node launches without errors, publishes expected topics) OR demonstrate ≥80% of specified behaviors per rubric.

**Rationale**: Physical AI and robotics engineering is fundamentally a practice discipline. Conceptual understanding without hands-on implementation leaves students unprepared for the tactile, iterative nature of real robotics work. Every module must culminate in a working artifact (code, simulation, or physical deployment).

**Testability**: Grading rubrics include binary "Artifact Check" (launches without errors: Yes/No) and scored "Behavior Checklist" (e.g., 4/5 required behaviors demonstrated = 80%).

### III. Sim-to-Real Pipeline

Weeks 3-13 MUST include ≥1 graded assignment component demonstrating sim→hardware transfer (e.g., "Run navigation stack in Gazebo, then deploy identical launch file to Jetson and submit performance comparison log"). Students must understand domain randomization, sensor calibration, performance profiling, and deployment constraints at each stage.

Weeks 1-2 (ROS 2 basics, simulation environment setup) focus on foundational skills; sim-to-real integration begins Week 3 with "Deploy your first node to Jetson" assignment.

**Rationale**: The sim-to-real gap is the most critical challenge in embodied AI. Students who cannot transfer learned policies or navigation stacks from simulation to hardware will struggle in professional robotics roles. This principle ensures no "simulation-only" graduates.

**Testability**: Weeks 3-13 assignment submissions include sim→hardware comparison artifact (performance logs, latency measurements, or side-by-side video).

### IV. Future-Proof Stack

Course MUST prioritize currently industry-dominant tools as of 2025 and beyond: ROS 2 Humble/Iron, NVIDIA Isaac Sim with ROS integration, Nav2, modern Vision-Language-Action (VLA) approaches, and actively maintained hardware platforms.

**Rationale**: Deprecated technologies (ROS 1, Gazebo Classic, unsupported Jetson models, MoveIt1 without MoveIt2 migration paths) create technical debt for students and limit their job market competitiveness. All tooling must have vendor/community support through at least 2028.

**Specific Requirements**:
- ROS 2 Humble (LTS until May 2027) or Iron minimum
- NVIDIA Isaac Sim 4.x+ with ROS 2 bridge
- Nav2 for autonomous navigation
- MoveIt2 for manipulation planning
- Jetson Orin Nano/NX/AGX (NOT TX1/TX2/Xavier)
- Ubuntu 22.04 LTS (Jammy Jellyfish)
- Python 3.10+ for all student code

### V. Progressive Complexity

Course structure MUST start simple (single ROS 2 node) and build incrementally to full autonomous humanoid with voice commands. No module should require understanding of components not yet taught. Each week must add one layer of complexity on a solid foundation.

**Rationale**: Cognitive overload from premature complexity is the primary cause of student dropout in robotics courses. A clear learning ladder—from "Hello World" node to full VLA-driven humanoid—ensures students build confidence and competence simultaneously.

**Progression Map**:
1. ROS 2 basics (nodes, topics, services)
2. Simulation environment setup (Isaac Sim/Gazebo)
3. Single-degree-of-freedom control
4. Multi-joint manipulation
5. Mobile navigation
6. Perception (cameras, LiDAR)
7. Voice command integration (ASR → LLM planning)
8. Full autonomy (VLA-driven pick-and-place)

## Key Standards

### Tool & Version Explicitness

All software tools, hardware models, and library versions MUST be explicitly stated and currently supported (as of 2025). No ambiguous "install ROS" or "use a Jetson" instructions.

**Enforcement**:
- Every lab manual must include exact version commands (e.g., `ros2 --version` output)
- Hardware guides must specify exact SKUs and purchase links
- Deprecated tools trigger immediate curriculum revision

### Code Quality & ROS Package Standards

Student code MUST meet industry ROS package standards to ensure maintainability, reproducibility, and professional readiness.

**Enforcement**:
- **Package Structure**: Every ROS 2 package MUST include:
  - `package.xml` with correct dependencies listed
  - `setup.py` or `CMakeLists.txt` with proper build configuration
  - `launch/` directory with at least one `.launch.py` file
  - `README.md` documenting: purpose, dependencies, launch instructions, parameters
- **Documentation**: Every ROS node MUST have module-level docstring specifying:
  - Purpose (1-2 sentences)
  - Subscribed topics (name, message type, purpose)
  - Published topics (name, message type, publish rate)
  - Parameters (name, type, default value, valid range)
- **Code Style**: Python code MUST pass `ruff check` (or `flake8`) with ≤5 violations per 100 lines
- **Testing**: Packages MUST include ≥2 unit tests (pytest or ros2 test framework) demonstrating node functionality
- **Version Control**: Assignments submitted via Git with ≥3 meaningful commits (descriptive messages, not "final", "fix", "asdf")

**Testability**: Automated CI checks for linting violations and test count; manual rubric review for documentation quality (0-5 point scale: 5=all sections complete, 4=missing parameter docs, etc.)

### Graded Hands-On Assignments

Every module MUST contain at least one graded hands-on assignment that runs on student hardware (personal workstation, lab Jetson, or cloud GPU instance). ≥95% of graded assignments (minimum 12 of 13 weeks) MUST run on hardware.

**Enforcement**:
- Minimum one graded assignment per week
- Assignment must produce a verifiable artifact (ROS 2 bag file, simulation video, physical robot demo)
- Assignment code runs on target hardware with only documented setup steps (environment variables, launch commands listed in README). No debugging, code edits, or manual topic publishing permitted during grading.
- Up to 1 theory-focused week (e.g., inverse kinematics derivations, PID tuning math) permitted if:
  - Followed immediately by hands-on implementation week applying theory
  - Theory module includes worked examples from real robot logs/data

### Equitable Hardware Access

All students MUST have viable path to complete hardware assignments regardless of personal financial resources.

**Enforcement**:
- **Lab Loaners**: Department MUST maintain loaner pool of ≥0.3× enrollment (e.g., 30 Jetson Orin Nanos for 100-student course), allocated first-come-first-served or need-based
- **Cloud Credits**: MUST provide minimum $200/student in AWS/Azure/Paperspace credits for cloud GPU access (Isaac Sim on g5.2xlarge instances)
- **Hardship Exception Process**: If student cannot afford Budget tier (<$1k) AND lab loaners exhausted:
  1. Student submits hardship request by Week 2
  2. Instructor approves cloud-only path with virtual robot demos accepted for physical deployment assignments
  3. Student completes alternative "sim-to-real theory" module (analyzing deployment constraints via case studies)
- **Equipment Failure Protocol**: If student's hardware fails mid-semester OR lab equipment breaks:
  - 1-week automatic extension for affected assignment
  - Priority access to lab loaners for makeup work
  - Option to redo assignment on different hardware without penalty

**Testability**: Track loaner checkout logs, cloud credit redemption rates, hardship request count (<10% enrollment = acceptable access rate).

### Capstone End-to-End Autonomy

Capstone project MUST demonstrate: voice command → LLM planning → ROS 2 action sequence → navigation + manipulation in simulation AND (optional) real robot.

**Acceptance Criteria**:
- **Simulation**: "Pick up the red cup on the table and bring it to me" with minimal manual intervention limited to:
  - Initial Gazebo world load command (`ros2 launch ...`)
  - Robot spawn/reset (if physics engine requires)
  - Single voice command trigger ("Alexa, pick up the red cup")
  - **NOT permitted**: Manual ROS topic publishing, teleoperation during task execution, code debugging during graded run
- **Real robot** (optional but encouraged): Same task on physical platform with teleoperation fallback available
- **Deliverables**: Students must submit: system architecture diagram, ROS 2 node graph (`ros2 node list` + topic connections), simulation recording, code repository with README

**Testability**: Graded rubric with binary pass/fail on each phase (voice recognition → LLM output → motion planning → grasp → navigation → delivery).

### Hardware Recommendations Transparency

All hardware recommendations MUST include exact model numbers, minimum specs, and realistic cost tiers. No generic "you need a GPU" guidance.

**Enforcement**:
- Three tiers published: Budget (<$1k), Mid-range ($3k–$5k), Premium ($15k+)
- Each tier lists exact components with purchase links (updated annually)
- Lab alternatives provided for students without personal hardware (see Equitable Hardware Access standard)

### Safety & Ethical Deployment

Physical robot operation MUST address emergency stops, teleoperation fallback, and responsible AI guidelines (no autonomous weapons, privacy-invasive surveillance, or uncontrolled public deployment).

**Enforcement**:
- Week 1 safety training (emergency stop protocols, workspace setup)
- Every physical robot assignment requires safety checklist submission
- LLM/VLA modules include discussion of bias, failure modes, and human oversight

## Constraints

### Course Duration

Maximum 13 weeks (one academic semester). Content must be achievable within this timeframe without sacrificing hands-on depth.

### Target Audience

Advanced undergraduate or master's students with prior Python + basic ML experience. NO prior robotics experience required, but programming fluency (classes, async, debugging) is assumed.

### Primary Operating System

Ubuntu 22.04 LTS (Jammy Jellyfish), dual-boot or native Linux mandatory. WSL2 or VMs are NOT supported due to USB passthrough and GPU limitations for Isaac Sim.

**Justification**: ROS 2 Humble/Iron, Isaac Sim, and Jetson toolchains have fragile or nonexistent support on Windows/macOS. WSL2 introduces USB and GUI complexity that consumes hours of troubleshooting.

**Exception Process**: Students with institutional Windows machines they cannot modify OR Mac M1/M2 (cannot dual-boot x86 Linux) may request cloud-only exception via hardship process (see Equitable Hardware Access).

### GPU Requirement

NVIDIA RTX 4070 Ti or higher (12GB+ VRAM) for local Isaac Sim work, OR equivalent cloud instance (e.g., AWS g5.2xlarge with RTX A10G).

**Justification**: Isaac Sim's real-time ray tracing and physics simulation require significant GPU compute. Underpowered GPUs result in <5 FPS simulations, making development impractical.

### Budget Transparency

Three realistic lab tiers MUST be provided:
- **Budget** (<$1k per student/group): Cloud GPU + used Jetson Orin Nano
- **Mid-range** ($3k–$5k): RTX 4070 Ti + Jetson Orin NX + basic sensors (RealSense, RPLiDAR)
- **Premium** ($15k+ per student/group): RTX 4090 + Jetson AGX Orin + full sensor suite + manipulator arm (e.g., WidowX 250)

### No Deprecated Tools

MUST NOT include:
- ROS 1 (EOL April 2025)
- Gazebo Classic (replaced by Gazebo Harmonic/Ignition)
- Jetson TX1/TX2/Xavier (no Ubuntu 22.04 support)
- MoveIt1 without MoveIt2 migration path
- Python 2.x

## Success Criteria

### SC-001: Independent Environment Setup

By Week 3, ≤10% of students require instructor intervention beyond office hours for environment issues. Initial setup (Weeks 1-2) may require higher support; track week-over-week troubleshooting ticket reduction to verify documentation improvements.

**Measurement**: Post-setup survey + weekly troubleshooting ticket count. Target: Week 1 (≤30%), Week 2 (≤20%), Week 3+ (≤10%).

### SC-002: Capstone Autonomy Demonstration

100% of passing students MUST successfully complete the capstone task in simulation: "Pick up the red cup on the table and bring it to me" via natural voice command with minimal manual intervention as defined in Capstone standard (initial world load, robot spawn, voice trigger only).

**Measurement**: Graded demo rubric (pass/fail on voice→grasp→navigate→deliver). Record number of manual interventions per student; must be ≤3 per graded run.

### SC-003: Hardware-Graded Assignments

≥95% of graded assignments (minimum 12 of 13 weeks) MUST run on student-owned, lab-provided, or cloud hardware. Up to 1 theory-focused week permitted per semester if followed by implementation week.

**Measurement**: Assignment submission artifacts (ROS bags, videos, logs) verified for hardware execution. Track percentage weekly.

### SC-004: Instructor-Ready Curriculum Package

Course syllabus, weekly schedule, hardware guide, and grading rubric MUST be complete and immediately usable by any university or bootcamp instructor in 2026+ without requiring original author support.

**Measurement**: External instructor pilot test (can they teach Week 1 without contacting course creator?).

### SC-005: Hardware Longevity

All recommended hardware kits MUST be purchasable today (2025) and supported by manufacturers until at least 2028.

**Measurement**: Annual hardware guide audit + vendor support confirmation.

## Governance

This constitution is the supreme governing document for the Physical AI & Humanoid Robotics course. All curriculum decisions, tool selections, assignment designs, and grading policies MUST align with the principles, standards, constraints, and success criteria defined herein.

### Amendment Procedure

1. **Proposal**: Any instructor, student, or industry reviewer may propose an amendment via GitHub issue/PR or course governance channel.
2. **Impact Analysis**: Proposer must document:
   - Which principle/standard/constraint is affected
   - Rationale for change (industry shift, tool deprecation, pedagogical evidence)
   - Downstream impacts (assignments, hardware, grading)
   - Migration plan for existing students
3. **Review**: Course governance committee (minimum 3 instructors + 1 industry advisor) reviews within 14 days.
4. **Approval**: Requires 75% committee approval + public comment period (7 days).
5. **Versioning**: Amendments trigger version bump (see below).
6. **Propagation**: All templates, assignments, and documentation updated within 30 days.

### Versioning Policy

- **MAJOR** (X.0.0): Backward-incompatible changes (e.g., removing a principle, switching from ROS 2 to ROS 3, dropping Jetson requirement)
- **MINOR** (x.Y.0): New principles, sections, or material expansions (e.g., adding "Safety-First" principle, new hardware tier)
- **PATCH** (x.y.Z): Clarifications, wording fixes, typo corrections, non-semantic refinements

### Compliance Review

- **Weekly**: Instructors verify new assignments pass Constitution Check (see plan-template.md)
- **Semester**: Full curriculum audit against all principles, standards, constraints, success criteria
- **Annual**: Hardware guide update, tool version review, industry alignment check

### Conflict Resolution

When course requirements conflict with constitution:
1. Constitution takes precedence (course must adapt)
2. If adaptation impossible, amendment procedure triggered
3. Temporary exceptions require 100% committee approval + sunset date (<6 months)

**Version**: 1.1.0 | **Ratified**: 2025-12-06 | **Last Amended**: 2025-12-06
