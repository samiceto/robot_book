# Feature Specification: Physical AI & Humanoid Robotics Textbook

**Feature Branch**: `002-humanoid-robotics-book`
**Created**: 2025-12-06
**Status**: Draft
**Input**: User request for complete book plan: "Physical AI & Humanoid Robotics: From Simulated Brains to Walking Bodies" - a definitive 2026 practitioner textbook + lab manual targeting advanced undergraduate/Master's students, industry engineers transitioning to humanoid robotics, and university instructors.

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Student Self-Study and Lab Completion (Priority: P1)

An advanced undergraduate or Master's student with Python proficiency and basic deep learning knowledge purchases the book to learn humanoid robotics. They need to progress through chapters sequentially, complete hands-on labs, run code examples on their own hardware (RTX 4070+ GPU or cloud instance), and build confidence in deploying autonomous humanoid systems.

**Why this priority**: The book's core value is enabling self-directed learning. Students must be able to follow the material independently without instructor support.

**Independent Test**: A student with the specified prerequisites can work through any chapter's lab section in ≤15 hours, successfully run all code examples on their hardware setup, and complete the end-of-chapter project without external help beyond the book's content.

**Acceptance Scenarios**:

1. **Given** a student reads Chapter 3 (ROS 2 Fundamentals), **When** they follow the installation tutorial for Ubuntu 22.04, **Then** they successfully install ROS 2 Iron/Jazzy, verify the installation, and run the first "Hello ROS 2" example within 2 hours
2. **Given** a student completes Chapter 8 (Isaac Sim Digital Twins), **When** they follow the Isaac Sim setup guide, **Then** they install Isaac Sim 2024.2+, load the provided humanoid URDF, run the simulation at ≥30 FPS on RTX 4070 Ti, and complete the manipulation lab in ≤12 hours
3. **Given** a student reaches Chapter 14 (VLA Models for Manipulation), **When** they clone the companion GitHub repository and follow the setup instructions, **Then** all dependencies install successfully, the VLA model inference runs at ≥10 Hz on Jetson Orin Nano 8GB, and they complete the pick-and-place demo
4. **Given** a student finishes the final capstone chapter (Chapter 20), **When** they integrate voice recognition (Whisper) + LLM planning + Nav2 + VLA model, **Then** the complete pipeline runs end-to-end at ≥12 Hz on Jetson Orin Nano with <2 GB RAM usage

---

### User Story 2 - Instructor Course Adoption and Curriculum Integration (Priority: P2)

A university instructor teaching a robotics or AI course wants to adopt this textbook as the primary reference. They need clear chapter-to-week mapping, assignment prompts they can use directly, grading rubrics, and assurance that all technical content works with current (2025-2027) software versions.

**Why this priority**: Instructor adoption drives book sales and validates educational quality. The book must be "course-ready" not just self-study material.

**Independent Test**: An instructor can design a 13-week course using the book's chapter structure, assign labs as graded homework, and verify that all code examples and repos work on fresh Ubuntu 22.04/24.04 installations with ROS 2 Iron/Jazzy.

**Acceptance Scenarios**:

1. **Given** an instructor reviews the book's structure, **When** they plan a 13-week semester course, **Then** they find a clear mapping of chapters to weeks with recommended pacing (2-3 chapters per week for foundation, 1 chapter per week for advanced topics)
2. **Given** an instructor assigns Chapter 10's lab (Nav2 Configuration), **When** students submit their work, **Then** the instructor can use provided grading criteria (rubric in appendix) to objectively score submissions
3. **Given** an instructor sets up a lab environment in Q1 2026, **When** they follow the hardware recommendations (Budget/Mid/Premium tiers from Appendix A), **Then** all specified GPUs, Jetson boards, and peripherals are available for purchase with exact part numbers and valid links
4. **Given** an instructor encounters a software compatibility issue, **When** they check the book's errata page or GitHub issues, **Then** they find documented workarounds or updates for ROS 2/Isaac Sim version changes

---

### User Story 3 - Industry Engineer Skill Transition and Practical Deployment (Priority: P2)

An industry software engineer with web/mobile development background but no robotics experience wants to transition into humanoid robotics roles at companies like Figure AI, Tesla (Optimus team), or Agility Robotics. They need practical, production-relevant skills including edge deployment, performance optimization, and sim-to-real transfer.

**Why this priority**: Industry practitioners are a key audience and demand high-quality, directly applicable content. Their success validates the book's practical relevance.

**Independent Test**: An engineer can complete the book's advanced chapters (12-20), deploy a working voice-controlled pick-and-place demo on Jetson Orin hardware, and demonstrate the skills in a technical interview or portfolio project.

**Acceptance Scenarios**:

1. **Given** an engineer completes Chapter 16 (Jetson Orin Edge Deployment), **When** they flash a Jetson Orin Nano 8GB and deploy the provided inference pipeline, **Then** they achieve ≥15 Hz real-time performance for VLA model inference with optimizations (TensorRT, quantization, batching)
2. **Given** an engineer studies Chapter 18 (Sim-to-Real Transfer), **When** they apply domain randomization and reality gap mitigation techniques, **Then** they successfully transfer a grasping policy trained in Isaac Sim to a physical robotic arm with <20% performance degradation
3. **Given** an engineer interviews for a humanoid robotics role, **When** they demonstrate the capstone project (Chapter 20: End-to-End Autonomous Humanoid), **Then** they can explain the architecture (Whisper → LLM planner → Nav2 → VLA → actuators), debug common failures, and discuss tradeoffs in real-time
4. **Given** an engineer needs to stay current with rapidly evolving tools, **When** they access the book's companion GitHub repository, **Then** they find regularly updated code examples tested against latest ROS 2/Isaac Sim releases with CI passing on Ubuntu 22.04/24.04

---

### User Story 4 - Researcher Reproducibility and Baseline Implementation (Priority: P3)

A PhD student or research scientist working on embodied AI needs a reliable baseline implementation of modern VLA pipelines to compare against their novel algorithms. They require reproducible code, clear documentation of hyperparameters, and validated performance benchmarks.

**Why this priority**: Research reproducibility is critical for academic credibility. The book should provide reference implementations that researchers can cite and build upon.

**Independent Test**: A researcher can clone a companion repository (e.g., Chapter 14's VLA implementation), run the exact setup on their hardware, reproduce reported metrics (inference latency, task success rate), and modify the code for their experiments.

**Acceptance Scenarios**:

1. **Given** a researcher clones the OpenVLA integration repository (Chapter 14), **When** they follow the README setup instructions, **Then** they reproduce the reported 12 Hz inference rate on Jetson Orin Nano 8GB ±10% variance
2. **Given** a researcher wants to compare their VLA model against the book's baseline, **When** they review the documented evaluation protocol (test scenes, success criteria, metrics), **Then** they can run the same evaluation and report comparable results
3. **Given** a researcher modifies the provided Nav2 configuration (Chapter 11), **When** they test their custom planner, **Then** the modular code structure allows swapping components without breaking the full pipeline
4. **Given** a researcher cites the book's methodology, **When** they reference specific techniques (e.g., "domain randomization strategy from Chapter 18"), **Then** they find sufficient technical detail to replicate the approach without ambiguity

---

### User Story 5 - Self-Learner Web Developer Transitioning to Robotics (Priority: P3)

A self-taught programmer with strong Python skills but no formal robotics or deep learning education wants to break into humanoid robotics. They need the book to fill foundational gaps (ROS 2, computer vision, control theory basics) while building practical projects for their portfolio.

**Why this priority**: Self-learners represent a growing segment of technical education. The book should be accessible without requiring a full undergraduate robotics curriculum.

**Independent Test**: A programmer with Python proficiency but no ROS/robotics background can start at Chapter 1, progressively build skills, and complete at least 3 portfolio-worthy projects (e.g., Nav2 autonomous navigation, VLA pick-and-place, voice-controlled humanoid) by the end of the book.

**Acceptance Scenarios**:

1. **Given** a self-learner starts Chapter 1 (Introduction to Physical AI), **When** they encounter unfamiliar concepts (kinematics, SLAM, reinforcement learning), **Then** the book provides either inline explanations or references to prerequisite resources (e.g., "See Appendix B: Control Theory Primer")
2. **Given** a self-learner completes Part 2 (Digital Twins & Simulation), **When** they build their first Isaac Sim humanoid scene, **Then** they have a working demo they can screen-record and add to their portfolio with provided documentation templates
3. **Given** a self-learner struggles with a complex topic (e.g., bipedal locomotion in Chapter 17), **When** they review the chapter's "Common Pitfalls" sidebar and troubleshooting section, **Then** they find solutions to typical errors (unstable gaits, contact force issues)
4. **Given** a self-learner finishes the book, **When** they apply for junior robotics engineer roles, **Then** they can demonstrate 3-5 working projects from the book's labs on GitHub with clear READMEs and demo videos

---

### Edge Cases

- What happens when a student's hardware (RTX 4060) doesn't meet minimum requirements (RTX 4070 Ti) and cloud costs exceed their budget?
- How does the book handle rapid software deprecation (e.g., Isaac Sim 2025.x breaks compatibility with 2024.x code)?
- What happens if a recommended VLA model (OpenVLA, RT-2-X) is taken offline or license-restricted mid-publication cycle?
- How does an instructor adapt the book for a 10-week quarter system instead of 13-week semester?
- What happens when a student attempts to run code on Windows/macOS instead of Ubuntu 22.04/24.04?
- How does the book support students who want to use alternative hardware (AMD GPUs, Apple Silicon Macs) not officially supported by Isaac Sim?
- What happens if a researcher cannot reproduce results due to undocumented dependencies or environment differences?
- How does the book handle breaking changes in ROS 2 Jazzy vs. Iron (different APIs, package names)?
- What happens when GitHub Actions CI fails on the companion repositories due to GPU runner limitations?

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: The book MUST be organized into 6 parts covering: (1) Foundations & ROS 2, (2) Digital Twins & Simulation, (3) Perception & Edge Deployment, (4) Embodied Cognition & VLA Models, (5) Bipedal Locomotion & Control, (6) Capstone Integration & Sim-to-Real Transfer
- **FR-002**: The book MUST contain 18-22 chapters with total length 550-650 pages (~160,000-190,000 words including code listings)
- **FR-003**: Every chapter MUST include: learning objectives, theoretical foundation (30-40% of content), hands-on lab section (40-50%), code walkthrough with line-by-line explanations, end-of-chapter project, and further reading references (minimum 10 citations per chapter in APA 7th edition)
- **FR-004**: The book MUST provide exact hardware specifications in Appendix A: Budget tier (RTX 4070 Ti, <$1200), Mid tier (RTX 4080, $2500-3500), Premium tier (RTX 4090 + Jetson AGX Orin, $5000+) with exact part numbers, current (2025) prices, and purchase links from at least 2 vendors per component
- **FR-005**: All code examples MUST be tested weekly during writing on: Ubuntu 22.04 + ROS 2 Iron + Isaac Sim 2024.2 AND Ubuntu 24.04 + ROS 2 Jazzy + Isaac Sim 2025.x to ensure forward compatibility
- **FR-006**: The book MUST include at least 12 companion GitHub repositories (one per major topic: ROS 2 basics, Isaac Sim integration, Nav2 config, VLA inference, Jetson deployment, etc.) with MIT license, comprehensive READMEs, and CI pipelines (GitHub Actions) that verify builds on Ubuntu 22.04 + RTX 4090 runner
- **FR-007**: Every code snippet in the book MUST follow Black formatter + ROS 2 Python style guide (PEP8 + flake8) with linting verified in pre-commit hooks
- **FR-008**: The book MUST specify the primary software stack with exact version ranges: ROS 2 Iron Irwini (2023-2027 support) or Jazzy Jalisco (2024-2029 support), NVIDIA Isaac Sim 2024.2+ (Omniverse), Isaac ROS 3.0+, Nav2 (latest stable), Ubuntu 22.04 LTS or 24.04 LTS
- **FR-009**: The book MUST document 3-5 major architectural decisions with explicit tradeoff analysis in dedicated "Design Decisions" sections: (1) Humanoid platform choice, (2) VLA model backbone, (3) Primary simulation engine, (4) Edge hardware tier, (5) Code license
- **FR-010**: The book MUST provide a quickstart guide (Chapter 2 or Appendix) that gets a reader from zero to running their first ROS 2 + Isaac Sim demo in <4 hours on fresh Ubuntu 22.04 installation
- **FR-011**: Each hands-on lab MUST include acceptance criteria and validation steps so readers can verify correct completion (e.g., "Your humanoid should navigate to waypoint within 30 seconds without collision")
- **FR-012**: The book MUST include troubleshooting sections in every lab chapter addressing common errors: NVIDIA driver conflicts, ROS 2 build failures, Isaac Sim crashes, Nav2 tuning issues, Jetson deployment failures
- **FR-013**: The capstone project (Chapter 20 or final chapter) MUST integrate all learned skills: voice input (Whisper) → LLM task planning → ROS 2 action execution → Nav2 navigation → VLA manipulation → Isaac Sim rendering, with end-to-end demo running at ≥12 Hz on Jetson Orin Nano 8GB using <2 GB RAM
- **FR-014**: The book MUST provide performance benchmarks for all major components: Isaac Sim FPS (Budget: ≥30, Mid: ≥60, Premium: ≥120), VLA inference latency (Jetson Orin Nano: ≥10 Hz, Orin NX: ≥15 Hz, Orin AGX: ≥30 Hz), Nav2 planning time (<500ms for 10m path), end-to-end pipeline latency (<100ms p95 on RTX 4090)
- **FR-015**: The book MUST include accessibility features: all figures have descriptive alt text, code listings include syntax highlighting with colorblind-safe palette, equations provided in both LaTeX and plain-text explanations
- **FR-016**: The book MUST provide a web-based version using Docusaurus for frontend UI with search functionality, syntax-highlighted code blocks, interactive diagrams (where applicable), and mobile-responsive layout
- **FR-017**: The book MUST include appendices covering: (A) Hardware Buyer's Guide, (B) Math & Control Theory Primer, (C) Troubleshooting Reference, (D) Grading Rubrics for Instructors, (E) Glossary of Terms, (F) Software Installation Checklists
- **FR-018**: Every major algorithm or pipeline MUST reference at least one fully reproducible open-source implementation with GitHub link, commit SHA, and verified compatibility with book's software stack
- **FR-019**: The book MUST define clear prerequisites in Chapter 1: Python proficiency (OOP, async, type hints), basic deep learning (PyTorch or TensorFlow, model inference, fine-tuning), Linux CLI familiarity (bash, package managers, permissions), and recommended prerequisite courses or resources for readers lacking background
- **FR-020**: The book MUST provide a glossary defining all domain-specific terms on first use with page references (e.g., "URDF (Unified Robot Description Format) - see p.45")
- **FR-021**: The book MUST include safety and ethics considerations in robotics deployment (Chapter 19 or distributed across chapters): fail-safe behaviors, human-in-the-loop control, sim-to-real gap awareness, responsible AI deployment, potential societal impacts
- **FR-022**: The book MUST provide instructor resources in a separate repository/appendix: lecture slide templates (one per chapter), assignment prompts with solution sketches, grading rubrics, quiz/exam question banks (50+ questions), and suggested 13-week course syllabus
- **FR-023**: The book MUST document a clear update and errata process: dedicated GitHub repository for issue tracking, quarterly review of software version compatibility, errata webpage linked from book frontmatter, and commitment to maintain code examples for minimum 3 years post-publication
- **FR-024**: The book MUST include case studies or interviews with industry practitioners from humanoid robotics companies (Figure AI, Tesla Optimus, Agility Robotics, Boston Dynamics) discussing real-world deployment challenges and lessons learned
- **FR-025**: The book MUST provide cloud deployment alternatives for readers without local GPU access: AWS EC2 setup guide (g5.xlarge or similar), NVIDIA Omniverse Cloud configuration, estimated costs (<$300/quarter for part-time use), and performance comparison vs. local RTX 4070 Ti

### Key Entities

- **Book**: The complete deliverable comprising 18-22 chapters, appendices, companion code repositories, Docusaurus web version, and instructor resources
- **Chapter**: A self-contained unit covering one major topic with learning objectives, theory (30-40%), hands-on lab (40-50%), code walkthrough, end-of-chapter project, and references (10+ citations)
- **Part**: A thematic grouping of 3-4 chapters (6 parts total: Foundations, Simulation, Perception, VLA Models, Locomotion, Capstone)
- **Hands-On Lab**: A practical exercise within a chapter requiring 4-12 hours to complete, with starter code, step-by-step instructions, validation criteria, and troubleshooting guide
- **Companion Repository**: A GitHub repo (MIT license) containing tested code for a major topic, with README, requirements.txt/pyproject.toml, CI pipeline, and verified compatibility with book's software stack
- **Hardware Tier**: One of three costed configurations (Budget <$1200, Mid $2500-3500, Premium $5000+) with exact part numbers, performance benchmarks, and vendor links
- **Software Stack**: The locked set of tool versions (ROS 2 Iron/Jazzy, Isaac Sim 2024.2+, Isaac ROS 3.0+, Nav2, Ubuntu 22.04/24.04) verified to work together
- **Capstone Project**: The culminating end-to-end demo integrating Whisper + LLM + Nav2 + VLA + Isaac Sim with performance target ≥12 Hz on Jetson Orin Nano 8GB
- **Docusaurus Site**: The web-based book version with search, syntax highlighting, interactive elements, and mobile-responsive design
- **Instructor Resources**: Supplementary materials for course adoption including slide templates, assignment prompts, grading rubrics, and question banks

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: At least 3 independent student testers (external volunteers) can complete any chapter's lab section in <15 hours using only the book as reference, with all code examples running successfully on their hardware (RTX 4070 Ti class or cloud equivalent)
- **SC-002**: 100% of companion GitHub repositories have passing CI pipelines on Ubuntu 22.04 + ROS 2 Iron + Isaac Sim 2024.2 verified weekly during writing and monthly for 3 years post-publication
- **SC-003**: The capstone end-to-end demo (Whisper + LLM + Nav2 + VLA) runs at ≥12 Hz on Jetson Orin Nano 8GB with <2 GB RAM usage when following Chapter 20's deployment guide
- **SC-004**: All URDF/SDF robot models validate with `gz sdf check` and `isaac-sim validator` with zero errors
- **SC-005**: The book contains 550-650 pages and 160,000-190,000 words (including code listings, excluding appendices)
- **SC-006**: Every chapter includes minimum 10 citations in APA 7th edition format, with at least 70% of citations from 2020 or later (ensuring currency)
- **SC-007**: Hardware procurement guide (Appendix A) has 100% valid purchase links from at least 2 vendors per component verified within 30 days of publication date
- **SC-008**: Zero broken external links in the book's web version (Docusaurus) at publication time, with quarterly link-checking automated via CI
- **SC-009**: At least 2 university instructors pilot the book in their courses and provide structured feedback on chapter pacing, difficulty, and completeness
- **SC-010**: Performance benchmarks are reproducible within ±15% on reference hardware: Isaac Sim ≥30 FPS (Budget tier), VLA inference ≥10 Hz (Jetson Orin Nano), Nav2 planning <500ms
- **SC-011**: The book's Docusaurus web version achieves 90+ Lighthouse scores for performance, accessibility, and SEO
- **SC-012**: All code snippets in the book pass Black formatting and flake8 linting with zero violations
- **SC-013**: The quickstart guide (Chapter 2 or Appendix) enables a reader to go from fresh Ubuntu 22.04 install to running first ROS 2 + Isaac Sim demo in <4 hours as verified by 3 independent testers
- **SC-014**: The book includes at least 3 case studies or practitioner interviews from industry (Figure AI, Tesla, Agility Robotics, Boston Dynamics, or equivalent)
- **SC-015**: Instructor resources include minimum 50 quiz/exam questions covering all 6 parts with answer keys and difficulty ratings

### Non-Functional Success Criteria

- **Usability**: Readers with specified prerequisites (Python, basic DL, Linux CLI) can self-study without instructor support, verified by tester feedback
- **Maintainability**: Code examples are modular and version-pinned (requirements.txt with exact versions) to ensure long-term reproducibility
- **Accessibility**: All figures have alt text, code uses colorblind-safe syntax highlighting, equations have plain-text explanations (WCAG 2.1 AA compliant)
- **Portability**: Book source is Markdown → LaTeX → PDF with separate Leanpub/Kindle builds, Docusaurus for web, all from single source
- **Scalability**: Content supports both 10-week quarter (accelerated pacing: 2 chapters/week) and 13-week semester (standard pacing: 1.5 chapters/week) course structures
- **Vendor Neutrality**: Where possible, provide alternatives (e.g., multiple VLA models, cloud providers) to avoid lock-in

## Assumptions

1. **Reader Hardware Access**: Readers either have local GPU (RTX 4070 Ti class minimum), access to university lab workstations, or budget for cloud instances (<$300/quarter)
2. **Software Stability**: ROS 2 Iron (LTS until 2027) and Jazzy (LTS until 2029) maintain backward compatibility; Isaac Sim 2024.2-2026.x versions remain compatible with core APIs
3. **Open-Source VLA Availability**: At least one major open-source VLA model (OpenVLA, RT-2-X, Octo) remains publicly available with pre-trained weights through 2027
4. **Jetson Orin Availability**: Jetson Orin Nano 8GB ($249) and Orin NX 16GB remain available for purchase at stable pricing through 2026-2027
5. **Humanoid Platform Proxy**: Book can use a proxy platform (e.g., Unitree Go2 quadruped, simplified humanoid URDF) if commercial humanoids (G1, Figure 02) are too expensive or unavailable for typical readers
6. **Ubuntu LTS Support**: Ubuntu 22.04 LTS (supported until 2027) and 24.04 LTS (supported until 2029) are the primary target platforms; Windows/macOS support is best-effort only
7. **GitHub Availability**: GitHub remains accessible for hosting code repositories and CI/CD pipelines; alternatives (GitLab, Bitbucket) may be considered if GitHub policies change
8. **English Language**: Book is written in English; translations are out of scope but community contributions may be accepted post-publication
9. **Internet Access**: Readers have internet for package downloads (Isaac Sim ~50 GB, ROS packages ~5 GB), cloud instances (if chosen), and companion repository cloning; fully air-gapped setups are not supported
10. **Publication Timeline**: First draft completed in 12-14 months (Q1-Q2 2027), technical review 2-3 months, production 3-4 months, publication by Q4 2027 or Q1 2028
11. **Instructor Adoption**: At least 5-10 universities adopt the book within first year post-publication, driving errata reports and community engagement
12. **Licensing Clarity**: MIT license for code is acceptable to readers and institutions; companion repositories can be forked and modified for commercial/educational use
13. **Performance Hardware**: Readers attempting Jetson deployment have Jetson Orin Nano 8GB minimum; older Jetson Xavier or Nano 4GB are unsupported
14. **Cloud Pricing Stability**: AWS EC2 GPU instance pricing (g5.xlarge ~$1/hour) remains within $300/quarter student budget with spot instances and auto-shutdown
15. **Case Study Access**: Industry practitioners are willing to contribute case studies or interviews (2-3 pages each) for inclusion in the book

## Out of Scope

- **Beginner Programming**: The book assumes Python proficiency; it does not teach basic programming, data structures, or algorithms
- **Deep Learning Fundamentals**: The book assumes familiarity with neural networks, backpropagation, and model training; it does not provide a full DL course
- **Traditional Robotics Theory**: Advanced control theory (optimal control, Lyapunov stability), kinematics derivations, and dynamics equations are covered only as needed; readers needing deep theory are referred to classic robotics textbooks
- **Non-Humanoid Platforms**: Drones, autonomous vehicles, industrial robotic arms, and quadrupeds are mentioned only as contrasts to humanoids; they are not primary focus
- **Custom Hardware Design**: The book does not cover actuator design, sensor fabrication, PCB design, or mechanical engineering aspects of building humanoids from scratch
- **Regulatory Compliance**: Safety certifications (ISO 13482, CE marking), regulatory approval, and commercialization processes are excluded
- **Multi-Robot Coordination**: Swarm robotics, multi-agent systems, and distributed coordination are optional advanced topics, not core curriculum
- **Production DevOps**: CI/CD for robotics fleets, Kubernetes orchestration, cloud robotics infrastructure, and production monitoring are excluded
- **Business and Entrepreneurship**: Commercialization strategies, business models, and startup advice are not covered
- **Real-Time Operating Systems**: RTOS topics, hard real-time guarantees, and embedded systems programming below the ROS 2 abstraction layer are excluded
- **Alternative Middleware**: ROS 1, YARP, LCM, and other middleware are mentioned only for context; the book focuses exclusively on ROS 2
- **Simulation Engines Not Covered**: Webots, CoppeliaSim, and other simulators are mentioned but not deeply covered; Isaac Sim is primary, with MuJoCo/PyBullet as occasional alternatives

## Dependencies

- **ROS 2 Iron Irwini or Jazzy Jalisco**: LTS distributions with support through 2027 (Iron) or 2029 (Jazzy)
- **NVIDIA Isaac Sim 2024.2+**: Compatible with Ubuntu 22.04/24.04, ROS 2 Iron/Jazzy, and RTX 4000-series GPUs
- **NVIDIA Isaac ROS 3.0+**: GPU-accelerated ROS packages for perception, navigation, and manipulation
- **Nav2 Navigation Stack**: Latest stable release compatible with ROS 2 Iron/Jazzy
- **Ubuntu 22.04 LTS or 24.04 LTS**: Supported through 2027 (22.04) and 2029 (24.04)
- **NVIDIA GPU Drivers**: 525.x+ supporting CUDA 12.x for RTX 4000-series on Ubuntu 22.04/24.04
- **Jetson Orin JetPack 5.x or 6.x**: Supporting ROS 2 Iron/Jazzy and Isaac ROS on Jetson Orin Nano/NX/AGX
- **Python 3.10+**: Required for ROS 2 Iron/Jazzy, Isaac Sim Python API, and modern ML frameworks
- **PyTorch 2.0+**: For VLA model inference, fine-tuning, and integration with Isaac Sim
- **OpenAI Whisper or Local Model**: Speech recognition for voice-controlled demos (API or self-hosted)
- **LLM API or Local Model**: GPT-4, Claude, Llama 3+, or similar for task planning in capstone
- **Open-Source VLA Models**: OpenVLA, RT-2-X, Octo, or equivalent with pre-trained weights and inference code
- **Docusaurus 3.x**: For web-based book version with search, syntax highlighting, and mobile support
- **GitHub Actions**: For CI/CD pipelines testing code repositories on Ubuntu 22.04 + RTX 4090 runners
- **LaTeX Distribution**: TeX Live or MiKTeX for PDF generation from Markdown source
- **Git and GitHub**: For distributing code repositories, tracking errata, and community contributions

## Risks and Mitigations

### Risk 1: Rapid Software Deprecation (Isaac Sim, ROS 2, VLA Models)

**Impact**: If Isaac Sim 2025.x breaks compatibility with 2024.2 code, or ROS 2 Jazzy deprecates APIs used in the book, all examples may fail, requiring emergency updates.

**Likelihood**: Medium (robotics software evolves rapidly; breaking changes occur despite LTS promises)

**Mitigation**:
- Pin exact software versions in all companion repositories (requirements.txt, Docker images)
- Test code weekly against both current stable (Iron + Isaac Sim 2024.2) and bleeding-edge (Jazzy + Isaac Sim 2025.x) versions
- Maintain version-specific branches in companion repos (e.g., `ros2-iron`, `ros2-jazzy`)
- Establish relationship with NVIDIA Isaac Sim team for early access to beta releases and deprecation notices
- Include migration guides in errata/updates section for major version transitions

### Risk 2: Hardware Unavailability or Price Volatility

**Impact**: If RTX 4070 Ti GPUs become unavailable or Jetson Orin Nano prices double, readers cannot procure recommended hardware, blocking hands-on learning.

**Likelihood**: Medium (GPU markets are volatile; supply chain disruptions and generational transitions cause shortages)

**Mitigation**:
- Provide performance benchmarks for multiple GPU generations (RTX 4000-series and eventual RTX 5000-series)
- Document cloud alternatives (AWS, NVIDIA Omniverse Cloud) with cost estimates and setup guides
- Test code on lower-tier hardware (RTX 4060, integrated Jetson Orin in dev kits) to define minimum viable specs
- Maintain relationships with educational hardware suppliers (NVIDIA, system integrators) for bulk purchase discounts
- Update Appendix A (Hardware Buyer's Guide) quarterly for first year post-publication

### Risk 3: VLA Model Availability or Licensing Changes

**Impact**: If OpenVLA, RT-2-X, or Octo are taken offline, license-restricted, or weights removed, students cannot complete VLA chapters (14-15).

**Likelihood**: Low-Medium (research labs occasionally remove models; corporate acquisitions can restrict access)

**Mitigation**:
- Archive pre-trained VLA weights in institutional storage (with license compliance) as backup
- Provide 2-3 alternative VLA models in Chapter 14-15 (if OpenVLA unavailable, use RT-2-X or Octo)
- Design VLA code to be model-agnostic (standardized interfaces) so models are swappable
- Monitor VLA research community and Hugging Face for new open-source releases
- Include fallback to fine-tuning smaller open-source models (Llama-3.1-8B) if no VLA available

### Risk 4: Student Hardware Diversity and Compatibility Issues

**Impact**: If students attempt to run code on unsupported platforms (Windows, macOS, AMD GPUs), they encounter driver conflicts, missing packages, and frustration.

**Likelihood**: High (students will try unsupported setups despite prerequisites)

**Mitigation**:
- Clearly document supported platforms in Chapter 1 and on book cover: Ubuntu 22.04/24.04 + NVIDIA GPUs only
- Provide troubleshooting section in Appendix C for common unsupported setup issues (WSL2, Docker on macOS, AMD GPU workarounds)
- Recommend cloud fallback (AWS, Omniverse Cloud) for students on unsupported hardware
- Community contributions for Windows/macOS compatibility guides are welcome but not officially supported
- Test all code on clean Ubuntu 22.04/24.04 VMs to ensure reproducibility

### Risk 5: Book Length Exceeds Target (Content Creep)

**Impact**: If chapters expand beyond planned word counts, book reaches 800+ pages, making it too expensive to print and overwhelming for readers.

**Likelihood**: High (technical books often exceed initial estimates; authors tend to add content)

**Mitigation**:
- Enforce strict word count budgets per chapter (tracked in writing plan: 7,000-10,000 words/chapter)
- Move tangential content to appendices or companion blog posts (linked from book)
- Prioritize depth over breadth: cover fewer topics thoroughly rather than many topics superficially
- Conduct mid-project review at 50% completion (10 chapters) to assess pacing and adjust scope
- Consider splitting into Volume 1 (Foundations + Simulation) and Volume 2 (VLA + Locomotion + Capstone) if length exceeds 700 pages

### Risk 6: Instructor Adoption Lower Than Expected

**Impact**: If <5 universities adopt the book in Year 1, sales are low, community feedback is sparse, and errata discovery is delayed.

**Likelihood**: Medium (textbook market is competitive; adoption requires active outreach)

**Mitigation**:
- Conduct pre-publication outreach to 20-30 robotics professors for early review copies and feedback
- Offer free instructor resources (slides, assignments, rubrics) to incentivize adoption
- Present the book at academic conferences (ICRA, IROS, RSS) and workshops
- Provide discounted bulk pricing for university bookstores
- Engage with online robotics communities (ROS Discourse, Reddit r/robotics, Twitter/X) to build awareness

### Risk 7: Capstone Complexity Exceeds Student Capabilities

**Impact**: If the end-to-end voice-controlled humanoid capstone (Chapter 20) is too ambitious, students fail to complete it, leading to frustration and negative reviews.

**Likelihood**: Medium (integrating 6+ systems—Whisper, LLM, Nav2, VLA, Isaac Sim, Jetson—is challenging)

**Mitigation**:
- Provide incremental capstone milestones in Chapter 20: voice transcription working → LLM planning working → navigation working → manipulation working → full integration
- Offer "capstone-lite" alternative with reduced scope (scripted commands instead of LLM, or navigation-only without manipulation)
- Include detailed debugging guide for common integration failures (ROS 2 topic mismatches, timing issues, memory limits)
- Test capstone with 5+ external beta readers before publication to calibrate difficulty
- Provide reference implementation in companion repository as fallback for stuck students

### Risk 8: Errata and Update Maintenance Burden

**Impact**: If software versions change frequently post-publication, maintaining errata, updated code, and compatibility notes becomes unsustainable.

**Likelihood**: High (robotics software evolves rapidly; maintenance is ongoing)

**Mitigation**:
- Commit to quarterly reviews of software compatibility for first year, biannual thereafter
- Automate link checking and CI testing to detect breakage early
- Build community around the book (GitHub Discussions, Discord server) to crowdsource issue reports
- Designate 1 maintainer (author or contributor) responsible for triaging issues and coordinating fixes
- Plan for potential "2nd edition" in 3-4 years with major updates if fundamental tools change (e.g., ROS 3, Isaac Sim 2027)

## Notes

- This specification defines the **content and structure** of the book, not the writing process itself (writing workflow is covered in the implementation plan)
- The book is designed as both a textbook (for university courses) and a self-study guide (for industry engineers and self-learners)
- **Writing approach**: Research-concurrent + code-concurrent (all code tested weekly on real hardware during writing, not just at end)
- **Primary authorship model**: Single author or small team (2-3 co-authors) to ensure consistency; community contributions for errata/updates post-publication
- **Publication model**: Traditional publisher (O'Reilly, Manning, No Starch Press) for print/ebook + self-published Leanpub version for faster updates + Docusaurus web version (free/open)
- **Revenue model**: Book sales + potential bulk licensing to bootcamps/universities + optional paid workshops/courses based on book content
- **Timeline**: 12-14 months first draft (Phase 1-6 writing), 2-3 months technical review (beta readers + industry reviewers), 3-4 months production (copyediting, layout, indexing), target publication Q4 2027 or Q1 2028
- **Code repository structure**: Monorepo with subfolders per chapter (chapter-03-ros2-basics, chapter-08-isaac-sim, etc.) OR separate repos per major topic (decision to be documented in plan.md)
- **Companion website**: Docusaurus-based site hosted on GitHub Pages or Vercel with: full book content (web version), searchable code examples, errata/updates, discussion forum (GitHub Discussions), and links to purchase print/ebook
- **Accessibility commitment**: WCAG 2.1 AA compliance for web version, alt text for all figures, colorblind-safe code highlighting, plain-text equation explanations
- **License for book content**: Traditional copyright for prose (publisher retains rights) + Creative Commons BY-NC-SA 4.0 for web version (if dual-published) + MIT for all code examples and repositories
- **Figures and diagrams**: Mix of custom-created (Inkscape, draw.io, Blender renders) + screenshots (Isaac Sim, RViz, Gazebo) + cited third-party figures (with permissions)
- **Testing infrastructure**: GitHub Actions runners with GPU support (RTX 4090 or cloud GPU instances) for CI testing all code weekly; estimated cost $200-500/month during writing phase
