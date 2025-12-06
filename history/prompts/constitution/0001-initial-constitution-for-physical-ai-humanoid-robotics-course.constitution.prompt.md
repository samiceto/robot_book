---
id: 0001
title: Initial Constitution for Physical AI Humanoid Robotics Course
stage: constitution
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5
feature: none
branch: master
user: Sami Ullah
command: /sp.constitution
labels: ["constitution", "robotics", "education", "curriculum", "hardware", "ros2", "isaac-sim"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - None (constitution document)
---

## Prompt

```text
Project: Constitution for the "Physical AI & Humanoid Robotics" course

Core principles:
- Real-world readiness: every learning objective must map to deployable skills on actual humanoid/embodied platforms
- Hands-on over theory: students must build, simulate, and (when possible) deploy on physical hardware
- Sim-to-Real pipeline: every module must explicitly teach the bridge from simulation (Isaac Sim/Gazebo) → edge device (Jetson) → real robot
- Future-proof stack: prioritize currently industry-dominant tools (ROS 2 Humble/Iron, NVIDIA Isaac Sim/ROS, Nav2, modern VLA approaches)
- Progressive complexity: start simple (single node → full autonomous humanoid with voice commands)

Key standards:
- All tools and versions must be explicitly stated and currently supported (as of 2025)
- Every module must contain at least one graded hands-on assignment that runs on student hardware/cloud
- Capstone project must demonstrate end-to-end autonomy: voice command → LLM planning → ROS 2 action sequence → navigation + manipulation in simulation and (optional) real robot
- Hardware recommendations must include exact model numbers, minimum specs, and realistic cost tiers
- Safety and ethical deployment of physical robots must be addressed (emergency stops, teleoperation fallback, responsible AI guidelines)

Constraints:
- Course duration: 13 weeks maximum
- Target audience: advanced undergrad or master's students with prior Python + basic ML experience (no prior robotics required)
- Primary OS: Ubuntu 22.04 LTS (dual-boot or native Linux mandatory for frictionless experience)
- GPU requirement: NVIDIA RTX 4070 Ti or higher (or equivalent cloud instance) for Isaac Sim
- Budget transparency: provide three realistic lab tiers (budget < $1k, mid-range $3k–$5k, premium $15k+ per student/group)
- No deprecated tools (e.g., ROS 1, Gazebo Classic, old NVIDIA Jetson TX1/TX2, MoveIt1 without MoveIt2)

Success criteria:
- Students can independently set up a full ROS 2 + Isaac Sim + Jetson development environment from scratch
- Capstone robot (simulated or real) successfully completes: "Pick up the red cup on the table and bring it to me" via natural voice command with zero manual intervention
- 100% of graded assignments run on student-owned or lab-provided hardware (no "lecture-only" modules)
- Course syllabus, weekly schedule, hardware guide, and grading rubric are complete and immediately usable by any university or bootcamp instructor in 2026+
- All recommended hardware kits are purchasable today and supported by manufacturers until at least 2028
```

## Response snapshot

Created constitution v1.0.0 for Physical AI & Humanoid Robotics course with 5 core principles:

1. **Real-World Readiness** - All learning objectives map to deployable skills on actual platforms
2. **Hands-On Over Theory** - Primary learning mode is active construction and debugging
3. **Sim-to-Real Pipeline** - Explicit teaching of simulation → edge device → real robot bridge
4. **Future-Proof Stack** - ROS 2 Humble/Iron, NVIDIA Isaac Sim 4.x+, Nav2, MoveIt2, Jetson Orin
5. **Progressive Complexity** - ROS 2 basics → single DOF → manipulation → navigation → VLA autonomy

Key Standards implemented:
- Tool & Version Explicitness (exact SKUs, versions, commands)
- Graded Hands-On Assignments (minimum one per week, hardware-verified artifacts)
- Capstone End-to-End Autonomy (voice → LLM → ROS 2 → nav + manipulation)
- Hardware Recommendations Transparency (3 tiers: <$1k, $3k-$5k, $15k+)
- Safety & Ethical Deployment (emergency stops, teleoperation fallback, responsible AI)

Constraints documented:
- 13-week semester, Ubuntu 22.04 LTS native, RTX 4070 Ti+ GPU, no deprecated tools

Success Criteria (5 measurable outcomes):
- SC-001: <5% students need instructor help beyond office hours for environment setup
- SC-002: 100% passing students complete capstone autonomy demo
- SC-003: 100% graded assignments run on hardware
- SC-004: Curriculum immediately usable by external instructors in 2026+
- SC-005: Hardware purchasable today with support until 2028+

## Outcome

- ✅ Impact: Constitution v1.0.0 ratified; comprehensive governance for robotics curriculum with hardware-first, industry-aligned approach
- 🧪 Tests: N/A (governance document)
- 📁 Files: Created `.specify/memory/constitution.md` with sync impact report
- 🔁 Next prompts: Create course syllabus (`/sp.specify`), define Week 1 module, hardware guide, capstone spec
- 🧠 Reflection: Constitution balances pedagogical rigor (progressive complexity, hands-on) with industry reality (ROS 2, Jetson Orin, sim-to-real). Explicit version requirements and 3-tier budget transparency prevent common course failure modes (deprecated tools, inaccessible hardware).

## Evaluation notes (flywheel)

- Failure modes observed: None (initial creation)
- Graders run and results (PASS/FAIL): N/A (constitution is the grading standard)
- Prompt variant (if applicable): N/A (first version)
- Next experiment (smallest change to try): Validate constitution against existing robotics course syllabi to identify missing constraints or overly prescriptive requirements; consider adding deployment safety principle if Week 1 content reveals gaps
