---
id: 0001
title: Physical AI Robotics Capstone Course Specification
stage: spec
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5-20250929
feature: 001-robotics-capstone-course
branch: 001-robotics-capstone-course
user: system
command: /sp.specify
labels: ["education", "robotics", "course-design", "physical-ai", "humanoid", "specification"]
links:
  spec: specs/001-robotics-capstone-course/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-robotics-capstone-course/spec.md
  - specs/001-robotics-capstone-course/checklists/requirements.md
tests:
  - Validation checklist: All 14 items PASSED
---

## Prompt

Complete Specification for the "Physical AI & Humanoid Robotics" Capstone Course

Target audience:
- University department heads, lab directors, and bootcamp founders who want to launch a cutting-edge, industry-aligned Physical AI / Humanoid Robotics course in 2026
- Instructors and TAs who will actually teach and support the course

Focus:
- Deliver a fully executable 13-week capstone course that takes students from zero robotics experience to deploying a voice-controlled autonomous humanoid (simulated + optional real hardware)
- Heavy emphasis on the modern 2025–2026 embodied-AI stack: ROS 2 Humble/Iron, NVIDIA Isaac Sim + Isaac ROS, Nav2, Vision-Language-Action models, Jetson Orin edge deployment

Success criteria:
- The final specification package is immediately usable: any competent instructor can run the course in Q1 2026 with no missing pieces
- Contains complete week-by-week syllabus, slide deck outlines, 13+ graded assignment prompts, full grading rubrics, and setup tutorials
- Capstone demo works end-to-end: spoken command → Whisper → LLM planner → ROS 2 action sequence → navigation + manipulation (tested in Isaac Sim and on at least one physical platform)
- Provides three fully costed lab tiers (Budget <$1k, Mid $3–5k, Premium $15k+) with exact part numbers and purchase links that are still valid in 2025
- Includes cloud-native fallback path (AWS/NVIDIA Omniverse) that keeps total student cost under $300/quarter if no local GPU
- Every tool recommended is still officially supported in 2026 and has active community/package maintenance

Constraints:
- Total course length: maximum 13 weeks
- Weekly student time budget: 12–15 hours (including lectures, labs, and project work)
- Prerequisite knowledge: Python, basic deep learning; no prior ROS or robotics required
- Primary OS: Ubuntu 22.04 LTS (all setup instructions must assume Linux)
- GPU requirement for simulation: RTX 4070 Ti / 4080 / 4090 class or equivalent cloud instance
- All assignments must be doable on student-owned hardware or provided lab kits
- Capstone must run at ≥15 Hz real-time inference on Jetson Orin Nano/Orin NX 8–16 GB

Not building:
- A beginner-level robotics or ROS introduction course
- A purely theoretical AI embodiment course with no hardware deployment
- A course focused on non-humanoid platforms (drones, self-driving cars, fixed-base arms as primary target)
- Vendor-specific product marketing (no mandatory use of a particular commercial humanoid)
- Full research survey of all humanoid robots ever built
- Custom LLM training from scratch (use of existing open-source VLA models is allowed)

Deliver the complete, ready-to-launch course specification package in Markdown with embedded file structure.

## Response snapshot

Created comprehensive feature specification for Physical AI & Humanoid Robotics Capstone Course with:

**Specification Structure:**
- 5 prioritized user stories (P1-P3) covering course launch, student progression, capstone deployment, instructor support, and hardware procurement
- 30 detailed functional requirements (FR-001 to FR-030) defining complete course package deliverables
- 10 key entities (Course Package, Weekly Module, Hardware Lab Tier, Assignment, Capstone Project, etc.)
- 15 measurable success criteria (SC-001 to SC-015) plus 5 non-functional criteria
- 15 documented assumptions about student access, infrastructure, and tooling
- Comprehensive "Out of Scope" section excluding 14 specific areas
- 13 external dependencies with version requirements
- 8 identified risks with detailed mitigation strategies

**Key Features:**
- Three-tier hardware approach (Budget <$1k, Mid $3-5k, Premium $15k+) plus cloud fallback
- Week-by-week progression: ROS 2 basics (1-2) → Isaac Sim (3-4) → Nav2 (5-6) → VLA models (7-8) → voice/LLM integration (9-10) → Jetson deployment (11) → capstone (12-13)
- End-to-end capstone: voice command → Whisper → LLM planner → ROS 2 → Isaac Sim navigation & manipulation
- Comprehensive instructor support materials (slide decks, rubrics, troubleshooting guides, TA guides)

**Validation:**
- Created requirements.md checklist with 14 validation criteria
- All items PASSED validation (no [NEEDS CLARIFICATION] markers, all requirements testable, success criteria measurable and technology-agnostic)
- Specification ready for `/sp.plan` phase

**Files Created:**
- specs/001-robotics-capstone-course/spec.md (344 lines, comprehensive specification)
- specs/001-robotics-capstone-course/checklists/requirements.md (validation checklist)

## Outcome

- ✅ Impact: Created production-ready specification for launching Physical AI & Humanoid Robotics capstone course in Q1 2026. Defines complete course package deliverables enabling instructors to teach 13-week program from zero robotics to autonomous humanoid deployment.
- 🧪 Tests: Validation checklist completed with 14/14 items passed. All requirements testable, success criteria measurable and technology-agnostic, no clarifications needed.
- 📁 Files: Created spec.md (344 lines, 30 FRs, 15 SCs, 5 user stories, 8 risks) and requirements.md (validation checklist with detailed assessment)
- 🔁 Next prompts: `/sp.plan` to design implementation architecture for creating the course package. Consider `/sp.adr` after planning if significant architectural decisions identified.
- 🧠 Reflection: Specification successfully balances comprehensiveness with clarity. Three-tier hardware approach plus cloud fallback provides institutional flexibility. Strong risk analysis addresses operational challenges (vendor changes, dependency breakage, student prerequisites, capstone complexity, cloud costs). Success criteria appropriately mix quantitative (FPS, time, cost, percentages) with qualitative (surveys, confidence) metrics.

## Evaluation notes (flywheel)

- Failure modes observed: None. Specification completed without needing clarifications due to comprehensive user input and reasonable defaults for unspecified details.
- Graders run and results (PASS/FAIL): Requirements checklist validation - PASS (14/14 items). Content quality: PASS. Requirement completeness: PASS. Feature readiness: PASS.
- Prompt variant (if applicable): Standard /sp.specify workflow with agent-native PHR creation fallback (shell script approach)
- Next experiment (smallest change to try): After planning phase, evaluate if hardware procurement specifications need market research validation for 2025 part availability and pricing accuracy
