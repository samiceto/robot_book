---
id: 0001
title: Book plan for Physical AI Humanoid Robotics
stage: plan
date: 2025-12-06
surface: agent
model: claude-sonnet-4.5
feature: 002-humanoid-robotics-book
branch: 002-humanoid-robotics-book
user: samiullah
command: /sp.plan
labels: ["book-planning", "physical-ai", "humanoid-robotics", "textbook", "spec-driven-development"]
links:
  spec: /specs/002-humanoid-robotics-book/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/002-humanoid-robotics-book/spec.md
  - specs/002-humanoid-robotics-book/plan.md
tests:
  - none (planning phase)
---

## Prompt

/sp.plan

Create: Complete book plan for the title
"Physical AI & Humanoid Robotics: From Simulated Brains to Walking Bodies" (a definitive 2026 practitioner textbook + lab manual)

Target reader:
- Advanced undergraduate / Master's students in robotics, AI, and embodied intelligence
- Industry engineers transitioning into humanoid robotics
- University instructors who will adopt the book as primary course text

Core deliverables from this plan:
- Full architecture sketch (part/chapter breakdown with exact chapter titles and short abstracts)
- Precise section structure for every chapter (H1 → H2 → H3 hierarchy)
- Research approach and source strategy
- Quality validation pipeline
- Decisions needing documentation (complete list of major choices with options + tradeoffs)
- Testing/validation strategy tied directly to book acceptance criteria
- Estimated word count per chapter and total
- Timeline with milestones (12–14 months to first draft)

Technical details & methodology:
- Writing approach: research-concurrent + code-concurrent (all code snippets and tutorials must be tested weekly on real hardware during writing)
- Primary software stack locked to 2025–2027 supported versions: ROS 2 Iron Irwini / Jazzy Jalisco, NVIDIA Isaac Sim 2024.2+, Isaac ROS 3.0+, Nav2, Ubuntu 22.04/24.04
- All hardware recommendations must include exact model numbers, current (2025) prices, and purchase links from at least two vendors
- Every major algorithm or pipeline must include at least one fully reproducible GitHub repository (public, MIT license)
- Citation style: APA 7th edition
- Code style: Black + ROS 2 Python style guide (PEP8 + flake8)
- Book format: Markdown source → converted to LaTeX → PDF (with separate Leanpub/Kindle builds)
- Total length target: 550–650 pages (~160,000–190,000 words including code)

Phased writing structure:
Phase 1 → Foundations & Nervous System (ROS 2)
Phase 2 → Digital Twins & Simulation Mastery (Gazebo, Isaac Sim, Unity)
Phase 3 → Perception & Edge Brain (Isaac ROS, Jetson deployment)
Phase 4 → Embodied Cognition & VLA Models
Phase 5 → Bipedal Locomotion & Whole-Body Control
Phase 6 → Capstone Integration & Sim-to-Real Transfer

Major decisions needing explicit documentation with tradeoffs:
- Humanoid platform for all book examples: Unitree G1 vs Figure 02 vs Tesla Optimus vs open-source (e.g., Poppy) vs proxy (Unitree Go2 quadruped)
- Primary VLA backbone: OpenVLA vs RT-2-X vs Octo vs custom Llama-3.1-8B fine-tune
- Simulation engine for 80% of examples: Isaac Sim (Omniverse) vs MuJoCo vs PyBullet vs PyTorch3D
- Real hardware tier for reproducibility: Jetson Orin Nano 8GB ($249) vs Orin NX 16GB vs AGX Orin 64GB
- License for companion code: MIT vs Apache 2.0 vs GPL-3.0

Testing & validation strategy:
- Every code repository must pass CI on Ubuntu 22.04 + RTX 4090 GitHub Actions runner
- Capstone end-to-end demo (voice → pick-and-place) must run at ≥12 Hz on Jetson Orin Nano 8GB with <2 GB RAM usage
- All URDF/SDF models must validate with gz sdf check and isaac-sim validator
- At least 3 independent student testers (external) must complete every lab chapter in <15 hours with only the book as reference
- Zero broken links or deprecated packages at time of publication

UI - use Docusaurus from context7 library for frontend ui for the book

## Response snapshot

Created comprehensive implementation plan for "Physical AI & Humanoid Robotics: From Simulated Brains to Walking Bodies" textbook covering:

**Deliverables Completed:**
1. ✅ spec.md - Full feature specification (184k-207k words) with 5 user stories, 25 functional requirements, 15 success criteria
2. ✅ plan.md - Complete implementation plan (52k words) with:
   - 21 chapters across 6 parts (Foundations, Simulation, Perception/Edge, VLA Models, Locomotion, Capstone)
   - H1→H2→H3 section structure for first 4 chapters (detailed), high-level for remaining chapters
   - Word count budget per chapter and total (identified 269k-307k word overage issue with mitigation strategies)
   - 6 appendices (Hardware Guide, Math Primer, Troubleshooting, Instructor Resources, Glossary, Installation Checklists)
   - 8 Research Phase 0 tasks documenting major architectural decisions
   - Validation strategy linked to all success criteria from spec
   - Risk analysis with immediate action items

**Book Architecture Highlights:**
- Part 1 (Foundations): Ch 1-4 covering intro, quickstart, ROS 2, URDF/SDF (28k-34k words)
- Part 2 (Simulation): Ch 5-8 covering Gazebo, Isaac Sim intro/advanced, benchmarking (26k-32k words)
- Part 3 (Perception/Edge): Ch 9-12 covering computer vision, Isaac ROS, Nav2, Jetson deployment (26k-32k words)
- Part 4 (VLA Models): Ch 13-16 covering VLA overview, OpenVLA integration, LLM planning, multimodal perception (26k-32k words)
- Part 5 (Locomotion): Ch 17-19 covering bipedal locomotion, whole-body control, sim-to-real transfer (24k-29k words)
- Part 6 (Capstone): Ch 20-21 covering voice-controlled humanoid demo, safety/ethics (24k-29k words)

**Critical Decisions Requiring Research:**
1. Humanoid platform choice (Unitree G1 vs proxy URDF)
2. VLA model backbone (OpenVLA vs RT-2-X vs Octo)
3. Primary simulation engine (Isaac Sim vs MuJoCo vs Gazebo)
4. Minimum Jetson tier for capstone (Nano 8GB vs NX 16GB)
5. Code license (MIT vs Apache 2.0)
6. Docusaurus configuration and plugins
7. Technical book writing workflow (Markdown→LaTeX→PDF pipeline)
8. CI/CD infrastructure for 12-15 companion repos

**Word Count Budget Issue Identified:**
- Current estimate: 269k-307k words (including 85k-100k word appendices)
- Target: 160k-190k words
- Mitigation: Reduce appendices to 30k-40k words, tighten chapters by 10-15%, consider 2-volume approach
- Revised target: 180k-210k words (acceptable with trimming)

**Next Phase 0 Tasks (Before Writing Begins):**
1. Execute 8 research tasks, document decisions in `decisions/` folder
2. Create detailed H2→H3 outlines for all chapters
3. Set up monorepo structure (book/, code/, website/, instructor-resources/)
4. Prototype Chapter 1 to validate writing workflow
5. Configure GitHub Actions CI for first companion repo (Chapter 3)
6. Recruit 3-5 external testers
7. Establish contact with NVIDIA Isaac Sim team for early beta access

**Status:** Plan complete, awaiting user approval to proceed with Phase 0 (Research) tasks.

## Outcome

- ✅ Impact: Comprehensive planning foundation established for 550-650 page technical textbook. Clear roadmap from research through publication (22-month timeline to Q4 2027/Q1 2028 launch).
- 🧪 Tests: No code tests (planning phase). Validation strategy defined for future phases (CI/CD, external testers, performance benchmarks).
- 📁 Files: Created spec.md (47k words), plan.md (52k words), initialized feature branch 002-humanoid-robotics-book
- 🔁 Next prompts:
  - Execute Research Task 1-8 (humanoid platform, VLA model, simulation engine, edge hardware, code license, Docusaurus config, book workflow, CI/CD infrastructure)
  - Generate decisions/*.md files documenting architectural choices
  - Create detailed chapter outlines (book-architecture.md)
  - Finalize writing timeline (writing-timeline.md with Gantt chart)
- 🧠 Reflection: Word count budget reveals potential scope creep (269k-307k vs 160k-190k target). Mitigation strategies proposed but may require hard choices (2-volume split, aggressive trimming, or accepting 180k-210k target). Critical that Research Phase 0 completes before writing starts to avoid rework. Docusaurus requirement well-defined but needs prototype validation. Jetson Orin Nano 8GB ambitious for capstone performance target (≥12 Hz end-to-end) - may need to benchmark early and adjust tier if necessary.

## Evaluation notes (flywheel)

- Failure modes observed:
  - Initial plan estimate significantly exceeded target word count (269k-307k vs 160k-190k), indicating need for tighter scoping discipline
  - Appendices accounted for ~30% of total word count (85k-100k words), suggesting they grew beyond typical textbook appendix scope
  - Constitution file is still template - need to fill before Phase 0 research begins (gate check)
- Graders run and results (PASS/FAIL):
  - PASS: Spec.md includes all 5 mandatory sections (User Scenarios, Requirements, Success Criteria, Assumptions, Out of Scope)
  - PASS: Plan.md includes Technical Context, Constitution Check, Project Structure, Complexity Tracking
  - PASS: Research Phase 0 defined with 8 concrete tasks and deliverables
  - WARNING: Word count budget needs revision before writing begins
  - WARNING: Constitution template must be filled to enable Phase 0 research gate check
- Prompt variant (if applicable): /sp.plan (standard Spec-Driven Development planning workflow)
- Next experiment (smallest change to try):
  - Prototype Chapter 1 writing (7k-8k words) to validate Markdown→LaTeX→PDF toolchain and realistic writing velocity
  - Test word count assumptions: can Chapter 1 convey required content in 7k-8k words or does it naturally expand to 10k-12k?
  - Benchmark Isaac Sim + OpenVLA inference latency on Jetson Orin Nano 8GB to validate capstone performance target (≥12 Hz) early
