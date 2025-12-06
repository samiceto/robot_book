---
id: 0002
title: Constitution Amendment v1.1.0 Add Code Quality and Equity Standards
stage: constitution
date: 2025-12-06
surface: agent
model: claude-sonnet-4-5
feature: none
branch: master
user: Sami Ullah
command: /sp.constitution
labels: ["constitution", "amendment", "code-quality", "equity", "accessibility", "realism"]
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
Review my Constitution at .specify/memory/constitution.md and improve it:

1. Are all standards testable (not vague)?
   - ❌ Vague: "Papers should be well-written"
   - ✅ Testable: "Flesch-Kincaid grade 10-12; active voice 75%+ of time"

2. Did I cover essential categories?
   - Citation accuracy
   - Source verification
   - Writing clarity
   - Plagiarism checking
   - Review process

3. Are any standards unrealistic?

Suggest 2-3 concrete improvements.

[After review, user requested:]
improve as you suggested the improvements .specify/memory/constitution.md
```

## Response snapshot

**Version bump**: 1.0.0 → 1.1.0 (MINOR) - Added new standards and material expansions

**Three improvements implemented**:

### 1. Code Quality & ROS Package Standards (NEW Section)
**Problem**: Constitution specified hardware/tools but ignored software engineering practices critical for ROS development.

**Added enforcement criteria**:
- Package Structure: `package.xml`, `setup.py`/`CMakeLists.txt`, `launch/`, `README.md` all mandatory
- Documentation: Module-level docstrings with topics/parameters/purpose specifications
- Code Style: Python must pass `ruff check` with ≤5 violations per 100 lines
- Testing: ≥2 unit tests required per package
- Version Control: ≥3 meaningful Git commits (not "final", "fix", "asdf")

**Testability**: Automated CI checks for linting/test count; 0-5 point manual rubric for docs

### 2. Equitable Hardware Access (NEW Section)
**Problem**: Budget tiers documented but no policy for students who can't afford ANY tier, lab shortages, or equipment failures.

**Added enforcement criteria**:
- Lab Loaners: ≥0.3× enrollment ratio (30 Jetsons for 100 students)
- Cloud Credits: Minimum $200/student for AWS/Azure/Paperspace
- Hardship Exception Process: 3-step documented workflow for students unable to afford Budget tier
- Equipment Failure Protocol: 1-week auto-extension + priority loaner access
- Exception for institutional Windows/Mac M1 machines via cloud-only path

**Testability**: Track loaner checkout logs, cloud credit redemption, hardship requests (<10% = acceptable)

### 3. Revised Absolutist Standards for Realism
**Problem**: Several "100%", "zero", "every module" requirements unachievable due to edge cases.

**Changes made**:
- **Principle I (Real-World Readiness)**: Added "≥1 hands-on task demonstrating deployment" with testability checklist
- **Principle II (Hands-On)**: Defined "working artifact" as "≥80% of specified behaviors OR passes automated checks"
- **Principle III (Sim-to-Real)**: Changed "Every module" → "Weeks 3-13" (Weeks 1-2 are foundations)
- **Graded Assignments**: 100% → ≥95% (12 of 13 weeks); allows 1 theory week if followed by implementation
- **Capstone "zero intervention"** → "minimal manual intervention" with explicit boundaries (world load, robot spawn, voice trigger OK; topic publishing/teleoperation/debugging NOT OK)
- **SC-001 Environment Setup**: <5% immediately → ≤10% by Week 3 (acknowledges Weeks 1-2 ramp-up complexity)
- **SC-002 Capstone**: Clarified ≤3 manual interventions per graded run permitted
- **Primary OS Constraint**: Added exception process for institutional machines/Mac M1

**Testability**: All revised metrics remain quantifiable but account for realistic constraints

## Outcome

- ✅ Impact: Constitution v1.1.0 amended; added 2 new Key Standards sections + revised 8 vague/unrealistic clauses to be measurable and achievable
- 🧪 Tests: N/A (governance document defines testing standards for course content)
- 📁 Files: Updated `.specify/memory/constitution.md` with comprehensive sync impact report
- 🔁 Next prompts: Create assignment template with ROS package checklist; design hardship request form; draft grading rubric incorporating code quality criteria
- 🧠 Reflection: Amendment addresses critical gaps identified in testability audit (code quality, equity, absolutist language). Constitution now balances rigor with realism while maintaining industry-aligned standards.

## Evaluation notes (flywheel)

- Failure modes observed: Original v1.0.0 had vague language ("explicitly teach", "map to deployable skills"), unrealistic absolutes (100% hardware, <5% setup help, zero intervention), and missing critical categories (code quality, equity policy, collaboration guidelines)
- Graders run and results (PASS/FAIL): Testability audit identified 4 vague principles, 3 missing categories, 4 unrealistic standards. Post-amendment: All principles have testability clauses, 2 critical categories added (code quality + equity), absolutist language revised to realistic thresholds.
- Prompt variant (if applicable): User-driven iterative improvement (review → suggest → implement)
- Next experiment (smallest change to try): Validate updated constitution against sample Week 1 assignment to verify code quality checklist is actionable; pilot hardship request form with 3-5 hypothetical student scenarios to identify edge cases
