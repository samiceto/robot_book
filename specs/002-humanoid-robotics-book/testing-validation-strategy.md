# Testing and Validation Strategy

**Date**: 2025-12-06
**Status**: Phase 2 Complete
**Related Tasks**: T027
**Scope**: All 21 chapters, 12-15 companion code repositories, performance benchmarks

## Purpose

This document defines the comprehensive testing and validation strategy for the Physical AI & Humanoid Robotics textbook. It ensures:
1. All code examples run successfully on specified hardware tiers
2. Performance benchmarks are reproducible and validated
3. Technical claims are accurate and verifiable
4. Hardware compatibility is maintained across dual ROS 2 distributions (Iron + Jazzy)

---

## Testing Hierarchy

### Level 1: Author Self-Testing (Continuous)

**Scope**: Every code snippet, lab, and end-of-chapter project
**Frequency**: During chapter writing (Phase 3)
**Responsibility**: Primary author + co-authors

**Process**:
1. Write code example or lab
2. Test on local development machine (RTX 4090 + Ubuntu 24.04 + ROS 2 Jazzy)
3. Test on Budget tier hardware (RTX 4070 Ti + Ubuntu 22.04 + ROS 2 Iron)
4. Document results in chapter frontmatter:
   ```yaml
   ---
   tested_on:
     - hardware: RTX 4090 24GB, Ubuntu 24.04, ROS 2 Jazzy
       status: pass
       notes: FPS 125-140 (exceeds 120 FPS target)
     - hardware: RTX 4070 Ti 12GB, Ubuntu 22.04, ROS 2 Iron
       status: pass
       notes: FPS 35-40 (meets 30 FPS target)
   ---
   ```
5. Commit code to companion repository with passing CI badge

**Tools**:
- Local RTX 4090 workstation (primary development)
- Budget tier RTX 4070 Ti (validation machine)
- Jetson Orin Nano 8GB (edge deployment validation for Chapters 12, 20)

---

### Level 2: Continuous Integration (Automated)

**Scope**: All 12-15 companion code repositories
**Frequency**: Every commit to main branch, every pull request
**Responsibility**: GitHub Actions CI/CD pipeline

**Process**:
1. Trigger: Git push or pull request to `main` branch
2. CI runs matrix testing:
   - **CPU Tests** (GitHub-hosted runners, Ubuntu 22.04 + 24.04):
     - Code linting (Black, flake8)
     - ROS 2 build (`colcon build`)
     - Unit tests (`pytest` or `colcon test`)
   - **GPU Tests** (Self-hosted RTX 4090 runner):
     - Isaac Sim integration tests (30 FPS validation)
     - VLA model inference tests (≥12 Hz on RTX 4090)
     - Domain randomization tests (Replicator API)
3. Results reported in GitHub pull request
4. PR merge blocked if any tests fail

**CI Configuration** (already created in T007):
```yaml
# .github/workflows/ci.yml
name: CI Tests

on:
  push:
    branches: [main]
  pull_request:
    branches: [main]

jobs:
  cpu-tests:
    runs-on: ubuntu-latest
    strategy:
      matrix:
        ros_distro: [iron, jazzy]
        ubuntu_version: [22.04, 24.04]
    steps:
      - uses: actions/checkout@v4
      - name: Setup ROS 2
        uses: ros-tooling/setup-ros@v0.6
        with:
          required-ros-distributions: ${{ matrix.ros_distro }}
      - name: Install dependencies
        run: rosdep install --from-paths src --ignore-src -y
      - name: Build workspace
        run: colcon build --symlink-install
      - name: Run tests
        run: colcon test && colcon test-result --verbose

  gpu-tests:
    runs-on: [self-hosted, gpu, rtx4090]
    container:
      image: ghcr.io/your-org/isaac-sim-ros2-jazzy:latest
      options: --gpus all --rm
    steps:
      - uses: actions/checkout@v4
      - name: Run Isaac Sim tests
        run: pytest tests/test_isaac_sim_integration.py -v
      - name: Validate 30 FPS benchmark
        run: python tests/benchmark_isaac_sim.py --min-fps 30
```

**Validation Frequency**:
- **Per-commit**: Automated CI runs on every commit to main
- **Weekly**: Full test suite run on all 12-15 repos (scheduled GitHub Actions)
- **Monthly**: Compatibility test against latest Isaac Sim/ROS 2 releases

---

### Level 3: External Technical Review (Phase 4)

**Scope**: All 21 chapters, appendices, and companion code repositories
**Frequency**: Once after first draft complete (Weeks 49-52)
**Responsibility**: 5-8 external technical reviewers

**Reviewer Roles**:
1. **Technical Reviewers** (3-5 reviewers):
   - Verify code examples run as documented
   - Validate hardware specifications and performance claims
   - Test on their own hardware (if available: RTX GPUs, Jetson Orin)
   - Review mathematical derivations and algorithms

2. **Academic Reviewers** (2-3 professors):
   - Verify pedagogical approach and learning progression
   - Assess completeness of explanations
   - Check citation accuracy (APA 7th edition compliance)
   - Validate prerequisite assumptions

3. **Industry Practitioners** (2-3 engineers):
   - Verify real-world relevance of tool choices
   - Assess practicality of techniques for production deployment
   - Validate industry case studies (Appendix I)
   - Review career guidance (Chapter 21)

**Review Process**:
1. **Week 49**: Distribute chapters via GitHub pull requests
2. **Weeks 50-52**: Reviewers provide feedback (3-week window)
   - Technical reviewers: GitHub PR comments, issue reports
   - Academic reviewers: Google Docs comments, rubric evaluation
   - Industry practitioners: Email feedback, video calls
3. **Week 53**: Consolidate feedback, prioritize fixes (P1/P2/P3)
4. **Weeks 54-56**: Integrate feedback, resolve all P1 and P2 issues

**Deliverables**:
- 50+ technical review comments collected
- Prioritized feedback list with resolution status
- Updated chapters addressing all critical (P1) and important (P2) feedback

---

### Level 4: Student Tester Validation (Phase 4)

**Scope**: Hands-on labs (18 labs across Chapters 2-20, excluding Ch 13, 21)
**Frequency**: Once after first draft complete (Weeks 49-56)
**Responsibility**: 10-15 student testers (volunteers or paid)

**Tester Profile**:
- Advanced undergraduate or Master's students
- Python proficiency + basic deep learning knowledge (book prerequisites)
- Access to Budget tier hardware (RTX 4070 Ti) OR cloud GPU instance
- Willing to commit 20-30 hours over 4 weeks

**Testing Protocol**:
1. **Week 49**: Recruit 10-15 student testers
   - Advertise on university robotics lab mailing lists, r/robotics, ROS Discourse
   - Compensate with free book copy + $200-300 honorarium (or volunteer credit)
2. **Weeks 50-53**: Students complete assigned labs (4-5 labs per tester)
   - Each tester completes 4-5 labs from different parts of the book
   - Document time spent, difficulties encountered, unclear instructions
   - Submit feedback via Google Form survey
3. **Week 54**: Analyze feedback, identify common pain points
4. **Weeks 55-56**: Update chapters to address common issues

**Feedback Form** (Google Form):
```
Lab Tester Feedback Form

Chapter: [Dropdown: Ch 2, Ch 3, ..., Ch 20]
Lab Title: [Free text]

1. Did you successfully complete the lab? [Yes / Partially / No]
2. If no, what blocked you? [Free text]
3. Time spent on lab: [Dropdown: <4h, 4-8h, 8-12h, 12-16h, >16h]
4. Expected time from chapter: [Auto-populated from chapter]
5. Were the instructions clear? [1-5 Likert scale]
6. Were there any missing steps? [Free text]
7. Did all code examples run successfully? [Yes / No - which failed?]
8. Hardware used: [Dropdown: RTX 4070 Ti, RTX 4080, RTX 4090, Cloud GPU, Other]
9. OS and ROS 2: [Dropdown: Ubuntu 22.04 + Iron, Ubuntu 24.04 + Jazzy, Other]
10. Additional comments: [Free text]
```

**Success Criteria**:
- ≥80% of students complete labs within expected time (±15%)
- ≥90% of students rate instructions as clear (4 or 5 on Likert scale)
- ≤10% of labs have blocking issues preventing completion

---

## Performance Benchmark Validation

### Benchmark 1: Isaac Sim Rendering Performance (SC-010)

**Requirement**: Isaac Sim rendering ≥30 FPS (Budget tier), ≥60 FPS (Mid tier), ≥120 FPS (Premium tier)

**Test Setup**:
- **Scene**: Humanoid (23 DOF) + 5 manipulation objects + textured environment
- **PhysX Settings**: Time step 1/60s (60 Hz physics)
- **Rendering**: 1920×1080 resolution, RTX rasterization (not ray tracing)
- **Duration**: 60-second simulation, measure average FPS

**Test Execution**:
```python
# tests/benchmark_isaac_sim.py
import omni.isaac.sim
import time

def benchmark_rendering(duration_sec=60):
    scene = load_scene("humanoid_manipulation.usd")
    start_time = time.time()
    frame_count = 0

    while time.time() - start_time < duration_sec:
        scene.step()
        frame_count += 1

    fps = frame_count / duration_sec
    print(f"Average FPS: {fps:.1f}")
    assert fps >= 30, f"FPS {fps:.1f} below Budget tier target (30 FPS)"

benchmark_rendering()
```

**Validation Schedule**:
- **Chapter 6 (Isaac Sim Intro)**: Validate Budget tier (≥30 FPS on RTX 4070 Ti)
- **Chapter 7 (Isaac Sim Advanced)**: Validate Mid tier (≥60 FPS on RTX 4080) and Premium tier (≥120 FPS on RTX 4090)
- **Phase 4 (Technical Review)**: Re-validate all tiers on external reviewer hardware

**Acceptance Criteria**:
- Budget tier (RTX 4070 Ti): 30-50 FPS (meets ≥30 FPS target ✅)
- Mid tier (RTX 4080): 60-80 FPS (meets ≥60 FPS target ✅)
- Premium tier (RTX 4090): 120-150 FPS (meets ≥120 FPS target ✅)

---

### Benchmark 2: VLA Inference Latency (FR-014, SC-003)

**Requirement**: VLA inference ≥10 Hz on Jetson Orin Nano 8GB (with optimization)

**Test Setup**:
- **Model**: OpenVLA 7B (INT8 quantization + TensorRT)
- **Hardware**: Jetson Orin Nano 8GB (JetPack 6.x, ROS 2 Iron/Jazzy)
- **Input**: 640×480 RGB image + language instruction ("Pick up red cube")
- **Output**: 7-DOF action vector (6-DOF pose + gripper)
- **Duration**: 100 inference runs, measure average latency

**Test Execution**:
```python
# tests/benchmark_vla_inference.py
import torch
import time
from transformers import OpenVLAModel

def benchmark_vla(model, input_image, instruction, num_runs=100):
    latencies = []

    for _ in range(num_runs):
        start = time.time()
        action = model.predict(input_image, instruction)
        latencies.append(time.time() - start)

    avg_latency = sum(latencies) / len(latencies)
    hz = 1.0 / avg_latency

    print(f"Average latency: {avg_latency*1000:.1f} ms")
    print(f"Inference rate: {hz:.1f} Hz")
    assert hz >= 10, f"Inference rate {hz:.1f} Hz below target (10 Hz)"

model = load_openvla_int8_tensorrt("openvla-7b-jetson.trt")
benchmark_vla(model, test_image, "Pick up red cube")
```

**Validation Schedule**:
- **Chapter 14 (OpenVLA Integration)**: Validate RTX 4090 baseline (≥15 Hz FP16)
- **Chapter 12 (Jetson Deployment)**: Validate Jetson Orin Nano 8GB (≥10 Hz INT8 + TensorRT)
- **Chapter 20 (Capstone)**: Validate end-to-end pipeline (Whisper + LLM + Nav2 + VLA ≥12 Hz)

**Acceptance Criteria**:
- RTX 4090 (FP16): ≥15 Hz ✅
- Jetson Orin Nano 8GB (INT8 + TensorRT): ≥10 Hz ✅
- Jetson Orin NX 16GB (INT8/FP16): ≥15 Hz ✅

---

### Benchmark 3: Capstone End-to-End Pipeline (SC-003)

**Requirement**: Complete capstone pipeline ≥12 Hz on Jetson Orin Nano 8GB, <2 GB RAM

**Test Setup**:
- **Hardware**: Jetson Orin Nano 8GB (JetPack 6.x, ROS 2 Iron/Jazzy)
- **Pipeline**: Whisper (speech-to-text) → LLM (task planning) → Nav2 (navigation) + OpenVLA (manipulation)
- **Scenario**: "Go to the table and pick up the red mug"
- **Metrics**:
  - End-to-end latency (perception → decision → action)
  - Memory usage (`tegrastats` monitoring)
  - Task success rate (8/10 trials)

**Test Execution**:
```python
# tests/benchmark_capstone.py
import subprocess
import re

def benchmark_capstone():
    # Start tegrastats monitoring
    proc = subprocess.Popen(['tegrastats'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)

    # Run capstone demo for 60 seconds
    start_time = time.time()
    frame_count = 0
    max_ram_mb = 0

    while time.time() - start_time < 60:
        # Capture frame, run pipeline
        pipeline.step()
        frame_count += 1

        # Parse tegrastats output for RAM usage
        output = proc.stdout.readline().decode()
        ram_match = re.search(r'RAM (\d+)/\d+MB', output)
        if ram_match:
            ram_mb = int(ram_match.group(1))
            max_ram_mb = max(max_ram_mb, ram_mb)

    hz = frame_count / 60.0
    print(f"Pipeline rate: {hz:.1f} Hz")
    print(f"Peak RAM usage: {max_ram_mb} MB")

    assert hz >= 12, f"Pipeline {hz:.1f} Hz below target (12 Hz)"
    assert max_ram_mb <= 2048, f"RAM {max_ram_mb} MB exceeds target (<2 GB)"
```

**Validation Schedule**:
- **Chapter 20 (Capstone)**: Final integration test on Jetson Orin Nano 8GB
- **Phase 4 (Technical Review)**: External reviewers validate on their Jetson hardware (if available)

**Acceptance Criteria**:
- Pipeline rate: ≥12 Hz sustained for 60 seconds ✅
- Peak RAM usage: <2 GB (2048 MB) ✅
- Task success rate: ≥80% (8/10 trials) ✅

---

### Benchmark 4: Nav2 Planning Time (FR-014)

**Requirement**: Nav2 planning time <500ms for 10m path

**Test Setup**:
- **Planner**: Smac Planner (Hybrid A*)
- **Scene**: Warehouse environment with static obstacles
- **Path**: 10m straight-line distance with 3 obstacles requiring detours
- **Trials**: 10 navigation goals, measure planning time for each

**Test Execution**:
```python
# tests/benchmark_nav2_planning.py
import rclpy
from nav2_msgs.action import NavigateToPose

def benchmark_nav2_planning(goals):
    planning_times = []

    for goal in goals:
        start = time.time()
        result = navigator.navigate_to_pose(goal)
        planning_time = result.planning_duration.sec + result.planning_duration.nanosec / 1e9
        planning_times.append(planning_time)

    avg_time = sum(planning_times) / len(planning_times)
    max_time = max(planning_times)

    print(f"Average planning time: {avg_time*1000:.0f} ms")
    print(f"Max planning time: {max_time*1000:.0f} ms")

    assert max_time < 0.5, f"Max planning time {max_time*1000:.0f} ms exceeds 500ms target"
```

**Validation Schedule**:
- **Chapter 11 (Nav2)**: Validate planning time on RTX 4090 and RTX 4070 Ti
- **Phase 4 (Technical Review)**: External reviewers validate on their hardware

**Acceptance Criteria**:
- Average planning time: <300ms ✅
- Max planning time: <500ms ✅

---

## Hardware Compatibility Testing

### Dual ROS 2 Distribution Testing (FR-005, SC-002)

**Requirement**: 100% of code examples work on Ubuntu 22.04 + ROS 2 Iron AND Ubuntu 24.04 + ROS 2 Jazzy

**Test Matrix**:
| OS | ROS 2 Distro | Isaac Sim Version | Test Frequency |
|----|--------------|-------------------|----------------|
| Ubuntu 22.04 LTS | Iron Irwini | 2024.2 | Weekly (CI) |
| Ubuntu 24.04 LTS | Jazzy Jalisco | 2024.2 | Weekly (CI) |

**CI Matrix Configuration** (already in T007):
```yaml
strategy:
  matrix:
    ros_distro: [iron, jazzy]
    ubuntu_version: [22.04, 24.04]
```

**Validation Schedule**:
- **Phase 3 (Writing)**: Every code commit tested on both distributions (CI)
- **Phase 4 (Review)**: External reviewers test on their preferred distribution
- **Phase 5 (Production)**: Final validation before publication

**Acceptance Criteria**:
- 100% of companion repositories have passing CI on both Ubuntu 22.04 + Iron and Ubuntu 24.04 + Jazzy (SC-002 ✅)

---

### Hardware Tier Testing (FR-004, FR-014)

**Requirement**: All code examples tested on Budget tier (RTX 4070 Ti), performance claims validated on Mid and Premium tiers

**Hardware Tiers**:
1. **Budget Tier** (RTX 4070 Ti 12GB, $800):
   - **Target**: All examples run successfully, performance targets met
   - **Validation**: Chapter-by-chapter testing during Phase 3
2. **Mid Tier** (RTX 4080 16GB, $1200):
   - **Target**: Higher performance (≥60 FPS Isaac Sim)
   - **Validation**: Benchmark validation in Chapter 6, 8
3. **Premium Tier** (RTX 4090 24GB, $1999):
   - **Target**: Highest performance (≥120 FPS Isaac Sim)
   - **Validation**: Used for development + CI/CD runner

**Edge Hardware**:
1. **Jetson Orin Nano 8GB** ($249):
   - **Target**: Edge deployment, ≥10 Hz VLA inference, ≥12 Hz capstone
   - **Validation**: Chapters 12, 20
2. **Jetson Orin NX 16GB** ($599, optional):
   - **Target**: Recommended upgrade, ≥15 Hz VLA inference
   - **Validation**: Chapter 12 (optional hardware tier)

**Cloud Alternatives** (FR-025):
- AWS EC2 g5.xlarge (NVIDIA T4, $1/hour): Validate in Chapter 2 setup guide
- Estimated cost: <$300/quarter for part-time use
- Validation: Budget tier performance comparison (T4 vs. RTX 4070 Ti)

---

## Code Quality Standards

### Code Linting and Formatting (FR-007, SC-012)

**Requirement**: All code snippets pass Black formatting and flake8 linting with zero violations

**Tools**:
- **Black**: Automatic Python code formatter (line length: 88 chars)
- **flake8**: Python linter (PEP8 compliance)
- **ROS 2 Style Guide**: Additional checks for ROS 2 code conventions

**Pre-Commit Hook** (`.pre-commit-config.yaml`):
```yaml
repos:
  - repo: https://github.com/psf/black
    rev: 24.8.0
    hooks:
      - id: black
        language_version: python3.10
        args: [--line-length=88]

  - repo: https://github.com/PyCQA/flake8
    rev: 7.1.1
    hooks:
      - id: flake8
        args: [--max-line-length=88, --extend-ignore=E203,W503]
```

**Validation**:
- **Phase 3 (Writing)**: CI runs Black + flake8 on every commit
- **Phase 4 (Review)**: Zero violations required for publication
- **SC-012**: All code snippets in the book pass Black formatting and flake8 linting ✅

---

### URDF/SDF Validation (SC-004)

**Requirement**: All URDF/SDF robot models validate with `gz sdf check` and `isaac-sim validator` with zero errors

**Tools**:
- **`check_urdf`** (ROS 2): Validates URDF syntax and kinematic tree
- **`gz sdf --check`** (Gazebo): Validates SDF syntax
- **`isaac-sim validator`** (Isaac Sim): Validates USD/URDF for PhysX compatibility

**Validation Commands**:
```bash
# URDF validation
check_urdf code/shared/urdf_models/physicalai_humanoid/humanoid_12dof.urdf

# SDF validation
gz sdf --check code/shared/urdf_models/physicalai_humanoid/humanoid_23dof.sdf

# Isaac Sim USD validation
isaac-sim-validator code/shared/urdf_models/physicalai_humanoid/humanoid.usd
```

**Validation Schedule**:
- **Chapter 4 (URDF)**: Validate custom humanoid URDF models
- **Chapter 6 (Isaac Sim)**: Validate USD conversion
- **Phase 4 (Review)**: Re-validate all robot models

**Acceptance Criteria**:
- 100% of URDF/SDF models pass validation with zero errors (SC-004 ✅)

---

## Documentation Quality

### Link Checking (SC-008)

**Requirement**: Zero broken external links in the book's web version (Docusaurus) at publication time

**Tool**: `markdown-link-check`

**Validation Command**:
```bash
# Check all Markdown files for broken links
find book/chapters -name "*.md" | xargs markdown-link-check --quiet
```

**CI Integration**:
```yaml
# .github/workflows/link-check.yml
name: Link Check

on:
  push:
    branches: [main]
  schedule:
    - cron: '0 0 * * 0'  # Weekly on Sundays

jobs:
  link-check:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - name: Check links
        uses: gaurav-nelson/github-action-markdown-link-check@v1
```

**Validation Schedule**:
- **Phase 3 (Writing)**: Weekly link checks (CI scheduled job)
- **Phase 4 (Review)**: Quarterly link checks (external links may break over time)
- **Phase 5 (Production)**: Final link check before publication

**Acceptance Criteria**:
- 100% of external links valid at publication time (SC-008 ✅)
- Quarterly link checks automated via CI

---

### Citation Compliance (SC-006)

**Requirement**: Every chapter includes minimum 10 citations in APA 7th edition format, with ≥70% from 2020 or later

**Validation**:
1. **Automated Count**:
   ```bash
   # Count citations in references.bib
   grep -c "^@" references.bib
   ```
2. **Manual Review**: Verify APA 7th format compliance using Zotero or Mendeley
3. **Currency Check**: Count citations from 2020+ (≥70% threshold)

**Validation Schedule**:
- **Phase 3 (Writing)**: After each chapter, count citations (target: ≥10 per chapter)
- **Phase 4 (Review)**: Academic reviewers verify APA format compliance
- **Phase 5 (Production)**: Final bibliography review

**Acceptance Criteria**:
- All chapters have ≥10 citations (SC-006 ✅)
- ≥70% of citations from 2020 or later ✅

---

## External Testing Resources

### Technical Reviewer Onboarding

**Reviewer Recruitment** (Week 49):
- Post on ROS Discourse, r/robotics, LinkedIn
- Target: PhD students, postdocs, industry engineers with ROS 2 + Isaac Sim experience
- Compensation: Free book copy + acknowledgment in book

**Reviewer Guidelines** (shared via email):
```markdown
# Technical Reviewer Guidelines

Thank you for volunteering to review "Physical AI & Humanoid Robotics"!

## Your Tasks (3-week commitment, 10-15 hours total)

1. **Assigned Chapters**: You will review 3-4 chapters from one Part (e.g., Part 3: Perception & Edge)
2. **Code Validation**: Test all code examples in your assigned chapters
   - Verify examples run on your hardware (RTX GPU + Ubuntu 22.04/24.04)
   - Report any errors, missing dependencies, or performance issues
3. **Technical Accuracy**: Review mathematical derivations, algorithms, and hardware specs
4. **Feedback Format**: Comment directly on GitHub pull requests OR email feedback

## Your Hardware (Optional but Helpful)

- **Minimum**: RTX 4070 Ti (or cloud GPU: AWS g5.xlarge)
- **Preferred**: RTX 4080 or RTX 4090 (for performance validation)
- **Optional**: Jetson Orin Nano/NX (for edge deployment chapters)

## Timeline

- **Week 1 (Dec 7-13)**: Read assigned chapters, set up development environment
- **Week 2 (Dec 14-20)**: Test code examples, document issues
- **Week 3 (Dec 21-27)**: Submit final feedback via GitHub PR or email

## Questions?

Email: author@physicalai-book.com
```

---

### Student Tester Recruitment

**Recruitment Channels**:
- University robotics lab mailing lists (CMU, MIT, Berkeley, Stanford)
- Reddit r/robotics, r/ROS
- ROS Discourse community forum
- LinkedIn robotics groups

**Tester Compensation**:
- **Option A**: Free book copy (print + ebook) + acknowledgment in book
- **Option B**: $200-300 honorarium + free book (if budget allows)
- **Option C**: University course credit (if instructor is co-author)

**Tester Onboarding**:
```markdown
# Student Tester Onboarding

Welcome to the Physical AI & Humanoid Robotics book testing program!

## Your Tasks (4-week commitment, 20-30 hours total)

1. **Assigned Labs**: You will complete 4-5 hands-on labs from different parts of the book
2. **Time Tracking**: Document how long each lab takes (compare to chapter's estimate)
3. **Feedback**: Submit feedback via Google Form after each lab
4. **Success/Failure**: Report whether you successfully completed each lab

## Your Hardware

- **Required**: RTX 4070 Ti or better (OR cloud GPU: AWS g5.xlarge, <$50 budget provided)
- **OS**: Ubuntu 22.04 LTS (ROS 2 Iron) OR Ubuntu 24.04 LTS (ROS 2 Jazzy)
- **Storage**: 100 GB free (for Isaac Sim, ROS 2, code repos)

## Timeline

- **Week 1 (Dec 7-13)**: Complete Lab 1, submit feedback
- **Week 2 (Dec 14-20)**: Complete Lab 2, submit feedback
- **Week 3 (Dec 21-27)**: Complete Labs 3-4, submit feedback
- **Week 4 (Dec 28-Jan 3)**: Complete Lab 5 (if assigned), final feedback

## Support

- Slack channel: #book-testers (invite link sent separately)
- Weekly office hours: Fridays 2-3pm PT (Zoom link in Slack)
```

---

## Documentation and Reporting

### Weekly Testing Report (Phase 3)

**Format**:
```markdown
## Week N Testing Report

**Chapters Tested**: 5, 6
**Code Repos Tested**: chapter-05-gazebo, chapter-06-isaac-sim-intro

**CI Status**:
- chapter-05-gazebo: ✅ All tests passing (Ubuntu 22.04 + Iron, Ubuntu 24.04 + Jazzy)
- chapter-06-isaac-sim-intro: ✅ All tests passing, FPS 38 FPS on RTX 4070 Ti (target: 30 FPS)

**Hardware Validation**:
- Chapter 6 (Isaac Sim): RTX 4070 Ti → 38 FPS ✅ (exceeds 30 FPS target)
- Chapter 6 (Isaac Sim): RTX 4090 → 128 FPS ✅ (exceeds 120 FPS target)

**Issues Found**:
- Chapter 5: Gazebo world file missing texture (fixed in commit abc123)
- Chapter 6: Isaac Sim USD conversion warning (non-blocking, documented in troubleshooting)

**Action Items**:
- Update Chapter 5 troubleshooting section to mention texture path issue
- Add Chapter 6 USD conversion warning to Appendix C (Troubleshooting Reference)
```

**Distribution**: Weekly email to co-authors and project manager

---

### Final Validation Checklist (Phase 5)

**Pre-Publication Checklist**:
- [ ] All 12-15 companion repositories have passing CI (Ubuntu 22.04 + Iron, Ubuntu 24.04 + Jazzy)
- [ ] Isaac Sim FPS benchmarks validated on all 3 hardware tiers (Budget, Mid, Premium)
- [ ] VLA inference latency validated on RTX 4090 (≥15 Hz) and Jetson Orin Nano (≥10 Hz)
- [ ] Capstone end-to-end pipeline validated on Jetson Orin Nano (≥12 Hz, <2 GB RAM)
- [ ] Nav2 planning time validated (<500ms for 10m path)
- [ ] All URDF/SDF models pass validation (zero errors)
- [ ] Zero broken links in web version (Docusaurus)
- [ ] All chapters have ≥10 citations, ≥70% from 2020+
- [ ] All code snippets pass Black + flake8 (zero violations)
- [ ] All P1 and P2 reviewer feedback integrated
- [ ] Student tester feedback incorporated (≥80% success rate on labs)

---

## Success Criteria

**Primary Metric**: ≥80% of student testers complete labs within expected time (±15%), with all code examples running successfully

**Secondary Metrics**:
1. **CI Pass Rate**: 100% of companion repos have passing CI on dual ROS 2 distributions (SC-002 ✅)
2. **Performance Benchmarks**: All benchmarks reproducible within ±15% (SC-010 ✅)
3. **Code Quality**: Zero Black/flake8 violations (SC-012 ✅)
4. **Documentation**: Zero broken links at publication (SC-008 ✅)
5. **Citations**: All chapters ≥10 citations, ≥70% from 2020+ (SC-006 ✅)

---

## Document Status

**Phase**: Phase 2 In Progress (Foundational Research)
**Next Milestone**: Begin Chapter 1 writing (Phase 3, Week 17)
**Last Updated**: 2025-12-06
**Review Frequency**: Weekly during Phase 3 (Writing), Monthly during Phase 4-5
