# Physical AI & Humanoid Robotics: From Simulated Brains to Walking Bodies

A comprehensive 550-650 page practitioner textbook covering the modern 2025-2027 embodied AI stack, targeting advanced undergraduate/Master's students, industry engineers, and university instructors.

## 📚 About This Book

This book integrates theoretical foundations with hands-on labs, covering:

- **ROS 2 Fundamentals** (Iron/Jazzy distributions)
- **NVIDIA Isaac Sim** for realistic humanoid simulation
- **Vision-Language-Action (VLA) Models** for manipulation
- **Edge Deployment** on NVIDIA Jetson Orin
- **Bipedal Locomotion** and whole-body control
- **End-to-End Integration** for voice-controlled autonomous humanoids

## 🎯 Target Audience

- Advanced undergraduate and Master's students in robotics/AI
- Industry engineers transitioning to humanoid robotics roles (Figure AI, Tesla Optimus, Agility Robotics)
- University instructors teaching robotics courses
- Researchers seeking reproducible baselines

## 📖 Book Structure

**6 Parts, 21 Chapters:**

1. **Part 1: Foundations & ROS 2** (Chapters 1-4)
2. **Part 2: Digital Twins & Simulation** (Chapters 5-8)
3. **Part 3: Perception & Edge Brain** (Chapters 9-12)
4. **Part 4: Embodied Cognition & VLA Models** (Chapters 13-16)
5. **Part 5: Bipedal Locomotion & Control** (Chapters 17-19)
6. **Part 6: Capstone Integration** (Chapters 20-21)

## 🛠️ Technical Stack

- **OS**: Ubuntu 22.04 LTS or 24.04 LTS
- **ROS 2**: Iron Irwini (2023-2027) or Jazzy Jalisco (2024-2029)
- **Simulation**: NVIDIA Isaac Sim 2024.2+, Gazebo Harmonic
- **Perception**: Isaac ROS 3.0+, Nav2
- **Deep Learning**: PyTorch 2.0+
- **Edge Hardware**: NVIDIA Jetson Orin (Nano 8GB, NX 16GB, AGX 64GB)
- **Languages**: Python 3.10+, C++ (for ROS 2)

## 💻 Hardware Requirements

### Budget Tier (<$1200)
- GPU: RTX 4070 Ti 12GB (~$800-1000)
- Total system: <$1200

### Mid Tier ($2500-3500)
- GPU: RTX 4080 16GB (~$1200-1500)
- Total system: $2500-3500

### Premium Tier ($5000+)
- GPU: RTX 4090 24GB (~$1600-2000)
- Edge: Jetson AGX Orin 64GB (~$2000)
- Total system: $5000+

### Edge Testing
- Jetson Orin Nano 8GB: $249

**Cloud Alternative**: AWS EC2 g5.xlarge (~$1/hour, <$300/quarter for part-time use)

## 📁 Repository Structure

```
robot_book/
├── book/                          # Book content (Markdown source)
│   ├── chapters/                  # 21 chapters organized by parts
│   ├── appendices/                # Hardware guide, math primer, troubleshooting
│   ├── frontmatter/               # Preface, about author, how to use
│   └── images/                    # Figures and diagrams
├── code/                          # Companion code repositories (12-15 repos)
│   ├── chapter-03-ros2-basics/
│   ├── chapter-06-isaac-sim-intro/
│   ├── chapter-14-vla-integration/
│   └── chapter-20-capstone/
├── website/                       # Docusaurus web version
│   ├── docs/                      # Converted Markdown chapters
│   ├── src/                       # Custom pages (index, resources, errata)
│   └── static/                    # Static assets
├── instructor-resources/          # Course materials
│   ├── slides/                    # Lecture slides (one per chapter)
│   ├── assignments/               # Assignment prompts and solutions
│   ├── quizzes/                   # Quiz questions per part
│   └── syllabus-templates/        # 13-week semester, 10-week quarter
├── specs/                         # Project specifications and planning
│   └── 002-humanoid-robotics-book/
│       ├── spec.md                # Feature specification
│       ├── plan.md                # Implementation plan
│       ├── tasks.md               # Task breakdown
│       └── decisions/             # Architectural decisions
├── .github/workflows/             # CI/CD workflows
├── .pandoc/                       # LaTeX templates for PDF generation
├── requirements.txt               # Python dependencies (pinned versions)
├── pyproject.toml                 # Black formatter config
├── .flake8                        # Linting config
├── Makefile                       # Build automation (Markdown → PDF/EPUB)
└── README.md                      # This file
```

## 🚀 Quick Start

### Prerequisites

- Ubuntu 22.04 LTS or 24.04 LTS
- Python 3.10+
- Git

### Setup Development Environment

1. **Clone the repository**:
   ```bash
   git clone https://github.com/your-org/robot_book.git
   cd robot_book
   ```

2. **Install Python dependencies**:
   ```bash
   python3 -m pip install -r requirements.txt
   ```

3. **Install ROS 2 Iron** (Ubuntu 22.04):
   ```bash
   sudo apt update
   sudo apt install software-properties-common
   sudo add-apt-repository universe
   sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
     -o /usr/share/keyrings/ros-archive-keyring.gpg
   echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
     http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
     sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
   sudo apt update
   sudo apt install ros-iron-desktop
   ```

4. **Build Docusaurus web version**:
   ```bash
   cd website
   npm install
   npm start
   ```
   Open http://localhost:3000 in your browser.

5. **Generate PDF** (requires LaTeX):
   ```bash
   make pdf
   ```

## 📝 Writing Workflow

### Creating a New Chapter

1. Copy the chapter template:
   ```bash
   cp .specify/templates/chapter-template.md \
      book/chapters/partN-name/##-chapter-title.md
   ```

2. Fill in learning objectives, sections, hands-on lab, and end-of-chapter project.

3. Create companion code repository:
   ```bash
   mkdir -p code/chapter-##-topic
   cp .github/workflows/ci-template.yml code/chapter-##-topic/.github/workflows/ci.yml
   ```

4. Add minimum 10 citations to `references.bib` in APA 7th edition format.

5. Validate word count (target range in chapter template).

### Code Quality Checks

```bash
# Format Python code with Black
black .

# Lint with flake8
flake8 .

# Type check with mypy
mypy .
```

## 🧪 Testing Infrastructure

Each companion code repository includes:

- **Dual ROS 2 testing**: Ubuntu 22.04 + Iron AND Ubuntu 24.04 + Jazzy
- **Code quality**: Black formatting + flake8 linting
- **Build verification**: `colcon build` succeeds
- **Test execution**: `colcon test` passes
- **Weekly CI**: Automated testing every Monday

## 📚 Success Criteria

- **SC-001**: 3 external testers complete any lab in <15 hours
- **SC-002**: 100% CI passing (Ubuntu 22.04/24.04 + ROS 2 Iron/Jazzy)
- **SC-003**: Capstone runs at ≥12 Hz on Jetson Orin Nano 8GB, <2 GB RAM
- **SC-004**: All URDF/SDF validate with `gz sdf check` (zero errors)
- **SC-005**: 550-650 pages, 160,000-190,000 words
- **SC-006**: ≥10 citations per chapter, ≥70% from 2020+
- **SC-010**: Performance benchmarks within ±15% on reference hardware
- **SC-012**: All code passes Black + flake8 with zero violations

## 🤝 Contributing

This is an open book project! Contributions are welcome for:

- Errata corrections
- Code example improvements
- Additional case studies
- Translations

See `CONTRIBUTING.md` for guidelines.

## 📜 License

- **Book Content**: Copyright © 2025 (Traditional or CC BY-NC-SA 4.0 for web version)
- **Code Examples**: MIT License (see individual repositories)

## 📞 Contact

- **Author**: samiceto
- **GitHub**: [https://github.com/samiceto/robot_book](https://github.com/samiceto/robot_book)
- **Website**: [https://samiceto.github.io/robot_book](https://samiceto.github.io/robot_book)

## 🙏 Acknowledgments

- NVIDIA for Isaac Sim and Jetson Orin platforms
- Open Robotics for ROS 2
- All external testers and reviewers
- Industry practitioners who contributed case studies

---

**Status**: Phase 1 (Setup) Complete ✅ | Phase 2 (Research) In Progress

**Target Publication**: Q4 2027 or Q1 2028
