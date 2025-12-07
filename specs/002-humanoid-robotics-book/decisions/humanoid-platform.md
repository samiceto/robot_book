# Architectural Decision: Humanoid Platform Selection

**Date**: 2025-12-06
**Status**: Decided
**Decision Makers**: Book Author Team
**Related Tasks**: T010

## Context

The Physical AI & Humanoid Robotics textbook needs a consistent humanoid platform for all examples, labs, and the capstone project. The platform must be:
- Accessible to students (affordable or simulation-only)
- Compatible with Isaac Sim and ROS 2
- Representative of modern humanoid capabilities
- Well-documented for educational purposes

## Options Considered

### Option 1: Unitree G1 Commercial Humanoid
**Cost**: $16,000
**Availability**: Commercially available (2024-2025)
**Specifications**:
- 23 degrees of freedom (DOF)
- Full-body humanoid with bipedal locomotion
- Advanced sensors (cameras, IMU, force-torque sensors)
- Documented kinematics and dynamics

**Pros**:
- Real commercial humanoid used in industry
- High pedagogical value (students learn on production platform)
- Complete sensor suite for realistic labs
- Active community and support

**Cons**:
- $16,000 cost prohibitive for most students
- Availability may be limited (supply chain, regional restrictions)
- Requires physical space and safety infrastructure
- Maintenance and repair costs add to total ownership
- Not all readers will have access to physical hardware

**Risk**: High barrier to entry limits book accessibility

### Option 2: Figure 02 / Tesla Optimus
**Cost**: Not publicly available
**Availability**: Waitlist only (Figure 02), not commercially available (Optimus)

**Pros**:
- Cutting-edge platforms with significant industry interest
- Would attract readers interested in these specific companies

**Cons**:
- Not available for purchase by general public as of 2025
- No public URDF/technical specifications
- Cannot be used for hands-on labs
- Availability timeline uncertain

**Decision**: **REJECTED** - Cannot base textbook on unavailable platforms

### Option 3: Open-Source Humanoids (Poppy, THOR, GummiArm)
**Cost**: $2,000-$5,000 (DIY build)
**Availability**: Open-source designs, requires assembly

**Pros**:
- Lower cost than commercial platforms
- Full source code and design files available
- Community-driven improvements
- Educational focus

**Cons**:
- Requires significant mechanical/electrical assembly skills
- Limited DOF compared to commercial platforms (typically 12-18 DOF)
- Inconsistent documentation quality
- May not represent state-of-the-art capabilities
- Reliability issues (3D printed parts, servos)

**Risk**: Variability in student builds leads to inconsistent lab experiences

### Option 4: Proxy Platform - Simplified Humanoid URDF + Isaac Sim
**Cost**: $0 (simulation only) or cloud GPU costs ($200-300/quarter)
**Availability**: Immediately available to all readers

**Specifications**:
- Custom URDF with realistic humanoid kinematics
- Scalable complexity: Simple version (12 DOF) → Advanced version (23+ DOF)
- Multiple variants for different use cases (manipulation-focused, locomotion-focused)
- Full Isaac Sim integration with PhysX physics

**Pros**:
- **Zero hardware cost** - all readers can participate in labs
- **Immediate availability** - no procurement delays
- **Consistent experience** - everyone uses identical platform
- **Scalable complexity** - can start simple, add complexity as book progresses
- **Safe experimentation** - can test dangerous scenarios without risk
- **Easy modification** - readers can customize for their research
- **Cloud-compatible** - works on AWS/GCP GPU instances for readers without local hardware
- **Pedagogical flexibility** - can demonstrate concepts not possible on real hardware (e.g., extreme parameter tuning, failure modes)

**Cons**:
- Sim-to-real gap - students don't experience real-world constraints (friction, sensor noise, latency)
- Less impressive for portfolio (simulation-only vs. real robot)
- Doesn't teach hardware debugging skills

**Mitigation**:
- Include dedicated Chapter 19 on sim-to-real transfer
- Provide case studies from industry practitioners on real deployment
- Document Isaac Sim realism features (domain randomization, sensor noise, contact physics)
- Offer optional appendix on adapting code to Unitree G1 or other commercial platforms

### Option 5: Hybrid Approach - Proxy URDF + Optional Real Robot
**Cost**: $0 (baseline) + optional hardware for advanced users
**Availability**: Universal baseline, optional upgrades

**Pros**:
- Combines accessibility of simulation with option for real hardware
- Allows different reader segments to engage at their level
- Code designed to be platform-agnostic (ROS 2 abstraction)

**Cons**:
- Requires maintaining two code paths (sim-only vs. real hardware)
- Testing complexity increases

## Decision

**SELECTED: Option 4 - Simplified Humanoid URDF + Isaac Sim (with Option 5 enhancements)**

### Primary Platform: Custom "PhysicalAI-Humanoid" URDF

**Specifications**:
- **Base Model**: 12 DOF humanoid
  - Torso: 1 DOF (yaw rotation)
  - Legs: 2 × 3 DOF (hip pitch/roll/yaw, knee pitch, ankle pitch/roll)
  - Arms: 2 × 2 DOF (shoulder pitch/roll, elbow pitch)
  - Head: 2 DOF (pan/tilt)
- **Advanced Model**: 23 DOF humanoid (introduced in Part 5: Locomotion)
  - Adds wrist rotations, finger actuation, additional torso DOF
- **Sensors**:
  - Front-facing RGB camera (640×480, 30 Hz)
  - Depth camera (Intel RealSense D435i equivalent)
  - IMU (100 Hz)
  - Joint encoders (position, velocity)
  - Optional: Force-torque sensors in feet (for locomotion chapters)

**Isaac Sim Integration**:
- Provided as `.urdf`, `.xacro`, and Isaac Sim `.usd` formats
- PhysX collision and physics parameters pre-tuned
- Example scenes for each chapter (manipulation, navigation, locomotion)
- Material properties (friction, restitution) realistic for sim-to-real transfer

**Companion Code Repository**: `code/shared/urdf_models/physicalai_humanoid/`

### Optional Real Hardware Path

For readers with access to commercial humanoids (Unitree G1, Figure 02 when available, etc.):
- Appendix G: "Adapting Code to Real Humanoid Platforms"
- Provides ROS 2 interface abstraction layer
- Documents platform-specific considerations (control rates, sensor specs, safety limits)
- Community-contributed adaptations linked from book website

## Rationale

1. **Accessibility**: Prioritizing zero-cost simulation ensures all readers can complete labs regardless of budget or location.
2. **Consistency**: Using a standardized URDF eliminates variability in student setups, making troubleshooting easier.
3. **Pedagogical Control**: Custom URDF allows progressive complexity—students aren't overwhelmed by 23 DOF humanoid in Chapter 2.
4. **Isaac Sim Alignment**: Isaac Sim is the primary simulation platform (decided in T012); using a custom URDF optimized for Isaac Sim maximizes simulation realism.
5. **Future-Proofing**: Code abstracted via ROS 2 allows readers to migrate to real hardware when platforms become more accessible (2026-2027+).
6. **Industry Relevance**: While simulation-based, the VLA models, Nav2, and Jetson deployment chapters use real production tools used by Figure AI, Tesla, etc.

## Consequences

### Positive
- ✅ Universal accessibility (no hardware purchase required)
- ✅ Consistent lab experience for all readers
- ✅ Safer experimentation (no risk of damaging $16k robot)
- ✅ Easier TA support for university courses (all students have identical setup)
- ✅ Enables cloud-based labs (AWS/GCP GPU instances)
- ✅ Can demonstrate concepts impossible on real hardware (e.g., teleportation for testing edge cases)

### Negative
- ❌ Sim-to-real gap not experienced firsthand
- ❌ Hardware debugging skills not developed
- ❌ Less impressive for portfolio (simulation-only demonstrations)

### Neutral
- ⚖️ Requires dedicating Chapter 19 to sim-to-real transfer to address reality gap
- ⚖️ May need to add more industry case studies to compensate for lack of real hardware experience

## Implementation

1. **Chapter 2 (Quickstart)**: Introduce "PhysicalAI-Humanoid" base model (12 DOF)
2. **Chapter 4 (URDF/SDF)**: Detailed walkthrough of humanoid URDF structure
3. **Chapter 6 (Isaac Sim Intro)**: Load humanoid in Isaac Sim, test physics
4. **Chapter 17 (Bipedal Locomotion)**: Introduce advanced model (23 DOF) for whole-body control
5. **Chapter 19 (Sim-to-Real)**: Document reality gap, domain randomization, transfer strategies
6. **Appendix G** (Optional): Guide for adapting code to Unitree G1, Figure 02 (when available)

## Validation

- ✅ Cost: $0 hardware cost meets accessibility goal
- ✅ Availability: Immediately available to all readers
- ✅ Documentation: Full URDF documentation in Chapter 4
- ✅ Isaac Sim compatibility: Custom-designed for Isaac Sim integration
- ✅ Pedagogical value: Progressive complexity (12 DOF → 23 DOF) matches learning curve

## References

- Unitree G1 Specifications: https://www.unitree.com/g1
- Poppy Project: https://www.poppy-project.org/
- ROS 2 URDF Tutorials: https://docs.ros.org/en/jazzy/Tutorials/Intermediate/URDF/URDF-Main.html
- Isaac Sim USD Import: https://docs.omniverse.nvidia.com/isaacsim/latest/gui_tutorials/tutorial_gui_import_urdf.html

## Revision History

- 2025-12-06: Initial decision - Selected Option 4 (Simplified URDF + Isaac Sim) with Option 5 enhancements
