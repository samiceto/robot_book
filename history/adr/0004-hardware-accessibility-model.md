# ADR-0004: Hardware Accessibility Model

> **Scope**: Define the strategy for providing access to physical hardware (humanoid robots, Jetson devices) for book examples and exercises.
>
- **Status:** Proposed
- **Date:** 2025-12-06
- **Feature:** 002-humanoid-robotics-book
- **Context:** Providing access to specialized hardware like humanoid robots and edge AI devices is challenging due to cost, availability, and setup complexity. A tiered approach is needed to balance accessibility with the depth of practical examples.

## Decision

A tiered hardware accessibility model will be implemented:

1.  **Tier 1: Simulation (Free & Widely Accessible)**: All core concepts will be demonstrable using NVIDIA Isaac Sim, which can be run on standard development machines (with a decent GPU). This tier provides a fully functional virtual environment.
2.  **Tier 2: Edge Device Access (Limited Availability)**: For chapters focusing on edge deployment, examples will target NVIDIA Jetson Orin Nano (8GB). Access to these devices will be facilitated through:
    *   University labs or partner institutions.
    *   Optional recommended hardware purchase lists for individual students or professionals.
    *   Cloud-based GPU instances (e.g., AWS, GCP, Azure) pre-configured with Jetson-compatible environments, accessible via remote desktop or SSH.
3.  **Tier 3: Humanoid Robot Access (Most Limited)**: For chapters involving full-body humanoid robotics (e.g., manipulation, locomotion), access will be primarily through:
    *   University labs or research facilities equipped with robots like Unitree G1 or similar.
    *   Highly detailed simulation in Isaac Sim, serving as the primary mechanism for most users.

This tiered approach ensures that all users can engage with the core material through simulation, while those with access to physical hardware can perform advanced practical exercises.

## Consequences

### Positive

- **Broad Accessibility**: Enables users without physical hardware to learn and experiment through simulation.
- **Cost-Effectiveness**: Reduces the barrier to entry by relying on simulation and offering tiered access to physical hardware.
- **Focused Learning**: Allows for practical exercises tailored to specific hardware capabilities (simulation vs. edge vs. full humanoid).

### Negative

- **Sim-to-Real Gap**: Simulation may not perfectly replicate real-world hardware behavior, especially for complex physical interactions.
- **Hardware Dependency for Advanced Topics**: Users without access to physical robots will rely solely on simulation for the most advanced chapters.
- **Setup Complexity for Cloud Instances**: Users opting for cloud instances may face additional setup and cost considerations.

## Alternatives Considered

1.  **Simulation Only**:
    *   *Description*: Rely exclusively on simulation for all examples.
    *   *Why Rejected*: Lacks the practical experience of deploying and running code on real edge devices or humanoid robots, which is a key aspect of the book's target audience.

2.  **Mandatory Hardware Purchase**:
    *   *Description*: Require users to purchase specific hardware (Jetson, humanoid robot) to complete the book's examples.
    *   *Why Rejected*: Prohibitively expensive for most students and professionals, severely limiting the book's reach and accessibility.

3.  **Cloud-Based Robot Access (Dedicated)**:
    *   *Description*: Provide direct, dedicated access to physical robots hosted in the cloud for all users.
    *   *Why Rejected*: Extremely high operational cost, complex scheduling and resource management, difficult to ensure consistent availability.

## References

- Feature Spec: `specs/002-humanoid-robotics-book/spec.md`
- Implementation Plan: `specs/002-humanoid-robotics-book/plan.md`
- Related ADRs: `history/adr/0001-book-publishing-technology-stack.md`, `history/adr/0002-code-repository-architecture-and-distribution-model.md`, `history/adr/0003-dual-platform-support-strategy.md`
- Evaluator Evidence: `null`
