# ADR-0003: Dual Platform Support Strategy

> **Scope**: Define the strategy for supporting multiple operating systems and ROS 2 distributions across the book's examples and the CI/CD pipeline.
>
- **Status:** Proposed
- **Date:** 2025-12-06
- **Feature:** 002-humanoid-robotics-book
- **Context:** The book targets advanced students and engineers who may use different OS versions and ROS 2 distributions. Ensuring consistent and reproducible examples across these environments is critical.

## Decision

Primary support will be for Ubuntu LTS releases, specifically **Ubuntu 22.04 (Jammy Jellyfish)** and **Ubuntu 24.04 (Noble Numbat)**, paired with their corresponding LTS ROS 2 distributions: **ROS 2 Iron Irwini** and **ROS 2 Jazzy Jalisco**. All code examples, build scripts, and CI/CD pipelines will be developed and tested against these configurations.

For other platforms (e.g., different Ubuntu versions, other Linux distributions, macOS, Windows), best-effort support will be provided through documentation and community contributions. Explicitly, we will not create dedicated build/test environments for these secondary platforms, but will document known compatibility issues or workarounds.

## Consequences

### Positive

- **Clear Target Environments**: Provides well-defined targets for development, testing, and user support, reducing ambiguity.
- **Reproducibility**: Ensures that examples and the CI/CD pipeline are consistently reproducible for the majority of the target audience.
- **Focus**: Allows the development team to concentrate efforts on two primary, well-supported configurations.
- **Community Engagement**: Encourages community contributions for support on less common platforms.

### Negative

- **Limited Support for Other Platforms**: Users on unsupported platforms may encounter issues that cannot be immediately resolved.
- **Potential for Compatibility Drift**: Maintaining compatibility across two distinct ROS 2 distributions (Iron and Jazzy) requires diligent testing.

## Alternatives Considered

1.  **Single ROS 2 Distribution Support (e.g., only Jazzy)**:
    *   *Description*: Focus solely on the latest LTS ROS 2 distribution.
    *   *Why Rejected*: Many educational institutions and existing projects may still be on the previous LTS (Iron Irwini), leading to compatibility issues for a significant portion of the target audience.

2.  **Broadest Possible Support (All ROS 2 Distributions + Multiple OS)**:
    *   *Description*: Attempt to support all active ROS 2 distributions and multiple operating systems (e.g., Fedora, macOS, Windows).
    *   *Why Rejected*: Extremely resource-intensive, leading to slower development, higher testing burden, and increased risk of code fragility. Unrealistic for a textbook project with limited resources.

## References

- Feature Spec: `specs/002-humanoid-robotics-book/spec.md`
- Implementation Plan: `specs/002-humanoid-robotics-book/plan.md`
- Related ADRs: `history/adr/0001-book-publishing-technology-stack.md`, `history/adr/0002-code-repository-architecture-and-distribution-model.md`
- Evaluator Evidence: `null`
