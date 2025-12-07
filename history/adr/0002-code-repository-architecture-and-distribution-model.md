# ADR-0002: Code Repository Architecture and Distribution Model

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).
>
- **Status:** Accepted
- **Date:** 2025-12-06
- **Feature:** 002-humanoid-robotics-book
- **Context:** To support a multi-format publishing pipeline, independent chapter development, and robust testing with GitHub Classroom, a scalable and maintainable code repository structure is required.

## Decision

Adopt a hybrid monorepo architecture for the book project. The top-level repository will house shared assets, documentation (ADRs, specs, plans, PHRs), CI/CD configurations, and a Docusaurus website. Each of the 12-15 book chapters will reside in its own independent subdirectory, treated as a distinct package with its own `package.json` (or equivalent) and build scripts. This structure allows for independent development, testing, and versioning of chapters while maintaining a central source of truth for the overall project. All content will be licensed under the MIT License to ensure maximum accessibility and reusability.

## Consequences

### Positive

- **GitHub Classroom Integration**: Each chapter's subdirectory can be easily forked as a separate repository for student assignments, simplifying the submission and grading workflow.
- **Independent Versioning**: Chapters can be versioned independently, allowing for phased releases or updates without affecting other parts of the book.
- **Modular Development**: Developers can focus on specific chapters without needing to clone or build the entire project, improving development efficiency.
- **Clear Ownership**: Easier to assign ownership and track contributions per chapter.
- **MIT License**: Maximizes reusability for educational and commercial purposes.

### Negative

- **Code Duplication**: Potential for minor code duplication across chapters if not carefully managed (e.g., common utility functions). This can be mitigated by a shared `common` or `utils` directory at the root.
- **CI/CD Complexity**: Managing CI/CD pipelines for multiple independent packages within a monorepo requires careful configuration (e.g., using path filtering in GitHub Actions).
- **Dependency Management**: While chapters are independent, managing shared dependencies across the monorepo may require a root-level dependency management strategy.

## Alternatives Considered

1.  **Single Large Monorepo**:
    - *Description*: All chapters and shared assets in a single, flat repository.
    - *Why Rejected*: Difficult to manage with GitHub Classroom (forking the entire repo for each student assignment is impractical), lacks clear separation for independent chapter versioning, can become unwieldy as the project grows.

2.  **Fully Separate Repositories**:
    - *Description*: Each chapter, plus shared assets and website, in its own distinct GitHub repository.
    - *Why Rejected*: Extremely difficult to maintain project-wide consistency (documentation, CI/CD, licensing), complex to track cross-chapter dependencies, harder to provide a cohesive user experience for the entire book.

3.  **Licensing (Other Options)**:
    - *Apache 2.0*: Considered due to its permissive nature, but MIT is simpler and more widely understood in academic/open-source communities for this type of educational content.
    - *GPL-3.0*: Rejected due to its copyleft nature, which could deter commercial adoption or use in proprietary projects, contradicting the goal of broad accessibility.

## References

- Feature Spec: `specs/002-humanoid-robotics-book/spec.md`
- Implementation Plan: `specs/002-humanoid-robotics-book/plan.md`
- Related ADRs: `history/adr/0001-book-publishing-technology-stack.md`
- Evaluator Evidence: `null`
