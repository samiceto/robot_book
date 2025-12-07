# ADR-0001: Book Publishing Technology Stack

> **Scope**: Document decision clusters, not individual technology choices. Group related decisions that work together (e.g., "Frontend Stack" not separate ADRs for framework, styling, deployment).

- **Status:** Accepted
- **Date:** 2025-12-06
- **Feature:** 002-humanoid-robotics-book
- **Context:** Need to write a 550-650 page technical textbook that supports multiple output formats (print PDF, digital ebook, web version), enables version control and collaboration, handles technical content (code blocks, equations, diagrams), and allows for long-term maintenance and updates as software versions evolve. The book must be accessible to students (web version) while also being publishable through traditional channels (O'Reilly, Manning, No Starch Press).

<!-- Significance checklist (ALL must be true to justify this ADR)
     1) Impact: Long-term consequence for architecture/platform/security? YES - Affects entire writing workflow, build process, maintainability for 3+ years
     2) Alternatives: Multiple viable options considered with tradeoffs? YES - LaTeX-only, proprietary tools (Word/InDesign), web-only platforms
     3) Scope: Cross-cutting concern (not an isolated detail)? YES - Every chapter, all contributors, build pipeline, errata updates
-->

## Decision

**Adopt a multi-format publishing pipeline with Markdown as single source of truth:**

- **Source Format**: Markdown with CommonMark + GFM (GitHub Flavored Markdown)
- **Print/PDF**: Markdown → Pandoc → LaTeX → PDF (using custom LaTeX template)
- **Digital Ebook**: Leanpub platform (Markdown native, incremental publishing)
- **Web Version**: Docusaurus 3.x (searchable, mobile-responsive, free access)
- **Version Control**: Git-friendly (plain text, diff-able, branch-able)
- **Code Syntax**: Prism (web) + minted/listings (LaTeX) for Python, C++, XML, YAML
- **Equations**: KaTeX (web) + LaTeX math mode (print)
- **Diagrams**: Mermaid (web) + exported SVG (print)

**Toolchain Workflow:**
```
book/chapters/*.md (single source)
    ├─→ Pandoc → LaTeX → pdflatex → book.pdf (print)
    ├─→ Leanpub import → book.epub/.mobi (digital)
    └─→ Docusaurus build → static site (web)
```

## Consequences

### Positive

- **Single Source of Truth**: Write once in Markdown, generate multiple formats automatically
- **Version Control Friendly**: Git works natively with plain text Markdown (meaningful diffs, merge conflicts resolvable, branching for chapters)
- **Collaboration Ready**: Contributors can edit Markdown without LaTeX expertise, pull requests show readable diffs
- **Automated Builds**: CI/CD can generate all formats on commit (test print PDF + web build on every PR)
- **Long-term Maintainability**: Updating software versions or fixing errors only requires editing Markdown, regenerate all formats
- **Free Web Access**: Docusaurus provides searchable, accessible web version at zero hosting cost (GitHub Pages/Vercel)
- **Incremental Publishing**: Leanpub enables "publish early, update often" model for digital version while print is in production
- **Tooling Ecosystem**: Pandoc, Docusaurus, Leanpub are mature, well-documented, actively maintained

### Negative

- **Multi-Format Complexity**: Must maintain 3 build pipelines (Pandoc, Leanpub, Docusaurus), each with quirks
- **Markdown Limitations**: Advanced LaTeX features (custom environments, complex tables) may not translate to web/ebook cleanly
- **Build Dependencies**: Requires Pandoc, LaTeX distribution (TeX Live/MiKTeX), Node.js (Docusaurus), careful version pinning
- **Format-Specific Tweaks**: May need conditional content (e.g., "see Figure 3-2 on page 45" in print vs "see Figure 3-2 below" in web)
- **Learning Curve**: Authors must learn Markdown conventions, Pandoc metadata blocks, Docusaurus frontmatter
- **Testing Burden**: Every change must be validated across all 3 output formats (print PDF, web, ebook)
- **LaTeX Fallback Complexity**: For advanced typesetting needs, may need to embed raw LaTeX in Markdown (breaks web/ebook)

## Alternatives Considered

### Alternative A: LaTeX-Only (Traditional Academic Publishing)
**Approach**: Write entire book in LaTeX, generate PDF directly, convert to ebook with tex4ht or similar

**Pros**:
- Maximum typographic control (professional print quality)
- Single build pipeline (simpler)
- Standard for academic textbooks (Springer, Cambridge University Press)

**Cons**:
- LaTeX is **not Git-friendly** (diffs are unreadable, merge conflicts hard to resolve)
- Steep learning curve for contributors (limits collaboration)
- No native web version (tex4ht output is poor quality)
- Vendor lock-in to LaTeX ecosystem (hard to migrate)

**Why Rejected**: Collaboration and web accessibility are critical for this project. External testers and community contributors cannot be expected to learn LaTeX. Web version is mandatory per spec (FR-016: Docusaurus requirement).

### Alternative B: Proprietary Tools (Microsoft Word + InDesign)
**Approach**: Write in Word, hand off to publisher for InDesign layout, generate ebook separately

**Pros**:
- WYSIWYG editing (easier for non-technical authors)
- Publisher handles production (less work for author)

**Cons**:
- **Binary formats are not version-control friendly** (no meaningful diffs, merge conflicts impossible)
- Expensive licensing (InDesign $20+/month)
- No automation (manual export to PDF, ebook requires separate tooling)
- Vendor lock-in (Microsoft, Adobe)
- No web version workflow

**Why Rejected**: Binary formats break Git workflow. Spec requires 3-year maintenance commitment (FR-023), which is impractical without version control. No path to web version (FR-016 requirement).

### Alternative C: Web-Only Platform (GitBook, mdBook, Jupyter Book)
**Approach**: Write in Markdown for web-native platform, export to PDF as afterthought

**Pros**:
- Simplest workflow (one build target)
- Excellent web experience (native platform)
- Git-friendly Markdown source

**Cons**:
- Print PDF quality is **secondary** (web-to-PDF conversion often poor)
- Traditional publisher integration is hard (O'Reilly/Manning expect LaTeX or InDesign)
- Limited ebook support (EPUB export often buggy)
- Platform lock-in (GitBook is proprietary SaaS)

**Why Rejected**: Print quality matters for university adoption and traditional publishing. Spec explicitly requires multi-format output (print PDF, ebook, web) with professional print quality (550-650 pages is a serious textbook, not just web docs).

### Alternative D: Jupyter Book (Academic Python Publishing)
**Approach**: Write in Jupyter notebooks + Markdown, build with Jupyter Book (Sphinx-based)

**Pros**:
- Excellent for code-heavy content (executable notebooks)
- Good academic adoption (NumPy, SciPy, PyData ecosystem)
- Generates web + PDF

**Cons**:
- Jupyter notebooks are **JSON (not plain text)**, making diffs noisy
- Sphinx PDF output quality is mediocre (not suitable for traditional publishing)
- Mixed Markdown + notebooks complicates structure
- Execution state in notebooks creates reproducibility issues

**Why Rejected**: While appealing for code-heavy content, Jupyter notebooks break Git-friendly workflow (JSON diffs are unreadable). Sphinx PDF quality insufficient for 550-650 page professional textbook. Execution state in notebooks adds complexity (what if cell fails during build?).

## References

- Feature Spec: [specs/002-humanoid-robotics-book/spec.md](../specs/002-humanoid-robotics-book/spec.md) (FR-016: Docusaurus web version, FR-023: 3-year maintenance)
- Implementation Plan: [specs/002-humanoid-robotics-book/plan.md](../specs/002-humanoid-robotics-book/plan.md) (Lines 10-11: Book format, Lines 1334-1353: Research Task 7)
- Related ADRs: ADR-0006 (Phased Research-Concurrent Writing Approach) - affects build/test workflow
- Evaluator Evidence: Plan.md lines 1334-1353 document toolchain research requirements (Pandoc, LaTeX templates, Leanpub workflow)
