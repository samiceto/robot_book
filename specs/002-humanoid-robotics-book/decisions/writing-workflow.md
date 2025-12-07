# Architectural Decision: Technical Book Writing Workflow and Toolchain

**Date**: 2025-12-06
**Status**: Decided
**Decision Makers**: Book Author Team
**Related Tasks**: T016

## Context

Writing a 550-650 page technical book requires a robust workflow and toolchain that:
- Supports dual publishing: Print (PDF) + Ebook (EPUB) + Web (HTML)
- Enables collaborative editing and version control
- Validates technical accuracy (code examples, commands, citations)
- Maintains consistent style and formatting
- Allows iterative feedback from reviewers and testers
- Integrates with CI/CD for automated validation

## Options Considered

### Option 1: Markdown → Pandoc → LaTeX → PDF/EPUB (with Docusaurus for Web)
**Source Format**: Markdown (.md)
**Conversion**: Pandoc Universal Document Converter
**Print Format**: LaTeX → PDF
**Ebook Format**: EPUB3
**Web Format**: Docusaurus (React/MDX)

**Workflow**:
```
book/chapters/*.md (source)
    ↓
Pandoc → LaTeX (.tex) → pdflatex → book.pdf (print)
    ↓
Pandoc → EPUB3 → book.epub (ebook)
    ↓
Docusaurus → Static HTML (web)
```

**Pros**:
- ✅ **Single Source**: Write once in Markdown, publish to 3 formats
- ✅ **Version Control**: Git-friendly plain text (easy diffs, merge conflicts)
- ✅ **Pandoc Flexibility**: Supports 40+ input/output formats
- ✅ **LaTeX Quality**: Professional typography for print (TeX algorithms for line breaking, hyphenation)
- ✅ **Code Blocks**: Syntax highlighting via Pandoc filters + Prism (web) / listings (LaTeX)
- ✅ **Math Rendering**: LaTeX math works in all formats (PDF, EPUB, Web via KaTeX)
- ✅ **Open-Source**: No vendor lock-in, no licensing costs
- ✅ **Automation**: Makefile orchestrates builds, CI/CD integration
- ✅ **Industry Standard**: Used by O'Reilly (Atlas platform), Manning, Pragmatic Programmers

**Cons**:
- ⚠️ **LaTeX Learning Curve**: Customizing templates requires LaTeX expertise
- ⚠️ **Build Complexity**: Multiple tools (Pandoc, pdflatex, biber for citations)
- ⚠️ **EPUB Limitations**: Pandoc EPUB output may need manual tweaks for complex layouts
- ⚠️ **Divergence Risk**: Web version (Docusaurus MDX) may drift from Markdown source if custom components added

**Mitigation**:
- Use Pandoc Lua filters for custom formatting (keeps Markdown as single source)
- Automate EPUB validation with `epubcheck` in CI/CD
- Document Docusaurus-specific MDX syntax in `.specify/templates/chapter-template.md`

### Option 2: LaTeX → PDF/EPUB (LaTeX as Source)
**Source Format**: LaTeX (.tex)
**Print Format**: pdflatex → PDF
**Ebook Format**: tex4ebook → EPUB3
**Web Format**: LaTeXML → HTML

**Pros**:
- ✅ **Print Quality**: Best possible typography for print books
- ✅ **Full Control**: Low-level control over page layout, floats, cross-references
- ✅ **Academic Standard**: Used in academic publishing (Springer, Elsevier)

**Cons**:
- ❌ **Steep Learning Curve**: Authors must learn LaTeX syntax (barrier for contributors)
- ❌ **Git Unfriendly**: LaTeX diffs harder to read than Markdown
- ❌ **Web Conversion**: LaTeXML output less clean than Markdown → HTML
- ❌ **Collaboration**: Harder for non-LaTeX users to contribute (technical reviewers, editors)
- ❌ **EPUB Conversion**: tex4ebook generates suboptimal EPUB (requires manual cleanup)

**Decision**: **REJECTED** - LaTeX learning curve too steep for collaborative writing

### Option 3: AsciiDoc → Asciidoctor → PDF/EPUB/HTML
**Source Format**: AsciiDoc (.adoc)
**Conversion**: Asciidoctor toolchain
**Output**: PDF (via asciidoctor-pdf), EPUB, HTML

**Pros**:
- ✅ **Single Toolchain**: Asciidoctor handles all conversions
- ✅ **Richer Markup**: More semantic than Markdown (admonitions, callouts, includes)
- ✅ **O'Reilly Approved**: Used by O'Reilly for Atlas platform

**Cons**:
- ❌ **PDF Quality**: asciidoctor-pdf not as polished as LaTeX
- ❌ **Math Support**: Limited LaTeX math support (requires AsciiMath or MathJax workarounds)
- ❌ **Community Size**: Smaller community than Markdown or LaTeX
- ❌ **Docusaurus Incompatibility**: No native Docusaurus support for AsciiDoc (requires conversion)

**Decision**: **REJECTED** - Math support and Docusaurus integration weaker than Markdown

### Option 4: Word/Google Docs → Publisher Tools
**Source Format**: Microsoft Word (.docx) or Google Docs
**Conversion**: Publisher-specific tools (InDesign, Framemaker)

**Pros**:
- ✅ **Familiar**: Most authors know Word/Google Docs
- ✅ **WYSIWYG**: Visual editing, no markup syntax

**Cons**:
- ❌ **Version Control**: Binary format (Word) or cloud-only (Google Docs) - poor Git integration
- ❌ **Code Formatting**: Weak syntax highlighting, code blocks hard to manage
- ❌ **Math**: Equation editor clunky for LaTeX equations
- ❌ **Automation**: Harder to automate builds, validation, CI/CD
- ❌ **Lock-In**: Dependent on Microsoft/Google platforms

**Decision**: **REJECTED** - Not suitable for technical book with code examples and version control requirements

## Decision

**SELECTED: Option 1 - Markdown → Pandoc → LaTeX → PDF/EPUB (with Docusaurus for Web)**

### Rationale

1. **Single Source Truth**: Write once in Markdown, publish to print (PDF), ebook (EPUB), and web (HTML)
2. **Version Control**: Plain text Markdown enables Git-based collaboration, easy code review
3. **LaTeX Print Quality**: Professional typography for print edition (TeX line breaking, microtypography)
4. **Pandoc Flexibility**: Supports citations (BibTeX), cross-references, custom filters
5. **Developer-Friendly**: Markdown familiar to software developers (GitHub README, documentation)
6. **Automation**: Makefile + CI/CD automates builds, validation, testing
7. **Math Support**: LaTeX equations render correctly in all formats (PDF, EPUB with MathML, Web with KaTeX)
8. **Industry Proven**: Used by O'Reilly Atlas, Manning, Pragmatic Programmers

### Why Not LaTeX as Source?

While LaTeX produces the best print quality:
- **Collaboration**: LaTeX syntax too complex for non-expert reviewers and contributors
- **Git Diffs**: LaTeX diffs harder to read than Markdown
- **Web Publishing**: Markdown → HTML cleaner than LaTeX → HTML conversion

### Why Not AsciiDoc?

AsciiDoc is a strong alternative, but:
- **Math Support**: LaTeX math support weaker than Markdown + Pandoc
- **PDF Quality**: asciidoctor-pdf not as polished as Pandoc → LaTeX → PDF
- **Docusaurus**: No native AsciiDoc support (would require conversion step)

## Implementation Strategy

### Writing Workflow

**Phase 1: Outline and Structure**
1. Create chapter outline in `.specify/templates/chapter-template.md`
2. Define learning objectives, sections, labs, word count budget
3. Review with co-authors and technical advisors

**Phase 2: First Draft**
1. Write in Markdown following chapter template
2. Use VS Code with extensions:
   - `yzhang.markdown-all-in-one` (shortcuts, table of contents)
   - `DavidAnson.vscode-markdownlint` (style enforcement)
   - `bierner.markdown-mermaid` (diagrams)
3. Commit to Git feature branch (`chapter/05-gazebo-basics`)
4. Run local validation: `make validate-chapter chapter=05`

**Phase 3: Code Examples and Labs**
1. Write code in companion repo: `code/chapter-05-gazebo/`
2. Test code on all hardware tiers (Budget, Mid, Premium)
3. Embed code in Markdown with syntax highlighting:
   ```python
   # code/chapter-05-gazebo/launch_humanoid.py
   import rclpy
   from rclpy.node import Node
   ...
   ```
4. Link to full code: `See [launch_humanoid.py](https://github.com/.../launch_humanoid.py)`

**Phase 4: Review and Feedback**
1. Create pull request for chapter
2. Request reviews from:
   - Technical reviewers (verify code accuracy)
   - Subject matter experts (verify theory)
   - Copy editor (grammar, style)
3. Address feedback, iterate
4. CI/CD runs automated checks (see below)

**Phase 5: Build and Validate**
1. Merge to main branch
2. CI/CD builds PDF, EPUB, Web versions
3. Validate outputs:
   - PDF: Check page breaks, figure placement
   - EPUB: Run `epubcheck` for validation
   - Web: Check Lighthouse accessibility score
4. Tag release: `git tag v1.0-chapter-05`

### Pandoc Conversion Pipeline

**Markdown → LaTeX → PDF** (Print Edition):
```bash
# Makefile target: make pdf
pandoc book/chapters/part*/*.md \
  --from markdown+smart+tex_math_dollars \
  --to latex \
  --template .pandoc/template.latex \
  --pdf-engine pdflatex \
  --toc --toc-depth 2 \
  --number-sections \
  --top-level-division chapter \
  --listings \
  --bibliography references.bib \
  --citeproc --csl .pandoc/apa.csl \
  -o output/book.pdf
```

**Markdown → EPUB3** (Ebook):
```bash
# Makefile target: make epub
pandoc book/chapters/part*/*.md \
  --from markdown+smart \
  --to epub3 \
  --toc --toc-depth 2 \
  --epub-cover-image book/images/cover.png \
  --epub-metadata book/frontmatter/metadata.xml \
  --mathml \
  --bibliography references.bib \
  --citeproc --csl .pandoc/apa.csl \
  -o output/book.epub

# Validate EPUB
epubcheck output/book.epub
```

**Markdown → HTML** (Web via Docusaurus):
```bash
# Docusaurus build (already configured in T003)
cd website && npm run build
```

### LaTeX Template Customization

**Book Layout** (`.pandoc/template.latex`):
- **Page Size**: 7" × 10" (standard technical book size)
- **Margins**: 1" inner, 0.75" outer (binding-friendly)
- **Font**: 11pt Linux Libertine (serif), Inconsolata (monospace for code)
- **Line Spacing**: 1.2 (comfortable reading)
- **Chapters**: Start on right-hand page (`openright`)
- **Figures**: Place near reference (`[htbp]` float specifier)

**Code Listings** (Syntax Highlighting):
```latex
\usepackage{listings}
\lstset{
  basicstyle=\ttfamily\small,
  breaklines=true,
  frame=single,
  numbers=left,
  numberstyle=\tiny\color{gray},
  keywordstyle=\color{blue},
  commentstyle=\color{green!60!black},
  stringstyle=\color{red},
}
```

**Bibliography** (APA 7th Edition):
```latex
\usepackage[backend=biber,style=apa,sorting=nyt]{biblatex}
\addbibresource{references.bib}
```

### Citation and Bibliography Management

**BibTeX Format** (`references.bib`):
```bibtex
@book{lynch2017modern,
  author = {Lynch, Kevin M. and Park, Frank C.},
  year = {2017},
  title = {Modern Robotics: Mechanics, Planning, and Control},
  publisher = {Cambridge University Press},
  isbn = {978-1107156302},
  doi = {10.1017/9781316661239}
}

@inproceedings{brohan2023rt2,
  author = {Brohan, Anthony and Brown, Noah and Carbajal, Justice and others},
  title = {RT-2: Vision-Language-Action Models Transfer Web Knowledge to Robotic Control},
  booktitle = {Conference on Robot Learning (CoRL)},
  year = {2023},
  url = {https://robotics-transformer-x.github.io/}
}
```

**In-Text Citations** (Pandoc Markdown):
```markdown
Humanoid locomotion requires bipedal stability control [@lynch2017modern, Chapter 11].

Recent VLA models like RT-2 demonstrate zero-shot generalization to novel tasks [@brohan2023rt2].
```

**Bibliography Rendering**:
- **Print (PDF)**: Rendered via `biblatex` with APA style
- **Ebook (EPUB)**: Rendered via Pandoc `--citeproc` with APA CSL
- **Web (HTML)**: Rendered via Docusaurus with manual citations (or remark-cite plugin)

### Style Guide and Consistency

**Markdown Style** (Enforced by markdownlint):
- **Headings**: ATX style (`#` syntax, not underline style)
- **Lists**: Consistent indentation (2 spaces)
- **Line Length**: 120 characters (wrap at sentence boundaries)
- **Code Blocks**: Always specify language for syntax highlighting
- **Links**: Use reference-style for repeated links

**Technical Writing Best Practices**:
1. **Active Voice**: "The robot moves" (not "The robot is moved")
2. **Present Tense**: "ROS 2 uses DDS" (not "ROS 2 will use DDS")
3. **Second Person**: "You will install Isaac Sim" (not "The reader installs...")
4. **Consistent Terminology**: "humanoid robot" (not "humanoid" / "robot" / "platform" interchangeably)
5. **Avoid Jargon**: Define acronyms on first use: "Vision-Language-Action (VLA) model"

**Code Style** (see T005 for Python config):
- **Python**: Black formatter (88 chars), flake8 linter
- **C++**: ROS 2 style guide (Google style with modifications)
- **YAML**: 2-space indentation, no tabs

### Validation and Testing

**Automated Checks (CI/CD)**:
1. **Markdown Linting**: `markdownlint book/chapters/**/*.md`
2. **Spell Check**: `aspell --mode=markdown check *.md`
3. **Link Check**: `markdown-link-check --quiet book/chapters/**/*.md`
4. **Code Validation**: Extract code blocks, run syntax check
5. **Build Test**: `make pdf epub` (verify builds succeed)
6. **EPUB Validation**: `epubcheck output/book.epub`
7. **Word Count**: Verify each chapter within budget (see T025)

**Manual Review Checklist**:
- [ ] Learning objectives align with chapter content
- [ ] All code examples tested on Budget tier hardware
- [ ] All figures have captions and cross-references
- [ ] All citations in APA format with DOI/URL
- [ ] Labs have clear step-by-step instructions
- [ ] End-of-chapter exercises have solutions (instructor resources)
- [ ] Accessibility: Alt text for all images, WCAG 2.1 AA

### Feedback and Iteration

**External Reviewers** (per plan.md FR-019):
- **Technical Reviewers** (3-5): Verify code accuracy, hardware compatibility
- **Academic Reviewers** (2-3): Verify pedagogical approach, learning progression
- **Industry Practitioners** (2-3): Verify real-world relevance, tool choices
- **Student Testers** (10-15): Complete labs, provide feedback on clarity

**Review Workflow**:
1. Share draft chapters via GitHub pull requests
2. Reviewers comment directly on Markdown files
3. Author addresses feedback, marks comments resolved
4. Iterate until consensus reached
5. Merge to main branch

**Feedback Tracking**:
- GitHub Issues for major feedback (e.g., "Add section on MPC in Chapter 18")
- Pull Request comments for inline feedback (e.g., "Clarify this sentence")
- Google Form survey for student testers (quantitative feedback)

## Toolchain Dependencies

**Required Tools**:
- **Pandoc**: 3.0+ (universal document converter)
- **LaTeX**: TeX Live 2024 (pdflatex, biber, listings package)
- **Node.js**: 18+ (Docusaurus)
- **Python**: 3.10+ (validation scripts)
- **Git**: 2.30+ (version control)

**Optional Tools**:
- **VS Code**: Markdown editing with extensions
- **epubcheck**: EPUB validation
- **markdownlint-cli**: Markdown linting
- **aspell**: Spell checking

**Installation** (Ubuntu 24.04):
```bash
# Pandoc
sudo apt install pandoc pandoc-citeproc

# LaTeX (TeX Live)
sudo apt install texlive-full  # ~6 GB, includes all packages

# Node.js 18
curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash -
sudo apt install nodejs

# Python tools
pip install black flake8 markdownlint-cli

# EPUB validation
sudo apt install epubcheck
```

## Consequences

### Positive
- ✅ Single-source Markdown enables dual publishing (print, ebook, web) without duplication
- ✅ Git version control provides complete history, easy collaboration
- ✅ Pandoc automation reduces manual formatting work
- ✅ LaTeX print quality matches professional publishers (O'Reilly, Manning)
- ✅ CI/CD validates every commit (broken links, spell check, build errors)
- ✅ Developer-friendly toolchain (Markdown, Git, CLI tools) lowers barrier for contributors
- ✅ Open-source toolchain (Pandoc, LaTeX, Docusaurus) - no licensing costs

### Negative
- ❌ LaTeX template customization requires LaTeX expertise (mitigated by starting with template from T004)
- ❌ Build complexity (Pandoc, pdflatex, biber) requires Makefile documentation
- ❌ EPUB output may need manual tweaks for complex layouts (tables, multi-column)
- ❌ Docusaurus MDX may diverge from Markdown source if custom components added

### Neutral
- ⚖️ PDF build time ~30-60 seconds for full book (acceptable for CI/CD)
- ⚖️ Requires TeX Live install (~6 GB disk space)
- ⚖️ Multiple output formats require testing on each (PDF, EPUB, Web)

## Validation Against Requirements

- ✅ **Dual Publishing**: Markdown → PDF (print), EPUB (ebook), HTML (web)
- ✅ **Version Control**: Git-friendly plain text format
- ✅ **Code Validation**: CI/CD runs syntax checks on embedded code
- ✅ **Citations**: BibTeX → Pandoc citeproc → APA 7th edition
- ✅ **Style Consistency**: markdownlint enforces Markdown style
- ✅ **Collaboration**: Pull request workflow for reviews
- ✅ **Automation**: Makefile + CI/CD automates builds and validation

## References

- Pandoc User Guide: https://pandoc.org/MANUAL.html
- O'Reilly Atlas Platform: https://docs.atlas.oreilly.com/writing_in_asciidoc.html
- Manning Author Guidelines: https://www.manning.com/write-for-us
- Pragmatic Bookshelf Markdown Guide: https://pragprog.com/titles/
- LaTeX Book Class: https://ctan.org/pkg/book
- epubcheck: https://www.w3.org/publishing/epubcheck/
- Technical Writing Style Guide: https://developers.google.com/style

## Revision History

- 2025-12-06: Initial decision - Selected Markdown → Pandoc → LaTeX → PDF/EPUB workflow with Docusaurus for web
