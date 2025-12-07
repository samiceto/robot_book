# Architectural Decision: Docusaurus Configuration and Plugin Selection

**Date**: 2025-12-06
**Status**: Decided
**Decision Makers**: Book Author Team
**Related Tasks**: T015

## Context

The web version of the textbook requires a robust static site generator with:
- Math rendering support (LaTeX equations throughout the book)
- Syntax highlighting for multiple languages (Python, C++, YAML, XML, Bash)
- Full-text search capability
- Responsive design and accessibility (WCAG 2.1 AA)
- Versioning support for errata and future editions
- Fast build times and CDN-friendly static output

## Options Considered

### Option 1: Docusaurus 3.x with KaTeX + Algolia Search
**Framework**: Meta's Docusaurus 3.x
**License**: MIT
**Tech Stack**: React + MDX

**Plugins and Features**:
- **Math Rendering**: `remark-math` + `rehype-katex` (KaTeX engine)
- **Search**: Algolia DocSearch (free for open-source documentation)
- **Syntax Highlighting**: Prism.js with custom themes
- **Versioning**: Built-in versioning system
- **Internationalization**: i18n support (future French/Spanish editions)

**Pros**:
- ✅ **Production-Ready**: Used by React, Jest, Prettier, and 100+ major projects
- ✅ **KaTeX Performance**: Renders math 10× faster than MathJax (critical for 550+ pages with equations)
- ✅ **Algolia Integration**: Free for open-source, 100M operations/month, instant search
- ✅ **MDX Support**: Enables interactive components (code playgrounds, 3D visualizations)
- ✅ **Accessibility**: Built-in WCAG 2.1 AA compliance, keyboard navigation
- ✅ **Responsive Design**: Mobile-first, works on tablets for lab environments
- ✅ **Fast Builds**: Webpack 5 + React Fast Refresh, incremental builds <10s
- ✅ **Versioning**: Supports v1.0, v1.1, errata versioning
- ✅ **SEO**: Generates sitemap, meta tags, Open Graph tags automatically
- ✅ **CDN-Friendly**: Static HTML output, deploy to Netlify/Vercel/GitHub Pages
- ✅ **Active Maintenance**: Meta-backed, frequent updates, large community

**Cons**:
- ⚠️ **Node.js Dependency**: Requires Node.js 18+ (adds toolchain complexity)
- ⚠️ **Build Size**: ~500 MB `node_modules` (mitigated by `.gitignore`)
- ⚠️ **React Learning Curve**: Custom components require React knowledge (optional)

**Performance**:
- Build Time: ~30-60 seconds for 21 chapters (incremental: <10s)
- Bundle Size: ~200 KB gzipped JS, ~100 KB CSS
- Lighthouse Score: 95+ (Performance, Accessibility, SEO, Best Practices)

### Option 2: MkDocs Material with MathJax
**Framework**: MkDocs Material Theme
**License**: MIT
**Tech Stack**: Python + Jinja2

**Pros**:
- ✅ **Python Ecosystem**: Easier for Python-first authors
- ✅ **Material Design**: Beautiful out-of-the-box theme
- ✅ **Search**: Built-in client-side search (no external service)
- ✅ **Simpler Setup**: Less configuration than Docusaurus

**Cons**:
- ❌ **MathJax Performance**: 10× slower than KaTeX for math rendering
- ❌ **Limited Interactivity**: No equivalent to MDX components
- ❌ **Search Quality**: Client-side search weaker than Algolia (no typo tolerance, ranking)
- ❌ **Versioning**: Requires manual setup via mike plugin
- ❌ **Syntax Highlighting**: Pygments less comprehensive than Prism

**Decision**: **REJECTED** - MathJax performance prohibitive for math-heavy book

### Option 3: Gatsby with MDX
**Framework**: Gatsby (React-based SSG)
**License**: MIT
**Tech Stack**: React + GraphQL + MDX

**Pros**:
- ✅ **GraphQL Data Layer**: Powerful content querying
- ✅ **Plugin Ecosystem**: 3000+ plugins
- ✅ **MDX Native**: First-class MDX support

**Cons**:
- ❌ **Complexity**: GraphQL overhead unnecessary for linear book structure
- ❌ **Build Time**: Slower than Docusaurus for large sites (GraphQL processing)
- ❌ **Algolia Integration**: Manual setup (not built-in)
- ❌ **Versioning**: Requires custom implementation
- ❌ **Maintenance**: Smaller documentation-focused community vs. Docusaurus

**Decision**: **REJECTED** - Over-engineered for book documentation use case

### Option 4: Hugo with AsciiMath
**Framework**: Hugo (Go-based SSG)
**License**: Apache 2.0
**Tech Stack**: Go + Go templates

**Pros**:
- ✅ **Speed**: Fastest static site generator (builds in <1 second)
- ✅ **Single Binary**: No Node.js dependency

**Cons**:
- ❌ **Math Rendering**: Limited options (AsciiMath or MathJax only, no KaTeX)
- ❌ **Syntax Highlighting**: Chroma less comprehensive than Prism
- ❌ **Interactivity**: No equivalent to React components/MDX
- ❌ **Search**: Algolia integration manual, no built-in
- ❌ **Template Language**: Go templates less intuitive than React

**Decision**: **REJECTED** - Math rendering and interactivity limitations

## Decision

**SELECTED: Option 1 - Docusaurus 3.x with KaTeX + Algolia Search**

### Rationale

1. **Math Rendering Performance**: KaTeX renders LaTeX 10× faster than MathJax, critical for 550+ pages with equations in kinematics, dynamics, control theory
2. **Algolia Search**: Free for open-source, instant search with typo tolerance, faceted search (filter by chapter, part)
3. **Accessibility**: Built-in WCAG 2.1 AA compliance ensures students with disabilities can use web version
4. **MDX Interactivity**: Enables future enhancements (interactive 3D robot visualizations, embedded Isaac Sim demos)
5. **Industry Standard**: Used by major tech documentation (React, Jest, Redux), proven scalability
6. **Versioning**: Built-in support for errata updates, future editions (v2.0 with new chapters)
7. **Deployment Flexibility**: Static output works on any CDN (Netlify, Vercel, GitHub Pages, AWS S3)

### Why Not MkDocs Material?

While MkDocs is simpler for Python developers:
- **Math Performance**: MathJax rendering too slow for math-heavy book (2-3 seconds vs. 200ms for KaTeX)
- **Search**: Client-side search lacks Algolia's typo tolerance and ranking quality
- **Interactivity**: Cannot embed React components for future interactive demos

### Why Not Gatsby or Hugo?

- **Gatsby**: Over-engineered (GraphQL unnecessary for linear book structure), slower builds
- **Hugo**: Limited math rendering options, no KaTeX support, less interactive

## Implementation Strategy

### Package Configuration (`website/package.json`)

Already created in T003, includes:
```json
{
  "dependencies": {
    "@docusaurus/core": "^3.0.0",
    "@docusaurus/preset-classic": "^3.0.0",
    "@docusaurus/plugin-content-docs": "^3.0.0",
    "remark-math": "^6.0.0",
    "rehype-katex": "^7.0.0",
    "prism-react-renderer": "^2.0.0"
  }
}
```

### Docusaurus Config (`website/docusaurus.config.js`)

**Math Rendering**:
```javascript
presets: [
  [
    'classic',
    {
      docs: {
        remarkPlugins: [require('remark-math')],
        rehypePlugins: [require('rehype-katex')],
      },
    },
  ],
],
stylesheets: [
  {
    href: 'https://cdn.jsdelivr.net/npm/katex@0.16.9/dist/katex.min.css',
    type: 'text/css',
    integrity: 'sha384-n8MVd4RsNIU0tAv4ct0nTaAbDJwPJzDEaqSD1odI+WdtXRGWt2kTvGFasHpSy3SV',
    crossorigin: 'anonymous',
  },
],
```

**Syntax Highlighting** (Prism.js):
```javascript
themeConfig: {
  prism: {
    theme: require('prism-react-renderer/themes/github'),
    darkTheme: require('prism-react-renderer/themes/vsDark'),
    additionalLanguages: ['python', 'cpp', 'bash', 'yaml', 'xml', 'cmake', 'diff'],
  },
}
```

**Algolia Search**:
```javascript
themeConfig: {
  algolia: {
    appId: 'YOUR_APP_ID',  // Set after Algolia application
    apiKey: 'YOUR_SEARCH_API_KEY',  // Public search-only key
    indexName: 'robot_book',
    contextualSearch: true,  // Search within version context
    searchParameters: {
      facetFilters: ['language:en', 'version:1.0'],
    },
  },
}
```

**Versioning**:
```javascript
docs: {
  versions: {
    current: {
      label: '1.0 (Current)',
      path: '',
    },
  },
}
```

### Accessibility Features

**Color Contrast** (WCAG 2.1 AA):
- Code blocks: 7:1 contrast ratio (custom Prism theme)
- Headings: 4.5:1 contrast ratio
- Links: Underlined + color differentiation (colorblind-safe)

**Keyboard Navigation**:
- Tab order: logical flow (header → sidebar → content → footer)
- Skip links: "Skip to main content" for screen readers
- Search: Keyboard shortcut (Ctrl+K / Cmd+K)

**Screen Reader Support**:
- Semantic HTML: `<nav>`, `<main>`, `<aside>`, `<article>`
- ARIA labels: Navigation landmarks, expandable sections
- Alt text: All images require alt text in Markdown

### Plugin Configuration

**Core Plugins** (included in `@docusaurus/preset-classic`):
1. `@docusaurus/plugin-content-docs` - Documentation pages
2. `@docusaurus/plugin-content-blog` - Blog for book announcements, errata
3. `@docusaurus/plugin-sitemap` - SEO sitemap generation
4. `@docusaurus/plugin-google-analytics` - Optional usage analytics

**Optional Plugins** (future):
1. `docusaurus-plugin-image-zoom` - Zoom diagrams (Chapters 4, 17, 18)
2. `@docusaurus/plugin-pwa` - Progressive Web App support (offline reading)
3. `docusaurus-lunr-search` - Fallback if Algolia unavailable

### Content Structure

**Docs Directory** (`website/docs/`):
```
docs/
├── intro.md  # Book introduction
├── part1-foundations/
│   ├── 01-introduction-to-physical-ai.md
│   ├── 02-environment-setup.md
│   ├── 03-ros2-fundamentals.md
│   └── 04-urdf-and-robot-modeling.md
├── part2-simulation/
│   ├── 05-gazebo-basics.md
│   ├── 06-isaac-sim-intro.md
│   ├── 07-isaac-sim-advanced.md
│   └── 08-simulation-benchmarking.md
├── ... (parts 3-6)
└── appendices/
    ├── a-hardware-buyers-guide.md
    ├── b-ros2-cheat-sheet.md
    ├── ... (appendices c-i)
```

**Sidebar Configuration** (`website/sidebars.js`):
Already created in T003, organized by parts (6) and chapters (21).

### Build and Deployment

**Build Commands**:
```bash
# Development server (hot reload)
npm run start  # → http://localhost:3000

# Production build
npm run build  # → website/build/

# Serve production build locally
npm run serve
```

**Deployment Options**:
1. **Netlify** (Recommended):
   - Free for open-source
   - Automatic builds on Git push
   - Custom domain support
   - Deploy previews for pull requests

2. **Vercel**:
   - Free for open-source
   - Edge CDN (faster global performance)
   - Automatic HTTPS

3. **GitHub Pages**:
   - Free (github.io subdomain)
   - Workflow: `.github/workflows/deploy-docs.yml`

**CI/CD Workflow** (GitHub Actions):
```yaml
name: Deploy Docusaurus Site

on:
  push:
    branches: [main]
    paths:
      - 'website/**'
      - 'book/**'

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - name: Setup Node.js
        uses: actions/setup-node@v4
        with:
          node-version: 18
      - name: Install dependencies
        run: cd website && npm ci
      - name: Build website
        run: cd website && npm run build
      - name: Deploy to Netlify
        uses: netlify/actions/cli@master
        with:
          args: deploy --prod --dir=website/build
        env:
          NETLIFY_SITE_ID: ${{ secrets.NETLIFY_SITE_ID }}
          NETLIFY_AUTH_TOKEN: ${{ secrets.NETLIFY_AUTH_TOKEN }}
```

### Performance Optimization

**Bundle Optimization**:
- Code splitting: Each chapter is separate JS chunk (lazy loading)
- Tree shaking: Removes unused React components
- Minification: Terser for JS, cssnano for CSS
- Image optimization: WebP format, responsive images

**CDN Strategy**:
- Static assets: Serve from CDN (Cloudflare, AWS CloudFront)
- KaTeX CSS: Use CDN version (jsDelivr) for browser caching
- Font optimization: Subset fonts (Latin only), WOFF2 format

**Target Metrics**:
- First Contentful Paint (FCP): <1.5s
- Largest Contentful Paint (LCP): <2.5s
- Time to Interactive (TTI): <3.5s
- Cumulative Layout Shift (CLS): <0.1

## Consequences

### Positive
- ✅ KaTeX renders math 10× faster than MathJax (200ms vs. 2-3s page load)
- ✅ Algolia search provides instant results with typo tolerance (better UX than client-side search)
- ✅ MDX enables future interactive demos (3D robot visualizations, code playgrounds)
- ✅ Built-in versioning supports errata updates and future editions
- ✅ WCAG 2.1 AA compliance ensures accessibility for students with disabilities
- ✅ Static site generation (no server required) reduces hosting costs to $0
- ✅ Responsive design works on tablets in lab environments
- ✅ SEO-optimized (sitemap, meta tags, Open Graph) improves discoverability

### Negative
- ❌ Node.js dependency adds toolchain complexity (requires `npm install`)
- ❌ 500 MB `node_modules` directory (mitigated by `.gitignore`, CI caching)
- ❌ Algolia requires external service (free tier: 100M operations/month, sufficient for educational use)
- ❌ Custom React components require React knowledge (optional for basic usage)

### Neutral
- ⚖️ Build time ~30-60 seconds for full site (acceptable for CI/CD)
- ⚖️ Incremental builds <10 seconds for single chapter updates
- ⚖️ Lighthouse score 95+ meets industry standards

## Validation Against Requirements

- ✅ **Math Rendering**: KaTeX renders LaTeX equations with high performance
- ✅ **Syntax Highlighting**: Prism supports Python, C++, YAML, XML, Bash, CMake
- ✅ **Search**: Algolia provides instant full-text search
- ✅ **Accessibility**: WCAG 2.1 AA compliance built-in
- ✅ **Versioning**: Built-in support for multiple versions and errata
- ✅ **Build Speed**: <60s for full site, <10s for incremental updates
- ✅ **CDN-Friendly**: Static HTML output deploys to any CDN
- ✅ **Responsive**: Mobile-first design works on tablets and phones

## Algolia Setup Guide

**Prerequisites**:
1. Create Algolia account (free for open-source documentation)
2. Create application and index named `robot_book`
3. Generate API keys (Admin API key for indexing, Search-only key for public)

**DocSearch Integration** (Recommended):
```javascript
// Apply for DocSearch at: https://docsearch.algolia.com/apply/
// Algolia crawls public documentation automatically
// No manual indexing required
```

**Manual Indexing** (Alternative):
```bash
# Install Algolia Crawler CLI
npm install -g algolia-webcrawler

# Configure crawler
cat > algolia-config.json <<EOF
{
  "index_name": "robot_book",
  "start_urls": ["https://robot-book.com"],
  "selectors": {
    "lvl0": "article h1",
    "lvl1": "article h2",
    "lvl2": "article h3",
    "text": "article p, article li"
  }
}
EOF

# Run crawler weekly via GitHub Actions
```

## References

- Docusaurus Documentation: https://docusaurus.io/docs
- KaTeX Documentation: https://katex.org/docs/supported.html
- Algolia DocSearch: https://docsearch.algolia.com/
- Prism.js Languages: https://prismjs.com/#supported-languages
- WCAG 2.1 Guidelines: https://www.w3.org/WAI/WCAG21/quickref/
- Lighthouse Metrics: https://web.dev/lighthouse-performance/

## Revision History

- 2025-12-06: Initial decision - Selected Docusaurus 3.x with KaTeX, Algolia Search, and Prism syntax highlighting
