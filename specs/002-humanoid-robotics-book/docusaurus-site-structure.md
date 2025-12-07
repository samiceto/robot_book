# Docusaurus Site Structure and Information Architecture

**Date**: 2025-12-06
**Status**: Phase 2 Complete
**Related Tasks**: T029
**Related Decisions**: T015 (Docusaurus Configuration)
**Purpose**: Complete site map, navigation hierarchy, content organization, and deployment workflow for the web version

---

## 1. Site Map and Information Architecture

### 1.1 URL Structure

**Base URL**: `https://robot-book.com` (or `https://<author>.github.io/robot-book/`)

**URL Scheme**:
```
/                                   → Landing page
/docs/intro                          → Book introduction
/docs/part1-foundations/             → Part 1 overview
/docs/part1-foundations/ch01         → Chapter 1
/docs/part1-foundations/ch02         → Chapter 2
/docs/part1-foundations/ch03         → Chapter 3
/docs/part1-foundations/ch04         → Chapter 4
/docs/part2-simulation/              → Part 2 overview
/docs/part2-simulation/ch05          → Chapter 5
...
/docs/part6-capstone/ch20            → Chapter 20
/docs/part6-capstone/ch21            → Chapter 21
/docs/appendices/a-hardware-guide    → Appendix A
/docs/appendices/b-ros2-cheatsheet   → Appendix B
...
/docs/appendices/i-glossary          → Appendix I
/blog                                → Announcements, errata, updates
/instructor-resources                → Instructor-only materials (slides, quizzes, solutions)
```

**Versioning URLs**:
```
/docs/                              → Current version (1.0)
/docs/1.0/                          → Version 1.0 (locked)
/docs/1.1/                          → Version 1.1 (future)
/docs/next/                         → Development version (unreleased chapters)
```

### 1.2 Page Types and Templates

**1. Landing Page** (`website/src/pages/index.js`)
- Hero section: Book cover, title, subtitle, "Start Reading" CTA
- Feature cards: "Free Web Version", "Interactive Code Examples", "GPU-Accelerated Simulation"
- Preview: Table of contents preview (6 parts, 21 chapters)
- Instructor resources link
- GitHub repository link

**2. Part Overview Pages** (`docs/partN-<name>/index.md`)
- Part title and learning outcomes
- Chapter listing with descriptions
- Estimated time to complete
- Prerequisites from previous parts
- Key technologies covered
- Companion repository links

**3. Chapter Pages** (`docs/partN-<name>/chNN.md`)
- Chapter title (H1)
- Section headings (H2)
- Subsection headings (H3)
- Code listings with syntax highlighting
- Math equations (KaTeX)
- Diagrams and figures (WebP images)
- Labs and projects in expandable sections
- Navigation: Previous/Next chapter buttons
- Table of contents (right sidebar)

**4. Appendix Pages** (`docs/appendices/<slug>.md`)
- Appendix title
- Reference material (tables, cheat sheets, FAQs)
- Quick navigation sidebar

**5. Blog Pages** (`website/blog/`)
- Announcements (new chapters released)
- Errata updates
- Community contributions
- Author notes

**6. Instructor Resources Page** (`website/src/pages/instructor-resources.md`)
- Password-protected or email-gated access
- Links to slide decks, assignments, quizzes, exam questions, solutions
- Course syllabi templates (13-week semester, 10-week quarter)

---

## 2. Complete Navigation Hierarchy (Sidebar Configuration)

**File**: `website/sidebars.js`

```javascript
module.exports = {
  docs: [
    {
      type: 'doc',
      id: 'intro',
      label: 'Introduction',
    },

    // Part 1: Foundations & ROS 2
    {
      type: 'category',
      label: 'Part 1: Foundations & ROS 2',
      link: {
        type: 'doc',
        id: 'part1-foundations/index',
      },
      collapsed: false,
      items: [
        {
          type: 'doc',
          id: 'part1-foundations/ch01',
          label: 'Ch 1: Introduction to Physical AI',
        },
        {
          type: 'doc',
          id: 'part1-foundations/ch02',
          label: 'Ch 2: Development Environment Setup',
        },
        {
          type: 'doc',
          id: 'part1-foundations/ch03',
          label: 'Ch 3: ROS 2 Fundamentals',
        },
        {
          type: 'doc',
          id: 'part1-foundations/ch04',
          label: 'Ch 4: URDF and Robot Modeling',
        },
      ],
    },

    // Part 2: Digital Twins & Simulation Mastery
    {
      type: 'category',
      label: 'Part 2: Simulation Mastery',
      link: {
        type: 'doc',
        id: 'part2-simulation/index',
      },
      collapsed: true,
      items: [
        {
          type: 'doc',
          id: 'part2-simulation/ch05',
          label: 'Ch 5: Gazebo Basics',
        },
        {
          type: 'doc',
          id: 'part2-simulation/ch06',
          label: 'Ch 6: Isaac Sim Introduction',
        },
        {
          type: 'doc',
          id: 'part2-simulation/ch07',
          label: 'Ch 7: Isaac Sim Advanced Features',
        },
        {
          type: 'doc',
          id: 'part2-simulation/ch08',
          label: 'Ch 8: Simulation Benchmarking',
        },
      ],
    },

    // Part 3: Perception & Edge Brain
    {
      type: 'category',
      label: 'Part 3: Perception & Edge AI',
      link: {
        type: 'doc',
        id: 'part3-perception/index',
      },
      collapsed: true,
      items: [
        {
          type: 'doc',
          id: 'part3-perception/ch09',
          label: 'Ch 9: Computer Vision Fundamentals',
        },
        {
          type: 'doc',
          id: 'part3-perception/ch10',
          label: 'Ch 10: Object Detection with YOLO',
        },
        {
          type: 'doc',
          id: 'part3-perception/ch11',
          label: 'Ch 11: 3D Perception (Point Clouds)',
        },
        {
          type: 'doc',
          id: 'part3-perception/ch12',
          label: 'Ch 12: Deploying on Jetson Edge Devices',
        },
      ],
    },

    // Part 4: Embodied Cognition & VLA Models
    {
      type: 'category',
      label: 'Part 4: Embodied Cognition & VLAs',
      link: {
        type: 'doc',
        id: 'part4-cognition/index',
      },
      collapsed: true,
      items: [
        {
          type: 'doc',
          id: 'part4-cognition/ch13',
          label: 'Ch 13: Imitation Learning Foundations',
        },
        {
          type: 'doc',
          id: 'part4-cognition/ch14',
          label: 'Ch 14: Vision-Language-Action Models',
        },
        {
          type: 'doc',
          id: 'part4-cognition/ch15',
          label: 'Ch 15: Training OpenVLA on Custom Data',
        },
        {
          type: 'doc',
          id: 'part4-cognition/ch16',
          label: 'Ch 16: Fine-Tuning and Prompt Engineering',
        },
      ],
    },

    // Part 5: Bipedal Locomotion & Whole-Body Control
    {
      type: 'category',
      label: 'Part 5: Locomotion & Control',
      link: {
        type: 'doc',
        id: 'part5-locomotion/index',
      },
      collapsed: true,
      items: [
        {
          type: 'doc',
          id: 'part5-locomotion/ch17',
          label: 'Ch 17: Kinematics and Inverse Kinematics',
        },
        {
          type: 'doc',
          id: 'part5-locomotion/ch18',
          label: 'Ch 18: Bipedal Walking with MPC',
        },
        {
          type: 'doc',
          id: 'part5-locomotion/ch19',
          label: 'Ch 19: Whole-Body Control',
        },
      ],
    },

    // Part 6: Capstone Integration & Sim-to-Real Transfer
    {
      type: 'category',
      label: 'Part 6: Capstone & Sim-to-Real',
      link: {
        type: 'doc',
        id: 'part6-capstone/index',
      },
      collapsed: true,
      items: [
        {
          type: 'doc',
          id: 'part6-capstone/ch20',
          label: 'Ch 20: Capstone Project Integration',
        },
        {
          type: 'doc',
          id: 'part6-capstone/ch21',
          label: 'Ch 21: Sim-to-Real Transfer',
        },
      ],
    },

    // Appendices
    {
      type: 'category',
      label: 'Appendices',
      link: {
        type: 'doc',
        id: 'appendices/index',
      },
      collapsed: true,
      items: [
        {
          type: 'doc',
          id: 'appendices/a-hardware-guide',
          label: 'Appendix A: Hardware Buyer\'s Guide',
        },
        {
          type: 'doc',
          id: 'appendices/b-ros2-cheatsheet',
          label: 'Appendix B: ROS 2 Cheat Sheet',
        },
        {
          type: 'doc',
          id: 'appendices/c-isaac-sim-reference',
          label: 'Appendix C: Isaac Sim API Reference',
        },
        {
          type: 'doc',
          id: 'appendices/d-vla-model-zoo',
          label: 'Appendix D: VLA Model Zoo',
        },
        {
          type: 'doc',
          id: 'appendices/e-math-refresher',
          label: 'Appendix E: Math Refresher',
        },
        {
          type: 'doc',
          id: 'appendices/f-troubleshooting',
          label: 'Appendix F: Troubleshooting Guide',
        },
        {
          type: 'doc',
          id: 'appendices/g-further-reading',
          label: 'Appendix G: Further Reading',
        },
        {
          type: 'doc',
          id: 'appendices/h-contributing',
          label: 'Appendix H: Contributing to Companion Code',
        },
        {
          type: 'doc',
          id: 'appendices/i-glossary',
          label: 'Appendix I: Glossary',
        },
      ],
    },
  ],
};
```

---

## 3. Directory Structure

### 3.1 Complete Website Directory Layout

```
website/
├── blog/                                 # Blog posts (announcements, errata)
│   ├── 2025-01-15-book-launch.md
│   ├── 2025-02-10-errata-chapter-5.md
│   └── authors.yml
├── docs/                                 # Main documentation
│   ├── intro.md                          # Book introduction
│   ├── part1-foundations/
│   │   ├── index.md                      # Part 1 overview
│   │   ├── ch01.md                       # Chapter 1
│   │   ├── ch02.md                       # Chapter 2
│   │   ├── ch03.md                       # Chapter 3
│   │   └── ch04.md                       # Chapter 4
│   ├── part2-simulation/
│   │   ├── index.md                      # Part 2 overview
│   │   ├── ch05.md                       # Chapter 5
│   │   ├── ch06.md                       # Chapter 6
│   │   ├── ch07.md                       # Chapter 7
│   │   └── ch08.md                       # Chapter 8
│   ├── part3-perception/
│   │   ├── index.md
│   │   ├── ch09.md
│   │   ├── ch10.md
│   │   ├── ch11.md
│   │   └── ch12.md
│   ├── part4-cognition/
│   │   ├── index.md
│   │   ├── ch13.md
│   │   ├── ch14.md
│   │   ├── ch15.md
│   │   └── ch16.md
│   ├── part5-locomotion/
│   │   ├── index.md
│   │   ├── ch17.md
│   │   ├── ch18.md
│   │   └── ch19.md
│   ├── part6-capstone/
│   │   ├── index.md
│   │   ├── ch20.md
│   │   └── ch21.md
│   └── appendices/
│       ├── index.md
│       ├── a-hardware-guide.md
│       ├── b-ros2-cheatsheet.md
│       ├── c-isaac-sim-reference.md
│       ├── d-vla-model-zoo.md
│       ├── e-math-refresher.md
│       ├── f-troubleshooting.md
│       ├── g-further-reading.md
│       ├── h-contributing.md
│       └── i-glossary.md
├── src/
│   ├── components/                       # Custom React components
│   │   ├── HomepageFeatures/
│   │   │   ├── index.js                  # Feature cards
│   │   │   └── styles.module.css
│   │   ├── CodePlayground/
│   │   │   ├── index.js                  # Interactive code editor
│   │   │   └── styles.module.css
│   │   ├── VideoPlayer/
│   │   │   ├── index.js                  # Embedded video player
│   │   │   └── styles.module.css
│   │   ├── LabSection/
│   │   │   ├── index.js                  # Expandable lab instructions
│   │   │   └── styles.module.css
│   │   └── FigureCaption/
│   │       ├── index.js                  # Image with caption
│   │       └── styles.module.css
│   ├── css/
│   │   └── custom.css                    # Global styles
│   └── pages/
│       ├── index.js                      # Landing page
│       ├── instructor-resources.md       # Instructor materials
│       └── styles.module.css
├── static/                               # Static assets
│   ├── img/
│   │   ├── logo.svg                      # Book logo
│   │   ├── cover.png                     # Book cover
│   │   ├── ch01/                         # Chapter 1 images
│   │   │   ├── fig-1-1-physical-ai-stack.webp
│   │   │   ├── fig-1-2-humanoid-landscape.webp
│   │   │   └── ...
│   │   ├── ch02/                         # Chapter 2 images
│   │   │   └── ...
│   │   └── ... (ch03-ch21, appendices)
│   ├── videos/                           # Demo videos (YouTube embeds preferred)
│   │   ├── ch02-lab-demo.mp4
│   │   └── ...
│   └── files/                            # Downloadable files
│       ├── errata.pdf
│       └── citation.bib
├── docusaurus.config.js                  # Main configuration
├── sidebars.js                           # Sidebar configuration
├── package.json                          # Dependencies
├── babel.config.js                       # Babel configuration
└── .gitignore                            # Ignore node_modules, build/
```

---

## 4. Custom React Components

### 4.1 LabSection Component

**Purpose**: Expandable/collapsible lab instructions to reduce page length
**Location**: `website/src/components/LabSection/index.js`

```javascript
import React, { useState } from 'react';
import styles from './styles.module.css';

export default function LabSection({ title, timeEstimate, setup, tasks, validation, troubleshooting }) {
  const [isExpanded, setIsExpanded] = useState(false);

  return (
    <div className={styles.labSection}>
      <button
        className={styles.labHeader}
        onClick={() => setIsExpanded(!isExpanded)}
      >
        <span className={styles.labIcon}>🧪</span>
        <h3>{title}</h3>
        <span className={styles.timeEstimate}>⏱ {timeEstimate}</span>
        <span className={styles.expandIcon}>{isExpanded ? '▼' : '▶'}</span>
      </button>

      {isExpanded && (
        <div className={styles.labContent}>
          <div className={styles.labSetup}>
            <h4>Setup</h4>
            <p>{setup}</p>
          </div>

          <div className={styles.labTasks}>
            <h4>Tasks</h4>
            <ol>
              {tasks.map((task, index) => (
                <li key={index}>{task}</li>
              ))}
            </ol>
          </div>

          <div className={styles.labValidation}>
            <h4>Validation</h4>
            <p>{validation}</p>
          </div>

          <div className={styles.labTroubleshooting}>
            <h4>Troubleshooting</h4>
            <ul>
              {troubleshooting.map((item, index) => (
                <li key={index}>{item}</li>
              ))}
            </ul>
          </div>
        </div>
      )}
    </div>
  );
}
```

**Usage in MDX**:
```mdx
import LabSection from '@site/src/components/LabSection';

<LabSection
  title="Hands-On Lab: Building a Multi-Node System"
  timeEstimate="8 hours"
  setup="ROS 2 Iron environment from Chapter 2"
  tasks={[
    "Create a sensor_simulator node publishing /imu at 50 Hz",
    "Create a filter node subscribing to /imu",
    "Create a calibration_service node offering /calibrate_imu service",
    "Write a launch file starting all nodes"
  ]}
  validation="Launch file starts all nodes, topics publish at expected rates"
  troubleshooting={[
    "Node crashes: Check Python syntax",
    "Topic rate issues: Verify QoS policies",
    "Service timeouts: Check service definition"
  ]}
/>
```

### 4.2 CodePlayground Component

**Purpose**: Interactive code editor with syntax highlighting and "Run" button (future feature)
**Location**: `website/src/components/CodePlayground/index.js`

```javascript
import React, { useState } from 'react';
import CodeBlock from '@theme/CodeBlock';
import styles from './styles.module.css';

export default function CodePlayground({ code, language, editable = false }) {
  const [currentCode, setCurrentCode] = useState(code);
  const [output, setOutput] = useState('');

  const handleRun = async () => {
    // Future: Integrate with CodeSandbox API or custom Python interpreter
    setOutput('Feature coming soon: Execute code in browser');
  };

  return (
    <div className={styles.playground}>
      <CodeBlock language={language}>
        {currentCode}
      </CodeBlock>
      {editable && (
        <button className={styles.runButton} onClick={handleRun}>
          ▶ Run Code
        </button>
      )}
      {output && (
        <div className={styles.output}>
          <h4>Output:</h4>
          <pre>{output}</pre>
        </div>
      )}
    </div>
  );
}
```

### 4.3 FigureCaption Component

**Purpose**: Consistent figure rendering with captions and alt text
**Location**: `website/src/components/FigureCaption/index.js`

```javascript
import React from 'react';
import styles from './styles.module.css';

export default function FigureCaption({ src, alt, caption, figNumber }) {
  return (
    <figure className={styles.figure}>
      <img src={src} alt={alt} loading="lazy" />
      <figcaption>
        <strong>Figure {figNumber}:</strong> {caption}
      </figcaption>
    </figure>
  );
}
```

**Usage in MDX**:
```mdx
import FigureCaption from '@site/src/components/FigureCaption';

<FigureCaption
  src="/img/ch01/fig-1-1-physical-ai-stack.webp"
  alt="Physical AI stack diagram showing perception, cognition, and action layers"
  caption="The Physical AI Stack: Perception → Cognition → Action"
  figNumber="1-1"
/>
```

### 4.4 VideoPlayer Component

**Purpose**: Embedded YouTube videos with fallback for self-hosted MP4
**Location**: `website/src/components/VideoPlayer/index.js`

```javascript
import React from 'react';
import styles from './styles.module.css';

export default function VideoPlayer({ youtubeId, fallbackSrc, title }) {
  return (
    <div className={styles.videoContainer}>
      {youtubeId ? (
        <iframe
          className={styles.video}
          src={`https://www.youtube.com/embed/${youtubeId}`}
          title={title}
          frameBorder="0"
          allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture"
          allowFullScreen
        />
      ) : (
        <video className={styles.video} controls>
          <source src={fallbackSrc} type="video/mp4" />
          Your browser does not support the video tag.
        </video>
      )}
      <p className={styles.videoCaption}>{title}</p>
    </div>
  );
}
```

---

## 5. Content Migration Strategy

### 5.1 Source Material Location

**Book Chapters**: `book/chapters/partN/<filename>.md`
**Destination**: `website/docs/partN-<name>/chNN.md`

### 5.2 Migration Script

**File**: `scripts/migrate-to-docusaurus.py`

```python
#!/usr/bin/env python3
"""
Migrate Markdown chapters from book/ to website/docs/ with Docusaurus formatting.
"""

import os
import re
from pathlib import Path

def add_frontmatter(content, chapter_num, chapter_title, part_num, part_name):
    """Add Docusaurus frontmatter to chapter Markdown."""
    frontmatter = f"""---
id: ch{chapter_num:02d}
title: "Chapter {chapter_num}: {chapter_title}"
sidebar_label: "Ch {chapter_num}: {chapter_title}"
sidebar_position: {chapter_num}
description: "{chapter_title}"
keywords:
  - robotics
  - physical ai
  - humanoid robotics
  - ROS 2
---

"""
    return frontmatter + content

def convert_code_blocks(content):
    """Convert code blocks to Docusaurus format with titles."""
    # Replace generic code blocks with titled versions
    # Before: ```python
    # After: ```python title="example.py"
    # (Manual review required for actual file names)
    return content

def convert_math_blocks(content):
    """Ensure KaTeX-compatible math blocks."""
    # Inline math: $...$
    # Display math: $$...$$
    return content

def convert_images(content, chapter_num):
    """Convert image paths to Docusaurus static paths."""
    # Before: ![Alt text](images/fig-1-1.png)
    # After: ![Alt text](/img/ch01/fig-1-1.webp)
    def replace_image(match):
        alt_text = match.group(1)
        filename = match.group(2)
        # Convert PNG/JPG to WebP, update path
        new_filename = filename.replace('.png', '.webp').replace('.jpg', '.webp')
        new_path = f"/img/ch{chapter_num:02d}/{new_filename}"
        return f"![{alt_text}]({new_path})"

    pattern = r'!\[(.*?)\]\((.*?)\)'
    return re.sub(pattern, replace_image, content)

def migrate_chapter(source_path, dest_path, chapter_num, chapter_title, part_num, part_name):
    """Migrate a single chapter from book/ to website/docs/."""
    with open(source_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Apply transformations
    content = add_frontmatter(content, chapter_num, chapter_title, part_num, part_name)
    content = convert_code_blocks(content)
    content = convert_math_blocks(content)
    content = convert_images(content, chapter_num)

    # Write to destination
    dest_path.parent.mkdir(parents=True, exist_ok=True)
    with open(dest_path, 'w', encoding='utf-8') as f:
        f.write(content)

    print(f"Migrated: {source_path} → {dest_path}")

def main():
    # Example: Migrate Chapter 1
    migrate_chapter(
        source_path=Path("book/chapters/part1/01-introduction.md"),
        dest_path=Path("website/docs/part1-foundations/ch01.md"),
        chapter_num=1,
        chapter_title="Introduction to Physical AI and Humanoid Robotics",
        part_num=1,
        part_name="Foundations & ROS 2"
    )

    # TODO: Loop through all 21 chapters

if __name__ == "__main__":
    main()
```

**Usage**:
```bash
python scripts/migrate-to-docusaurus.py
```

### 5.3 Image Optimization

**Convert images to WebP format** (10-30% smaller than PNG/JPG):
```bash
# Install cwebp (WebP encoder)
sudo apt install webp

# Batch convert all images
for img in static/img/ch*/*.png; do
  cwebp -q 85 "$img" -o "${img%.png}.webp"
done

for img in static/img/ch*/*.jpg; do
  cwebp -q 85 "$img" -o "${img%.jpg}.webp"
done
```

---

## 6. Docusaurus Configuration File

**File**: `website/docusaurus.config.js`

```javascript
// @ts-check
const {themes} = require('prism-react-renderer');
const lightCodeTheme = themes.github;
const darkCodeTheme = themes.vsDark;

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'From Simulated Brains to Walking Bodies',
  favicon: 'img/favicon.ico',

  // Production URL
  url: 'https://robot-book.com',
  baseUrl: '/',

  // GitHub Pages deployment
  organizationName: 'your-github-username',
  projectName: 'robot-book',

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
          editUrl: 'https://github.com/your-username/robot-book/tree/main/website/',
          remarkPlugins: [require('remark-math')],
          rehypePlugins: [require('rehype-katex')],
          showLastUpdateTime: true,
          showLastUpdateAuthor: true,
        },
        blog: {
          showReadingTime: true,
          feedOptions: {
            type: ['rss', 'atom'],
            xslt: true,
          },
          editUrl: 'https://github.com/your-username/robot-book/tree/main/website/',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
        gtag: {
          trackingID: 'G-XXXXXXXXXX',  // Google Analytics (optional)
          anonymizeIP: true,
        },
      }),
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

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      image: 'img/social-card.png',
      navbar: {
        title: 'Physical AI & Humanoid Robotics',
        logo: {
          alt: 'Robot Book Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'docs',
            position: 'left',
            label: 'Read the Book',
          },
          {to: '/blog', label: 'Blog', position: 'left'},
          {to: '/instructor-resources', label: 'Instructor Resources', position: 'left'},
          {
            type: 'docsVersionDropdown',
            position: 'right',
          },
          {
            href: 'https://github.com/your-username/robot-book',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Book',
            items: [
              {
                label: 'Start Reading',
                to: '/docs/intro',
              },
              {
                label: 'PDF/EPUB Version',
                href: 'https://github.com/your-username/robot-book/releases',
              },
            ],
          },
          {
            title: 'Resources',
            items: [
              {
                label: 'Companion Code (GitHub)',
                href: 'https://github.com/your-username/robot-book-code',
              },
              {
                label: 'Instructor Resources',
                to: '/instructor-resources',
              },
              {
                label: 'Errata',
                to: '/blog/tags/errata',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'Discussions',
                href: 'https://github.com/your-username/robot-book/discussions',
              },
              {
                label: 'Issues',
                href: 'https://github.com/your-username/robot-book/issues',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} [Author Name]. Licensed under CC BY-NC-SA 4.0 (text) and MIT (code).`,
      },
      prism: {
        theme: lightCodeTheme,
        darkTheme: darkCodeTheme,
        additionalLanguages: ['python', 'cpp', 'bash', 'yaml', 'xml', 'cmake', 'diff', 'json'],
      },
      algolia: {
        appId: 'YOUR_APP_ID',
        apiKey: 'YOUR_SEARCH_API_KEY',
        indexName: 'robot_book',
        contextualSearch: true,
        searchParameters: {
          facetFilters: ['language:en', 'version:1.0'],
        },
      },
      tableOfContents: {
        minHeadingLevel: 2,
        maxHeadingLevel: 3,
      },
      docs: {
        sidebar: {
          hideable: true,
          autoCollapseCategories: true,
        },
      },
    }),

  plugins: [
    // Optional: Image zoom plugin
    // [
    //   '@docusaurus/plugin-ideal-image',
    //   {
    //     quality: 85,
    //     max: 2000,
    //     min: 500,
    //     steps: 4,
    //     disableInDev: false,
    //   },
    // ],
  ],
};

module.exports = config;
```

---

## 7. Build and Deployment Workflows

### 7.1 Local Development

```bash
# Install dependencies
cd website
npm install

# Start development server (hot reload at http://localhost:3000)
npm run start

# Build for production
npm run build

# Serve production build locally
npm run serve
```

### 7.2 CI/CD Deployment (GitHub Actions)

**File**: `.github/workflows/deploy-docs.yml`

```yaml
name: Deploy Docusaurus Site

on:
  push:
    branches: [main]
    paths:
      - 'website/**'
      - 'book/chapters/**'
      - '.github/workflows/deploy-docs.yml'
  pull_request:
    branches: [main]
    paths:
      - 'website/**'

jobs:
  build-and-deploy:
    runs-on: ubuntu-latest

    steps:
      - name: Checkout repository
        uses: actions/checkout@v4

      - name: Setup Node.js
        uses: actions/setup-node@v4
        with:
          node-version: 18
          cache: 'npm'
          cache-dependency-path: website/package-lock.json

      - name: Install dependencies
        run: cd website && npm ci

      - name: Build website
        run: cd website && npm run build

      - name: Test build output
        run: |
          test -d website/build
          test -f website/build/index.html
          echo "Build size: $(du -sh website/build | cut -f1)"

      # Option 1: Deploy to Netlify
      - name: Deploy to Netlify
        if: github.ref == 'refs/heads/main'
        uses: netlify/actions/cli@master
        with:
          args: deploy --prod --dir=website/build
        env:
          NETLIFY_SITE_ID: ${{ secrets.NETLIFY_SITE_ID }}
          NETLIFY_AUTH_TOKEN: ${{ secrets.NETLIFY_AUTH_TOKEN }}

      # Option 2: Deploy to GitHub Pages
      # - name: Deploy to GitHub Pages
      #   if: github.ref == 'refs/heads/main'
      #   uses: peaceiris/actions-gh-pages@v3
      #   with:
      #     github_token: ${{ secrets.GITHUB_TOKEN }}
      #     publish_dir: ./website/build

      # Option 3: Deploy to Vercel
      # - name: Deploy to Vercel
      #   if: github.ref == 'refs/heads/main'
      #   uses: amondnet/vercel-action@v25
      #   with:
      #     vercel-token: ${{ secrets.VERCEL_TOKEN }}
      #     vercel-org-id: ${{ secrets.ORG_ID }}
      #     vercel-project-id: ${{ secrets.PROJECT_ID }}
      #     working-directory: ./website
```

### 7.3 Deployment Options Comparison

| Feature | Netlify | Vercel | GitHub Pages |
|---------|---------|--------|--------------|
| **Cost** | Free (open-source) | Free (open-source) | Free |
| **Build Time** | ~2-3 min | ~2-3 min | ~3-5 min |
| **Custom Domain** | ✅ Yes | ✅ Yes | ✅ Yes |
| **HTTPS** | ✅ Auto | ✅ Auto | ✅ Auto |
| **Deploy Previews** | ✅ Yes (PRs) | ✅ Yes (PRs) | ❌ No |
| **CDN** | ✅ Global | ✅ Edge (faster) | ✅ GitHub CDN |
| **Analytics** | ✅ Built-in | ✅ Built-in | ❌ (requires GA) |
| **Redirects** | ✅ `_redirects` | ✅ `vercel.json` | ⚠️ Limited |
| **Recommendation** | **Best for open-source books** | Best for performance | Good for simple sites |

**Recommended**: Netlify for ease of use and deploy previews

---

## 8. Algolia Search Setup

### 8.1 DocSearch Application (Recommended)

**Prerequisites**:
1. Website must be public and indexed
2. Apply at: https://docsearch.algolia.com/apply/

**Application Form**:
- Website URL: `https://robot-book.com`
- Documentation URL: `https://robot-book.com/docs/intro`
- GitHub Repository: `https://github.com/your-username/robot-book`
- Project Description: "Open-source textbook on Physical AI and Humanoid Robotics"

**Approval Process**:
- Algolia reviews within 1-2 weeks
- Provides `appId`, `apiKey`, `indexName` for `docusaurus.config.js`
- Automatic weekly re-indexing

### 8.2 Manual Algolia Crawler (Alternative)

**File**: `algolia-config.json`

```json
{
  "index_name": "robot_book",
  "start_urls": [
    {
      "url": "https://robot-book.com/docs/",
      "selectors_key": "docs",
      "tags": ["docs"]
    },
    {
      "url": "https://robot-book.com/blog/",
      "selectors_key": "blog",
      "tags": ["blog"]
    }
  ],
  "selectors": {
    "docs": {
      "lvl0": {
        "selector": ".theme-doc-sidebar-item-category-level-1 > .menu__link--active",
        "global": true,
        "default_value": "Documentation"
      },
      "lvl1": "article h1",
      "lvl2": "article h2",
      "lvl3": "article h3",
      "text": "article p, article li, article td"
    },
    "blog": {
      "lvl0": {
        "selector": "",
        "default_value": "Blog"
      },
      "lvl1": "article h1",
      "lvl2": "article h2",
      "text": "article p, article li"
    }
  },
  "selectors_exclude": [
    ".theme-doc-footer",
    ".theme-doc-toc-mobile",
    ".theme-blog-post-paginator"
  ],
  "min_indexed_level": 1,
  "custom_settings": {
    "attributesForFaceting": ["language", "version", "tags"],
    "attributesToRetrieve": ["hierarchy", "content", "url"],
    "attributesToHighlight": ["hierarchy", "content"],
    "attributesToSnippet": ["content:15"],
    "camelCaseAttributes": ["hierarchy"],
    "searchableAttributes": [
      "unordered(hierarchy.lvl0)",
      "unordered(hierarchy.lvl1)",
      "unordered(hierarchy.lvl2)",
      "unordered(hierarchy.lvl3)",
      "content"
    ],
    "distinct": true,
    "attributeForDistinct": "url",
    "customRanking": ["desc(weight.pageRank)", "desc(weight.level)", "asc(weight.position)"],
    "ranking": ["words", "filters", "typo", "attribute", "proximity", "exact", "custom"],
    "highlightPreTag": "<mark>",
    "highlightPostTag": "</mark>",
    "minWordSizefor1Typo": 3,
    "minWordSizefor2Typos": 7
  }
}
```

**Run Crawler** (via GitHub Actions weekly):
```yaml
# .github/workflows/algolia-crawler.yml
name: Algolia Crawler

on:
  schedule:
    - cron: '0 2 * * 0'  # Weekly on Sunday at 2 AM UTC
  workflow_dispatch:

jobs:
  crawler:
    runs-on: ubuntu-latest
    steps:
      - name: Checkout
        uses: actions/checkout@v4

      - name: Install Algolia Crawler CLI
        run: npm install -g algolia-webcrawler

      - name: Run Crawler
        run: |
          algolia-webcrawler \
            --config algolia-config.json \
            --app-id ${{ secrets.ALGOLIA_APP_ID }} \
            --api-key ${{ secrets.ALGOLIA_ADMIN_KEY }}
```

---

## 9. Versioning Strategy

### 9.1 Creating a New Version

**When to version**:
- Major errata requiring significant changes (e.g., ROS 2 version upgrade)
- New edition with additional chapters
- Breaking changes to companion code

**Create version 1.0** (locks current docs):
```bash
cd website
npm run docusaurus docs:version 1.0
```

**Result**:
- Creates `versioned_docs/version-1.0/` (locked snapshot of current docs)
- Creates `versioned_sidebars/version-1.0-sidebars.json`
- Updates `versions.json`

**Outcome**:
- `https://robot-book.com/docs/` → Current version (1.1 or next)
- `https://robot-book.com/docs/1.0/` → Version 1.0 (locked)
- `https://robot-book.com/docs/next/` → Unreleased version (development)

### 9.2 Version Dropdown Configuration

**File**: `docusaurus.config.js`

```javascript
docs: {
  versions: {
    current: {
      label: '1.1 (Current)',
      path: '',
      banner: 'none',
    },
    '1.0': {
      label: '1.0',
      path: '1.0',
      banner: 'unmaintained',
    },
  },
}
```

---

## 10. Instructor Resources Access Control

### 10.1 Email-Gated Access

**File**: `website/src/pages/instructor-resources.md`

```mdx
---
title: Instructor Resources
description: Teaching materials for course instructors
---

import EmailGate from '@site/src/components/EmailGate';

# Instructor Resources

This page contains supplementary teaching materials for instructors adopting this textbook in academic courses.

<EmailGate
  requiredDomain="@edu"
  successMessage="Access granted! Check your email for download links."
  failureMessage="Access restricted to academic email addresses (.edu, .ac.uk, etc.)."
>

## Available Resources

- **Lecture Slides**: 21 PowerPoint slide decks (40-60 slides each)
- **Assignments**: 21 assignment prompts with solutions
- **Grading Rubrics**: 3 templates (labs, projects, capstone)
- **Quiz Questions**: 6 quiz sets (one per part)
- **Exam Question Bank**: 50+ questions with answer keys
- **Syllabi Templates**: 13-week semester, 10-week quarter

[Download All Resources (ZIP, 500 MB)](mailto:author@example.com?subject=Instructor%20Resources%20Request)

**Note**: To verify your instructor status, please email from your academic institution email address (.edu, .ac.uk, etc.).

</EmailGate>
```

### 10.2 Alternative: GitHub Release Assets (Private Repository)

**Strategy**:
1. Create private GitHub repository: `robot-book-instructor-resources`
2. Grant access to instructors upon email verification
3. Host materials as GitHub Release assets

---

## 11. Performance Optimization and Monitoring

### 11.1 Lighthouse Targets

**Core Web Vitals**:
- **First Contentful Paint (FCP)**: <1.5s
- **Largest Contentful Paint (LCP)**: <2.5s
- **Time to Interactive (TTI)**: <3.5s
- **Cumulative Layout Shift (CLS)**: <0.1
- **Speed Index**: <3.0s

**Lighthouse Score Targets**:
- Performance: ≥90
- Accessibility: ≥95 (WCAG 2.1 AA)
- Best Practices: ≥95
- SEO: ≥95

### 11.2 Bundle Analysis

**Analyze bundle size**:
```bash
cd website
npm run build -- --bundle-analyzer
```

**Expected Bundle Size**:
- JS (gzipped): ~200 KB
- CSS (gzipped): ~100 KB
- Total page weight (with images): ~1.5 MB (first load), ~300 KB (subsequent)

### 11.3 Image Optimization

**WebP Conversion**:
- Convert all PNG/JPG to WebP (10-30% reduction)
- Use responsive images (`srcset`) for mobile devices
- Lazy loading: `loading="lazy"` attribute

**Example**:
```mdx
<img
  src="/img/ch01/fig-1-1-physical-ai-stack.webp"
  alt="Physical AI stack diagram"
  loading="lazy"
  width="800"
  height="600"
/>
```

---

## 12. Maintenance and Update Procedures

### 12.1 Errata Updates

**Process**:
1. User reports error via GitHub Issues
2. Author verifies and creates fix in `book/chapters/`
3. Run migration script to update `website/docs/`
4. Create blog post announcing errata: `website/blog/YYYY-MM-DD-errata-chapterNN.md`
5. Commit changes and push (triggers CI/CD deployment)

**Errata Blog Post Template**:
```mdx
---
slug: errata-chapter-5
title: Errata - Chapter 5 Correction
authors: [author-name]
tags: [errata, chapter-5]
---

## Correction: Chapter 5, Section 5.3.2

**Issue**: Incorrect QoS policy example (reliability should be RELIABLE, not BEST_EFFORT)

**Fixed Code**:
```python
qos_profile = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,  # Corrected
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10
)
```

**Impact**: Low - Does not affect core functionality, but may cause confusion

**Version**: This errata is included in web version as of 2025-02-10 and will be reflected in the next print edition (v1.1).
```

### 12.2 Adding New Chapters (Future Editions)

**Process**:
1. Write new chapter in `book/chapters/partN/`
2. Run migration script to create `website/docs/partN-<name>/chNN.md`
3. Update `sidebars.js` to include new chapter
4. Update `book-architecture.md` with new chapter structure
5. Create version snapshot: `npm run docusaurus docs:version 1.0` (locks previous edition)
6. Deploy as version 1.1 (current)

---

## 13. Accessibility Compliance (WCAG 2.1 AA)

### 13.1 Checklist

- ✅ **Color Contrast**: 4.5:1 for normal text, 7:1 for code blocks
- ✅ **Keyboard Navigation**: All interactive elements accessible via Tab
- ✅ **Screen Reader Support**: Semantic HTML (`<nav>`, `<main>`, `<article>`)
- ✅ **Alt Text**: All images have descriptive alt text
- ✅ **ARIA Labels**: Navigation landmarks, expandable sections
- ✅ **Focus Indicators**: Visible focus outlines on interactive elements
- ✅ **Responsive Design**: Works on screen readers, mobile devices, tablets

### 13.2 Testing Tools

**Automated Testing**:
```bash
# Install axe-core for accessibility testing
npm install -g @axe-core/cli

# Test homepage
axe https://robot-book.com --exit

# Test chapter page
axe https://robot-book.com/docs/part1-foundations/ch01 --exit
```

**Manual Testing**:
- Screen reader: NVDA (Windows), VoiceOver (macOS)
- Keyboard navigation: Tab through all interactive elements
- Color blindness: Use Chrome DevTools (Rendering > Emulate vision deficiencies)

---

## 14. Analytics and Monitoring

### 14.1 Google Analytics (Optional)

**File**: `docusaurus.config.js`

```javascript
gtag: {
  trackingID: 'G-XXXXXXXXXX',
  anonymizeIP: true,
}
```

**Events to Track**:
- Page views per chapter
- Search queries (Algolia provides analytics)
- Outbound links (companion code repositories)
- Download events (PDF/EPUB, instructor resources)

### 14.2 Netlify Analytics (Built-in)

**Metrics**:
- Unique visitors
- Page views
- Top pages
- Traffic sources

**Access**: Netlify Dashboard > Site > Analytics

---

## 15. Content Update Automation

### 15.1 Sync Book Chapters to Website

**File**: `.github/workflows/sync-book-to-website.yml`

```yaml
name: Sync Book Chapters to Website

on:
  push:
    branches: [main]
    paths:
      - 'book/chapters/**'

jobs:
  sync:
    runs-on: ubuntu-latest
    steps:
      - name: Checkout
        uses: actions/checkout@v4

      - name: Run migration script
        run: python scripts/migrate-to-docusaurus.py

      - name: Commit changes
        run: |
          git config user.name "GitHub Actions"
          git config user.email "actions@github.com"
          git add website/docs/
          git commit -m "chore: sync book chapters to website [skip ci]" || true
          git push
```

---

## 16. SEO Optimization

### 16.1 Meta Tags (Auto-Generated)

**Frontmatter Example**:
```mdx
---
id: ch01
title: "Chapter 1: Introduction to Physical AI and Humanoid Robotics"
description: "Learn the fundamentals of Physical AI, explore the humanoid robotics landscape, and understand the technical challenges of bipedal locomotion and real-time decision-making."
keywords:
  - physical ai
  - humanoid robotics
  - embodied intelligence
  - ros 2
  - isaac sim
---
```

**Generated Meta Tags**:
```html
<title>Chapter 1: Introduction to Physical AI and Humanoid Robotics | Physical AI & Humanoid Robotics</title>
<meta name="description" content="Learn the fundamentals of Physical AI, explore the humanoid robotics landscape, and understand the technical challenges of bipedal locomotion and real-time decision-making.">
<meta name="keywords" content="physical ai, humanoid robotics, embodied intelligence, ros 2, isaac sim">
<meta property="og:title" content="Chapter 1: Introduction to Physical AI and Humanoid Robotics">
<meta property="og:description" content="Learn the fundamentals of Physical AI...">
<meta property="og:image" content="https://robot-book.com/img/social-card.png">
<meta property="og:url" content="https://robot-book.com/docs/part1-foundations/ch01">
<meta name="twitter:card" content="summary_large_image">
```

### 16.2 Sitemap and Robots.txt

**Auto-Generated**:
- Sitemap: `https://robot-book.com/sitemap.xml` (Docusaurus generates automatically)
- Robots.txt: `https://robot-book.com/robots.txt`

**Robots.txt Content**:
```
User-agent: *
Allow: /

Sitemap: https://robot-book.com/sitemap.xml
```

---

## 17. Validation Checklist

### 17.1 Pre-Launch Checklist

- [ ] All 21 chapters migrated to `website/docs/`
- [ ] All images converted to WebP and placed in `static/img/chNN/`
- [ ] Algolia search configured and indexed
- [ ] Lighthouse score ≥90 (Performance, Accessibility, SEO)
- [ ] Tested on mobile devices (iPhone, Android)
- [ ] Tested on tablets (iPad for lab environments)
- [ ] Keyboard navigation works (Tab through all elements)
- [ ] Screen reader compatibility (NVDA, VoiceOver)
- [ ] All internal links verified (no broken links)
- [ ] Code blocks have syntax highlighting
- [ ] Math equations render correctly (KaTeX)
- [ ] Custom domain configured (if applicable)
- [ ] SSL certificate active (HTTPS)
- [ ] Analytics configured (Google Analytics or Netlify Analytics)
- [ ] Instructor resources page functional
- [ ] Blog RSS feed working
- [ ] Deployment CI/CD tested (merge to main triggers deploy)
- [ ] Version dropdown functional (if versioning enabled)
- [ ] Footer links correct (GitHub, companion code, errata)

### 17.2 Post-Launch Monitoring

**Week 1**:
- Monitor Lighthouse scores daily
- Check Algolia search quality (test common queries)
- Review Google Analytics for 404 errors
- Test mobile performance on 3G/4G networks

**Month 1**:
- Gather user feedback (GitHub Discussions)
- Address reported issues (errata, broken links)
- Optimize slow-loading pages (bundle analysis)

---

## 18. Future Enhancements

### 18.1 Interactive Features (Phase 2)

**Code Playgrounds**:
- Integrate CodeSandbox API for browser-based Python execution
- Allow users to edit and run code examples directly in browser
- Use Pyodide (Python in WebAssembly) for client-side execution

**3D Visualizations**:
- Embed Three.js or Babylon.js for interactive 3D robot models
- Allow users to rotate URDF models in browser
- Visualize kinematics (forward/inverse kinematics) interactively

**Isaac Sim Demos**:
- Embed recorded Isaac Sim demos as videos
- Link to cloud-hosted Isaac Sim demos (NVIDIA Omniverse Cloud)

**Progress Tracking**:
- User accounts (optional) to track chapter completion
- Bookmark chapters
- Save code playground snippets

### 18.2 Internationalization (i18n)

**Supported Locales** (future):
- English (en) - Default
- Spanish (es)
- French (fr)
- Chinese (zh-CN)

**Configuration**:
```javascript
i18n: {
  defaultLocale: 'en',
  locales: ['en', 'es', 'fr', 'zh-CN'],
  localeConfigs: {
    en: { label: 'English' },
    es: { label: 'Español' },
    fr: { label: 'Français' },
    'zh-CN': { label: '简体中文' },
  },
}
```

---

## 19. Cost Breakdown

### 19.1 Free Tier (Open-Source)

**Hosting**:
- Netlify: Free (100 GB bandwidth/month)
- Vercel: Free (100 GB bandwidth/month)
- GitHub Pages: Free (1 GB storage, 100 GB bandwidth/month)

**Search**:
- Algolia DocSearch: Free for open-source (100M operations/month)

**Domain** (optional):
- Custom domain: ~$12/year (e.g., robot-book.com via Namecheap)

**Total**: $0/month (or $1/month with custom domain)

### 19.2 Paid Tier (High Traffic)

**Hosting** (if exceeding free tier):
- Netlify Pro: $19/month (400 GB bandwidth)
- Vercel Pro: $20/month (1 TB bandwidth)

**Search** (if exceeding free tier):
- Algolia: $1/month per 10K requests (beyond 100M/month)

**CDN** (optional):
- Cloudflare: Free (unlimited bandwidth)

**Total**: ~$0-$20/month (depending on traffic)

---

## 20. Success Metrics

### 20.1 Key Performance Indicators (KPIs)

**Technical**:
- Lighthouse Performance Score: ≥90
- Page Load Time (LCP): <2.5s
- Uptime: ≥99.9%

**User Engagement**:
- Average session duration: ≥5 minutes
- Pages per session: ≥3
- Bounce rate: <50%

**Search Quality**:
- Algolia click-through rate: ≥20%
- Average search queries per session: ≥1

**Adoption**:
- Unique visitors: 10,000+ in first 6 months
- GitHub stars: 500+ in first year
- Course adoptions: 10+ institutions in first year

---

## 21. Conclusion

This Docusaurus site structure provides a production-ready, accessible, and performant web version of the textbook. Key strengths:

1. **Performance**: KaTeX math rendering (10× faster than MathJax), lazy-loaded images, code splitting
2. **Accessibility**: WCAG 2.1 AA compliance, keyboard navigation, screen reader support
3. **Search**: Algolia DocSearch for instant, typo-tolerant search
4. **Versioning**: Built-in support for errata updates and future editions
5. **Deployment**: Zero-downtime deployments via Netlify/Vercel/GitHub Pages
6. **Cost**: $0/month for open-source projects (free tier hosting + Algolia DocSearch)

**Next Steps** (Phase 3):
1. Migrate all 21 chapters from `book/chapters/` to `website/docs/`
2. Optimize all images (convert to WebP)
3. Configure Algolia search (apply for DocSearch)
4. Deploy to Netlify and configure custom domain
5. Test accessibility (axe-core, screen readers)
6. Launch and monitor performance (Lighthouse, Google Analytics)
