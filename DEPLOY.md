# GitHub Pages Deployment Guide

This repository is configured for simple GitHub Pages deployment from the `/docs` folder on the master branch.

## Quick Start

### 1. Build the Site

Run the build script:

```bash
./build-for-github-pages.sh
```

This will:
- Clear the Docusaurus cache
- Build the site and output to `/docs` folder
- Take 5-10 minutes depending on your system

### 2. Commit and Push

After the build completes successfully:

```bash
git add .
git commit -m "Build site for GitHub Pages deployment"
git push origin master
```

### 3. Configure GitHub Pages

1. Go to your repository on GitHub: https://github.com/samiceto/robot_book
2. Click **Settings** → **Pages** (in the left sidebar)
3. Under "Build and deployment":
   - **Source**: Select "Deploy from a branch"
   - **Branch**: Select `master` and `/docs` folder
   - Click **Save**

### 4. Access Your Site

Your site will be live at: **https://samiceto.github.io/robot_book/**

It may take a few minutes for the first deployment.

## Project Structure

```
robot_book/
├── website/              # Docusaurus source files
│   ├── docs/            # Markdown documentation files
│   ├── src/             # Custom React components and CSS
│   ├── static/          # Static assets (images, etc.)
│   ├── docusaurus.config.js
│   ├── sidebars.js
│   └── package.json
├── code/                # Code examples referenced in docs
├── docs/                # Built site (created after running build script)
├── README.md
├── LICENSE
└── build-for-github-pages.sh

```

## Development

To preview the site locally while editing:

```bash
cd website
npm start
```

This opens a local development server at http://localhost:3000 with hot-reload.

## Updating Content

1. Edit markdown files in `website/docs/`
2. Run the build script: `./build-for-github-pages.sh`
3. Commit and push the changes (including the updated `/docs` folder)
4. GitHub Pages will automatically serve the new content

## Troubleshooting

### Build fails with MDX errors

The docs use LaTeX math expressions. If you add new math, escape curly braces in inline math:
- Wrong: `$I_{xx}$`
- Correct: `$I_\{xx\}$`

### Site shows 404

- Ensure you committed and pushed the `/docs` folder
- Check GitHub Pages settings are correct (master branch, /docs folder)
- Wait 2-3 minutes after pushing for GitHub to rebuild

### Changes not showing

- Clear your browser cache
- Check the commit included the updated `/docs` folder
- Verify the build completed successfully without errors

## Clean Build

If you encounter issues, try a clean build:

```bash
cd website
rm -rf node_modules/.cache .docusaurus
cd ..
./build-for-github-pages.sh
```
