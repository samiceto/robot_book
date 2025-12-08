#!/bin/bash
# Build script for GitHub Pages deployment
# This script builds the Docusaurus site and outputs to /docs folder

echo "Building Docusaurus site for GitHub Pages..."
echo "This may take 5-10 minutes depending on your system..."

cd website

# Clear cache for fresh build
echo "Clearing cache..."
rm -rf .docusaurus node_modules/.cache

# Build the site (outputs to ../docs)
echo "Building site..."
npm run build

if [ $? -eq 0 ]; then
    echo ""
    echo "✅ Build successful!"
    echo ""
    echo "Next steps:"
    echo "1. Add .nojekyll file to docs folder"
    echo "2. Commit and push all changes:"
    echo "   git add ."
    echo "   git commit -m 'Build site for GitHub Pages'"
    echo "   git push origin master"
    echo ""
    echo "3. On GitHub, go to Settings > Pages"
    echo "4. Select 'Deploy from a branch'"
    echo "5. Choose 'master' branch and '/docs' folder"
    echo "6. Click Save"
    echo ""
    echo "Your site will be live at: https://samiceto.github.io/robot_book/"
else
    echo ""
    echo "❌ Build failed. Please check the errors above."
    exit 1
fi
