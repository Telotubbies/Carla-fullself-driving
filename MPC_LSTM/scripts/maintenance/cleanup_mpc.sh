#!/bin/bash
# 🧹 Cleanup Old MPC Files
# 
# This script removes old/unused files from the MPC control module
# and organizes the control directory structure.

set -e

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
CONTROL_DIR="$PROJECT_ROOT/control"

cd "$CONTROL_DIR"

echo -e "${YELLOW}🧹 Cleaning up MPC control directory...${NC}"
echo ""

# 1. Remove Python cache files
echo "1. Removing Python cache files..."
find . -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
find . -name "*.pyc" -delete 2>/dev/null || true
find . -name "*.pyo" -delete 2>/dev/null || true
echo -e "${GREEN}✅ Removed cache files${NC}"

# 2. Remove backup files
echo ""
echo "2. Removing backup files..."
find . -name "*.bak" -delete 2>/dev/null || true
find . -name "*.old" -delete 2>/dev/null || true
find . -name "*~" -delete 2>/dev/null || true
find . -name "*.swp" -delete 2>/dev/null || true
find . -name "*.swo" -delete 2>/dev/null || true
echo -e "${GREEN}✅ Removed backup files${NC}"

# 3. Check for duplicate/unused Python files
echo ""
echo "3. Checking for unused files..."
ACTIVE_FILES=("mpc_controller.py" "lane_path_planner.py" "__init__.py")
ALL_PY_FILES=$(find . -maxdepth 1 -name "*.py" -type f | xargs -n1 basename)

for file in $ALL_PY_FILES; do
    if [[ ! " ${ACTIVE_FILES[@]} " =~ " ${file} " ]]; then
        echo -e "${YELLOW}⚠️  Found potentially unused file: $file${NC}"
        echo "   (Keeping for now - review manually)"
    fi
done

# 4. Clean unused directory (keep README only)
echo ""
echo "4. Cleaning unused directory..."
if [ -d "unused" ]; then
    # Remove all files except README.md
    find unused/ -type f ! -name "README.md" -delete 2>/dev/null || true
    echo -e "${GREEN}✅ Cleaned unused directory${NC}"
fi

# 5. Summary
echo ""
echo -e "${GREEN}✅ Cleanup complete!${NC}"
echo ""
echo "📁 Active MPC files:"
ls -1 *.py 2>/dev/null | while read file; do
    echo "   ✅ $file"
done

echo ""
echo "📊 Directory structure:"
tree -L 2 -I '__pycache__' . 2>/dev/null || find . -maxdepth 2 -type f -name "*.py" | head -10

