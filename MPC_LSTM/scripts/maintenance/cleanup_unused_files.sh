#!/bin/bash
# 🧹 Cleanup Unused and Unnecessary Files
# 
# This script removes unused files, old logs, and unnecessary files

set -e

# Colors
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
BLUE='\033[0;34m'
NC='\033[0m'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$PROJECT_ROOT"

echo -e "${BLUE}🧹 Cleaning up unused and unnecessary files...${NC}"
echo ""

# 1. Remove backup files
echo -e "${YELLOW}1. Removing backup files...${NC}"
find . -maxdepth 3 -type f \( -name "*.bak" -o -name "*.old" -o -name "*~" -o -name "*.swp" -o -name "*.swo" -o -name "*.tmp" \) -delete 2>/dev/null || true
echo -e "${GREEN}✅ Removed backup files${NC}"

# 2. Remove Python cache
echo ""
echo -e "${YELLOW}2. Removing Python cache...${NC}"
find . -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null || true
find . -type f -name "*.pyc" -delete 2>/dev/null || true
find . -type f -name "*.pyo" -delete 2>/dev/null || true
echo -e "${GREEN}✅ Removed Python cache${NC}"

# 3. Clean old logs (keep last 7 days)
echo ""
echo -e "${YELLOW}3. Cleaning old logs (keeping last 7 days)...${NC}"
find logs -name "*.log" -mtime +7 -type f -delete 2>/dev/null || true
find logs -name "*.csv" -mtime +7 -type f -delete 2>/dev/null || true
echo -e "${GREEN}✅ Cleaned old logs${NC}"

# 4. Remove duplicate/old scripts from root (keep only essential)
echo ""
echo -e "${YELLOW}4. Checking for duplicate scripts...${NC}"
# Keep only essential entry points
ESSENTIAL_SCRIPTS=("train.sh" "auto_flow.sh" "run_pipeline.sh" "run_inference.sh")
for script in *.sh; do
    if [ -f "$script" ] && [[ ! " ${ESSENTIAL_SCRIPTS[@]} " =~ " ${script} " ]]; then
        if [ ! -L "$script" ]; then
            echo -e "${YELLOW}   Found: $script (check if needed)${NC}"
        fi
    fi
done

# 5. Clean old data directories (keep only latest 3)
echo ""
echo -e "${YELLOW}5. Cleaning old data directories (keeping latest 3)...${NC}"
DATA_DIRS=$(ls -td data/autopilot_* 2>/dev/null | tail -n +4)
if [ -n "$DATA_DIRS" ]; then
    for dir in $DATA_DIRS; do
        echo -e "${YELLOW}   Old data: $dir${NC}"
        # Ask or auto-remove? For now, just list
    done
    echo -e "${GREEN}✅ Listed old data directories${NC}"
else
    echo -e "${GREEN}✅ No old data directories to clean${NC}"
fi

# 6. Remove empty directories
echo ""
echo -e "${YELLOW}6. Removing empty directories...${NC}"
find . -type d -empty -delete 2>/dev/null || true
echo -e "${GREEN}✅ Removed empty directories${NC}"

# 7. Clean .DS_Store and Thumbs.db
echo ""
echo -e "${YELLOW}7. Removing system files...${NC}"
find . -name ".DS_Store" -delete 2>/dev/null || true
find . -name "Thumbs.db" -delete 2>/dev/null || true
echo -e "${GREEN}✅ Removed system files${NC}"

# Summary
echo ""
echo -e "${GREEN}✅ Cleanup complete!${NC}"
echo ""
echo "📊 Summary:"
echo "   ✅ Backup files removed"
echo "   ✅ Python cache removed"
echo "   ✅ Old logs cleaned (7+ days)"
echo "   ✅ Empty directories removed"
echo "   ✅ System files removed"
echo ""
echo "💡 Note: Old data directories listed but not removed"
echo "   Remove manually if needed: rm -rf data/autopilot_YYYYMMDD_HHMMSS"

