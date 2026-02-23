#!/bin/bash
# Production-level project organization script
# This script organizes the entire project into a production-ready structure

set -e

PROJECT_ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$PROJECT_ROOT"

echo "🏗️  Production-Level Project Organization"
echo "=========================================="
echo ""

# Color codes
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

# 1. Organize Documentation
echo -e "${BLUE}📚 Step 1: Organizing Documentation...${NC}"
mkdir -p docs/{guides,api,architecture,deployment}

# Move duplicate README files to docs/guides
for doc in QUICK_START.md QUICKSTART.md MPC_QUICK_START.md PIPELINE_STEPS.md; do
    if [ -f "$doc" ]; then
        mv "$doc" docs/guides/
        echo -e "   ${GREEN}✅${NC} Moved $doc → docs/guides/"
    fi
done

# Consolidate status docs
mkdir -p docs/status
for doc in docs/*STATUS*.md; do
    if [ -f "$doc" ]; then
        mv "$doc" docs/status/ 2>/dev/null || true
    fi
done
for doc in docs/CURRENT_STATUS.md docs/FINAL_STATUS.md; do
    if [ -f "$doc" ]; then
        mv "$doc" docs/status/ 2>/dev/null || true
    fi
done

echo -e "   ${GREEN}✅${NC} Documentation organized"
echo ""

# 2. Organize Scripts
echo -e "${BLUE}📦 Step 2: Organizing Scripts...${NC}"

# Move root-level scripts to appropriate directories
mkdir -p scripts/{entry_points,maintenance}

# Entry point scripts (main entry points)
for script in run_complete_pipeline.sh run_full_auto_pipeline.sh run_mpc_inference.sh; do
    if [ -f "$script" ]; then
        mv "$script" scripts/entry_points/
        echo -e "   ${GREEN}✅${NC} Moved $script → scripts/entry_points/"
    fi
done

# Maintenance scripts
for script in organize_files.sh organize_remaining.sh analyze_collection.sh; do
    if [ -f "$script" ]; then
        mv "$script" scripts/maintenance/
        echo -e "   ${GREEN}✅${NC} Moved $script → scripts/maintenance/"
    fi
done

# Remove duplicate/old scripts from root
for script in start.sh check_all.sh; do
    if [ -f "$script" ] && [ ! -L "$script" ]; then
        # Check if it's a symlink
        if [ ! -L "$script" ]; then
            rm -f "$script"
            echo -e "   ${YELLOW}⚠️${NC}  Removed duplicate $script (use scripts/ version)"
        fi
    fi
done

echo -e "   ${GREEN}✅${NC} Scripts organized"
echo ""

# 3. Create Production Structure
echo -e "${BLUE}🏗️  Step 3: Creating Production Structure...${NC}"

# Create configs directory
mkdir -p configs/{production,development,testing}
if [ -f config.yaml ]; then
    cp config.yaml configs/development/config.yaml
    echo -e "   ${GREEN}✅${NC} Created configs/ directory"
fi

# Create proper test structure
mkdir -p tests/{unit,integration,e2e}
if [ -f tests/test_components.py ]; then
    mv tests/test_components.py tests/integration/
    echo -e "   ${GREEN}✅${NC} Organized tests/"
fi

# Create logs structure
mkdir -p logs/{training,inference,errors,archived}
echo -e "   ${GREEN}✅${NC} Created logs/ structure"

# Create deployment directory
mkdir -p deployment/{docker,kubernetes,scripts}
echo -e "   ${GREEN}✅${NC} Created deployment/ directory"

echo ""

# 4. Create .gitignore if not exists
echo -e "${BLUE}📝 Step 4: Setting up .gitignore...${NC}"
if [ ! -f .gitignore ]; then
    cat > .gitignore << 'EOF'
# Python
__pycache__/
*.py[cod]
*$py.class
*.so
.Python
env/
venv/
ENV/
build/
dist/
*.egg-info/

# CARLA
*.log
*.csv
data/autopilot_*/
data/diverse_*/
logs/*.log
logs/*.csv
logs/old/
logs/archived/

# Models
*.pth
*.pt
*.h5
*.ckpt
checkpoints/

# IDE
.vscode/
.idea/
*.swp
*.swo
*~

# OS
.DS_Store
Thumbs.db

# Temporary
*.tmp
temp/
tmp/
EOF
    echo -e "   ${GREEN}✅${NC} Created .gitignore"
else
    echo -e "   ${YELLOW}ℹ️${NC}  .gitignore already exists"
fi
echo ""

# 5. Create Production README
echo -e "${BLUE}📖 Step 5: Creating Production README...${NC}"
# This will be done by the Python script
echo -e "   ${GREEN}✅${NC} README will be updated"
echo ""

# 6. Organize logs
echo -e "${BLUE}📋 Step 6: Organizing Logs...${NC}"
# Move old logs to archived
if [ -d logs/old ]; then
    mv logs/old/* logs/archived/ 2>/dev/null || true
    echo -e "   ${GREEN}✅${NC} Moved old logs to archived/"
fi

# Organize logs by type
for log in logs/*.log; do
    if [ -f "$log" ]; then
        if [[ "$log" == *"training"* ]] || [[ "$log" == *"train"* ]] || [[ "$log" == *"finetune"* ]]; then
            mv "$log" logs/training/ 2>/dev/null || true
        elif [[ "$log" == *"inference"* ]] || [[ "$log" == *"main"* ]]; then
            mv "$log" logs/inference/ 2>/dev/null || true
        fi
    fi
done

echo -e "   ${GREEN}✅${NC} Logs organized"
echo ""

# 7. Create entry point symlinks
echo -e "${BLUE}🔗 Step 7: Creating Entry Point Symlinks...${NC}"
if [ -f scripts/entry_points/run_complete_pipeline.sh ]; then
    ln -sf scripts/entry_points/run_complete_pipeline.sh run_pipeline.sh
    echo -e "   ${GREEN}✅${NC} Created run_pipeline.sh → scripts/entry_points/run_complete_pipeline.sh"
fi

if [ -f scripts/entry_points/run_mpc_inference.sh ]; then
    ln -sf scripts/entry_points/run_mpc_inference.sh run_inference.sh
    echo -e "   ${GREEN}✅${NC} Created run_inference.sh → scripts/entry_points/run_mpc_inference.sh"
fi

echo ""

# Summary
echo -e "${GREEN}✅ Organization Complete!${NC}"
echo ""
echo "📁 New Structure:"
echo "   📚 docs/{guides,api,architecture,deployment,status}"
echo "   📦 scripts/{entry_points,maintenance,setup,monitoring,data_collection,training}"
echo "   ⚙️  configs/{production,development,testing}"
echo "   🧪 tests/{unit,integration,e2e}"
echo "   📋 logs/{training,inference,errors,archived}"
echo "   🚀 deployment/{docker,kubernetes,scripts}"
echo ""
echo "💡 Entry Points:"
echo "   ./run_pipeline.sh      - Run complete pipeline"
echo "   ./run_inference.sh      - Run inference"
echo ""

