#!/bin/bash
# Organize project files into proper structure

set -e

cd "$(dirname "$0")"

echo "📁 Organizing project files..."
echo "=============================="
echo ""

# Create directories
mkdir -p scripts/{setup,monitoring,data_collection}
mkdir -p docs
mkdir -p logs/old
mkdir -p data/temp

echo "✅ Created directory structure"
echo ""

# Move scripts to scripts/
echo "📦 Moving scripts..."

# Setup scripts
if [ -f "setup.sh" ]; then
    mv setup.sh scripts/setup/
    echo "   → setup.sh → scripts/setup/"
fi

if [ -f "install_rocm.sh" ]; then
    mv install_rocm.sh scripts/setup/
    echo "   → install_rocm.sh → scripts/setup/"
fi

# Monitoring scripts
if [ -f "check_all.sh" ]; then
    mv check_all.sh scripts/monitoring/
    echo "   → check_all.sh → scripts/monitoring/"
fi

if [ -f "check_pipeline.sh" ]; then
    mv check_pipeline.sh scripts/monitoring/
    echo "   → check_pipeline.sh → scripts/monitoring/"
fi

if [ -f "monitor_pipeline.sh" ]; then
    mv monitor_pipeline.sh scripts/monitoring/
    echo "   → monitor_pipeline.sh → scripts/monitoring/"
fi

if [ -f "check_resnet_and_data.py" ]; then
    mv check_resnet_and_data.py scripts/monitoring/
    echo "   → check_resnet_and_data.py → scripts/monitoring/"
fi

# Data collection scripts
if [ -f "collect_diverse_pipeline.sh" ]; then
    mv collect_diverse_pipeline.sh scripts/data_collection/
    echo "   → collect_diverse_pipeline.sh → scripts/data_collection/"
fi

if [ -f "start_all.sh" ]; then
    mv start_all.sh scripts/data_collection/
    echo "   → start_all.sh → scripts/data_collection/"
fi

if [ -f "run_all.sh" ]; then
    mv run_all.sh scripts/data_collection/
    echo "   → run_all.sh → scripts/data_collection/"
fi

if [ -f "run_full_pipeline.sh" ]; then
    mv run_full_pipeline.sh scripts/data_collection/
    echo "   → run_full_pipeline.sh → scripts/data_collection/"
fi

if [ -f "run_complete_auto.sh" ]; then
    mv run_complete_auto.sh scripts/data_collection/
    echo "   → run_complete_auto.sh → scripts/data_collection/"
fi

if [ -f "start_carla_and_run.sh" ]; then
    mv start_carla_and_run.sh scripts/data_collection/
    echo "   → start_carla_and_run.sh → scripts/data_collection/"
fi

if [ -f "quick_start_inference.sh" ]; then
    mv quick_start_inference.sh scripts/data_collection/
    echo "   → quick_start_inference.sh → scripts/data_collection/"
fi

# Move documentation
echo ""
echo "📚 Moving documentation..."

if [ -f "DATA_COLLECTION_GUIDE.md" ]; then
    mv DATA_COLLECTION_GUIDE.md docs/
    echo "   → DATA_COLLECTION_GUIDE.md → docs/"
fi

if [ -f "QUICKSTART_TH.md" ]; then
    mv QUICKSTART_TH.md docs/
    echo "   → QUICKSTART_TH.md → docs/"
fi

if [ -f "validation_summary.txt" ]; then
    mv validation_summary.txt docs/
    echo "   → validation_summary.txt → docs/"
fi

# Move old logs
echo ""
echo "📋 Moving old logs..."

for log in *.log; do
    if [ -f "$log" ]; then
        mv "$log" logs/old/ 2>/dev/null || true
        echo "   → $log → logs/old/"
    fi
done

# Create symlinks for commonly used scripts
echo ""
echo "🔗 Creating symlinks..."

if [ -f "scripts/monitoring/check_all.sh" ]; then
    ln -sf scripts/monitoring/check_all.sh check_all.sh
    echo "   ✓ check_all.sh (symlink)"
fi

if [ -f "scripts/data_collection/start_all.sh" ]; then
    ln -sf scripts/data_collection/start_all.sh start_all.sh
    echo "   ✓ start_all.sh (symlink)"
fi

echo ""
echo "✅ File organization complete!"
echo ""
echo "📁 New structure:"
echo "   scripts/"
echo "     ├── setup/          (setup & installation)"
echo "     ├── monitoring/     (check & monitor scripts)"
echo "     └── data_collection/ (data collection scripts)"
echo "   docs/                 (documentation)"
echo "   logs/old/            (old log files)"
echo ""
