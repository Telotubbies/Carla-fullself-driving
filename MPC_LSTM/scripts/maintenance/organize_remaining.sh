#!/bin/bash
# Organize remaining files

cd "$(dirname "$0")"

echo "📁 Organizing remaining files..."
echo ""

# Move remaining scripts
echo "📦 Moving remaining scripts..."

# Auto complete scripts
for script in auto_complete_*.sh; do
    if [ -f "$script" ]; then
        mv "$script" scripts/data_collection/
        echo "   → $script → scripts/data_collection/"
    fi
done

# Fix scripts
for script in fix_*.sh; do
    if [ -f "$script" ]; then
        mv "$script" scripts/setup/
        echo "   → $script → scripts/setup/"
    fi
done

# Check scripts
for script in check_*.sh; do
    if [ -f "$script" ]; then
        mv "$script" scripts/monitoring/
        echo "   → $script → scripts/monitoring/"
    fi
done

# Monitor scripts
for script in monitor_*.sh; do
    if [ -f "$script" ]; then
        mv "$script" scripts/monitoring/
        echo "   → $script → scripts/monitoring/"
    fi
done

# Run scripts
for script in run_*.sh run.sh start.sh; do
    if [ -f "$script" ]; then
        mv "$script" scripts/data_collection/
        echo "   → $script → scripts/data_collection/"
    fi
done

# Restart scripts
for script in restart_*.sh; do
    if [ -f "$script" ]; then
        mv "$script" scripts/data_collection/
        echo "   → $script → scripts/data_collection/"
    fi
done

# Move documentation
echo ""
echo "📚 Moving documentation..."

# README files
for doc in README_*.md; do
    if [ -f "$doc" ]; then
        mv "$doc" docs/
        echo "   → $doc → docs/"
    fi
done

# Status files
for doc in *STATUS*.md CURRENT_STATUS.md FINAL_STATUS.md; do
    if [ -f "$doc" ]; then
        mv "$doc" docs/
        echo "   → $doc → docs/"
    fi
done

# Guide files
for doc in *GUIDE*.md TRAINING_GUIDE.md VALIDATION_SUMMARY.md IMPLEMENTATION_SUMMARY.md; do
    if [ -f "$doc" ]; then
        mv "$doc" docs/
        echo "   → $doc → docs/"
    fi
done

# Move remaining logs
echo ""
echo "📋 Moving remaining logs..."

for log in *.log *.txt inference_*.txt training_*.txt pipeline_*.txt; do
    if [ -f "$log" ] && [ ! -L "$log" ]; then
        mv "$log" logs/old/ 2>/dev/null || true
        echo "   → $log → logs/old/"
    fi
done

# Move test files
echo ""
echo "🧪 Moving test files..."

if [ -f "test_components.py" ]; then
    mkdir -p tests
    mv test_components.py tests/
    echo "   → test_components.py → tests/"
fi

echo ""
echo "✅ Remaining files organized!"
