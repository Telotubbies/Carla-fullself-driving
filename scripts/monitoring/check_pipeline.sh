#!/bin/bash
# Quick check for pipeline status

cd "$(dirname "$0")"

echo "📊 Diverse Data Collection Pipeline Status"
echo "=========================================="
echo ""

# Check CARLA
if pgrep -f "CarlaUE4" > /dev/null; then
    echo "✅ CARLA: Running"
else
    echo "❌ CARLA: Not running"
fi

# Check pipeline
if pgrep -f "collect_diverse_data.py" > /dev/null; then
    echo "✅ Pipeline: Running"
    PID=$(pgrep -f "collect_diverse_data.py" | head -1)
    echo "   PID: $PID"
else
    echo "❌ Pipeline: Not running"
fi

# Check latest data
LATEST_DIR=$(ls -td data/diverse_* 2>/dev/null | head -1)
if [ -n "$LATEST_DIR" ] && [ -f "$LATEST_DIR/data.csv" ]; then
    FRAMES=$(wc -l < "$LATEST_DIR/data.csv" | tr -d ' ')
    FRAMES=$((FRAMES - 1))
    echo ""
    echo "📁 Latest data: $LATEST_DIR"
    echo "   Frames collected: $FRAMES"
fi

# Show latest progress
if [ -f diverse_collection_final.log ]; then
    echo ""
    echo "📝 Latest progress:"
    tail -5 diverse_collection_final.log | grep -E "(Progress|Collected|Switching|Steering)" | tail -3
fi

echo ""
echo "Full log: tail -f diverse_collection_final.log"
