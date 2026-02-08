#!/bin/bash
# Monitor diverse data collection pipeline

cd "$(dirname "$0")"

echo "📊 Monitoring Diverse Data Collection Pipeline"
echo "=============================================="
echo ""

# Check if pipeline is running
if pgrep -f "collect_diverse_pipeline.sh" > /dev/null || pgrep -f "collect_diverse_data.py" > /dev/null; then
    echo "✅ Pipeline is running"
    echo ""
    
    # Show latest log
    if [ -f diverse_collection.log ]; then
        echo "📝 Latest log entries:"
        tail -20 diverse_collection.log | grep -E "(Progress|Collected|Switching|Steering|✅|❌)" | tail -10
    fi
    
    # Check data directory
    LATEST_DIR=$(ls -td data/diverse_* 2>/dev/null | head -1)
    if [ -n "$LATEST_DIR" ]; then
        if [ -f "$LATEST_DIR/data.csv" ]; then
            FRAMES=$(wc -l < "$LATEST_DIR/data.csv" | tr -d ' ')
            FRAMES=$((FRAMES - 1))  # Subtract header
            echo ""
            echo "📁 Current data directory: $LATEST_DIR"
            echo "   Frames collected: $FRAMES"
        fi
    fi
else
    echo "❌ Pipeline is not running"
    echo ""
    echo "To start: ./collect_diverse_pipeline.sh"
fi

echo ""
echo "To view full log: tail -f diverse_collection.log"
