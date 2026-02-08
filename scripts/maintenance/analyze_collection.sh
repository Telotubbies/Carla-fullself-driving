#!/bin/bash
# Analyze data collection log

LOG_FILE="logs/data_collection.log"

if [ ! -f "$LOG_FILE" ]; then
    echo "❌ Log file not found: $LOG_FILE"
    exit 1
fi

echo "📊 Data Collection Analysis"
echo "=========================="
echo ""

# Total frames collected
TOTAL=$(grep "Collected.*frames" "$LOG_FILE" | tail -1 | grep -oP '\d+(?=/20000)')
if [ -z "$TOTAL" ]; then
    TOTAL=0
fi

echo "📈 Progress:"
echo "   Collected: $TOTAL / 20000 frames ($(echo "scale=1; $TOTAL*100/20000" | bc)%)"

# FPS
FPS=$(grep "Collected.*frames" "$LOG_FILE" | tail -1 | grep -oP '\d+\.\d+(?= fps)' | tail -1)
if [ -n "$FPS" ]; then
    echo "   FPS: $FPS"
    REMAINING=$(grep "Collected.*frames" "$LOG_FILE" | tail -1 | grep -oP '\d+(?=s remaining)')
    if [ -n "$REMAINING" ]; then
        MIN=$((REMAINING / 60))
        SEC=$((REMAINING % 60))
        echo "   Estimated time remaining: ${MIN}m ${SEC}s"
    fi
fi

# Stuck incidents
STUCK_COUNT=$(grep -c "Vehicle appears stuck" "$LOG_FILE")
if [ "$STUCK_COUNT" -gt 0 ]; then
    echo ""
    echo "⚠️  Issues:"
    echo "   Stuck incidents: $STUCK_COUNT"
    echo "   Stuck at steps:"
    grep "Vehicle appears stuck" "$LOG_FILE" | grep -oP 'step \K\d+' | while read step; do
        echo "     - Step $step"
    done
fi

# Status
if pgrep -f "collect_autopilot_data.py" > /dev/null; then
    echo ""
    echo "✅ Status: Running"
else
    echo ""
    echo "❌ Status: Stopped"
    if grep -q "Data collection complete" "$LOG_FILE"; then
        echo "   ✅ Collection completed successfully"
    fi
fi

echo ""
