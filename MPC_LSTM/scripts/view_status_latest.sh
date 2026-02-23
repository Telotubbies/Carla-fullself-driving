#!/bin/bash
# View latest status log file

cd "$(dirname "$0")/.."

LATEST_LOG=$(ls -t logs/status_*.log 2>/dev/null | head -1)

if [ -n "$LATEST_LOG" ]; then
    echo "📊 Latest Status Log: $LATEST_LOG"
    echo ""
    cat "$LATEST_LOG"
else
    echo "⚠️  No status log found. Generating new one..."
    python3 scripts/view_status.py
fi

