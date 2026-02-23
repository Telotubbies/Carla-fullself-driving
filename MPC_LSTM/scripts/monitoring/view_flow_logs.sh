#!/bin/bash
# View flow logs

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
FLOW_LOG_DIR="$PROJECT_ROOT/logs/flow"

echo "📋 Flow Logs"
echo "=============="
echo ""

if [ ! -d "$FLOW_LOG_DIR" ]; then
    echo "No flow logs found. Run auto_flow.sh first."
    exit 1
fi

# Find latest flow run
LATEST_RUN=$(ls -td "$FLOW_LOG_DIR"/steps_* 2>/dev/null | head -1)

if [ -z "$LATEST_RUN" ]; then
    echo "No flow runs found."
    exit 1
fi

TIMESTAMP=$(basename "$LATEST_RUN" | sed 's/steps_//')
MAIN_LOG="$FLOW_LOG_DIR/auto_flow_${TIMESTAMP}.log"
SUMMARY_LOG="$FLOW_LOG_DIR/summary_${TIMESTAMP}.log"

echo "Latest Flow Run: $TIMESTAMP"
echo ""

# Show summary if exists
if [ -f "$SUMMARY_LOG" ]; then
    echo "📊 Summary:"
    cat "$SUMMARY_LOG"
    echo ""
fi

# List step logs
echo "📁 Step Logs:"
for step_log in "$LATEST_RUN"/*.log; do
    if [ -f "$step_log" ]; then
        step_name=$(basename "$step_log" .log)
        step_status=$(grep "Status:" "$step_log" 2>/dev/null | tail -1 | awk '{print $2}' || echo "UNKNOWN")
        step_size=$(du -h "$step_log" | cut -f1)
        echo "   $step_name: $step_status ($step_size)"
    fi
done

echo ""
echo "💡 View logs:"
echo "   Main log:    cat $MAIN_LOG"
echo "   Summary:     cat $SUMMARY_LOG"
echo "   Step log:    cat $LATEST_RUN/<step_name>.log"
echo ""

# Interactive menu
if [ "$1" == "--interactive" ] || [ "$1" == "-i" ]; then
    echo "Select log to view:"
    echo "1) Main log"
    echo "2) Summary"
    echo "3) Step logs"
    echo "4) All logs"
    read -p "Choice [1-4]: " choice
    
    case $choice in
        1)
            less "$MAIN_LOG"
            ;;
        2)
            cat "$SUMMARY_LOG"
            ;;
        3)
            echo "Available step logs:"
            ls -1 "$LATEST_RUN"/*.log | nl
            read -p "Select step number: " step_num
            step_file=$(ls -1 "$LATEST_RUN"/*.log | sed -n "${step_num}p")
            if [ -n "$step_file" ]; then
                less "$step_file"
            fi
            ;;
        4)
            less "$MAIN_LOG"
            ;;
        *)
            echo "Invalid choice"
            ;;
    esac
fi

