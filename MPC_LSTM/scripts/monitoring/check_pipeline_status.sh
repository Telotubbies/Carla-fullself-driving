#!/bin/bash
# Check pipeline progress

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$PROJECT_DIR"

echo "📊 Pipeline Status"
echo "=================="
echo ""

# Check if pipeline is running
if pgrep -f "run_full_pipeline.sh\|collect_autopilot_data\|train_lstm\|extract_features" > /dev/null; then
    echo "✅ Pipeline is running"
    echo ""
else
    echo "⚠️  Pipeline not running"
    echo ""
fi

# Check data collection
DATA_DIR=$(ls -td data/autopilot_* 2>/dev/null | head -1)
if [ -n "$DATA_DIR" ]; then
    echo "📁 Latest data directory: $DATA_DIR"
    
    if [ -f "$DATA_DIR/data.csv" ]; then
        ROWS=$(wc -l < "$DATA_DIR/data.csv")
        echo "   Data rows: $((ROWS-1))"
    fi
    
    if [ -d "$DATA_DIR/processed" ]; then
        echo "   ✅ Preprocessed"
    else
        echo "   ⏳ Not preprocessed yet"
    fi
    
    if [ -f "$DATA_DIR/features.npy" ]; then
        echo "   ✅ Features extracted"
    else
        echo "   ⏳ Features not extracted yet"
    fi
    
    if [ -f "$DATA_DIR/lstm_model/best_model.pth" ]; then
        echo "   ✅ LSTM trained"
        echo "   Model: $DATA_DIR/lstm_model/best_model.pth"
    else
        echo "   ⏳ LSTM not trained yet"
    fi
else
    echo "📁 No data directory found"
fi

echo ""

# Check logs
if [ -f "pipeline_log.txt" ]; then
    echo "📝 Latest log entries:"
    tail -10 pipeline_log.txt
fi

