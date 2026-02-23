#!/bin/bash
# Monitor training progress

GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
NC='\033[0m'

cd "$(dirname "$0")"

echo -e "${BLUE}Monitoring Training Progress...${NC}"
echo ""

while true; do
    if ! pgrep -f "train_lstm.py" > /dev/null; then
        echo -e "${YELLOW}Training process not found. Checking if completed...${NC}"
        if [ -f "data/autopilot_20260208_130934/lstm_model/best_model.pth" ]; then
            echo -e "${GREEN}✅ Training completed! Model found.${NC}"
            break
        else
            echo -e "${YELLOW}Training may have stopped. Check training_log.txt${NC}"
            break
        fi
    fi
    
    echo -e "${BLUE}=== Training Status ===${NC}"
    tail -5 training_log.txt 2>/dev/null | grep -E "(Epoch|Loss|Saved)" || tail -3 training_log.txt
    echo ""
    
    sleep 10
done

