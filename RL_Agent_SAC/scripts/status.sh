#!/bin/bash
# Quick Status Check Script

BASE_DIR="/home/a/Desktop/CARLA_0.9.16/RL_Agent_SAC"
cd "$BASE_DIR"

echo "╔═══════════════════════════════════════════════════════════════╗"
echo "║  SYSTEM STATUS - CARLA SAC Training                           ║"
echo "╚═══════════════════════════════════════════════════════════════╝"
echo ""

# Check processes
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "RUNNING PROCESSES"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Auto Manager
if pgrep -f "auto_manage.py.*run" > /dev/null; then
    PID=$(pgrep -f "auto_manage.py.*run" | head -1)
    echo "✅ Auto Manager: Running (PID: $PID)"
else
    echo "❌ Auto Manager: Not running"
fi

# CARLA
if pgrep -f "CarlaUE4" > /dev/null; then
    PID=$(pgrep -f "CarlaUE4" | head -1)
    echo "✅ CARLA Simulator: Running (PID: $PID)"
else
    echo "❌ CARLA Simulator: Not running"
fi

# Training
if pgrep -f "train_sac.py" > /dev/null; then
    PID=$(pgrep -f "train_sac.py" | head -1)
    echo "✅ Training Process: Running (PID: $PID)"
else
    echo "❌ Training Process: Not running"
fi

# Dashboard
if pgrep -f "app_fastapi" > /dev/null; then
    PID=$(pgrep -f "app_fastapi" | head -1)
    echo "✅ Dashboard: Running (PID: $PID)"
else
    echo "❌ Dashboard: Not running"
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "PORTS"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Check CARLA port
if nc -z localhost 2000 2>/dev/null || timeout 1 bash -c "echo > /dev/tcp/localhost/2000" 2>/dev/null; then
    echo "✅ CARLA Port 2000: OPEN"
else
    echo "❌ CARLA Port 2000: CLOSED"
fi

# Check Dashboard port
if nc -z localhost 5001 2>/dev/null || timeout 1 bash -c "echo > /dev/tcp/localhost/5001" 2>/dev/null; then
    echo "✅ Dashboard Port 5001: OPEN"
    echo "   → http://localhost:5001"
else
    echo "❌ Dashboard Port 5001: CLOSED"
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "LATEST LOGS"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Latest training log
LATEST_TRAIN_LOG=$(ls -t logs/sac_training_*.log 2>/dev/null | head -1)
if [ -n "$LATEST_TRAIN_LOG" ]; then
    echo "Training Log: $LATEST_TRAIN_LOG"
    echo "Last 3 lines:"
    tail -3 "$LATEST_TRAIN_LOG" 2>/dev/null | sed 's/^/  /'
else
    echo "No training log found"
fi

echo ""
echo "Auto Manager Log: logs/auto_manage.log"
echo "Last 3 lines:"
tail -3 logs/auto_manage.log 2>/dev/null | sed 's/^/  /'

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "QUICK ACTIONS"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  View logs:    tail -f logs/auto_manage.log"
echo "  View training: tail -f logs/sac_training_*.log"
echo "  Stop all:     pkill -f 'auto_manage|train_sac|CarlaUE4|app_fastapi'"
echo "  Status:       ./scripts/status.sh"
echo ""

