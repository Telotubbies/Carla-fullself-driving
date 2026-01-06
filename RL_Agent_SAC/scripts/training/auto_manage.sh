#!/bin/bash
# Auto Management Wrapper for SAC Training
# Delegates all logic to the robust Python implementation.

RL_AGENT_DIR="/home/a/Desktop/CARLA_0.9.16/RL_Agent_SAC"
PYTHON_SCRIPT="${RL_AGENT_DIR}/scripts/training/auto_manage.py"
VENV_PYTHON="${RL_AGENT_DIR}/venv/bin/python"

# Ensure we use the venv python if available, else system python
if [ -f "${VENV_PYTHON}" ]; then
    PYTHON_CMD="${VENV_PYTHON}"
else
    PYTHON_CMD="python3"
fi

# Pass arguments directly to the python script
cd "${RL_AGENT_DIR}" || exit 1

case "$1" in
    start)
        echo "Starting SAC Auto Manager (Python)..."
        nohup "${PYTHON_CMD}" "${PYTHON_SCRIPT}" > /dev/null 2>&1 &
        echo "Started in background. Check logs: ${RL_AGENT_DIR}/logs/auto_manage.log"
        ;;
    stop)
        echo "Stopping SAC Auto Manager..."
        pkill -f "auto_manage.py.*RL_Agent_SAC"
        echo "Stopped."
        ;;
    restart)
        "${RL_AGENT_DIR}/scripts/training/auto_manage.sh" stop
        sleep 2
        "${RL_AGENT_DIR}/scripts/training/auto_manage.sh" start
        ;;
    status)
        if pgrep -f "auto_manage.py" > /dev/null && pgrep -f "RL_Agent_SAC.*auto_manage" > /dev/null; then
            echo "✅ SAC Auto Manager (Python) is running."
            tail -n 5 "${RL_AGENT_DIR}/logs/auto_manage.log" 2>/dev/null || echo "No log file found."
        elif pgrep -f "RL_Agent_SAC.*auto_manage.py" > /dev/null; then
            echo "✅ SAC Auto Manager (Python) is running."
            tail -n 5 "${RL_AGENT_DIR}/logs/auto_manage.log" 2>/dev/null || echo "No log file found."
        else
            echo "❌ SAC Auto Manager is NOT running."
        fi
        ;;
    *)
        echo "Usage: $0 {start|stop|restart|status}"
        exit 1
        ;;
esac

