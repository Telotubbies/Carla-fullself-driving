#!/bin/bash
# Auto Management Wrapper (Refactored)
# Delegates all logic to the robust Python implementation.

RL_AGENT_DIR="/home/a/Desktop/CARLA_0.9.16/RL_Agent"
PYTHON_SCRIPT="${RL_AGENT_DIR}/scripts/auto_manage.py"
VENV_PYTHON="${RL_AGENT_DIR}/venv/bin/python"

# Ensure we use the venv python if available, else system python
if [ -f "${VENV_PYTHON}" ]; then
    PYTHON_CMD="${VENV_PYTHON}"
else
    PYTHON_CMD="python3"
fi

# Pass arguments directly to the python script
# The python script handles "run" (monitor loop) vs launcher behavior
cd "${RL_AGENT_DIR}" || exit 1

case "$1" in
        start)
        echo "Starting Auto Manager (Python)..."
        "${PYTHON_CMD}" "${PYTHON_SCRIPT}"
            ;;
        stop)
        echo "Stopping Auto Manager..."
        pkill -f "auto_manage.py"
        echo "Stopped."
            ;;
        restart)
        "${RL_AGENT_DIR}/scripts/auto_manage.sh" stop
            sleep 2
        "${RL_AGENT_DIR}/scripts/auto_manage.sh" start
        ;;
    status)
        if pgrep -f "auto_manage.py" > /dev/null; then
            echo "✅ Auto Manager (Python) is running."
            tail -n 5 "${RL_AGENT_DIR}/logs/auto_manage.log"
        else
            echo "❌ Auto Manager is NOT running."
        fi
        ;;
    *)
        echo "Usage: $0 {start|stop|restart|status}"
            exit 1
            ;;
    esac
