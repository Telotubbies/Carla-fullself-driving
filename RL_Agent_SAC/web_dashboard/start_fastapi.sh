#!/bin/bash
# Start FastAPI Dashboard for SAC Training

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_DIR/web_dashboard"

echo "=========================================="
echo "🚀 Starting SAC Training Dashboard"
echo "=========================================="
echo ""

# Use port 5001 for SAC (PPO uses 5000)
PORT=5001

# Check if port is already in use
if lsof -Pi :$PORT -sTCP:LISTEN -t >/dev/null ; then
    echo "⚠️  Port $PORT is already in use"
    echo "   Stopping existing process..."
    lsof -ti:$PORT | xargs kill -9 2>/dev/null
    sleep 2
fi

echo "Starting FastAPI dashboard on port $PORT..."
echo ""

# Activate virtual environment if exists
if [ -d "../venv" ]; then
    source ../venv/bin/activate
fi

# Run FastAPI dashboard
uvicorn app_fastapi:app --host 0.0.0.0 --port $PORT --reload
