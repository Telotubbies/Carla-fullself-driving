#!/bin/bash
# Production Dashboard Startup Script

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "╔═══════════════════════════════════════════════════════════════╗"
echo "║  Starting Production Dashboard                                ║"
echo "╚═══════════════════════════════════════════════════════════════╝"
echo ""

# Load environment variables
if [ -f .env.production ]; then
    export $(cat .env.production | grep -v '^#' | xargs)
    echo "✅ Loaded production environment variables"
else
    echo "⚠️  .env.production not found, using defaults"
    export PRODUCTION=true
    export DEBUG=false
    export LOG_LEVEL=INFO
    export HOST=0.0.0.0
    export PORT=5001
    export WORKERS=2
fi

# Check if React build exists
REACT_BUILD="../static/react"
if [ ! -d "$REACT_BUILD" ] || [ ! -f "$REACT_BUILD/index.html" ]; then
    echo "⚠️  React build not found. Building React dashboard..."
    cd react_dashboard
    npm install
    npm run build
    cd ..
    echo "✅ React dashboard built"
fi

# Check Python dependencies
echo "📦 Checking Python dependencies..."
python3 -c "import fastapi, uvicorn, slowapi, cachetools" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "⚠️  Missing dependencies. Installing..."
    pip install -r requirements.txt
fi

# Start production server
echo ""
echo "🚀 Starting production server..."
echo "   Host: $HOST"
echo "   Port: $PORT"
echo "   Workers: $WORKERS"
echo "   Mode: PRODUCTION"
echo ""

if [ "$WORKERS" -gt 1 ]; then
    # Use gunicorn with uvicorn workers for multi-worker setup
    if command -v gunicorn &> /dev/null; then
        gunicorn app_fastapi_production:app \
            --workers $WORKERS \
            --worker-class uvicorn.workers.UvicornWorker \
            --bind $HOST:$PORT \
            --access-logfile - \
            --error-logfile - \
            --log-level info
    else
        echo "⚠️  Gunicorn not found, using uvicorn with single worker"
        uvicorn app_fastapi_production:app \
            --host $HOST \
            --port $PORT \
            --log-level info \
            --no-access-log
    fi
else
    # Single worker with uvicorn
    uvicorn app_fastapi_production:app \
        --host $HOST \
        --port $PORT \
        --log-level info \
        --no-access-log
fi

