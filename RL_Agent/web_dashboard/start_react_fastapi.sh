#!/bin/bash
# Start FastAPI + React Dashboard
# This script starts FastAPI backend and optionally React dev server

cd "$(dirname "$0")"
cd ..

source venv/bin/activate

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "🚀 Starting FastAPI + React Dashboard"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# Check if React build exists
REACT_BUILD="../web_dashboard/static/react/index.html"
if [ -f "$REACT_BUILD" ]; then
    echo "✅ React build found - using production build"
    echo ""
    echo "📊 Dashboard: http://localhost:5000"
    echo "📡 API: http://localhost:5000/api/status"
    echo "📚 API Docs: http://localhost:5000/docs"
    echo ""
    cd web_dashboard
    python app_fastapi.py
else
    echo "⚠️  React build not found"
    echo ""
    echo "Options:"
    echo "  1. Build React first: cd web_dashboard/react_dashboard && npm install && npm run build"
    echo "  2. Start React dev server separately: cd web_dashboard/react_dashboard && npm run dev"
    echo ""
    echo "Starting FastAPI (will serve fallback dashboard)..."
    echo ""
    cd web_dashboard
    python app_fastapi.py
fi

