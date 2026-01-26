#!/bin/bash
# Build Production Dashboard

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "╔═══════════════════════════════════════════════════════════════╗"
echo "║  Building Production Dashboard                                ║"
echo "╚═══════════════════════════════════════════════════════════════╝"
echo ""

# Build React Dashboard
echo "📦 Building React Dashboard..."
cd react_dashboard

# Install dependencies if needed
if [ ! -d "node_modules" ]; then
    echo "📥 Installing npm dependencies..."
    npm install
fi

# Build for production
echo "🔨 Building production bundle..."
npm run build

if [ $? -ne 0 ]; then
    echo "❌ React build failed!"
    exit 1
fi

# Copy build to static directory
echo "📋 Copying build to static directory..."
cd ..
mkdir -p static/react
cp -r react_dashboard/dist/* static/react/

echo ""
echo "✅ Production build complete!"
echo "   Build output: static/react/"
echo ""
echo "🚀 To start production server:"
echo "   ./start_production.sh"
echo ""

