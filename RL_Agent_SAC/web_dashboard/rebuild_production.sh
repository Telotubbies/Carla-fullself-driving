#!/bin/bash
# Complete Production Dashboard Rebuild Script

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

echo "╔═══════════════════════════════════════════════════════════════╗"
echo "║  PRODUCTION DASHBOARD REBUILD                                  ║"
echo "╚═══════════════════════════════════════════════════════════════╝"
echo ""

# Step 1: Install Python dependencies
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Step 1: Installing Python Dependencies"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Check if venv exists
if [ -d "../venv" ]; then
    echo "✅ Using virtual environment"
    source ../venv/bin/activate
else
    echo "⚠️  Virtual environment not found, using system Python"
fi

# Install dependencies
pip install -q slowapi cachetools 2>/dev/null || {
    echo "⚠️  Installing with --user flag..."
    pip install --user slowapi cachetools
}

echo "✅ Python dependencies installed"
echo ""

# Step 2: Install React dependencies
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Step 2: Installing React Dependencies"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

cd react_dashboard

if [ ! -d "node_modules" ]; then
    echo "📥 Installing npm dependencies..."
    npm install --silent
    echo "✅ npm dependencies installed"
else
    echo "✅ node_modules already exists"
fi

cd ..
echo ""

# Step 3: Build React Dashboard
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Step 3: Building React Dashboard"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

cd react_dashboard
echo "🔨 Building production bundle..."
npm run build

if [ $? -ne 0 ]; then
    echo "❌ React build failed!"
    exit 1
fi

cd ..
echo "✅ React build complete"
echo ""

# Step 4: Copy build to static directory
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Step 4: Copying Build Files"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

mkdir -p static/react
echo "📋 Copying build files..."
rm -rf static/react/*
cp -r react_dashboard/dist/* static/react/

if [ -f "static/react/index.html" ]; then
    echo "✅ Build files copied successfully"
else
    echo "❌ Build files not found!"
    exit 1
fi
echo ""

# Step 5: Verify production files
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Step 5: Verification"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Check Python files
if [ -f "app_fastapi_production.py" ]; then
    echo "✅ Production backend: app_fastapi_production.py"
else
    echo "❌ Production backend not found!"
    exit 1
fi

# Check React build
if [ -f "static/react/index.html" ]; then
    echo "✅ React build: static/react/index.html"
    ASSET_COUNT=$(find static/react/assets -type f 2>/dev/null | wc -l)
    echo "   Assets: $ASSET_COUNT files"
else
    echo "❌ React build not found!"
    exit 1
fi

# Check scripts
if [ -f "start_production.sh" ] && [ -x "start_production.sh" ]; then
    echo "✅ Production startup script: start_production.sh"
else
    echo "⚠️  Making startup script executable..."
    chmod +x start_production.sh
fi

echo ""
echo "╔═══════════════════════════════════════════════════════════════╗"
echo "║  ✅ PRODUCTION DASHBOARD REBUILD COMPLETE                     ║"
echo "╚═══════════════════════════════════════════════════════════════╝"
echo ""
echo "📊 Production Dashboard Ready!"
echo ""
echo "🚀 To start production server:"
echo "   ./start_production.sh"
echo ""
echo "📚 Documentation:"
echo "   See PRODUCTION_README.md for details"
echo ""

