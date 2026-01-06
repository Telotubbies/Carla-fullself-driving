#!/bin/bash
# Senior-level Training Startup Script
# Comprehensive pre-flight checks and safe training start

set -e

BASE_DIR="/home/a/Desktop/CARLA_0.9.16/RL_Agent_SAC"
cd "$BASE_DIR"

echo "═══════════════════════════════════════════════════════════════════════════════"
echo "🚀 SENIOR-LEVEL TRAINING STARTUP - COMPREHENSIVE VALIDATION"
echo "═══════════════════════════════════════════════════════════════════════════════"
echo ""

# Step 1: Pre-flight checks
echo "📋 STEP 1: PRE-FLIGHT CHECKS"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if ! ./venv/bin/python scripts/preflight_check.py; then
    echo ""
    echo "❌ PRE-FLIGHT CHECKS FAILED"
    echo "   Please fix the errors above before starting training"
    exit 1
fi
echo ""

# Step 2: System tests
echo "📋 STEP 2: SYSTEM TESTS"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
if ! ./venv/bin/python scripts/test_system.py; then
    echo ""
    echo "❌ SYSTEM TESTS FAILED"
    echo "   Please fix the errors above before starting training"
    exit 1
fi
echo ""

# Step 3: Verify all processes are stopped
echo "📋 STEP 3: CLEANUP EXISTING PROCESSES"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Stopping existing processes..."
./scripts/training/auto_manage.sh stop 2>/dev/null || true
sleep 2
echo "✅ Cleanup complete"
echo ""

# Step 4: Start auto manager (which will start everything)
echo "📋 STEP 4: STARTING AUTO MANAGER"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
./scripts/training/auto_manage.sh start
echo ""

# Step 5: Wait and verify
echo "📋 STEP 5: VERIFICATION"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "Waiting 10 seconds for services to start..."
sleep 10

# Check processes
CARLA_RUNNING=$(pgrep -f "CarlaUE4" >/dev/null && echo "✅" || echo "❌")
TRAIN_RUNNING=$(pgrep -f "train_sac" >/dev/null && echo "✅" || echo "❌")
AUTO_RUNNING=$(pgrep -f "auto_manage" >/dev/null && echo "✅" || echo "❌")
DASH_RUNNING=$(pgrep -f "uvicorn.*app_fastapi" >/dev/null && echo "✅" || echo "❌")

echo "  CARLA: $CARLA_RUNNING"
echo "  Training: $TRAIN_RUNNING"
echo "  Auto Manager: $AUTO_RUNNING"
echo "  Dashboard: $DASH_RUNNING"
echo ""

if [ "$CARLA_RUNNING" = "✅" ] && [ "$TRAIN_RUNNING" = "✅" ] && [ "$AUTO_RUNNING" = "✅" ]; then
    echo "✅ ALL SYSTEMS OPERATIONAL"
    echo ""
    echo "📊 Monitoring:"
    echo "  - Training logs: tail -f logs/rl_training_new.log"
    echo "  - Auto manager: tail -f logs/auto_manage.log"
    echo "  - Dashboard: http://localhost:5001"
    echo ""
    echo "═══════════════════════════════════════════════════════════════════════════════"
else
    echo "⚠️  Some services failed to start. Check logs for details."
    echo "═══════════════════════════════════════════════════════════════════════════════"
    exit 1
fi

