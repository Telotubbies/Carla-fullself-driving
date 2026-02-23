#!/bin/bash
# 🎯 Pipeline Orchestrator - Central Entry Point
#
# Usage:
#   ./scripts/run_orchestrator.sh                    # Run all phases
#   ./scripts/run_orchestrator.sh --status           # Show status
#   ./scripts/run_orchestrator.sh --phases collect_data,finetune_resnet
#   ./scripts/run_orchestrator.sh --resume           # Resume from checkpoint

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
cd "$PROJECT_ROOT"

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${BLUE}🎯 CARLA LSTM-MPC Pipeline Orchestrator${NC}"
echo ""

# Set environment variables
export HSA_OVERRIDE_GFX_VERSION=11.0.0
export PYTHONUNBUFFERED=1

# Run orchestrator
python3 scripts/orchestrator_main.py "$@"

