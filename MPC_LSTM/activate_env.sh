#!/bin/bash
# Activate LSTM-MPC project virtual environment
# Usage: source activate_env.sh

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/venv/bin/activate"
echo "LSTM-MPC env activated (Python $(python --version))"
