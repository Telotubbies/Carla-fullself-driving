#!/bin/bash
# Run all tests

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
cd "$PROJECT_ROOT"

echo "🧪 Running Test Suite"
echo "===================="
echo ""

# Set Python path
export PYTHONPATH="$PROJECT_ROOT:$PYTHONPATH"

# Run unit tests
echo "📋 Unit Tests:"
echo "--------------"
python3 -m pytest tests/unit/ -v --tb=short || python3 -m unittest discover -s tests/unit -p "test_*.py" -v

echo ""
echo "📋 Integration Tests:"
echo "---------------------"
python3 -m pytest tests/integration/ -v --tb=short || python3 -m unittest discover -s tests/integration -p "test_*.py" -v

echo ""
echo "✅ Test suite complete"

