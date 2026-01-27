#!/bin/bash
# Disk cleanup script for CARLA training

BASE_DIR="/home/a/Desktop/CARLA_0.9.16/RL_Agent_SAC"
cd "$BASE_DIR"

echo "🧹 Disk Cleanup Script"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# 1. Remove corrupted checkpoints
echo "1. Removing corrupted checkpoints..."
find checkpoints -name "*.zip" -type f | while read f; do
    if ! unzip -t "$f" >/dev/null 2>&1; then
        echo "  Removing corrupted: $(basename $f)"
        rm -f "$f"
    fi
done

# 2. Keep only latest 3 checkpoints
echo "2. Keeping only latest 3 checkpoints..."
LATEST_CP=$(ls -t checkpoints/checkpoint/*.zip 2>/dev/null | head -1)
if [ -n "$LATEST_CP" ]; then
    LATEST_STEP=$(basename "$LATEST_CP" | grep -oE "[0-9]+" | head -1)
    KEEP_STEPS=$(($LATEST_STEP - 2000))
    find checkpoints/checkpoint -name "rl_model_*.zip" -type f | while read f; do
        STEP=$(basename "$f" | grep -oE "[0-9]+" | head -1)
        if [ -n "$STEP" ] && [ "$STEP" -lt "$KEEP_STEPS" ]; then
            echo "  Removing old: $(basename $f) (step $STEP)"
            rm -f "$f"
        fi
    done
fi

# 3. Remove old logs (7+ days)
echo "3. Removing old log files..."
find logs -name "*.log" -type f -mtime +7 -delete
echo "  ✅ Old logs removed"

# 4. Compress large logs
echo "4. Compressing large log files..."
find logs -name "*.log" -type f -size +50M -exec gzip {} \;
echo "  ✅ Large logs compressed"

# 5. Remove enhanced checkpoints
if [ -d checkpoints/enhanced ]; then
    echo "5. Removing enhanced checkpoints..."
    rm -rf checkpoints/enhanced
    echo "  ✅ Enhanced checkpoints removed"
fi

echo ""
echo "✅ Cleanup complete!"
df -h . | tail -1 | awk '{printf "Disk: %s used, %s available\n", $5, $4}'
