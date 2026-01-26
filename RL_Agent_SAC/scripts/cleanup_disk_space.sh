#!/bin/bash
# Comprehensive disk space cleanup script

BASE_DIR="/home/a/Desktop/CARLA_0.9.16/RL_Agent_SAC"
cd "$BASE_DIR"

echo "🧹 Disk Space Cleanup Script"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""

# Get disk space before
DISK_BEFORE=$(df -h . | tail -1 | awk '{print $4}')
echo "📊 Disk space before: $DISK_BEFORE available"
echo ""

# 1. Remove incompatible checkpoints (old model architecture)
echo "1. Removing incompatible checkpoints (old velocity=5 model)..."
INCOMPATIBLE_CP=$(find checkpoints/checkpoint -name "rl_model_2000_steps.zip" -type f 2>/dev/null)
if [ -n "$INCOMPATIBLE_CP" ]; then
    SIZE=$(du -h "$INCOMPATIBLE_CP" | cut -f1)
    rm -f "$INCOMPATIBLE_CP"
    echo "   ✅ Removed incompatible checkpoint: $(basename $INCOMPATIBLE_CP) ($SIZE)"
else
    echo "   ℹ️  No incompatible checkpoint found"
fi

# Remove huge replay buffer files
REPLAY_BUFFER=$(find checkpoints/checkpoint -name "*replay_buffer*.pkl" -type f 2>/dev/null)
if [ -n "$REPLAY_BUFFER" ]; then
    for f in $REPLAY_BUFFER; do
        SIZE=$(du -h "$f" | cut -f1)
        rm -f "$f"
        echo "   ✅ Removed replay buffer: $(basename $f) ($SIZE)"
    done
else
    echo "   ℹ️  No replay buffer files found"
fi
echo ""

# 2. Remove enhanced checkpoints (duplicate)
echo "2. Removing enhanced checkpoints (duplicate)..."
if [ -d "checkpoints/enhanced" ]; then
    SIZE=$(du -sh checkpoints/enhanced | cut -f1)
    rm -rf checkpoints/enhanced
    echo "   ✅ Removed enhanced checkpoints: $SIZE"
else
    echo "   ℹ️  No enhanced checkpoints found"
fi
echo ""

# 3. Remove error model
echo "3. Removing error model..."
if [ -f "checkpoints/error_model.zip" ]; then
    SIZE=$(du -h checkpoints/error_model.zip | cut -f1)
    rm -f checkpoints/error_model.zip
    echo "   ✅ Removed error model: $SIZE"
else
    echo "   ℹ️  No error model found"
fi
echo ""

# 4. Compress large log files
echo "4. Compressing large log files (>10MB)..."
find logs -name "*.log" -type f -size +10M ! -name "*.gz" | while read f; do
    SIZE=$(du -h "$f" | cut -f1)
    gzip "$f"
    echo "   ✅ Compressed: $(basename $f) ($SIZE)"
done
echo ""

# 5. Remove old logs (7+ days)
echo "5. Removing old log files (7+ days)..."
OLD_LOGS=$(find logs -name "*.log" -type f -mtime +7 | wc -l)
if [ "$OLD_LOGS" -gt 0 ]; then
    find logs -name "*.log" -type f -mtime +7 -delete
    echo "   ✅ Removed $OLD_LOGS old log file(s)"
else
    echo "   ℹ️  No old logs to remove"
fi
echo ""

# 6. Clean up SQLite database
echo "6. Cleaning up SQLite database..."
if [ -f "checkpoints/training_checkpoints.db" ]; then
    DB_SIZE_BEFORE=$(du -h checkpoints/training_checkpoints.db | cut -f1)
    python3 << 'PYTHON_SCRIPT'
import sqlite3
db_path = "checkpoints/training_checkpoints.db"
conn = sqlite3.connect(db_path)
# Remove old checkpoints (keep only latest)
conn.execute("DELETE FROM checkpoints WHERE timestep < (SELECT MAX(timestep) FROM checkpoints)")
conn.execute("VACUUM")
conn.close()
print("   ✅ Database cleaned and vacuumed")
PYTHON_SCRIPT
    DB_SIZE_AFTER=$(du -h checkpoints/training_checkpoints.db | cut -f1)
    echo "   Database: $DB_SIZE_BEFORE → $DB_SIZE_AFTER"
else
    echo "   ℹ️  No database found"
fi
echo ""

# 7. Remove temporary files
echo "7. Removing temporary files..."
TEMP_FILES=$(find . -name "*.tmp" -o -name "*.pyc" -o -name "__pycache__" | wc -l)
if [ "$TEMP_FILES" -gt 0 ]; then
    find . -name "*.tmp" -delete
    find . -name "*.pyc" -delete
    find . -type d -name "__pycache__" -exec rm -rf {} + 2>/dev/null
    echo "   ✅ Removed $TEMP_FILES temporary file(s)"
else
    echo "   ℹ️  No temporary files found"
fi
echo ""

# Get disk space after
DISK_AFTER=$(df -h . | tail -1 | awk '{print $4}')
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "📊 Disk space after: $DISK_AFTER available"
echo ""
echo "✅ Cleanup complete!"

