#!/bin/bash
# Comprehensive System Check Before Running Training
# ตรวจสอบระบบทั้งหมดก่อนเริ่ม training

BASE_DIR="/home/a/Desktop/CARLA_0.9.16/RL_Agent_SAC"
CARLA_DIR="/home/a/Desktop/CARLA_0.9.16"
cd "$BASE_DIR"

echo "╔═══════════════════════════════════════════════════════════════╗"
echo "║  SYSTEM PRE-FLIGHT CHECK - CARLA SAC Training                 ║"
echo "╚═══════════════════════════════════════════════════════════════╝"
echo ""

ERRORS=0
WARNINGS=0

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

check_pass() {
    echo -e "${GREEN}✅ PASS${NC}: $1"
}

check_fail() {
    echo -e "${RED}❌ FAIL${NC}: $1"
    ((ERRORS++))
}

check_warn() {
    echo -e "${YELLOW}⚠️  WARN${NC}: $1"
    ((WARNINGS++))
}

check_info() {
    echo -e "${BLUE}ℹ️  INFO${NC}: $1"
}

echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "1. DIRECTORY STRUCTURE"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Check base directories
if [ -d "$BASE_DIR" ]; then
    check_pass "Base directory exists: $BASE_DIR"
else
    check_fail "Base directory missing: $BASE_DIR"
fi

if [ -d "$CARLA_DIR" ]; then
    check_pass "CARLA directory exists: $CARLA_DIR"
else
    check_fail "CARLA directory missing: $CARLA_DIR"
fi

# Check important subdirectories
for dir in "carla_env" "models" "training" "utils" "config" "checkpoints" "logs"; do
    if [ -d "$BASE_DIR/$dir" ]; then
        check_pass "Directory exists: $dir/"
    else
        check_fail "Directory missing: $dir/"
    fi
done

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "2. CONFIGURATION FILES"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

CONFIG_FILE="$BASE_DIR/config/sac_config.yaml"
if [ -f "$CONFIG_FILE" ]; then
    check_pass "Config file exists: config/sac_config.yaml"
    
    # Check important config values
    if grep -q "save_optimizer: false" "$CONFIG_FILE"; then
        check_pass "save_optimizer: false (disk space optimized)"
    else
        check_warn "save_optimizer not set to false (may use more disk space)"
    fi
    
    if grep -q "max_checkpoints_to_keep: 1" "$CONFIG_FILE"; then
        check_pass "max_checkpoints_to_keep: 1 (disk space optimized)"
    else
        check_warn "max_checkpoints_to_keep not set to 1"
    fi
    
    SAVE_FREQ=$(grep "save_freq:" "$CONFIG_FILE" | awk '{print $2}')
    if [ -n "$SAVE_FREQ" ]; then
        check_info "Checkpoint save frequency: $SAVE_FREQ steps"
        if [ "$SAVE_FREQ" -ge 2000 ]; then
            check_pass "Save frequency >= 2000 (good for disk space)"
        else
            check_warn "Save frequency < 2000 (may use more disk space)"
        fi
    fi
else
    check_fail "Config file missing: config/sac_config.yaml"
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "3. PYTHON ENVIRONMENT"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Check Python
if command -v python3 &> /dev/null; then
    PYTHON_VERSION=$(python3 --version 2>&1 | awk '{print $2}')
    check_pass "Python3 found: $PYTHON_VERSION"
    
    # Check Python version (need 3.8+)
    PYTHON_MAJOR=$(echo $PYTHON_VERSION | cut -d. -f1)
    PYTHON_MINOR=$(echo $PYTHON_VERSION | cut -d. -f2)
    if [ "$PYTHON_MAJOR" -ge 3 ] && [ "$PYTHON_MINOR" -ge 8 ]; then
        check_pass "Python version >= 3.8"
    else
        check_fail "Python version < 3.8 (need 3.8+)"
    fi
else
    check_fail "Python3 not found"
fi

# Check virtual environment
if [ -d "$BASE_DIR/venv" ]; then
    check_pass "Virtual environment exists: venv/"
    if [ -f "$BASE_DIR/venv/bin/activate" ]; then
        check_pass "venv activation script exists"
    fi
else
    check_warn "Virtual environment not found (recommended to use venv)"
fi

# Check requirements.txt
if [ -f "$BASE_DIR/requirements.txt" ]; then
    check_pass "requirements.txt exists"
    REQ_COUNT=$(wc -l < "$BASE_DIR/requirements.txt")
    check_info "Found $REQ_COUNT dependencies in requirements.txt"
else
    check_warn "requirements.txt not found"
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "4. KEY PYTHON MODULES"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Check if we can import key modules
if [ -d "$BASE_DIR/venv" ]; then
    PYTHON_CMD="$BASE_DIR/venv/bin/python3"
else
    PYTHON_CMD="python3"
fi

KEY_MODULES=("torch" "numpy" "gymnasium" "stable_baselines3" "carla" "yaml" "psutil")
for module in "${KEY_MODULES[@]}"; do
    if $PYTHON_CMD -c "import $module" 2>/dev/null; then
        VERSION=$($PYTHON_CMD -c "import $module; print(getattr($module, '__version__', 'unknown'))" 2>/dev/null)
        check_pass "$module installed (version: $VERSION)"
    else
        check_fail "$module not installed"
    fi
done

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "5. CARLA SIMULATOR"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Check CARLA executable
CARLA_EXEC="$CARLA_DIR/CarlaUE4.sh"
if [ -f "$CARLA_EXEC" ]; then
    check_pass "CARLA executable exists: CarlaUE4.sh"
    if [ -x "$CARLA_EXEC" ]; then
        check_pass "CARLA executable is executable"
    else
        check_warn "CARLA executable not executable (chmod +x needed?)"
    fi
else
    check_fail "CARLA executable missing: CarlaUE4.sh"
fi

# Check if CARLA is running
if pgrep -f "CarlaUE4" > /dev/null; then
    check_info "CARLA process is running"
    CARLA_PORT=$(netstat -tlnp 2>/dev/null | grep :2000 | awk '{print $4}' | cut -d: -f2)
    if [ -n "$CARLA_PORT" ]; then
        check_pass "CARLA server listening on port 2000"
    else
        check_warn "CARLA running but port 2000 not accessible"
    fi
else
    check_info "CARLA not running (will be started by auto_manage)"
fi

# Check PythonAPI
if [ -d "$CARLA_DIR/PythonAPI/carla" ]; then
    check_pass "CARLA PythonAPI exists"
    if [ -f "$CARLA_DIR/PythonAPI/carla/dist/carla-0.9.16"*.whl ]; then
        check_pass "CARLA Python wheel found"
    else
        check_warn "CARLA Python wheel not found (may need to install)"
    fi
else
    check_fail "CARLA PythonAPI missing"
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "6. DISK SPACE"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Get disk space
DISK_INFO=$(df -h "$BASE_DIR" | tail -1)
DISK_TOTAL=$(echo $DISK_INFO | awk '{print $2}')
DISK_USED=$(echo $DISK_INFO | awk '{print $3}')
DISK_AVAIL=$(echo $DISK_INFO | awk '{print $4}')
DISK_PERCENT=$(echo $DISK_INFO | awk '{print $5}' | sed 's/%//')

check_info "Disk space: $DISK_USED used / $DISK_TOTAL total ($DISK_PERCENT% used)"
check_info "Available: $DISK_AVAIL"

if [ "$DISK_PERCENT" -lt 80 ]; then
    check_pass "Disk usage < 80%"
elif [ "$DISK_PERCENT" -lt 90 ]; then
    check_warn "Disk usage >= 80% (consider cleanup)"
else
    check_fail "Disk usage >= 90% (cleanup required!)"
fi

# Check checkpoint database size
DB_FILE="$BASE_DIR/checkpoints/training_checkpoints.db"
if [ -f "$DB_FILE" ]; then
    DB_SIZE=$(du -h "$DB_FILE" | cut -f1)
    DB_SIZE_BYTES=$(stat -f%z "$DB_FILE" 2>/dev/null || stat -c%s "$DB_FILE" 2>/dev/null)
    DB_SIZE_MB=$((DB_SIZE_BYTES / 1024 / 1024))
    check_info "Checkpoint DB size: $DB_SIZE ($DB_SIZE_MB MB)"
    
    if [ "$DB_SIZE_MB" -lt 1000 ]; then
        check_pass "Database size < 1GB"
    elif [ "$DB_SIZE_MB" -lt 5000 ]; then
        check_warn "Database size >= 1GB (consider cleanup)"
    else
        check_fail "Database size >= 5GB (cleanup required!)"
    fi
else
    check_info "Checkpoint database not found (will be created)"
fi

# Check checkpoint directory
if [ -d "$BASE_DIR/checkpoints/checkpoint" ]; then
    CP_COUNT=$(find "$BASE_DIR/checkpoints/checkpoint" -name "*.zip" 2>/dev/null | wc -l)
    CP_SIZE=$(du -sh "$BASE_DIR/checkpoints/checkpoint" 2>/dev/null | cut -f1)
    check_info "Checkpoint files: $CP_COUNT files ($CP_SIZE total)"
    
    if [ "$CP_COUNT" -le 5 ]; then
        check_pass "Checkpoint count <= 5"
    elif [ "$CP_COUNT" -le 20 ]; then
        check_warn "Checkpoint count > 5 (consider cleanup)"
    else
        check_fail "Checkpoint count > 20 (cleanup required!)"
    fi
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "7. SYSTEM RESOURCES"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Check RAM
if command -v free &> /dev/null; then
    RAM_TOTAL=$(free -g | awk '/^Mem:/{print $2}')
    RAM_AVAIL=$(free -g | awk '/^Mem:/{print $7}')
    RAM_USED=$((RAM_TOTAL - RAM_AVAIL))
    RAM_PERCENT=$((RAM_USED * 100 / RAM_TOTAL))
    
    check_info "RAM: ${RAM_USED}GB used / ${RAM_TOTAL}GB total (${RAM_PERCENT}% used)"
    check_info "Available: ${RAM_AVAIL}GB"
    
    if [ "$RAM_TOTAL" -ge 16 ]; then
        check_pass "RAM >= 16GB (recommended)"
    elif [ "$RAM_TOTAL" -ge 8 ]; then
        check_warn "RAM < 16GB (may be slow)"
    else
        check_fail "RAM < 8GB (insufficient!)"
    fi
fi

# Check GPU
if command -v nvidia-smi &> /dev/null; then
    GPU_INFO=$(nvidia-smi --query-gpu=name,memory.total,memory.used --format=csv,noheader,nounits 2>/dev/null | head -1)
    if [ -n "$GPU_INFO" ]; then
        GPU_NAME=$(echo $GPU_INFO | cut -d',' -f1)
        GPU_MEM_TOTAL=$(echo $GPU_INFO | cut -d',' -f2 | tr -d ' ')
        GPU_MEM_USED=$(echo $GPU_INFO | cut -d',' -f3 | tr -d ' ')
        GPU_MEM_AVAIL=$((GPU_MEM_TOTAL - GPU_MEM_USED))
        GPU_MEM_PERCENT=$((GPU_MEM_USED * 100 / GPU_MEM_TOTAL))
        
        check_pass "GPU detected: $GPU_NAME"
        check_info "GPU Memory: ${GPU_MEM_USED}MB used / ${GPU_MEM_TOTAL}MB total (${GPU_MEM_PERCENT}%)"
        check_info "Available: ${GPU_MEM_AVAIL}MB"
        
        if [ "$GPU_MEM_TOTAL" -ge 8192 ]; then
            check_pass "GPU memory >= 8GB (good)"
        elif [ "$GPU_MEM_TOTAL" -ge 4096 ]; then
            check_warn "GPU memory < 8GB (may be limited)"
        else
            check_fail "GPU memory < 4GB (insufficient!)"
        fi
    fi
elif command -v rocm-smi &> /dev/null; then
    check_info "ROCm GPU detected (AMD)"
    GPU_INFO=$(rocm-smi --showid --showmeminfo vram --csv 2>/dev/null | head -2 | tail -1)
    if [ -n "$GPU_INFO" ]; then
        check_pass "AMD GPU detected"
    fi
else
    check_warn "No GPU detected (will use CPU - very slow)"
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "8. KEY FILES"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

KEY_FILES=(
    "training/train_sac.py"
    "carla_env/carla_rl_env.py"
    "models/custom_policy.py"
    "utils/sqlite_checkpoint.py"
    "scripts/training/auto_manage.py"
    "web_dashboard/app_fastapi.py"
)

for file in "${KEY_FILES[@]}"; do
    if [ -f "$BASE_DIR/$file" ]; then
        check_pass "File exists: $file"
    else
        check_fail "File missing: $file"
    fi
done

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "9. PERMISSIONS"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Check write permissions
if [ -w "$BASE_DIR" ]; then
    check_pass "Base directory is writable"
else
    check_fail "Base directory not writable"
fi

if [ -w "$BASE_DIR/checkpoints" ]; then
    check_pass "Checkpoints directory is writable"
else
    check_fail "Checkpoints directory not writable"
fi

if [ -w "$BASE_DIR/logs" ]; then
    check_pass "Logs directory is writable"
else
    check_fail "Logs directory not writable"
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "10. RUNNING PROCESSES"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

# Check for existing training processes
if pgrep -f "train_sac.py" > /dev/null; then
    check_warn "Training process already running (may conflict)"
    pgrep -f "train_sac.py" | while read pid; do
        check_info "  PID $pid: $(ps -p $pid -o cmd= | cut -c1-80)"
    done
else
    check_pass "No training process running"
fi

# Check for auto_manage
if pgrep -f "auto_manage.py" > /dev/null; then
    check_info "Auto-manager process running"
else
    check_info "Auto-manager not running (will start with training)"
fi

# Check for dashboard
if pgrep -f "app_fastapi.py" > /dev/null; then
    check_info "Dashboard process running"
else
    check_info "Dashboard not running (will start with auto-manager)"
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "SUMMARY"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

echo ""
if [ $ERRORS -eq 0 ] && [ $WARNINGS -eq 0 ]; then
    echo -e "${GREEN}✅ ALL CHECKS PASSED${NC}"
    echo "System is ready for training!"
    exit 0
elif [ $ERRORS -eq 0 ]; then
    echo -e "${YELLOW}⚠️  $WARNINGS WARNING(S)${NC}"
    echo "System is ready but has some warnings"
    exit 0
else
    echo -e "${RED}❌ $ERRORS ERROR(S), $WARNINGS WARNING(S)${NC}"
    echo "Please fix errors before starting training"
    exit 1
fi

