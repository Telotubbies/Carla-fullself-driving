#!/bin/bash
# Process Check Script - แยก Server และ Client เป็นส่วนๆ

RL_AGENT_DIR="/home/a/Desktop/CARLA_0.9.16/RL_Agent"
cd "$RL_AGENT_DIR"

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

echo -e "${CYAN}=== 🔍 Process Check (Server & Client Separated) ===${NC}"
echo "   Time: $(date '+%Y-%m-%d %H:%M:%S')"
echo ""

# ============================================================================
# SECTION 1: CARLA SERVER
# ============================================================================
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}📡 SECTION 1: CARLA SERVER${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

echo "1.1 Process Status:"
if pgrep -f "CarlaUE4" > /dev/null; then
    CARLA_COUNT=$(pgrep -f "CarlaUE4" | wc -l)
    echo -e "   ${GREEN}✅ Running ($CARLA_COUNT processes)${NC}"
    pgrep -f "CarlaUE4" | head -3 | while read pid; do
        ps -p $pid -o pid,pcpu,pmem,etime,cmd --no-headers 2>/dev/null | awk '{printf "      PID: %s | CPU: %s%% | MEM: %s%% | Time: %s\n", $1, $2, $3, $4}'
    done
else
    echo -e "   ${RED}❌ Not running${NC}"
fi

echo ""
echo "1.2 Connection Status:"
python3 << 'PYEOF'
import carla
try:
    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)
    world = client.get_world()
    map_name = world.get_map().name
    actors = len(world.get_actors())
    print("   ✅ Connected")
    print(f"      Map: {map_name}")
    print(f"      Actors: {actors}")
except Exception as e:
    print(f"   ❌ Connection failed: {e}")
PYEOF

echo ""
echo "1.3 Server Resources:"
if pgrep -f "CarlaUE4" > /dev/null; then
    CARLA_PID=$(pgrep -f "CarlaUE4" | head -1)
    ps -p $CARLA_PID -o pid,pcpu,pmem,vsz,rss --no-headers 2>/dev/null | awk '{printf "      CPU: %s%% | MEM: %s%% | Virtual: %s MB | Resident: %s MB\n", $2, $3, int($4/1024), int($5/1024)}'
else
    echo "      (No process)"
fi

# ============================================================================
# SECTION 2: TRAINING CLIENT
# ============================================================================
echo ""
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}🤖 SECTION 2: TRAINING CLIENT${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

echo "2.1 Process Status:"
if pgrep -f "train.py" > /dev/null; then
    RL_PID=$(pgrep -f "train.py" | head -1)
    ps -p $RL_PID -o pid,user,pcpu,pmem,etime,stat --no-headers 2>/dev/null | awk '{printf "   ✅ Running\n      PID: %s | CPU: %s%% | MEM: %s%% | Time: %s | State: %s\n", $1, $3, $4, $5, $6}'
else
    echo -e "   ${RED}❌ Not running${NC}"
fi

echo ""
echo "2.2 Client Connection to Server:"
if pgrep -f "train.py" > /dev/null; then
    if [ -f "logs/rl_training.log" ]; then
        echo "   Latest connection activity:"
        tail -20 logs/rl_training.log | grep -E "(Connecting|Connected|timeout|Timeout|error|Error)" | tail -3 | sed 's/^/      /' || echo "      (No recent connection logs)"
    else
        echo "      (No log file)"
    fi
else
    echo "   (No training process)"
fi

echo ""
echo "2.3 Training Progress:"
if [ -f "logs/rl_training.log" ]; then
    echo "   Current step:"
    tail -10 logs/rl_training.log | grep -E "Step [0-9]+:|Callback.*Step|timestep" | tail -1 | sed 's/^/      /' || echo "      (checking...)"
    echo ""
    echo "   Latest checkpoint:"
    if [ -d "checkpoints/checkpoint" ]; then
        LATEST_CP=$(ls -t checkpoints/checkpoint/*.zip 2>/dev/null | head -1)
        if [ -n "$LATEST_CP" ]; then
            STEPS=$(echo "$LATEST_CP" | grep -oP '\d+(?=_steps)' || echo "unknown")
            PROGRESS=$(echo "scale=2; $STEPS / 500000 * 100" | bc 2>/dev/null || echo "0")
            echo "      File: $(basename $LATEST_CP)"
            echo "      Steps: $STEPS / 500,000 ($PROGRESS%)"
        else
            echo "      No checkpoint files"
        fi
    else
        echo "      No checkpoint directory"
    fi
else
    echo "   (No log file)"
fi

echo ""
echo "2.4 Client Resources:"
if pgrep -f "train.py" > /dev/null; then
    RL_PID=$(pgrep -f "train.py" | head -1)
    ps -p $RL_PID -o pid,pcpu,pmem,vsz,rss --no-headers 2>/dev/null | awk '{printf "      CPU: %s%% | MEM: %s%% | Virtual: %s MB | Resident: %s MB\n", $2, $3, int($4/1024), int($5/1024)}'
else
    echo "      (No process)"
fi

# ============================================================================
# SECTION 3: SYSTEM PROCESSES
# ============================================================================
echo ""
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}🖥️  SECTION 3: SYSTEM PROCESSES${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

echo "3.1 Master Pipeline:"
if pgrep -f "master_pipeline.sh" > /dev/null; then
    MP_PID=$(pgrep -f "master_pipeline.sh" | head -1)
    ps -p $MP_PID -o pid,user,pcpu,pmem,etime --no-headers 2>/dev/null | awk '{printf "   ✅ Running (PID: %s | CPU: %s%% | MEM: %s%% | Time: %s)\n", $1, $3, $4, $5}'
else
    echo -e "   ${RED}❌ Not running${NC}"
fi

echo ""
echo "3.2 Dashboard:"
if pgrep -f "app.py" > /dev/null; then
    DASH_PID=$(pgrep -f "app.py" | head -1)
    ps -p $DASH_PID -o pid,user,pcpu,pmem,etime --no-headers 2>/dev/null | awk '{printf "   ✅ Running (PID: %s | CPU: %s%% | MEM: %s%% | Time: %s)\n", $1, $3, $4, $5}'
    HTTP_CODE=$(curl -s -o /dev/null -w "%{http_code}" http://localhost:5000/api/status 2>/dev/null || echo "000")
    if [ "$HTTP_CODE" = "200" ]; then
        echo -e "      Web: http://localhost:5000 ${GREEN}✅${NC}"
    else
        echo -e "      Web: Not accessible (HTTP $HTTP_CODE) ${YELLOW}⚠️${NC}"
    fi
else
    echo -e "   ${RED}❌ Not running${NC}"
fi

# ============================================================================
# SECTION 4: RESOURCE USAGE
# ============================================================================
echo ""
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${BLUE}📊 SECTION 4: RESOURCE USAGE${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

echo "4.1 CPU:"
echo "   Cores: $(nproc)"
echo "   Load: $(uptime | awk -F'load average:' '{print $2}')"
echo ""
echo "4.2 Memory:"
free -h | grep Mem | awk '{printf "   Total: %s | Used: %s | Free: %s | Available: %s (%.1f%% used)\n", $2, $3, $4, $7, ($3/$2)*100}'
echo ""
echo "4.3 GPU:"
if command -v rocm-smi &> /dev/null; then
    rocm-smi --showuse --showmemuse --showtemp 2>/dev/null | grep -E "GPU\[0\]" | head -4 | sed 's/^/   /'
    # Also show temperature separately if available
    GPU_TEMP=$(rocm-smi --showtemp --json 2>/dev/null | python3 -c "import sys, json; data=json.load(sys.stdin); print(next((v.get('Temperature (Sensor junction) (C)', v.get('Temperature (Sensor edge) (C)', 'N/A')) for k,v in data.items() if 'card' in k.lower()), 'N/A'))" 2>/dev/null || echo "N/A")
    if [ "${GPU_TEMP}" != "N/A" ] && [ "${GPU_TEMP}" != "None" ] && [ -n "${GPU_TEMP}" ]; then
        echo "   GPU Temperature: ${GPU_TEMP}°C"
    fi
elif command -v nvidia-smi &> /dev/null; then
    nvidia-smi --query-gpu=index,utilization.gpu,utilization.memory,memory.used,memory.total,temperature.gpu --format=csv,noheader,nounits 2>/dev/null | awk -F', ' '{printf "   GPU %s: Usage=%s%% | Mem=%s%% (%s/%s MB) | Temp=%s°C\n", $1, $2, $3, $4, $5, $6}'
else
    echo "   (GPU monitoring not available)"
fi

# ============================================================================
# SUMMARY
# ============================================================================
echo ""
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo -e "${GREEN}✅ SUMMARY${NC}"
echo -e "${BLUE}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
echo ""

echo "Server (CARLA):"
if pgrep -f "CarlaUE4" > /dev/null; then
    python3 << 'PYEOF'
import carla
try:
    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)
    world = client.get_world()
    print("   ✅ Running and responding")
except:
    print("   ⚠️  Running but not responding")
PYEOF
else
    echo -e "   ${RED}❌ Not running${NC}"
fi

echo ""
echo "Client (Training):"
if pgrep -f "train.py" > /dev/null; then
    echo -e "   ${GREEN}✅ Running${NC}"
else
    echo -e "   ${RED}❌ Not running${NC}"
fi

echo ""
echo "System:"
echo "   Master Pipeline: $(pgrep -f 'master_pipeline.sh' > /dev/null && echo -e "${GREEN}✅${NC}" || echo -e "${RED}❌${NC}")"
echo "   Dashboard: $(pgrep -f 'app.py' > /dev/null && echo -e "${GREEN}✅${NC}" || echo -e "${RED}❌${NC}")"

echo ""

