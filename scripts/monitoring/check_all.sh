#!/bin/bash
# Complete system check script

GREEN='\033[0;32m'
BLUE='\033[0;34m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m'

cd "$(dirname "$0")"

echo -e "${BLUE}════════════════════════════════════════${NC}"
echo -e "${BLUE}🔍 Complete System Check${NC}"
echo -e "${BLUE}════════════════════════════════════════${NC}"
echo ""

# 1. Processes
echo -e "${BLUE}1️⃣  PROCESSES${NC}"
echo "────────────────────────────────────────"
TRAIN=$(pgrep -f "train_lstm.py" > /dev/null && echo -e "${GREEN}✅ Running${NC}" || echo -e "${RED}❌ Stopped${NC}")
CARLA=$(pgrep -f "CarlaUE4" > /dev/null && echo -e "${GREEN}✅ Running${NC}" || echo -e "${RED}❌ Stopped${NC}")
INF=$(pgrep -f "main.py.*inference" > /dev/null && echo -e "${GREEN}✅ Running${NC}" || echo -e "${RED}❌ Stopped${NC}")
echo "Training:   $TRAIN"
echo "CARLA:      $CARLA"
echo "Inference:  $INF"
echo ""

# 2. Data & Model
echo -e "${BLUE}2️⃣  DATA & MODEL${NC}"
echo "────────────────────────────────────────"
LATEST_DATA=$(ls -td data/autopilot_* 2>/dev/null | head -1)
if [ -n "$LATEST_DATA" ]; then
    echo "Latest: $LATEST_DATA"
    [ -f "$LATEST_DATA/data.csv" ] && echo -e "${GREEN}✅ Data CSV${NC}" || echo -e "${RED}❌ Data CSV missing${NC}"
    [ -f "$LATEST_DATA/features.npy" ] && echo -e "${GREEN}✅ Features${NC}" || echo -e "${RED}❌ Features missing${NC}"
    [ -f "$LATEST_DATA/processed/data_processed.csv" ] && echo -e "${GREEN}✅ Processed data${NC}" || echo -e "${RED}❌ Processed data missing${NC}"
    [ -f "$LATEST_DATA/lstm_model/best_model.pth" ] && echo -e "${GREEN}✅ Trained model${NC}" || echo -e "${RED}❌ Model missing${NC}"
else
    echo -e "${RED}❌ No data directory${NC}"
fi
echo ""

# 3. Config
echo -e "${BLUE}3️⃣  CONFIG${NC}"
echo "────────────────────────────────────────"
python3 << 'EOF'
import yaml
try:
    with open('config.yaml', 'r') as f:
        config = yaml.safe_load(f)
    
    model_path = config.get('temporal', {}).get('trained_model_path')
    if model_path:
        import os
        if os.path.exists(model_path):
            print(f"✅ Model path: {model_path}")
        else:
            print(f"❌ Model path set but file missing: {model_path}")
    else:
        print("❌ Model path not set")
    
    print(f"✅ Town: {config.get('carla', {}).get('town', 'N/A')}")
    print(f"✅ Vehicle: {config.get('carla', {}).get('vehicle', 'N/A')}")
except Exception as e:
    print(f"❌ Error: {e}")
EOF
echo ""

# 4. CARLA Connection
echo -e "${BLUE}4️⃣  CARLA CONNECTION${NC}"
echo "────────────────────────────────────────"
timeout 3 python3 << 'EOF' 2>/dev/null
import carla
try:
    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)
    world = client.get_world()
    print("✅ Connected")
    print(f"   World: {world.get_map().name}")
    print(f"   Actors: {len(world.get_actors())}")
except Exception as e:
    print(f"❌ Connection failed: {e}")
EOF
echo ""

# 5. GPU
echo -e "${BLUE}5️⃣  GPU${NC}"
echo "────────────────────────────────────────"
python3 << 'EOF'
import torch
try:
    if torch.cuda.is_available():
        if hasattr(torch.version, 'hip') and torch.version.hip:
            print("✅ ROCm")
            print(f"   Device: {torch.cuda.get_device_name(0)}")
        else:
            print("✅ CUDA")
            print(f"   Device: {torch.cuda.get_device_name(0)}")
    else:
        print("⚠️  CPU only")
except Exception as e:
    print(f"❌ Error: {e}")
EOF
echo ""

# 6. Recent Logs
echo -e "${BLUE}6️⃣  RECENT ACTIVITY${NC}"
echo "────────────────────────────────────────"
if [ -f "inference_log.txt" ]; then
    echo "Last control command:"
    tail -1 inference_log.txt | grep -o "Control:.*" || tail -1 inference_log.txt
else
    echo "No inference log"
fi
echo ""

# Summary
echo -e "${BLUE}════════════════════════════════════════${NC}"
echo -e "${BLUE}📊 SUMMARY${NC}"
echo -e "${BLUE}════════════════════════════════════════${NC}"

ALL_OK=true
[ -z "$(pgrep -f 'CarlaUE4')" ] && ALL_OK=false
[ -z "$(pgrep -f 'main.py.*inference')" ] && ALL_OK=false
[ ! -f "$LATEST_DATA/lstm_model/best_model.pth" ] && ALL_OK=false

if [ "$ALL_OK" = true ]; then
    echo -e "${GREEN}✅ All systems operational!${NC}"
else
    echo -e "${YELLOW}⚠️  Some issues detected${NC}"
fi
echo ""


