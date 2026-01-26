# 🚗 CARLA RL Agent - Training Plan

## 📋 Overview

แผนการ training สำหรับ vision-based autonomous driving agent ที่ใช้ SAC algorithm

---

## 🎯 Training Phases (Progressive Difficulty)

### **Phase 1: Basic Navigation (Steps 0 - 50,000)**
**Goal:** เรียนรู้การขับรถพื้นฐานในสภาพแวดล้อมง่าย

**Configuration:**
- **Difficulty:** 0.0 - 0.3 (เริ่มง่าย)
- **Traffic:** Disabled (ไม่มีรถอื่น)
- **Pedestrians:** 0-2 คน
- **Weather:** Clear (ไม่มีฝน/หมอก)
- **Towns:** Town01_Opt, Town02 (ง่าย)
- **Goal Distance:** 100-200m (ใกล้)

**Expected Outcomes:**
- ✅ เรียนรู้การควบคุมพื้นฐาน (steering, throttle, brake)
- ✅ ไปถึง goal ได้ (success rate > 50%)
- ✅ หลีกเลี่ยงการชน (collision rate < 30%)
- ✅ Episode reward: 500-1000

**Milestones:**
- Step 10k: ไปถึง goal ได้ 30%+
- Step 25k: ไปถึง goal ได้ 50%+
- Step 50k: ไปถึง goal ได้ 70%+

---

### **Phase 2: Lane Keeping & Speed Control (Steps 50,000 - 150,000)**
**Goal:** เรียนรู้การขับในเลนและควบคุมความเร็ว

**Configuration:**
- **Difficulty:** 0.3 - 0.6
- **Traffic:** Disabled (ยังไม่มีรถอื่น)
- **Pedestrians:** 2-5 คน
- **Weather:** Clear + Light Rain (เริ่มมีสภาพอากาศ)
- **Towns:** Town01_Opt, Town02, Town03 (เพิ่มความหลากหลาย)
- **Goal Distance:** 150-300m (ไกลขึ้น)

**Expected Outcomes:**
- ✅ อยู่ในเลนได้ดี (lane keeping > 70%)
- ✅ ควบคุมความเร็วได้ (maintain 50-70 km/h)
- ✅ ไปถึง goal ได้ (success rate > 80%)
- ✅ Episode reward: 1000-1500

**Milestones:**
- Step 75k: Lane keeping > 60%
- Step 100k: Lane keeping > 70%, success rate > 80%
- Step 150k: Lane keeping > 75%, success rate > 85%

---

### **Phase 3: Obstacle Avoidance (Steps 150,000 - 300,000)**
**Goal:** เรียนรู้การหลบรถและคนเดินถนน

**Configuration:**
- **Difficulty:** 0.6 - 0.8
- **Traffic:** Enabled (5-10 คัน)
- **Pedestrians:** 5-10 คน
- **Weather:** Random (มีฝน/หมอก)
- **Towns:** All towns (Town01-Town05)
- **Goal Distance:** 200-400m

**Expected Outcomes:**
- ✅ หลบรถได้ (avoidance rate > 60%)
- ✅ หลบคนเดินถนนได้ (pedestrian avoidance > 70%)
- ✅ ยังคงไปถึง goal ได้ (success rate > 70%)
- ✅ Episode reward: 1200-1800

**Milestones:**
- Step 200k: Obstacle avoidance > 50%
- Step 250k: Obstacle avoidance > 60%, success rate > 70%
- Step 300k: Obstacle avoidance > 65%, success rate > 75%

---

### **Phase 4: Complex Scenarios (Steps 300,000 - 450,000)**
**Goal:** เรียนรู้การขับในสถานการณ์ซับซ้อน

**Configuration:**
- **Difficulty:** 0.8 - 1.0 (ยากสุด)
- **Traffic:** Enabled (10-15 คัน)
- **Pedestrians:** 10-15 คน
- **Weather:** Heavy rain, fog, night (สภาพอากาศยาก)
- **Towns:** All towns + complex routes
- **Goal Distance:** 300-500m (ไกลมาก)

**Expected Outcomes:**
- ✅ จัดการกับสภาพอากาศยากได้
- ✅ หลบอุปสรรคได้ดี (avoidance > 70%)
- ✅ ไปถึง goal ได้ในสภาพแวดล้อมยาก (success rate > 65%)
- ✅ Episode reward: 1500-2000

**Milestones:**
- Step 350k: Success rate in difficult weather > 50%
- Step 400k: Success rate > 60%, avoidance > 70%
- Step 450k: Success rate > 65%, overall performance stable

---

### **Phase 5: Fine-tuning & Generalization (Steps 450,000 - 500,000)**
**Goal:** ปรับแต่งและเพิ่มความแข็งแกร่ง

**Configuration:**
- **Difficulty:** 1.0 (ยากสุด)
- **Traffic:** Enabled (15 คัน)
- **Pedestrians:** 15 คน
- **Weather:** Full randomization (ทุกสภาพอากาศ)
- **Towns:** All towns, random routes
- **Goal Distance:** 200-500m (random)

**Expected Outcomes:**
- ✅ Performance stable ในทุกสภาพแวดล้อม
- ✅ Success rate > 70% ในทุก town
- ✅ Episode reward: 1800-2500
- ✅ Generalization ดี (ทำงานได้ในสถานการณ์ใหม่)

**Milestones:**
- Step 475k: Success rate > 70% across all towns
- Step 500k: Final evaluation, model ready for deployment

---

## 📊 Training Metrics to Monitor

### **Primary Metrics:**
1. **Episode Reward** (target: 2000+)
2. **Success Rate** (target: > 70%)
3. **Collision Rate** (target: < 20%)
4. **Lane Keeping Ratio** (target: > 75%)
5. **Goal Distance Progress** (target: 100% reached)

### **Secondary Metrics:**
1. **Obstacle Avoidance Rate** (target: > 70%)
2. **Average Speed** (target: 50-70 km/h)
3. **Smooth Steering** (target: low jerk)
4. **Episode Length** (target: 150-250 steps)

---

## 🔄 Curriculum Learning Strategy

### **Difficulty Progression:**
```
Steps 0-50k:    Difficulty 0.0 → 0.3  (ง่าย → ปานกลาง)
Steps 50k-150k: Difficulty 0.3 → 0.6  (ปานกลาง → ค่อนข้างยาก)
Steps 150k-300k: Difficulty 0.6 → 0.8 (ค่อนข้างยาก → ยาก)
Steps 300k-450k: Difficulty 0.8 → 1.0 (ยาก → ยากสุด)
Steps 450k-500k: Difficulty 1.0      (ยากสุด, fine-tuning)
```

### **Reward-Based Curriculum:**
- **Enable:** `reward_based: true`
- **Threshold:** `reward_threshold: 10.0`
- **Window:** `reward_window_size: 50`
- **Logic:** เพิ่ม difficulty เมื่อ average reward > threshold

### **Traffic & Pedestrians Progression:**
```
Phase 1: 0 vehicles, 0-2 pedestrians
Phase 2: 0 vehicles, 2-5 pedestrians
Phase 3: 5-10 vehicles, 5-10 pedestrians
Phase 4: 10-15 vehicles, 10-15 pedestrians
Phase 5: 15 vehicles, 15 pedestrians
```

---

## 🎨 Domain Randomization Strategy

### **Weather Randomization:**
- **Phase 1-2:** Clear only
- **Phase 3:** Clear + Light Rain
- **Phase 4:** All weather (rain, fog, night)
- **Phase 5:** Full randomization

### **Town Randomization:**
- **Phase 1:** Town01_Opt, Town02
- **Phase 2:** Town01_Opt, Town02, Town03
- **Phase 3-5:** All towns (Town01-Town05)

### **Vehicle Physics Randomization:**
- **Enabled:** Throughout all phases
- **Mass:** 1200-2500 kg
- **Friction:** 0.6-1.2
- **Engine Power:** 100-300 kW

---

## ⚙️ Hyperparameter Schedule

### **Learning Rate:**
- **Initial:** 0.0003 (stable)
- **Final:** 0.00005 (fine-tuning)
- **Schedule:** Linear decay over 500k steps

### **Buffer Size:**
- **Phase 1-2:** 250k (เก็บประสบการณ์หลากหลาย)
- **Phase 3-5:** 250k (maintain)

### **Training Frequency:**
- **Phase 1:** `train_freq: 4`, `gradient_steps: 1` (stable)
- **Phase 2-5:** `train_freq: 4`, `gradient_steps: 1` (maintain)

---

## 📈 Evaluation Strategy

### **Evaluation Frequency:**
- **Every:** 50,000 steps
- **Episodes:** 10 episodes per evaluation
- **Metrics:** Success rate, average reward, collision rate

### **Evaluation Scenarios:**
1. **Easy:** Town01_Opt, clear weather, no traffic
2. **Medium:** Town02, light rain, 5 vehicles
3. **Hard:** Town03, heavy rain, 10 vehicles, 10 pedestrians

---

## 🎯 Success Criteria

### **Phase Completion:**
- ✅ Average reward > phase target
- ✅ Success rate > phase target
- ✅ Collision rate < 30%
- ✅ Stable learning (no degradation)

### **Final Model:**
- ✅ Success rate > 70% across all scenarios
- ✅ Average reward > 2000
- ✅ Collision rate < 20%
- ✅ Lane keeping > 75%
- ✅ Obstacle avoidance > 70%

---

## 🔧 Configuration Updates by Phase

### **Phase 1 Config:**
```yaml
curriculum_learning:
  enabled: true
  initial_difficulty: 0.0
  max_difficulty: 0.3
  enable_traffic: false
  num_pedestrians: [0, 2]
  num_vehicles: [0, 0]

environment:
  enable_traffic: false
  num_pedestrians: 2
  num_vehicles: 0
  towns: [Town01_Opt, Town02]
```

### **Phase 2 Config:**
```yaml
curriculum_learning:
  max_difficulty: 0.6
  num_pedestrians: [2, 5]
  num_vehicles: [0, 0]

environment:
  num_pedestrians: 5
  towns: [Town01_Opt, Town02, Town03]
```

### **Phase 3 Config:**
```yaml
curriculum_learning:
  max_difficulty: 0.8
  enable_traffic: true
  num_pedestrians: [5, 10]
  num_vehicles: [5, 10]

environment:
  enable_traffic: true
  num_pedestrians: 10
  num_vehicles: 10
```

### **Phase 4-5 Config:**
```yaml
curriculum_learning:
  max_difficulty: 1.0
  num_pedestrians: [10, 15]
  num_vehicles: [10, 15]

environment:
  num_pedestrians: 15
  num_vehicles: 15
  towns: [Town01_Opt, Town02, Town03, Town04, Town05]
```

---

## 📝 Training Logs & Monitoring

### **Key Metrics to Track:**
1. **Training Logs:** `logs/sac_training_*.log`
2. **Dashboard:** Real-time monitoring at `http://localhost:5001`
3. **Checkpoints:** Every 1,000 steps
4. **Evaluation:** Every 50,000 steps

### **What to Watch:**
- ⚠️ **Reward not increasing:** อาจต้องปรับ learning rate หรือ reward shaping
- ⚠️ **High collision rate:** อาจต้องเพิ่ม penalty หรือปรับ obstacle avoidance
- ⚠️ **Low success rate:** อาจต้องลด difficulty หรือเพิ่ม training steps
- ⚠️ **Overfitting:** อาจต้องเพิ่ม domain randomization

---

## 🚀 Quick Start

1. **Start Phase 1:**
   ```bash
   # Update config for Phase 1
   # Start training
   python3 scripts/training/auto_manage.py run
   ```

2. **Monitor Progress:**
   - Dashboard: `http://localhost:5001`
   - Logs: `logs/sac_training_*.log`

3. **Move to Next Phase:**
   - เมื่อบรรลุ milestones ของ phase ปัจจุบัน
   - Update config สำหรับ phase ถัดไป
   - Continue training (resume from checkpoint)

---

## 📚 References

- **SAC Algorithm:** Soft Actor-Critic for continuous control
- **Curriculum Learning:** Progressive difficulty increase
- **Domain Randomization:** Improve generalization
- **Vision-Based RL:** ResNet18 + LSTM encoder

---

**Last Updated:** 2026-01-08
**Current Phase:** Phase 1 (Basic Navigation)
**Current Step:** ~1,000 steps
**Status:** 🟢 Training Active

