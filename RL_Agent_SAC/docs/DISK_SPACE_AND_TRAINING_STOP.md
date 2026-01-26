# 💾 Disk Space & Training Stop Analysis

## 🔍 สรุปปัญหา

### 1. **Disk Space Issue: 418GB Checkpoint!**

**ปัญหา:**
- `checkpoints/checkpoint/` ใช้พื้นที่ **418GB**
- มี checkpoint ไฟล์เดียว: `rl_model_2000_steps.zip` (156MB)
- แต่ directory ใช้พื้นที่ 418GB → **น่าจะมีไฟล์ซ่อนหรือ hard link**

### 2. **Training Stop Reason**

**สาเหตุหลัก:**
1. ✅ **Dimension Mismatch** (แก้แล้ว)
   - Error: `RuntimeError: mat1 and mat2 shapes cannot be multiplied (1x7 and 5x16)`
   - Model คาดหวัง velocity=5 แต่ environment ส่ง velocity=7
   - **แก้แล้ว:** `models/custom_policy.py` → `nn.Linear(7, 16)`

2. ❌ **Checkpoint Incompatible**
   - Error: `Error(s) in loading state_dict for VisionSACPolicy`
   - Checkpoint เก่า (2000 steps) ถูก train ด้วย velocity=5
   - Model ใหม่ใช้ velocity=7 → **ไม่ compatible**
   - **ต้องลบ checkpoint เก่าและ retrain ใหม่**

---

## 🛠️ วิธีแก้ไข

### 1. ลบ Checkpoint เก่า (Incompatible)

```bash
# ลบ checkpoint ที่ไม่ compatible
rm -f checkpoints/checkpoint/rl_model_2000_steps.zip

# หรือใช้ cleanup script
./scripts/cleanup_disk_space.sh
```

### 2. ลดขนาดไฟล์

#### A. ลบ Enhanced Checkpoints (Duplicate)
```bash
rm -rf checkpoints/enhanced  # ใช้พื้นที่ 156MB
```

#### B. ลบ Error Model
```bash
rm -f checkpoints/error_model.zip  # ใช้พื้นที่ 156MB
```

#### C. บีบอัด Log Files
```bash
# บีบอัด log ไฟล์ใหญ่ (>10MB)
find logs -name "*.log" -type f -size +10M ! -name "*.gz" -exec gzip {} \;
```

#### D. ลบ Log เก่า
```bash
# ลบ log เก่า 7+ วัน
find logs -name "*.log" -type f -mtime +7 -delete
```

#### E. Cleanup SQLite Database
```python
import sqlite3
conn = sqlite3.connect('checkpoints/training_checkpoints.db')
conn.execute("DELETE FROM checkpoints WHERE timestep < (SELECT MAX(timestep) FROM checkpoints)")
conn.execute("VACUUM")
conn.close()
```

### 3. ใช้ Cleanup Scripts

```bash
# Comprehensive cleanup
./scripts/cleanup_disk_space.sh

# Compress checkpoints (optional)
python3 scripts/compress_checkpoints.py

# Analyze training logs
python3 scripts/analyze_training_stop.py
```

---

## 📊 Disk Space Breakdown

| Directory/File | Size | Action |
|----------------|------|--------|
| `checkpoints/checkpoint/` | **418GB** | ⚠️ ตรวจสอบ |
| `checkpoints/enhanced/` | 156MB | ✅ ลบได้ |
| `checkpoints/error_model.zip` | 156MB | ✅ ลบได้ |
| `checkpoints/training_checkpoints.db` | 32KB | ✅ OK |
| `logs/carla.log` | 63MB | ✅ บีบอัดได้ |
| `logs/tensorboard/` | 2.7MB | ✅ OK |

---

## 🎯 Training Stop - Root Causes

### Cause 1: Dimension Mismatch ✅ FIXED
- **Error:** `RuntimeError: mat1 and mat2 shapes cannot be multiplied (1x7 and 5x16)`
- **Fix:** แก้ `models/custom_policy.py` → `nn.Linear(7, 16)`
- **Status:** ✅ แก้แล้ว

### Cause 2: Checkpoint Incompatible ❌ NEEDS ACTION
- **Error:** `Error(s) in loading state_dict for VisionSACPolicy`
- **Reason:** Checkpoint เก่า train ด้วย velocity=5, model ใหม่ใช้ velocity=7
- **Fix:** ลบ checkpoint เก่า → retrain ใหม่
- **Status:** ⏭️ ต้องทำ

---

## 📝 Next Steps

1. ✅ **Model แก้แล้ว** (velocity dimension)
2. ⏭️ **ลบ checkpoint เก่า** (incompatible)
3. ⏭️ **Cleanup disk space** (418GB issue)
4. ⏭️ **Retrain ใหม่** (start fresh)

---

## 🔧 Scripts Available

1. **`scripts/cleanup_disk_space.sh`**
   - ลบ incompatible checkpoints
   - ลบ enhanced checkpoints
   - บีบอัด logs
   - Cleanup database

2. **`scripts/compress_checkpoints.py`**
   - บีบอัด checkpoint files
   - ลบ checkpoint เก่า
   - Optimize compression

3. **`scripts/analyze_training_stop.py`**
   - วิเคราะห์ training logs
   - หาสาเหตุ crash
   - แสดง error summary

---

## ⚠️ หมายเหตุ

### Checkpoint Compatibility
- Checkpoint ที่ train ด้วย velocity=5 **ไม่สามารถใช้ได้** กับ model ใหม่
- ต้อง retrain ใหม่ทั้งหมด
- Auto-manager จะ start training ใหม่อัตโนมัติ

### Disk Space
- 418GB ใน checkpoint directory น่าจะเป็นปัญหา hard link หรือ hidden files
- ตรวจสอบด้วย: `du -sh checkpoints/checkpoint/*`
- ลบไฟล์ที่ไม่จำเป็น

---

**อัปเดต:** 2026-01-26  
**สถานะ:** 
- ✅ Model fixed
- ⏭️ Checkpoint cleanup needed
- ⏭️ Disk space cleanup needed

