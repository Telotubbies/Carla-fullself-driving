# 💾 Disk Space Solution - Complete Guide

## 🎯 สรุปปัญหาและวิธีแก้

### ❌ **ปัญหา:** Disk Space 418GB

**สาเหตุ:**
- `rl_model_replay_buffer_2000_steps.pkl` = **418GB** (448GB)
- Replay buffer เก็บข้อมูล training ไว้มากเกินไป
- ไม่จำเป็นต้องเก็บ replay buffer ใน checkpoint

---

## ✅ วิธีแก้ไข

### 1. ลบ Replay Buffer Files

```bash
# ลบ replay buffer ไฟล์ใหญ่
rm -f checkpoints/checkpoint/*replay_buffer*.pkl

# หรือใช้ cleanup script
./scripts/cleanup_disk_space.sh
```

### 2. ป้องกันปัญหาในอนาคต

**แก้ไข config:**
```yaml
# config/sac_config.yaml
checkpoints:
  save_replay_buffer: false  # ← ไม่ต้อง save replay buffer
  save_optimizer: false      # ← ไม่ต้อง save optimizer
  max_checkpoints_to_keep: 1 # ← เก็บแค่ latest
```

---

## 🛠️ Scripts สำหรับบีบอัด/ลดขนาด

### 1. `scripts/cleanup_disk_space.sh`
**ทำอะไร:**
- ลบ incompatible checkpoints
- ลบ replay buffer files
- ลบ enhanced checkpoints
- บีบอัด log files
- ลบ old logs
- Cleanup database

**วิธีใช้:**
```bash
./scripts/cleanup_disk_space.sh
```

### 2. `scripts/compress_checkpoints.py`
**ทำอะไร:**
- บีบอัด checkpoint files ด้วย maximum compression
- ลบ checkpoint เก่า
- Optimize file sizes

**วิธีใช้:**
```bash
python3 scripts/compress_checkpoints.py
# หรือไม่บีบอัด (แค่ cleanup)
python3 scripts/compress_checkpoints.py --no-compress
```

### 3. `scripts/analyze_training_stop.py`
**ทำอะไร:**
- วิเคราะห์ training logs
- หาสาเหตุ crash
- แสดง error summary

**วิธีใช้:**
```bash
python3 scripts/analyze_training_stop.py
```

---

## 📊 Disk Space Breakdown

| Item | Size | Action | Status |
|------|------|--------|--------|
| `replay_buffer_*.pkl` | **418GB** | ✅ ลบได้ | ✅ Removed |
| `rl_model_2000_steps.zip` | 156MB | ✅ ลบได้ (incompatible) | ✅ Removed |
| `enhanced/` | 156MB | ✅ ลบได้ | ✅ Removed |
| `error_model.zip` | 156MB | ✅ ลบได้ | ✅ Removed |
| `carla.log` | 68MB | ✅ บีบอัด | ✅ Compressed |
| Old logs | 30 files | ✅ ลบได้ | ✅ Removed |
| Temp files | 3743 files | ✅ ลบได้ | ✅ Removed |

**Total Freed:** ~418GB + 156MB + 156MB + 156MB + 68MB = **~419GB**

---

## 🔍 Training Stop - Root Causes

### Cause 1: Dimension Mismatch ✅ FIXED
- **Error:** `RuntimeError: mat1 and mat2 shapes cannot be multiplied (1x7 and 5x16)`
- **Fix:** แก้ `models/custom_policy.py` → `nn.Linear(7, 16)`
- **Status:** ✅ แก้แล้ว

### Cause 2: Checkpoint Incompatible ✅ REMOVED
- **Error:** `Error(s) in loading state_dict for VisionSACPolicy`
- **Reason:** Checkpoint เก่า train ด้วย velocity=5, model ใหม่ใช้ velocity=7
- **Fix:** ลบ checkpoint เก่า
- **Status:** ✅ Removed

### Cause 3: Replay Buffer Too Large ✅ REMOVED
- **Problem:** Replay buffer 418GB
- **Fix:** ลบ replay buffer files
- **Status:** ✅ Removed

---

## 📝 Best Practices

### 1. Checkpoint Configuration
```yaml
checkpoints:
  save_replay_buffer: false  # ไม่ต้อง save
  save_optimizer: false      # ไม่ต้อง save (ประหยัด 30-50%)
  max_checkpoints_to_keep: 1 # เก็บแค่ latest
  save_freq: 2000            # Save ทุก 2k steps
```

### 2. Regular Cleanup
```bash
# Run cleanup weekly
./scripts/cleanup_disk_space.sh

# Monitor disk space
df -h .
```

### 3. Log Management
```bash
# Compress large logs
find logs -name "*.log" -size +10M ! -name "*.gz" -exec gzip {} \;

# Remove old logs
find logs -name "*.log" -mtime +7 -delete
```

---

## 🎯 Summary

### ✅ Completed:
1. ✅ แก้ dimension mismatch (model architecture)
2. ✅ ลบ incompatible checkpoint
3. ✅ ลบ replay buffer 418GB
4. ✅ Cleanup disk space
5. ✅ สร้าง cleanup scripts

### ⏭️ Next Steps:
1. Training จะ start ใหม่ (no checkpoint)
2. Auto-manager จะ restart อัตโนมัติ
3. Monitor training progress

---

**อัปเดต:** 2026-01-26  
**สถานะ:** ✅ **ALL FIXED** - Disk space freed, training ready to start

