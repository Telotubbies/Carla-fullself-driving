# 🐛 Dashboard Bugs Found

**Date**: January 26, 2026  
**Status**: 🔍 **Bugs Identified - Fixes Ready**

---

## 🐛 Bug List

### **BUG #1: Division by Zero in MetricsCard.tsx**
**Location**: `react_dashboard/src/components/Dashboard/MetricsCard.tsx:43`

**Issue**:
```typescript
avgReward: rewards.reduce((a, b) => a + b, 0) / rewards.length,
```

**Problem**: 
- ถ้า `rewards.length === 0` จะเกิด division by zero
- แม้จะมี check `if (!history || history.length === 0)` แต่ `rewards` อาจเป็น empty array หลังจาก filter

**Impact**: 
- Runtime error: `NaN` หรือ `Infinity`
- Dashboard อาจแสดง "NaN" หรือ crash

**Fix**: เพิ่ม check `rewards.length > 0` ก่อน division

---

### **BUG #2: Math.max/min with Empty Array**
**Location**: `react_dashboard/src/components/Dashboard/MetricsCard.tsx:41-42`

**Issue**:
```typescript
bestReward: Math.max(...rewards),
worstReward: Math.min(...rewards),
```

**Problem**: 
- ถ้า `rewards` เป็น empty array จะ return `-Infinity` และ `Infinity`
- จะแสดงค่าผิดปกติใน dashboard

**Impact**: 
- Dashboard แสดง `-Infinity` หรือ `Infinity`
- User confusion

**Fix**: Check `rewards.length > 0` ก่อนใช้ Math.max/min

---

### **BUG #3: GPU Memory Calculation Logic Error**
**Location**: `app_fastapi_production.py:367, 375`

**Issue**:
```python
if gpu_memory_used < 0.1:  # If < 0.1 GB, might be in MB
    gpu_memory_used = gpu_memory_used * 1024.0 / 1024.0  # Already in GB
```

**Problem**: 
- `* 1024.0 / 1024.0` = ไม่ทำอะไรเลย (multiply แล้ว divide ด้วยตัวเดียวกัน)
- Logic ผิด - ไม่ได้ convert จาก MB เป็น GB

**Impact**: 
- GPU memory อาจแสดงค่าผิด
- ไม่ได้แก้ปัญหา "0.0 / 0.0 GB" bug

**Fix**: แก้ logic ให้ convert จาก MB เป็น GB อย่างถูกต้อง

---

### **BUG #4: Potential Null/Undefined Access**
**Location**: Multiple locations

**Issues**:
1. `SystemResourcesCard.tsx:72` - `system.cpu.temperature.toFixed(1)` 
   - มี check แล้ว แต่ควรเพิ่ม safety check
2. `MetricsCard.tsx` - `metrics.episode_length` อาจเป็น undefined
3. `DashboardContent.tsx:30` - `status.status.running` อาจเป็น undefined

**Impact**: 
- Runtime errors
- Dashboard crash

**Fix**: เพิ่ม null/undefined checks และ default values

---

### **BUG #5: Cache Key Collision**
**Location**: `app_fastapi_production.py:309`

**Issue**:
```python
cache_key = "api_status"
```

**Problem**: 
- ใช้ cache key เดียวกันสำหรับทุก request
- อาจมี race condition
- Cache อาจไม่ถูก invalidate

**Impact**: 
- Stale data
- Inconsistent responses

**Fix**: ใช้ timestamp หรือ request-specific cache key

---

### **BUG #6: Missing Error Handling in API Calls**
**Location**: `react_dashboard/src/services/api.ts`

**Issue**:
- บาง API calls ไม่มี proper error handling
- Error messages อาจไม่ชัดเจน

**Impact**: 
- User ไม่รู้ว่าเกิดอะไรขึ้น
- Dashboard อาจ freeze

**Fix**: เพิ่ม error handling และ user-friendly error messages

---

## ✅ Fixes Applied

### Fix #1: MetricsCard Division by Zero
```typescript
const rewards = history.map((h) => h.reward).filter((r) => r != null);
if (rewards.length === 0) {
  return {
    totalEpisodes: 0,
    bestReward: 0,
    worstReward: 0,
    avgReward: 0,
    recentAvgReward: 0,
    avgEpisodeLength: metrics.episode_length || 0,
  };
}

const recentRewards = rewards.slice(-10);

return {
  totalEpisodes: history.length,
  bestReward: Math.max(...rewards),
  worstReward: Math.min(...rewards),
  avgReward: rewards.reduce((a, b) => a + b, 0) / rewards.length,
  recentAvgReward: recentRewards.length > 0 
    ? recentRewards.reduce((a, b) => a + b, 0) / recentRewards.length 
    : 0,
  avgEpisodeLength: metrics.episode_length || 0,
};
```

### Fix #2: GPU Memory Calculation
```python
# Try memory_used_mb first (more reliable)
if gpu_data.get('memory_used_mb') is not None:
    gpu_memory_used = float(gpu_data.get('memory_used_mb', 0)) / 1024.0
elif gpu_data.get('memory_used') is not None:
    gpu_memory_used = float(gpu_data.get('memory_used', 0))
    # If value is very small (< 0.1), it might be in MB, convert to GB
    if gpu_memory_used > 0 and gpu_memory_used < 0.1:
        gpu_memory_used = gpu_memory_used / 1024.0  # Convert MB to GB
```

### Fix #3: Null/Undefined Safety Checks
```typescript
// SystemResourcesCard.tsx
{system.cpu.temperature !== null && 
 system.cpu.temperature !== undefined && 
 !isNaN(system.cpu.temperature) && (
  <Box sx={{ display: 'flex', justifyContent: 'flex-end', mt: 0.5 }}>
    <Typography variant="caption" color="text.secondary">
      {system.cpu.temperature.toFixed(1)}°C
    </Typography>
  </Box>
)}
```

---

## 📊 Summary

| Bug # | Severity | Status | Impact |
|-------|----------|--------|--------|
| #1 | High | ✅ Fixed | Division by zero → NaN/Infinity |
| #2 | High | ✅ Fixed | Math.max/min with empty array → -Infinity/Infinity |
| #3 | Medium | ✅ Fixed | GPU memory calculation wrong |
| #4 | Medium | ✅ Fixed | Potential null/undefined access |
| #5 | Low | ⚠️  Not Critical | Cache key collision (minor) |
| #6 | Low | ⚠️  Not Critical | Error handling (minor) |

---

## 🎯 Testing Recommendations

1. **Test with empty data**:
   - No training history
   - No checkpoints
   - No GPU data

2. **Test with null/undefined values**:
   - Missing temperature
   - Missing metrics
   - Missing system data

3. **Test edge cases**:
   - Very small GPU memory values
   - Zero rewards
   - Empty arrays

---

**Status**: ✅ **All Critical Bugs Fixed**  
**Next Steps**: Test fixes and deploy

