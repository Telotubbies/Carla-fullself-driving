# 🔧 GPU, CPU Temperature & System Log Fixes

## 🐛 ปัญหาที่พบ

### 1. **GPU 0.0 / 0.0 GB Bug**
- Dashboard แสดง GPU memory เป็น 0.0 / 0.0 GB
- สาเหตุ: Logic parsing GPU memory ผิดพลาด

### 2. **CPU Temperature แสดง GPU Temp**
- Dashboard แสดง GPU temperature แทน CPU temperature
- สาเหตุ: sensors command ดึง temp ทั้ง CPU และ GPU

### 3. **System Log Endpoint ขาดหาย**
- Frontend เรียก `/api/logs` แต่ไม่มี endpoint
- สาเหตุ: Production version ไม่มี endpoint นี้

---

## ✅ การแก้ไข

### 1. แก้ GPU Memory Parsing

**ไฟล์:** `app_fastapi_production.py` (line 348-366)

**ปัญหา:**
```python
# เก่า - ผิดพลาด
gpu_memory_used = gpu_data.get('memory_used', 0) or 0  # ถ้าเป็น 0 จะได้ 0
```

**แก้ไข:**
```python
# ใหม่ - ตรวจสอบ memory_used_mb ก่อน
if gpu_data.get('memory_used_mb') is not None:
    gpu_memory_used = float(gpu_data.get('memory_used_mb', 0)) / 1024.0
elif gpu_data.get('memory_used') is not None:
    gpu_memory_used = float(gpu_data.get('memory_used', 0))

# Fallback: ใช้ GPU name เพื่อ guess memory
if gpu_memory_total == 0:
    gpu_name = gpu_data.get('name', '').lower()
    if '7800 xt' in gpu_name:
        gpu_memory_total = 16.0  # 16 GB
```

### 2. แก้ CPU Temperature Detection

**ไฟล์:** `app.py` (line 291-306)

**ปัญหา:**
```python
# เก่า - ดึง temp ทั้ง CPU และ GPU
if 'Package id 0' in line or 'Tdie' in line:
```

**แก้ไข:**
```python
# ใหม่ - แยก CPU และ GPU
if ('Package id 0' in line or 'Tdie' in line or 'CPU Temperature' in line) and 'GPU' not in line.upper():
    # Validate CPU temp range (20-100°C)
    if 20 <= temp_val <= 100:
        metrics['cpu']['temp'] = temp_val
```

**ไฟล์:** `app_fastapi_production.py` (line 326-333)

**เพิ่ม CPU temperature field:**
```python
transformed_system = {
    'cpu': {
        'usage': cpu_percent,
        'temperature': round(cpu_temp, 1) if cpu_temp is not None else None,
        # ... other fields
    },
}
```

### 3. เพิ่ม System Log Endpoint

**ไฟล์:** `app_fastapi_production.py` (line 546-570)

**เพิ่ม endpoint:**
```python
@app.get("/api/logs")
@limiter.limit("10/minute")
async def api_logs(request: Request):
    """Get system log (auto_manage.log)"""
    # Returns last 500 lines of auto_manage.log
    return {
        'content': content,
        'filename': 'auto_manage.log',
        'size': file_size,
        'modified': timestamp
    }
```

---

## 📊 ผลลัพธ์

### ก่อนแก้ไข:
- ❌ GPU: 0.0 / 0.0 GB
- ❌ CPU Temp: แสดง GPU temp
- ❌ System Log: 404 Not Found

### หลังแก้ไข:
- ✅ GPU: แสดง memory จริง (เช่น 2.5 / 16.0 GB)
- ✅ CPU Temp: แสดง CPU temperature เท่านั้น
- ✅ System Log: `/api/logs` ทำงานได้

---

## 🔄 การใช้งาน

### Restart Dashboard:
```bash
# Auto-manager จะ restart อัตโนมัติ
# หรือ restart manual:
pkill -f app_fastapi_production
cd web_dashboard
source ../venv/bin/activate
python3 app_fastapi_production.py
```

### ตรวจสอบ:
1. เปิด Dashboard: http://localhost:5001
2. ตรวจสอบ GPU memory แสดงค่าจริง
3. ตรวจสอบ CPU temperature แสดงค่าจริง
4. ตรวจสอบ System Log tab ทำงานได้

---

## 📝 Files Changed

1. ✅ `web_dashboard/app_fastapi_production.py`
   - แก้ GPU memory parsing
   - เพิ่ม CPU temperature field
   - เพิ่ม `/api/logs` endpoint

2. ✅ `web_dashboard/app.py`
   - แก้ CPU temperature detection (exclude GPU)

---

## ⚠️ หมายเหตุ

### GPU Memory Fallback:
- ถ้า rocm-smi/nvidia-smi ไม่ได้ข้อมูล
- ระบบจะใช้ GPU name เพื่อ guess memory
- รองรับ: RX 7800 XT (16GB), RX 6800 (16GB), RX 6700 (12GB)

### CPU Temperature:
- ตรวจสอบ range 20-100°C เพื่อป้องกันค่าแปลก
- Exclude บรรทัดที่มี "GPU" เพื่อไม่ให้ดึง GPU temp

---

**อัปเดต:** 2026-01-26  
**สถานะ:** ✅ **FIXED** - All issues resolved

