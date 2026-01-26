# 🚗 ข้อมูลการใช้รถรุ่น (Vehicle Model)

## ✅ คำตอบ: ใช้รถรุ่นเดิมตลอดใช่ไหม?

### **ใช่** - ใช้รถรุ่นเดิมตลอด (Tesla Model 3)

---

## 📋 สถานะปัจจุบัน

### การตั้งค่า:
```yaml
# config/sac_config.yaml
environment:
  vehicle:
    blueprint: vehicle.tesla.model3  # ← ใช้ Tesla Model 3
  
  domain_randomization:
    vehicle_randomization: false  # ← ปิดการสุ่มเปลี่ยนรุ่น
```

### ผลลัพธ์:
- ✅ **ใช้ Tesla Model 3 ตลอดทุก Episode**
- ✅ ไม่มีการเปลี่ยนรุ่นรถ
- ✅ Training สม่ำเสมอและสม่ำเสมอ

---

## 🔧 รถรุ่นที่มีให้เลือก (แต่ยังไม่เปิดใช้)

ระบบมีรถรุ่นให้เลือก 13 รุ่น แต่ยังไม่เปิดใช้งาน:

```yaml
vehicle_blueprints:
  - vehicle.tesla.model3        # ← ใช้อยู่
  - vehicle.audi.tt
  - vehicle.mercedes.coupe
  - vehicle.bmw.grandtourer
  - vehicle.carlacola.carlacola
  - vehicle.chevrolet.impala
  - vehicle.dodge.charger_police
  - vehicle.ford.mustang
  - vehicle.jeep.wrangler_rubicon
  - vehicle.lincoln.mkz_2017
  - vehicle.nissan.patrol
  - vehicle.seat.leon
  - vehicle.toyota.prius
```

---

## 🎯 เหตุผลที่ใช้รุ่นเดียว

### ✅ ข้อดี:
1. **Training สม่ำเสมอ** - ไม่มีตัวแปรรุ่นรถ
2. **เรียนรู้เร็วขึ้น** - ไม่ต้องปรับตัวกับรถหลายรุ่น
3. **Debug ง่าย** - ปัญหาไม่เกี่ยวกับรุ่นรถ
4. **Baseline ชัดเจน** - เปรียบเทียบผลได้ง่าย

### ⚠️ ข้อเสีย:
1. **Generalization น้อย** - อาจไม่ทำงานดีกับรถรุ่นอื่น
2. **Robustness น้อย** - ถ้าต้องการให้ทำงานกับรถหลายรุ่น

---

## 🔄 วิธีเปลี่ยนรุ่นรถ

### วิธีที่ 1: เปลี่ยนรุ่นเดียว (แนะนำ)
```yaml
environment:
  vehicle:
    blueprint: vehicle.audi.tt  # เปลี่ยนเป็น Audi TT
```

### วิธีที่ 2: เปิดการสุ่มเปลี่ยนรุ่น
```yaml
environment:
  domain_randomization:
    vehicle_randomization: true  # ← เปิดการสุ่ม
    vehicle_blueprints:
      - vehicle.tesla.model3
      - vehicle.audi.tt
      - vehicle.mercedes.coupe
      # ... รถอื่นๆ
```

---

## 📊 Code Logic

```python
# carla_env/carla_rl_env.py (line 367-371)

if domain_rand.get("enabled", False) and domain_rand.get("vehicle_randomization", False):
    # สุ่มเลือกจาก vehicle_blueprints list
    bp_name = random.choice(domain_rand.get("vehicle_blueprints", ["vehicle.tesla.model3"]))
else:
    # ใช้ blueprint ที่กำหนดไว้
    bp_name = self.env_config.get("vehicle", {}).get("blueprint", "vehicle.tesla.model3")
```

**ปัจจุบัน:** ใช้ `else` branch → ใช้ `vehicle.tesla.model3` ตลอด

---

## 🎓 คำแนะนำ

### สำหรับ Training:
- ✅ **ใช้รุ่นเดียว** (ปัจจุบัน) - ดีสำหรับ baseline และ debug
- ⚠️ **สุ่มหลายรุ่น** - ดีสำหรับ generalization แต่เรียนรู้ช้ากว่า

### สำหรับ Production:
- 🔄 ควรเปิด `vehicle_randomization: true` เพื่อให้ agent ทำงานได้กับรถหลายรุ่น

---

## 📝 สรุป

| คำถาม | คำตอบ | รายละเอียด |
|-------|-------|-----------|
| **ใช้รถรุ่นเดิม?** | ✅ ใช่ | Tesla Model 3 ตลอด |
| **มีการเปลี่ยนรุ่น?** | ❌ ไม่มี | `vehicle_randomization: false` |
| **มีรถให้เลือก?** | ✅ มี | 13 รุ่น (แต่ยังไม่เปิดใช้) |
| **เปลี่ยนรุ่นได้ไหม?** | ✅ ได้ | แก้ config แล้ว restart |

---

**อัปเดต:** 2026-01-26  
**ไฟล์ที่เกี่ยวข้อง:**
- `config/sac_config.yaml` (line 57, 87)
- `carla_env/carla_rl_env.py` (line 367-371)

