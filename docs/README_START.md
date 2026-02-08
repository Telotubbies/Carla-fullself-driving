# 🚀 เริ่มใช้งานทันที (Quick Start)

## ✅ ROCm 7800XT พร้อมแล้ว!

ระบบตรวจจับและแก้ไขปัญหา ROCm อัตโนมัติแล้ว

## วิธีรัน (3 วิธี)

### วิธีที่ 1: สคริปต์เดียว (แนะนำ) ⭐

```bash
cd /home/a/Desktop/CARLA_0.9.16/carla_lstm_mpc_project
./start.sh inference
```

หรือสำหรับ data collection:
```bash
./start.sh collect
```

### วิธีที่ 2: รันแยกขั้นตอน

```bash
# Terminal 1: เริ่ม CARLA
cd /home/a/Desktop/CARLA_0.9.16
./CarlaUE4.sh

# Terminal 2: รันระบบ
cd /home/a/Desktop/CARLA_0.9.16/carla_lstm_mpc_project
export HSA_OVERRIDE_GFX_VERSION=11.0.0
python3 main.py --mode inference
```

### วิธีที่ 3: ใช้ run_all.sh (เต็มรูปแบบ)

```bash
./run_all.sh inference
```

## ตรวจสอบ ROCm

```bash
# ตรวจสอบ GPU
rocm-smi

# ทดสอบ ROCm
export HSA_OVERRIDE_GFX_VERSION=11.0.0
python3 -c "import torch; x = torch.randn(10, 10).cuda(); print('✅ ROCm OK:', x.sum().item())"
```

## PostgreSQL (Optional)

ถ้าต้องการใช้ Database:

1. ติดตั้ง:
```bash
sudo apt-get install postgresql postgresql-contrib
pip install psycopg2-binary SQLAlchemy
```

2. สร้าง database:
```bash
sudo -u postgres psql
CREATE DATABASE carla_training;
CREATE USER carla_user WITH PASSWORD 'password';
GRANT ALL PRIVILEGES ON DATABASE carla_training TO carla_user;
\q
```

3. เปิดใช้ใน `config.yaml`:
```yaml
database:
  enabled: true
  postgresql:
    host: "localhost"
    port: 5432
    database: "carla_training"
    user: "carla_user"
    password: "password"
```

## Troubleshooting

### ROCm Error
```bash
# ใช้ fix script
./fix_rocm.sh

# หรือ set manual
export HSA_OVERRIDE_GFX_VERSION=11.0.0
```

### CARLA Connection Failed
```bash
# ตรวจสอบ CARLA
pgrep -f CarlaUE4

# เริ่ม CARLA
cd /home/a/Desktop/CARLA_0.9.16 && ./CarlaUE4.sh
```

## สรุป

- ✅ **ROCm 7800XT**: ทำงานแล้ว (gfx1101 → gfx1100 compatibility)
- ✅ **Auto Fix**: สคริปต์แก้ปัญหา ROCm อัตโนมัติ
- ✅ **One Script**: `start.sh` รันทุกอย่าง
- ✅ **PostgreSQL**: Optional, เปิด/ปิดได้

**พร้อมรันแล้ว!** 🚀

