# คู่มือเริ่มต้นใช้งาน (ภาษาไทย)

## สำหรับ AMD 7800XT (ROCm) และ PostgreSQL

## การติดตั้ง

### 1. ติดตั้ง PyTorch with ROCm

```bash
./install_rocm.sh
```

หรือติดตั้งด้วยตนเอง:
```bash
pip install torch torchvision --index-url https://download.pytorch.org/whl/rocm5.7
```

### 2. ติดตั้ง Dependencies

```bash
pip install -r requirements.txt
```

### 3. ติดตั้ง PostgreSQL (Optional)

ถ้าต้องการใช้ Database:

```bash
# ติดตั้ง PostgreSQL
sudo apt-get install postgresql postgresql-contrib
sudo systemctl start postgresql

# สร้าง database
sudo -u postgres psql
CREATE DATABASE carla_training;
CREATE USER carla_user WITH PASSWORD 'your_password';
GRANT ALL PRIVILEGES ON DATABASE carla_training TO carla_user;
\q

# ติดตั้ง Python libraries
pip install psycopg2-binary SQLAlchemy
```

## การใช้งาน

### วิธีที่ 1: ใช้สคริปต์เดียว (แนะนำ)

```bash
# รันทุกอย่างในสคริปต์เดียว
./run_all.sh inference

# หรือสำหรับ data collection
./run_all.sh collect
```

สคริปต์จะทำ:
1. ✅ ตรวจสอบ dependencies
2. ✅ Setup environment
3. ✅ เริ่ม CARLA อัตโนมัติ
4. ✅ รันระบบ

### วิธีที่ 2: รันแยกขั้นตอน

```bash
# 1. เริ่ม CARLA (ใน terminal แยก)
cd /home/a/Desktop/CARLA_0.9.16
./CarlaUE4.sh

# 2. รันระบบ (ใน terminal อีกอัน)
cd /home/a/Desktop/CARLA_0.9.16/carla_lstm_mpc_project
python main.py --mode inference
```

## PostgreSQL Database

### เปิดใช้งาน Database

แก้ไข `config.yaml`:

```yaml
database:
  enabled: true
  postgresql:
    host: "localhost"
    port: 5432
    database: "carla_training"
    user: "carla_user"
    password: "your_password"
```

### ข้อดีของ Database
- ✅ เก็บข้อมูลได้มาก (ไม่จำกัดขนาดไฟล์)
- ✅ Query ข้อมูลได้ง่าย
- ✅ Backup/Restore ง่าย
- ✅ รองรับ concurrent access

### ข้อเสีย
- ❌ ต้องติดตั้งและดูแล PostgreSQL
- ❌ ใช้ทรัพยากรมากกว่า CSV
- ❌ ต้องตั้งค่าเพิ่มเติม

### คำแนะนำ
- **ใช้ Database** ถ้าต้องการเก็บข้อมูลจำนวนมาก (>100GB) หรือต้องการ query ข้อมูล
- **ใช้ CSV** ถ้าข้อมูลไม่มาก (<10GB) หรือต้องการความเรียบง่าย

## ตรวจสอบ ROCm

```bash
# ตรวจสอบ GPU
rocm-smi

# ตรวจสอบ PyTorch
python3 -c "import torch; print('ROCm:', hasattr(torch.version, 'hip')); print('GPU:', torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'None')"
```

## Troubleshooting

### ROCm ไม่ทำงาน
```bash
# ตรวจสอบ ROCm
rocm-smi

# ตรวจสอบ environment
echo $HSA_OVERRIDE_GFX_VERSION

# Reinstall PyTorch
./install_rocm.sh
```

### Database connection failed
```bash
# ตรวจสอบ PostgreSQL
sudo systemctl status postgresql

# ตรวจสอบ connection
psql -h localhost -U carla_user -d carla_training
```

### CARLA connection failed
```bash
# ตรวจสอบ CARLA
pgrep -f CarlaUE4

# ตรวจสอบ port
netstat -tuln | grep 2000
```

## Performance Tips

1. **ใช้ GPU**: ตรวจสอบว่าใช้ ROCm/CUDA
2. **ลด Resolution**: ลด camera resolution ถ้า FPS ต่ำ
3. **ปิด Visualization**: ปิด graphs ถ้าต้องการ performance สูงสุด
4. **Database Batch**: ใช้ batch save สำหรับ database

## ตัวอย่างการใช้งาน

### Inference Mode
```bash
./run_all.sh inference
```

### Data Collection (with Database)
```bash
# เปิด database ใน config.yaml ก่อน
./run_all.sh collect
```

### Data Collection (CSV only)
```bash
# database.enabled: false ใน config.yaml
./run_all.sh collect
```

## สรุป

- ✅ **ROCm Support**: รองรับ AMD 7800XT อัตโนมัติ
- ✅ **PostgreSQL**: Optional, เปิด/ปิดได้ใน config
- ✅ **One Script**: `run_all.sh` รันทุกอย่างอัตโนมัติ
- ✅ **Auto Detection**: ตรวจจับ GPU อัตโนมัติ (ROCm > CUDA > CPU)

