# ROCm Support สำหรับ AMD 7800XT

## การติดตั้ง PyTorch with ROCm

### วิธีที่ 1: ใช้สคริปต์อัตโนมัติ

```bash
./install_rocm.sh
```

### วิธีที่ 2: ติดตั้งด้วยตนเอง

```bash
# ตรวจสอบว่า ROCm ติดตั้งแล้ว
ls /opt/rocm

# ติดตั้ง PyTorch with ROCm
pip uninstall torch torchvision
pip install torch torchvision --index-url https://download.pytorch.org/whl/rocm5.7
```

### ตรวจสอบการติดตั้ง

```bash
python3 -c "import torch; print('ROCm:', hasattr(torch.version, 'hip')); print('GPU:', torch.cuda.get_device_name(0) if torch.cuda.is_available() else 'None')"
```

## การใช้งาน

ระบบจะตรวจจับ ROCm อัตโนมัติและใช้ GPU สำหรับ:
- ResNet feature extraction
- LSTM prediction
- Training (ถ้ามี)

## ตรวจสอบ Performance

```bash
# ดู GPU usage
rocm-smi

# ดู PyTorch device
python3 -c "from utils.device_utils import get_device_info; import json; print(json.dumps(get_device_info(), indent=2))"
```

## Troubleshooting

### ROCm ไม่ถูกตรวจจับ
- ตรวจสอบว่า ROCm ติดตั้งแล้ว: `rocm-smi`
- ตรวจสอบ PyTorch: `python3 -c "import torch; print(torch.version.hip)"`
- ตรวจสอบ environment variables: `echo $HSA_OVERRIDE_GFX_VERSION`

### Performance ช้า
- ตรวจสอบว่าใช้ GPU: `rocm-smi` ควรแสดง activity
- ลด batch size หรือ resolution
- ตรวจสอบ memory: `rocm-smi --showmeminfo`

## PostgreSQL Database (Optional)

### ติดตั้ง PostgreSQL

```bash
# Ubuntu/Debian
sudo apt-get install postgresql postgresql-contrib
sudo systemctl start postgresql

# สร้าง database
sudo -u postgres psql
CREATE DATABASE carla_training;
CREATE USER carla_user WITH PASSWORD 'your_password';
GRANT ALL PRIVILEGES ON DATABASE carla_training TO carla_user;
\q
```

### ติดตั้ง Python libraries

```bash
pip install psycopg2-binary SQLAlchemy
```

### เปิดใช้งานใน config.yaml

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
- เก็บข้อมูลได้มากกว่า CSV
- Query และ filter ข้อมูลได้ง่าย
- Backup และ restore ง่าย
- รองรับ concurrent access

### ข้อเสีย
- ต้องติดตั้งและดูแล PostgreSQL
- ใช้ทรัพยากรมากกว่า CSV
- ต้องตั้งค่าเพิ่มเติม

## คำแนะนำ

**สำหรับ Training:**
- ใช้ Database ถ้าต้องการเก็บข้อมูลจำนวนมาก (>100GB)
- ใช้ CSV ถ้าข้อมูลไม่มาก (<10GB)

**สำหรับ Inference:**
- ไม่จำเป็นต้องใช้ Database

