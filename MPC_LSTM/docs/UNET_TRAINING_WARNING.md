# ⚠️ คำเตือน: U-Net Training ต้องมี Labels

## ❌ ปัญหา: Train โดยไม่มี Label

ถ้า train U-Net โดย**ไม่มี lane labels** (หรือใช้ zero masks) → **จะจับขอบถนนไม่ได้**

### ทำไม?

ดู code ใน `training/train_lane_unet.py`:

```python
# Load mask
mask_path = self.masks_dir / f"{img_path.stem}_lane.png"
if mask_path.exists():
    mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
else:
    mask = np.zeros((256, 256), dtype=np.uint8)  # ⚠️ ใช้ zero mask!
```

**ผลลัพธ์:**
- Model จะเรียนรู้ว่า "ทุกอย่างเป็น background"
- Output จะเป็น background เสมอ
- **ไม่สามารถจับขอบถนนได้**

---

## ✅ วิธีแก้: ต้องสร้าง Labels ก่อน

### Step 1: สร้าง Lane Labels จาก CARLA

```bash
# ต้องมี CARLA กำลังรันอยู่
python3 training/create_lane_labels.py \
    --images-dir data/autopilot_XXX/images \
    --output-dir data/autopilot_XXX/lane_masks \
    --carla-host localhost \
    --carla-port 2000
```

**จะสร้าง:**
- `lane_masks/image_000000_lane.png`
- `lane_masks/image_000001_lane.png`
- ... (lane masks จาก CARLA map API)

### Step 2: Train U-Net ด้วย Labels

```bash
python3 training/train_lane_unet.py \
    --images-dir data/autopilot_XXX/images \
    --masks-dir data/autopilot_XXX/lane_masks \
    --epochs 20
```

**ตอนนี้ model จะเรียนรู้ได้จริง!**

---

## 🔍 ตรวจสอบว่ามี Labels หรือไม่

```bash
# ตรวจสอบว่ามี lane masks
ls data/autopilot_XXX/lane_masks/*_lane.png | wc -l

# ถ้าได้ 0 → ต้องสร้าง labels ก่อน!
```

---

## 📊 เปรียบเทียบ

| สถานการณ์ | มี Labels? | ผลลัพธ์ |
|----------|-----------|---------|
| Train โดยไม่มี labels | ❌ | Model output เป็น background เสมอ → **จับขอบถนนไม่ได้** |
| Train ด้วย CARLA labels | ✅ | Model เรียนรู้ lane patterns → **จับขอบถนนได้** |
| Train ด้วย edge detection | ⚠️ | อาจได้บ้าง แต่ไม่แม่นเท่า CARLA labels |

---

## 💡 ทางเลือกอื่น (ถ้าไม่มี CARLA)

### Option 1: ใช้ Edge Detection เป็น Pseudo-Labels

```python
# สร้าง pseudo-labels จาก edge detection
import cv2

for img_path in image_files:
    img = cv2.imread(str(img_path))
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    edges = cv2.Canny(gray, 50, 150)
    
    # Save as pseudo-label
    cv2.imwrite(f"{mask_dir}/{img_path.stem}_lane.png", edges)
```

**ข้อเสีย:** ไม่แม่นเท่า CARLA labels

### Option 2: ใช้ Unsupervised Learning

- Self-supervised learning
- Contrastive learning
- แต่ซับซ้อนกว่า และผลลัพธ์อาจไม่ดีเท่า supervised

---

## ✅ สรุป

**U-Net จะจับขอบถนนได้จริง ก็ต่อเมื่อ:**
1. ✅ มี lane labels จาก CARLA
2. ✅ Train ด้วย labels เหล่านั้น
3. ✅ Model เรียนรู้ lane patterns

**ถ้า train โดยไม่มี labels → จะจับขอบถนนไม่ได้!**

---

## 🚀 Quick Fix

```bash
# 1. ตรวจสอบว่ามี labels หรือไม่
ls data/autopilot_XXX/lane_masks/*_lane.png | wc -l

# 2. ถ้าได้ 0 → สร้าง labels
python3 training/create_lane_labels.py \
    --images-dir data/autopilot_XXX/images \
    --output-dir data/autopilot_XXX/lane_masks

# 3. Train U-Net
python3 training/train_lane_unet.py \
    --images-dir data/autopilot_XXX/images \
    --masks-dir data/autopilot_XXX/lane_masks
```

