# 🚀 Quick Start - Auto Pipeline

## วิธีที่ง่ายที่สุด (Auto ทั้งหมด)

```bash
./run_full_auto_pipeline.sh
```

**จะทำอะไร:**
1. ✅ ตรวจสอบ CARLA (ถ้าไม่มีจะเริ่มให้)
2. ✅ เก็บ Data (20,000 frames)
3. ✅ Preprocess data
4. ✅ Extract features (ResNet)
5. ✅ Train LSTM (50 epochs)
6. ✅ Update config
7. ✅ Run Inference with GUI

**เวลา:** ~30-60 นาที (ขึ้นกับ GPU)

---

## วิธีที่ 2 (Auto Complete - ข้ามขั้นตอนที่ทำแล้ว)

```bash
./scripts/data_collection/run_complete_auto.sh
```

**จะทำอะไร:**
- ✅ ตรวจสอบว่ามี data แล้วหรือยัง
- ✅ ตรวจสอบว่ามี features แล้วหรือยัง
- ✅ ตรวจสอบว่ามี model แล้วหรือยัง
- ✅ ถ้ามีแล้วจะข้าม ถ้าไม่มีจะทำให้
- ✅ สุดท้ายรัน inference

**เหมาะสำหรับ:** ถ้ามี data แล้ว แต่ยังไม่ได้ train

---

## วิธีที่ 3 (แยกขั้นตอน)

```bash
# 1. Collect data
python3 training/collect_autopilot_data.py --frames 20000

# 2. Preprocess
python3 -c "from training.data_preprocessing import preprocess_dataset; preprocess_dataset('data/autopilot_XXX')"

# 3. Extract features
python3 training/extract_features.py --data-dir data/autopilot_XXX

# 4. Train LSTM
python3 training/train_lstm.py --data-dir data/autopilot_XXX --epochs 50

# 5. Run inference
python3 main.py --mode inference
```

---

## สรุป

**แนะนำ:** ใช้ `./run_full_auto_pipeline.sh` 
- รันทั้งหมดอัตโนมัติ
- ไม่ต้องทำอะไรเลย
- รอจนเสร็จ
