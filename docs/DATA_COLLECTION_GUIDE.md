# Data Collection Guide - เพิ่ม Steering Diversity

## ปัญหาที่พบ
- Steering variance ต่ำมาก (0.003377)
- Non-zero steering: เพียง 4.6%
- Autopilot วิ่งตรงมาก ไม่มี steering หลากหลาย

## วิธีแก้ไขตาม Research Papers

### 1. Multiple Spawn Points (CARLA Paper, 2017)
**Strategy**: ใช้หลาย spawn points เพื่อให้ได้เส้นทางที่หลากหลาย

```bash
python3 training/collect_diverse_data.py \
    --frames 50000 \
    --spawn-points 10 \
    --frames-per-spawn 5000
```

**ผลลัพธ์**: ได้ข้อมูลจากหลายเส้นทางที่มีการเลี้ยวที่แตกต่างกัน

### 2. Data Augmentation (Bojarski et al., 2016)
**Strategy**: 
- Horizontal flip (mirror images + negate steering)
- Brightness/contrast adjustment
- Steering noise injection

```bash
python3 training/data_augmentation.py \
    --data-dir data/diverse_YYYYMMDD_HHMMSS \
    --factor 2.0 \
    --balance
```

**ผลลัพธ์**: เพิ่มความหลากหลายของ steering angles

### 3. Steering Distribution Balancing
**Strategy**: Oversample underrepresented steering angles

- Left turns (steering < -0.1): 30%
- Right turns (steering > 0.1): 30%
- Straight (|steering| <= 0.1): 40%

### 4. Traffic Manager Diversity
**Strategy**: ใช้ Traffic Manager behaviors ที่หลากหลาย
- Random aggressiveness
- Speed variations
- Ignore lights probability

## Pipeline แบบอัตโนมัติ

```bash
./collect_diverse_pipeline.sh
```

สคริปต์นี้จะ:
1. เก็บข้อมูลจาก 10 spawn points
2. Augment ข้อมูล (2x)
3. Balance steering distribution
4. Preprocess และ extract features

## References

1. **Bojarski et al. (2016)**: "End-to-End Learning for Self-Driving Cars"
   - Horizontal flip augmentation
   - Steering angle balancing

2. **Kendall et al. (2019)**: "Learning to Drive in a Day"
   - Multiple routes and spawn points
   - Diverse weather conditions

3. **Dosovitskiy et al. (2017)**: "CARLA: An Open Urban Driving Simulator"
   - Multiple spawn points
   - Route diversity

## Expected Results

หลังจากการ collect และ augment:
- **Steering variance**: > 0.1 (เพิ่มจาก 0.003)
- **Non-zero steering**: > 30% (เพิ่มจาก 4.6%)
- **High steering (|>0.3|)**: > 10%
- **Total samples**: ~100,000 (หลัง augmentation)

## Tips

1. **ใช้หลาย spawn points**: อย่างน้อย 10 จุด
2. **Augment ข้อมูล**: อย่างน้อย 2x
3. **Balance steering**: ให้มี left/right/straight ใกล้เคียงกัน
4. **ตรวจสอบ diversity**: ใช้ `check_resnet_and_data.py` หลัง collect

