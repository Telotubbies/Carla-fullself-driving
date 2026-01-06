import re
import sys
from pathlib import Path
def extract_training_stats(log_file: str):
    
    with open(log_file, 'r') as f:
        lines = f.readlines()
    print("=" * 70)
    print("📊 สถิติการฝึกอบรม: รถ อุปสรรค ความเร็ว")
    print("=" * 70)
    print()
    print("🚗 การตั้งค่า Traffic:")
    traffic_spawns = [l for l in lines if 'Starting traffic spawn' in l]
    if traffic_spawns:
        last_spawn = traffic_spawns[-1]
        match = re.search(r'(\d+) vehicles, (\d+) pedestrians', last_spawn)
        if match:
            num_vehicles = match.group(1)
            num_pedestrians = match.group(2)
            print(f"   ✅ รถอื่น (Traffic Vehicles): {num_vehicles} คัน")
            print(f"   ✅ คนเดินถนน (Pedestrians): {num_pedestrians} คน")
            print(f"   ✅ Traffic Manager: เปิดใช้งาน (รถเคลื่อนที่)")
    else:
        print("   ⚠️  ไม่พบข้อมูล traffic spawn")
    print()
    resets = [l for l in lines if 'Environment reset: Starting reset' in l]
    print(f"📈 Episodes: {len(resets)} episodes")
    print()
    callback_lines = [l for l in lines if 'Callback: Step' in l]
    if callback_lines:
        last_callback = callback_lines[-1]
        match = re.search(r'Callback: Step (\d+)', last_callback)
        if match:
            step = int(match.group(1))
            print(f"⏱️  Training Progress:")
            print(f"   Callback Step: {step}")
            if step >= 2048:
                print(f"   ✅ ถึง n_steps=2048 แล้ว! กำลัง optimize...")
            else:
                remaining = 2048 - step
                print(f"   ⏳ เหลืออีก {remaining} steps ก่อน optimize")
    print()
    print("📊 Metrics จาก Step ล่าสุด:")
    step_lines = [l for l in lines if 'Step' in l and 'Completed' in l]
    if step_lines:
        print(f"   Total steps: {len(step_lines)}")
        recent_rewards = []
        for line in step_lines[-10:]:
            match = re.search(r'reward=([\d.]+)', line)
            if match:
                recent_rewards.append(float(match.group(1)))
        if recent_rewards:
            avg_reward = sum(recent_rewards) / len(recent_rewards)
            min_reward = min(recent_rewards)
            max_reward = max(recent_rewards)
            print(f"   Reward (10 steps ล่าสุด):")
            print(f"      เฉลี่ย: {avg_reward:.2f}")
            print(f"      ต่ำสุด: {min_reward:.2f}")
            print(f"      สูงสุด: {max_reward:.2f}")
    print()
    print("📋 Info Metrics ที่ track:")
    info_lines = [l for l in lines if 'Info available:' in l]
    if info_lines:
        last_info = info_lines[-1]
        match = re.search(r"Info available: \[(.*?)\]", last_info)
        if match:
            metrics = match.group(1).replace("'", "").split(", ")
            print(f"   ✅ {len(metrics)} metrics:")
            for metric in metrics:
                print(f"      - {metric}")
    print()
    collisions = [l for l in lines if 'collision' in l.lower() and ('penalty' in l.lower() or 'Collision' in l)]
    if collisions:
        print(f"⚠️  Collisions: พบ {len(collisions)} ครั้ง")
    else:
        print("✅ Collisions: ไม่พบการชน")
    print()
    vehicle_types = [l for l in lines if 'Randomized vehicle:' in l or 'Agent vehicle spawned:' in l]
    if vehicle_types:
        print("🚙 Vehicle Types ที่ใช้:")
        unique_vehicles = set()
        for line in vehicle_types[-10:]:
            match = re.search(r'vehicle\.([\w.]+)', line)
            if match:
                unique_vehicles.add(match.group(1))
        for vtype in unique_vehicles:
            print(f"   - {vtype}")
    print()
    print("=" * 70)
    print("💡 หมายเหตุ:")
    print("   - Speed, collision, distance ถูก track ใน info dict")
    print("   - ดูค่าจริงได้จาก TensorBoard หรือ log callback")
    print("   - Config: num_vehicles=10, num_pedestrians=5, enable_traffic=true")
    print("=" * 70)
if __name__ == "__main__":
    if len(sys.argv) > 1:
        log_file = sys.argv[1]
    else:
        log_dir = Path(__file__).parent.parent / "logs"
        log_files = sorted(log_dir.glob("training_*.log"), key=lambda x: x.stat().st_mtime, reverse=True)
        if log_files:
            log_file = str(log_files[0])
            print(f"📝 ใช้ log file: {log_file}\n")
        else:
            print("❌ ไม่พบ log file")
            sys.exit(1)
    extract_training_stats(log_file)