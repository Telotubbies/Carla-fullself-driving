#!/usr/bin/env python3
"""Quick test to see if environment works"""
import sys
import yaml
import numpy as np
from carla_env.carla_rl_env import CarlaRLEnv
import logging

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

print("=" * 60)
print("🧪 Testing CARLA Environment")
print("=" * 60)

# Load config
print("\n[1/4] Loading config...")
with open('config/phase1_optimized.yaml', 'r') as f:
    config = yaml.safe_load(f)
print("✅ Config loaded")

# Create environment
print("\n[2/4] Creating environment...")
try:
    env = CarlaRLEnv(config)
    print("✅ Environment created")
except Exception as e:
    print(f"❌ Failed to create environment: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

# Reset
print("\n[3/4] Resetting environment...")
try:
    obs, info = env.reset()
    print(f"✅ Reset successful!")
    print(f"   Observation shape: {obs.shape}")
    print(f"   Observation dtype: {obs.dtype}")
    print(f"   Observation range: [{obs.min():.3f}, {obs.max():.3f}]")
except Exception as e:
    print(f"❌ Reset failed: {e}")
    import traceback
    traceback.print_exc()
    env.close()
    sys.exit(1)

# Test steps
print("\n[4/4] Testing steps...")
try:
    for i in range(10):
        action = np.array([0.0, 0.5, 0.0])  # No steering, 50% throttle, no brake
        print(f"   Step {i+1}...", end=" ", flush=True)
        obs, reward, done, truncated, info = env.step(action)
        print(f"✅ reward={reward:.2f}, done={done}, shape={obs.shape}")
        
        if done:
            print("   Episode done, resetting...")
            obs, info = env.reset()
    
    print("\n✅ All steps successful!")
except Exception as e:
    print(f"\n❌ Step failed: {e}")
    import traceback
    traceback.print_exc()
finally:
    env.close()
    print("\n✅ Environment closed")

print("\n" + "=" * 60)
print("✅ TEST COMPLETE")
print("=" * 60)



