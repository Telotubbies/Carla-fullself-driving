#!/usr/bin/env python3
"""
Test CARLA RL Environment
Quick test to verify environment setup
"""

import sys
import yaml
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from carla_env.carla_rl_env import CarlaRLEnv


def test_environment():
    """Test CARLA RL environment"""
    print("=" * 60)
    print("Testing CARLA RL Environment")
    print("=" * 60)
    
    # Load config
    config_path = Path(__file__).parent.parent / 'config' / 'phase1_config.yaml'
    
    if not config_path.exists():
        print(f"❌ Config file not found: {config_path}")
        return False
    
    print(f"\n[1/4] Loading configuration...")
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    print("✅ Configuration loaded")
    
    print(f"\n[2/4] Creating environment...")
    try:
        env = CarlaRLEnv(config)
        print("✅ Environment created")
        print(f"   Observation space: {env.observation_space}")
        print(f"   Action space: {env.action_space}")
    except Exception as e:
        print(f"❌ Failed to create environment: {e}")
        return False
    
    print(f"\n[3/4] Testing reset...")
    try:
        obs, info = env.reset()
        print(f"✅ Environment reset successful")
        print(f"   Observation shape: {obs.shape}")
        print(f"   Observation dtype: {obs.dtype}")
        print(f"   Observation range: [{obs.min():.3f}, {obs.max():.3f}]")
    except Exception as e:
        print(f"❌ Reset failed: {e}")
        import traceback
        traceback.print_exc()
        env.close()
        return False
    
    print(f"\n[4/4] Testing step...")
    try:
        # Random action
        import numpy as np
        action = env.action_space.sample()
        print(f"   Action: {action}")
        
        obs, reward, done, truncated, info = env.step(action)
        print(f"✅ Step successful")
        print(f"   Reward: {reward:.3f}")
        print(f"   Done: {done}")
        print(f"   Info: {info}")
    except Exception as e:
        print(f"❌ Step failed: {e}")
        import traceback
        traceback.print_exc()
        env.close()
        return False
    
    # Cleanup
    print(f"\n[Cleanup] Closing environment...")
    env.close()
    print("✅ Environment closed")
    
    print("\n" + "=" * 60)
    print("✅ All tests passed!")
    print("=" * 60)
    return True


if __name__ == '__main__':
    success = test_environment()
    sys.exit(0 if success else 1)

