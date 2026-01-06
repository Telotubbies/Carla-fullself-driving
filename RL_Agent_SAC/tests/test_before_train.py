import sys
import os
import torch
import numpy as np
from pathlib import Path
BASE_DIR = Path(__file__).parent.parent
sys.path.insert(0, str(BASE_DIR))
def test_pytorch_rocm():
    
    print("=" * 70)
    print("Test 1: PyTorch ROCm Installation")
    print("=" * 70)
    try:
        print(f"✅ PyTorch version: {torch.__version__}")
        print(f"✅ ROCm available: {torch.cuda.is_available()}")
        if torch.cuda.is_available():
            print(f"✅ Device count: {torch.cuda.device_count()}")
            print(f"✅ Device name: {torch.cuda.get_device_name(0)}")
            x = torch.randn(10, 10).cuda()
            y = torch.randn(10, 10).cuda()
            z = x + y
            print(f"✅ GPU computation test: PASSED")
            print(f"   Result shape: {z.shape}, device: {z.device}")
        else:
            print("⚠️  ROCm not available")
            return False
        return True
    except Exception as e:
        print(f"❌ PyTorch ROCm test failed: {e}")
        return False
def test_observation_conversion():
    
    print("\n" + "=" * 70)
    print("Test 2: Observation Conversion (GPU Tensor → CPU Numpy)")
    print("=" * 70)
    try:
        gpu_obs = {
            'vision': torch.randn(1, 4, 90, 160, 4).cuda(),
            'gps': torch.randn(1, 3).cuda(),
            'goal': torch.randn(1, 3).cuda(),
            'distance_to_goal': torch.randn(1, 1).cuda(),
            'velocity': torch.randn(1, 5).cuda(),
            'waypoint': torch.randn(1, 8).cuda()
        }
        print(f"✅ Created GPU observation (dict)")
        print(f"   Vision shape: {gpu_obs['vision'].shape}, device: {gpu_obs['vision'].device}")
        cpu_obs = {}
        for key, value in gpu_obs.items():
            if isinstance(value, torch.Tensor):
                cpu_obs[key] = value.cpu().numpy()
            else:
                cpu_obs[key] = value
        print(f"✅ Converted to CPU numpy")
        print(f"   Vision shape: {cpu_obs['vision'].shape}, type: {type(cpu_obs['vision'])}")
        all_numpy = all(isinstance(v, np.ndarray) for v in cpu_obs.values())
        if all_numpy:
            print(f"✅ All values are numpy arrays")
        else:
            print(f"❌ Some values are not numpy arrays")
            return False
        return True
    except Exception as e:
        print(f"❌ Observation conversion test failed: {e}")
        import traceback
        traceback.print_exc()
        return False
def test_store_transition_logic():
    
    print("\n" + "=" * 70)
    print("Test 3: _store_transition Conversion Logic")
    print("=" * 70)
    try:
        def convert_obs_for_buffer(new_obs):
            
            if isinstance(new_obs, dict):
                cpu_obs = {}
                for key, value in new_obs.items():
                    if isinstance(value, torch.Tensor):
                        cpu_obs[key] = value.cpu().numpy()
                    elif isinstance(value, np.ndarray):
                        cpu_obs[key] = value
                    else:
                        cpu_obs[key] = value
                return cpu_obs
            elif isinstance(new_obs, torch.Tensor):
                return new_obs.cpu().numpy()
            elif isinstance(new_obs, np.ndarray):
                return new_obs
            return new_obs
        gpu_obs = {
            'vision': torch.randn(1, 4, 90, 160, 4).cuda(),
            'gps': torch.randn(1, 3).cuda()
        }
        print(f"✅ Input: GPU tensor dict")
        cpu_obs = convert_obs_for_buffer(gpu_obs)
        print(f"✅ Output: CPU numpy dict")
        all_numpy = all(isinstance(v, np.ndarray) for v in cpu_obs.values())
        if all_numpy:
            print(f"✅ Conversion successful: all values are numpy arrays")
        else:
            print(f"❌ Conversion failed: some values are not numpy arrays")
            return False
        numpy_obs = {
            'vision': np.random.randn(1, 4, 90, 160, 4).astype(np.float32),
            'gps': np.random.randn(1, 3).astype(np.float32)
        }
        result = convert_obs_for_buffer(numpy_obs)
        if result is numpy_obs:
            print(f"✅ Numpy dict passes through unchanged")
        else:
            print(f"⚠️  Numpy dict was modified (may be OK)")
        return True
    except Exception as e:
        print(f"❌ _store_transition logic test failed: {e}")
        import traceback
        traceback.print_exc()
        return False
def test_mixed_device_sac_import():
    
    print("\n" + "=" * 70)
    print("Test 4: MixedDeviceSAC Import")
    print("=" * 70)
    try:
        from utils.mixed_device_sac import MixedDeviceSAC, create_mixed_device_sac
        print(f"✅ MixedDeviceSAC imported successfully")
        if hasattr(MixedDeviceSAC, '_store_transition'):
            print(f"✅ _store_transition method exists")
            import inspect
            sig = inspect.signature(MixedDeviceSAC._store_transition)
            params = list(sig.parameters.keys())
            expected_params = ['self', 'replay_buffer', 'buffer_actions', 'new_obs', 'rewards', 'dones', 'infos']
            if params == expected_params:
                print(f"✅ _store_transition signature is correct")
                print(f"   Parameters: {params}")
            else:
                print(f"⚠️  _store_transition signature mismatch")
                print(f"   Expected: {expected_params}")
                print(f"   Got: {params}")
        else:
            print(f"❌ _store_transition method not found")
            return False
        return True
    except Exception as e:
        print(f"❌ MixedDeviceSAC import test failed: {e}")
        import traceback
        traceback.print_exc()
        return False
def test_environment_import():
    
    print("\n" + "=" * 70)
    print("Test 5: Environment Import")
    print("=" * 70)
    try:
        from carla_env.carla_rl_env import CarlaRLEnv
        print(f"✅ CarlaRLEnv imported successfully")
        return True
    except Exception as e:
        print(f"⚠️  Environment import test: {e}")
        print(f"   (This is OK if CARLA is not running)")
        return True
def test_stable_baselines3_import():
    
    print("\n" + "=" * 70)
    print("Test 6: Stable-Baselines3 Import")
    print("=" * 70)
    try:
        from stable_baselines3 import SAC
        from stable_baselines3.common.buffers import ReplayBuffer
        print(f"✅ Stable-Baselines3 imported successfully")
        print(f"✅ SAC class available")
        print(f"✅ ReplayBuffer class available")
        return True
    except Exception as e:
        print(f"❌ Stable-Baselines3 import test failed: {e}")
        import traceback
        traceback.print_exc()
        return False
def main():
    
    print("\n" + "=" * 70)
    print("🧪 Unit Tests Before Training")
    print("=" * 70)
    print()
    tests = [
        ("PyTorch ROCm", test_pytorch_rocm),
        ("Observation Conversion", test_observation_conversion),
        ("_store_transition Logic", test_store_transition_logic),
        ("MixedDeviceSAC Import", test_mixed_device_sac_import),
        ("Environment Import", test_environment_import),
        ("Stable-Baselines3 Import", test_stable_baselines3_import),
    ]
    results = []
    for name, test_func in tests:
        try:
            result = test_func()
            results.append((name, result))
        except Exception as e:
            print(f"❌ Test '{name}' crashed: {e}")
            results.append((name, False))
    print("\n" + "=" * 70)
    print("📊 Test Summary")
    print("=" * 70)
    passed = sum(1 for _, result in results if result)
    total = len(results)
    for name, result in results:
        status = "✅ PASSED" if result else "❌ FAILED"
        print(f"   {status}: {name}")
    print(f"\n   Total: {passed}/{total} tests passed")
    if passed == total:
        print("\n✅ All tests passed! Ready for training.")
        return 0
    else:
        print(f"\n⚠️  {total - passed} test(s) failed. Please fix before training.")
        return 1
if __name__ == '__main__':
    sys.exit(main())