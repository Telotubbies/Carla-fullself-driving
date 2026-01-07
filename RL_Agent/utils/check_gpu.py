#!/usr/bin/env python3
"""
GPU Detection and Verification Script for AMD RX 7800 XT
Checks PyTorch GPU availability and ROCm compatibility
"""

import sys
import subprocess

def check_rocm_installation():
    """Check if ROCm is installed"""
    try:
        result = subprocess.run(['rocminfo'], capture_output=True, text=True)
        if result.returncode == 0:
            print("✅ ROCm is installed")
            print(result.stdout[:500])  # First 500 chars
            return True
        else:
            print("❌ ROCm not found or not working")
            return False
    except FileNotFoundError:
        print("❌ ROCm not installed (rocminfo not found)")
        return False

def check_pytorch_gpu():
    """Check PyTorch GPU availability"""
    try:
        import torch
        print(f"\n📦 PyTorch Version: {torch.__version__}")
        
        # Check if CUDA (HIP) is available
        if torch.cuda.is_available():
            print("✅ GPU is available in PyTorch")
            print(f"   Device Count: {torch.cuda.device_count()}")
            
            for i in range(torch.cuda.device_count()):
                print(f"\n   GPU {i}:")
                print(f"      Name: {torch.cuda.get_device_name(i)}")
                print(f"      Memory: {torch.cuda.get_device_properties(i).total_memory / 1e9:.2f} GB")
                print(f"      Compute Capability: {torch.cuda.get_device_properties(i).major}.{torch.cuda.get_device_properties(i).minor}")
            
            # Test tensor creation
            try:
                x = torch.randn(1000, 1000).cuda()
                y = torch.randn(1000, 1000).cuda()
                z = torch.matmul(x, y)
                print("\n✅ GPU computation test: PASSED")
                return True
            except Exception as e:
                print(f"\n❌ GPU computation test: FAILED - {e}")
                return False
        else:
            print("❌ GPU is NOT available in PyTorch")
            print("   This might be normal if using CPU-only PyTorch")
            print("   For AMD GPU support, ensure PyTorch was built with ROCm")
            return False
            
    except ImportError:
        print("❌ PyTorch not installed")
        return False

def check_amd_gpu():
    """Check for AMD GPU using system commands"""
    try:
        # Try rocm-smi first
        result = subprocess.run(['rocm-smi'], capture_output=True, text=True)
        if result.returncode == 0:
            print("\n🖥️  AMD GPU Information (rocm-smi):")
            print(result.stdout)
            return True
    except FileNotFoundError:
        pass
    
    # Fallback to lspci
    try:
        result = subprocess.run(['lspci'], capture_output=True, text=True)
        amd_gpus = [line for line in result.stdout.split('\n') if 'AMD' in line or 'Radeon' in line]
        if amd_gpus:
            print("\n🖥️  AMD GPU Information (lspci):")
            for gpu in amd_gpus:
                print(f"   {gpu}")
            return True
        else:
            print("\n⚠️  No AMD GPU found in lspci output")
            return False
    except FileNotFoundError:
        print("\n⚠️  Cannot check GPU (lspci not available)")
        return False

def main():
    print("=" * 60)
    print("AMD GPU & ROCm Compatibility Check")
    print("=" * 60)
    
    # Check system GPU
    print("\n[1/3] Checking System GPU...")
    gpu_found = check_amd_gpu()
    
    # Check ROCm
    print("\n[2/3] Checking ROCm Installation...")
    rocm_ok = check_rocm_installation()
    
    # Check PyTorch
    print("\n[3/3] Checking PyTorch GPU Support...")
    pytorch_ok = check_pytorch_gpu()
    
    # Summary
    print("\n" + "=" * 60)
    print("SUMMARY")
    print("=" * 60)
    print(f"System GPU Detected: {'✅' if gpu_found else '❌'}")
    print(f"ROCm Installed: {'✅' if rocm_ok else '❌'}")
    print(f"PyTorch GPU Available: {'✅' if pytorch_ok else '❌'}")
    
    if pytorch_ok:
        print("\n🎉 Your system is ready for RL training with AMD GPU!")
    else:
        print("\n⚠️  GPU training may not work. Check PyTorch installation.")
        print("   For AMD GPUs, you may need PyTorch built with ROCm support.")
        print("   Visit: https://pytorch.org/get-started/locally/")
    
    print("=" * 60)
    
    return pytorch_ok

if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)

