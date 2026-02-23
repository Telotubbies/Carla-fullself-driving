#!/usr/bin/env python3
"""
Comprehensive check for ResNet encoder and data validation.
"""

import sys
import numpy as np
import pandas as pd
from pathlib import Path
import torch
from PIL import Image

# Add project to path
sys.path.insert(0, str(Path(__file__).parent))

from perception.resnet_encoder import ResNetEncoder
from utils.device_utils import get_device

def check_resnet():
    """Check ResNet encoder."""
    print("=" * 60)
    print("ResNet Encoder Check")
    print("=" * 60)
    print()
    
    try:
        encoder = ResNetEncoder()
        print(f"✅ Encoder initialized")
        print(f"   Device: {encoder.device}")
        print(f"   Feature dim: {encoder.feature_dim}")
        print()
        
        # Test with dummy image
        dummy_img = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        features = encoder.encode(dummy_img)
        
        print(f"✅ Dummy image encoding successful")
        print(f"   Feature shape: {features.shape}")
        print(f"   Mean: {np.mean(features):.6f}")
        print(f"   Std: {np.std(features):.6f}")
        print(f"   Min: {np.min(features):.6f}")
        print(f"   Max: {np.max(features):.6f}")
        print(f"   NaN: {np.isnan(features).any()}")
        print(f"   Inf: {np.isinf(features).any()}")
        print()
        
        # Test with real image
        data_dir = Path('data/autopilot_20260208_130934')
        if data_dir.exists():
            img_files = list(data_dir.glob('images/*.png'))
            if img_files:
                test_img_path = img_files[0]
                test_img = np.array(Image.open(test_img_path))
                features_real = encoder.encode(test_img)
                
                print(f"✅ Real image encoding successful")
                print(f"   Image: {test_img_path.name}")
                print(f"   Feature shape: {features_real.shape}")
                print(f"   Mean: {np.mean(features_real):.6f}")
                print(f"   Std: {np.std(features_real):.6f}")
                print()
                
                # Compare with saved features
                features_path = data_dir / 'features.npy'
                if features_path.exists():
                    saved_features = np.load(features_path)
                    if len(saved_features) > 0:
                        first_feature = saved_features[0]
                        diff = np.abs(features_real - first_feature)
                        print(f"   Difference with saved[0]:")
                        print(f"     Mean: {np.mean(diff):.6f}")
                        print(f"     Max: {np.max(diff):.6f}")
                        if np.mean(diff) < 0.1:
                            print("     ✅ Features match!")
                        else:
                            print("     ⚠️  Features differ significantly")
        print()
        return True
    except Exception as e:
        print(f"❌ ResNet check failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def check_data():
    """Check data quality."""
    print("=" * 60)
    print("Data Quality Check")
    print("=" * 60)
    print()
    
    data_dir = Path('data/autopilot_20260208_130934')
    data_path = data_dir / 'data.csv'
    valid_path = data_dir / 'data_valid.csv'
    
    if not data_path.exists():
        print(f"❌ Data file not found: {data_path}")
        return False
    
    df = pd.read_csv(data_path)
    print(f"📊 Original data: {len(df)} rows")
    print()
    
    if valid_path.exists():
        df_valid = pd.read_csv(valid_path)
        print(f"✅ Valid data: {len(df_valid)} rows ({len(df_valid)/len(df)*100:.1f}%)")
        print()
        
        # Velocity analysis
        print("🚗 Velocity Analysis:")
        print(f"   Mean: {df_valid['velocity'].mean():.2f} m/s ({df_valid['velocity'].mean()*3.6:.1f} km/h)")
        print(f"   Median: {df_valid['velocity'].median():.2f} m/s")
        print(f"   Moving frames (>1 m/s): {(df_valid['velocity'] > 1.0).sum()} / {len(df_valid)} ({(df_valid['velocity'] > 1.0).sum()/len(df_valid)*100:.1f}%)")
        print(f"   Fast frames (>5 m/s): {(df_valid['velocity'] > 5.0).sum()} / {len(df_valid)} ({(df_valid['velocity'] > 5.0).sum()/len(df_valid)*100:.1f}%)")
        print()
        
        # Steering analysis
        print("🔄 Steering Analysis:")
        print(f"   Mean: {df_valid['steering'].mean():.6f}")
        print(f"   Std: {df_valid['steering'].std():.6f}")
        print(f"   Range: [{df_valid['steering'].min():.6f}, {df_valid['steering'].max():.6f}]")
        print(f"   Non-zero steering: {(np.abs(df_valid['steering']) > 0.01).sum()} / {len(df_valid)} ({(np.abs(df_valid['steering']) > 0.01).sum()/len(df_valid)*100:.1f}%)")
        print()
        
        # Throttle analysis
        print("⚡ Throttle Analysis:")
        print(f"   Mean: {df_valid['throttle'].mean():.3f}")
        print(f"   Active throttle (>0.1): {(df_valid['throttle'] > 0.1).sum()} / {len(df_valid)} ({(df_valid['throttle'] > 0.1).sum()/len(df_valid)*100:.1f}%)")
        print()
        
        # Distance analysis
        dx = df_valid['x'].diff()
        dy = df_valid['y'].diff()
        distance = np.sqrt(dx**2 + dy**2)
        total_distance = distance.sum()
        print("📏 Distance Analysis:")
        print(f"   Total distance: {total_distance:.2f} m")
        print(f"   Average step: {distance.mean():.4f} m")
        print()
        
        # Issues
        print("⚠️  Potential Issues:")
        issues = []
        if df_valid['velocity'].mean() < 2.0:
            issues.append(f"Low average velocity: {df_valid['velocity'].mean()*3.6:.1f} km/h")
        if df_valid['steering'].std() < 0.01:
            issues.append(f"Very low steering variance: {df_valid['steering'].std():.6f}")
        if (df_valid['velocity'] > 5.0).sum() < len(df_valid) * 0.1:
            issues.append(f"Few fast frames: only {(df_valid['velocity'] > 5.0).sum()/len(df_valid)*100:.1f}% > 5 m/s")
        if total_distance < 100:
            issues.append(f"Short total distance: {total_distance:.2f} m")
        
        if issues:
            for issue in issues:
                print(f"   - {issue}")
        else:
            print("   ✅ No major issues detected")
        print()
    
    return True

def check_features():
    """Check extracted features."""
    print("=" * 60)
    print("Feature Quality Check")
    print("=" * 60)
    print()
    
    data_dir = Path('data/autopilot_20260208_130934')
    features_path = data_dir / 'features.npy'
    
    if not features_path.exists():
        print(f"❌ Features file not found: {features_path}")
        return False
    
    features = np.load(features_path)
    print(f"✅ Features loaded: {features.shape}")
    print()
    
    print("📊 Feature Statistics:")
    print(f"   Mean: {np.mean(features):.6f}")
    print(f"   Std: {np.std(features):.6f}")
    print(f"   Min: {np.min(features):.6f}")
    print(f"   Max: {np.max(features):.6f}")
    print()
    
    print("🔍 Quality Checks:")
    nan_count = np.isnan(features).sum()
    inf_count = np.isinf(features).sum()
    zero_count = (np.abs(features) < 1e-6).sum()
    zero_ratio = zero_count / features.size
    
    print(f"   NaN: {nan_count} ({nan_count/features.size*100:.4f}%)")
    print(f"   Inf: {inf_count} ({inf_count/features.size*100:.4f}%)")
    print(f"   Near-zero: {zero_count} ({zero_ratio*100:.2f}%)")
    print()
    
    feature_std = np.std(features, axis=0)
    print(f"   Feature std per dimension:")
    print(f"     Mean: {np.mean(feature_std):.6f}")
    print(f"     Zero-variance dims: {(feature_std < 1e-6).sum()} / {features.shape[1]}")
    print()
    
    print("✅ Overall Assessment:")
    issues = []
    if nan_count > 0:
        issues.append("Contains NaN")
    if inf_count > 0:
        issues.append("Contains Inf")
    if zero_ratio > 0.1:
        issues.append(f"Too many zeros ({zero_ratio*100:.1f}%)")
    if (feature_std < 1e-6).sum() > features.shape[1] * 0.1:
        issues.append(f"Too many zero-variance dims ({(feature_std < 1e-6).sum()}/{features.shape[1]})")
    
    if issues:
        print("   ⚠️  Issues:")
        for issue in issues:
            print(f"      - {issue}")
    else:
        print("   ✅ Features look good!")
    print()
    
    return True

def main():
    """Run all checks."""
    print()
    print("🔍 Comprehensive ResNet and Data Validation Check")
    print()
    
    results = []
    results.append(("ResNet Encoder", check_resnet()))
    results.append(("Data Quality", check_data()))
    results.append(("Feature Quality", check_features()))
    
    print("=" * 60)
    print("Summary")
    print("=" * 60)
    for name, result in results:
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"{status} - {name}")
    print()

if __name__ == "__main__":
    import os
    os.environ['HSA_OVERRIDE_GFX_VERSION'] = '11.0.0'
    main()

