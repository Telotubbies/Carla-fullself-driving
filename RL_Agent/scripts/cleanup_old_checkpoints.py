#!/usr/bin/env python3
"""
Cleanup Old Checkpoints Script
ลบ checkpoint เก่าที่ไม่ได้ใช้แล้ว เพื่อประหยัดพื้นที่

Strategy:
- เก็บ latest checkpoint (สำหรับ resume)
- เก็บ enhanced checkpoints ล่าสุด (keep_last_n)
- เก็บ IL pretrained models
- ลบ backup folders เก่า
- ลบ checkpoints ที่ไม่มี config
"""

import os
import re
import shutil
from pathlib import Path
from datetime import datetime
import argparse

# Configuration
BASE_DIR = Path(__file__).parent.parent
KEEP_LATEST_N = 5  # เก็บ checkpoint ล่าสุด N ตัว
KEEP_ENHANCED_N = 10  # เก็บ enhanced checkpoint ล่าสุด N ตัว
KEEP_BACKUP_DAYS = 7  # เก็บ backup ที่สร้างภายใน 7 วัน

def get_checkpoint_steps(checkpoint_path: Path) -> int:
    """Extract timestep from checkpoint filename"""
    match = re.search(r'rl_model_(\d+)_steps', checkpoint_path.name)
    return int(match.group(1)) if match else -1

def get_enhanced_steps(enhanced_path: Path) -> int:
    """Extract timestep from enhanced checkpoint directory name"""
    match = re.search(r'checkpoint_(\d+)', enhanced_path.name)
    return int(match.group(1)) if match else -1

def cleanup_old_checkpoints(dry_run: bool = True, keep_latest_n: int = KEEP_LATEST_N):
    """Cleanup old checkpoints, keep only latest N"""
    print("=" * 70)
    print("🧹 Checkpoint Cleanup Script")
    print("=" * 70)
    print(f"Mode: {'DRY RUN (no files will be deleted)' if dry_run else 'DELETE MODE'}")
    print(f"Keep latest {keep_latest_n} checkpoints")
    print()
    
    total_freed = 0
    deleted_count = 0
    
    # 1. Cleanup checkpoints_new/checkpoint
    checkpoint_dir = BASE_DIR / "checkpoints_new" / "checkpoint"
    if checkpoint_dir.exists():
        print(f"📂 Processing: {checkpoint_dir}")
        zips = list(checkpoint_dir.glob("rl_model_*_steps.zip"))
        
        if zips:
            # Sort by timestep
            zips.sort(key=get_checkpoint_steps, reverse=True)
            
            # Keep latest N
            to_keep = zips[:keep_latest_n]
            to_delete = zips[keep_latest_n:]
            
            print(f"   Total checkpoints: {len(zips)}")
            print(f"   Keeping: {len(to_keep)}")
            print(f"   Will delete: {len(to_delete)}")
            
            for cp in to_keep:
                steps = get_checkpoint_steps(cp)
                size = cp.stat().st_size / (1024 * 1024)  # MB
                print(f"   ✅ Keep: {cp.name} ({steps:,} steps, {size:.1f} MB)")
            
            for cp in to_delete:
                steps = get_checkpoint_steps(cp)
                size = cp.stat().st_size / (1024 * 1024)  # MB
                print(f"   ❌ Delete: {cp.name} ({steps:,} steps, {size:.1f} MB)")
                
                if not dry_run:
                    try:
                        # Delete checkpoint zip
                        cp.unlink()
                        # Delete config if exists
                        config = cp.parent / cp.name.replace(".zip", "_config.yaml")
                        if config.exists():
                            config.unlink()
                        total_freed += size
                        deleted_count += 1
                    except Exception as e:
                        print(f"      ⚠️  Error deleting {cp.name}: {e}")
        print()
    
    # 2. Cleanup enhanced checkpoints
    enhanced_dir = BASE_DIR / "checkpoints_new" / "enhanced"
    if enhanced_dir.exists():
        print(f"📂 Processing: {enhanced_dir}")
        enhanced_dirs = [d for d in enhanced_dir.iterdir() if d.is_dir() and d.name.startswith("checkpoint_")]
        
        if enhanced_dirs:
            # Sort by timestep
            enhanced_dirs.sort(key=get_enhanced_steps, reverse=True)
            
            # Keep latest N
            to_keep = enhanced_dirs[:KEEP_ENHANCED_N]
            to_delete = enhanced_dirs[KEEP_ENHANCED_N:]
            
            print(f"   Total enhanced checkpoints: {len(enhanced_dirs)}")
            print(f"   Keeping: {len(to_keep)}")
            print(f"   Will delete: {len(to_delete)}")
            
            for ed in to_keep:
                steps = get_enhanced_steps(ed)
                size = sum(f.stat().st_size for f in ed.rglob('*') if f.is_file()) / (1024 * 1024)  # MB
                print(f"   ✅ Keep: {ed.name} ({steps:,} steps, {size:.1f} MB)")
            
            for ed in to_delete:
                steps = get_enhanced_steps(ed)
                size = sum(f.stat().st_size for f in ed.rglob('*') if f.is_file()) / (1024 * 1024)  # MB
                print(f"   ❌ Delete: {ed.name} ({steps:,} steps, {size:.1f} MB)")
                
                if not dry_run:
                    try:
                        shutil.rmtree(ed)
                        total_freed += size
                        deleted_count += 1
                    except Exception as e:
                        print(f"      ⚠️  Error deleting {ed.name}: {e}")
        print()
    
    # 3. Cleanup old backup folders
    print(f"📂 Processing: Backup folders")
    backup_dirs = []
    for pattern in ["checkpoints_backup_*", "backup_*"]:
        backup_dirs.extend(BASE_DIR.glob(pattern))
    
    if backup_dirs:
        print(f"   Found {len(backup_dirs)} backup folders")
        for bd in backup_dirs:
            if bd.is_dir():
                # Check age
                mtime = bd.stat().st_mtime
                age_days = (datetime.now().timestamp() - mtime) / (24 * 3600)
                size = sum(f.stat().st_size for f in bd.rglob('*') if f.is_file()) / (1024 * 1024)  # MB
                
                if age_days > KEEP_BACKUP_DAYS:
                    print(f"   ❌ Delete: {bd.name} ({age_days:.1f} days old, {size:.1f} MB)")
                    if not dry_run:
                        try:
                            shutil.rmtree(bd)
                            total_freed += size
                            deleted_count += 1
                        except Exception as e:
                            print(f"      ⚠️  Error deleting {bd.name}: {e}")
                else:
                    print(f"   ✅ Keep: {bd.name} ({age_days:.1f} days old, {size:.1f} MB)")
    print()
    
    # 4. Cleanup checkpoints without config
    checkpoint_dir = BASE_DIR / "checkpoints_new" / "checkpoint"
    if checkpoint_dir.exists():
        print(f"📂 Processing: Checkpoints without config")
        zips = list(checkpoint_dir.glob("rl_model_*_steps.zip"))
        invalid = []
        
        for cp in zips:
            config = cp.parent / cp.name.replace(".zip", "_config.yaml")
            if not config.exists() or config.stat().st_size == 0:
                invalid.append(cp)
        
        if invalid:
            print(f"   Found {len(invalid)} checkpoints without valid config")
            for cp in invalid:
                size = cp.stat().st_size / (1024 * 1024)  # MB
                print(f"   ❌ Delete: {cp.name} (no config, {size:.1f} MB)")
                
                if not dry_run:
                    try:
                        cp.unlink()
                        total_freed += size
                        deleted_count += 1
                    except Exception as e:
                        print(f"      ⚠️  Error deleting {cp.name}: {e}")
        else:
            print(f"   ✅ All checkpoints have valid config")
    print()
    
    # Summary
    print("=" * 70)
    if dry_run:
        print("📊 DRY RUN SUMMARY")
        print("   No files were deleted")
        print("   Run with --execute to actually delete files")
    else:
        print("📊 CLEANUP SUMMARY")
        print(f"   ✅ Deleted {deleted_count} items")
        print(f"   💾 Freed {total_freed:.1f} MB ({total_freed/1024:.2f} GB)")
    print("=" * 70)

def main():
    parser = argparse.ArgumentParser(description='Cleanup old checkpoints')
    parser.add_argument('--execute', action='store_true', 
                       help='Actually delete files (default: dry run)')
    parser.add_argument('--keep-latest', type=int, default=KEEP_LATEST_N,
                       help=f'Number of latest checkpoints to keep (default: {KEEP_LATEST_N})')
    
    args = parser.parse_args()
    
    cleanup_old_checkpoints(dry_run=not args.execute, keep_latest_n=args.keep_latest)

if __name__ == '__main__':
    main()

