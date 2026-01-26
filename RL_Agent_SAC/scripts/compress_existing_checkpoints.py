#!/usr/bin/env python3
"""
Script to compress existing checkpoint files to reduce disk space.
Run this to compress all existing checkpoints.
"""
import sys
import logging
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from utils.checkpoint_compression import compress_checkpoint_directory, get_checkpoint_size_info

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)

def main():
    base_dir = Path(__file__).parent.parent
    checkpoint_dir = base_dir / "checkpoints" / "checkpoint"
    
    print("╔═══════════════════════════════════════════════════════════════╗")
    print("║  CHECKPOINT COMPRESSION TOOL                                  ║")
    print("╚═══════════════════════════════════════════════════════════════╝")
    print("")
    
    if not checkpoint_dir.exists():
        print(f"❌ Checkpoint directory not found: {checkpoint_dir}")
        return 1
    
    # Analyze existing checkpoints
    zip_files = list(checkpoint_dir.glob("*.zip"))
    if not zip_files:
        print("ℹ️  No checkpoint files found")
        return 0
    
    print(f"📊 Found {len(zip_files)} checkpoint file(s)")
    print("")
    
    # Show current sizes
    total_size_before = sum(f.stat().st_size for f in zip_files) / (1024 * 1024)
    print(f"📦 Total size before compression: {total_size_before:.2f} MB")
    print("")
    
    # Analyze each checkpoint
    print("🔍 Analyzing checkpoints...")
    for zip_file in zip_files:
        info = get_checkpoint_size_info(str(zip_file))
        if "error" not in info:
            print(f"  {zip_file.name}:")
            print(f"    Size: {info['file_size_mb']:.2f} MB")
            print(f"    Compression ratio: {info['compression_ratio']*100:.1f}%")
            if info['compression_ratio'] >= 0.99:
                print(f"    ⚠️  No compression (STORED format)")
            print("")
    
    # Compress checkpoints
    print("🗜️  Compressing checkpoints...")
    print("")
    compressed_count, total_saved = compress_checkpoint_directory(
        str(checkpoint_dir),
        compression_level=9,
        compression_method=8  # ZIP_DEFLATED
    )
    
    # Final stats
    zip_files_after = list(checkpoint_dir.glob("*.zip"))
    total_size_after = sum(f.stat().st_size for f in zip_files_after) / (1024 * 1024)
    
    print("")
    print("╔═══════════════════════════════════════════════════════════════╗")
    print("║  COMPRESSION SUMMARY                                         ║")
    print("╚═══════════════════════════════════════════════════════════════╝")
    print(f"  Before: {total_size_before:.2f} MB")
    print(f"  After:  {total_size_after:.2f} MB")
    print(f"  Saved:  {total_saved:.2f} MB ({total_saved/total_size_before*100:.1f}%)")
    print(f"  Files compressed: {compressed_count}/{len(zip_files)}")
    print("")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())

