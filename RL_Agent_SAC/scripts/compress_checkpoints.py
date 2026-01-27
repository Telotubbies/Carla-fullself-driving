#!/usr/bin/env python3
"""
Compress and optimize checkpoint files to reduce disk space usage.
"""
import os
import sys
import gzip
import shutil
from pathlib import Path
from typing import List, Tuple
import logging

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

def get_file_size_mb(filepath: Path) -> float:
    """Get file size in MB"""
    return filepath.stat().st_size / (1024 * 1024)

def compress_zip_file(zip_path: Path, compression_level: int = 9) -> Tuple[bool, float]:
    """
    Recompress a zip file with higher compression level.
    Returns: (success, space_saved_mb)
    """
    try:
        original_size = get_file_size_mb(zip_path)
        
        # Create temporary compressed file
        temp_path = zip_path.with_suffix('.zip.tmp')
        
        # Use zipfile with maximum compression
        import zipfile
        with zipfile.ZipFile(zip_path, 'r') as zip_in:
            with zipfile.ZipFile(temp_path, 'w', zipfile.ZIP_DEFLATED, compresslevel=compression_level) as zip_out:
                for item in zip_in.infolist():
                    data = zip_in.read(item.filename)
                    zip_out.writestr(item, data)
        
        new_size = get_file_size_mb(temp_path)
        space_saved = original_size - new_size
        
        if space_saved > 0:
            # Replace original with compressed version
            shutil.move(str(temp_path), str(zip_path))
            return True, space_saved
        else:
            # No improvement, remove temp file
            temp_path.unlink()
            return False, 0.0
            
    except Exception as e:
        logger.error(f"Failed to compress {zip_path.name}: {e}")
        if temp_path.exists():
            temp_path.unlink()
        return False, 0.0

def cleanup_old_checkpoints(checkpoint_dir: Path, keep_latest: int = 1, min_size_mb: float = 100.0):
    """
    Remove old checkpoints, keeping only the latest N.
    Also removes checkpoints smaller than min_size_mb (likely corrupted).
    """
    zip_files = sorted(checkpoint_dir.glob("*.zip"), key=lambda x: x.stat().st_mtime, reverse=True)
    
    if len(zip_files) <= keep_latest:
        logger.info(f"Only {len(zip_files)} checkpoint(s), keeping all")
        return 0, 0.0
    
    removed_count = 0
    freed_space = 0.0
    
    # Remove old checkpoints
    for zip_file in zip_files[keep_latest:]:
        size_mb = get_file_size_mb(zip_file)
        freed_space += size_mb
        zip_file.unlink()
        removed_count += 1
        logger.info(f"Removed old checkpoint: {zip_file.name} ({size_mb:.1f} MB)")
    
    # Remove small/corrupted checkpoints
    for zip_file in zip_files[:keep_latest]:
        size_mb = get_file_size_mb(zip_file)
        if size_mb < min_size_mb:
            freed_space += size_mb
            zip_file.unlink()
            removed_count += 1
            logger.warning(f"Removed small/corrupted checkpoint: {zip_file.name} ({size_mb:.1f} MB)")
    
    return removed_count, freed_space

def optimize_checkpoint_directory(base_dir: Path, compress: bool = True, cleanup: bool = True):
    """
    Optimize checkpoint directory by compressing and cleaning up.
    """
    checkpoint_dir = base_dir / "checkpoints" / "checkpoint"
    
    if not checkpoint_dir.exists():
        logger.warning(f"Checkpoint directory not found: {checkpoint_dir}")
        return
    
    logger.info(f"🔍 Analyzing checkpoint directory: {checkpoint_dir}")
    
    zip_files = list(checkpoint_dir.glob("*.zip"))
    if not zip_files:
        logger.info("No checkpoint files found")
        return
    
    total_size_before = sum(get_file_size_mb(f) for f in zip_files)
    logger.info(f"📊 Found {len(zip_files)} checkpoint(s), total size: {total_size_before:.1f} MB")
    
    # Cleanup old checkpoints
    if cleanup:
        logger.info("🧹 Cleaning up old checkpoints...")
        removed, freed = cleanup_old_checkpoints(checkpoint_dir, keep_latest=1, min_size_mb=100.0)
        if removed > 0:
            logger.info(f"✅ Removed {removed} checkpoint(s), freed {freed:.1f} MB")
    
    # Recompress remaining checkpoints
    if compress:
        logger.info("🗜️  Recompressing checkpoints with maximum compression...")
        zip_files = list(checkpoint_dir.glob("*.zip"))
        total_saved = 0.0
        compressed_count = 0
        
        for zip_file in zip_files:
            logger.info(f"Compressing {zip_file.name}...")
            success, saved = compress_zip_file(zip_file, compression_level=9)
            if success:
                compressed_count += 1
                total_saved += saved
                logger.info(f"  ✅ Saved {saved:.1f} MB")
            else:
                logger.info(f"  ⚠️  No improvement")
        
        if compressed_count > 0:
            logger.info(f"✅ Compressed {compressed_count} file(s), saved {total_saved:.1f} MB")
    
    # Final stats
    zip_files = list(checkpoint_dir.glob("*.zip"))
    total_size_after = sum(get_file_size_mb(f) for f in zip_files)
    total_freed = total_size_before - total_size_after
    
    logger.info("")
    logger.info("📊 Summary:")
    logger.info(f"   Before: {total_size_before:.1f} MB")
    logger.info(f"   After:  {total_size_after:.1f} MB")
    logger.info(f"   Freed:  {total_freed:.1f} MB ({total_freed/total_size_before*100:.1f}%)")

def main():
    base_dir = Path(__file__).parent.parent
    compress = '--no-compress' not in sys.argv
    cleanup = '--no-cleanup' not in sys.argv
    
    logger.info("🗜️  Checkpoint Compression & Cleanup Tool")
    logger.info("=" * 60)
    
    optimize_checkpoint_directory(base_dir, compress=compress, cleanup=cleanup)
    
    logger.info("")
    logger.info("✅ Done!")

if __name__ == "__main__":
    main()

