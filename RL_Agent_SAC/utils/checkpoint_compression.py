"""
Checkpoint Compression Utilities
Compresses checkpoints to reduce disk space usage.
"""
import zipfile
import shutil
import logging
from pathlib import Path
from typing import Tuple, Optional
import os

logger = logging.getLogger(__name__)


def compress_checkpoint(
    checkpoint_path: str,
    compression_level: int = 9,
    compression_method: int = zipfile.ZIP_DEFLATED,
    backup: bool = True
) -> Tuple[bool, float]:
    """
    Recompress a checkpoint zip file with better compression.
    
    Args:
        checkpoint_path: Path to checkpoint zip file
        compression_level: Compression level (0-9, 9 = maximum)
        compression_method: ZIP_DEFLATED or ZIP_LZMA (requires zlib or lzma)
        backup: Create backup before compressing
    
    Returns:
        (success, space_saved_mb)
    """
    checkpoint_path = Path(checkpoint_path)
    
    if not checkpoint_path.exists():
        logger.error(f"Checkpoint not found: {checkpoint_path}")
        return False, 0.0
    
    original_size_mb = checkpoint_path.stat().st_size / (1024 * 1024)
    logger.info(f"🗜️  Compressing checkpoint: {checkpoint_path.name} ({original_size_mb:.2f} MB)")
    
    # Create backup if requested
    backup_path = None
    if backup:
        backup_path = checkpoint_path.with_suffix('.zip.backup')
        try:
            shutil.copy2(checkpoint_path, backup_path)
            logger.debug(f"Created backup: {backup_path}")
        except Exception as e:
            logger.warning(f"Failed to create backup: {e}")
    
    # Create temporary compressed file
    temp_path = checkpoint_path.with_suffix('.zip.tmp')
    
    try:
        # Read original zip
        with zipfile.ZipFile(checkpoint_path, 'r') as zip_in:
            # Check if already compressed
            all_compressed = all(
                f.compress_type == compression_method 
                for f in zip_in.filelist 
                if f.file_size > 0
            )
            
            if all_compressed:
                logger.info(f"  ⚠️  Checkpoint already compressed with method {compression_method}")
                if backup_path and backup_path.exists():
                    backup_path.unlink()
                return False, 0.0
            
            # Write compressed version
            with zipfile.ZipFile(
                temp_path, 
                'w', 
                compression_method, 
                compresslevel=compression_level
            ) as zip_out:
                for item in zip_in.infolist():
                    data = zip_in.read(item.filename)
                    # Preserve file info but use new compression
                    info = zipfile.ZipInfo(
                        filename=item.filename,
                        date_time=item.date_time
                    )
                    info.compress_type = compression_method
                    info.external_attr = item.external_attr
                    zip_out.writestr(info, data, compress_type=compression_method, compresslevel=compression_level)
        
        new_size_mb = temp_path.stat().st_size / (1024 * 1024)
        space_saved_mb = original_size_mb - new_size_mb
        
        if space_saved_mb > 0.1:  # Only replace if we save at least 0.1 MB
            # Replace original with compressed version
            shutil.move(str(temp_path), str(checkpoint_path))
            logger.info(f"  ✅ Compressed: {original_size_mb:.2f} MB → {new_size_mb:.2f} MB (saved {space_saved_mb:.2f} MB, {space_saved_mb/original_size_mb*100:.1f}%)")
            
            # Remove backup if compression successful
            if backup_path and backup_path.exists():
                backup_path.unlink()
            
            return True, space_saved_mb
        else:
            # No significant improvement, keep original
            logger.info(f"  ⚠️  No significant improvement ({space_saved_mb:.2f} MB saved), keeping original")
            temp_path.unlink()
            if backup_path and backup_path.exists():
                backup_path.unlink()
            return False, 0.0
            
    except Exception as e:
        logger.error(f"Failed to compress checkpoint: {e}", exc_info=True)
        if temp_path.exists():
            temp_path.unlink()
        if backup_path and backup_path.exists():
            logger.warning(f"Restoring from backup: {backup_path}")
            try:
                shutil.copy2(backup_path, checkpoint_path)
                backup_path.unlink()
            except Exception as restore_error:
                logger.error(f"Failed to restore from backup: {restore_error}")
        return False, 0.0


def compress_checkpoint_directory(
    checkpoint_dir: str,
    compression_level: int = 9,
    compression_method: int = zipfile.ZIP_DEFLATED
) -> Tuple[int, float]:
    """
    Compress all checkpoint files in a directory.
    
    Returns:
        (compressed_count, total_space_saved_mb)
    """
    checkpoint_dir = Path(checkpoint_dir)
    
    if not checkpoint_dir.exists():
        logger.warning(f"Checkpoint directory not found: {checkpoint_dir}")
        return 0, 0.0
    
    zip_files = list(checkpoint_dir.glob("*.zip"))
    if not zip_files:
        logger.info("No checkpoint files found")
        return 0, 0.0
    
    logger.info(f"🗜️  Compressing {len(zip_files)} checkpoint(s) in {checkpoint_dir}")
    
    compressed_count = 0
    total_saved = 0.0
    
    for zip_file in zip_files:
        success, saved = compress_checkpoint(
            str(zip_file),
            compression_level=compression_level,
            compression_method=compression_method,
            backup=True
        )
        if success:
            compressed_count += 1
            total_saved += saved
    
    if compressed_count > 0:
        logger.info(f"✅ Compressed {compressed_count}/{len(zip_files)} checkpoint(s), saved {total_saved:.2f} MB total")
    else:
        logger.info(f"⚠️  No checkpoints needed compression")
    
    return compressed_count, total_saved


def get_checkpoint_size_info(checkpoint_path: str) -> dict:
    """
    Get detailed size information about a checkpoint.
    
    Returns:
        Dictionary with size information
    """
    checkpoint_path = Path(checkpoint_path)
    
    if not checkpoint_path.exists():
        return {"error": "File not found"}
    
    info = {
        "file_path": str(checkpoint_path),
        "file_size_mb": checkpoint_path.stat().st_size / (1024 * 1024),
        "files": []
    }
    
    try:
        with zipfile.ZipFile(checkpoint_path, 'r') as z:
            total_uncompressed = sum(f.file_size for f in z.filelist)
            total_compressed = sum(f.compress_size for f in z.filelist)
            
            info["total_uncompressed_mb"] = total_uncompressed / (1024 * 1024)
            info["total_compressed_mb"] = total_compressed / (1024 * 1024)
            info["compression_ratio"] = total_compressed / total_uncompressed if total_uncompressed > 0 else 1.0
            info["space_saved_mb"] = (total_uncompressed - total_compressed) / (1024 * 1024)
            
            for f in sorted(z.filelist, key=lambda x: x.file_size, reverse=True):
                if f.file_size > 0:
                    info["files"].append({
                        "name": f.filename,
                        "uncompressed_mb": f.file_size / (1024 * 1024),
                        "compressed_mb": f.compress_size / (1024 * 1024),
                        "compression_ratio": f.compress_size / f.file_size if f.file_size > 0 else 1.0,
                        "compression_method": f.compress_type  # 0=STORED, 8=DEFLATED, 14=LZMA
                    })
    except Exception as e:
        info["error"] = str(e)
    
    return info

