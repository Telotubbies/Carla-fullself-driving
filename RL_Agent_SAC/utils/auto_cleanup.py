"""
Auto Cleanup Utility for Managing Disk Space
Automatically removes old logs, checkpoints, and temporary files
"""
import os
import shutil
import logging
import sqlite3
from pathlib import Path
from datetime import datetime, timedelta
from typing import Dict, List, Tuple
import psutil

logger = logging.getLogger(__name__)


class AutoCleanupManager:
    """Manages automatic cleanup of old files to free disk space"""
    
    def __init__(
        self,
        base_dir: Path,
        min_free_space_gb: float = 10.0,
        keep_latest_logs: int = 3,
        keep_latest_checkpoints: int = 1,
        log_retention_days: int = 3,
        checkpoint_retention_days: int = 7,
        cleanup_interval_hours: int = 6
    ):
        self.base_dir = Path(base_dir)
        self.min_free_space_gb = min_free_space_gb
        self.keep_latest_logs = keep_latest_logs
        self.keep_latest_checkpoints = keep_latest_checkpoints
        self.log_retention_days = log_retention_days
        self.checkpoint_retention_days = checkpoint_retention_days
        self.cleanup_interval_hours = cleanup_interval_hours
        
        self.logs_dir = self.base_dir / "logs"
        self.checkpoints_dir = self.base_dir / "checkpoints"
        self.temp_dir = self.base_dir / "temp"
        
        self.last_cleanup_time = None
    
    def get_disk_space(self) -> Dict[str, float]:
        """Get current disk space usage"""
        try:
            stat = shutil.disk_usage(self.base_dir)
            total_gb = stat.total / (1024 ** 3)
            used_gb = stat.used / (1024 ** 3)
            free_gb = stat.free / (1024 ** 3)
            used_percent = (stat.used / stat.total) * 100
            
            return {
                'total_gb': total_gb,
                'used_gb': used_gb,
                'free_gb': free_gb,
                'used_percent': used_percent
            }
        except Exception as e:
            logger.error(f"Failed to get disk space: {e}")
            return {'total_gb': 0, 'used_gb': 0, 'free_gb': 0, 'used_percent': 0}
    
    def needs_cleanup(self) -> bool:
        """Check if cleanup is needed"""
        disk_space = self.get_disk_space()
        
        # Check if free space is below threshold
        if disk_space['free_gb'] < self.min_free_space_gb:
            logger.warning(f"⚠️  Low disk space: {disk_space['free_gb']:.2f} GB free (threshold: {self.min_free_space_gb} GB)")
            return True
        
        # Check if it's time for periodic cleanup
        if self.last_cleanup_time:
            hours_since_cleanup = (datetime.now() - self.last_cleanup_time).total_seconds() / 3600
            if hours_since_cleanup >= self.cleanup_interval_hours:
                logger.info(f"⏰ Periodic cleanup due (last cleanup: {hours_since_cleanup:.1f} hours ago)")
                return True
        
        return False
    
    def cleanup_old_logs(self) -> Tuple[int, float]:
        """Remove old log files, keeping only the latest N"""
        if not self.logs_dir.exists():
            return 0, 0.0
        
        removed_count = 0
        freed_space_gb = 0.0
        
        try:
            # Get all log files sorted by modification time (newest first)
            log_files = sorted(
                self.logs_dir.glob("*.log"),
                key=lambda p: p.stat().st_mtime,
                reverse=True
            )
            
            # Keep only the latest N logs
            files_to_remove = log_files[self.keep_latest_logs:]
            
            # Also remove logs older than retention period
            cutoff_date = datetime.now() - timedelta(days=self.log_retention_days)
            for log_file in log_files:
                if log_file not in files_to_remove:
                    file_time = datetime.fromtimestamp(log_file.stat().st_mtime)
                    if file_time < cutoff_date:
                        files_to_remove.append(log_file)
            
            # Remove files
            for log_file in files_to_remove:
                try:
                    file_size_gb = log_file.stat().st_size / (1024 ** 3)
                    log_file.unlink()
                    removed_count += 1
                    freed_space_gb += file_size_gb
                    logger.debug(f"🗑️  Removed old log: {log_file.name} ({file_size_gb:.2f} GB)")
                except Exception as e:
                    logger.warning(f"Failed to remove {log_file}: {e}")
            
            if removed_count > 0:
                logger.info(f"✅ Cleaned up {removed_count} old log files, freed {freed_space_gb:.2f} GB")
        
        except Exception as e:
            logger.error(f"Error cleaning up logs: {e}")
        
        return removed_count, freed_space_gb
    
    def cleanup_old_checkpoints(self) -> Tuple[int, float]:
        """Remove old checkpoint files, keeping only the latest N"""
        removed_count = 0
        freed_space_gb = 0.0
        
        try:
            # Cleanup regular checkpoints
            checkpoint_dir = self.checkpoints_dir / "checkpoint"
            if checkpoint_dir.exists():
                # FIRST: Remove all replay buffer files (they're huge and not needed)
                replay_buffer_files = list(checkpoint_dir.glob("*replay_buffer*.pkl"))
                for replay_file in replay_buffer_files:
                    try:
                        file_size_gb = replay_file.stat().st_size / (1024 ** 3)
                        replay_file.unlink()
                        removed_count += 1
                        freed_space_gb += file_size_gb
                        logger.info(f"🗑️  Removed replay buffer: {replay_file.name} ({file_size_gb:.2f} GB)")
                    except Exception as e:
                        logger.warning(f"Failed to remove replay buffer {replay_file}: {e}")
                
                # THEN: Cleanup regular checkpoint zip files
                checkpoint_files = sorted(
                    checkpoint_dir.glob("*.zip"),
                    key=lambda p: p.stat().st_mtime,
                    reverse=True
                )
                
                files_to_remove = checkpoint_files[self.keep_latest_checkpoints:]
                
                # Also remove checkpoints older than retention period
                cutoff_date = datetime.now() - timedelta(days=self.checkpoint_retention_days)
                for checkpoint_file in checkpoint_files:
                    if checkpoint_file not in files_to_remove:
                        file_time = datetime.fromtimestamp(checkpoint_file.stat().st_mtime)
                        if file_time < cutoff_date:
                            files_to_remove.append(checkpoint_file)
                
                for checkpoint_file in files_to_remove:
                    try:
                        file_size_gb = checkpoint_file.stat().st_size / (1024 ** 3)
                        checkpoint_file.unlink()
                        removed_count += 1
                        freed_space_gb += file_size_gb
                        logger.debug(f"🗑️  Removed old checkpoint: {checkpoint_file.name} ({file_size_gb:.2f} GB)")
                    except Exception as e:
                        logger.warning(f"Failed to remove {checkpoint_file}: {e}")
            
            # Cleanup enhanced checkpoints
            enhanced_dir = self.checkpoints_dir / "enhanced"
            if enhanced_dir.exists():
                for checkpoint_folder in enhanced_dir.iterdir():
                    if checkpoint_folder.is_dir():
                        model_file = checkpoint_folder / "model.zip"
                        if model_file.exists():
                            # Check if this checkpoint is old
                            file_time = datetime.fromtimestamp(model_file.stat().st_mtime)
                            cutoff_date = datetime.now() - timedelta(days=self.checkpoint_retention_days)
                            
                            if file_time < cutoff_date:
                                try:
                                    folder_size_gb = sum(
                                        f.stat().st_size for f in checkpoint_folder.rglob('*') if f.is_file()
                                    ) / (1024 ** 3)
                                    shutil.rmtree(checkpoint_folder)
                                    removed_count += 1
                                    freed_space_gb += folder_size_gb
                                    logger.debug(f"🗑️  Removed old enhanced checkpoint: {checkpoint_folder.name} ({folder_size_gb:.2f} GB)")
                                except Exception as e:
                                    logger.warning(f"Failed to remove {checkpoint_folder}: {e}")
            
            if removed_count > 0:
                logger.info(f"✅ Cleaned up {removed_count} old checkpoint files, freed {freed_space_gb:.2f} GB")
        
        except Exception as e:
            logger.error(f"Error cleaning up checkpoints: {e}")
        
        return removed_count, freed_space_gb
    
    def cleanup_sqlite_database(self) -> Tuple[int, float]:
        """Cleanup old entries from SQLite checkpoint database"""
        removed_count = 0
        freed_space_gb = 0.0
        
        try:
            db_path = self.checkpoints_dir / "training_checkpoints.db"
            if not db_path.exists():
                return 0, 0.0
            
            # Get database size before cleanup
            db_size_before = db_path.stat().st_size
            
            conn = sqlite3.connect(str(db_path))
            cursor = conn.cursor()
            
            # Get all checkpoints sorted by timestep (newest first)
            cursor.execute("SELECT id, timestep FROM checkpoints ORDER BY timestep DESC")
            all_checkpoints = cursor.fetchall()
            
            # Keep only the latest N checkpoints
            if len(all_checkpoints) > self.keep_latest_checkpoints:
                checkpoints_to_remove = all_checkpoints[self.keep_latest_checkpoints:]
                
                for checkpoint_id, timestep in checkpoints_to_remove:
                    cursor.execute("DELETE FROM checkpoints WHERE id = ?", (checkpoint_id,))
                    removed_count += 1
                    logger.debug(f"🗑️  Removed checkpoint from DB: timestep {timestep}")
            
            conn.commit()
            conn.execute("VACUUM")  # Reclaim space
            conn.commit()
            conn.close()
            
            # Get database size after cleanup
            if db_path.exists():
                db_size_after = db_path.stat().st_size
                freed_space_gb = (db_size_before - db_size_after) / (1024 ** 3)
            
            if removed_count > 0:
                logger.info(f"✅ Cleaned up {removed_count} old checkpoint entries from DB, freed {freed_space_gb:.2f} GB")
        
        except Exception as e:
            logger.error(f"Error cleaning up SQLite database: {e}")
        
        return removed_count, freed_space_gb
    
    def cleanup_temp_files(self) -> Tuple[int, float]:
        """Remove temporary files"""
        removed_count = 0
        freed_space_gb = 0.0
        
        try:
            # Cleanup temp directory
            if self.temp_dir.exists():
                for temp_file in self.temp_dir.rglob("*"):
                    if temp_file.is_file():
                        try:
                            file_size_gb = temp_file.stat().st_size / (1024 ** 3)
                            temp_file.unlink()
                            removed_count += 1
                            freed_space_gb += file_size_gb
                        except Exception as e:
                            logger.debug(f"Failed to remove temp file {temp_file}: {e}")
            
            # Cleanup Python cache files
            for cache_dir in self.base_dir.rglob("__pycache__"):
                try:
                    cache_size = sum(f.stat().st_size for f in cache_dir.rglob("*") if f.is_file())
                    shutil.rmtree(cache_dir)
                    removed_count += 1
                    freed_space_gb += cache_size / (1024 ** 3)
                except Exception as e:
                    logger.debug(f"Failed to remove cache {cache_dir}: {e}")
            
            # Cleanup .pyc files
            for pyc_file in self.base_dir.rglob("*.pyc"):
                try:
                    file_size_gb = pyc_file.stat().st_size / (1024 ** 3)
                    pyc_file.unlink()
                    removed_count += 1
                    freed_space_gb += file_size_gb
                except Exception as e:
                    logger.debug(f"Failed to remove {pyc_file}: {e}")
            
            if removed_count > 0:
                logger.info(f"✅ Cleaned up {removed_count} temporary files, freed {freed_space_gb:.2f} GB")
        
        except Exception as e:
            logger.error(f"Error cleaning up temp files: {e}")
        
        return removed_count, freed_space_gb
    
    def run_cleanup_if_needed(self) -> Dict[str, any]:
        """Run cleanup if needed, returns cleanup result"""
        return self.run_cleanup(force=False)
    
    def run_cleanup(self, force: bool = False) -> Dict[str, any]:
        """Run cleanup process"""
        if not force and not self.needs_cleanup():
            return {
                'cleaned': False,
                'reason': 'No cleanup needed',
                'freed_space_gb': 0.0
            }
        
        disk_space_before = self.get_disk_space()
        
        logger.info("🧹 Starting automatic cleanup...")
        logger.info(f"   Disk space before: {disk_space_before['free_gb']:.2f} GB free")
        
        total_freed = 0.0
        results = {
            'logs': {'removed': 0, 'freed_gb': 0.0},
            'checkpoints': {'removed': 0, 'freed_gb': 0.0},
            'sqlite': {'removed': 0, 'freed_gb': 0.0},
            'temp': {'removed': 0, 'freed_gb': 0.0}
        }
        
        # Cleanup in order of priority
        removed, freed = self.cleanup_old_logs()
        results['logs'] = {'removed': removed, 'freed_gb': freed}
        total_freed += freed
        
        removed, freed = self.cleanup_old_checkpoints()
        results['checkpoints'] = {'removed': removed, 'freed_gb': freed}
        total_freed += freed
        
        removed, freed = self.cleanup_sqlite_database()
        results['sqlite'] = {'removed': removed, 'freed_gb': freed}
        total_freed += freed
        
        removed, freed = self.cleanup_temp_files()
        results['temp'] = {'removed': removed, 'freed_gb': freed}
        total_freed += freed
        
        disk_space_final = self.get_disk_space()
        
        self.last_cleanup_time = datetime.now()
        
        logger.info(f"✅ Cleanup completed!")
        logger.info(f"   Total freed: {total_freed:.2f} GB")
        logger.info(f"   Disk space after: {disk_space_final['free_gb']:.2f} GB free")
        logger.info(f"   Space improvement: {disk_space_final['free_gb'] - disk_space_before['free_gb']:.2f} GB")
        
        return {
            'cleaned': True,
            'freed_space_gb': total_freed,
            'disk_space_before': disk_space_before,
            'disk_space_after': disk_space_final,
            'results': results
        }


def get_cleanup_manager(base_dir: Path = None) -> AutoCleanupManager:
    """Get or create cleanup manager instance"""
    if base_dir is None:
        base_dir = Path(__file__).parent.parent
    
    return AutoCleanupManager(
        base_dir=base_dir,
        min_free_space_gb=10.0,  # Trigger cleanup when < 10GB free
        keep_latest_logs=3,
        keep_latest_checkpoints=1,
        log_retention_days=3,
        checkpoint_retention_days=7,
        cleanup_interval_hours=6
    )
