import logging
import sys
from pathlib import Path
from datetime import datetime
from typing import Optional


def setup_logger(
    name: str = "carla_sac",
    log_dir: Optional[str] = None,
    level: int = logging.INFO,
    console: bool = True,
    file: bool = True
) -> logging.Logger:
    """
    Setup logger with console and file handlers.
    
    Args:
        name: Logger name
        log_dir: Directory for log files
        level: Logging level
        console: Whether to log to console
        file: Whether to log to file
    
    Returns:
        Configured logger instance
    """
    
    logger = logging.getLogger(name)
    logger.setLevel(level)
    
    # Remove existing handlers
    logger.handlers.clear()
    
    # Create formatter
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    
    # Console handler
    if console:
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setLevel(level)
        console_handler.setFormatter(formatter)
        logger.addHandler(console_handler)
    
    # File handler
    if file and log_dir:
        log_path = Path(log_dir)
        log_path.mkdir(parents=True, exist_ok=True)
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        log_file = log_path / f"{name}_{timestamp}.log"
        
        file_handler = logging.FileHandler(log_file)
        file_handler.setLevel(level)
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)
        
        logger.info(f"Logging to file: {log_file}")
    
    return logger


class TrainingLogger:
    """Logger for tracking training metrics."""
    
    def __init__(self, log_dir: str):
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        
        self.metrics_file = self.log_dir / "training_metrics.csv"
        
        # Initialize CSV file
        if not self.metrics_file.exists():
            with open(self.metrics_file, 'w') as f:
                f.write("iteration,episode_reward_mean,episode_reward_min,episode_reward_max,"
                       "episode_len_mean,timesteps_total,collision_rate,success_rate\n")
    
    def log_iteration(self, iteration: int, metrics: dict):
        """Log metrics for a training iteration."""
        with open(self.metrics_file, 'a') as f:
            f.write(f"{iteration},"
                   f"{metrics.get('episode_reward_mean', 0)},"
                   f"{metrics.get('episode_reward_min', 0)},"
                   f"{metrics.get('episode_reward_max', 0)},"
                   f"{metrics.get('episode_len_mean', 0)},"
                   f"{metrics.get('timesteps_total', 0)},"
                   f"{metrics.get('collision_rate', 0)},"
                   f"{metrics.get('success_rate', 0)}\n")
    
    def get_metrics_file(self) -> Path:
        """Get path to metrics CSV file."""
        return self.metrics_file
