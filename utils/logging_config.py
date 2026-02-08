"""
Production-level logging configuration.

This module provides centralized logging configuration for the entire project.
"""

import logging
import sys
from pathlib import Path
from typing import Optional
from datetime import datetime


def setup_logging(
    log_dir: Path = Path("logs"),
    log_level: str = "INFO",
    log_to_file: bool = True,
    log_to_console: bool = True,
    module_name: Optional[str] = None
) -> logging.Logger:
    """
    Setup logging configuration for a module.
    
    Args:
        log_dir: Directory to save log files
        log_level: Logging level (DEBUG, INFO, WARNING, ERROR, CRITICAL)
        log_to_file: Whether to log to file
        log_to_console: Whether to log to console
        module_name: Name of the module (for log file naming)
    
    Returns:
        Configured logger instance
    """
    # Create log directory
    log_dir.mkdir(parents=True, exist_ok=True)
    
    # Create subdirectories
    (log_dir / "training").mkdir(exist_ok=True)
    (log_dir / "inference").mkdir(exist_ok=True)
    (log_dir / "errors").mkdir(exist_ok=True)
    
    # Get logger
    logger_name = module_name or __name__
    logger = logging.getLogger(logger_name)
    logger.setLevel(getattr(logging, log_level.upper()))
    
    # Clear existing handlers
    logger.handlers.clear()
    
    # Formatter
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    
    # Console handler
    if log_to_console:
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setLevel(getattr(logging, log_level.upper()))
        console_handler.setFormatter(formatter)
        logger.addHandler(console_handler)
    
    # File handler
    if log_to_file:
        if module_name:
            log_file = log_dir / f"{module_name}_{datetime.now().strftime('%Y%m%d')}.log"
        else:
            log_file = log_dir / f"system_{datetime.now().strftime('%Y%m%d')}.log"
        
        file_handler = logging.FileHandler(log_file, encoding='utf-8')
        file_handler.setLevel(getattr(logging, log_level.upper()))
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)
    
    return logger


def get_logger(module_name: str) -> logging.Logger:
    """
    Get a logger for a specific module.
    
    Args:
        module_name: Name of the module
    
    Returns:
        Logger instance
    """
    return logging.getLogger(module_name)

