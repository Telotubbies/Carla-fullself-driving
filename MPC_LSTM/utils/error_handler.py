"""
Production-level error handling utilities.

This module provides centralized error handling and exception management.
"""

import logging
import traceback
from typing import Optional, Callable, Any
from functools import wraps
from pathlib import Path
from datetime import datetime

logger = logging.getLogger(__name__)


class ProjectError(Exception):
    """Base exception for project-specific errors."""
    pass


class CARLAConnectionError(ProjectError):
    """Raised when CARLA connection fails."""
    pass


class ModelLoadError(ProjectError):
    """Raised when model loading fails."""
    pass


class DataValidationError(ProjectError):
    """Raised when data validation fails."""
    pass


class TrainingError(ProjectError):
    """Raised when training fails."""
    pass


def log_error(
    error: Exception,
    context: Optional[str] = None,
    log_file: Optional[Path] = None
) -> None:
    """
    Log an error with full traceback.
    
    Args:
        error: Exception to log
        context: Additional context information
        log_file: Optional log file path
    """
    error_log = Path("logs/errors") if log_file is None else log_file.parent
    error_log.mkdir(parents=True, exist_ok=True)
    
    if log_file is None:
        log_file = error_log / f"errors_{datetime.now().strftime('%Y%m%d')}.log"
    
    error_msg = f"Error: {type(error).__name__}: {str(error)}"
    if context:
        error_msg = f"[{context}] {error_msg}"
    
    logger.error(error_msg)
    logger.error(traceback.format_exc())
    
    # Write to error log file
    with open(log_file, 'a', encoding='utf-8') as f:
        f.write(f"\n{'='*60}\n")
        f.write(f"{datetime.now().isoformat()}\n")
        f.write(f"{error_msg}\n")
        f.write(f"{traceback.format_exc()}\n")
        f.write(f"{'='*60}\n")


def handle_errors(
    context: Optional[str] = None,
    reraise: bool = False,
    default_return: Any = None
):
    """
    Decorator for error handling.
    
    Args:
        context: Context description for error logging
        reraise: Whether to re-raise the exception
        default_return: Default return value on error
    
    Example:
        @handle_errors(context="Data loading", default_return=None)
        def load_data():
            ...
    """
    def decorator(func: Callable) -> Callable:
        @wraps(func)
        def wrapper(*args, **kwargs):
            try:
                return func(*args, **kwargs)
            except Exception as e:
                log_error(e, context=context or func.__name__)
                if reraise:
                    raise
                return default_return
        return wrapper
    return decorator


def safe_execute(
    func: Callable,
    *args,
    context: Optional[str] = None,
    default_return: Any = None,
    **kwargs
) -> Any:
    """
    Safely execute a function with error handling.
    
    Args:
        func: Function to execute
        *args: Positional arguments
        context: Context description
        default_return: Default return value on error
        **kwargs: Keyword arguments
    
    Returns:
        Function result or default_return on error
    """
    try:
        return func(*args, **kwargs)
    except Exception as e:
        log_error(e, context=context or func.__name__)
        return default_return

