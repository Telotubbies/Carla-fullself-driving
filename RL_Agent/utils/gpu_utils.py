"""
GPU utilities for AMD GPU support
"""

import torch
import logging


def check_gpu_available(config: dict) -> bool:
    """
    Check if GPU is available and compatible
    
    Args:
        config: Configuration dictionary
    
    Returns:
        True if GPU is available and ready
    """
    device_config = config.get('device', {})
    use_gpu = device_config.get('use_gpu', True)
    
    if not use_gpu:
        logging.info("GPU check skipped (use_gpu=false in config)")
        return False
    
    if not torch.cuda.is_available():
        logging.warning("⚠️  CUDA/HIP not available in PyTorch")
        return False
    
    gpu_id = device_config.get('gpu_id', 0)
    gpu_name = torch.cuda.get_device_name(gpu_id)
    gpu_memory = torch.cuda.get_device_properties(gpu_id).total_memory / 1e9
    
    logging.info(f"✅ GPU Available: {gpu_name}")
    logging.info(f"   Memory: {gpu_memory:.2f} GB")
    
    # Test GPU computation
    try:
        x = torch.randn(100, 100, device=f'cuda:{gpu_id}')
        y = torch.randn(100, 100, device=f'cuda:{gpu_id}')
        z = torch.matmul(x, y)
        logging.info("✅ GPU computation test: PASSED")
        return True
    except Exception as e:
        logging.error(f"❌ GPU computation test: FAILED - {e}")
        return False


def get_device(config: dict) -> torch.device:
    """
    Get appropriate device based on configuration
    
    Args:
        config: Configuration dictionary
    
    Returns:
        torch.device object
    """
    device_config = config.get('device', {})
    use_gpu = device_config.get('use_gpu', True)
    
    if use_gpu and torch.cuda.is_available():
        gpu_id = device_config.get('gpu_id', 0)
        return torch.device(f'cuda:{gpu_id}')
    else:
        return torch.device('cpu')

