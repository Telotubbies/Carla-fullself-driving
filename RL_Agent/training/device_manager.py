"""
Device Manager

Manages GPU/CPU device selection and verification.
Follows Single Responsibility Principle - only handles device management.
"""

import logging
import torch
from typing import Dict, Any


class DeviceManager:
    """Manages PyTorch device selection and verification."""
    
    @staticmethod
    def setup_device(config: Dict[str, Any]) -> torch.device:
        """
        Setup and verify GPU device with fallback to CPU.
        
        Args:
            config: Configuration dictionary with device settings
            
        Returns:
            PyTorch device object
        """
        device_config = config.get('device', {})
        use_gpu = device_config.get('use_gpu', True)
        
        logging.info(f"Device configuration: use_gpu={use_gpu}")
        
        if not use_gpu:
            logging.info("✅ Using CPU (GPU disabled in config)")
            return torch.device('cpu')
        
        if not torch.cuda.is_available():
            logging.warning("GPU requested but not available. Falling back to CPU.")
            return torch.device('cpu')
        
        try:
            gpu_id = device_config.get('gpu_id', 0)
            device = torch.device(f'cuda:{gpu_id}')
            
            # Test GPU with a simple operation
            DeviceManager._test_gpu(device)
            
            # Log GPU information
            DeviceManager._log_gpu_info(device, gpu_id)
            
            return device
            
        except Exception as e:
            logging.warning(f"Error setting up GPU: {e}. Falling back to CPU.")
            return torch.device('cpu')
    
    @staticmethod
    def _test_gpu(device: torch.device) -> None:
        """Test GPU with a simple operation."""
        try:
            # Simple allocation and computation test (works better with ROCm)
            test_tensor = torch.tensor([1.0, 2.0], device=device)
            test_result = test_tensor * 2
            del test_tensor, test_result
            torch.cuda.empty_cache()
        except Exception as e:
            logging.warning(f"GPU test failed: {e}. Falling back to CPU.")
            raise
    
    @staticmethod
    def _log_gpu_info(device: torch.device, gpu_id: int) -> None:
        """Log GPU information."""
        gpu_name = torch.cuda.get_device_name(gpu_id)
        gpu_memory = torch.cuda.get_device_properties(gpu_id).total_memory / 1e9
        
        logging.info(f"✅ Using GPU: {gpu_name} ({gpu_memory:.2f} GB)")
        
        if torch.version.cuda:
            logging.info(f"   PyTorch CUDA Version: {torch.version.cuda}")
        else:
            logging.info(f"   PyTorch Backend: ROCm/HIP (AMD GPU)")
            logging.info(f"   PyTorch Version: {torch.__version__}")

