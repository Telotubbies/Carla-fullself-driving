import torch
import logging
_original_ones = torch.ones
_original_zeros = torch.zeros
_original_tensor = torch.tensor
def _safe_ones(*args, device=None, **kwargs):
    
    if device is not None and str(device).startswith('cuda'):
        try:
            result = _original_ones(*args, device='cpu', **kwargs)
            result = result.to(device)
            return result
        except Exception as e:
            logging.warning(f"Failed to create tensor on GPU {device}, using CPU: {e}")
            return _original_ones(*args, device='cpu', **kwargs)
    return _original_ones(*args, device=device, **kwargs)
def _safe_zeros(*args, device=None, **kwargs):
    
    if device is not None and str(device).startswith('cuda'):
        try:
            result = _original_zeros(*args, device='cpu', **kwargs)
            result = result.to(device)
            return result
        except Exception as e:
            logging.warning(f"Failed to create tensor on GPU {device}, using CPU: {e}")
            return _original_zeros(*args, device='cpu', **kwargs)
    return _original_zeros(*args, device=device, **kwargs)
def _safe_tensor(data, device=None, **kwargs):
    
    if device is not None and str(device).startswith('cuda'):
        try:
            result = _original_tensor(data, device='cpu', **kwargs)
            result = result.to(device)
            return result
        except Exception as e:
            logging.warning(f"Failed to create tensor on GPU {device}, using CPU: {e}")
            return _original_tensor(data, device='cpu', **kwargs)
    return _original_tensor(data, device=device, **kwargs)
def apply_gpu_fix():
    
    try:
        import torch
        import torch as th
        torch.ones = _safe_ones
        torch.zeros = _safe_zeros
        torch.tensor = _safe_tensor
        th.ones = _safe_ones
        th.zeros = _safe_zeros
        th.tensor = _safe_tensor
        try:
            import stable_baselines3.sac.sac as sac_module
            import importlib
            importlib.reload(sac_module)
            if hasattr(sac_module, 'th'):
                sac_module.th.ones = _safe_ones
                sac_module.th.zeros = _safe_zeros
                sac_module.th.tensor = _safe_tensor
                logging.info("   Patched stable_baselines3.sac.sac.th")
            try:
                from stable_baselines3.common import torch_layers
                if hasattr(torch_layers, 'th'):
                    torch_layers.th.ones = _safe_ones
                    torch_layers.th.zeros = _safe_zeros
                    torch_layers.th.tensor = _safe_tensor
            except Exception:
                pass
        except Exception as e:
            logging.debug(f"Could not patch stable_baselines3 directly: {e}")
            pass
        logging.info("✅ GPU fix applied (ROCm/AMD compatibility)")
        logging.info("   Patched: torch.ones, torch.zeros, torch.tensor, th.ones, th.zeros, th.tensor")
        return True
    except Exception as e:
        logging.warning(f"Failed to apply GPU fix: {e}")
        return False
def remove_gpu_fix():
    
    try:
        import torch as th
        th.ones = _original_ones
        th.zeros = _original_zeros
        th.tensor = _original_tensor
        try:
            from stable_baselines3.common import torch_layers
            if hasattr(torch_layers, 'th'):
                import torch
                torch_layers.th.ones = torch.ones
                torch_layers.th.zeros = torch.zeros
                torch_layers.th.tensor = torch.tensor
        except Exception:
            pass
        logging.info("GPU fix removed")
        return True
    except Exception as e:
        logging.warning(f"Failed to remove GPU fix: {e}")
        return False