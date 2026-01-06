import torch
import torch.nn as nn
import numpy as np
import logging
from typing import Optional, Dict, Any
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.buffers import ReplayBuffer
from .mixed_device_manager import MixedDeviceManager
try:
    from .gpu_fix import apply_gpu_fix
    apply_gpu_fix()
    import stable_baselines3.sac.sac as sac_module
    if hasattr(sac_module, 'th'):
        from .gpu_fix import _safe_ones, _safe_zeros, _safe_tensor
        sac_module.th.ones = _safe_ones
        sac_module.th.zeros = _safe_zeros
        sac_module.th.tensor = _safe_tensor
        logging.debug("Patched stable_baselines3.sac.sac.th in mixed_device_sac")
except Exception as e:
    logging.debug(f"GPU fix application: {e}")
class MixedDeviceSAC(SAC):
    
    def __init__(
        self,
        *args,
        mixed_device_manager: Optional[MixedDeviceManager] = None,
        **kwargs
    ):
        
        super().__init__(*args, **kwargs)
        self.mixed_device_manager = mixed_device_manager
        self.original_device = self.device
        if mixed_device_manager is not None:
            logging.info("✅ Mixed Device SAC initialized")
            logging.info(f"   Primary device: {self.device}")
            logging.info(f"   Mixed device mode: Enabled")
        else:
            logging.info("Mixed Device SAC initialized (mixed device disabled)")
    def train(self, gradient_steps: int, batch_size: int = 64) -> None:
        
        if self.mixed_device_manager is None or self.device.type != 'cuda':
            return super().train(gradient_steps, batch_size)
        current_step = self.num_timesteps
        use_mixed, cpu_batch_ratio = self.mixed_device_manager.should_use_cpu(current_step)
        cpu_batch_count = int(gradient_steps * cpu_batch_ratio) if use_mixed else 0
        gpu_batch_count = gradient_steps - cpu_batch_count
        if use_mixed:
            initial_gpu = self.mixed_device_manager.stats['gpu_batches']
            initial_cpu = self.mixed_device_manager.stats['cpu_batches']
        adjusted_batch_size = batch_size
        if use_mixed and cpu_batch_ratio > 0.2:
            adjusted_batch_size = int(batch_size * (1 - cpu_batch_ratio * 0.5))
            adjusted_batch_size = max(adjusted_batch_size, 32)
            if adjusted_batch_size < batch_size:
                logging.debug(f"Reduced batch size from {batch_size} to {adjusted_batch_size} due to GPU pressure")
        if current_step % 100 == 0 and current_step >= self.learning_starts:
            buffer_size = 0
            if hasattr(self, 'replay_buffer'):
                # Try multiple methods to get buffer size (DictReplayBuffer doesn't support len())
                if hasattr(self.replay_buffer, 'size'):
                    try:
                        buffer_size = self.replay_buffer.size
                    except (AttributeError, TypeError):
                        pass
                if buffer_size == 0:
                    try:
                        # Try len() first (works for regular ReplayBuffer)
                        buffer_size = len(self.replay_buffer)
                    except (TypeError, AttributeError):
                        # DictReplayBuffer doesn't support len(), try other methods
                        try:
                            buffer_size = getattr(self.replay_buffer, 'pos', 0)
                        except (AttributeError, TypeError):
                            try:
                                # Try buffer_size attribute
                                buffer_size = getattr(self.replay_buffer, 'buffer_size', 0)
                            except (AttributeError, TypeError):
                                buffer_size = 0
            logging.info(f"🔄 Training from replay buffer: step={current_step}, buffer_size={buffer_size}, gradient_steps={gradient_steps}, batch_size={adjusted_batch_size}, GPU batches: {gpu_batch_count}, CPU batches: {cpu_batch_count}")
        if gpu_batch_count > 0:
            try:
                original_device = self.device
                self.device = self.mixed_device_manager.gpu_device
                super().train(gpu_batch_count, adjusted_batch_size)
                self.device = original_device
                if use_mixed:
                    self.mixed_device_manager.stats['gpu_batches'] += gpu_batch_count
        except torch.cuda.OutOfMemoryError as e:
            logging.warning(f"GPU OOM during training: {e}")
            self.mixed_device_manager.handle_oom(current_step)
            torch.cuda.empty_cache()
            smaller_batch = max(adjusted_batch_size // 2, 16)
                original_device = self.device
                self.device = self.mixed_device_manager.gpu_device
                super().train(gpu_batch_count, smaller_batch)
                self.device = original_device
                if use_mixed:
                    self.mixed_device_manager.stats['gpu_batches'] += gpu_batch_count
        if cpu_batch_count > 0:
            try:
                original_device = self.device
                cpu_device = self.mixed_device_manager.cpu_device
                self.device = cpu_device
                self.policy = self.policy.to(cpu_device)
                if hasattr(self, 'critic') and self.critic is not None:
                    self.critic = self.critic.to(cpu_device)
                if hasattr(self, 'critic_target') and self.critic_target is not None:
                    self.critic_target = self.critic_target.to(cpu_device)
                super().train(cpu_batch_count, adjusted_batch_size)
                self.policy = self.policy.to(original_device)
                if hasattr(self, 'critic') and self.critic is not None:
                    self.critic = self.critic.to(original_device)
                if hasattr(self, 'critic_target') and self.critic_target is not None:
                    self.critic_target = self.critic_target.to(original_device)
                self.device = original_device
                self.mixed_device_manager.stats['cpu_batches'] += cpu_batch_count
                logging.debug(f"Processed {cpu_batch_count} batches on CPU")
            except Exception as e:
                logging.warning(f"Error processing CPU batches: {e}, falling back to GPU")
                # CRITICAL: Ensure ALL models are restored to GPU before fallback
                try:
                    gpu_device = self.mixed_device_manager.gpu_device
                    # Restore policy (actor in SAC)
                    if hasattr(self, 'policy') and self.policy is not None:
                        self.policy = self.policy.to(gpu_device)
                        # Also ensure policy.actor is on GPU if it exists
                        if hasattr(self.policy, 'actor') and self.policy.actor is not None:
                            self.policy.actor = self.policy.actor.to(gpu_device)
                    # Restore critic networks
                    if hasattr(self, 'critic') and self.critic is not None:
                        self.critic = self.critic.to(gpu_device)
                    if hasattr(self, 'critic_target') and self.critic_target is not None:
                        self.critic_target = self.critic_target.to(gpu_device)
                    # Ensure device attribute is set correctly
                    self.device = gpu_device
                    logging.debug(f"✅ All models restored to GPU ({gpu_device}) before fallback")
                except Exception as restore_error:
                    logging.error(f"❌ CRITICAL: Error restoring GPU devices: {restore_error}")
                    # Try to continue anyway, but log the error
                
                # Fallback: run on GPU
                try:
                    original_device = self.device
                    self.device = self.mixed_device_manager.gpu_device
                    super().train(cpu_batch_count, adjusted_batch_size)
                    self.device = original_device
                    if use_mixed:
                        self.mixed_device_manager.stats['gpu_batches'] += cpu_batch_count
                except Exception as fallback_error:
                    logging.error(f"❌ CRITICAL: Fallback to GPU also failed: {fallback_error}")
                    raise  # Re-raise to let caller handle
        return
    def _sample_action(
        self,
        learning_starts: int,
        action_noise = None,
        n_envs: int = 1,
    ) -> tuple:
        
        def convert_to_numpy(obj):
            
            if isinstance(obj, torch.Tensor):
                return obj.detach().cpu().numpy()
            elif isinstance(obj, np.ndarray):
                return obj
            elif isinstance(obj, dict):
                return {k: convert_to_numpy(v) for k, v in obj.items()}
            elif isinstance(obj, (list, tuple)):
                converted = [convert_to_numpy(item) for item in obj]
                return type(obj)(converted)
            else:
                return obj
        if hasattr(self, '_last_obs') and self._last_obs is not None:
            self._last_obs = convert_to_numpy(self._last_obs)
        result = super()._sample_action(learning_starts, action_noise, n_envs)
        if hasattr(self, '_last_obs') and self._last_obs is not None:
            def has_tensor(obj):
                if isinstance(obj, torch.Tensor):
                    return True
                elif isinstance(obj, dict):
                    return any(has_tensor(v) for v in obj.values())
                elif isinstance(obj, (list, tuple)):
                    return any(has_tensor(v) for v in obj)
                return False
            if has_tensor(self._last_obs):
                self._last_obs = convert_to_numpy(self._last_obs)
        return result
    def _store_transition(
        self,
        replay_buffer,
        buffer_actions,
        new_obs,
        rewards,
        dones,
        infos
    ):
        
        def convert_to_numpy(obj):
            
            if isinstance(obj, torch.Tensor):
                return obj.detach().cpu().numpy()
            elif isinstance(obj, np.ndarray):
                return obj
            elif isinstance(obj, dict):
                return {k: convert_to_numpy(v) for k, v in obj.items()}
            elif isinstance(obj, (list, tuple)):
                converted = [convert_to_numpy(item) for item in obj]
                return type(obj)(converted)
            else:
                return obj
        if not hasattr(self, '_store_transition_debug_count'):
            self._store_transition_debug_count = 0
        self._store_transition_debug_count += 1
        if self._store_transition_debug_count <= 3:
            def log_structure(obj, name="", depth=0):
                
                indent = "  " * depth
                if isinstance(obj, torch.Tensor):
                    logging.info(f"{indent}{name}: Tensor, shape={obj.shape}, device={obj.device}")
                elif isinstance(obj, np.ndarray):
                    logging.info(f"{indent}{name}: Numpy, shape={obj.shape}, dtype={obj.dtype}")
                elif isinstance(obj, dict):
                    logging.info(f"{indent}{name}: Dict with keys: {list(obj.keys())}")
                    for k, v in obj.items():
                        log_structure(v, f"{k}", depth + 1)
                elif isinstance(obj, (list, tuple)):
                    logging.info(f"{indent}{name}: {type(obj).__name__} with {len(obj)} items")
                    if len(obj) > 0:
                        log_structure(obj[0], "[0]", depth + 1)
                else:
                    logging.info(f"{indent}{name}: {type(obj).__name__}")
            logging.info(f"🔍 _store_transition: Input observation (check #{self._store_transition_debug_count})")
            log_structure(new_obs, "new_obs")
        new_obs_converted = convert_to_numpy(new_obs)
        buffer_actions_converted = convert_to_numpy(buffer_actions)
        def has_tensor(obj):
            if isinstance(obj, torch.Tensor):
                return True
            elif isinstance(obj, dict):
                return any(has_tensor(v) for v in obj.values())
            elif isinstance(obj, (list, tuple)):
                return any(has_tensor(v) for v in obj)
            return False
        if has_tensor(new_obs_converted):
            logging.error(f"❌ _store_transition: Found remaining tensor in new_obs after conversion! (check #{self._store_transition_debug_count})")
            if isinstance(new_obs_converted, dict):
                for key, value in new_obs_converted.items():
                    if has_tensor(value):
                        logging.error(f"   Key '{key}': {type(value)}, is_tensor: {isinstance(value, torch.Tensor)}")
                        if isinstance(value, torch.Tensor):
                            logging.error(f"      Shape: {value.shape}, Device: {value.device}")
            logging.warning("⚠️  Force converting remaining tensors...")
            new_obs_converted = convert_to_numpy(new_obs_converted)
        else:
            if self._store_transition_debug_count <= 3:
                logging.info(f"✅ _store_transition: All tensors converted to numpy (check #{self._store_transition_debug_count})")
        return super()._store_transition(replay_buffer, buffer_actions_converted, new_obs_converted, rewards, dones, infos)
    def _on_step(self) -> bool:
        
        if self.mixed_device_manager is not None:
            current_step = self.num_timesteps
            use_mixed, cpu_ratio = self.mixed_device_manager.should_use_cpu(current_step)
            if use_mixed and current_step % 1000 == 0:
                stats = self.mixed_device_manager.get_stats()
                logging.info(f"Mixed device active: {stats['cpu_batch_ratio']*100:.1f}% CPU, "
                           f"GPU memory: {stats['gpu_memory_usage']*100:.1f}%")
        return True
def create_mixed_device_sac(
    policy,
    env,
    mixed_device_manager: Optional[MixedDeviceManager] = None,
    **sac_kwargs
) -> MixedDeviceSAC:
    
    return MixedDeviceSAC(
        policy,
        env,
        mixed_device_manager=mixed_device_manager,
        **sac_kwargs
    )