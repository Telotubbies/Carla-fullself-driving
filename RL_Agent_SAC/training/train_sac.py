import argparse
import atexit
import json
import logging
import os
import re
import signal
import sys
import threading
from datetime import datetime
from pathlib import Path
import numpy as np
import torch
import yaml
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import (
    BaseCallback,
    CallbackList,
    CheckpointCallback,
    EvalCallback,
)
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.utils import set_random_seed
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv
sys.path.insert(0, str(Path(__file__).parent.parent))
from carla_env.carla_rl_env import CarlaRLEnv
from models.sac_policy import VisionSACPolicy
from utils.gpu_utils import check_gpu_available
from utils.logging_utils import setup_logging, TrainingLogger
from utils.prioritized_replay_buffer import PrioritizedReplayBuffer
from utils.sqlite_checkpoint import SQLiteCheckpointManager
from utils.mixed_device_manager import MixedDeviceManager
from utils.mixed_device_sac import MixedDeviceSAC, create_mixed_device_sac
from utils.gpu_fix import apply_gpu_fix
def load_config(config_path: str) -> dict:
    
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    return config
def make_env(config: dict, rank: int = 0, seed: int = 0):
    
    def _init():
        base_port = config.get('environment', {}).get('carla_port', 2000)
        port = base_port
        env = CarlaRLEnv(config, port=port, rank=rank)
        env = Monitor(env, filename=None, allow_early_resets=True)
        env.reset(seed=seed + rank)
        return env
    set_random_seed(seed + rank)
    return _init
def create_training_directories(base_dir: str, config: dict = None):
    
    checkpoint_path = 'checkpoints'
    if config and 'checkpoints' in config:
        checkpoint_path = config['checkpoints'].get('save_path', 'checkpoints')
    dirs = {
        'logs': os.path.join(base_dir, 'logs'),
        'checkpoints': os.path.join(base_dir, checkpoint_path),
        'tensorboard': os.path.join(base_dir, 'logs', 'tensorboard')
    }
    for dir_path in dirs.values():
        os.makedirs(dir_path, exist_ok=True)
    return dirs
def setup_device(config: dict) -> torch.device:
    
    device_config = config.get('device', {})
    use_gpu = device_config.get('use_gpu', True)
    gpu_id = device_config.get('gpu_id', 0)
    if use_gpu and torch.cuda.is_available():
        if gpu_id >= torch.cuda.device_count():
            logging.warning(f"GPU {gpu_id} not available, falling back to CPU")
            logging.info(f"   Available GPUs: {torch.cuda.device_count()}")
            return torch.device('cpu')
        device = torch.device(f'cuda:{gpu_id}')
        try:
            gpu_name = torch.cuda.get_device_name(gpu_id)
            test_tensor = torch.tensor([1.0], device='cpu')
            test_tensor = test_tensor.to(device)
            try:
                test_result = test_tensor + 1.0
                _ = test_result.item()
                del test_tensor, test_result
            except:
                del test_tensor
            torch.cuda.empty_cache()
            logging.info(f"✅ GPU detected and tested: {gpu_name}")
            logging.info(f"   Device: cuda:{gpu_id}")
            logging.info(f"   Memory: {torch.cuda.get_device_properties(gpu_id).total_memory / 1e9:.2f} GB")
            logging.info(f"   GPU test: PASSED (using CPU->GPU tensor transfer)")
        except Exception as e:
            error_str = str(e)
            is_hip_error = 'HIP error' in error_str or 'invalid device function' in error_str
            try:
                props = torch.cuda.get_device_properties(gpu_id)
                if is_hip_error:
                    logging.info(f"⚠️  GPU tensor test failed (expected with ROCm/AMD GPUs): {error_str[:80]}")
                    logging.info(f"   GPU detected: {gpu_name} ({props.total_memory / 1e9:.2f} GB)")
                    logging.info("   This is normal - will use CPU fallback during model creation")
                    logging.info("   Training will work with CPU->GPU transfer method")
                else:
                    logging.warning(f"GPU tensor test failed: {e}")
                    logging.info(f"   GPU detected: {gpu_name} ({props.total_memory / 1e9:.2f} GB)")
                    logging.info("   Will attempt GPU training (may fallback to CPU if needed)")
            except Exception as e2:
                logging.warning(f"GPU detection failed: {e2}")
                logging.info("   Falling back to CPU")
                return torch.device('cpu')
        return device
    else:
        if use_gpu:
            logging.warning("GPU requested but not available, using CPU")
        else:
            logging.info("Using CPU (use_gpu=False)")
        logging.info(f"   Device: cpu")
        return torch.device('cpu')
def create_model(config: dict, env, device: torch.device, log_dir: str, mixed_device_manager=None):
    
    training_config = config.get('training', {})
    sac_config = training_config.get('sac', {})
    policy_kwargs = {
        'config': config,
        'net_arch': config.get('network', {}).get('policy', {}).get('hidden_sizes', [512, 256, 128])
    }
    lr_schedule_config = sac_config.get('lr_schedule', {})
    learning_rate = sac_config.get('learning_rate', 3e-4)
    if lr_schedule_config.get('type') == 'linear':
        initial_lr = lr_schedule_config.get('initial_lr', learning_rate)
        final_lr = lr_schedule_config.get('final_lr', learning_rate * 0.1)
        total_steps = lr_schedule_config.get('total_steps', config.get('training', {}).get('total_timesteps', 500000))
        def linear_schedule(progress_remaining: float) -> float:
            
            return final_lr + (initial_lr - final_lr) * progress_remaining
        learning_rate = linear_schedule
        logging.info(f"Using linear LR schedule: {initial_lr} → {final_lr} over {total_steps} steps")
    else:
        logging.info(f"Using fixed learning rate: {learning_rate}")
    sac_params = {
        'policy': VisionSACPolicy,
        'env': env,
        'learning_rate': learning_rate,
        'buffer_size': sac_config.get('buffer_size', 100000),
        'learning_starts': sac_config.get('learning_starts', 1000),
        'batch_size': sac_config.get('batch_size', 256),
        'tau': sac_config.get('tau', 0.005),
        'gamma': sac_config.get('gamma', 0.99),
        'train_freq': sac_config.get('train_freq', 1),
        'gradient_steps': sac_config.get('gradient_steps', 1),
        'ent_coef': sac_config.get('ent_coef', 'auto'),
        'target_update_interval': sac_config.get('target_update_interval', 1),
        'target_entropy': sac_config.get('target_entropy', 'auto'),
        'use_sde': sac_config.get('use_sde', False),
        'sde_sample_freq': sac_config.get('sde_sample_freq', -1),
        'use_sde_at_warmup': sac_config.get('use_sde_at_warmup', False),
        'policy_kwargs': policy_kwargs,
        'tensorboard_log': os.path.join(log_dir, 'tensorboard'),
        'device': device,
        'verbose': 1
    }
    if mixed_device_manager is not None:
        logging.info(f"Creating MixedDeviceSAC model with device: {device}")
        sac_params['mixed_device_manager'] = mixed_device_manager
        model_class = MixedDeviceSAC
    else:
        logging.info(f"Creating standard SAC model with device: {device}")
        model_class = SAC
    try:
        model = model_class(**sac_params)
        logging.info(f"✅ SAC model created successfully with device: {device}")
        logging.info(f"   Model device: {next(model.policy.parameters()).device}")
        if mixed_device_manager is not None:
            logging.info(f"   Mixed device mode: Enabled")
        return model
    except RuntimeError as e:
        error_str = str(e)
        if 'HIP error' in error_str or 'invalid device function' in error_str:
            if device.type == 'cuda':
                logging.info("ℹ️  GPU error during model creation (expected with ROCm/AMD GPUs)")
                logging.info("   Attempting CPU fallback with GPU transfer...")
                logging.info("   This is normal for AMD GPUs - model will be created on CPU first")
                try:
                    cpu_device = torch.device('cpu')
                    sac_params_cpu = sac_params.copy()
                    sac_params_cpu['device'] = cpu_device
                    model = model_class(**sac_params_cpu)
                    try:
                        model.policy = model.policy.to(device)
                        if hasattr(model, 'critic') and model.critic is not None:
                            model.critic = model.critic.to(device)
                        if hasattr(model, 'critic_target') and model.critic_target is not None:
                            model.critic_target = model.critic_target.to(device)
                        model.device = device
                        logging.info(f"✅ Model created on CPU and moved to GPU: {device}")
                        logging.info(f"   Model device: {next(model.policy.parameters()).device}")
                    except Exception as move_error:
                        logging.warning(f"Failed to move model to GPU: {move_error}")
                        logging.info("   Continuing with CPU training")
                        model.device = cpu_device
                    if mixed_device_manager is not None:
                        logging.info(f"   Mixed device mode: Enabled")
                    return model
                except Exception as cpu_error:
                    logging.error(f"Failed to create model on CPU: {cpu_error}", exc_info=True)
                    raise
            else:
                raise
        else:
            raise
    except Exception as e:
        logging.error(f"Error creating model with {device}: {e}", exc_info=True)
        if device.type == 'cuda':
            logging.warning("Falling back to CPU due to model creation error...")
            device = torch.device('cpu')
            return create_model(config, env, device, log_dir, mixed_device_manager=None)
        else:
            raise
def setup_callbacks(config: dict, dirs: dict, eval_env=None, sqlite_manager=None, training_env=None, mixed_device_manager=None):
    
    callbacks = []
    class CurriculumCallback(BaseCallback):
        
        def __init__(self, training_env, verbose=0):
            super().__init__(verbose)
            self._training_env = training_env
            self.last_update_timestep = 0
            self.update_freq = 10000
        @property
        def training_env(self):
            return self._training_env
        @training_env.setter
        def training_env(self, value):
            self._training_env = value
        def _on_step(self) -> bool:
            if self.num_timesteps - self.last_update_timestep >= self.update_freq:
                if hasattr(self.training_env, 'envs') and len(self.training_env.envs) > 0:
                    for env_wrapper in self.training_env.envs:
                        if hasattr(env_wrapper, 'env') and hasattr(env_wrapper.env, 'update_curriculum'):
                            env_wrapper.env.update_curriculum(self.num_timesteps)
                            env_wrapper.env.update_reward_scale(self.num_timesteps)
                elif hasattr(self.training_env, 'env') and hasattr(self.training_env.env, 'update_curriculum'):
                    self.training_env.env.update_curriculum(self.num_timesteps)
                    self.training_env.env.update_reward_scale(self.num_timesteps)
                elif hasattr(self.training_env, 'update_curriculum'):
                    self.training_env.update_curriculum(self.num_timesteps)
                    self.training_env.update_reward_scale(self.num_timesteps)
                self.last_update_timestep = self.num_timesteps
            return True
    if training_env is not None:
        curriculum_callback = CurriculumCallback(training_env)
        callbacks.append(curriculum_callback)
    class LoggingCallback(BaseCallback):
        def __init__(self, verbose=0):
            super().__init__(verbose)
            self.step_count = 0
            self.log_interval = 1
        def _on_step(self) -> bool:
            self.step_count += 1
            logging.info(f"Callback: Step {self.step_count} - _on_step() called")
            try:
                handlers = logging.getLogger().handlers
                if handlers:
                    handlers[0].flush()
            except Exception:
                pass
            if len(self.locals.get('infos', [])) > 0:
                info = self.locals['infos'][0]
                logging.info(f"Callback: Step {self.step_count} - Info available: {list(info.keys())}")
                if 'episode' in info:
                    ep_rew = info['episode']['r']
                    ep_len = info['episode']['l']
                    logging.info(f"Callback: Step {self.step_count} - Episode reward: {ep_rew:.2f}, Episode length: {ep_len}")
            else:
                n_steps = self.locals.get('n_steps', 32)
                logging.info(f"Callback: Step {self.step_count}/{n_steps} - Collecting experiences (no episode info yet)")
            return True
        def _on_rollout_start(self) -> None:
            logging.info("Callback: _on_rollout_start() called - Starting to collect experiences")
            try:
                handlers = logging.getLogger().handlers
                if handlers:
                    handlers[0].flush()
            except Exception:
                pass
        def _on_rollout_end(self) -> None:
            logging.info("Callback: _on_rollout_end() called - Finished collecting experiences, starting training update")
            try:
                handlers = logging.getLogger().handlers
                if handlers:
                    handlers[0].flush()
            except Exception:
                pass
    callbacks.append(LoggingCallback())
    if mixed_device_manager is not None:
        class MixedDeviceCallback(BaseCallback):
            
            def __init__(self, mixed_device_manager, verbose=0):
                super().__init__(verbose)
                self.mixed_device_manager = mixed_device_manager
                self.last_log_step = 0
                self.log_interval = 1000
            def _on_step(self) -> bool:
                use_cpu, cpu_ratio = self.mixed_device_manager.should_use_cpu(self.num_timesteps)
                if self.num_timesteps - self.last_log_step >= self.log_interval:
                    self.mixed_device_manager.log_stats(self.num_timesteps)
                    self.last_log_step = self.num_timesteps
                return True
            def _on_rollout_end(self) -> None:
                use_cpu, cpu_ratio = self.mixed_device_manager.should_use_cpu(self.num_timesteps)
                if use_cpu:
                    stats = self.mixed_device_manager.get_stats()
                    logging.debug(f"Mixed device mode active: {stats['cpu_batch_ratio']*100:.1f}% batches on CPU")
        mixed_device_callback = MixedDeviceCallback(mixed_device_manager)
        callbacks.append(mixed_device_callback)
        logging.info("✅ Mixed Device Manager callback enabled (GPU/CPU load balancing)")
    class EnhancedCheckpointCallback(BaseCallback):
        
        def __init__(self, save_freq, checkpoint_dir, save_format='both', config=None, verbose=0):
            super().__init__(verbose)
            self.save_freq = save_freq
            self.checkpoint_dir = checkpoint_dir
            self.save_format = save_format
            self.config = config
            self.last_save_timestep = 0
            self.save_in_progress = False
            self.save_thread = None
            self.save_lock = threading.Lock()
            os.makedirs(os.path.join(checkpoint_dir, 'checkpoint'), exist_ok=True)
            os.makedirs(os.path.join(checkpoint_dir, 'enhanced'), exist_ok=True)
        def _on_step(self) -> bool:
            if self.num_timesteps - self.last_save_timestep >= self.save_freq:
                self._save_checkpoint()
            return True
        def _save_checkpoint(self):
            
            with self.save_lock:
                if self.save_in_progress:
                    logging.warning(f"⚠️  Previous checkpoint save still in progress, skipping timestep {self.num_timesteps}")
                    return
                if self.save_thread is not None and self.save_thread.is_alive():
                    logging.warning("⚠️  Previous save thread still running, waiting...")
                    self.save_thread.join(timeout=5)
                    if self.save_thread.is_alive():
                        logging.error("❌ Previous save thread did not complete, skipping new save")
                        self.save_in_progress = False
                        return
                self.save_in_progress = True
                step = self.num_timesteps
                model = self.model
                self.save_thread = threading.Thread(
                    target=self._save_checkpoint_sync,
                    args=(step, model),
                    daemon=True,
                    name=f"CheckpointSave-{step}"
                )
                self.save_thread.start()
                logging.info(f"💾 Started async checkpoint save for timestep {step}")
                self.last_save_timestep = step
        def _save_checkpoint_sync(self, step, model):
            
            import torch
            # Store original device before save
            original_device = None
            try:
                if hasattr(model, 'device'):
                    original_device = str(model.device)
                elif hasattr(model, 'policy') and hasattr(model.policy, 'device'):
                    original_device = str(model.policy.device)
            except:
                pass
            
            try:
                logging.info(f"💾 Starting checkpoint save for step {step}...")
                if self.save_format in ['old', 'both']:
                    old_path = os.path.join(self.checkpoint_dir, 'checkpoint', f'rl_model_{step}_steps.zip')
                    try:
                        import time
                        start_time = time.time()
                        save_completed = threading.Event()
                        save_error = [None]
                        def do_save():
                            try:
                                model.save(old_path)
                                save_completed.set()
                            except Exception as e:
                                save_error[0] = e
                                save_completed.set()
                        save_thread = threading.Thread(target=do_save, daemon=True)
                        save_thread.start()
                        save_thread.join(timeout=30.0)
                        if not save_completed.is_set():
                            logging.error(f"❌ Checkpoint save TIMEOUT after 30s: {old_path}")
                            return
                        elif save_error[0] is not None:
                            raise save_error[0]
                        save_time = time.time() - start_time
                        logging.info(f"✅ Checkpoint save completed in {save_time:.2f}s: {old_path}")
                        
                        # Restore model device after save
                        if original_device:
                            try:
                                import torch
                                device_obj = torch.device(original_device)
                                if hasattr(model, 'policy'):
                                    model.policy = model.policy.to(device_obj)
                                if hasattr(model, 'device'):
                                    model.device = device_obj
                                logging.debug(f"✅ Model device restored to {original_device} after checkpoint save")
                            except Exception as e:
                                logging.warning(f"⚠️  Could not restore model device after checkpoint: {e}")
                    except Exception as e:
                        logging.error(f"❌ Checkpoint save error: {e}", exc_info=True)
                        # Try to restore device even on error
                        if original_device:
                            try:
                                import torch
                                device_obj = torch.device(original_device)
                                if hasattr(model, 'policy'):
                                    model.policy = model.policy.to(device_obj)
                            except:
                                pass
                        return
                    if os.path.exists(old_path):
                        if self.config is not None:
                            config_path = old_path.replace('.zip', '_config.yaml')
                            try:
                                import yaml
                                with open(config_path, 'w') as f:
                                    yaml.dump(self.config, f, default_flow_style=False)
                                logging.info(f"✅ Saved checkpoint config: {config_path}")
                            except Exception as e:
                                logging.warning(f"Failed to save checkpoint config: {e}")
                    else:
                        logging.error(f"❌ Checkpoint file not created: {old_path}")
                if self.save_format in ['new', 'both']:
                    new_dir = os.path.join(self.checkpoint_dir, 'enhanced', f'checkpoint_{step}')
                    try:
                        os.makedirs(new_dir, exist_ok=True)
                        model_path = os.path.join(new_dir, 'model.zip')
                        try:
                            import time
                            start_time = time.time()
                            save_completed = threading.Event()
                            save_error = [None]
                            def do_save():
                                try:
                                    model.save(model_path)
                                    save_completed.set()
                                except Exception as e:
                                    save_error[0] = e
                                    save_completed.set()
                            save_thread = threading.Thread(target=do_save, daemon=True)
                            save_thread.start()
                            save_thread.join(timeout=30.0)
                            if not save_completed.is_set():
                                logging.error(f"❌ Enhanced checkpoint save TIMEOUT after 30s: {model_path}")
                                return
                            elif save_error[0] is not None:
                                raise save_error[0]
                            save_time = time.time() - start_time
                            logging.info(f"✅ Enhanced checkpoint save completed in {save_time:.2f}s: {model_path}")
                        except Exception as e:
                            logging.error(f"❌ Enhanced checkpoint save error: {e}", exc_info=True)
                            return
                        if os.path.exists(model_path):
                            metadata = {
                                'timestep': step,
                                'timestamp': datetime.now().isoformat(),
                                'algorithm': 'SAC'
                            }
                            if self.config is not None:
                                metadata['config'] = self.config
                            metadata_path = os.path.join(new_dir, 'metadata.json')
                            with open(metadata_path, 'w') as f:
                                json.dump(metadata, f, indent=2)
                            if self.config is not None:
                                try:
                                    import yaml
                                    config_path = os.path.join(new_dir, 'config.yaml')
                                    with open(config_path, 'w') as f:
                                        yaml.dump(self.config, f, default_flow_style=False)
                                except Exception as e:
                                    logging.warning(f"Failed to save config YAML: {e}")
                    except Exception as e:
                        logging.error(f"Failed to save enhanced checkpoint: {e}", exc_info=True)
                try:
                    import requests
                    requests.post('http://localhost:5000/api/checkpoint_saved',
                               json={'timestep': step, 'timestamp': datetime.now().isoformat()},
                               timeout=0.1)
                except Exception:
                    pass
            except Exception as e:
                logging.error(f"❌ Failed to save checkpoint: {e}", exc_info=True)
            finally:
                with self.save_lock:
                    self.save_in_progress = False
    checkpoint_config = config.get('checkpoints', {})
    save_format = config.get('training', {}).get('save_format', 'both')
    save_freq = checkpoint_config.get('save_freq', 10000)
    enhanced_checkpoint_callback = EnhancedCheckpointCallback(
        save_freq=save_freq,
        checkpoint_dir=dirs['checkpoints'],
        save_format=save_format,
        config=config,
        verbose=1
    )
    callbacks.append(enhanced_checkpoint_callback)
    logging.info(f"✅ Enhanced checkpoint callback enabled (format: {save_format}, save every {save_freq} steps)")
    checkpoint_callback = CheckpointCallback(
        save_freq=save_freq,
        save_path=os.path.join(dirs['checkpoints'], 'checkpoint'),
        name_prefix='rl_model',
        save_replay_buffer=True,
        save_vecnormalize=False
    )
    if save_format != 'old':
        callbacks.append(checkpoint_callback)
    eval_config = config.get('training', {})
    if eval_env is not None and eval_config.get('eval_freq', 0) > 0:
        eval_callback = EvalCallback(
            eval_env,
            best_model_save_path=os.path.join(dirs['checkpoints'], 'best_model'),
            log_path=os.path.join(dirs['logs'], 'evaluations'),
            eval_freq=eval_config.get('eval_freq', 50000),
            n_eval_episodes=eval_config.get('eval_episodes', 10),
            deterministic=True,
            render=False
        )
        callbacks.append(eval_callback)
        logging.info(f"✅ Evaluation callback enabled: eval every {eval_config.get('eval_freq', 50000)} steps")
    if sqlite_manager is not None:
        class SQLiteCheckpointCallback(BaseCallback):
            def __init__(self, sqlite_manager, save_freq, verbose=0):
                super().__init__(verbose)
                self.sqlite_manager = sqlite_manager
                self.save_freq = save_freq
                self.last_save_timestep = 0
                self.save_thread = None
                self.save_in_progress = False
            def _on_step(self) -> bool:
                return True
            def _save_checkpoint_async(self, model, timestep, episode, reward, metadata):
                
                import torch
                original_device = None
                try:
                    # Store original device before save
                    if hasattr(model, 'device'):
                        original_device = model.device
                    elif hasattr(model, 'policy') and hasattr(model.policy, 'device'):
                        original_device = model.policy.device
                    
                    self.sqlite_manager.save_checkpoint(
                        model=model,
                        timestep=timestep,
                        episode=episode,
                        reward=reward,
                        metadata=metadata
                    )
                    logging.info(f"💾 SQLite checkpoint saved at timestep {timestep}")
                    
                    # Ensure model is still on correct device after save
                    if original_device:
                        try:
                            if hasattr(model, 'policy'):
                                model.policy = model.policy.to(original_device)
                            if hasattr(model, 'device'):
                                model.device = original_device
                            logging.debug(f"✅ Model device restored to {original_device} after checkpoint save")
                        except Exception as e:
                            logging.warning(f"⚠️  Could not restore model device after checkpoint: {e}")
                            
                except Exception as e:
                    logging.error(f"Failed to save SQLite checkpoint: {e}", exc_info=True)
                    # Try to restore device even on error
                    if original_device:
                        try:
                            if hasattr(model, 'policy'):
                                model.policy = model.policy.to(original_device)
                        except:
                            pass
                finally:
                    self.save_in_progress = False
            def _on_rollout_end(self) -> None:
                if self.num_timesteps - self.last_save_timestep >= self.save_freq:
                    if self.save_in_progress:
                        logging.warning(f"⚠️  Previous SQLite save still in progress, skipping timestep {self.num_timesteps}")
                        return
                    try:
                        infos = self.locals.get('infos', [])
                        reward = None
                        episode = None
                        if infos and len(infos) > 0:
                            info = infos[0]
                            if 'episode' in info:
                                reward = info['episode'].get('r')
                                episode = info['episode'].get('l')
                        metadata = {
                            'algorithm': 'SAC',
                            'config': config.get('training', {}).get('algorithm', 'SAC')
                        }
                        self.save_in_progress = True
                        self.save_thread = threading.Thread(
                            target=self._save_checkpoint_async,
                            args=(self.model, self.num_timesteps, episode, reward, metadata)
                        )
                        self.save_thread.daemon = True
                        self.save_thread.start()
                        self.last_save_timestep = self.num_timesteps
                    except Exception as e:
                        logging.error(f"Failed to queue SQLite checkpoint save: {e}", exc_info=True)
                        self.save_in_progress = False
        save_freq = checkpoint_config.get('save_freq', 10000)
        sqlite_callback = SQLiteCheckpointCallback(sqlite_manager, save_freq)
        callbacks.append(sqlite_callback)
        logging.info(f"✅ SQLite checkpoint callback enabled: save every {save_freq} steps")
    return CallbackList(callbacks)
_env = None
_eval_env = None
_model = None
_shutdown_requested = False
def cleanup_resources():
    
    global _env, _eval_env, _model, _shutdown_requested
    if '_shutdown_requested' not in globals():
        globals()['_shutdown_requested'] = False
    if _shutdown_requested:
        return
    _shutdown_requested = True
    logging.info("🧹 Cleaning up resources...")
    try:
        if _env is not None:
            try:
                import threading
                close_completed = threading.Event()
                close_error = [None]
                def do_close():
                    try:
                        _env.close()
                        close_completed.set()
                    except Exception as e:
                        close_error[0] = e
                        close_completed.set()
                close_thread = threading.Thread(target=do_close, daemon=True)
                close_thread.start()
                close_thread.join(timeout=10.0)
                if not close_completed.is_set():
                    logging.error("❌ Training environment close TIMEOUT - may be hanging")
                elif close_error[0] is None:
                    logging.info("✅ Training environment closed")
                else:
                    logging.warning(f"Error closing training environment: {close_error[0]}")
            except Exception as e:
                logging.warning(f"Error closing training environment: {e}")
        if _eval_env is not None:
            try:
                import threading
                close_completed = threading.Event()
                close_error = [None]
                def do_close():
                    try:
                        _eval_env.close()
                        close_completed.set()
                    except Exception as e:
                        close_error[0] = e
                        close_completed.set()
                close_thread = threading.Thread(target=do_close, daemon=True)
                close_thread.start()
                close_thread.join(timeout=10.0)
                if not close_completed.is_set():
                    logging.error("❌ Evaluation environment close TIMEOUT - may be hanging")
                elif close_error[0] is None:
                    logging.info("✅ Evaluation environment closed")
                else:
                    logging.warning(f"Error closing evaluation environment: {close_error[0]}")
            except Exception as e:
                logging.warning(f"Error closing evaluation environment: {e}")
        if _model is not None:
            try:
                import time
                time.sleep(2.0)
            except Exception as e:
                logging.warning(f"Error waiting for checkpoint threads: {e}")
        if torch.cuda.is_available():
            try:
                torch.cuda.empty_cache()
                logging.info("✅ GPU memory cleared")
            except Exception as e:
                logging.warning(f"Error clearing GPU memory: {e}")
        import gc
        gc.collect()
        logging.info("✅ Cleanup completed")
    except Exception as e:
        logging.error(f"Error during cleanup: {e}", exc_info=True)
def signal_handler(signum, frame):
    
    logging.info(f"⚠️  Received interrupt signal {signum}, shutting down gracefully...")
    logging.info(f"   Current timestep: {_model.num_timesteps if _model else 'N/A'}")
    cleanup_resources()
    sys.exit(0)
def main():
    global _env, _eval_env, _model
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    atexit.register(cleanup_resources)
    parser = argparse.ArgumentParser(description='Train Vision-Only RL Agent for CARLA using SAC')
    parser.add_argument(
        '--config',
        type=str,
        default='config/sac_config.yaml',
        help='Path to configuration file'
    )
    parser.add_argument(
        '--resume',
        type=str,
        default=None,
        help='Path to checkpoint to resume from'
    )
    parser.add_argument(
        '--num-envs',
        type=int,
        default=1,
        help='Number of parallel environments'
    )
    args = parser.parse_args()
    base_path = Path(__file__).parent.parent
    config_path = base_path / args.config
    config = load_config(str(config_path))
    log_dir = base_path / 'logs'
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    log_file = log_dir / f'sac_training_{timestamp}.log'
    setup_logging(str(log_file))
    logging.info("=" * 60)
    logging.info("Vision-Only RL Agent Training - SAC Algorithm")
    logging.info("=" * 60)
    logging.info(f"Configuration: {args.config}")
    logging.info(f"Timestamp: {timestamp}")
    base_dir = Path(__file__).parent.parent
    dirs = create_training_directories(str(base_dir), config)
    logging.info(f"Logs directory: {dirs['logs']}")
    logging.info(f"Checkpoints directory: {dirs['checkpoints']}")
    if config.get('device', {}).get('use_gpu', True) and torch.cuda.is_available():
        apply_gpu_fix()
    if 'AMD_SERIALIZE_KERNEL' not in os.environ:
        os.environ['AMD_SERIALIZE_KERNEL'] = '3'
        logging.info("✅ Set AMD_SERIALIZE_KERNEL=3 for better HIP error reporting")
    if 'HSA_OVERRIDE_GFX_VERSION' not in os.environ:
        pass
    device = setup_device(config)
    if device.type == 'cuda':
        gpu_check_result = check_gpu_available(config)
        if not gpu_check_result:
            logging.info("⚠️  GPU computation test failed (expected with ROCm/AMD GPUs)")
            logging.info("   This is normal - model creation will use CPU fallback")
            logging.info("   Training will work with CPU->GPU transfer method")
    else:
        logging.info("Skipping GPU check (using CPU)")
    seed = config.get('seed', 42)
    set_random_seed(seed, using_cuda=(device.type == 'cuda'))
    logging.info(f"Random seed: {seed}")
    logging.info("Creating environments...")
    num_envs = args.num_envs
    if num_envs > 1:
        logging.info(f"Using {num_envs} parallel environments with SubprocVecEnv")
        logging.info("   Note: SAC is off-policy, so multiple envs may not provide much benefit")
        base_port = config.get('environment', {}).get('carla_port', 2000)
        logging.info(f"   All environments will use the same CARLA server (port {base_port})")
        _env = SubprocVecEnv([make_env(config, i, seed) for i in range(num_envs)])
        env = _env
    else:
        logging.info("Using single environment with DummyVecEnv")
        _env = DummyVecEnv([make_env(config, 0, seed)])
        env = _env
    _eval_env = DummyVecEnv([make_env(config, 0, seed + 1000)])
    eval_env = _eval_env
    logging.info(f"Created {num_envs} training environment(s)")
    sqlite_manager = None
    use_sqlite = config.get('checkpoints', {}).get('use_sqlite', True)
    clear_db = config.get('checkpoints', {}).get('clear_on_start', False)
    if use_sqlite:
        db_path = config.get('checkpoints', {}).get('sqlite_db', 'checkpoints/training_checkpoints.db')
        db_path = os.path.join(base_dir, db_path)
        os.makedirs(os.path.dirname(db_path), exist_ok=True)
        sqlite_manager = SQLiteCheckpointManager(db_path)
        if clear_db:
            try:
                sqlite_manager.clear_database()
                logging.info("✅ Cleared SQLite checkpoint database")
            except Exception as e:
                logging.warning(f"Failed to clear database: {e}")
        logging.info(f"✅ SQLite checkpoint manager initialized: {db_path}")
    _model = None
    model = None
    resume_from_checkpoint = False
    if args.resume:
        logging.info(f"🔄 Loading checkpoint: {args.resume}")
        try:
            _model = SAC.load(args.resume, env=env, device=device)
            model = _model
            resume_from_checkpoint = True
            logging.info(f"✅ Checkpoint loaded: timestep {model.num_timesteps:,}")
        except Exception as e:
            logging.error(f"❌ Failed to load checkpoint: {e}", exc_info=True)
            raise
    elif sqlite_manager:
        checkpoint_info = sqlite_manager.get_latest_checkpoint_info()
        if checkpoint_info:
            logging.info(f"📂 Found SQLite checkpoint: timestep={checkpoint_info['timestep']}")
            try:
                temp_checkpoint = sqlite_manager.extract_checkpoint_to_temp(checkpoint_info['id'])
                _model = SAC.load(temp_checkpoint, env=env, device=device)
                model = _model
                resume_from_checkpoint = True
                logging.info(f"✅ Loaded checkpoint from SQLite: timestep {model.num_timesteps:,}")
            except Exception as e:
                logging.warning(f"Failed to load from SQLite: {e}")
                logging.info("Starting new training...")
    mixed_device_manager = None
    device_config = config.get('device', {})
    use_mixed_device = device_config.get('use_mixed_device', True)
    if use_mixed_device and device.type == 'cuda':
        gpu_memory_threshold = device_config.get('gpu_memory_threshold', 0.85)
        gpu_util_threshold = device_config.get('gpu_util_threshold', 0.90)
        check_interval = device_config.get('mixed_device_check_interval', 100)
        mixed_device_manager = MixedDeviceManager(
            gpu_device=device,
            cpu_device=torch.device('cpu'),
            gpu_memory_threshold=gpu_memory_threshold,
            gpu_util_threshold=gpu_util_threshold,
            check_interval=check_interval
        )
        logging.info("✅ Mixed Device Manager initialized (GPU/CPU load balancing enabled)")
    else:
        if device.type == 'cuda':
            logging.info("Mixed Device Manager disabled (use_mixed_device=False)")
        else:
            logging.info("Mixed Device Manager disabled (using CPU only)")
    il_config = config.get('training', {}).get('imitation_learning', {})
    il_enabled = il_config.get('enabled', False)
    il_pretrained_path = il_config.get('pretrained_path', None)
    resume_from_checkpoint = model is not None
    if il_enabled and il_pretrained_path and os.path.exists(il_pretrained_path):
        if model is None:
            logging.info("Creating RL model...")
            _model = create_model(config, env, device, dirs['logs'], mixed_device_manager=mixed_device_manager)
            model = _model
        logging.info(f"🔄 Loading IL pre-trained weights from: {il_pretrained_path}")
        try:
            checkpoint = torch.load(il_pretrained_path, map_location=device, weights_only=False)
            if 'policy_state_dict' in checkpoint:
                il_state_dict = checkpoint['policy_state_dict']
                current_state_dict = model.policy.state_dict()
                filtered_state_dict = {}
                skipped_layers = []
                loaded_layers = []
                for key, value in il_state_dict.items():
                    if key in current_state_dict:
                        if current_state_dict[key].shape == value.shape:
                            filtered_state_dict[key] = value
                            loaded_layers.append(key)
                        else:
                            skipped_layers.append(f"{key} (shape mismatch: {value.shape} vs {current_state_dict[key].shape})")
                    else:
                        skipped_layers.append(f"{key} (not in current model)")
                if filtered_state_dict:
                    model.policy.load_state_dict(filtered_state_dict, strict=False)
                    logging.info(f"✅ IL pre-trained weights loaded successfully!")
                    logging.info(f"   • Loaded {len(loaded_layers)} compatible layers")
                    if skipped_layers:
                        logging.info(f"   • Skipped {len(skipped_layers)} incompatible layers:")
                        for skip in skipped_layers[:5]:
                            logging.info(f"     - {skip}")
                        if len(skipped_layers) > 5:
                            logging.info(f"     ... and {len(skipped_layers) - 5} more")
                    if resume_from_checkpoint:
                        logging.info("   (IL weights applied on top of resumed checkpoint - vision encoder + compatible layers)")
                else:
                    logging.warning("⚠️  No compatible layers found in IL checkpoint")
            else:
                logging.warning("⚠️  No 'policy_state_dict' in checkpoint, skipping IL weights")
        except Exception as e:
            logging.warning(f"⚠️  Failed to load IL pre-trained weights: {e}")
            import traceback
            logging.debug(f"Traceback: {traceback.format_exc()}")
    elif il_enabled and il_pretrained_path:
        logging.warning(f"⚠️  IL pre-trained path specified but file not found: {il_pretrained_path}")
        if model is None:
            logging.info("Creating RL model...")
            _model = create_model(config, env, device, dirs['logs'], mixed_device_manager=mixed_device_manager)
            model = _model
    elif model is None:
        logging.info("Creating new SAC model...")
        _model = create_model(config, env, device, dirs['logs'], mixed_device_manager=mixed_device_manager)
        model = _model
    callbacks = setup_callbacks(
        config, dirs,
        eval_env=eval_env,
        sqlite_manager=sqlite_manager,
        training_env=env,
        mixed_device_manager=mixed_device_manager
    )
    prefetch_enabled = config.get('training', {}).get('prefetching', {}).get('enabled', False)
    if prefetch_enabled:
        logging.info("✅ Data Prefetching enabled (will pre-load next batch while training)")
    total_timesteps = config.get('training', {}).get('total_timesteps', 500000)
    logging.info(f"Starting training for {total_timesteps} timesteps...")
    logging.info("=" * 60)
    logging.info("Testing environment before training...")
    try:
        reset_result = env.reset()
        if isinstance(reset_result, tuple):
            test_obs, _ = reset_result
        else:
            test_obs = reset_result
        if isinstance(test_obs, dict):
            obs_info = ", ".join([f"{k}: {v.shape if hasattr(v, 'shape') else type(v).__name__}" for k, v in test_obs.items()])
            logging.info(f"Environment reset successful, obs (Dict): {obs_info}")
        else:
            logging.info(f"Environment reset successful, obs shape: {test_obs.shape}")
        test_action = env.action_space.sample()
        if not isinstance(test_action, np.ndarray):
            test_action = np.array(test_action)
        test_action = test_action.flatten()
        if len(test_action) < 3:
            test_action = np.pad(test_action, (0, 3 - len(test_action)), mode='constant', constant_values=0.0)
        elif len(test_action) > 3:
            test_action = test_action[:3]
        step_result = env.step(test_action)
        if len(step_result) == 5:
            test_obs, test_reward, test_done, test_truncated, test_info = step_result
        else:
            test_obs, test_reward, test_done, test_info = step_result
            test_truncated = test_done
        if isinstance(test_action, np.ndarray) and test_action.ndim == 0:
            test_action = np.array([test_action])
        logging.info(f"Environment step successful, reward: {test_reward[0] if hasattr(test_reward, '__len__') else test_reward:.2f}")
        logging.info("Environment is ready for training!")
    except Exception as e:
        logging.error(f"Environment test failed: {e}", exc_info=True)
        raise
    logging.info("Starting SAC training loop...")
    sac_config = config.get('training', {}).get('sac', {})
    learning_starts = sac_config.get('learning_starts', 1000)
    logging.info(f"   SAC will collect {learning_starts} steps before first training update")
    logging.info(f"   This may take a few minutes depending on environment speed...")
    try:
        logging.info("Calling model.learn()...")
        logging.info("   About to enter SAC collect phase (learning_starts)...")
        reset_timesteps = not resume_from_checkpoint
        if resume_from_checkpoint:
            logging.info(f"🔄 Resuming training from timestep {model.num_timesteps} (reset_num_timesteps=False)")
        else:
            logging.info(f"🆕 Starting new training (reset_num_timesteps=True)")
        logging.info(f"   Entering model.learn() with total_timesteps={total_timesteps}, reset_num_timesteps={reset_timesteps}")
        logging.info(f"   Current model.num_timesteps: {model.num_timesteps}")
        try:
            model.learn(
                total_timesteps=total_timesteps,
                callback=callbacks,
                log_interval=1,
                progress_bar=False,
                reset_num_timesteps=reset_timesteps
            )
            logging.info("model.learn() completed successfully!")
        except Exception as learn_error:
            logging.error(f"❌ model.learn() raised exception: {learn_error}", exc_info=True)
            logging.error(f"   Exception type: {type(learn_error).__name__}")
            raise
        final_model_path = os.path.join(dirs['checkpoints'], 'final_model')
        model.save(final_model_path)
        logging.info(f"Training completed. Final model saved to {final_model_path}")
    except KeyboardInterrupt:
        logging.info("\n🛑 Training interrupted by user (Ctrl+C)")
        try:
            interrupt_model_path = os.path.join(dirs['checkpoints'], 'interrupted_model')
            model.save(interrupt_model_path)
            logging.info(f"✅ Model saved to {interrupt_model_path}")
            if sqlite_manager and model:
                try:
                    sqlite_manager.save_checkpoint(
                        model=model,
                        timestep=model.num_timesteps,
                        metadata={'interrupted': True, 'reason': 'KeyboardInterrupt'}
                    )
                    logging.info("✅ Checkpoint saved to SQLite")
                except Exception as e:
                    logging.warning(f"Failed to save to SQLite: {e}")
        except Exception as e:
            logging.warning(f"Failed to save interrupted model: {e}")
    except RuntimeError as e:
        error_str = str(e)
        if 'HIP error' in error_str or 'invalid device function' in error_str or 'CUDA error' in error_str:
            logging.error(f"GPU error during training: {e}")
            logging.warning("⚠️  HIP/CUDA error detected - GPU operations failing")
            logging.info("   Attempting to continue training on CPU...")
            try:
                cpu_device = torch.device('cpu')
                logging.info("Moving model to CPU...")
                try:
                    model.policy = model.policy.to(cpu_device)
                except Exception as e:
                    logging.warning(f"Failed to move policy to CPU: {e}, trying alternative method")
                    for param in model.policy.parameters():
                        param.data = param.data.cpu()
                if hasattr(model, 'critic') and model.critic is not None:
                    try:
                        model.critic = model.critic.to(cpu_device)
                    except Exception as e:
                        logging.warning(f"Failed to move critic to CPU: {e}")
                        for param in model.critic.parameters():
                            param.data = param.data.cpu()
                if hasattr(model, 'critic_target') and model.critic_target is not None:
                    try:
                        model.critic_target = model.critic_target.to(cpu_device)
                    except Exception as e:
                        logging.warning(f"Failed to move critic_target to CPU: {e}")
                        for param in model.critic_target.parameters():
                            param.data = param.data.cpu()
                model.device = cpu_device
                if mixed_device_manager is not None:
                    mixed_device_manager.gpu_device = cpu_device
                    logging.info("   Mixed Device Manager: Switched to CPU mode")
                logging.warning("⚠️  Continuing training on CPU (GPU not compatible)")
                logging.info("   Training will be slower but should work")
                model.learn(
                    total_timesteps=total_timesteps,
                    callback=callbacks,
                    log_interval=1,
                    progress_bar=False,
                    reset_num_timesteps=reset_timesteps
                )
                logging.info("✅ model.learn() completed successfully on CPU!")
            except Exception as cpu_error:
                logging.error(f"Failed to continue on CPU: {cpu_error}", exc_info=True)
                raise
        else:
            raise
    except Exception as e:
        logging.error(f"❌ Training failed: {e}", exc_info=True)
        try:
            if model:
                error_model_path = os.path.join(dirs['checkpoints'], 'error_model')
                model.save(error_model_path)
                logging.info(f"✅ Model saved to {error_model_path} before exit")
        except Exception as save_error:
            logging.warning(f"Failed to save model on error: {save_error}")
        raise
    finally:
        cleanup_resources()
if __name__ == '__main__':
    main()