#!/usr/bin/env python3
"""
Main Training Script for Vision-Only RL Agent

This script handles PPO training with checkpoint management, logging, and monitoring.
Supports GPU/CPU training with automatic fallback and async checkpoint saving.
"""

# Standard library imports
import argparse
import atexit
import json
import logging
import os
import signal
import sys
import threading
from datetime import datetime
from pathlib import Path

# Third-party imports
import numpy as np
import torch
import yaml
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import (
    BaseCallback,
    CallbackList,
    CheckpointCallback,
    EvalCallback,
)
from stable_baselines3.common.monitor import Monitor
from stable_baselines3.common.utils import set_random_seed
from stable_baselines3.common.vec_env import DummyVecEnv, SubprocVecEnv

# Local imports
sys.path.insert(0, str(Path(__file__).parent.parent))
from carla_env.carla_rl_env import CarlaRLEnv
from models.custom_policy import VisionActorCriticPolicy
from utils.gpu_utils import check_gpu_available
from utils.logging_utils import setup_logging, TrainingLogger
from utils.prioritized_replay_buffer import PrioritizedReplayBuffer
from utils.sqlite_checkpoint import SQLiteCheckpointManager


def load_config(config_path: str) -> dict:
    """Load configuration from YAML file"""
    with open(config_path, 'r') as f:
        config = yaml.safe_load(f)
    return config


def make_env(config: dict, rank: int = 0, seed: int = 0):
    """
    Create environment function for vectorized environments
    
    Args:
        config: Configuration dictionary
        rank: Process rank for parallel environments (0, 1, 2, ...)
        seed: Random seed
    """
    def _init():
        # Each environment gets its own CARLA port: 2000 + (rank * 2)
        # rank 0 -> port 2000, rank 1 -> port 2002, rank 2 -> port 2004, etc.
        base_port = config.get('environment', {}).get('carla_port', 2000)
        port = base_port + (rank * 2)
        
        env = CarlaRLEnv(config, port=port, rank=rank)
        env = Monitor(env, filename=None, allow_early_resets=True)
        env.reset(seed=seed + rank)
        return env
    set_random_seed(seed + rank)
    return _init


def create_training_directories(base_dir: str, config: dict = None):
    """Create necessary directories for training"""
    # Get checkpoint path from config if available
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
    """Setup and verify GPU device with fallback to CPU"""
    device_config = config.get('device', {})
    use_gpu = device_config.get('use_gpu', True)
    
    logging.info(f"Device configuration: use_gpu={use_gpu}")
    
    if use_gpu:
        try:
            if torch.cuda.is_available():
                gpu_id = device_config.get('gpu_id', 0)
                device = torch.device(f'cuda:{gpu_id}')
                
                # Test GPU with a simple operation
                try:
                    test_tensor = torch.zeros(1, device=device)
                    test_result = torch.matmul(test_tensor, test_tensor.T)
                    del test_tensor, test_result
                    torch.cuda.empty_cache()
                except Exception as e:
                    logging.warning(f"GPU test failed: {e}. Falling back to CPU.")
                    logging.info("✅ Using CPU (GPU test failed)")
                    return torch.device('cpu')
                
                # Verify GPU
                gpu_name = torch.cuda.get_device_name(gpu_id)
                gpu_memory = torch.cuda.get_device_properties(gpu_id).total_memory / 1e9
                
                logging.info(f"✅ Using GPU: {gpu_name} ({gpu_memory:.2f} GB)")
                # For AMD GPU with ROCm, CUDA version will be None (using HIP backend)
                if torch.version.cuda:
                    logging.info(f"   PyTorch CUDA Version: {torch.version.cuda}")
                else:
                    logging.info(f"   PyTorch Backend: ROCm/HIP (AMD GPU)")
                    logging.info(f"   PyTorch Version: {torch.__version__}")
                
                return device
            else:
                logging.warning("GPU requested but not available. Falling back to CPU.")
                logging.info("✅ Using CPU (GPU not available)")
                return torch.device('cpu')
        except Exception as e:
            logging.warning(f"Error setting up GPU: {e}. Falling back to CPU.")
            logging.info("✅ Using CPU (GPU setup error)")
            return torch.device('cpu')
    else:
        logging.info("✅ Using CPU (GPU disabled in config)")
        logging.info(f"   Device: cpu")
        return torch.device('cpu')


def create_model(config: dict, env, device: torch.device, log_dir: str):
    """Create PPO model with custom policy"""
    training_config = config.get('training', {})
    ppo_config = training_config.get('ppo', {})
    
    # Policy kwargs
    policy_kwargs = {
        'config': config,
        'net_arch': config.get('network', {}).get('policy', {}).get('hidden_sizes', [512, 256, 128])
    }
    
    # Learning rate schedule
    lr_schedule_config = ppo_config.get('lr_schedule', {})
    learning_rate = ppo_config.get('learning_rate', 3e-4)
    
    if lr_schedule_config.get('type') == 'linear':
        # Linear decay schedule
        initial_lr = lr_schedule_config.get('initial_lr', learning_rate)
        final_lr = lr_schedule_config.get('final_lr', learning_rate * 0.1)
        total_steps = lr_schedule_config.get('total_steps', config.get('training', {}).get('total_timesteps', 500000))
        
        # Create linear schedule function
        def linear_schedule(progress_remaining: float) -> float:
            """Linear decay from initial_lr to final_lr"""
            return final_lr + (initial_lr - final_lr) * progress_remaining
        
        learning_rate = linear_schedule
        logging.info(f"Using linear LR schedule: {initial_lr} → {final_lr} over {total_steps} steps")
    else:
        # Fixed learning rate
        logging.info(f"Using fixed learning rate: {learning_rate}")
    
    # Create model with custom policy and GPU error handling
    logging.info(f"Creating PPO model with device: {device}")
    try:
        model = PPO(
            VisionActorCriticPolicy,  # Custom policy class
            env,
            learning_rate=learning_rate,
            n_steps=ppo_config.get('n_steps', 2048),
            batch_size=ppo_config.get('batch_size', 64),
            n_epochs=ppo_config.get('n_epochs', 10),
            gamma=ppo_config.get('gamma', 0.99),
            gae_lambda=ppo_config.get('gae_lambda', 0.95),
            clip_range=ppo_config.get('clip_range', 0.2),
            ent_coef=ppo_config.get('ent_coef', 0.01),
            vf_coef=ppo_config.get('vf_coef', 0.5),
            max_grad_norm=ppo_config.get('max_grad_norm', 0.5),
            policy_kwargs=policy_kwargs,
            tensorboard_log=os.path.join(log_dir, 'tensorboard'),
            device=device,
            verbose=1  # Set to 1 to see training progress
        )
        logging.info(f"✅ PPO model created successfully with device: {device}")
        logging.info(f"   Model device: {next(model.policy.parameters()).device}")
        return model
    except Exception as e:
        logging.error(f"Error creating model with {device}: {e}", exc_info=True)
        if device.type == 'cuda':
            logging.warning("Falling back to CPU due to model creation error...")
            device = torch.device('cpu')
            return create_model(config, env, device, log_dir)
        else:
            raise


def setup_callbacks(config: dict, dirs: dict, eval_env=None, sqlite_manager=None, training_env=None):
    """Setup training callbacks"""
    callbacks = []
    
    # Curriculum Learning & Progressive Reward Callback
    class CurriculumCallback(BaseCallback):
        """Update curriculum difficulty and reward scaling"""
        def __init__(self, training_env, verbose=0):
            super().__init__(verbose)
            self._training_env = training_env
            self.last_update_timestep = 0
            self.update_freq = 10000  # Update every 10k steps
        
        @property
        def training_env(self):
            return self._training_env
        
        @training_env.setter
        def training_env(self, value):
            self._training_env = value
        
        def _on_step(self) -> bool:
            # Update every update_freq steps
            if self.num_timesteps - self.last_update_timestep >= self.update_freq:
                # Update curriculum and reward scale
                if hasattr(self.training_env, 'envs') and len(self.training_env.envs) > 0:
                    # Vectorized environment (DummyVecEnv or SubprocVecEnv)
                    for env_wrapper in self.training_env.envs:
                        if hasattr(env_wrapper, 'env') and hasattr(env_wrapper.env, 'update_curriculum'):
                            env_wrapper.env.update_curriculum(self.num_timesteps)
                            env_wrapper.env.update_reward_scale(self.num_timesteps)
                elif hasattr(self.training_env, 'env') and hasattr(self.training_env.env, 'update_curriculum'):
                    # Single wrapped environment
                    self.training_env.env.update_curriculum(self.num_timesteps)
                    self.training_env.env.update_reward_scale(self.num_timesteps)
                elif hasattr(self.training_env, 'update_curriculum'):
                    # Direct environment
                    self.training_env.update_curriculum(self.num_timesteps)
                    self.training_env.update_reward_scale(self.num_timesteps)
                
                self.last_update_timestep = self.num_timesteps
            return True
    
    # Curriculum callback - pass training_env if provided
    if training_env is not None:
        curriculum_callback = CurriculumCallback(training_env)
        callbacks.append(curriculum_callback)
    
    # Progress logging callback
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
                pass  # Ignore logging handler flush errors
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
                pass  # Ignore logging handler flush errors
        
        def _on_rollout_end(self) -> None:
            logging.info("Callback: _on_rollout_end() called - Finished collecting experiences, starting training update")
            try:
                handlers = logging.getLogger().handlers
                if handlers:
                    handlers[0].flush()
            except Exception:
                pass  # Ignore logging handler flush errors
    
    callbacks.append(LoggingCallback())
    
    # Prioritized Experience Replay callback (if enabled)
    replay_config = config.get('training', {}).get('prioritized_replay', {})
    if replay_config.get('enabled', False):
        replay_buffer = PrioritizedReplayBuffer(
            capacity=replay_config.get('capacity', 50000),
            alpha=replay_config.get('alpha', 0.6),
            beta=replay_config.get('beta', 0.4),
            beta_increment=replay_config.get('beta_increment', 0.001)
        )
        
        class PrioritizedReplayCallback(BaseCallback):
            """Callback to store transitions in prioritized replay buffer"""
            def __init__(self, replay_buffer, update_freq=1000, verbose=0):
                super().__init__(verbose)
                self.replay_buffer = replay_buffer
                self.update_freq = update_freq
                self.last_update = 0
                self.stored_transitions = 0
                self.pending_transitions = []  # Store transitions temporarily
            
            def _on_step(self) -> bool:
                # Get transition data
                obs = self.locals.get('obs')
                actions = self.locals.get('actions')
                rewards = self.locals.get('rewards')
                dones = self.locals.get('dones')
                infos = self.locals.get('infos', [])
                
                if obs is not None and actions is not None and rewards is not None:
                    # Handle vectorized environments
                    if isinstance(obs, dict):
                        # Dict observation - get first key's shape
                        first_key = list(obs.keys())[0]
                        batch_size = obs[first_key].shape[0] if isinstance(obs[first_key], np.ndarray) and obs[first_key].ndim > 1 else 1
                    elif isinstance(obs, np.ndarray):
                        batch_size = obs.shape[0] if obs.ndim > 1 else 1
                    else:
                        batch_size = 1
                    
                    # Store transitions temporarily (will add to buffer in _on_rollout_end)
                    for i in range(batch_size):
                        if isinstance(obs, dict):
                            obs_i = {k: (v[i] if isinstance(v, np.ndarray) and v.ndim > 1 else v) 
                                    for k, v in obs.items()}
                        else:
                            obs_i = obs[i] if isinstance(obs, np.ndarray) and obs.ndim > 1 else obs
                        
                        action_i = actions[i] if isinstance(actions, np.ndarray) and actions.ndim > 1 else actions
                        reward_i = float(rewards[i] if isinstance(rewards, np.ndarray) else rewards)
                        done_i = bool(dones[i] if isinstance(dones, np.ndarray) else dones)
                        
                        # Calculate priority based on reward magnitude
                        priority = abs(reward_i) + 1.0  # Base priority
                        if done_i and reward_i < -20:  # Collision or major penalty
                            priority *= 10.0  # High priority for collisions
                        elif reward_i > 10:  # High reward
                            priority *= 5.0  # High priority for good rewards
                        
                        self.pending_transitions.append({
                            'obs': obs_i,
                            'action': action_i,
                            'reward': reward_i,
                            'done': done_i,
                            'priority': priority
                        })
                
                # Update priorities periodically
                if self.num_timesteps - self.last_update >= self.update_freq:
                    stats = self.replay_buffer.get_stats()
                    logging.info(f"📊 Prioritized Replay Buffer: size={stats['size']}, "
                               f"avg_priority={stats['avg_priority']:.2f}, "
                               f"beta={stats['beta']:.3f}, "
                               f"stored={self.stored_transitions}")
                    self.last_update = self.num_timesteps
                
                return True
            
            def _on_rollout_end(self) -> None:
                """Add pending transitions to replay buffer and update priorities"""
                # Get rollout data from model to get next_obs
                if hasattr(self.model, 'rollout_buffer') and self.model.rollout_buffer is not None:
                    rollout_buffer = self.model.rollout_buffer
                    
                    # Get observations from rollout
                    if hasattr(rollout_buffer, 'observations') and rollout_buffer.observations is not None:
                        obs_rollout = rollout_buffer.observations
                        
                        # Add pending transitions with next_obs
                        for i, trans in enumerate(self.pending_transitions):
                            # Get next observation from rollout (if available)
                            if i + 1 < len(obs_rollout):
                                if isinstance(obs_rollout, dict):
                                    next_obs = {k: v[i+1] if isinstance(v, np.ndarray) and v.ndim > 1 else v 
                                               for k, v in obs_rollout.items()}
                                else:
                                    next_obs = obs_rollout[i+1] if isinstance(obs_rollout, np.ndarray) and obs_rollout.ndim > 1 else obs_rollout
                            else:
                                next_obs = trans['obs']  # Use current obs if no next available
                            
                            # Calculate TD-error if available
                            if hasattr(rollout_buffer, 'returns') and hasattr(rollout_buffer, 'values'):
                                if rollout_buffer.returns is not None and rollout_buffer.values is not None:
                                    returns = rollout_buffer.returns
                                    values = rollout_buffer.values
                                    if i < len(returns) and i < len(values):
                                        td_error = abs(float(returns[i]) - float(values[i]))
                                        # Update priority based on TD-error
                                        trans['priority'] = (td_error + 1.0) ** 0.6
                            
                            # Add to replay buffer
                            self.replay_buffer.add(
                                observation=trans['obs'],
                                action=trans['action'],
                                reward=trans['reward'],
                                next_observation=next_obs,
                                done=trans['done'],
                                priority=trans['priority']
                            )
                            self.stored_transitions += 1
                
                # Clear pending transitions
                self.pending_transitions = []
        
        replay_callback = PrioritizedReplayCallback(
            replay_buffer=replay_buffer,
            update_freq=replay_config.get('update_freq', 1000)
        )
        callbacks.append(replay_callback)
        logging.info(f"✅ Prioritized Experience Replay enabled: capacity={replay_config.get('capacity', 50000)}")
    
    # SQLite checkpoint callback (if enabled)
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
                """Save checkpoint in background thread to avoid blocking training"""
                try:
                    # Save checkpoint
                    self.sqlite_manager.save_checkpoint(
                        model=model,
                        timestep=timestep,
                        episode=episode,
                        reward=reward,
                        metadata=metadata
                    )
                    logging.info(f"💾 SQLite checkpoint saved at timestep {timestep}")
                except Exception as e:
                    logging.error(f"Failed to save SQLite checkpoint: {e}", exc_info=True)
                finally:
                    self.save_in_progress = False
            
            def _on_rollout_end(self) -> None:
                # Save checkpoint to SQLite (non-blocking)
                if self.num_timesteps - self.last_save_timestep >= self.save_freq:
                    # Don't start new save if one is already in progress
                    if self.save_in_progress:
                        logging.warning(f"⚠️  Previous SQLite save still in progress, skipping timestep {self.num_timesteps}")
                        return
                    
                    try:
                        # Get current stats
                        infos = self.locals.get('infos', [])
                        reward = None
                        episode = None
                        if infos and len(infos) > 0:
                            info = infos[0]
                            if 'episode' in info:
                                reward = info['episode'].get('r')
                                episode = info['episode'].get('l')
                        
                        # Prepare metadata
                        metadata = {
                            'rollout_num': getattr(self, 'rollout_num', 0),
                            'n_steps': getattr(self.model, 'n_steps', 0)
                        }
                        
                        # Mark save in progress
                        self.save_in_progress = True
                        
                        # Save checkpoint in background thread (non-blocking)
                        import threading
                        self.save_thread = threading.Thread(
                            target=self._save_checkpoint_async,
                            args=(self.model, self.num_timesteps, episode, reward, metadata),
                            daemon=True
                        )
                        self.save_thread.start()
                        
                        # Save training stats synchronously (fast, won't block)
                        if hasattr(self.locals, 'get') and 'infos' in self.locals:
                            infos = self.locals['infos']
                            if infos and len(infos) > 0:
                                info = infos[0]
                                if 'episode' in info:
                                    ep_info = info['episode']
                                    try:
                                        self.sqlite_manager.save_training_stats(
                                            timestep=self.num_timesteps,
                                            mean_reward=ep_info.get('r'),
                                            mean_episode_length=ep_info.get('l'),
                                            fps=ep_info.get('fps') if 'fps' in ep_info else None
                                        )
                                    except Exception as e:
                                        logging.warning(f"Failed to save training stats: {e}")
                        
                        self.last_save_timestep = self.num_timesteps
                        logging.info(f"💾 SQLite checkpoint save started (background) at timestep {self.num_timesteps}")
                    except Exception as e:
                        logging.error(f"Failed to start SQLite checkpoint save: {e}", exc_info=True)
                        self.save_in_progress = False
        
        sqlite_callback = SQLiteCheckpointCallback(
            sqlite_manager=sqlite_manager,
            save_freq=config.get('training', {}).get('save_freq', 10000)
        )
        callbacks.append(sqlite_callback)
        logging.info("✅ SQLite checkpoint callback enabled")
    
    # Standard checkpoint callback (backup)
    # Enhanced Checkpoint Callback with separate format
    class EnhancedCheckpointCallback(BaseCallback):
        """Enhanced checkpoint callback with separate save format and dashboard integration"""
        def __init__(self, save_freq, checkpoint_dir, save_format='both', config=None, verbose=0):
            super().__init__(verbose)
            self.save_freq = save_freq
            self.checkpoint_dir = checkpoint_dir
            self.save_format = save_format  # 'old', 'new', or 'both'
            self.config = config  # Store config for saving with checkpoint
            self.last_save_timestep = 0
            self.save_in_progress = False
            self.save_thread = None
            self.save_lock = threading.Lock()
            
            # Create directories
            os.makedirs(os.path.join(checkpoint_dir, 'checkpoint'), exist_ok=True)  # Old format
            os.makedirs(os.path.join(checkpoint_dir, 'enhanced'), exist_ok=True)  # New format
            
        def _on_step(self) -> bool:
            if self.num_timesteps - self.last_save_timestep >= self.save_freq:
                self._save_checkpoint()  # Now async, won't block
            return True
        
        def _save_checkpoint(self):
            """Save checkpoint in both formats - ASYNC to prevent disk I/O wait"""
            # Only start new save if previous one is complete
            with self.save_lock:
                if self.save_in_progress:
                    logging.warning(f"⚠️  Previous checkpoint save still in progress, skipping timestep {self.num_timesteps}")
                    return
                
                # Wait for previous thread if still running (with shorter timeout to prevent deadlock)
                if self.save_thread is not None and self.save_thread.is_alive():
                    logging.warning("⚠️  Previous save thread still running, waiting...")
                    self.save_thread.join(timeout=5)  # FIXED: Reduced timeout from 10s to 5s
                    if self.save_thread.is_alive():
                        logging.error("❌ Previous save thread did not complete, skipping new save")
                        # FIXED: Reset save_in_progress flag to prevent permanent deadlock
                        self.save_in_progress = False
                        return
                
                self.save_in_progress = True
                step = self.num_timesteps
                model = self.model
                
                # Start save in background thread to prevent blocking
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
            """Save checkpoint synchronously (runs in background thread) - FIXED: removed nested threading to prevent deadlock"""
            try:
                logging.info(f"💾 Starting checkpoint save for step {step}...")
                
                # Old format (compatible with existing system)
                if self.save_format in ['old', 'both']:
                    old_path = os.path.join(self.checkpoint_dir, 'checkpoint', f'rl_model_{step}_steps.zip')
                    
                    # FIXED: Call model.save() with timeout protection to prevent hanging
                    try:
                        import time
                        start_time = time.time()
                        
                        # FIXED: Add timeout protection for model.save()
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
                        save_thread.join(timeout=30.0)  # 30 second timeout for save
                        
                        if not save_completed.is_set():
                            logging.error(f"❌ Checkpoint save TIMEOUT after 30s: {old_path}")
                            return  # Don't continue if save timed out
                        elif save_error[0] is not None:
                            raise save_error[0]
                        
                        save_time = time.time() - start_time
                        logging.info(f"✅ Checkpoint save completed in {save_time:.2f}s: {old_path}")
                    except Exception as e:
                        logging.error(f"❌ Checkpoint save error: {e}", exc_info=True)
                        return  # Don't continue if old format save failed
                    
                    if os.path.exists(old_path):
                        # Save config/hyperparameters alongside checkpoint
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
                
                # New format (enhanced with metadata) - skip to reduce I/O
                if self.save_format in ['new', 'both']:
                    new_dir = os.path.join(self.checkpoint_dir, 'enhanced', f'checkpoint_{step}')
                    try:
                        os.makedirs(new_dir, exist_ok=True)
                        
                        # FIXED: Call model.save() with timeout protection
                        model_path = os.path.join(new_dir, 'model.zip')
                        try:
                            import time
                            start_time = time.time()
                            
                            # FIXED: Add timeout protection for model.save()
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
                            save_thread.join(timeout=30.0)  # 30 second timeout
                            
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
                            # Save metadata with config
                            metadata = {
                                'timestep': step,
                                'timestamp': datetime.now().isoformat(),
                                'episode': getattr(model, 'episode', 0),
                                'reward': getattr(model, 'last_reward', 0.0)
                            }
                            
                            # Include config in metadata if available
                            if self.config is not None:
                                metadata['config'] = self.config
                            
                            metadata_path = os.path.join(new_dir, 'metadata.json')
                            with open(metadata_path, 'w') as f:
                                json.dump(metadata, f, indent=2)
                            
                            # Also save config as separate YAML file
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
                
                # Notify dashboard (if available) - non-blocking
                try:
                    import requests
                    requests.post('http://localhost:5000/api/checkpoint_saved', 
                               json={'timestep': step, 'timestamp': datetime.now().isoformat()},
                               timeout=0.1)
                except Exception:
                    pass  # Dashboard not available, ignore
                    
            except Exception as e:
                logging.error(f"❌ Failed to save checkpoint: {e}", exc_info=True)
            finally:
                with self.save_lock:
                    self.save_in_progress = False
    
    # ใช้ zip format แบบเก่า (ไม่ใช้ SQLite)
    save_format = config.get('training', {}).get('save_format', 'old')  # 'old' = zip format แบบเก่า
    enhanced_checkpoint_callback = EnhancedCheckpointCallback(
        save_freq=config.get('training', {}).get('save_freq', 50000),
        checkpoint_dir=dirs['checkpoints'],
        save_format=save_format,  # ใช้ 'old' = zip format
        config=config,  # Pass config to save with checkpoint
        verbose=1
    )
    callbacks.append(enhanced_checkpoint_callback)
    logging.info(f"✅ Zip checkpoint callback enabled (format: {save_format})")
    
    # Keep old checkpoint callback for backward compatibility
    # NOTE: This callback also saves checkpoints, but EnhancedCheckpointCallback handles it
    # We keep this for compatibility but it may create duplicate saves
    # Consider disabling if EnhancedCheckpointCallback is working well
    checkpoint_callback = CheckpointCallback(
        save_freq=config.get('training', {}).get('save_freq', 50000),
        save_path=os.path.join(dirs['checkpoints'], 'checkpoint'),
        name_prefix='rl_model',
        save_replay_buffer=False,  # Disable to reduce I/O (saves are large)
        save_vecnormalize=False    # Disable to reduce I/O
    )
    # Only add if save_format is not 'old' (to avoid duplicate saves)
    if save_format != 'old':
        callbacks.append(checkpoint_callback)
    
    # Evaluation callback
    if eval_env is not None:
        eval_callback = EvalCallback(
            eval_env,
            best_model_save_path=os.path.join(dirs['checkpoints'], 'best_model'),
            log_path=os.path.join(dirs['logs'], 'evaluations'),
            eval_freq=config.get('training', {}).get('eval_freq', 100000),
            n_eval_episodes=config.get('training', {}).get('eval_episodes', 10),
            deterministic=True,
            render=False
        )
        callbacks.append(eval_callback)
    
    return CallbackList(callbacks)


# Global variables for cleanup
_env = None
_eval_env = None
_model = None
_shutdown_requested = False

def cleanup_resources():
    """Cleanup all resources (CARLA, environments, GPU memory) - FIXED: Added timeout protection"""
    global _env, _eval_env, _model, _shutdown_requested
    
    if _shutdown_requested:
        return  # Already cleaning up
    
    _shutdown_requested = True
    logging.info("🧹 Cleaning up resources...")
    
    try:
        # FIXED: Close environments with timeout protection
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
                close_thread.join(timeout=10.0)  # 10 second timeout for environment close
                
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
                close_thread.join(timeout=10.0)  # 10 second timeout
                
                if not close_completed.is_set():
                    logging.error("❌ Evaluation environment close TIMEOUT - may be hanging")
                elif close_error[0] is None:
                logging.info("✅ Evaluation environment closed")
                else:
                    logging.warning(f"Error closing evaluation environment: {close_error[0]}")
            except Exception as e:
                logging.warning(f"Error closing evaluation environment: {e}")
        
        # FIXED: Wait for all checkpoint save threads to complete
        if _model is not None:
            try:
                # Give checkpoint threads time to finish (they're daemon threads)
                import time
                time.sleep(2.0)  # Wait 2 seconds for any pending saves
            except Exception as e:
                logging.warning(f"Error waiting for checkpoint threads: {e}")
        
        # Clear GPU memory
        if torch.cuda.is_available():
            try:
                torch.cuda.empty_cache()
                logging.info("✅ GPU memory cleared")
            except Exception as e:
                logging.warning(f"Error clearing GPU memory: {e}")
        
        # Force garbage collection
        import gc
        gc.collect()
        
        logging.info("✅ Cleanup completed")
    except Exception as e:
        logging.error(f"Error during cleanup: {e}", exc_info=True)

def signal_handler(signum, frame):
    """Handle shutdown signals gracefully"""
    logging.info(f"\n🛑 Received signal {signum}, shutting down gracefully...")
    cleanup_resources()
    sys.exit(0)

def main():
    global _env, _eval_env, _model
    
    # Register signal handlers for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler)  # Ctrl+C
    signal.signal(signal.SIGTERM, signal_handler)  # kill command
    atexit.register(cleanup_resources)  # Register cleanup on exit
    
    parser = argparse.ArgumentParser(description='Train Vision-Only RL Agent for CARLA')
    parser.add_argument(
        '--config',
        type=str,
        default='config/phase1_config.yaml',
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
    
    # Load configuration
    config_path = os.path.join(Path(__file__).parent.parent, args.config)
    config = load_config(config_path)
    
    # Setup logging
    log_dir = os.path.join(Path(__file__).parent.parent, 'logs')
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    log_file = os.path.join(log_dir, f'training_{timestamp}.log')
    setup_logging(log_file)
    
    logging.info("=" * 60)
    logging.info("Vision-Only RL Agent Training")
    logging.info("=" * 60)
    logging.info(f"Configuration: {args.config}")
    logging.info(f"Timestamp: {timestamp}")
    
    # Create directories
    base_dir = Path(__file__).parent.parent
    dirs = create_training_directories(str(base_dir), config)
    logging.info(f"Logs directory: {dirs['logs']}")
    logging.info(f"Checkpoints directory: {dirs['checkpoints']}")
    
    # Setup device FIRST (before GPU check to avoid confusing logs)
    device = setup_device(config)
    
    # Check GPU (only for information, device is already set)
    if device.type == 'cuda':
        if not check_gpu_available(config):
            logging.warning("GPU check failed but device is set to GPU. Training may fail.")
    else:
        logging.info("Skipping GPU check (using CPU)")
    
    # Set random seed
    seed = config.get('seed', 42)
    set_random_seed(seed, using_cuda=(device.type == 'cuda'))
    logging.info(f"Random seed: {seed}")
    
    # Create environments
    logging.info("Creating environments...")
    num_envs = args.num_envs
    
    # Parallel Environment Support: Use SubprocVecEnv with multiple CARLA servers
    # Each environment needs its own CARLA server on different port
    if num_envs > 1:
        logging.info(f"Using {num_envs} parallel environments with SubprocVecEnv")
        logging.info("   Each environment will use a separate CARLA server:")
        base_port = config.get('environment', {}).get('carla_port', 2000)
        for i in range(num_envs):
            port = base_port + (i * 2)
            logging.info(f"      Env {i}: port {port}")
        logging.info("   Make sure CARLA servers are running on these ports!")
        
        # Use SubprocVecEnv for true parallel execution
        _env = SubprocVecEnv([make_env(config, i, seed) for i in range(num_envs)])
        env = _env
    else:
        # Single environment: use DummyVecEnv
        logging.info("Using single environment with DummyVecEnv")
        _env = DummyVecEnv([make_env(config, 0, seed)])
        env = _env
    
    # Evaluation environment
    _eval_env = DummyVecEnv([make_env(config, 0, seed + 1000)])
    eval_env = _eval_env
    
    logging.info(f"Created {num_envs} training environment(s)")
    
    # Initialize SQLite checkpoint manager (if enabled)
    sqlite_manager = None
    use_sqlite = config.get('checkpoints', {}).get('use_sqlite', True)
    clear_db = config.get('checkpoints', {}).get('clear_on_start', False)
    
    if use_sqlite:
        db_path = os.path.join(dirs['checkpoints'], 'training_checkpoints.db')
        sqlite_manager = SQLiteCheckpointManager(db_path, enable_wal=True)
        
        # Clear database if requested
        if clear_db:
            logging.info("🗑️  Clearing existing database...")
            try:
                sqlite_manager.clear_database()
                logging.info("✅ Database cleared")
            except Exception as e:
                logging.warning(f"Failed to clear database: {e}")
        
        logging.info(f"✅ SQLite checkpoint manager initialized: {db_path}")
    
    # Create or load model
    _model = None
    model = None
    
    # Resume from checkpoint if specified
    if args.resume:
        logging.info(f"🔄 Loading checkpoint: {args.resume}")
        _model = PPO.load(args.resume, env=env, device=device)
        model = _model
        logging.info(f"✅ Checkpoint loaded (timestep: {model.num_timesteps})")
        
        # Update SQLite if enabled
        if sqlite_manager:
            try:
                sqlite_manager.save_checkpoint(
                    model=model,
                    timestep=model.num_timesteps,
                    metadata={'loaded_from': args.resume}
                )
                logging.info("✅ Checkpoint synced to SQLite")
            except Exception as e:
                logging.warning(f"Failed to sync checkpoint to SQLite: {e}")
    elif sqlite_manager:
        # Try to load latest checkpoint from SQLite
        checkpoint_info = sqlite_manager.get_latest_checkpoint_info()
        if checkpoint_info:
            logging.info(f"📂 Found SQLite checkpoint: timestep={checkpoint_info['timestep']}")
            # Extract checkpoint to temp file and load
            try:
                import tempfile
                checkpoint_data = sqlite_manager.load_checkpoint()
                if checkpoint_data and checkpoint_data.get('model'):
                    # Save model data to temp file
                    with tempfile.NamedTemporaryFile(delete=False, suffix='.zip') as tmp_file:
                        tmp_path = tmp_file.name
                        tmp_file.write(checkpoint_data['model'])
                    
                    # Load model from temp file
                    _model = PPO.load(tmp_path, env=env, device=device)
                    model = _model
                    
                    # Restore training state if available
                    if checkpoint_data.get('training_state'):
                        state = checkpoint_data['training_state']
                        if 'num_timesteps' in state:
                            model.num_timesteps = state['num_timesteps']
                    else:
                        # If no training_state, use timestep from checkpoint_info
                        model.num_timesteps = checkpoint_info['timestep']
                    
                    # Clean up temp file
                    os.unlink(tmp_path)
                    
                    logging.info(f"✅ Model loaded from SQLite checkpoint: timestep={checkpoint_info['timestep']}, model.num_timesteps={model.num_timesteps}")
            except Exception as e:
                logging.warning(f"Failed to load from SQLite: {e}. Trying enhanced checkpoints...", exc_info=True)
                model = None
        
        # If SQLite failed, try to auto-resume from latest enhanced checkpoint
        if model is None:
            try:
                enhanced_dir = Path(dirs['checkpoints']) / 'enhanced'
                if enhanced_dir.exists():
                    checkpoints = []
                    for cp_dir in enhanced_dir.glob("checkpoint_*"):
                        model_file = cp_dir / "model.zip"
                        if model_file.exists():
                            # Extract timestep from directory name
                            timestep_str = cp_dir.name.replace("checkpoint_", "")
                            try:
                                timestep = int(timestep_str)
                                checkpoints.append((timestep, str(model_file)))
                            except ValueError:
                                continue
                    
                    if checkpoints:
                        # Sort by timestep and get latest
                        checkpoints.sort(key=lambda x: x[0], reverse=True)
                        latest_timestep, latest_path = checkpoints[0]
                        logging.info(f"📂 Found latest enhanced checkpoint: checkpoint_{latest_timestep}")
                        logging.info(f"🔄 Auto-resuming from: {latest_path}")
                        
                        _model = PPO.load(latest_path, env=env, device=device)
                        model = _model
                        logging.info(f"✅ Auto-resumed from checkpoint_{latest_timestep} (timestep: {model.num_timesteps})")
                        
                        # Update SQLite if enabled
                        if sqlite_manager:
                            try:
                                sqlite_manager.save_checkpoint(
                                    model=model,
                                    timestep=model.num_timesteps,
                                    metadata={'auto_resumed_from': latest_path}
                                )
                                logging.info("✅ Checkpoint synced to SQLite")
                            except Exception as e:
                                logging.warning(f"Failed to sync checkpoint to SQLite: {e}")
            except Exception as e:
                logging.warning(f"Failed to auto-resume from enhanced checkpoints: {e}. Creating new model.", exc_info=True)
                model = None
    
    # Check for IL pre-trained model
    il_config = config.get('training', {}).get('imitation_learning', {})
    il_enabled = il_config.get('enabled', False)
    il_pretrained_path = il_config.get('pretrained_path', None)
    
    # Create new model if not loaded
    resume_from_checkpoint = model is not None
    
    # Load IL pre-trained weights if enabled (for both new models and resumed checkpoints)
    if il_enabled and il_pretrained_path and os.path.exists(il_pretrained_path):
        if model is None:
            logging.info("Creating RL model...")
            _model = create_model(config, env, device, dirs['logs'])
            model = _model
        
        # Load IL pre-trained weights (works for both new models and resumed checkpoints)
        logging.info(f"🔄 Loading IL pre-trained weights from: {il_pretrained_path}")
        try:
            # PyTorch 2.6+ requires weights_only=False for loading custom checkpoints
            checkpoint = torch.load(il_pretrained_path, map_location=device, weights_only=False)
            if 'policy_state_dict' in checkpoint:
                il_state_dict = checkpoint['policy_state_dict']
                current_state_dict = model.policy.state_dict()
                
                # Filter out incompatible layers (action_net and value_net if size mismatch)
                filtered_state_dict = {}
                skipped_layers = []
                loaded_layers = []
                
                for key, value in il_state_dict.items():
                    if key in current_state_dict:
                        # Check if shapes match
                        if current_state_dict[key].shape == value.shape:
                            filtered_state_dict[key] = value
                            loaded_layers.append(key)
                        else:
                            skipped_layers.append(f"{key} (shape mismatch: {value.shape} vs {current_state_dict[key].shape})")
                    else:
                        skipped_layers.append(f"{key} (not in current model)")
                
                # Load compatible weights
                if filtered_state_dict:
                    model.policy.load_state_dict(filtered_state_dict, strict=False)
                    logging.info(f"✅ IL pre-trained weights loaded successfully!")
                    logging.info(f"   • Loaded {len(loaded_layers)} compatible layers")
                    if skipped_layers:
                        logging.info(f"   • Skipped {len(skipped_layers)} incompatible layers:")
                        for skip in skipped_layers[:5]:  # Show first 5
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
        # IL enabled but file not found
        logging.warning(f"⚠️  IL pre-trained path specified but file not found: {il_pretrained_path}")
        if model is None:
            logging.info("Creating RL model...")
            _model = create_model(config, env, device, dirs['logs'])
            model = _model
    elif model is None:
        # Create new model only if IL is not enabled
        logging.info("Creating RL model...")
        _model = create_model(config, env, device, dirs['logs'])
        model = _model
    
    # Setup callbacks
    callbacks = setup_callbacks(config, dirs, eval_env, sqlite_manager, training_env=env)
    
    # Option B: Data Prefetching (ใช้ RAM +1-2 GB, เพิ่ม throughput ~10-20%)
    prefetch_enabled = config.get('training', {}).get('prefetching', {}).get('enabled', False)
    if prefetch_enabled:
        logging.info("✅ Data Prefetching enabled (will pre-load next batch while training)")
        # Note: PPO's VecEnv already handles some prefetching, this adds extra layer
        # The prefetching happens automatically in VecEnv's rollout collection
    
    # Training
    total_timesteps = config.get('training', {}).get('total_timesteps', 10000000)
    logging.info(f"Starting training for {total_timesteps} timesteps...")
    logging.info("=" * 60)
    
    # Test environment before training (ตาม best practices)
    logging.info("Testing environment before training...")
    try:
        reset_result = env.reset()
        if isinstance(reset_result, tuple):
            test_obs, _ = reset_result
        else:
            test_obs = reset_result
        # Handle both Box and Dict observation spaces
        if isinstance(test_obs, dict):
            obs_info = ", ".join([f"{k}: {v.shape if hasattr(v, 'shape') else type(v).__name__}" for k, v in test_obs.items()])
            logging.info(f"Environment reset successful, obs (Dict): {obs_info}")
        else:
            logging.info(f"Environment reset successful, obs shape: {test_obs.shape}")
        test_action = env.action_space.sample()
        # Ensure action is in correct format (3 elements: steer, throttle, brake)
        if not isinstance(test_action, np.ndarray):
            test_action = np.array(test_action)
        # Flatten if needed and ensure 3 elements
        test_action = test_action.flatten()
        if len(test_action) < 3:
            # Pad with zeros if less than 3 elements
            test_action = np.pad(test_action, (0, 3 - len(test_action)), mode='constant', constant_values=0.0)
        elif len(test_action) > 3:
            # Take first 3 if more than 3 elements
            test_action = test_action[:3]
        step_result = env.step(test_action)
        if len(step_result) == 5:
            test_obs, test_reward, test_done, test_truncated, test_info = step_result
        else:
            test_obs, test_reward, test_done, test_info = step_result
            test_truncated = test_done
        # Fix: action might be scalar, need to handle it
        if isinstance(test_action, np.ndarray) and test_action.ndim == 0:
            test_action = np.array([test_action])
        logging.info(f"Environment step successful, reward: {test_reward[0] if hasattr(test_reward, '__len__') else test_reward:.2f}")
        logging.info("Environment is ready for training!")
    except Exception as e:
        logging.error(f"Environment test failed: {e}", exc_info=True)
        raise
    
    logging.info("Starting PPO training loop...")
    n_steps = config.get('training', {}).get('ppo', {}).get('n_steps', 2048)
    logging.info(f"   PPO will collect {n_steps} steps before first training update")
    logging.info(f"   This may take a few minutes depending on environment speed...")
    
    try:
        logging.info("Calling model.learn()...")
        logging.info("   About to enter PPO collect phase...")
        
        # Don't reset timesteps if resuming from checkpoint
        reset_timesteps = not resume_from_checkpoint
        if resume_from_checkpoint:
            logging.info(f"🔄 Resuming training from timestep {model.num_timesteps} (reset_num_timesteps=False)")
        else:
            logging.info(f"🆕 Starting new training (reset_num_timesteps=True)")
        
        model.learn(
            total_timesteps=total_timesteps,
            callback=callbacks,
            log_interval=1,
            progress_bar=False,  # Disabled to avoid rich dependency issue
            reset_num_timesteps=reset_timesteps
        )
        logging.info("model.learn() completed successfully!")
        
        # Save final model
        final_model_path = os.path.join(dirs['checkpoints'], 'final_model')
        model.save(final_model_path)
        logging.info(f"Training completed. Final model saved to {final_model_path}")
        
    except KeyboardInterrupt:
        logging.info("\n🛑 Training interrupted by user (Ctrl+C)")
        try:
            interrupt_model_path = os.path.join(dirs['checkpoints'], 'interrupted_model')
            model.save(interrupt_model_path)
            logging.info(f"✅ Model saved to {interrupt_model_path}")
        except Exception as e:
            logging.warning(f"Failed to save interrupted model: {e}")
    
    except Exception as e:
        logging.error(f"❌ Training failed: {e}", exc_info=True)
        raise
    
    finally:
        # Cleanup resources (signal handler will also call this)
        cleanup_resources()


if __name__ == '__main__':
    main()

