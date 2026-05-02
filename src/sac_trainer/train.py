import ray
from ray import tune
from ray.tune.registry import register_env
from ray.rllib.algorithms.sac import SAC
import os
import sys
import yaml
from pathlib import Path
from typing import Dict, Any

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from src.carla_gym_env import CarlaEnv
from src.sac_trainer.config import get_sac_config, get_training_config
from src.sac_trainer.callbacks import CarlaCallbacks


def env_creator(env_config: Dict[str, Any]):
    """Create CARLA environment instance."""
    return CarlaEnv(env_config)


def train_sac(config_path: str = None):
    """
    Train SAC agent on CARLA environment.
    
    Args:
        config_path: Path to YAML configuration file (optional)
    """
    
    # Initialize Ray
    ray.init(ignore_reinit_error=True)
    
    # Register environment
    register_env("carla_env", env_creator)
    
    # Load configuration
    if config_path and os.path.exists(config_path):
        with open(config_path, 'r') as f:
            custom_config = yaml.safe_load(f)
        print(f"Loaded custom config from {config_path}")
    else:
        custom_config = {}
    
    # Get default configurations
    sac_config = get_sac_config()
    training_config = get_training_config()
    
    # Merge custom config
    if 'sac' in custom_config:
        for key, value in custom_config['sac'].items():
            setattr(sac_config, key, value)
    
    if 'training' in custom_config:
        training_config.update(custom_config['training'])
    
    # Setup directories
    log_dir = Path(training_config['log_dir'])
    checkpoint_dir = Path(training_config['checkpoint_dir'])
    tensorboard_dir = Path(training_config['tensorboard_dir'])
    
    log_dir.mkdir(parents=True, exist_ok=True)
    checkpoint_dir.mkdir(parents=True, exist_ok=True)
    tensorboard_dir.mkdir(parents=True, exist_ok=True)
    
    # Add callbacks
    sac_config.callbacks(CarlaCallbacks)
    
    # Build algorithm
    algo = sac_config.build()
    
    print("=" * 80)
    print("Starting SAC Training on CARLA")
    print("=" * 80)
    print(f"Experiment: {training_config['experiment_name']}")
    print(f"Run: {training_config['run_name']}")
    print(f"Checkpoint dir: {checkpoint_dir}")
    print(f"Tensorboard dir: {tensorboard_dir}")
    print("=" * 80)
    
    # Training loop
    num_iterations = training_config['num_iterations']
    checkpoint_freq = training_config['checkpoint_freq']
    
    best_reward = float('-inf')
    
    try:
        for iteration in range(num_iterations):
            print(f"\n{'=' * 80}")
            print(f"Iteration {iteration + 1}/{num_iterations}")
            print(f"{'=' * 80}")
            
            # Train
            result = algo.train()
            
            # Print results
            print(f"\nTraining Results:")
            print(f"  Episode Reward Mean: {result['episode_reward_mean']:.2f}")
            print(f"  Episode Reward Min: {result['episode_reward_min']:.2f}")
            print(f"  Episode Reward Max: {result['episode_reward_max']:.2f}")
            print(f"  Episode Length Mean: {result['episode_len_mean']:.2f}")
            print(f"  Episodes This Iter: {result['episodes_this_iter']}")
            print(f"  Timesteps Total: {result['timesteps_total']}")
            
            if 'evaluation' in result:
                print(f"\nEvaluation Results:")
                print(f"  Episode Reward Mean: {result['evaluation']['episode_reward_mean']:.2f}")
                print(f"  Episode Length Mean: {result['evaluation']['episode_len_mean']:.2f}")
            
            # Save checkpoint
            if (iteration + 1) % checkpoint_freq == 0:
                checkpoint_path = algo.save(checkpoint_dir)
                print(f"\nCheckpoint saved: {checkpoint_path}")
                
                # Save best model
                if result['episode_reward_mean'] > best_reward:
                    best_reward = result['episode_reward_mean']
                    best_checkpoint_path = checkpoint_dir / "best_model"
                    best_checkpoint_path.mkdir(exist_ok=True)
                    algo.save(str(best_checkpoint_path))
                    print(f"New best model saved! Reward: {best_reward:.2f}")
            
            # Log to file
            log_file = log_dir / f"{training_config['run_name']}.log"
            with open(log_file, 'a') as f:
                f.write(f"Iteration {iteration + 1}: "
                       f"reward_mean={result['episode_reward_mean']:.2f}, "
                       f"reward_min={result['episode_reward_min']:.2f}, "
                       f"reward_max={result['episode_reward_max']:.2f}, "
                       f"len_mean={result['episode_len_mean']:.2f}\n")
    
    except KeyboardInterrupt:
        print("\n\nTraining interrupted by user")
    
    finally:
        # Save final checkpoint
        if training_config['checkpoint_at_end']:
            final_checkpoint = algo.save(checkpoint_dir / "final_model")
            print(f"\nFinal checkpoint saved: {final_checkpoint}")
        
        # Cleanup
        algo.stop()
        ray.shutdown()
        
        print("\n" + "=" * 80)
        print("Training completed!")
        print("=" * 80)


def resume_training(checkpoint_path: str, num_iterations: int = 100):
    """
    Resume training from a checkpoint.
    
    Args:
        checkpoint_path: Path to checkpoint directory
        num_iterations: Number of additional iterations to train
    """
    
    # Initialize Ray
    ray.init(ignore_reinit_error=True)
    
    # Register environment
    register_env("carla_env", env_creator)
    
    # Restore algorithm
    print(f"Restoring from checkpoint: {checkpoint_path}")
    algo = SAC.from_checkpoint(checkpoint_path)
    
    print("=" * 80)
    print("Resuming SAC Training on CARLA")
    print("=" * 80)
    
    try:
        for iteration in range(num_iterations):
            print(f"\nIteration {iteration + 1}/{num_iterations}")
            result = algo.train()
            
            print(f"Episode Reward Mean: {result['episode_reward_mean']:.2f}")
            print(f"Episode Length Mean: {result['episode_len_mean']:.2f}")
            
            if (iteration + 1) % 10 == 0:
                checkpoint = algo.save()
                print(f"Checkpoint saved: {checkpoint}")
    
    except KeyboardInterrupt:
        print("\nTraining interrupted by user")
    
    finally:
        algo.stop()
        ray.shutdown()


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Train SAC on CARLA")
    parser.add_argument("--config", type=str, default=None,
                       help="Path to YAML config file")
    parser.add_argument("--resume", type=str, default=None,
                       help="Path to checkpoint to resume from")
    parser.add_argument("--iterations", type=int, default=1000,
                       help="Number of training iterations")
    
    args = parser.parse_args()
    
    if args.resume:
        resume_training(args.resume, args.iterations)
    else:
        train_sac(args.config)
