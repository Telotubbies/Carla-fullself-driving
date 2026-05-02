import ray
from ray.rllib.algorithms.sac import SAC
from ray.tune.registry import register_env
import numpy as np
import sys
from pathlib import Path
from typing import Dict, List
import json

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from src.carla_gym_env import CarlaEnv


def env_creator(env_config: Dict):
    """Create CARLA environment instance."""
    return CarlaEnv(env_config)


def evaluate_model(checkpoint_path: str, num_episodes: int = 10, render: bool = False):
    """
    Evaluate a trained SAC model.
    
    Args:
        checkpoint_path: Path to model checkpoint
        num_episodes: Number of episodes to evaluate
        render: Whether to render the environment
    
    Returns:
        Dictionary with evaluation metrics
    """
    
    # Initialize Ray
    ray.init(ignore_reinit_error=True)
    
    # Register environment
    register_env("carla_env", env_creator)
    
    # Load model
    print(f"Loading model from: {checkpoint_path}")
    algo = SAC.from_checkpoint(checkpoint_path)
    
    # Get environment config
    env_config = algo.config.env_config
    env = CarlaEnv(env_config)
    
    # Evaluation metrics
    episode_rewards = []
    episode_lengths = []
    collision_counts = []
    success_counts = []
    
    print("=" * 80)
    print(f"Evaluating model for {num_episodes} episodes")
    print("=" * 80)
    
    try:
        for episode in range(num_episodes):
            obs, info = env.reset()
            done = False
            truncated = False
            episode_reward = 0.0
            episode_length = 0
            collision_occurred = False
            
            print(f"\nEpisode {episode + 1}/{num_episodes}")
            
            while not (done or truncated):
                # Get action from policy
                action = algo.compute_single_action(obs, explore=False)
                
                # Step environment
                obs, reward, done, truncated, info = env.step(action)
                
                episode_reward += reward
                episode_length += 1
                
                if info.get('collision', False):
                    collision_occurred = True
                
                if render:
                    env.render()
            
            # Record metrics
            episode_rewards.append(episode_reward)
            episode_lengths.append(episode_length)
            collision_counts.append(1 if collision_occurred else 0)
            success_counts.append(1 if not collision_occurred and episode_length > 100 else 0)
            
            print(f"  Reward: {episode_reward:.2f}")
            print(f"  Length: {episode_length}")
            print(f"  Collision: {'Yes' if collision_occurred else 'No'}")
            print(f"  Success: {'Yes' if success_counts[-1] else 'No'}")
    
    finally:
        env.close()
        algo.stop()
        ray.shutdown()
    
    # Calculate statistics
    results = {
        'num_episodes': num_episodes,
        'mean_reward': np.mean(episode_rewards),
        'std_reward': np.std(episode_rewards),
        'min_reward': np.min(episode_rewards),
        'max_reward': np.max(episode_rewards),
        'mean_length': np.mean(episode_lengths),
        'std_length': np.std(episode_lengths),
        'collision_rate': np.mean(collision_counts),
        'success_rate': np.mean(success_counts),
        'episode_rewards': episode_rewards,
        'episode_lengths': episode_lengths,
    }
    
    # Print summary
    print("\n" + "=" * 80)
    print("Evaluation Summary")
    print("=" * 80)
    print(f"Episodes: {results['num_episodes']}")
    print(f"Mean Reward: {results['mean_reward']:.2f} ± {results['std_reward']:.2f}")
    print(f"Min/Max Reward: {results['min_reward']:.2f} / {results['max_reward']:.2f}")
    print(f"Mean Episode Length: {results['mean_length']:.2f} ± {results['std_length']:.2f}")
    print(f"Collision Rate: {results['collision_rate']:.2%}")
    print(f"Success Rate: {results['success_rate']:.2%}")
    print("=" * 80)
    
    return results


def save_evaluation_results(results: Dict, output_path: str):
    """Save evaluation results to JSON file."""
    with open(output_path, 'w') as f:
        json.dump(results, f, indent=2)
    print(f"\nResults saved to: {output_path}")


if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description="Evaluate SAC model on CARLA")
    parser.add_argument("checkpoint", type=str, help="Path to model checkpoint")
    parser.add_argument("--episodes", type=int, default=10,
                       help="Number of episodes to evaluate")
    parser.add_argument("--render", action="store_true",
                       help="Render the environment")
    parser.add_argument("--output", type=str, default="evaluation_results.json",
                       help="Output file for results")
    
    args = parser.parse_args()
    
    results = evaluate_model(args.checkpoint, args.episodes, args.render)
    save_evaluation_results(results, args.output)
