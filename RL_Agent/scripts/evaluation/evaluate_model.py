
import os
import time
import argparse
import random
import logging
import json
import numpy as np
from datetime import datetime
from pathlib import Path
import sys

# Append path
sys.path.insert(0, str(Path(__file__).parent.parent))

from carla_env.carla_rl_env import CarlaRLEnv
from stable_baselines3 import PPO

# Setup logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s | %(levelname)s | %(message)s')
logger = logging.getLogger("Evaluator")

def load_config(config_path):
    import yaml
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)

def run_evaluation(checkpoint_path, config_path, n_episodes=5, town=None):
    """Run evaluation on a specific checkpoint."""
    logger.info(f"Starting evaluation for {checkpoint_path}")
    
    # Load config
    config = load_config(config_path)
    
    # Override town if specified (to test generalization)
    if town:
        config['environment']['town'] = town
        logger.info(f"Overriding town to {town}")
        
    # Setup Env
    try:
        env = CarlaRLEnv(config, port=2000) # Ensure port matching or distinct if parallel
        
        # Load Model
        model = PPO.load(checkpoint_path, env=env)
        
        metrics = {
            'rewards': [],
            'lengths': [],
            'success_rate': 0.0,
            'collisions': 0,
            'distances': []
        }
        
        success_count = 0
        
        for ep in range(n_episodes):
            obs, info = env.reset()
            done = False
            ep_reward = 0
            ep_len = 0
            
            while not done:
                action, _ = model.predict(obs, deterministic=True)
                obs, reward, done, _, info = env.step(action)
                ep_reward += reward
                ep_len += 1
                
            metrics['rewards'].append(ep_reward)
            metrics['lengths'].append(ep_len)
            metrics['distances'].append(info.get('total_distance', 0.0))
            
            if not info.get('collision', False) and info.get('total_distance', 0) > 100: 
                 # Basic success criteria (no crash + distance > 100m)
                 # Or use info.get('goal_reached', False) if available
                 success_count += 1
            
            if info.get('collision', False):
                metrics['collisions'] += 1
                
            logger.info(f"Episode {ep+1}: Reward={ep_reward:.1f}, Dist={metrics['distances'][-1]:.1f}m")
            
        metrics['success_rate'] = success_count / n_episodes
        metrics['mean_reward'] = float(np.mean(metrics['rewards']))
        metrics['mean_length'] = float(np.mean(metrics['lengths']))
        metrics['mean_distance'] = float(np.mean(metrics['distances']))
        
        env.close()
        
        # Save Report
        report = {
            'checkpoint': str(checkpoint_path),
            'timestamp': datetime.now().isoformat(),
            'metrics': metrics
        }
        
        log_dir = Path("logs/evaluations")
        log_dir.mkdir(parents=True, exist_ok=True)
        
        report_file = log_dir / f"eval_report_{Path(checkpoint_path).stem}.json"
        with open(report_file, 'w') as f:
            json.dump(report, f, indent=4)
            
        logger.info(f"Evaluation complete. Report saved: {report_file}")
        return metrics
        
    except Exception as e:
        logger.error(f"Evaluation failed: {e}", exc_info=True)
        return None

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint", type=str, required=True, help="Path to checkpoint zip")
    parser.add_argument("--config", type=str, default="config/phase1_accelerated_learning.yaml")
    parser.add_argument("--episodes", type=int, default=3)
    parser.add_argument("--town", type=str, default=None)
    
    args = parser.parse_args()
    
    run_evaluation(args.checkpoint, args.config, args.episodes, args.town)
