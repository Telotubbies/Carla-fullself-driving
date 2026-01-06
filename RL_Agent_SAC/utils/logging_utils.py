import logging
import os
from pathlib import Path
from datetime import datetime
def setup_logging(log_file: str = None, level: int = logging.INFO):
    
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    console_handler = logging.StreamHandler()
    console_handler.setLevel(level)
    console_handler.setFormatter(formatter)
    handlers = [console_handler]
    if log_file:
        os.makedirs(os.path.dirname(log_file), exist_ok=True)
        file_handler = logging.FileHandler(log_file)
        file_handler.setLevel(level)
        file_handler.setFormatter(formatter)
        handlers.append(file_handler)
        log_dir = os.path.dirname(log_file)
        dashboard_log = os.path.join(log_dir, 'rl_training_new.log')
        try:
            dashboard_handler = logging.FileHandler(dashboard_log, mode='a')
            dashboard_handler.setLevel(level)
            dashboard_handler.setFormatter(formatter)
            handlers.append(dashboard_handler)
        except Exception:
            pass
    logging.basicConfig(
        level=level,
        handlers=handlers,
        force=True
    )
class TrainingLogger:
    
    def __init__(self, log_dir: str):
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.metrics = {
            'episode_rewards': [],
            'episode_lengths': [],
            'collisions': [],
            'speeds': []
        }
    def log_episode(self, reward: float, length: int, info: dict):
        
        self.metrics['episode_rewards'].append(reward)
        self.metrics['episode_lengths'].append(length)
        if 'collision' in info:
            self.metrics['collisions'].append(1 if info['collision'] else 0)
        if 'speed' in info:
            self.metrics['speeds'].append(info['speed'])
    def get_statistics(self) -> dict:
        
        stats = {}
        if self.metrics['episode_rewards']:
            stats['mean_reward'] = sum(self.metrics['episode_rewards']) / len(self.metrics['episode_rewards'])
            stats['max_reward'] = max(self.metrics['episode_rewards'])
            stats['min_reward'] = min(self.metrics['episode_rewards'])
        if self.metrics['episode_lengths']:
            stats['mean_length'] = sum(self.metrics['episode_lengths']) / len(self.metrics['episode_lengths'])
        if self.metrics['collisions']:
            stats['collision_rate'] = sum(self.metrics['collisions']) / len(self.metrics['collisions'])
        if self.metrics['speeds']:
            stats['mean_speed'] = sum(self.metrics['speeds']) / len(self.metrics['speeds'])
        return stats