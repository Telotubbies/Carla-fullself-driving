"""
Data Collector
เก็บ expert demonstrations สำหรับ imitation learning
"""

import numpy as np
import pickle
from pathlib import Path
from typing import Dict, List, Any, Optional
from .expert_controller import ExpertController


class DataCollector:
    """เก็บ demonstrations จาก expert controller"""
    
    def __init__(self, save_dir: str = "data/expert_demos"):
        """
        Args:
            save_dir: directory สำหรับบันทึกข้อมูล
        """
        self.save_dir = Path(save_dir)
        self.save_dir.mkdir(parents=True, exist_ok=True)
        
        self.demonstrations = []
        self.current_episode = {
            'observations': [],
            'actions': [],
            'rewards': [],
            'dones': [],
            'infos': []
        }
        
        self.episode_count = 0
        
    def start_episode(self):
        """เริ่ม episode ใหม่"""
        self.current_episode = {
            'observations': [],
            'actions': [],
            'rewards': [],
            'dones': [],
            'infos': []
        }
    
    def add_step(
        self,
        observation: Dict[str, np.ndarray],
        action: np.ndarray,
        reward: float,
        done: bool,
        info: Dict[str, Any]
    ):
        """เพิ่ม step ลงใน episode ปัจจุบัน"""
        self.current_episode['observations'].append(observation)
        self.current_episode['actions'].append(action)
        self.current_episode['rewards'].append(reward)
        self.current_episode['dones'].append(done)
        self.current_episode['infos'].append(info)
    
    def end_episode(self, success: bool = True):
        """จบ episode และบันทึก (ถ้า success)"""
        if success and len(self.current_episode['observations']) > 0:
            # คำนวณ episode statistics
            episode_data = {
                'observations': self.current_episode['observations'],
                'actions': self.current_episode['actions'],
                'rewards': self.current_episode['rewards'],
                'dones': self.current_episode['dones'],
                'infos': self.current_episode['infos'],
                'total_reward': sum(self.current_episode['rewards']),
                'length': len(self.current_episode['observations']),
                'success': success
            }
            
            self.demonstrations.append(episode_data)
            self.episode_count += 1
            
            return True
        
        return False
    
    def save(self, filename: Optional[str] = None):
        """บันทึก demonstrations ลง disk"""
        if filename is None:
            filename = f"expert_demos_{self.episode_count}_episodes.pkl"
        
        filepath = self.save_dir / filename
        
        with open(filepath, 'wb') as f:
            pickle.dump(self.demonstrations, f)
        
        print(f"✅ Saved {len(self.demonstrations)} demonstrations to {filepath}")
        return str(filepath)
    
    def load(self, filepath: str):
        """โหลด demonstrations จาก disk"""
        with open(filepath, 'rb') as f:
            self.demonstrations = pickle.load(f)
        
        self.episode_count = len(self.demonstrations)
        print(f"✅ Loaded {self.episode_count} demonstrations from {filepath}")
    
    def get_statistics(self) -> Dict[str, Any]:
        """คำนวณสถิติของ demonstrations"""
        if len(self.demonstrations) == 0:
            return {}
        
        total_rewards = [ep['total_reward'] for ep in self.demonstrations]
        lengths = [ep['length'] for ep in self.demonstrations]
        
        return {
            'num_episodes': len(self.demonstrations),
            'avg_reward': np.mean(total_rewards),
            'std_reward': np.std(total_rewards),
            'avg_length': np.mean(lengths),
            'std_length': np.std(lengths),
            'total_steps': sum(lengths)
        }
    
    def print_statistics(self):
        """แสดงสถิติ"""
        stats = self.get_statistics()
        
        if not stats:
            print("No demonstrations collected yet")
            return
        
        print("\n" + "="*80)
        print("📊 EXPERT DEMONSTRATIONS STATISTICS")
        print("="*80)
        print(f"Episodes: {stats['num_episodes']}")
        print(f"Total Steps: {stats['total_steps']}")
        print(f"Avg Reward: {stats['avg_reward']:.2f} ± {stats['std_reward']:.2f}")
        print(f"Avg Length: {stats['avg_length']:.1f} ± {stats['std_length']:.1f}")
        print("="*80 + "\n")
    
    def get_flat_data(self) -> Dict[str, np.ndarray]:
        """แปลง demonstrations เป็น flat arrays สำหรับ training"""
        if len(self.demonstrations) == 0:
            return {}
        
        all_obs_lidar = []
        all_obs_ego = []
        all_obs_camera = []
        all_actions = []
        
        for episode in self.demonstrations:
            for obs, action in zip(episode['observations'], episode['actions']):
                all_obs_lidar.append(obs['lidar_bev'])
                all_obs_ego.append(obs['ego_state'])
                if 'camera' in obs:
                    all_obs_camera.append(obs['camera'])
                all_actions.append(action)
        
        data = {
            'lidar_bev': np.array(all_obs_lidar),
            'ego_state': np.array(all_obs_ego),
            'actions': np.array(all_actions)
        }
        
        if all_obs_camera:
            data['camera'] = np.array(all_obs_camera)
        
        return data
    
    def clear(self):
        """ลบ demonstrations ทั้งหมด"""
        self.demonstrations = []
        self.episode_count = 0
        print("🗑️ Cleared all demonstrations")


def collect_expert_demonstrations(
    env,
    expert: ExpertController,
    num_episodes: int,
    save_dir: str = "data/expert_demos",
    verbose: bool = True
) -> DataCollector:
    """
    เก็บ expert demonstrations
    
    Args:
        env: CARLA environment
        expert: Expert controller
        num_episodes: จำนวน episodes ที่ต้องการเก็บ
        save_dir: directory สำหรับบันทึก
        verbose: แสดงความคืบหน้า
        
    Returns:
        DataCollector with collected demonstrations
    """
    collector = DataCollector(save_dir)
    
    successful_episodes = 0
    total_attempts = 0
    
    print(f"\n{'='*80}")
    print(f"🎓 COLLECTING EXPERT DEMONSTRATIONS")
    print(f"{'='*80}")
    print(f"Target: {num_episodes} successful episodes")
    print(f"{'='*80}\n")
    
    while successful_episodes < num_episodes:
        total_attempts += 1
        
        # Reset environment
        obs, info = env.reset()
        collector.start_episode()
        expert.reset()
        
        done = False
        truncated = False
        episode_reward = 0
        step = 0
        
        while not (done or truncated):
            # Get expert action
            action = expert.get_action(env.vehicle, env.map)
            
            # Execute action
            next_obs, reward, done, truncated, info = env.step(action)
            
            # Record step
            collector.add_step(obs, action, reward, done or truncated, info)
            
            obs = next_obs
            episode_reward += reward
            step += 1
        
        # Check if episode was successful (no collision)
        success = not info.get('collision', False)
        
        if collector.end_episode(success):
            successful_episodes += 1
            
            if verbose:
                print(f"✅ Episode {successful_episodes}/{num_episodes} "
                      f"(attempt {total_attempts}): "
                      f"reward={episode_reward:.1f}, steps={step}")
        else:
            if verbose:
                print(f"❌ Episode failed (attempt {total_attempts}): collision")
    
    # Save demonstrations
    collector.save()
    collector.print_statistics()
    
    return collector
