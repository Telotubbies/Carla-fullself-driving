"""
Curriculum Learning Stages Definition
กำหนด stages ต่างๆ สำหรับการเรียนรู้แบบค่อยเป็นค่อยไป
"""

from dataclasses import dataclass
from typing import List, Dict, Any
import numpy as np


@dataclass
class StageConfig:
    """Configuration สำหรับแต่ละ stage ของ curriculum"""
    
    name: str
    description: str
    target_episodes: int
    spawn_points: List[int]
    traffic_density: float
    reward_weights: Dict[str, float]
    success_criteria: Dict[str, float]
    
    def __repr__(self):
        return f"Stage({self.name}, episodes={self.target_episodes})"


class Stage:
    """Base class สำหรับ curriculum stage"""
    
    def __init__(self, config: StageConfig):
        self.config = config
        self.episodes_completed = 0
        self.success_count = 0
        self.metrics_history = {
            'rewards': [],
            'lane_deviations': [],
            'speeds': [],
            'collisions': [],
            'episode_lengths': []
        }
    
    def is_complete(self) -> bool:
        """ตรวจสอบว่า stage นี้เสร็จสมบูรณ์แล้วหรือยัง"""
        if self.episodes_completed < self.config.target_episodes:
            return False
        
        # ตรวจสอบ success criteria
        return self._check_success_criteria()
    
    def _check_success_criteria(self) -> bool:
        """ตรวจสอบว่าผ่าน success criteria หรือไม่"""
        if len(self.metrics_history['rewards']) < 100:
            return False
        
        # คำนวณ metrics จาก 100 episodes ล่าสุด
        recent_rewards = self.metrics_history['rewards'][-100:]
        recent_collisions = self.metrics_history['collisions'][-100:]
        recent_lane_devs = self.metrics_history['lane_deviations'][-100:]
        
        success_rate = (100 - sum(recent_collisions)) / 100.0
        avg_lane_dev = np.mean(recent_lane_devs)
        
        # ตรวจสอบตาม criteria
        criteria = self.config.success_criteria
        
        if 'success_rate' in criteria and success_rate < criteria['success_rate']:
            return False
        
        if 'max_lane_deviation' in criteria and avg_lane_dev > criteria['max_lane_deviation']:
            return False
        
        return True
    
    def record_episode(self, metrics: Dict[str, Any]):
        """บันทึกผลลัพธ์ของ episode"""
        self.episodes_completed += 1
        
        # บันทึก metrics
        self.metrics_history['rewards'].append(metrics.get('total_reward', 0))
        self.metrics_history['lane_deviations'].append(metrics.get('avg_lane_deviation', 0))
        self.metrics_history['speeds'].append(metrics.get('avg_speed', 0))
        self.metrics_history['collisions'].append(1 if metrics.get('collision', False) else 0)
        self.metrics_history['episode_lengths'].append(metrics.get('episode_length', 0))
        
        # นับ success
        if not metrics.get('collision', False):
            self.success_count += 1
    
    def get_progress(self) -> float:
        """คำนวณความคืบหน้าของ stage (0-1)"""
        return min(self.episodes_completed / self.config.target_episodes, 1.0)
    
    def get_statistics(self) -> Dict[str, Any]:
        """คำนวณสถิติของ stage"""
        if self.episodes_completed == 0:
            return {}
        
        recent_n = min(100, len(self.metrics_history['rewards']))
        
        return {
            'episodes_completed': self.episodes_completed,
            'success_count': self.success_count,
            'success_rate': self.success_count / self.episodes_completed,
            'avg_reward': np.mean(self.metrics_history['rewards'][-recent_n:]),
            'avg_lane_deviation': np.mean(self.metrics_history['lane_deviations'][-recent_n:]),
            'avg_speed': np.mean(self.metrics_history['speeds'][-recent_n:]),
            'collision_rate': np.mean(self.metrics_history['collisions'][-recent_n:]),
            'avg_episode_length': np.mean(self.metrics_history['episode_lengths'][-recent_n:])
        }


def create_default_stages() -> List[StageConfig]:
    """สร้าง default curriculum stages"""
    
    stages = [
        # Stage 1: Basic Control
        StageConfig(
            name="basic_control",
            description="เรียนรู้การควบคุมพื้นฐาน - ขับตรง, อยู่ในเลน",
            target_episodes=500,
            spawn_points=[0, 1, 2],  # ใช้ spawn points ที่เป็นถนนตรง
            traffic_density=0.0,  # ไม่มีรถอื่น
            reward_weights={
                'w_progress': 0.5,
                'w_comfort': 0.1,
                'w_collision': 200.0,
                'w_lane_deviation': 1.0,  # เน้น lane keeping
                'w_speed': 0.3,
            },
            success_criteria={
                'success_rate': 0.70,  # 70% ไม่ชน
                'max_lane_deviation': 0.5,  # เฉลี่ยไม่เกิน 0.5m
            }
        ),
        
        # Stage 2: Navigation
        StageConfig(
            name="navigation",
            description="เรียนรู้การเลี้ยว และตามเส้นทาง",
            target_episodes=1000,
            spawn_points=[0, 1, 2, 3, 4, 5],  # เพิ่ม spawn points ที่มีโค้ง
            traffic_density=0.1,  # รถอื่นเล็กน้อย
            reward_weights={
                'w_progress': 1.0,  # เน้น progress
                'w_comfort': 0.2,
                'w_collision': 200.0,
                'w_lane_deviation': 0.5,
                'w_speed': 0.3,
            },
            success_criteria={
                'success_rate': 0.60,  # 60% ไม่ชน
                'max_lane_deviation': 0.8,
            }
        ),
        
        # Stage 3: Complex Scenarios
        StageConfig(
            name="complex_scenarios",
            description="จัดการกับสถานการณ์ซับซ้อน - รถอื่น, คนเดิน",
            target_episodes=1500,
            spawn_points=list(range(20)),  # ใช้ spawn points ทั้งหมด
            traffic_density=0.3,  # รถอื่นปานกลาง
            reward_weights={
                'w_progress': 0.8,
                'w_comfort': 0.3,
                'w_collision': 200.0,
                'w_lane_deviation': 0.4,
                'w_speed': 0.4,
            },
            success_criteria={
                'success_rate': 0.50,  # 50% ไม่ชน (ยากขึ้น)
                'max_lane_deviation': 1.0,
            }
        ),
    ]
    
    return stages


def get_stage_by_name(name: str) -> StageConfig:
    """ดึง stage config ตามชื่อ"""
    stages = create_default_stages()
    for stage in stages:
        if stage.name == name:
            return stage
    raise ValueError(f"Stage '{name}' not found")


def get_stage_by_episode(episode_num: int) -> StageConfig:
    """ดึง stage config ตามจำนวน episode"""
    stages = create_default_stages()
    cumulative_episodes = 0
    
    for stage in stages:
        cumulative_episodes += stage.target_episodes
        if episode_num < cumulative_episodes:
            return stage
    
    # ถ้าเกินจำนวน episodes ทั้งหมด ให้ใช้ stage สุดท้าย
    return stages[-1]
