"""
Curriculum Manager
จัดการการเปลี่ยน stage และติดตาม progress ของ curriculum learning
"""

from typing import Dict, Any, Optional, List
import numpy as np
from .stages import Stage, StageConfig, create_default_stages


class CurriculumManager:
    """จัดการ curriculum learning process"""
    
    def __init__(self, stages: Optional[List[StageConfig]] = None):
        """
        Args:
            stages: List of StageConfig, ถ้าไม่ระบุจะใช้ default stages
        """
        if stages is None:
            stages = create_default_stages()
        
        self.stage_configs = stages
        self.stages = [Stage(config) for config in stages]
        self.current_stage_idx = 0
        self.total_episodes = 0
        
    @property
    def current_stage(self) -> Stage:
        """ดึง stage ปัจจุบัน"""
        return self.stages[self.current_stage_idx]
    
    @property
    def current_config(self) -> StageConfig:
        """ดึง config ของ stage ปัจจุบัน"""
        return self.current_stage.config
    
    def record_episode(self, metrics: Dict[str, Any]):
        """บันทึกผลลัพธ์ของ episode และตรวจสอบการเปลี่ยน stage"""
        self.total_episodes += 1
        self.current_stage.record_episode(metrics)
        
        # ตรวจสอบว่าควรเปลี่ยน stage หรือไม่
        if self.current_stage.is_complete() and not self.is_final_stage():
            self._transition_to_next_stage()
    
    def _transition_to_next_stage(self):
        """เปลี่ยนไปยัง stage ถัดไป"""
        old_stage = self.current_stage.config.name
        self.current_stage_idx += 1
        new_stage = self.current_stage.config.name
        
        print(f"\n{'='*80}")
        print(f"🎓 CURRICULUM TRANSITION: {old_stage} → {new_stage}")
        print(f"{'='*80}")
        print(f"Stage {self.current_stage_idx + 1}/{len(self.stages)}: {self.current_stage.config.description}")
        print(f"Target episodes: {self.current_stage.config.target_episodes}")
        print(f"{'='*80}\n")
    
    def is_final_stage(self) -> bool:
        """ตรวจสอบว่าอยู่ใน stage สุดท้ายหรือไม่"""
        return self.current_stage_idx >= len(self.stages) - 1
    
    def get_env_config(self) -> Dict[str, Any]:
        """ดึง environment config สำหรับ stage ปัจจุบัน"""
        config = self.current_config
        
        return {
            'use_fixed_spawn': True,
            'fixed_spawn_indices': config.spawn_points,
            'traffic_density': config.traffic_density,
            'reward_config': {
                'w_progress': config.reward_weights.get('w_progress', 1.0),
                'w_comfort': config.reward_weights.get('w_comfort', 0.1),
                'w_collision': config.reward_weights.get('w_collision', 200.0),
                'w_lane_deviation': config.reward_weights.get('w_lane_deviation', 0.5),
                'w_speed': config.reward_weights.get('w_speed', 0.2),
                'target_speed': 30.0 / 3.6,
            }
        }
    
    def get_progress(self) -> Dict[str, Any]:
        """ดึงข้อมูล progress ทั้งหมด"""
        stage_progress = self.current_stage.get_progress()
        overall_progress = (self.current_stage_idx + stage_progress) / len(self.stages)
        
        return {
            'current_stage_idx': self.current_stage_idx,
            'current_stage_name': self.current_stage.config.name,
            'current_stage_description': self.current_stage.config.description,
            'stage_progress': stage_progress,
            'overall_progress': overall_progress,
            'total_episodes': self.total_episodes,
            'stage_episodes': self.current_stage.episodes_completed,
            'is_final_stage': self.is_final_stage(),
        }
    
    def get_statistics(self) -> Dict[str, Any]:
        """ดึงสถิติทั้งหมด"""
        stats = {
            'total_episodes': self.total_episodes,
            'current_stage': self.current_stage.config.name,
            'stages': []
        }
        
        for idx, stage in enumerate(self.stages):
            stage_stats = stage.get_statistics()
            stage_stats['name'] = stage.config.name
            stage_stats['is_current'] = (idx == self.current_stage_idx)
            stage_stats['is_complete'] = stage.is_complete()
            stats['stages'].append(stage_stats)
        
        return stats
    
    def should_use_expert_data(self) -> bool:
        """ตรวจสอบว่าควรใช้ expert data หรือไม่ (สำหรับ imitation learning)"""
        # ใช้ expert data ใน stage แรกเท่านั้น
        return self.current_stage_idx == 0
    
    def get_expert_data_ratio(self) -> float:
        """คำนวณ ratio ของ expert data ที่ควรใช้"""
        if not self.should_use_expert_data():
            return 0.0
        
        # ลด ratio ตาม progress ของ stage
        progress = self.current_stage.get_progress()
        # เริ่มจาก 50% แล้วค่อยๆ ลดเหลือ 10%
        return 0.5 * (1.0 - progress * 0.8)
    
    def save_state(self, filepath: str):
        """บันทึก state ของ curriculum manager"""
        import pickle
        
        state = {
            'current_stage_idx': self.current_stage_idx,
            'total_episodes': self.total_episodes,
            'stages_history': [
                {
                    'config': stage.config,
                    'episodes_completed': stage.episodes_completed,
                    'success_count': stage.success_count,
                    'metrics_history': stage.metrics_history
                }
                for stage in self.stages
            ]
        }
        
        with open(filepath, 'wb') as f:
            pickle.dump(state, f)
        
        print(f"✅ Curriculum state saved to {filepath}")
    
    def load_state(self, filepath: str):
        """โหลด state ของ curriculum manager"""
        import pickle
        
        with open(filepath, 'rb') as f:
            state = pickle.load(f)
        
        self.current_stage_idx = state['current_stage_idx']
        self.total_episodes = state['total_episodes']
        
        # Restore stages history
        for idx, stage_state in enumerate(state['stages_history']):
            if idx < len(self.stages):
                self.stages[idx].episodes_completed = stage_state['episodes_completed']
                self.stages[idx].success_count = stage_state['success_count']
                self.stages[idx].metrics_history = stage_state['metrics_history']
        
        print(f"✅ Curriculum state loaded from {filepath}")
        print(f"   Current stage: {self.current_stage.config.name}")
        print(f"   Total episodes: {self.total_episodes}")
    
    def print_status(self):
        """แสดงสถานะปัจจุบัน"""
        progress = self.get_progress()
        stats = self.current_stage.get_statistics()
        
        print(f"\n{'='*80}")
        print(f"📚 CURRICULUM STATUS")
        print(f"{'='*80}")
        print(f"Stage: {progress['current_stage_name']} ({self.current_stage_idx + 1}/{len(self.stages)})")
        print(f"Description: {progress['current_stage_description']}")
        print(f"Progress: {progress['stage_progress']*100:.1f}% ({progress['stage_episodes']}/{self.current_config.target_episodes} episodes)")
        print(f"Overall Progress: {progress['overall_progress']*100:.1f}%")
        
        if stats:
            print(f"\nPerformance:")
            print(f"  Success Rate: {stats['success_rate']*100:.1f}%")
            print(f"  Avg Reward: {stats['avg_reward']:.2f}")
            print(f"  Avg Lane Deviation: {stats['avg_lane_deviation']:.3f}m")
            print(f"  Collision Rate: {stats['collision_rate']*100:.1f}%")
        
        print(f"{'='*80}\n")
