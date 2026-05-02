"""
Metrics Logger
จัดการการ log metrics แบบ real-time และ batch
"""

from typing import Dict, Any, List, Optional
import numpy as np
from collections import defaultdict, deque


class MetricsLogger:
    """Logger สำหรับเก็บและคำนวณ metrics"""
    
    def __init__(self, window_size: int = 100):
        """
        Args:
            window_size: ขนาด window สำหรับคำนวณ moving average
        """
        self.window_size = window_size
        self.metrics = defaultdict(list)
        self.windows = defaultdict(lambda: deque(maxlen=window_size))
        
    def log(self, key: str, value: float):
        """Log metric value"""
        self.metrics[key].append(value)
        self.windows[key].append(value)
    
    def log_dict(self, metrics: Dict[str, float]):
        """Log multiple metrics"""
        for key, value in metrics.items():
            self.log(key, value)
    
    def get_mean(self, key: str, window: bool = True) -> Optional[float]:
        """คำนวณค่าเฉลี่ย"""
        if window and key in self.windows and len(self.windows[key]) > 0:
            return np.mean(list(self.windows[key]))
        elif key in self.metrics and len(self.metrics[key]) > 0:
            return np.mean(self.metrics[key])
        return None
    
    def get_std(self, key: str, window: bool = True) -> Optional[float]:
        """คำนวณ standard deviation"""
        if window and key in self.windows and len(self.windows[key]) > 0:
            return np.std(list(self.windows[key]))
        elif key in self.metrics and len(self.metrics[key]) > 0:
            return np.std(self.metrics[key])
        return None
    
    def get_min(self, key: str, window: bool = True) -> Optional[float]:
        """คำนวณค่าต่ำสุด"""
        if window and key in self.windows and len(self.windows[key]) > 0:
            return np.min(list(self.windows[key]))
        elif key in self.metrics and len(self.metrics[key]) > 0:
            return np.min(self.metrics[key])
        return None
    
    def get_max(self, key: str, window: bool = True) -> Optional[float]:
        """คำนวณค่าสูงสุด"""
        if window and key in self.windows and len(self.windows[key]) > 0:
            return np.max(list(self.windows[key]))
        elif key in self.metrics and len(self.metrics[key]) > 0:
            return np.max(self.metrics[key])
        return None
    
    def get_last(self, key: str) -> Optional[float]:
        """ดึงค่าล่าสุด"""
        if key in self.metrics and len(self.metrics[key]) > 0:
            return self.metrics[key][-1]
        return None
    
    def get_summary(self, key: str, window: bool = True) -> Dict[str, float]:
        """ดึงสรุปสถิติของ metric"""
        return {
            'mean': self.get_mean(key, window) or 0.0,
            'std': self.get_std(key, window) or 0.0,
            'min': self.get_min(key, window) or 0.0,
            'max': self.get_max(key, window) or 0.0,
            'last': self.get_last(key) or 0.0,
        }
    
    def get_all_summaries(self, window: bool = True) -> Dict[str, Dict[str, float]]:
        """ดึงสรุปสถิติของ metrics ทั้งหมด"""
        summaries = {}
        for key in self.metrics.keys():
            summaries[key] = self.get_summary(key, window)
        return summaries
    
    def reset(self):
        """Reset metrics ทั้งหมด"""
        self.metrics.clear()
        self.windows.clear()
    
    def reset_key(self, key: str):
        """Reset metric เฉพาะ key"""
        if key in self.metrics:
            del self.metrics[key]
        if key in self.windows:
            del self.windows[key]
    
    def get_count(self, key: str) -> int:
        """ดึงจำนวนค่าที่ log ไว้"""
        return len(self.metrics.get(key, []))
    
    def has_key(self, key: str) -> bool:
        """ตรวจสอบว่ามี key นี้หรือไม่"""
        return key in self.metrics
    
    def get_keys(self) -> List[str]:
        """ดึง keys ทั้งหมด"""
        return list(self.metrics.keys())
    
    def print_summary(self, window: bool = True):
        """แสดงสรุปสถิติ"""
        print("\n" + "="*80)
        print("📊 METRICS SUMMARY")
        print("="*80)
        
        summaries = self.get_all_summaries(window)
        
        for key, summary in sorted(summaries.items()):
            print(f"\n{key}:")
            print(f"  Mean: {summary['mean']:.4f}")
            print(f"  Std:  {summary['std']:.4f}")
            print(f"  Min:  {summary['min']:.4f}")
            print(f"  Max:  {summary['max']:.4f}")
            print(f"  Last: {summary['last']:.4f}")
        
        print("="*80 + "\n")
