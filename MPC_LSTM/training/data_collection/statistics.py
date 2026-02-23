"""
Statistics Tracking for Data Collection.

Tracks steering distribution and collection statistics.
"""

import logging
import numpy as np
from typing import List, Dict, Any
from collections import defaultdict

logger = logging.getLogger(__name__)


class CollectionStatistics:
    """Tracks data collection statistics."""
    
    def __init__(self):
        """Initialize statistics tracker."""
        self.stats = {
            'total_frames': 0,
            'spawn_points_used': set(),
            'steering_left': 0,
            'steering_right': 0,
            'steering_straight': 0,
            'high_steering': 0,  # |steering| > 0.3
            'steering_distribution': []
        }
    
    def update(self, steering: float, spawn_idx: int = None):
        """
        Update statistics with new data point.
        
        Args:
            steering: Steering angle
            spawn_idx: Spawn point index (optional)
        """
        self.stats['total_frames'] += 1
        
        if spawn_idx is not None:
            self.stats['spawn_points_used'].add(spawn_idx)
        
        # Categorize steering
        if steering < -0.1:
            self.stats['steering_left'] += 1
        elif steering > 0.1:
            self.stats['steering_right'] += 1
        else:
            self.stats['steering_straight'] += 1
        
        if abs(steering) > 0.3:
            self.stats['high_steering'] += 1
        
        # Track distribution
        self.stats['steering_distribution'].append(steering)
        
        # Keep only recent 1000 for memory efficiency
        if len(self.stats['steering_distribution']) > 1000:
            self.stats['steering_distribution'] = self.stats['steering_distribution'][-1000:]
    
    def get_steering_diversity(self, window: int = 100) -> float:
        """
        Get steering diversity (standard deviation).
        
        Args:
            window: Number of recent samples to analyze
        
        Returns:
            Standard deviation of steering
        """
        if len(self.stats['steering_distribution']) < window:
            return 0.0
        
        recent = self.stats['steering_distribution'][-window:]
        return float(np.std(recent))
    
    def should_switch_spawn(
        self,
        current_frames: int,
        min_frames_per_spawn: int = 2000,
        min_steering_std: float = 0.05
    ) -> bool:
        """
        Decide if spawn point should be switched.
        
        Args:
            current_frames: Frames collected at current spawn
            min_frames_per_spawn: Minimum frames before switching
            min_steering_std: Minimum steering std to avoid switching
        
        Returns:
            True if should switch
        """
        if current_frames < min_frames_per_spawn:
            return False
        
        # Check steering diversity
        steering_std = self.get_steering_diversity()
        if steering_std < min_steering_std:
            logger.info(f"Low steering diversity (std={steering_std:.4f}), switching spawn point")
            return True
        
        # Switch after collecting enough frames
        if current_frames % min_frames_per_spawn == 0:
            return True
        
        return False
    
    def get_summary(self) -> Dict[str, Any]:
        """
        Get statistics summary.
        
        Returns:
            Dictionary with statistics
        """
        total = self.stats['total_frames']
        if total == 0:
            return {}
        
        return {
            'total_frames': total,
            'spawn_points_used': len(self.stats['spawn_points_used']),
            'steering_left_pct': (self.stats['steering_left'] / total) * 100,
            'steering_right_pct': (self.stats['steering_right'] / total) * 100,
            'steering_straight_pct': (self.stats['steering_straight'] / total) * 100,
            'high_steering_pct': (self.stats['high_steering'] / total) * 100,
            'steering_std': self.get_steering_diversity()
        }
    
    def log_summary(self):
        """Log statistics summary."""
        summary = self.get_summary()
        if summary:
            logger.info("📊 Collection Statistics:")
            logger.info(f"  Total frames: {summary['total_frames']}")
            logger.info(f"  Spawn points used: {summary['spawn_points_used']}")
            logger.info(f"  Steering distribution:")
            logger.info(f"    Left: {summary['steering_left_pct']:.1f}%")
            logger.info(f"    Right: {summary['steering_right_pct']:.1f}%")
            logger.info(f"    Straight: {summary['steering_straight_pct']:.1f}%")
            logger.info(f"    High (>0.3): {summary['high_steering_pct']:.1f}%")
            logger.info(f"  Steering std: {summary['steering_std']:.4f}")


