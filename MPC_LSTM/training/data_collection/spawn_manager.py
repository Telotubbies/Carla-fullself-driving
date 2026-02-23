"""
Spawn Point Management for Diverse Data Collection.

Handles selection and management of diverse spawn points.
"""

import logging
import numpy as np
import carla
from typing import List, Tuple

logger = logging.getLogger(__name__)


class SpawnPointManager:
    """Manages spawn point selection for diversity."""
    
    def __init__(self, world: carla.World):
        """
        Initialize spawn point manager.
        
        Args:
            world: CARLA world instance
        """
        self.world = world
        self.all_spawns = world.get_map().get_spawn_points()
        logger.info(f"Found {len(self.all_spawns)} spawn points")
    
    def select_diverse_spawn_points(self, num_points: int = 10) -> List[int]:
        """
        Select diverse spawn points.
        
        Strategy: Select points that are far apart to ensure route diversity.
        
        Args:
            num_points: Number of spawn points to select
        
        Returns:
            List of spawn point indices
        """
        if len(self.all_spawns) < num_points:
            return list(range(len(self.all_spawns)))
        
        # Select diverse spawn points (spread out)
        selected = [0]  # Always include first spawn point
        remaining = list(range(1, len(self.all_spawns)))
        
        while len(selected) < num_points and remaining:
            # Find point farthest from all selected points
            max_min_dist = -1
            best_idx = None
            
            for candidate_idx in remaining:
                candidate_loc = self.all_spawns[candidate_idx].location
                min_dist = float('inf')
                
                for selected_idx in selected:
                    selected_loc = self.all_spawns[selected_idx].location
                    dist = candidate_loc.distance(selected_loc)
                    min_dist = min(min_dist, dist)
                
                if min_dist > max_min_dist:
                    max_min_dist = min_dist
                    best_idx = candidate_idx
            
            if best_idx is not None:
                selected.append(best_idx)
                remaining.remove(best_idx)
            else:
                break
        
        logger.info(f"Selected {len(selected)} diverse spawn points")
        return selected
    
    def get_spawn_point(self, index: int) -> carla.Transform:
        """
        Get spawn point by index.
        
        Args:
            index: Spawn point index
        
        Returns:
            CARLA transform
        """
        if 0 <= index < len(self.all_spawns):
            return self.all_spawns[index]
        return None
    
    def calculate_distance(self, idx1: int, idx2: int) -> float:
        """
        Calculate distance between two spawn points.
        
        Args:
            idx1: First spawn point index
            idx2: Second spawn point index
        
        Returns:
            Distance in meters
        """
        if 0 <= idx1 < len(self.all_spawns) and 0 <= idx2 < len(self.all_spawns):
            loc1 = self.all_spawns[idx1].location
            loc2 = self.all_spawns[idx2].location
            return loc1.distance(loc2)
        return 0.0


