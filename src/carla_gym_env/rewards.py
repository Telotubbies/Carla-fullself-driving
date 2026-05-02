import numpy as np
from typing import Dict, Any
import carla


class RewardCalculator:
    """Calculate rewards for the CARLA environment."""
    
    def __init__(self, config: Dict[str, Any]):
        self.config = config
        
        # Reward weights
        self.w_progress = config.get('w_progress', 1.0)
        self.w_comfort = config.get('w_comfort', 0.1)
        self.w_collision = config.get('w_collision', 200.0)
        self.w_lane_deviation = config.get('w_lane_deviation', 0.5)
        self.w_speed = config.get('w_speed', 0.2)
        
        # Target speed (m/s)
        self.target_speed = config.get('target_speed', 30.0 / 3.6)  # 30 km/h
        
        # Previous values for computing derivatives
        self.prev_action = None
    
    def calculate(self, data: Dict[str, Any]) -> float:
        """
        Calculate total reward based on multiple components.
        
        Reward components:
        - Progress reward: r_p = v · cos(Δθ) · dt
        - Comfort penalty: r_c = -|jerk| - |lateral_accel|
        - Collision penalty: r_col = -200 (episode end)
        - Lane deviation: r_lane = -|lateral_offset| / lane_width
        - Speed tracking: reward for maintaining target speed
        """
        reward = 0.0
        
        # Extract data
        speed = data['speed']
        prev_speed = data['prev_speed']
        location = data['location']
        prev_location = data['prev_location']
        waypoint = data['waypoint']
        collision = data['collision']
        action = data['action']
        heading = data['heading']
        waypoint_heading = data['waypoint_heading']
        
        # 1. Progress reward
        if prev_location is not None:
            # Distance traveled
            distance = np.sqrt(
                (location.x - prev_location.x)**2 + 
                (location.y - prev_location.y)**2
            )
            
            # Heading alignment
            heading_error = self._normalize_angle(heading - waypoint_heading)
            alignment = np.cos(np.radians(heading_error))
            
            # Progress reward: forward movement aligned with road
            r_progress = distance * alignment * self.w_progress
            reward += r_progress
        
        # 2. Comfort penalty (penalize jerky movements)
        if self.prev_action is not None:
            # Action smoothness (penalize rapid changes)
            action_diff = np.abs(action - self.prev_action)
            jerk_penalty = -np.sum(action_diff) * self.w_comfort
            reward += jerk_penalty
        
        self.prev_action = action.copy()
        
        # 3. Collision penalty
        if collision:
            reward -= self.w_collision
        
        # 4. Lane deviation penalty
        lateral_offset = self._calculate_lateral_offset(location, waypoint.transform)
        lane_width = waypoint.lane_width
        
        if lane_width > 0:
            deviation_ratio = abs(lateral_offset) / (lane_width / 2.0)
            r_lane = -deviation_ratio * self.w_lane_deviation
            reward += r_lane
        
        # 5. Speed tracking reward
        speed_error = abs(speed - self.target_speed)
        r_speed = -speed_error * self.w_speed
        reward += r_speed
        
        # Bonus for maintaining good speed
        if abs(speed - self.target_speed) < 2.0:  # Within 2 m/s
            reward += 0.1
        
        return reward
    
    def _calculate_lateral_offset(self, location: carla.Location, waypoint_transform: carla.Transform) -> float:
        """Calculate lateral offset from waypoint center."""
        waypoint_loc = waypoint_transform.location
        waypoint_yaw = np.radians(waypoint_transform.rotation.yaw)
        
        # Vector from waypoint to vehicle
        dx = location.x - waypoint_loc.x
        dy = location.y - waypoint_loc.y
        
        # Project onto perpendicular to waypoint direction
        lateral_offset = -dx * np.sin(waypoint_yaw) + dy * np.cos(waypoint_yaw)
        
        return lateral_offset
    
    def _normalize_angle(self, angle: float) -> float:
        """Normalize angle to [-180, 180] degrees."""
        while angle > 180:
            angle -= 360
        while angle < -180:
            angle += 360
        return angle
    
    def reset(self):
        """Reset internal state."""
        self.prev_action = None
