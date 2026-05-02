"""
Enhanced Reward Calculator with Training Guidelines
คำนวณ reward ตาม guidelines ที่กำหนดไว้อย่างชัดเจน
"""

import numpy as np
import yaml
from pathlib import Path
from typing import Dict, Any


class EnhancedRewardCalculator:
    """
    Reward calculator ที่ใช้ training guidelines
    กำหนดพารามิเตอร์ชัดเจนสำหรับความเร็ว การเลี้ยว ความปลอดภัย
    """
    
    def __init__(self, guidelines_path: str = None, stage: str = "stage1"):
        """
        Args:
            guidelines_path: path to training_guidelines.yaml
            stage: curriculum stage (stage1, stage2, stage3)
        """
        if guidelines_path is None:
            guidelines_path = "config/training_guidelines.yaml"
        
        # โหลด guidelines
        with open(guidelines_path, 'r') as f:
            self.guidelines = yaml.safe_load(f)
        
        self.stage = stage
        self.weights = self.guidelines['reward_weights'][stage]
        
        # Cache สำหรับคำนวณ
        self.prev_speed = 0.0
        self.prev_steering = 0.0
        self.prev_throttle = 0.0
        self.stuck_counter = 0
        
    def calculate_reward(
        self,
        speed: float,              # m/s
        steering: float,           # -1 to 1
        throttle: float,           # 0 to 1
        brake: float,              # 0 to 1
        lateral_offset: float,     # meters from lane center
        heading_error: float,      # radians
        progress: float,           # meters moved forward
        collision: bool,
        acceleration: float = 0.0, # m/s²
    ) -> Dict[str, float]:
        """
        คำนวณ reward components ตาม guidelines
        
        Returns:
            dict with reward components and total
        """
        rewards = {}
        
        # 1. Progress Reward (ความคืบหน้า)
        rewards['progress'] = self._calculate_progress_reward(progress)
        
        # 2. Lane Keeping Reward (การอยู่ในเลน)
        rewards['lane_keeping'] = self._calculate_lane_keeping_reward(lateral_offset)
        
        # 3. Speed Tracking Reward (การควบคุมความเร็ว)
        rewards['speed_tracking'] = self._calculate_speed_tracking_reward(speed)
        
        # 4. Smooth Control Reward (ความนุ่มนวลในการควบคุม)
        rewards['smooth_control'] = self._calculate_smooth_control_reward(
            steering, throttle, brake, acceleration
        )
        
        # 5. Heading Alignment Reward (การจัดทิศทาง)
        rewards['heading_alignment'] = self._calculate_heading_reward(heading_error)
        
        # 6. Collision Penalty (โทษการชน)
        rewards['collision'] = -self.weights['collision'] if collision else 0.0
        
        # 7. Safety Distance (ถ้าอยู่ใน stage 3)
        if self.stage == 'stage3':
            rewards['safety_distance'] = 0.0  # TODO: implement with obstacle detection
        
        # คำนวณ total reward
        total_reward = sum(
            rewards[key] * self.weights.get(key, 0.0)
            for key in rewards.keys()
        )
        
        rewards['total'] = total_reward
        
        # Update cache
        self.prev_speed = speed
        self.prev_steering = steering
        self.prev_throttle = throttle
        
        return rewards
    
    def _calculate_progress_reward(self, progress: float) -> float:
        """คำนวณ reward จากความคืบหน้า"""
        cfg = self.guidelines['progress']
        
        if progress < cfg['min_progress_per_step']:
            # ติดอยู่กับที่
            self.stuck_counter += 1
            if self.stuck_counter > 10:
                return cfg['stuck_penalty']
        else:
            self.stuck_counter = 0
        
        if progress < 0:
            # ถอยหลัง
            return cfg['backward_penalty']
        
        # Reward ตามระยะทาง
        reward = progress * cfg['progress_reward_scale']
        
        # โบนัสถ้าเคลื่อนที่ได้ดี
        if progress >= cfg['good_progress_per_step']:
            reward *= 1.5
        
        return reward
    
    def _calculate_lane_keeping_reward(self, lateral_offset: float) -> float:
        """คำนวณ reward จากการอยู่ในเลน"""
        cfg = self.guidelines['lane_keeping']
        
        abs_offset = abs(lateral_offset)
        
        # โบนัสสำหรับอยู่กึ่งกลางเลน
        if abs_offset < 0.1:
            return cfg['perfect_lane_bonus']
        
        # Penalty ตามระยะห่าง
        if abs_offset <= cfg['acceptable_deviation']:
            # ยอมรับได้
            penalty = -abs_offset * 0.1
        elif abs_offset <= cfg['warning_deviation']:
            # เตือน
            penalty = -abs_offset * 0.5
        elif abs_offset <= cfg['critical_deviation']:
            # วิกฤต
            penalty = -abs_offset * 1.0
        else:
            # ออกนอกเลนมาก
            penalty = -abs_offset * cfg['deviation_penalty_scale'] * 2.0
        
        return penalty
    
    def _calculate_speed_tracking_reward(self, speed: float) -> float:
        """คำนวณ reward จากการควบคุมความเร็ว"""
        cfg = self.guidelines['speed']
        
        # แปลงเป็น km/h
        speed_kmh = speed * 3.6
        target_speed = cfg['target_speed']
        tolerance = cfg['speed_tolerance']
        
        # คำนวณความผิดพลาด
        speed_error = abs(speed_kmh - target_speed)
        
        if speed_error <= tolerance:
            # อยู่ในช่วงที่ยอมรับได้
            reward = 0.5 * (1.0 - speed_error / tolerance)
        else:
            # ผิดพลาดมาก
            penalty = -(speed_error - tolerance) * cfg['speed_penalty_scale']
            reward = penalty
        
        # โทษถ้าเร็วเกินไป
        if speed_kmh > cfg['max_speed']:
            reward -= 1.0
        
        return reward
    
    def _calculate_smooth_control_reward(
        self,
        steering: float,
        throttle: float,
        brake: float,
        acceleration: float
    ) -> float:
        """คำนวณ reward จากความนุ่มนวลในการควบคุม"""
        reward = 0.0
        
        # 1. Steering smoothness
        steering_cfg = self.guidelines['steering']
        steering_change = abs(steering - self.prev_steering)
        
        if steering_change <= steering_cfg['max_steering_change']:
            # นุ่มนวล
            reward += steering_cfg['smooth_steering_bonus']
        else:
            # กระตุก
            reward += steering_cfg['jerky_steering_penalty']
        
        # 2. Acceleration smoothness
        accel_cfg = self.guidelines['acceleration']
        throttle_change = abs(throttle - self.prev_throttle)
        
        if throttle_change <= accel_cfg['max_throttle_change']:
            reward += accel_cfg['smooth_accel_bonus']
        
        # 3. Comfort (ความสบาย)
        abs_accel = abs(acceleration)
        
        if acceleration > 0:
            # กำลังเร่ง
            if abs_accel <= accel_cfg['comfortable_acceleration']:
                reward += 0.1
            elif abs_accel >= accel_cfg['harsh_acceleration']:
                reward += accel_cfg['harsh_accel_penalty']
        else:
            # กำลังเบรก
            if abs_accel <= accel_cfg['comfortable_deceleration']:
                reward += 0.1
            elif abs_accel >= accel_cfg['harsh_deceleration']:
                reward += accel_cfg['harsh_accel_penalty']
        
        return reward
    
    def _calculate_heading_reward(self, heading_error: float) -> float:
        """คำนวณ reward จากการจัดทิศทาง"""
        cfg = self.guidelines['heading']
        
        # แปลงเป็นองศา
        heading_error_deg = abs(np.degrees(heading_error))
        
        if heading_error_deg <= cfg['acceptable_heading_error']:
            # ดี
            return cfg['good_heading_bonus']
        elif heading_error_deg <= cfg['warning_heading_error']:
            # พอใช้
            penalty = -heading_error_deg * 0.01
            return penalty
        elif heading_error_deg <= cfg['critical_heading_error']:
            # แย่
            penalty = -heading_error_deg * cfg['heading_error_penalty_scale']
            return penalty
        else:
            # แย่มาก
            return -2.0
    
    def should_terminate(
        self,
        lateral_offset: float,
        heading_error: float,
        speed: float,
        steps: int,
        collision: bool
    ) -> tuple[bool, str]:
        """
        ตรวจสอบว่าควรจบ episode หรือไม่
        
        Returns:
            (should_terminate, reason)
        """
        cfg = self.guidelines['termination']
        
        # 1. Collision
        if collision:
            return True, "collision"
        
        # 2. Max steps
        if steps >= cfg['max_steps']:
            return True, "max_steps"
        
        # 3. Lane deviation
        if abs(lateral_offset) > cfg['max_lane_deviation']:
            return True, "lane_deviation"
        
        # 4. Heading error
        heading_error_deg = abs(np.degrees(heading_error))
        if heading_error_deg > cfg['max_heading_error']:
            return True, "heading_error"
        
        # 5. Speed timeout (ติดอยู่กับที่นานเกินไป)
        if self.stuck_counter > cfg['min_speed_timeout']:
            return True, "stuck"
        
        return False, ""
    
    def is_success(
        self,
        distance_traveled: float,
        collision_count: int
    ) -> bool:
        """
        ตรวจสอบว่า episode สำเร็จหรือไม่
        
        Returns:
            True if success
        """
        cfg = self.guidelines['termination']
        
        # ต้องขับได้ระยะทางขั้นต่ำ และไม่ชน
        return (
            distance_traveled >= cfg['min_distance_for_success'] and
            collision_count <= cfg['max_collisions_allowed']
        )
    
    def reset(self):
        """Reset internal state"""
        self.prev_speed = 0.0
        self.prev_steering = 0.0
        self.prev_throttle = 0.0
        self.stuck_counter = 0
    
    def get_speed_limit_for_situation(self, curve_angle: float = 0.0) -> float:
        """
        ดึงความเร็วที่เหมาะสมตามสถานการณ์
        
        Args:
            curve_angle: มุมของโค้ง (องศา)
            
        Returns:
            speed limit in km/h
        """
        cfg = self.guidelines['speed']
        
        if abs(curve_angle) < 10:
            return cfg['straight_road']
        elif abs(curve_angle) < 30:
            return cfg['slight_curve']
        else:
            return cfg['sharp_curve']
