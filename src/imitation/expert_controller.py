"""
Expert Controller
PID-based controller สำหรับสร้าง expert demonstrations
"""

import numpy as np
import carla
from typing import Tuple, Optional


class PIDController:
    """PID Controller แบบง่าย"""
    
    def __init__(self, kp: float, ki: float, kd: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.integral = 0.0
        self.prev_error = 0.0
    
    def step(self, error: float, dt: float) -> float:
        """คำนวณ control output"""
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        
        self.prev_error = error
        return output
    
    def reset(self):
        """Reset controller state"""
        self.integral = 0.0
        self.prev_error = 0.0


class ExpertController:
    """
    Expert Controller สำหรับ CARLA
    ใช้ PID control สำหรับ lateral (steering) และ longitudinal (speed) control
    """
    
    def __init__(
        self,
        target_speed: float = 30.0 / 3.6,  # m/s
        lateral_kp: float = 1.0,
        lateral_ki: float = 0.0,
        lateral_kd: float = 0.1,
        longitudinal_kp: float = 0.5,
        longitudinal_ki: float = 0.1,
        longitudinal_kd: float = 0.05,
        lookahead_distance: float = 5.0,
    ):
        """
        Args:
            target_speed: ความเร็วเป้าหมาย (m/s)
            lateral_kp, lateral_ki, lateral_kd: PID gains สำหรับ steering
            longitudinal_kp, longitudinal_ki, longitudinal_kd: PID gains สำหรับ speed
            lookahead_distance: ระยะมองไปข้างหน้า (m)
        """
        self.target_speed = target_speed
        self.lookahead_distance = lookahead_distance
        
        # PID controllers
        self.lateral_controller = PIDController(lateral_kp, lateral_ki, lateral_kd)
        self.longitudinal_controller = PIDController(
            longitudinal_kp, longitudinal_ki, longitudinal_kd
        )
        
        self.dt = 0.05  # default timestep
    
    def get_action(
        self,
        vehicle: carla.Actor,
        world_map: carla.Map,
        dt: float = 0.05
    ) -> np.ndarray:
        """
        คำนวณ action จาก vehicle state ปัจจุบัน
        
        Args:
            vehicle: CARLA vehicle actor
            world_map: CARLA map
            dt: timestep
            
        Returns:
            action: [steering, throttle, brake]
        """
        self.dt = dt
        
        # ดึง vehicle state
        transform = vehicle.get_transform()
        velocity = vehicle.get_velocity()
        location = transform.location
        
        # คำนวณความเร็วปัจจุบัน
        current_speed = np.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        
        # ดึง waypoint ปัจจุบันและถัดไป
        current_waypoint = world_map.get_waypoint(location)
        
        # หา waypoint ที่อยู่ข้างหน้า
        lookahead_waypoints = current_waypoint.next(self.lookahead_distance)
        
        if len(lookahead_waypoints) > 0:
            target_waypoint = lookahead_waypoints[0]
        else:
            target_waypoint = current_waypoint
        
        # คำนวณ steering (lateral control)
        steering = self._compute_steering(transform, target_waypoint)
        
        # คำนวณ throttle/brake (longitudinal control)
        throttle, brake = self._compute_throttle_brake(current_speed)
        
        # Clip values
        steering = np.clip(steering, -1.0, 1.0)
        throttle = np.clip(throttle, 0.0, 1.0)
        brake = np.clip(brake, 0.0, 1.0)
        
        return np.array([steering, throttle, brake], dtype=np.float32)
    
    def _compute_steering(
        self,
        transform: carla.Transform,
        target_waypoint: carla.Waypoint
    ) -> float:
        """คำนวณ steering angle"""
        # คำนวณ vector จากรถไปยัง target waypoint
        target_location = target_waypoint.transform.location
        current_location = transform.location
        
        dx = target_location.x - current_location.x
        dy = target_location.y - current_location.y
        
        # คำนวณมุมที่ต้องการ
        target_angle = np.arctan2(dy, dx)
        
        # คำนวณมุมปัจจุบันของรถ
        current_angle = np.radians(transform.rotation.yaw)
        
        # คำนวณ heading error
        angle_error = target_angle - current_angle
        
        # Normalize angle to [-pi, pi]
        angle_error = np.arctan2(np.sin(angle_error), np.cos(angle_error))
        
        # ใช้ PID controller
        steering = self.lateral_controller.step(angle_error, self.dt)
        
        return steering
    
    def _compute_throttle_brake(self, current_speed: float) -> Tuple[float, float]:
        """คำนวณ throttle และ brake"""
        # คำนวณ speed error
        speed_error = self.target_speed - current_speed
        
        # ใช้ PID controller
        control = self.longitudinal_controller.step(speed_error, self.dt)
        
        if control > 0:
            # ต้องการเร่ง
            throttle = min(control, 1.0)
            brake = 0.0
        else:
            # ต้องการเบรก
            throttle = 0.0
            brake = min(-control, 1.0)
        
        return throttle, brake
    
    def reset(self):
        """Reset controller state"""
        self.lateral_controller.reset()
        self.longitudinal_controller.reset()
    
    def set_target_speed(self, speed: float):
        """ตั้งค่าความเร็วเป้าหมาย (m/s)"""
        self.target_speed = speed
    
    def get_target_speed_kmh(self) -> float:
        """ดึงความเร็วเป้าหมายใน km/h"""
        return self.target_speed * 3.6


class RuleBasedExpert(ExpertController):
    """
    Expert controller ที่มี rules เพิ่มเติม
    เช่น ลดความเร็วตอนเลี้ยว, หยุดเมื่อเจอสิ่งกีดขวาง
    """
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.base_target_speed = self.target_speed
    
    def get_action(
        self,
        vehicle: carla.Actor,
        world_map: carla.Map,
        dt: float = 0.05
    ) -> np.ndarray:
        """คำนวณ action พร้อม rules เพิ่มเติม"""
        
        # ดึง waypoint ปัจจุบัน
        transform = vehicle.get_transform()
        current_waypoint = world_map.get_waypoint(transform.location)
        
        # ตรวจสอบว่าอยู่ในโค้งหรือไม่
        if self._is_curve(current_waypoint):
            # ลดความเร็วในโค้ง
            self.target_speed = self.base_target_speed * 0.7
        else:
            # ความเร็วปกติ
            self.target_speed = self.base_target_speed
        
        # คำนวณ action ตามปกติ
        return super().get_action(vehicle, world_map, dt)
    
    def _is_curve(self, waypoint: carla.Waypoint, threshold: float = 0.3) -> bool:
        """ตรวจสอบว่าอยู่ในโค้งหรือไม่"""
        # ดู waypoint ข้างหน้า
        next_waypoints = waypoint.next(10.0)
        
        if len(next_waypoints) == 0:
            return False
        
        next_waypoint = next_waypoints[0]
        
        # คำนวณความแตกต่างของมุม
        current_yaw = waypoint.transform.rotation.yaw
        next_yaw = next_waypoint.transform.rotation.yaw
        
        angle_diff = abs(next_yaw - current_yaw)
        
        # Normalize to [0, 180]
        if angle_diff > 180:
            angle_diff = 360 - angle_diff
        
        return angle_diff > threshold
