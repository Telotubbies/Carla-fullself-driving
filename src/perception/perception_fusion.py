"""
Perception Fusion System
รวมข้อมูลจากทุก perception modules เข้าด้วยกัน
แบบ Tesla Autopilot
"""

import cv2
import numpy as np
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass

from .lane_detector import LaneDetector
from .object_detector import ObjectDetector, Detection
from .traffic_light_detector import TrafficLightDetector, TrafficLight, TrafficLightState


@dataclass
class PerceptionOutput:
    """รวมผลลัพธ์จากทุก perception modules"""
    # Lane detection
    lane_detected: bool
    lane_center_offset: float  # meters
    lane_heading_error: float  # radians
    left_lane_poly: Optional[np.ndarray]
    right_lane_poly: Optional[np.ndarray]
    
    # Object detection
    objects: List[Detection]
    closest_vehicle: Optional[Detection]
    objects_in_path: List[Detection]
    
    # Traffic lights
    traffic_lights: List[TrafficLight]
    active_traffic_light: Optional[TrafficLight]
    should_stop_for_light: bool
    
    # Safety
    collision_warning: bool
    safe_to_proceed: bool
    recommended_speed: float  # km/h
    
    # Visualization
    visualization: Optional[np.ndarray] = None


class PerceptionFusion:
    """
    Tesla-like Perception System
    รวมทุก perception modules:
    - Lane Detection
    - Object Detection  
    - Traffic Light Detection
    - Safety Assessment
    """
    
    def __init__(
        self,
        use_gpu: bool = True,
        image_size: Tuple[int, int] = (640, 480)
    ):
        self.image_size = image_size
        
        # Initialize perception modules
        print("🔄 Initializing Perception System...")
        
        self.lane_detector = LaneDetector(
            use_gpu=use_gpu,
            image_size=image_size
        )
        print("  ✅ Lane Detector ready")
        
        self.object_detector = ObjectDetector(
            model_type='yolov5',
            use_gpu=use_gpu
        )
        print("  ✅ Object Detector ready")
        
        self.traffic_light_detector = TrafficLightDetector()
        print("  ✅ Traffic Light Detector ready")
        
        print("✅ Perception System initialized")
        
        # Safety parameters
        self.min_safe_distance = 5.0  # meters
        self.warning_distance = 10.0  # meters
        self.target_speed = 30.0  # km/h
        
    def process(
        self,
        camera_image: np.ndarray,
        depth_map: Optional[np.ndarray] = None,
        lidar_data: Optional[np.ndarray] = None
    ) -> PerceptionOutput:
        """
        ประมวลผลข้อมูลจากทุก sensors
        
        Args:
            camera_image: RGB camera image
            depth_map: Depth map (optional)
            lidar_data: LiDAR point cloud (optional)
            
        Returns:
            PerceptionOutput with all perception results
        """
        # 1. Lane Detection
        lane_result = self.lane_detector.detect(camera_image)
        
        # 2. Object Detection
        objects = self.object_detector.detect(camera_image, depth_map)
        
        # 3. Traffic Light Detection
        traffic_lights = self.traffic_light_detector.detect(camera_image, depth_map)
        
        # 4. Analyze objects
        closest_vehicle = self.object_detector.get_closest_object(
            objects,
            class_filter=['vehicle']
        )
        
        # Get lane center for path analysis
        h, w = camera_image.shape[:2]
        lane_center_x = w // 2  # Default to image center
        
        objects_in_path = self.object_detector.get_objects_in_path(
            objects,
            image_width=w,
            lane_center_x=lane_center_x
        )
        
        # 5. Traffic light analysis
        active_light = self.traffic_light_detector.get_active_light(traffic_lights)
        should_stop = self.traffic_light_detector.should_stop(traffic_lights)
        
        # 6. Safety assessment
        collision_warning, safe_to_proceed, recommended_speed = self._assess_safety(
            lane_result,
            objects,
            closest_vehicle,
            objects_in_path,
            active_light,
            should_stop
        )
        
        # 7. Create output
        output = PerceptionOutput(
            # Lane
            lane_detected=lane_result['lane_detected'],
            lane_center_offset=lane_result['lane_center_offset'],
            lane_heading_error=lane_result['lane_heading'],
            left_lane_poly=lane_result['left_polynomial'],
            right_lane_poly=lane_result['right_polynomial'],
            
            # Objects
            objects=objects,
            closest_vehicle=closest_vehicle,
            objects_in_path=objects_in_path,
            
            # Traffic lights
            traffic_lights=traffic_lights,
            active_traffic_light=active_light,
            should_stop_for_light=should_stop,
            
            # Safety
            collision_warning=collision_warning,
            safe_to_proceed=safe_to_proceed,
            recommended_speed=recommended_speed
        )
        
        return output
    
    def _assess_safety(
        self,
        lane_result: dict,
        objects: List[Detection],
        closest_vehicle: Optional[Detection],
        objects_in_path: List[Detection],
        active_light: Optional[TrafficLight],
        should_stop_for_light: bool
    ) -> Tuple[bool, bool, float]:
        """
        ประเมินความปลอดภัย
        
        Returns:
            (collision_warning, safe_to_proceed, recommended_speed)
        """
        collision_warning = False
        safe_to_proceed = True
        recommended_speed = self.target_speed
        
        # Check lane deviation
        if abs(lane_result['lane_center_offset']) > 1.5:
            collision_warning = True
            safe_to_proceed = False
            recommended_speed = min(recommended_speed, 15.0)
        
        # Check objects in path
        if objects_in_path:
            collision_warning = True
            
            # Find closest object in path
            with_distance = [obj for obj in objects_in_path if obj.distance is not None]
            if with_distance:
                closest_in_path = min(with_distance, key=lambda o: o.distance)
                
                if closest_in_path.distance < self.min_safe_distance:
                    safe_to_proceed = False
                    recommended_speed = 0.0
                elif closest_in_path.distance < self.warning_distance:
                    recommended_speed = min(recommended_speed, 15.0)
        
        # Check closest vehicle
        if closest_vehicle and closest_vehicle.distance is not None:
            if closest_vehicle.distance < self.warning_distance:
                # Adaptive cruise control
                distance_ratio = closest_vehicle.distance / self.warning_distance
                recommended_speed = min(recommended_speed, self.target_speed * distance_ratio)
        
        # Check traffic light
        if should_stop_for_light:
            if active_light and active_light.state == TrafficLightState.RED:
                safe_to_proceed = False
                recommended_speed = 0.0
            elif active_light and active_light.state == TrafficLightState.YELLOW:
                recommended_speed = min(recommended_speed, 10.0)
        
        return collision_warning, safe_to_proceed, recommended_speed
    
    def visualize(
        self,
        camera_image: np.ndarray,
        perception_output: PerceptionOutput
    ) -> np.ndarray:
        """
        สร้าง visualization แบบ Tesla
        แสดงทุกอย่าง: lanes, objects, traffic lights, safety info
        """
        vis = camera_image.copy()
        h, w = vis.shape[:2]
        
        # 1. Draw lane detection
        if perception_output.lane_detected:
            # Draw lane overlay (semi-transparent green)
            lane_overlay = np.zeros_like(vis)
            
            if perception_output.left_lane_poly is not None:
                self._draw_lane_poly(lane_overlay, perception_output.left_lane_poly, h, (0, 255, 0))
            
            if perception_output.right_lane_poly is not None:
                self._draw_lane_poly(lane_overlay, perception_output.right_lane_poly, h, (0, 255, 0))
            
            vis = cv2.addWeighted(vis, 0.7, lane_overlay, 0.3, 0)
        
        # 2. Draw objects
        for obj in perception_output.objects:
            x1, y1, x2, y2 = obj.bbox
            
            # Color based on class and distance
            if obj in perception_output.objects_in_path:
                color = (255, 0, 0)  # Red for objects in path
                thickness = 3
            else:
                color = (0, 255, 0)  # Green for other objects
                thickness = 2
            
            cv2.rectangle(vis, (x1, y1), (x2, y2), color, thickness)
            
            # Label
            label = f"{obj.class_name}"
            if obj.distance is not None:
                label += f" {obj.distance:.1f}m"
            
            cv2.putText(vis, label, (x1, y1-5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        # 3. Draw traffic lights
        for tl in perception_output.traffic_lights:
            x1, y1, x2, y2 = tl.bbox
            
            colors = {
                TrafficLightState.RED: (255, 0, 0),
                TrafficLightState.YELLOW: (255, 255, 0),
                TrafficLightState.GREEN: (0, 255, 0)
            }
            color = colors.get(tl.state, (128, 128, 128))
            
            cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
            cv2.putText(vis, tl.state.name, (x1, y1-5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        # 4. Draw HUD (Heads-Up Display)
        self._draw_hud(vis, perception_output)
        
        return vis
    
    def _draw_lane_poly(self, image: np.ndarray, poly: np.ndarray, height: int, color: tuple):
        """วาด lane polynomial"""
        y_points = np.linspace(0, height-1, height)
        x_points = poly[0] * y_points**2 + poly[1] * y_points + poly[2]
        
        points = np.column_stack([x_points, y_points]).astype(np.int32)
        
        # Filter valid points
        valid = (points[:, 0] >= 0) & (points[:, 0] < image.shape[1])
        points = points[valid]
        
        if len(points) > 1:
            cv2.polylines(image, [points], False, color, 5)
    
    def _draw_hud(self, image: np.ndarray, output: PerceptionOutput):
        """วาด HUD แบบ Tesla"""
        h, w = image.shape[:2]
        
        # Status panel (top left)
        panel_h = 200
        panel_w = 300
        overlay = image.copy()
        cv2.rectangle(overlay, (10, 10), (panel_w, panel_h), (0, 0, 0), -1)
        image[:] = cv2.addWeighted(image, 0.7, overlay, 0.3, 0)
        
        y = 35
        line_h = 25
        
        # Title
        cv2.putText(image, "PERCEPTION STATUS", (20, y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        y += line_h + 5
        
        # Lane status
        lane_color = (0, 255, 0) if output.lane_detected else (0, 0, 255)
        cv2.putText(image, f"Lane: {'OK' if output.lane_detected else 'LOST'}", (20, y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, lane_color, 1)
        y += line_h
        
        # Lane offset
        offset_color = (0, 255, 0) if abs(output.lane_center_offset) < 0.5 else (255, 165, 0)
        cv2.putText(image, f"Offset: {output.lane_center_offset:+.2f}m", (20, y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, offset_color, 1)
        y += line_h
        
        # Objects count
        cv2.putText(image, f"Objects: {len(output.objects)}", (20, y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        y += line_h
        
        # Traffic light
        if output.active_traffic_light:
            tl_color = {
                TrafficLightState.RED: (255, 0, 0),
                TrafficLightState.YELLOW: (255, 255, 0),
                TrafficLightState.GREEN: (0, 255, 0)
            }.get(output.active_traffic_light.state, (128, 128, 128))
            
            cv2.putText(image, f"Light: {output.active_traffic_light.state.name}", (20, y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, tl_color, 1)
            y += line_h
        
        # Safety status
        safety_color = (0, 255, 0) if output.safe_to_proceed else (255, 0, 0)
        safety_text = "SAFE" if output.safe_to_proceed else "DANGER"
        cv2.putText(image, f"Status: {safety_text}", (20, y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, safety_color, 2)
        y += line_h
        
        # Recommended speed
        cv2.putText(image, f"Speed: {output.recommended_speed:.0f} km/h", (20, y),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        # Warning banner (if collision warning)
        if output.collision_warning:
            cv2.rectangle(image, (0, h-60), (w, h), (0, 0, 255), -1)
            cv2.putText(image, "⚠ COLLISION WARNING ⚠", (w//2-150, h-20),
                       cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 3)
