"""
Traffic Light Detection System
ตรวจจับไฟจราจรและสถานะ (แดง, เหลือง, เขียว)
"""

import cv2
import numpy as np
from typing import List, Optional, Tuple
from dataclasses import dataclass
from enum import Enum


class TrafficLightState(Enum):
    """สถานะไฟจราจร"""
    RED = 0
    YELLOW = 1
    GREEN = 2
    UNKNOWN = 3


@dataclass
class TrafficLight:
    """Traffic light detection result"""
    state: TrafficLightState
    confidence: float
    bbox: Tuple[int, int, int, int]  # x1, y1, x2, y2
    distance: Optional[float] = None


class TrafficLightDetector:
    """
    Traffic Light Detection แบบ Tesla
    - ตรวจจับไฟจราจร
    - จำแนกสถานะ (แดง, เหลือง, เขียว)
    - ประมาณระยะทาง
    """
    
    def __init__(self, confidence_threshold: float = 0.6):
        self.confidence_threshold = confidence_threshold
        
    def detect(
        self,
        image: np.ndarray,
        depth_map: Optional[np.ndarray] = None
    ) -> List[TrafficLight]:
        """
        ตรวจจับไฟจราจร
        
        Args:
            image: RGB image
            depth_map: Depth map (optional)
            
        Returns:
            List of TrafficLight objects
        """
        # Convert to HSV for color detection
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        
        traffic_lights = []
        
        # Detect red lights
        red_lights = self._detect_red_lights(hsv, image)
        traffic_lights.extend(red_lights)
        
        # Detect yellow lights
        yellow_lights = self._detect_yellow_lights(hsv, image)
        traffic_lights.extend(yellow_lights)
        
        # Detect green lights
        green_lights = self._detect_green_lights(hsv, image)
        traffic_lights.extend(green_lights)
        
        # Estimate distances
        if depth_map is not None:
            traffic_lights = self._estimate_distances(traffic_lights, depth_map)
        
        # Filter by confidence
        traffic_lights = [tl for tl in traffic_lights if tl.confidence >= self.confidence_threshold]
        
        return traffic_lights
    
    def _detect_red_lights(self, hsv: np.ndarray, image: np.ndarray) -> List[TrafficLight]:
        """ตรวจจับไฟแดง"""
        # Red color range in HSV
        lower_red1 = np.array([0, 120, 120])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 120, 120])
        upper_red2 = np.array([180, 255, 255])
        
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)
        
        return self._find_traffic_lights(red_mask, TrafficLightState.RED)
    
    def _detect_yellow_lights(self, hsv: np.ndarray, image: np.ndarray) -> List[TrafficLight]:
        """ตรวจจับไฟเหลือง"""
        # Yellow color range
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        
        yellow_mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        
        return self._find_traffic_lights(yellow_mask, TrafficLightState.YELLOW)
    
    def _detect_green_lights(self, hsv: np.ndarray, image: np.ndarray) -> List[TrafficLight]:
        """ตรวจจับไฟเขียว"""
        # Green color range
        lower_green = np.array([40, 100, 100])
        upper_green = np.array([80, 255, 255])
        
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        
        return self._find_traffic_lights(green_mask, TrafficLightState.GREEN)
    
    def _find_traffic_lights(
        self,
        mask: np.ndarray,
        state: TrafficLightState
    ) -> List[TrafficLight]:
        """หาไฟจราจรจาก mask"""
        # Morphological operations
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        traffic_lights = []
        
        for contour in contours:
            area = cv2.contourArea(contour)
            
            # Filter by area (traffic lights should be small)
            if 100 < area < 5000:
                x, y, w, h = cv2.boundingRect(contour)
                
                # Filter by aspect ratio (should be roughly circular or vertical)
                aspect_ratio = w / h if h > 0 else 0
                
                if 0.3 < aspect_ratio < 3.0:
                    # Calculate confidence based on area and circularity
                    perimeter = cv2.arcLength(contour, True)
                    circularity = 4 * np.pi * area / (perimeter * perimeter) if perimeter > 0 else 0
                    
                    confidence = min(circularity * 1.5, 1.0)
                    
                    traffic_light = TrafficLight(
                        state=state,
                        confidence=confidence,
                        bbox=(x, y, x+w, y+h)
                    )
                    traffic_lights.append(traffic_light)
        
        return traffic_lights
    
    def _estimate_distances(
        self,
        traffic_lights: List[TrafficLight],
        depth_map: np.ndarray
    ) -> List[TrafficLight]:
        """ประมาณระยะทาง"""
        for tl in traffic_lights:
            x1, y1, x2, y2 = tl.bbox
            
            # Get depth at traffic light center
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            
            if 0 <= cy < depth_map.shape[0] and 0 <= cx < depth_map.shape[1]:
                roi_depth = depth_map[y1:y2, x1:x2]
                if roi_depth.size > 0:
                    distance = np.median(roi_depth)
                    tl.distance = float(distance)
        
        return traffic_lights
    
    def visualize(self, image: np.ndarray, traffic_lights: List[TrafficLight]) -> np.ndarray:
        """วาดผลลัพธ์"""
        vis = image.copy()
        
        # Color map
        colors = {
            TrafficLightState.RED: (255, 0, 0),
            TrafficLightState.YELLOW: (255, 255, 0),
            TrafficLightState.GREEN: (0, 255, 0),
            TrafficLightState.UNKNOWN: (128, 128, 128)
        }
        
        for tl in traffic_lights:
            x1, y1, x2, y2 = tl.bbox
            color = colors[tl.state]
            
            # Draw bounding box
            cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
            
            # Draw label
            label = f"{tl.state.name} {tl.confidence:.2f}"
            if tl.distance is not None:
                label += f" {tl.distance:.1f}m"
            
            cv2.putText(vis, label, (x1, y1-5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        return vis
    
    def get_active_light(self, traffic_lights: List[TrafficLight]) -> Optional[TrafficLight]:
        """หาไฟที่ active (ใกล้ที่สุดและมี confidence สูง)"""
        if not traffic_lights:
            return None
        
        # Filter by confidence
        high_conf = [tl for tl in traffic_lights if tl.confidence > 0.7]
        
        if not high_conf:
            return None
        
        # Get closest
        with_distance = [tl for tl in high_conf if tl.distance is not None]
        
        if with_distance:
            return min(with_distance, key=lambda tl: tl.distance)
        
        return high_conf[0]
    
    def should_stop(self, traffic_lights: List[TrafficLight]) -> bool:
        """ตรวจสอบว่าควรหยุดหรือไม่"""
        active = self.get_active_light(traffic_lights)
        
        if active is None:
            return False
        
        # Stop if red or yellow
        return active.state in [TrafficLightState.RED, TrafficLightState.YELLOW]
