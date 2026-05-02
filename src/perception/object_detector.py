"""
Object Detection System
ตรวจจับวัตถุทุกประเภท: รถ, คน, จักรยาน, ป้าย ฯลฯ
ใช้ YOLO หรือ Faster R-CNN
"""

import cv2
import numpy as np
import torch
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass


@dataclass
class Detection:
    """Object detection result"""
    class_id: int
    class_name: str
    confidence: float
    bbox: Tuple[int, int, int, int]  # x1, y1, x2, y2
    distance: Optional[float] = None  # meters (if available)
    velocity: Optional[Tuple[float, float]] = None  # (vx, vy) m/s


class ObjectDetector:
    """
    Object Detection แบบ Tesla
    - ตรวจจับรถ, คน, จักรยาน, มอเตอร์ไซค์
    - ประมาณระยะทาง
    - Track objects across frames
    """
    
    # CARLA object classes
    CLASSES = {
        0: 'vehicle',
        1: 'pedestrian',
        2: 'bicycle',
        3: 'motorcycle',
        4: 'traffic_sign',
        5: 'traffic_light',
        6: 'obstacle'
    }
    
    def __init__(
        self,
        model_type: str = 'yolov5',
        model_path: Optional[str] = None,
        confidence_threshold: float = 0.5,
        nms_threshold: float = 0.4,
        use_gpu: bool = True
    ):
        self.model_type = model_type
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold
        self.device = torch.device('cuda' if use_gpu and torch.cuda.is_available() else 'cpu')
        
        # Load model
        self._load_model(model_path)
        
        # Object tracking
        self.tracked_objects = {}
        self.next_track_id = 0
        
    def _load_model(self, model_path: Optional[str]):
        """Load detection model"""
        if self.model_type == 'yolov5':
            try:
                # Try to load YOLOv5
                self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
                self.model.to(self.device)
                self.model.eval()
                print("✅ Loaded YOLOv5 model")
            except Exception as e:
                print(f"⚠️ Failed to load YOLOv5: {e}")
                print("   Using simple detection fallback")
                self.model = None
        else:
            print("⚠️ Using simple detection fallback")
            self.model = None
    
    def detect(
        self,
        image: np.ndarray,
        depth_map: Optional[np.ndarray] = None
    ) -> List[Detection]:
        """
        ตรวจจับวัตถุในภาพ
        
        Args:
            image: RGB image (H, W, 3)
            depth_map: Depth map for distance estimation (optional)
            
        Returns:
            List of Detection objects
        """
        if self.model is not None:
            # Use deep learning model
            detections = self._detect_with_dl(image)
        else:
            # Use simple fallback (for demo)
            detections = self._detect_simple(image)
        
        # Estimate distances if depth map available
        if depth_map is not None:
            detections = self._estimate_distances(detections, depth_map)
        
        # Track objects
        detections = self._track_objects(detections)
        
        return detections
    
    def _detect_with_dl(self, image: np.ndarray) -> List[Detection]:
        """ตรวจจับด้วย Deep Learning (YOLOv5)"""
        # Inference
        results = self.model(image)
        
        # Parse results
        detections = []
        pred = results.pred[0]  # predictions
        
        for *box, conf, cls in pred:
            if conf < self.confidence_threshold:
                continue
            
            x1, y1, x2, y2 = map(int, box)
            class_id = int(cls)
            
            # Map to CARLA classes
            class_name = self._map_to_carla_class(class_id, results.names[class_id])
            
            detection = Detection(
                class_id=class_id,
                class_name=class_name,
                confidence=float(conf),
                bbox=(x1, y1, x2, y2)
            )
            detections.append(detection)
        
        return detections
    
    def _detect_simple(self, image: np.ndarray) -> List[Detection]:
        """Simple detection fallback (for demo without model)"""
        # Use color-based detection as fallback
        detections = []
        
        # Detect red objects (potential vehicles/traffic lights)
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
        
        # Red mask
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)
        
        # Find contours
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:  # Minimum area
                x, y, w, h = cv2.boundingRect(contour)
                
                # Classify based on aspect ratio
                aspect_ratio = w / h if h > 0 else 0
                
                if 0.8 < aspect_ratio < 2.0:
                    class_name = 'vehicle'
                    class_id = 0
                else:
                    class_name = 'obstacle'
                    class_id = 6
                
                detection = Detection(
                    class_id=class_id,
                    class_name=class_name,
                    confidence=0.7,
                    bbox=(x, y, x+w, y+h)
                )
                detections.append(detection)
        
        return detections
    
    def _map_to_carla_class(self, yolo_class_id: int, yolo_class_name: str) -> str:
        """Map YOLO classes to CARLA classes"""
        # YOLO to CARLA mapping
        mapping = {
            'car': 'vehicle',
            'truck': 'vehicle',
            'bus': 'vehicle',
            'person': 'pedestrian',
            'bicycle': 'bicycle',
            'motorcycle': 'motorcycle',
            'traffic light': 'traffic_light',
            'stop sign': 'traffic_sign'
        }
        
        return mapping.get(yolo_class_name, 'obstacle')
    
    def _estimate_distances(
        self,
        detections: List[Detection],
        depth_map: np.ndarray
    ) -> List[Detection]:
        """ประมาณระยะทางจาก depth map"""
        for detection in detections:
            x1, y1, x2, y2 = detection.bbox
            
            # Get depth at object center
            cx = (x1 + x2) // 2
            cy = (y1 + y2) // 2
            
            if 0 <= cy < depth_map.shape[0] and 0 <= cx < depth_map.shape[1]:
                # Average depth in bounding box
                roi_depth = depth_map[y1:y2, x1:x2]
                if roi_depth.size > 0:
                    distance = np.median(roi_depth)
                    detection.distance = float(distance)
        
        return detections
    
    def _track_objects(self, detections: List[Detection]) -> List[Detection]:
        """Track objects across frames (simple IoU-based tracking)"""
        # TODO: Implement proper tracking (Kalman filter, SORT, etc.)
        return detections
    
    def visualize(self, image: np.ndarray, detections: List[Detection]) -> np.ndarray:
        """วาดผลลัพธ์บนภาพ"""
        vis = image.copy()
        
        # Color map for classes
        colors = {
            'vehicle': (0, 255, 0),      # Green
            'pedestrian': (255, 0, 0),   # Red
            'bicycle': (255, 255, 0),    # Yellow
            'motorcycle': (255, 165, 0), # Orange
            'traffic_light': (0, 255, 255), # Cyan
            'traffic_sign': (255, 0, 255),  # Magenta
            'obstacle': (128, 128, 128)  # Gray
        }
        
        for det in detections:
            x1, y1, x2, y2 = det.bbox
            color = colors.get(det.class_name, (255, 255, 255))
            
            # Draw bounding box
            cv2.rectangle(vis, (x1, y1), (x2, y2), color, 2)
            
            # Draw label
            label = f"{det.class_name} {det.confidence:.2f}"
            if det.distance is not None:
                label += f" {det.distance:.1f}m"
            
            # Background for text
            (w, h), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(vis, (x1, y1-20), (x1+w, y1), color, -1)
            cv2.putText(vis, label, (x1, y1-5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
        
        # Draw detection count
        cv2.putText(vis, f"Detections: {len(detections)}", (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        return vis
    
    def get_closest_object(
        self,
        detections: List[Detection],
        class_filter: Optional[List[str]] = None
    ) -> Optional[Detection]:
        """หาวัตถุที่ใกล้ที่สุด"""
        filtered = detections
        
        if class_filter:
            filtered = [d for d in detections if d.class_name in class_filter]
        
        if not filtered:
            return None
        
        # Sort by distance
        with_distance = [d for d in filtered if d.distance is not None]
        
        if not with_distance:
            return None
        
        return min(with_distance, key=lambda d: d.distance)
    
    def get_objects_in_path(
        self,
        detections: List[Detection],
        image_width: int,
        lane_center_x: Optional[int] = None,
        margin: int = 100
    ) -> List[Detection]:
        """หาวัตถุที่อยู่ในเส้นทาง"""
        if lane_center_x is None:
            lane_center_x = image_width // 2
        
        in_path = []
        for det in detections:
            x1, y1, x2, y2 = det.bbox
            obj_center_x = (x1 + x2) // 2
            
            # Check if object is in path
            if abs(obj_center_x - lane_center_x) < margin:
                in_path.append(det)
        
        return in_path
