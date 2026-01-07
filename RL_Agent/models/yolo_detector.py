"""
YOLO Nano Object Detection Module
Lightweight object detection for autonomous driving
"""

import torch
import torch.nn as nn
import numpy as np
from typing import List, Tuple, Dict, Optional
import cv2
try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None
    print("⚠️  ultralytics not installed. YOLO features will be disabled.")


class YOLONanoDetector:
    """
    YOLO Nano detector wrapper for CARLA environment
    Detects vehicles, pedestrians, and other objects
    """
    
    def __init__(
        self,
        model_path: Optional[str] = None,
        confidence_threshold: float = 0.5,
        device: str = 'cuda' if torch.cuda.is_available() else 'cpu'
    ):
        """
        Initialize YOLO Nano detector
        
        Args:
            model_path: Path to YOLO model weights (None = use default YOLOv8n)
            confidence_threshold: Minimum confidence for detections
            device: Device to run inference on ('cuda' or 'cpu')
        """
        self.confidence_threshold = confidence_threshold
        self.device = device
        
        # Load YOLO model
        if YOLO is None:
            raise ImportError("ultralytics not installed. Install with: pip install ultralytics")
        
        if model_path is None:
            # Use YOLOv8n (nano) as default - lightweight and fast
            self.model = YOLO('yolov8n.pt')  # Will download automatically
        else:
            self.model = YOLO(model_path)
        
        # Force CPU for YOLO to avoid ROCm issues
        # ultralytics YOLO handles device in predict() call
        self.model.eval()
        
        # CARLA-specific class mapping
        # YOLO COCO classes: 0=person, 2=car, 3=motorcycle, 5=bus, 7=truck
        self.relevant_classes = {
            0: 'pedestrian',
            2: 'car',
            3: 'motorcycle',
            5: 'bus',
            7: 'truck'
        }
        
        print(f"✅ YOLO Nano detector initialized on {device}")
    
    def detect(self, image: np.ndarray) -> Dict:
        """
        Detect objects in image
        
        Args:
            image: RGB image as numpy array (H, W, 3) in range [0, 255] or [0, 1]
        
        Returns:
            Dictionary with detection results:
            {
                'boxes': List of bounding boxes [[x1, y1, x2, y2], ...],
                'scores': List of confidence scores,
                'classes': List of class IDs,
                'class_names': List of class names,
                'features': Feature vector for RL (normalized)
            }
        """
        # Ensure image is in correct format
        if image.dtype == np.float32 or image.dtype == np.float64:
            if image.max() <= 1.0:
                image = (image * 255).astype(np.uint8)
            else:
                image = image.astype(np.uint8)
        
        # Run inference - force CPU to avoid ROCm issues
        with torch.no_grad():
            results = self.model(image, verbose=False, conf=self.confidence_threshold, device='cpu')
        
        # Parse results
        detections = {
            'boxes': [],
            'scores': [],
            'classes': [],
            'class_names': [],
            'features': None
        }
        
        if len(results) > 0 and results[0].boxes is not None:
            boxes = results[0].boxes
            
            for i in range(len(boxes)):
                box = boxes.xyxy[i].cpu().numpy()  # [x1, y1, x2, y2]
                conf = float(boxes.conf[i].cpu().numpy())
                cls = int(boxes.cls[i].cpu().numpy())
                
                # Only keep relevant classes
                if cls in self.relevant_classes:
                    detections['boxes'].append(box.tolist())
                    detections['scores'].append(conf)
                    detections['classes'].append(cls)
                    detections['class_names'].append(self.relevant_classes[cls])
        
        # Create feature vector for RL
        detections['features'] = self._create_feature_vector(detections, image.shape)
        
        return detections
    
    def _create_feature_vector(
        self,
        detections: Dict,
        image_shape: Tuple[int, int]
    ) -> np.ndarray:
        """
        Create normalized feature vector from detections for RL
        
        Returns:
            Feature vector: [num_objects, avg_confidence, 
                           nearest_object_distance, nearest_object_class,
                           object_density]
        """
        height, width = image_shape[:2]
        image_area = height * width
        
        num_objects = len(detections['boxes'])
        
        if num_objects == 0:
            # No detections: return zero vector
            return np.zeros(10, dtype=np.float32)
        
        # Average confidence
        avg_confidence = np.mean(detections['scores']) if detections['scores'] else 0.0
        
        # Find nearest object (center of image is reference)
        image_center = np.array([width / 2, height / 2])
        min_distance = float('inf')
        nearest_class = 0
        
        for box in detections['boxes']:
            box_center = np.array([(box[0] + box[2]) / 2, (box[1] + box[3]) / 2])
            distance = np.linalg.norm(box_center - image_center)
            if distance < min_distance:
                min_distance = distance
                # Get class of nearest object
                idx = detections['boxes'].index(box)
                nearest_class = detections['classes'][idx]
        
        # Normalize distance
        max_distance = np.sqrt(width**2 + height**2)
        normalized_distance = min_distance / max_distance if max_distance > 0 else 0.0
        
        # Object density (objects per unit area)
        total_box_area = sum(
            (box[2] - box[0]) * (box[3] - box[1])
            for box in detections['boxes']
        )
        density = total_box_area / image_area if image_area > 0 else 0.0
        
        # Class distribution (one-hot encoding for relevant classes)
        class_counts = {cls: 0 for cls in self.relevant_classes.keys()}
        for cls in detections['classes']:
            if cls in class_counts:
                class_counts[cls] += 1
        
        # Normalize class counts
        max_class_count = max(class_counts.values()) if class_counts.values() else 1
        class_distribution = [
            class_counts.get(cls, 0) / max_class_count
            for cls in sorted(self.relevant_classes.keys())
        ]
        
        # Combine features
        features = np.array([
            num_objects / 10.0,  # Normalized object count (max 10)
            avg_confidence,
            normalized_distance,
            nearest_class / 10.0,  # Normalized class ID
            density,
            *class_distribution  # 5 class distribution values
        ], dtype=np.float32)
        
        # Pad or truncate to fixed size (10 features)
        if len(features) < 10:
            features = np.pad(features, (0, 10 - len(features)), 'constant')
        elif len(features) > 10:
            features = features[:10]
        
        return features
    
    def visualize_detections(
        self,
        image: np.ndarray,
        detections: Dict,
        show: bool = True
    ) -> np.ndarray:
        """
        Visualize detections on image
        
        Args:
            image: Input image
            detections: Detection results from detect()
            show: Whether to display image
        
        Returns:
            Image with bounding boxes drawn
        """
        vis_image = image.copy()
        
        if image.dtype == np.float32 or image.dtype == np.float64:
            if vis_image.max() <= 1.0:
                vis_image = (vis_image * 255).astype(np.uint8)
        
        for box, score, cls_name in zip(
            detections['boxes'],
            detections['scores'],
            detections['class_names']
        ):
            x1, y1, x2, y2 = map(int, box)
            
            # Draw bounding box
            cv2.rectangle(vis_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # Draw label
            label = f"{cls_name}: {score:.2f}"
            cv2.putText(
                vis_image, label, (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2
            )
        
        if show:
            cv2.imshow('YOLO Detections', vis_image)
            cv2.waitKey(1)
        
        return vis_image


class YOLOFeatureExtractor(nn.Module):
    """
    PyTorch module for YOLO feature extraction
    Integrates YOLO detections into RL observation space
    """
    
    def __init__(
        self,
        feature_dim: int = 10,
        device: str = 'cuda' if torch.cuda.is_available() else 'cpu'
    ):
        super().__init__()
        self.feature_dim = feature_dim
        # YOLO will use CPU for inference (specified in detect() call)
        # This avoids ROCm compatibility issues
        self.device = device
        # Always use CPU for YOLO detector to avoid ROCm issues
        self.detector = YOLONanoDetector(device='cpu')
    
    def forward(self, image: torch.Tensor) -> torch.Tensor:
        """
        Extract YOLO features from image
        
        Args:
            image: Tensor of shape (batch, C, H, W) or (batch, H, W, C)
        
        Returns:
            Feature tensor of shape (batch, feature_dim)
        """
        batch_size = image.shape[0]
        features_list = []
        
        # Convert tensor to numpy
        if image.dim() == 4:
            # (batch, C, H, W) or (batch, H, W, C)
            if image.shape[1] == 3 or image.shape[1] == 4:
                # (batch, C, H, W)
                image_np = image.permute(0, 2, 3, 1).cpu().numpy()
            else:
                # (batch, H, W, C)
                image_np = image.cpu().numpy()
        else:
            raise ValueError(f"Unexpected image shape: {image.shape}")
        
        # Denormalize if needed
        if image_np.max() <= 1.0:
            image_np = (image_np * 255).astype(np.uint8)
        else:
            image_np = image_np.astype(np.uint8)
        
        # Process each image in batch
        for i in range(batch_size):
            detections = self.detector.detect(image_np[i])
            features_list.append(detections['features'])
        
        # Stack features
        features = torch.tensor(
            np.stack(features_list),
            dtype=torch.float32,
            device=self.device
        )
        
        return features

