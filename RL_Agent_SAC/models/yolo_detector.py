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
    
    def __init__(
        self,
        model_path: Optional[str] = None,
        confidence_threshold: float = 0.5,
        device: str = 'cuda' if torch.cuda.is_available() else 'cpu'
    ):
        
        self.confidence_threshold = confidence_threshold
        self.device = device
        if YOLO is None:
            raise ImportError("ultralytics not installed. Install with: pip install ultralytics")
        if model_path is None:
            self.model = YOLO('yolov8n.pt')
        else:
            self.model = YOLO(model_path)
        self.model.eval()
        self.relevant_classes = {
            0: 'pedestrian',
            2: 'car',
            3: 'motorcycle',
            5: 'bus',
            7: 'truck'
        }
        print(f"✅ YOLO Nano detector initialized on {device}")
    def detect(self, image: np.ndarray) -> Dict:
        
        if image.dtype == np.float32 or image.dtype == np.float64:
            if image.max() <= 1.0:
                image = (image * 255).astype(np.uint8)
            else:
                image = image.astype(np.uint8)
        with torch.no_grad():
            results = self.model(image, verbose=False, conf=self.confidence_threshold, device='cpu')
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
                box = boxes.xyxy[i].cpu().numpy()
                conf = float(boxes.conf[i].cpu().numpy())
                cls = int(boxes.cls[i].cpu().numpy())
                if cls in self.relevant_classes:
                    detections['boxes'].append(box.tolist())
                    detections['scores'].append(conf)
                    detections['classes'].append(cls)
                    detections['class_names'].append(self.relevant_classes[cls])
        detections['features'] = self._create_feature_vector(detections, image.shape)
        return detections
    def _create_feature_vector(
        self,
        detections: Dict,
        image_shape: Tuple[int, int]
    ) -> np.ndarray:
        
        height, width = image_shape[:2]
        image_area = height * width
        num_objects = len(detections['boxes'])
        if num_objects == 0:
            return np.zeros(10, dtype=np.float32)
        avg_confidence = np.mean(detections['scores']) if detections['scores'] else 0.0
        image_center = np.array([width / 2, height / 2])
        min_distance = float('inf')
        nearest_class = 0
        for box in detections['boxes']:
            box_center = np.array([(box[0] + box[2]) / 2, (box[1] + box[3]) / 2])
            distance = np.linalg.norm(box_center - image_center)
            if distance < min_distance:
                min_distance = distance
                idx = detections['boxes'].index(box)
                nearest_class = detections['classes'][idx]
        max_distance = np.sqrt(width**2 + height**2)
        normalized_distance = min_distance / max_distance if max_distance > 0 else 0.0
        total_box_area = sum(
            (box[2] - box[0]) * (box[3] - box[1])
            for box in detections['boxes']
        )
        density = total_box_area / image_area if image_area > 0 else 0.0
        class_counts = {cls: 0 for cls in self.relevant_classes.keys()}
        for cls in detections['classes']:
            if cls in class_counts:
                class_counts[cls] += 1
        max_class_count = max(class_counts.values()) if class_counts.values() else 1
        class_distribution = [
            class_counts.get(cls, 0) / max_class_count
            for cls in sorted(self.relevant_classes.keys())
        ]
        features = np.array([
            num_objects / 10.0,
            avg_confidence,
            normalized_distance,
            nearest_class / 10.0,
            density,
            *class_distribution
        ], dtype=np.float32)
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
            cv2.rectangle(vis_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
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
    
    def __init__(
        self,
        feature_dim: int = 10,
        device: str = 'cuda' if torch.cuda.is_available() else 'cpu'
    ):
        super().__init__()
        self.feature_dim = feature_dim
        self.device = device
        self.detector = YOLONanoDetector(device='cpu')
    def forward(self, image: torch.Tensor) -> torch.Tensor:
        
        batch_size = image.shape[0]
        features_list = []
        if image.dim() == 4:
            if image.shape[1] == 3 or image.shape[1] == 4:
                image_np = image.permute(0, 2, 3, 1).cpu().numpy()
            else:
                image_np = image.cpu().numpy()
        else:
            raise ValueError(f"Unexpected image shape: {image.shape}")
        if image_np.max() <= 1.0:
            image_np = (image_np * 255).astype(np.uint8)
        else:
            image_np = image_np.astype(np.uint8)
        for i in range(batch_size):
            detections = self.detector.detect(image_np[i])
            features_list.append(detections['features'])
        features = torch.tensor(
            np.stack(features_list),
            dtype=torch.float32,
            device=self.device
        )
        return features