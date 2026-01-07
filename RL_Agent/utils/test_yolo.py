#!/usr/bin/env python3
"""
Test script for YOLO Nano detector
"""

import sys
import numpy as np
import cv2
from pathlib import Path

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from models.yolo_detector import YOLONanoDetector


def test_yolo_detector():
    """Test YOLO Nano detector with sample image"""
    print("=" * 60)
    print("Testing YOLO Nano Detector")
    print("=" * 60)
    
    # Initialize detector
    print("\n[1/3] Initializing YOLO Nano detector...")
    detector = YOLONanoDetector(confidence_threshold=0.5)
    print("✅ Detector initialized")
    
    # Create test image (random RGB image)
    print("\n[2/3] Creating test image...")
    test_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    print(f"✅ Test image created: {test_image.shape}")
    
    # Run detection
    print("\n[3/3] Running detection...")
    detections = detector.detect(test_image)
    
    print(f"\n📊 Detection Results:")
    print(f"   Objects detected: {len(detections['boxes'])}")
    print(f"   Feature vector shape: {detections['features'].shape}")
    print(f"   Feature vector: {detections['features']}")
    
    if len(detections['boxes']) > 0:
        print(f"\n   Detected objects:")
        for i, (box, score, cls_name) in enumerate(zip(
            detections['boxes'],
            detections['scores'],
            detections['class_names']
        )):
            print(f"     {i+1}. {cls_name}: {score:.2f} at {box}")
    else:
        print("   (No objects detected in random image - this is expected)")
    
    print("\n✅ YOLO Nano detector test completed!")
    print("=" * 60)


if __name__ == '__main__':
    test_yolo_detector()

