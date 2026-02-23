"""
Compare different lane detection methods.

Methods to test:
1. CARLA waypoint projection (current)
2. U-Net model (if available)
3. Canny edge detection + Hough transform
4. Color-based segmentation
5. DeepLab/other segmentation models (if available)
"""

import sys
import numpy as np
import cv2
import time
from pathlib import Path
from typing import Dict, List, Tuple, Optional
import logging
import carla

# Add project root to path
project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))

from perception.lane_detector import LaneDetector, LaneUNet
from utils.device_utils import get_device
import torch

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class LaneDetectionComparator:
    """Compare different lane detection methods."""
    
    def __init__(self, world, vehicle, camera_transform=None):
        self.world = world
        self.vehicle = vehicle
        self.camera_transform = camera_transform
        self.results = {}
        
    def method_1_carla_waypoints(self, image: np.ndarray) -> Tuple[np.ndarray, float]:
        """Method 1: CARLA waypoint projection (current method)."""
        start_time = time.time()
        try:
            detector = LaneDetector(use_carla=True)
            mask, features = detector.detect_lanes(image, world=self.world, vehicle=self.vehicle)
            elapsed = time.time() - start_time
            return mask, elapsed
        except Exception as e:
            logger.error(f"Method 1 failed: {e}")
            return np.zeros(image.shape[:2], dtype=np.uint8), time.time() - start_time
    
    def method_2_unet_model(self, image: np.ndarray, model_path: Optional[str] = None) -> Tuple[np.ndarray, float]:
        """Method 2: U-Net model."""
        start_time = time.time()
        try:
            if model_path and Path(model_path).exists():
                detector = LaneDetector(model_path=model_path, use_carla=False)
                mask, features = detector.detect_lanes(image)
                elapsed = time.time() - start_time
                return mask, elapsed
            else:
                logger.warning("U-Net model not found, skipping")
                return np.zeros(image.shape[:2], dtype=np.uint8), time.time() - start_time
        except Exception as e:
            logger.error(f"Method 2 failed: {e}")
            return np.zeros(image.shape[:2], dtype=np.uint8), time.time() - start_time
    
    def method_3_canny_hough(self, image: np.ndarray) -> Tuple[np.ndarray, float]:
        """Method 3: Canny edge detection + Hough transform."""
        start_time = time.time()
        try:
            # Convert to grayscale
            if len(image.shape) == 3:
                gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
            else:
                gray = image
            
            # Apply Gaussian blur
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            
            # Canny edge detection
            edges = cv2.Canny(blurred, 50, 150)
            
            # ROI (Region of Interest) - lower half of image
            h, w = edges.shape
            roi_mask = np.zeros_like(edges)
            roi_mask[h//2:, :] = 1
            masked_edges = cv2.bitwise_and(edges, roi_mask)
            
            # Hough transform for line detection
            lines = cv2.HoughLinesP(
                masked_edges,
                rho=1,
                theta=np.pi/180,
                threshold=30,
                minLineLength=20,
                maxLineGap=10
            )
            
            # Create lane mask
            lane_mask = np.zeros((h, w), dtype=np.uint8)
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    cv2.line(lane_mask, (x1, y1), (x2, y2), 255, 3)
            
            elapsed = time.time() - start_time
            return lane_mask, elapsed
        except Exception as e:
            logger.error(f"Method 3 failed: {e}")
            return np.zeros(image.shape[:2], dtype=np.uint8), time.time() - start_time
    
    def method_4_color_segmentation(self, image: np.ndarray) -> Tuple[np.ndarray, float]:
        """Method 4: Color-based segmentation (white/yellow lane markings)."""
        start_time = time.time()
        try:
            # Convert to HSV
            hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
            
            # White lane markings
            lower_white = np.array([0, 0, 200])
            upper_white = np.array([180, 30, 255])
            mask_white = cv2.inRange(hsv, lower_white, upper_white)
            
            # Yellow lane markings
            lower_yellow = np.array([20, 100, 100])
            upper_yellow = np.array([30, 255, 255])
            mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
            
            # Combine masks
            lane_mask = cv2.bitwise_or(mask_white, mask_yellow)
            
            # Apply morphological operations to clean up
            kernel = np.ones((3, 3), np.uint8)
            lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_CLOSE, kernel)
            lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_OPEN, kernel)
            
            # ROI - lower half
            h, w = lane_mask.shape
            roi_mask = np.zeros_like(lane_mask)
            roi_mask[h//2:, :] = 1
            lane_mask = cv2.bitwise_and(lane_mask, roi_mask)
            
            elapsed = time.time() - start_time
            return lane_mask, elapsed
        except Exception as e:
            logger.error(f"Method 4 failed: {e}")
            return np.zeros(image.shape[:2], dtype=np.uint8), time.time() - start_time
    
    def method_5_adaptive_threshold(self, image: np.ndarray) -> Tuple[np.ndarray, float]:
        """Method 5: Adaptive threshold + morphological operations."""
        start_time = time.time()
        try:
            # Convert to grayscale
            if len(image.shape) == 3:
                gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
            else:
                gray = image
            
            # Apply adaptive threshold
            adaptive = cv2.adaptiveThreshold(
                gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY, 11, 2
            )
            
            # Invert (lane markings are usually bright)
            adaptive = cv2.bitwise_not(adaptive)
            
            # Morphological operations
            kernel = np.ones((5, 5), np.uint8)
            lane_mask = cv2.morphologyEx(adaptive, cv2.MORPH_CLOSE, kernel)
            lane_mask = cv2.morphologyEx(lane_mask, cv2.MORPH_OPEN, kernel)
            
            # ROI
            h, w = lane_mask.shape
            roi_mask = np.zeros_like(lane_mask)
            roi_mask[h//2:, :] = 1
            lane_mask = cv2.bitwise_and(lane_mask, roi_mask)
            
            elapsed = time.time() - start_time
            return lane_mask, elapsed
        except Exception as e:
            logger.error(f"Method 5 failed: {e}")
            return np.zeros(image.shape[:2], dtype=np.uint8), time.time() - start_time
    
    def calculate_iou(self, pred_mask: np.ndarray, gt_mask: np.ndarray) -> float:
        """Calculate Intersection over Union (IoU)."""
        if pred_mask.shape != gt_mask.shape:
            return 0.0
        
        pred_binary = (pred_mask > 0).astype(np.uint8)
        gt_binary = (gt_mask > 0).astype(np.uint8)
        
        intersection = np.logical_and(pred_binary, gt_binary).sum()
        union = np.logical_or(pred_binary, gt_binary).sum()
        
        if union == 0:
            return 1.0 if intersection == 0 else 0.0
        
        return intersection / union
    
    def calculate_accuracy(self, pred_mask: np.ndarray, gt_mask: np.ndarray) -> float:
        """Calculate pixel accuracy."""
        if pred_mask.shape != gt_mask.shape:
            return 0.0
        
        pred_binary = (pred_mask > 0).astype(np.uint8)
        gt_binary = (gt_mask > 0).astype(np.uint8)
        
        correct = (pred_binary == gt_binary).sum()
        total = pred_mask.size
        
        return correct / total if total > 0 else 0.0
    
    def get_ground_truth(self, image: np.ndarray) -> np.ndarray:
        """Get ground truth lane mask from CARLA."""
        try:
            detector = LaneDetector(use_carla=True)
            mask, _ = detector.detect_lanes(image, world=self.world, vehicle=self.vehicle)
            return mask
        except Exception as e:
            logger.error(f"Failed to get ground truth: {e}")
            return np.zeros(image.shape[:2], dtype=np.uint8)
    
    def compare_methods(self, image: np.ndarray, num_iterations: int = 10) -> Dict:
        """Compare all methods on the same image."""
        logger.info(f"Comparing {num_iterations} iterations...")
        
        # Get ground truth
        gt_mask = self.get_ground_truth(image)
        
        methods = {
            "1. CARLA Waypoints": self.method_1_carla_waypoints,
            "2. Canny + Hough": self.method_3_canny_hough,
            "3. Color Segmentation": self.method_4_color_segmentation,
            "4. Adaptive Threshold": self.method_5_adaptive_threshold,
        }
        
        # Check for U-Net model
        model_paths = [
            project_root / "data" / "autopilot_20260208_150902" / "resnet_lane_model" / "resnet_lane_final.pth",
            project_root / "models" / "lane_unet.pth",
        ]
        
        unet_model_path = None
        for path in model_paths:
            if path.exists():
                unet_model_path = str(path)
                methods["5. U-Net Model"] = lambda img: self.method_2_unet_model(img, unet_model_path)
                break
        
        results = {}
        
        for method_name, method_func in methods.items():
            logger.info(f"Testing {method_name}...")
            
            ious = []
            accuracies = []
            times = []
            
            for i in range(num_iterations):
                try:
                    mask, elapsed = method_func(image)
                    
                    # Calculate metrics
                    iou = self.calculate_iou(mask, gt_mask)
                    accuracy = self.calculate_accuracy(mask, gt_mask)
                    
                    ious.append(iou)
                    accuracies.append(accuracy)
                    times.append(elapsed)
                except Exception as e:
                    logger.error(f"{method_name} iteration {i+1} failed: {e}")
            
            if ious:
                results[method_name] = {
                    "mean_iou": np.mean(ious),
                    "std_iou": np.std(ious),
                    "mean_accuracy": np.mean(accuracies),
                    "std_accuracy": np.std(accuracies),
                    "mean_time": np.mean(times),
                    "std_time": np.std(times),
                    "min_time": np.min(times),
                    "max_time": np.max(times),
                }
        
        return results
    
    def print_results(self, results: Dict):
        """Print comparison results."""
        print("\n" + "="*80)
        print("LANE DETECTION METHODS COMPARISON")
        print("="*80)
        
        # Sort by IoU
        sorted_results = sorted(results.items(), key=lambda x: x[1]["mean_iou"], reverse=True)
        
        print(f"\n{'Method':<30} {'IoU':<15} {'Accuracy':<15} {'Time (ms)':<15}")
        print("-"*80)
        
        for method_name, metrics in sorted_results:
            print(f"{method_name:<30} "
                  f"{metrics['mean_iou']:.4f}±{metrics['std_iou']:.4f}  "
                  f"{metrics['mean_accuracy']:.4f}±{metrics['std_accuracy']:.4f}  "
                  f"{metrics['mean_time']*1000:.2f}±{metrics['std_time']*1000:.2f}")
        
        print("\n" + "="*80)
        print("WINNER BY METRIC:")
        print("="*80)
        
        # Best IoU
        best_iou = max(results.items(), key=lambda x: x[1]["mean_iou"])
        print(f"🏆 Best IoU: {best_iou[0]} ({best_iou[1]['mean_iou']:.4f})")
        
        # Best Accuracy
        best_acc = max(results.items(), key=lambda x: x[1]["mean_accuracy"])
        print(f"🏆 Best Accuracy: {best_acc[0]} ({best_acc[1]['mean_accuracy']:.4f})")
        
        # Fastest
        fastest = min(results.items(), key=lambda x: x[1]["mean_time"])
        print(f"⚡ Fastest: {fastest[0]} ({fastest[1]['mean_time']*1000:.2f} ms)")
        
        print("="*80 + "\n")


def main():
    """Main function to run comparison."""
    import argparse
    parser = argparse.ArgumentParser(description="Compare lane detection methods")
    parser.add_argument("--iterations", type=int, default=10, help="Number of iterations per method")
    parser.add_argument("--host", type=str, default="localhost", help="CARLA host")
    parser.add_argument("--port", type=int, default=2000, help="CARLA port")
    args = parser.parse_args()
    
    # Connect to CARLA
    try:
        from carla_env.carla_client import CarlaClient
        from carla_env.sensors import CameraSensor
        import yaml
        
        # Load config
        config_path = project_root / "config.yaml"
        if config_path.exists():
            with open(config_path, 'r') as f:
                full_config = yaml.safe_load(f)
                config = full_config.get('carla', {})
                camera_config = full_config.get('camera', {})
        else:
            # Default config
            config = {'host': args.host, 'port': args.port}
            camera_config = {'width': 640, 'height': 480, 'fov': 90, 'fps': 20,
                           'location': {'x': 2.0, 'y': 0.0, 'z': 1.4},
                           'rotation': {'pitch': 0.0, 'yaw': 0.0, 'roll': 0.0}}
        
        carla_client = CarlaClient(config)
        
        if not carla_client.connect():
            logger.error("Failed to connect to CARLA")
            return
        
        if not carla_client.load_world():
            logger.error("Failed to load CARLA world")
            return
        
        if not carla_client.spawn_vehicle():
            logger.error("Failed to spawn vehicle")
            return
        
        logger.info(f"Using vehicle: {carla_client.vehicle.type_id}")
        
        # Setup camera (use camera_config from above)
        camera = CameraSensor(carla_client.world, carla_client.vehicle, camera_config)
        
        # Wait for image (tick world in synchronous mode)
        image = None
        for i in range(20):
            # Tick world in synchronous mode
            carla_client.world.tick()
            time.sleep(0.1)
            image = camera.get_image_from_queue(timeout=0.5)
            if image is None:
                image = camera.get_image()
            if image is not None:
                break
            logger.info(f"Waiting for camera image... ({i+1}/20)")
        
        if image is None:
            logger.error("Failed to get camera image after 10 seconds")
            return
        
        logger.info(f"Got image: {image.shape}")
        
        # Create comparator
        comparator = LaneDetectionComparator(carla_client.world, carla_client.vehicle)
        
        # Run comparison
        results = comparator.compare_methods(image, num_iterations=args.iterations)
        
        # Print results
        comparator.print_results(results)
        
        # Cleanup
        camera.destroy()
        carla_client.cleanup()
        
    except Exception as e:
        logger.error(f"Failed to run comparison: {e}")
        import traceback
        traceback.print_exc()


if __name__ == "__main__":
    main()

