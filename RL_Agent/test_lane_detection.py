"""
Test Lane Detection Algorithms
ทดสอบ edge detection และ lane detection algorithms ต่างๆ
เพื่อหาสาเหตุที่ reward ได้น้อย
"""

import numpy as np
import cv2
import os
import sys
import argparse
from typing import Dict, List, Tuple, Optional
import matplotlib.pyplot as plt
from pathlib import Path
import time

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from carla_env.lane_detector import LaneDetector


class EdgeDetectionTester:
    """ทดสอบ edge detection algorithms ต่างๆ"""
    
    def __init__(self, image_width: int = 160, image_height: int = 90):
        self.image_width = image_width
        self.image_height = image_height
        
    def canny_edge_detection(self, image: np.ndarray, low: int = 50, high: int = 150, 
                             blur_kernel: int = 5, blur_sigma: float = 0) -> Tuple[np.ndarray, np.ndarray]:
        """Canny Edge Detection"""
        gray = self._to_grayscale(image)
        blurred = cv2.GaussianBlur(gray, (blur_kernel, blur_kernel), blur_sigma)
        edges = cv2.Canny(blurred, low, high)
        return blurred, edges
    
    def sobel_edge_detection(self, image: np.ndarray, ksize: int = 3, 
                            blur_kernel: int = 5) -> Tuple[np.ndarray, np.ndarray]:
        """Sobel Edge Detection"""
        gray = self._to_grayscale(image)
        blurred = cv2.GaussianBlur(gray, (blur_kernel, blur_kernel), 0)
        
        # Sobel in X and Y directions
        sobelx = cv2.Sobel(blurred, cv2.CV_64F, 1, 0, ksize=ksize)
        sobely = cv2.Sobel(blurred, cv2.CV_64F, 0, 1, ksize=ksize)
        
        # Combine
        sobel_combined = np.sqrt(sobelx**2 + sobely**2)
        sobel_combined = np.uint8(255 * sobel_combined / sobel_combined.max())
        
        return blurred, sobel_combined
    
    def laplacian_edge_detection(self, image: np.ndarray, ksize: int = 3,
                                 blur_kernel: int = 5) -> Tuple[np.ndarray, np.ndarray]:
        """Laplacian Edge Detection"""
        gray = self._to_grayscale(image)
        blurred = cv2.GaussianBlur(gray, (blur_kernel, blur_kernel), 0)
        laplacian = cv2.Laplacian(blurred, cv2.CV_64F, ksize=ksize)
        laplacian = np.uint8(np.absolute(laplacian))
        return blurred, laplacian
    
    def adaptive_canny(self, image: np.ndarray, blur_kernel: int = 5) -> Tuple[np.ndarray, np.ndarray]:
        """Adaptive Canny (uses median for thresholds)"""
        gray = self._to_grayscale(image)
        blurred = cv2.GaussianBlur(gray, (blur_kernel, blur_kernel), 0)
        
        # Calculate median
        median = np.median(blurred)
        
        # Adaptive thresholds
        low = int(max(0, 0.7 * median))
        high = int(min(255, 1.3 * median))
        
        edges = cv2.Canny(blurred, low, high)
        return blurred, edges
    
    def multi_scale_canny(self, image: np.ndarray, scales: List[float] = [1.0, 0.5, 2.0],
                         blur_kernel: int = 5) -> Tuple[np.ndarray, np.ndarray]:
        """Multi-scale Canny Edge Detection"""
        gray = self._to_grayscale(image)
        blurred = cv2.GaussianBlur(gray, (blur_kernel, blur_kernel), 0)
        
        all_edges = []
        for scale in scales:
            if scale != 1.0:
                h, w = blurred.shape
                scaled = cv2.resize(blurred, (int(w*scale), int(h*scale)))
                edges = cv2.Canny(scaled, 50, 150)
                edges = cv2.resize(edges, (w, h))
            else:
                edges = cv2.Canny(blurred, 50, 150)
            all_edges.append(edges)
        
        # Combine all scales
        combined = np.maximum.reduce(all_edges)
        return blurred, combined
    
    def _to_grayscale(self, image: np.ndarray) -> np.ndarray:
        """Convert image to grayscale"""
        if len(image.shape) == 3:
            if image.max() <= 1.0:
                gray = (image.mean(axis=2) * 255).astype(np.uint8)
            else:
                gray = cv2.cvtColor(image.astype(np.uint8), cv2.COLOR_RGB2GRAY)
        else:
            gray = image.astype(np.uint8) if image.max() <= 1.0 else image
        return gray


class LaneDetectionTester:
    """ทดสอบ lane detection ด้วย algorithms ต่างๆ"""
    
    def __init__(self, image_width: int = 160, image_height: int = 90):
        self.image_width = image_width
        self.image_height = image_height
        self.edge_tester = EdgeDetectionTester(image_width, image_height)
        
        # ROI
        self.roi_top = int(image_height * 0.5)
        self.roi_bottom = image_height
        self.roi_left = 0
        self.roi_right = image_width
        
    def test_all_algorithms(self, image: np.ndarray) -> Dict[str, Dict]:
        """ทดสอบ algorithms ทั้งหมด"""
        results = {}
        
        # 1. Original Canny (current implementation)
        print("Testing: Original Canny (50, 150)")
        results['canny_original'] = self._test_canny_params(image, 50, 150)
        
        # 2. Lower threshold Canny (more sensitive)
        print("Testing: Lower Threshold Canny (30, 100)")
        results['canny_lower'] = self._test_canny_params(image, 30, 100)
        
        # 3. Higher threshold Canny (less sensitive)
        print("Testing: Higher Threshold Canny (100, 200)")
        results['canny_higher'] = self._test_canny_params(image, 100, 200)
        
        # 4. Adaptive Canny
        print("Testing: Adaptive Canny")
        results['canny_adaptive'] = self._test_adaptive_canny(image)
        
        # 5. Sobel
        print("Testing: Sobel")
        results['sobel'] = self._test_sobel(image)
        
        # 6. Laplacian
        print("Testing: Laplacian")
        results['laplacian'] = self._test_laplacian(image)
        
        # 7. Multi-scale Canny
        print("Testing: Multi-scale Canny")
        results['canny_multiscale'] = self._test_multiscale_canny(image)
        
        # 8. Enhanced Canny (better preprocessing)
        print("Testing: Enhanced Canny (better preprocessing)")
        results['canny_enhanced'] = self._test_enhanced_canny(image)
        
        return results
    
    def _test_canny_params(self, image: np.ndarray, low: int, high: int) -> Dict:
        """ทดสอบ Canny ด้วย parameters ต่างๆ"""
        blurred, edges = self.edge_tester.canny_edge_detection(image, low, high)
        
        # Apply ROI
        roi_mask = np.zeros_like(edges)
        roi_mask[self.roi_top:self.roi_bottom, self.roi_left:self.roi_right] = 1
        masked_edges = cv2.bitwise_and(edges, roi_mask)
        
        # Hough Line Transform
        lines = cv2.HoughLinesP(
            masked_edges,
            rho=1,
            theta=np.pi/180,
            threshold=30,
            minLineLength=20,
            maxLineGap=10
        )
        
        # Count edges
        edge_count = np.sum(masked_edges > 0)
        edge_density = edge_count / (masked_edges.shape[0] * masked_edges.shape[1])
        
        # Count lines
        line_count = len(lines) if lines is not None else 0
        
        # Try to detect lanes
        left_lane, right_lane = self._detect_lanes_from_lines(lines)
        
        return {
            'blurred': blurred,
            'edges': edges,
            'masked_edges': masked_edges,
            'edge_count': edge_count,
            'edge_density': edge_density,
            'line_count': line_count,
            'left_lane_detected': left_lane is not None,
            'right_lane_detected': right_lane is not None,
            'both_lanes_detected': (left_lane is not None and right_lane is not None),
            'left_lane_points': len(left_lane) if left_lane is not None else 0,
            'right_lane_points': len(right_lane) if right_lane is not None else 0,
            'lines': lines
        }
    
    def _test_adaptive_canny(self, image: np.ndarray) -> Dict:
        """ทดสอบ Adaptive Canny"""
        blurred, edges = self.edge_tester.adaptive_canny(image)
        
        roi_mask = np.zeros_like(edges)
        roi_mask[self.roi_top:self.roi_bottom, self.roi_left:self.roi_right] = 1
        masked_edges = cv2.bitwise_and(edges, roi_mask)
        
        lines = cv2.HoughLinesP(
            masked_edges,
            rho=1,
            theta=np.pi/180,
            threshold=30,
            minLineLength=20,
            maxLineGap=10
        )
        
        edge_count = np.sum(masked_edges > 0)
        edge_density = edge_count / (masked_edges.shape[0] * masked_edges.shape[1])
        line_count = len(lines) if lines is not None else 0
        
        left_lane, right_lane = self._detect_lanes_from_lines(lines)
        
        return {
            'blurred': blurred,
            'edges': edges,
            'masked_edges': masked_edges,
            'edge_count': edge_count,
            'edge_density': edge_density,
            'line_count': line_count,
            'left_lane_detected': left_lane is not None,
            'right_lane_detected': right_lane is not None,
            'both_lanes_detected': (left_lane is not None and right_lane is not None),
            'left_lane_points': len(left_lane) if left_lane is not None else 0,
            'right_lane_points': len(right_lane) if right_lane is not None else 0,
            'lines': lines
        }
    
    def _test_sobel(self, image: np.ndarray) -> Dict:
        """ทดสอบ Sobel"""
        blurred, edges = self.edge_tester.sobel_edge_detection(image)
        
        # Threshold Sobel output
        _, edges_thresh = cv2.threshold(edges, 50, 255, cv2.THRESH_BINARY)
        
        roi_mask = np.zeros_like(edges_thresh)
        roi_mask[self.roi_top:self.roi_bottom, self.roi_left:self.roi_right] = 1
        masked_edges = cv2.bitwise_and(edges_thresh, roi_mask)
        
        lines = cv2.HoughLinesP(
            masked_edges,
            rho=1,
            theta=np.pi/180,
            threshold=30,
            minLineLength=20,
            maxLineGap=10
        )
        
        edge_count = np.sum(masked_edges > 0)
        edge_density = edge_count / (masked_edges.shape[0] * masked_edges.shape[1])
        line_count = len(lines) if lines is not None else 0
        
        left_lane, right_lane = self._detect_lanes_from_lines(lines)
        
        return {
            'blurred': blurred,
            'edges': edges,
            'masked_edges': masked_edges,
            'edge_count': edge_count,
            'edge_density': edge_density,
            'line_count': line_count,
            'left_lane_detected': left_lane is not None,
            'right_lane_detected': right_lane is not None,
            'both_lanes_detected': (left_lane is not None and right_lane is not None),
            'left_lane_points': len(left_lane) if left_lane is not None else 0,
            'right_lane_points': len(right_lane) if right_lane is not None else 0,
            'lines': lines
        }
    
    def _test_laplacian(self, image: np.ndarray) -> Dict:
        """ทดสอบ Laplacian"""
        blurred, edges = self.edge_tester.laplacian_edge_detection(image)
        
        # Threshold Laplacian output
        _, edges_thresh = cv2.threshold(edges, 30, 255, cv2.THRESH_BINARY)
        
        roi_mask = np.zeros_like(edges_thresh)
        roi_mask[self.roi_top:self.roi_bottom, self.roi_left:self.roi_right] = 1
        masked_edges = cv2.bitwise_and(edges_thresh, roi_mask)
        
        lines = cv2.HoughLinesP(
            masked_edges,
            rho=1,
            theta=np.pi/180,
            threshold=30,
            minLineLength=20,
            maxLineGap=10
        )
        
        edge_count = np.sum(masked_edges > 0)
        edge_density = edge_count / (masked_edges.shape[0] * masked_edges.shape[1])
        line_count = len(lines) if lines is not None else 0
        
        left_lane, right_lane = self._detect_lanes_from_lines(lines)
        
        return {
            'blurred': blurred,
            'edges': edges,
            'masked_edges': masked_edges,
            'edge_count': edge_count,
            'edge_density': edge_density,
            'line_count': line_count,
            'left_lane_detected': left_lane is not None,
            'right_lane_detected': right_lane is not None,
            'both_lanes_detected': (left_lane is not None and right_lane is not None),
            'left_lane_points': len(left_lane) if left_lane is not None else 0,
            'right_lane_points': len(right_lane) if right_lane is not None else 0,
            'lines': lines
        }
    
    def _test_multiscale_canny(self, image: np.ndarray) -> Dict:
        """ทดสอบ Multi-scale Canny"""
        blurred, edges = self.edge_tester.multi_scale_canny(image)
        
        roi_mask = np.zeros_like(edges)
        roi_mask[self.roi_top:self.roi_bottom, self.roi_left:self.roi_right] = 1
        masked_edges = cv2.bitwise_and(edges, roi_mask)
        
        lines = cv2.HoughLinesP(
            masked_edges,
            rho=1,
            theta=np.pi/180,
            threshold=30,
            minLineLength=20,
            maxLineGap=10
        )
        
        edge_count = np.sum(masked_edges > 0)
        edge_density = edge_count / (masked_edges.shape[0] * masked_edges.shape[1])
        line_count = len(lines) if lines is not None else 0
        
        left_lane, right_lane = self._detect_lanes_from_lines(lines)
        
        return {
            'blurred': blurred,
            'edges': edges,
            'masked_edges': masked_edges,
            'edge_count': edge_count,
            'edge_density': edge_density,
            'line_count': line_count,
            'left_lane_detected': left_lane is not None,
            'right_lane_detected': right_lane is not None,
            'both_lanes_detected': (left_lane is not None and right_lane is not None),
            'left_lane_points': len(left_lane) if left_lane is not None else 0,
            'right_lane_points': len(right_lane) if right_lane is not None else 0,
            'lines': lines
        }
    
    def _test_enhanced_canny(self, image: np.ndarray) -> Dict:
        """Enhanced Canny with better preprocessing"""
        gray = self.edge_tester._to_grayscale(image)
        
        # Enhanced preprocessing
        # 1. Histogram equalization
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        equalized = clahe.apply(gray)
        
        # 2. Bilateral filter (preserves edges)
        filtered = cv2.bilateralFilter(equalized, 9, 75, 75)
        
        # 3. Adaptive Canny
        median = np.median(filtered)
        low = int(max(0, 0.5 * median))
        high = int(min(255, 1.5 * median))
        
        edges = cv2.Canny(filtered, low, high)
        
        roi_mask = np.zeros_like(edges)
        roi_mask[self.roi_top:self.roi_bottom, self.roi_left:self.roi_right] = 1
        masked_edges = cv2.bitwise_and(edges, roi_mask)
        
        lines = cv2.HoughLinesP(
            masked_edges,
            rho=1,
            theta=np.pi/180,
            threshold=20,  # Lower threshold for more lines
            minLineLength=15,  # Shorter lines
            maxLineGap=15  # Larger gap tolerance
        )
        
        edge_count = np.sum(masked_edges > 0)
        edge_density = edge_count / (masked_edges.shape[0] * masked_edges.shape[1])
        line_count = len(lines) if lines is not None else 0
        
        left_lane, right_lane = self._detect_lanes_from_lines(lines)
        
        return {
            'blurred': filtered,
            'edges': edges,
            'masked_edges': masked_edges,
            'edge_count': edge_count,
            'edge_density': edge_density,
            'line_count': line_count,
            'left_lane_detected': left_lane is not None,
            'right_lane_detected': right_lane is not None,
            'both_lanes_detected': (left_lane is not None and right_lane is not None),
            'left_lane_points': len(left_lane) if left_lane is not None else 0,
            'right_lane_points': len(right_lane) if right_lane is not None else 0,
            'lines': lines
        }
    
    def _detect_lanes_from_lines(self, lines) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
        """Detect left and right lanes from Hough lines"""
        if lines is None or len(lines) == 0:
            return None, None
        
        left_points = []
        right_points = []
        
        for line in lines:
            x1, y1, x2, y2 = line[0]
            
            if x2 - x1 == 0:
                continue
            
            slope = (y2 - y1) / (x2 - x1)
            
            # Filter by slope
            if abs(slope) < 0.3:
                continue
            
            if slope < 0:  # Left lane
                left_points.extend([(x1, y1), (x2, y2)])
            else:  # Right lane
                right_points.extend([(x1, y1), (x2, y2)])
        
        left_lane = np.array(left_points) if left_points else None
        right_lane = np.array(right_points) if right_points else None
        
        return left_lane, right_lane


def load_image_from_carla(image_path: str) -> Optional[np.ndarray]:
    """Load image from CARLA (RGB format)"""
    if not os.path.exists(image_path):
        return None
    
    # Try to load as image
    image = cv2.imread(image_path)
    if image is not None:
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        return image
    
    # Try to load as numpy array
    try:
        image = np.load(image_path)
        return image
    except:
        pass
    
    return None


def capture_image_from_training() -> Optional[np.ndarray]:
    """Capture current image from training (if possible)"""
    # Try to get image from CARLA environment
    try:
        import carla
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)
        world = client.get_world()
        
        # Find a vehicle
        vehicles = world.get_actors().filter('vehicle.*')
        if len(vehicles) == 0:
            return None
        
        vehicle = vehicles[0]
        
        # Create camera
        blueprint_library = world.get_blueprint_library()
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '160')
        camera_bp.set_attribute('image_size_y', '90')
        
        transform = carla.Transform(carla.Location(x=2.5, z=0.7))
        camera = world.spawn_actor(camera_bp, transform, attach_to=vehicle)
        
        # Capture image
        image_data = None
        def callback(image):
            nonlocal image_data
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]  # Remove alpha
            image_data = array
        
        camera.listen(callback)
        time.sleep(1.0)
        camera.destroy()
        
        return image_data
    except Exception as e:
        print(f"Could not capture from CARLA: {e}")
        return None


def create_test_image() -> np.ndarray:
    """Create synthetic test image with lanes"""
    image = np.zeros((90, 160, 3), dtype=np.uint8)
    
    # Road (gray)
    image[45:, :] = [100, 100, 100]
    
    # Left lane line (white)
    for y in range(45, 90):
        x = 50 + int(0.1 * (y - 45))
        if 0 <= x < 160:
            image[y, max(0, x-2):min(160, x+2), :] = [255, 255, 255]
    
    # Right lane line (white)
    for y in range(45, 90):
        x = 110 + int(0.1 * (y - 45))
        if 0 <= x < 160:
            image[y, max(0, x-2):min(160, x+2), :] = [255, 255, 255]
    
    return image


def visualize_results(image: np.ndarray, results: Dict[str, Dict], output_dir: str = "test_output"):
    """Visualize test results"""
    os.makedirs(output_dir, exist_ok=True)
    
    # Create summary figure
    n_algorithms = len(results)
    fig, axes = plt.subplots(n_algorithms, 3, figsize=(15, 5 * n_algorithms))
    
    if n_algorithms == 1:
        axes = axes.reshape(1, -1)
    
    for idx, (name, result) in enumerate(results.items()):
        ax1, ax2, ax3 = axes[idx]
        
        # Original image
        ax1.imshow(image)
        ax1.set_title(f'{name}\nOriginal Image')
        ax1.axis('off')
        
        # Edges
        ax2.imshow(result['masked_edges'], cmap='gray')
        ax2.set_title(f'Edges\nDensity: {result["edge_density"]:.3f}, Lines: {result["line_count"]}')
        ax2.axis('off')
        
        # Overlay lines on original
        overlay = image.copy()
        if result['lines'] is not None:
            for line in result['lines']:
                x1, y1, x2, y2 = line[0]
                cv2.line(overlay, (x1, y1), (x2, y2), (0, 255, 0), 2)
        
        ax3.imshow(overlay)
        ax3.set_title(f'Detected Lines\nLeft: {result["left_lane_detected"]}, Right: {result["right_lane_detected"]}')
        ax3.axis('off')
    
    plt.tight_layout()
    plt.savefig(os.path.join(output_dir, 'all_algorithms_comparison.png'), dpi=150, bbox_inches='tight')
    print(f"Saved comparison to {output_dir}/all_algorithms_comparison.png")
    
    # Create summary report
    report_path = os.path.join(output_dir, 'test_report.txt')
    with open(report_path, 'w') as f:
        f.write("=" * 80 + "\n")
        f.write("LANE DETECTION ALGORITHM TEST REPORT\n")
        f.write("=" * 80 + "\n\n")
        
        for name, result in results.items():
            f.write(f"\n{name.upper()}\n")
            f.write("-" * 80 + "\n")
            f.write(f"Edge Count: {result['edge_count']}\n")
            f.write(f"Edge Density: {result['edge_density']:.4f}\n")
            f.write(f"Line Count: {result['line_count']}\n")
            f.write(f"Left Lane Detected: {result['left_lane_detected']}\n")
            f.write(f"Right Lane Detected: {result['right_lane_detected']}\n")
            f.write(f"Both Lanes Detected: {result['both_lanes_detected']}\n")
            f.write(f"Left Lane Points: {result['left_lane_points']}\n")
            f.write(f"Right Lane Points: {result['right_lane_points']}\n")
            f.write("\n")
        
        # Find best algorithm
        best = max(results.items(), 
                  key=lambda x: (x[1]['both_lanes_detected'], x[1]['line_count'], x[1]['edge_density']))
        f.write("\n" + "=" * 80 + "\n")
        f.write(f"BEST ALGORITHM: {best[0]}\n")
        f.write("=" * 80 + "\n")
    
    print(f"Saved report to {report_path}")


def main():
    parser = argparse.ArgumentParser(description='Test Lane Detection Algorithms')
    parser.add_argument('--image', type=str, help='Path to test image')
    parser.add_argument('--capture', action='store_true', help='Capture image from CARLA')
    parser.add_argument('--synthetic', action='store_true', help='Use synthetic test image')
    parser.add_argument('--output', type=str, default='test_output', help='Output directory')
    args = parser.parse_args()
    
    # Load or create image
    image = None
    
    if args.capture:
        print("Capturing image from CARLA...")
        image = capture_image_from_training()
        if image is None:
            print("Failed to capture from CARLA, using synthetic image")
            image = create_test_image()
    elif args.image:
        print(f"Loading image from {args.image}...")
        image = load_image_from_carla(args.image)
        if image is None:
            print("Failed to load image, using synthetic image")
            image = create_test_image()
    else:
        print("Using synthetic test image...")
        image = create_test_image()
    
    if image is None:
        print("ERROR: Could not load or create image")
        return
    
    # Resize if needed
    if image.shape[:2] != (90, 160):
        image = cv2.resize(image, (160, 90))
        print(f"Resized image to {image.shape}")
    
    print(f"Image shape: {image.shape}")
    print(f"Image dtype: {image.dtype}")
    print(f"Image range: [{image.min()}, {image.max()}]")
    
    # Test all algorithms
    tester = LaneDetectionTester(image_width=160, image_height=90)
    print("\n" + "=" * 80)
    print("TESTING ALL ALGORITHMS")
    print("=" * 80 + "\n")
    
    results = tester.test_all_algorithms(image)
    
    # Print summary
    print("\n" + "=" * 80)
    print("RESULTS SUMMARY")
    print("=" * 80)
    for name, result in results.items():
        print(f"\n{name}:")
        print(f"  Edge Density: {result['edge_density']:.4f}")
        print(f"  Line Count: {result['line_count']}")
        print(f"  Left Lane: {'✓' if result['left_lane_detected'] else '✗'}")
        print(f"  Right Lane: {'✓' if result['right_lane_detected'] else '✗'}")
        print(f"  Both Lanes: {'✓' if result['both_lanes_detected'] else '✗'}")
    
    # Find best
    best = max(results.items(), 
              key=lambda x: (x[1]['both_lanes_detected'], x[1]['line_count'], x[1]['edge_density']))
    print(f"\n{'=' * 80}")
    print(f"BEST ALGORITHM: {best[0]}")
    print(f"{'=' * 80}")
    
    # Visualize
    visualize_results(image, results, args.output)
    
    print(f"\n✅ Testing complete! Results saved to {args.output}/")


if __name__ == '__main__':
    main()

