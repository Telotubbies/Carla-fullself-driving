"""
Fine-tune Lane Detection Parameters
ทดสอบและปรับแต่ง parameters ต่างๆ เพื่อหาค่าที่เหมาะสมที่สุด
"""

import numpy as np
import cv2
import os
import sys
import argparse
from typing import Dict, List, Tuple, Optional
import matplotlib.pyplot as plt
from pathlib import Path
import itertools
import json

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from carla_env.lane_detector import LaneDetector


class ParameterTuner:
    """Fine-tune parameters สำหรับ lane detection"""
    
    def __init__(self, image_width: int = 160, image_height: int = 90):
        self.image_width = image_width
        self.image_height = image_height
        
    def create_test_image(self, difficulty: str = 'medium') -> np.ndarray:
        """
        Create synthetic test images with different difficulty levels
        
        Args:
            difficulty: 'easy', 'medium', 'hard'
        """
        image = np.zeros((90, 160, 3), dtype=np.uint8)
        
        # Road (gray)
        road_color = [100, 100, 100]
        if difficulty == 'hard':
            road_color = [80, 80, 80]  # Darker road
        image[45:, :] = road_color
        
        # Add some noise
        if difficulty == 'hard':
            noise = np.random.randint(-20, 20, image[45:, :].shape, dtype=np.int16)
            image[45:, :] = np.clip(image[45:, :].astype(np.int16) + noise, 0, 255).astype(np.uint8)
        
        # Left lane line (white/yellow)
        left_color = [255, 255, 255]
        if difficulty == 'hard':
            left_color = [200, 200, 200]  # Less bright
        
        for y in range(45, 90):
            x = 50 + int(0.1 * (y - 45))
            if 0 <= x < 160:
                width = 2 if difficulty == 'easy' else (1 if difficulty == 'medium' else 1)
                image[y, max(0, x-width):min(160, x+width), :] = left_color
        
        # Right lane line (white/yellow)
        for y in range(45, 90):
            x = 110 + int(0.1 * (y - 45))
            if 0 <= x < 160:
                width = 2 if difficulty == 'easy' else (1 if difficulty == 'medium' else 1)
                image[y, max(0, x-width):min(160, x+width), :] = left_color
        
        return image
    
    def test_canny_parameters(self, image: np.ndarray, 
                             low_range: Tuple[int, int] = (30, 100),
                             high_range: Tuple[int, int] = (100, 200),
                             step: int = 10) -> Dict:
        """ทดสอบ Canny thresholds ต่างๆ"""
        results = []
        
        for low in range(low_range[0], low_range[1] + 1, step):
            for high in range(high_range[0], high_range[1] + 1, step):
                if low >= high:
                    continue
                
                # Test with these parameters
                gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY) if len(image.shape) == 3 else image
                blurred = cv2.GaussianBlur(gray, (5, 5), 0)
                edges = cv2.Canny(blurred, low, high)
                
                # Apply ROI
                roi_mask = np.zeros_like(edges)
                roi_mask[45:90, 0:160] = 1
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
                
                # Evaluate
                left_lane, right_lane = self._detect_lanes_from_lines(lines)
                edge_count = np.sum(masked_edges > 0)
                edge_density = edge_count / (masked_edges.shape[0] * masked_edges.shape[1])
                line_count = len(lines) if lines is not None else 0
                
                score = self._calculate_score(left_lane, right_lane, edge_density, line_count)
                
                results.append({
                    'low': low,
                    'high': high,
                    'left_lane': left_lane is not None,
                    'right_lane': right_lane is not None,
                    'both_lanes': left_lane is not None and right_lane is not None,
                    'edge_density': edge_density,
                    'line_count': line_count,
                    'score': score
                })
        
        # Find best
        best = max(results, key=lambda x: x['score'])
        return {
            'results': results,
            'best': best,
            'total_tests': len(results)
        }
    
    def test_hough_parameters(self, image: np.ndarray,
                             threshold_range: Tuple[int, int] = (10, 50),
                             min_line_len_range: Tuple[int, int] = (10, 30),
                             max_line_gap_range: Tuple[int, int] = (5, 20),
                             step: int = 5) -> Dict:
        """ทดสอบ Hough Line Transform parameters"""
        results = []
        
        # Use best Canny parameters
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY) if len(image.shape) == 3 else image
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)
        
        roi_mask = np.zeros_like(edges)
        roi_mask[45:90, 0:160] = 1
        masked_edges = cv2.bitwise_and(edges, roi_mask)
        
        for threshold in range(threshold_range[0], threshold_range[1] + 1, step):
            for min_line_len in range(min_line_len_range[0], min_line_len_range[1] + 1, step):
                for max_line_gap in range(max_line_gap_range[0], max_line_gap_range[1] + 1, step):
                    lines = cv2.HoughLinesP(
                        masked_edges,
                        rho=1,
                        theta=np.pi/180,
                        threshold=threshold,
                        minLineLength=min_line_len,
                        maxLineGap=max_line_gap
                    )
                    
                    left_lane, right_lane = self._detect_lanes_from_lines(lines)
                    line_count = len(lines) if lines is not None else 0
                    
                    score = self._calculate_score(left_lane, right_lane, 0, line_count)
                    
                    results.append({
                        'threshold': threshold,
                        'min_line_len': min_line_len,
                        'max_line_gap': max_line_gap,
                        'left_lane': left_lane is not None,
                        'right_lane': right_lane is not None,
                        'both_lanes': left_lane is not None and right_lane is not None,
                        'line_count': line_count,
                        'score': score
                    })
        
        best = max(results, key=lambda x: x['score'])
        return {
            'results': results,
            'best': best,
            'total_tests': len(results)
        }
    
    def test_multiscale_parameters(self, image: np.ndarray,
                                  scale_combinations: List[List[float]] = None) -> Dict:
        """ทดสอบ Multi-scale Canny scales"""
        if scale_combinations is None:
            # Common scale combinations
            scale_combinations = [
                [1.0, 0.5, 2.0],
                [1.0, 0.75, 1.5],
                [1.0, 0.5, 1.5],
                [1.0, 0.5, 1.25, 2.0],
                [0.75, 1.0, 1.5],
            ]
        
        results = []
        
        for scales in scale_combinations:
            # Create detector with these scales
            detector = LaneDetector(
                image_width=self.image_width,
                image_height=self.image_height,
                edge_detection_method='multiscale_canny'
            )
            detector.multiscale_scales = scales
            
            left_lane, right_lane = detector.detect_lanes(image)
            
            score = self._calculate_score(left_lane, right_lane, 0, 0)
            
            results.append({
                'scales': scales,
                'left_lane': left_lane is not None,
                'right_lane': right_lane is not None,
                'both_lanes': left_lane is not None and right_lane is not None,
                'left_points': len(left_lane) if left_lane is not None else 0,
                'right_points': len(right_lane) if right_lane is not None else 0,
                'score': score
            })
        
        best = max(results, key=lambda x: x['score'])
        return {
            'results': results,
            'best': best,
            'total_tests': len(results)
        }
    
    def test_enhanced_canny_parameters(self, image: np.ndarray) -> Dict:
        """ทดสอบ Enhanced Canny parameters"""
        results = []
        
        clahe_limits = [1.5, 2.0, 2.5, 3.0]
        bilateral_ds = [5, 7, 9, 11]
        bilateral_sigmas = [50, 75, 100]
        
        for clahe_limit in clahe_limits:
            for bilateral_d in bilateral_ds:
                for sigma_color in bilateral_sigmas:
                    sigma_space = sigma_color
                    
                    # Create detector
                    detector = LaneDetector(
                        image_width=self.image_width,
                        image_height=self.image_height,
                        edge_detection_method='enhanced_canny'
                    )
                    detector.enhanced_clahe_clip_limit = clahe_limit
                    detector.enhanced_bilateral_d = bilateral_d
                    detector.enhanced_bilateral_sigma_color = sigma_color
                    detector.enhanced_bilateral_sigma_space = sigma_space
                    
                    left_lane, right_lane = detector.detect_lanes(image)
                    
                    score = self._calculate_score(left_lane, right_lane, 0, 0)
                    
                    results.append({
                        'clahe_limit': clahe_limit,
                        'bilateral_d': bilateral_d,
                        'sigma_color': sigma_color,
                        'sigma_space': sigma_space,
                        'left_lane': left_lane is not None,
                        'right_lane': right_lane is not None,
                        'both_lanes': left_lane is not None and right_lane is not None,
                        'score': score
                    })
        
        best = max(results, key=lambda x: x['score'])
        return {
            'results': results,
            'best': best,
            'total_tests': len(results)
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
            
            if abs(slope) < 0.3:
                continue
            
            if slope < 0:
                left_points.extend([(x1, y1), (x2, y2)])
            else:
                right_points.extend([(x1, y1), (x2, y2)])
        
        left_lane = np.array(left_points) if left_points else None
        right_lane = np.array(right_points) if right_points else None
        
        return left_lane, right_lane
    
    def _calculate_score(self, left_lane: Optional[np.ndarray],
                        right_lane: Optional[np.ndarray],
                        edge_density: float,
                        line_count: int) -> float:
        """Calculate score for parameter combination"""
        score = 0.0
        
        # Both lanes detected is most important (weight: 50)
        if left_lane is not None and right_lane is not None:
            score += 50.0
        elif left_lane is not None or right_lane is not None:
            score += 20.0  # Partial credit
        
        # Edge density should be reasonable (weight: 20)
        # Too low: not enough edges, too high: too much noise
        if 0.01 <= edge_density <= 0.1:
            score += 20.0
        elif 0.005 <= edge_density < 0.01 or 0.1 < edge_density <= 0.15:
            score += 10.0
        
        # Line count should be reasonable (weight: 20)
        # Too few: might miss lanes, too many: noise
        if 4 <= line_count <= 15:
            score += 20.0
        elif 2 <= line_count < 4 or 15 < line_count <= 25:
            score += 10.0
        
        # More points is better (weight: 10)
        if left_lane is not None:
            score += min(10.0, len(left_lane) * 0.5)
        if right_lane is not None:
            score += min(10.0, len(right_lane) * 0.5)
        
        return score


def main():
    parser = argparse.ArgumentParser(description='Fine-tune Lane Detection Parameters')
    parser.add_argument('--test', type=str, default='all',
                       choices=['canny', 'hough', 'multiscale', 'enhanced', 'all'],
                       help='Which parameters to test')
    parser.add_argument('--difficulty', type=str, default='medium',
                       choices=['easy', 'medium', 'hard'],
                       help='Test image difficulty')
    parser.add_argument('--output', type=str, default='fine_tune_output',
                       help='Output directory')
    args = parser.parse_args()
    
    os.makedirs(args.output, exist_ok=True)
    
    print("=" * 80)
    print("FINE-TUNING LANE DETECTION PARAMETERS")
    print("=" * 80)
    print(f"Test: {args.test}")
    print(f"Difficulty: {args.difficulty}")
    print()
    
    tuner = ParameterTuner()
    
    # Create test image
    print("Creating test image...")
    test_image = tuner.create_test_image(difficulty=args.difficulty)
    cv2.imwrite(os.path.join(args.output, 'test_image.png'),
                cv2.cvtColor(test_image, cv2.COLOR_RGB2BGR))
    print("✅ Test image saved")
    print()
    
    results_summary = {}
    
    # Test Canny parameters
    if args.test in ['canny', 'all']:
        print("Testing Canny Edge Detection parameters...")
        print("This may take a while...")
        canny_results = tuner.test_canny_parameters(test_image)
        results_summary['canny'] = canny_results['best']
        print(f"✅ Best Canny: low={canny_results['best']['low']}, "
              f"high={canny_results['best']['high']}, "
              f"score={canny_results['best']['score']:.2f}")
        print(f"   Both lanes: {canny_results['best']['both_lanes']}")
        print()
    
    # Test Hough parameters
    if args.test in ['hough', 'all']:
        print("Testing Hough Line Transform parameters...")
        print("This may take a while...")
        hough_results = tuner.test_hough_parameters(test_image)
        results_summary['hough'] = hough_results['best']
        print(f"✅ Best Hough: threshold={hough_results['best']['threshold']}, "
              f"min_line_len={hough_results['best']['min_line_len']}, "
              f"max_line_gap={hough_results['best']['max_line_gap']}, "
              f"score={hough_results['best']['score']:.2f}")
        print(f"   Both lanes: {hough_results['best']['both_lanes']}")
        print()
    
    # Test Multi-scale parameters
    if args.test in ['multiscale', 'all']:
        print("Testing Multi-scale Canny parameters...")
        multiscale_results = tuner.test_multiscale_parameters(test_image)
        results_summary['multiscale'] = multiscale_results['best']
        print(f"✅ Best Multi-scale: scales={multiscale_results['best']['scales']}, "
              f"score={multiscale_results['best']['score']:.2f}")
        print(f"   Both lanes: {multiscale_results['best']['both_lanes']}")
        print()
    
    # Test Enhanced Canny parameters
    if args.test in ['enhanced', 'all']:
        print("Testing Enhanced Canny parameters...")
        print("This may take a while...")
        enhanced_results = tuner.test_enhanced_canny_parameters(test_image)
        results_summary['enhanced'] = enhanced_results['best']
        print(f"✅ Best Enhanced: clahe_limit={enhanced_results['best']['clahe_limit']}, "
              f"bilateral_d={enhanced_results['best']['bilateral_d']}, "
              f"sigma={enhanced_results['best']['sigma_color']}, "
              f"score={enhanced_results['best']['score']:.2f}")
        print(f"   Both lanes: {enhanced_results['best']['both_lanes']}")
        print()
    
    # Save results
    results_file = os.path.join(args.output, 'best_parameters.json')
    with open(results_file, 'w') as f:
        json.dump(results_summary, f, indent=2)
    print(f"✅ Results saved to {results_file}")
    
    # Generate recommendations
    print()
    print("=" * 80)
    print("RECOMMENDATIONS")
    print("=" * 80)
    
    if 'canny' in results_summary:
        best = results_summary['canny']
        print(f"\n📌 Canny Edge Detection:")
        print(f"   canny_low: {best['low']}")
        print(f"   canny_high: {best['high']}")
    
    if 'hough' in results_summary:
        best = results_summary['hough']
        print(f"\n📌 Hough Line Transform:")
        print(f"   hough_threshold: {best['threshold']}")
        print(f"   hough_min_line_len: {best['min_line_len']}")
        print(f"   hough_max_line_gap: {best['max_line_gap']}")
    
    if 'multiscale' in results_summary:
        best = results_summary['multiscale']
        print(f"\n📌 Multi-scale Canny:")
        print(f"   multiscale_scales: {best['scales']}")
    
    if 'enhanced' in results_summary:
        best = results_summary['enhanced']
        print(f"\n📌 Enhanced Canny:")
        print(f"   enhanced_clahe_clip_limit: {best['clahe_limit']}")
        print(f"   enhanced_bilateral_d: {best['bilateral_d']}")
        print(f"   enhanced_bilateral_sigma_color: {best['sigma_color']}")
        print(f"   enhanced_bilateral_sigma_space: {best['sigma_space']}")
    
    print()
    print("=" * 80)
    print("✅ Fine-tuning complete!")
    print("=" * 80)


if __name__ == '__main__':
    main()

