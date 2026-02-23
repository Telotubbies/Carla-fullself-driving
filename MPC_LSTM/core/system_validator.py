"""
Full System Integration Validator.

Implements PHASE 6 requirements:
- Validate data flow consistency
- Validate tensor shapes
- Validate timing
- Measure system performance
"""

import logging
import numpy as np
import time
from typing import Dict, Any, List, Optional
from pathlib import Path

logger = logging.getLogger(__name__)


class SystemValidator:
    """Validates full system integration."""
    
    def __init__(self):
        """Initialize system validator."""
        self.timing_log = []
        self.shape_log = []
        self.data_flow_log = []
        logger.info("✅ SystemValidator initialized")
    
    def validate_data_flow(
        self,
        image_shape: tuple,
        segmentation_shape: Optional[tuple],
        features_shape: tuple,
        sequence_shape: tuple,
        prediction_shape: tuple,
        control_shape: tuple
    ) -> Dict[str, bool]:
        """
        Validate data flow consistency.
        
        Args:
            image_shape: Input image shape
            segmentation_shape: Segmentation mask shape (if available)
            features_shape: Feature vector shape
            sequence_shape: Sequence shape
            prediction_shape: Prediction shape
            control_shape: Control output shape
        
        Returns:
            Dictionary with validation results
        """
        results = {
            'image_valid': len(image_shape) == 3 and image_shape[2] == 3,
            'segmentation_valid': segmentation_shape is None or len(segmentation_shape) == 2,
            'features_valid': len(features_shape) == 1 and features_shape[0] > 0,
            'sequence_valid': len(sequence_shape) == 2,
            'prediction_valid': len(prediction_shape) == 1 and prediction_shape[0] == 4,
            'control_valid': len(control_shape) == 3  # (steering, throttle, brake)
        }
        
        results['all_valid'] = all(results.values())
        
        # Log for debugging
        if not results['all_valid']:
            logger.warning(f"Data flow validation failed: {results}")
            self.data_flow_log.append({
                'timestamp': time.time(),
                'results': results,
                'shapes': {
                    'image': image_shape,
                    'segmentation': segmentation_shape,
                    'features': features_shape,
                    'sequence': sequence_shape,
                    'prediction': prediction_shape,
                    'control': control_shape
                }
            })
        
        return results
    
    def validate_tensor_shapes(
        self,
        expected_shapes: Dict[str, tuple],
        actual_shapes: Dict[str, tuple]
    ) -> Dict[str, bool]:
        """
        Validate tensor shapes match expected.
        
        Args:
            expected_shapes: Dictionary of expected shapes
            actual_shapes: Dictionary of actual shapes
        
        Returns:
            Dictionary with validation results
        """
        results = {}
        for key, expected_shape in expected_shapes.items():
            if key in actual_shapes:
                actual_shape = actual_shapes[key]
                results[key] = expected_shape == actual_shape
                if not results[key]:
                    logger.warning(
                        f"Shape mismatch for {key}: expected {expected_shape}, got {actual_shape}"
                    )
                    self.shape_log.append({
                        'key': key,
                        'expected': expected_shape,
                        'actual': actual_shape,
                        'timestamp': time.time()
                    })
            else:
                results[key] = False
                logger.warning(f"Missing shape for {key}")
        
        results['all_valid'] = all(results.values())
        return results
    
    def measure_timing(
        self,
        component: str,
        elapsed_time: float
    ) -> None:
        """
        Measure component timing.
        
        Args:
            component: Component name
            elapsed_time: Elapsed time in seconds
        """
        self.timing_log.append({
            'component': component,
            'time': elapsed_time,
            'timestamp': time.time()
        })
    
    def get_performance_metrics(self) -> Dict[str, Any]:
        """
        Get system performance metrics.
        
        Returns:
            Dictionary with performance metrics
        """
        if len(self.timing_log) == 0:
            return {}
        
        # Group by component
        component_times = {}
        for entry in self.timing_log:
            component = entry['component']
            if component not in component_times:
                component_times[component] = []
            component_times[component].append(entry['time'])
        
        # Calculate statistics per component
        metrics = {}
        for component, times in component_times.items():
            times_array = np.array(times)
            metrics[component] = {
                'mean_ms': float(np.mean(times_array) * 1000),
                'std_ms': float(np.std(times_array) * 1000),
                'min_ms': float(np.min(times_array) * 1000),
                'max_ms': float(np.max(times_array) * 1000),
                'fps': float(1.0 / np.mean(times_array)),
                'samples': len(times)
            }
        
        # Overall system FPS
        if 'total' in component_times:
            total_times = np.array(component_times['total'])
            metrics['system_fps'] = float(1.0 / np.mean(total_times))
        
        return metrics
    
    def print_performance_report(self) -> None:
        """Print performance report."""
        metrics = self.get_performance_metrics()
        
        logger.info("=" * 60)
        logger.info("SYSTEM PERFORMANCE METRICS")
        logger.info("=" * 60)
        
        for component, stats in metrics.items():
            if component == 'system_fps':
                logger.info(f"System FPS: {stats:.2f}")
            else:
                logger.info(f"{component.upper()}:")
                logger.info(f"  Mean: {stats['mean_ms']:.2f} ms")
                logger.info(f"  Std:  {stats['std_ms']:.2f} ms")
                logger.info(f"  FPS:  {stats['fps']:.2f}")
                logger.info(f"  Samples: {stats['samples']}")
        
        logger.info("=" * 60)

