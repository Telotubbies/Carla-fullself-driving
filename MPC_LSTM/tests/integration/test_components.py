"""
Test script for individual components.

Tests each module independently before running full system.
"""

import sys
import numpy as np
import torch
import logging
from pathlib import Path

# Setup logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Add project root to path
project_root = Path(__file__).parent
sys.path.insert(0, str(project_root))

def test_resnet_encoder():
    """Test ResNet encoder."""
    logger.info("Testing ResNet Encoder...")
    try:
        from perception import ResNetEncoder
        
        encoder = ResNetEncoder(feature_dim=512, pretrained=True, freeze_backbone=True)
        
        # Create dummy image (640x480 RGB)
        dummy_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        
        # Encode
        features = encoder.encode(dummy_image)
        
        assert features.shape == (512,), f"Expected shape (512,), got {features.shape}"
        logger.info(f"✅ ResNet Encoder test passed. Feature shape: {features.shape}")
        return True
        
    except Exception as e:
        logger.error(f"❌ ResNet Encoder test failed: {e}")
        return False

def test_lstm_predictor():
    """Test LSTM predictor."""
    logger.info("Testing LSTM Predictor...")
    try:
        from temporal import LSTMPredictor, SequenceBuffer
        
        predictor = LSTMPredictor(
            input_size=512,
            hidden_size=256,
            num_layers=2,
            sequence_length=10
        )
        
        buffer = SequenceBuffer(sequence_length=10, feature_dim=512)
        
        # Add features to buffer
        for _ in range(10):
            feature = np.random.randn(512)
            buffer.add(feature)
        
        # Get sequence
        sequence = buffer.get_sequence()
        assert sequence.shape == (10, 512), f"Expected shape (10, 512), got {sequence.shape}"
        
        # Predict
        predicted_state = predictor.predict(sequence)
        assert predicted_state.shape == (4,), f"Expected shape (4,), got {predicted_state.shape}"
        
        logger.info(f"✅ LSTM Predictor test passed. Predicted state shape: {predicted_state.shape}")
        return True
        
    except Exception as e:
        logger.error(f"❌ LSTM Predictor test failed: {e}")
        return False

def test_mpc_controller():
    """Test MPC controller."""
    logger.info("Testing MPC Controller...")
    try:
        from control import MPCController
        
        config = {
            'mpc': {
                'horizon': 10,
                'dt': 0.05,
                'vehicle': {
                    'wheelbase': 2.875,
                    'max_steer': 0.5,
                    'max_accel': 3.0,
                    'max_decel': -3.0
                },
                'weights': {
                    'lateral_error': 10.0,
                    'heading_error': 5.0,
                    'velocity_error': 2.0,
                    'steer_rate': 1.0,
                    'accel_rate': 1.0
                },
                'constraints': {
                    'max_speed': 30.0,
                    'min_speed': 0.0
                }
            }
        }
        
        controller = MPCController(config)
        
        # Test with dummy state
        current_state = {
            'x': 0.0,
            'y': 0.0,
            'yaw': 0.0,
            'velocity': 5.0
        }
        
        steering, throttle, brake = controller.compute_control(current_state)
        
        assert -1.0 <= steering <= 1.0, f"Steering out of range: {steering}"
        assert 0.0 <= throttle <= 1.0, f"Throttle out of range: {throttle}"
        assert 0.0 <= brake <= 1.0, f"Brake out of range: {brake}"
        
        logger.info(f"✅ MPC Controller test passed. Control: steer={steering:.3f}, throttle={throttle:.3f}, brake={brake:.3f}")
        return True
        
    except Exception as e:
        logger.error(f"❌ MPC Controller test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_visualization():
    """Test visualization display."""
    logger.info("Testing Visualization Display...")
    try:
        from visualization import VisualizationDisplay
        
        config = {
            'window_width': 1280,
            'window_height': 720,
            'update_rate': 20,
            'show_camera': True,
            'show_trajectory': True,
            'show_graphs': True,
            'graph_history': 1000
        }
        
        display = VisualizationDisplay(config)
        
        # Test with dummy data
        dummy_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        vehicle_state = {
            'x': 0.0,
            'y': 0.0,
            'yaw': 0.0,
            'velocity': 5.0,
            'steering': 0.0,
            'throttle': 0.5,
            'brake': 0.0
        }
        
        # Update once (don't keep window open)
        result = display.update(dummy_image, vehicle_state)
        
        display.close()
        
        logger.info("✅ Visualization Display test passed")
        return True
        
    except Exception as e:
        logger.error(f"❌ Visualization Display test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    """Run all tests."""
    logger.info("=" * 50)
    logger.info("Component Tests")
    logger.info("=" * 50)
    
    results = []
    
    results.append(("ResNet Encoder", test_resnet_encoder()))
    results.append(("LSTM Predictor", test_lstm_predictor()))
    results.append(("MPC Controller", test_mpc_controller()))
    results.append(("Visualization", test_visualization()))
    
    logger.info("=" * 50)
    logger.info("Test Results")
    logger.info("=" * 50)
    
    for name, passed in results:
        status = "✅ PASS" if passed else "❌ FAIL"
        logger.info(f"{name}: {status}")
    
    all_passed = all(result[1] for result in results)
    
    if all_passed:
        logger.info("✅ All tests passed!")
        return 0
    else:
        logger.error("❌ Some tests failed!")
        return 1

if __name__ == '__main__':
    sys.exit(main())

