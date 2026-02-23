"""
Unit tests for control module.
"""

import unittest
import numpy as np
from unittest.mock import Mock, MagicMock

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from control import MPCController
from core.exceptions import ControlError, DataValidationError
from core.validators import StateValidator, ControlValidator


class TestMPCController(unittest.TestCase):
    """Test cases for MPCController."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.config = {
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
        self.controller = MPCController(self.config)
        self.test_state = {
            'x': 0.0,
            'y': 0.0,
            'yaw': 0.0,
            'velocity': 5.0
        }
    
    def test_compute_control_valid_state(self):
        """Test computing control with valid state."""
        steering, throttle, brake = self.controller.compute_control(self.test_state)
        self.assertIsInstance(steering, (int, float))
        self.assertIsInstance(throttle, (int, float))
        self.assertIsInstance(brake, (int, float))
        self.assertTrue(-1.1 <= steering <= 1.1)
        self.assertTrue(-0.1 <= throttle <= 1.1)
        self.assertTrue(-0.1 <= brake <= 1.1)
    
    def test_compute_control_invalid_state(self):
        """Test computing control with invalid state raises error."""
        invalid_state = {'x': np.nan, 'y': 0.0, 'yaw': 0.0, 'velocity': 5.0}
        with self.assertRaises((ControlError, DataValidationError)):
            self.controller.compute_control(invalid_state)
    
    def test_set_reference_trajectory(self):
        """Test setting reference trajectory."""
        trajectory = np.random.randn(11, 4).astype(np.float32)
        self.controller.set_reference_trajectory(trajectory)
        self.assertIsNotNone(self.controller.reference_trajectory)
    
    def test_set_lane_info(self):
        """Test setting lane information."""
        lane_info = {'left_lane': None, 'right_lane': None, 'center_line': None}
        self.controller.set_lane_info(lane_info)
        self.assertEqual(self.controller.lane_info, lane_info)


class TestStateValidator(unittest.TestCase):
    """Test cases for StateValidator."""
    
    def test_validate_valid_state(self):
        """Test validating valid state."""
        state = {'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'velocity': 5.0}
        self.assertTrue(StateValidator.validate(state))
    
    def test_validate_missing_keys(self):
        """Test validating state with missing keys raises error."""
        state = {'x': 0.0, 'y': 0.0}  # Missing yaw and velocity
        with self.assertRaises(DataValidationError):
            StateValidator.validate(state)
    
    def test_validate_nan_values(self):
        """Test validating state with NaN values raises error."""
        state = {'x': np.nan, 'y': 0.0, 'yaw': 0.0, 'velocity': 5.0}
        with self.assertRaises(DataValidationError):
            StateValidator.validate(state)


class TestControlValidator(unittest.TestCase):
    """Test cases for ControlValidator."""
    
    def test_validate_valid_control(self):
        """Test validating valid control values."""
        self.assertTrue(ControlValidator.validate(0.0, 0.5, 0.0))
    
    def test_validate_out_of_range_steering(self):
        """Test validating out of range steering raises error."""
        with self.assertRaises(DataValidationError):
            ControlValidator.validate(2.0, 0.5, 0.0)  # Steering > 1.1
    
    def test_validate_nan_values(self):
        """Test validating control with NaN values raises error."""
        with self.assertRaises(DataValidationError):
            ControlValidator.validate(np.nan, 0.5, 0.0)


if __name__ == '__main__':
    unittest.main()

