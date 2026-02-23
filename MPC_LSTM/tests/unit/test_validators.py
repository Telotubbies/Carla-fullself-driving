"""
Unit tests for validators.
"""

import unittest
import numpy as np

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from core.validators import (
    ImageValidator, StateValidator, FeatureValidator,
    PredictionValidator, ControlValidator
)
from core.exceptions import DataValidationError


class TestImageValidator(unittest.TestCase):
    """Test cases for ImageValidator."""
    
    def test_validate_valid_image(self):
        """Test validating valid image."""
        image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        self.assertTrue(ImageValidator.validate(image))
    
    def test_validate_none(self):
        """Test validating None raises error."""
        with self.assertRaises(DataValidationError):
            ImageValidator.validate(None)
    
    def test_validate_wrong_shape(self):
        """Test validating wrong shape raises error."""
        image = np.random.randint(0, 255, (100,), dtype=np.uint8)
        with self.assertRaises(DataValidationError):
            ImageValidator.validate(image)
    
    def test_validate_nan_values(self):
        """Test validating image with NaN values raises error."""
        image = np.full((480, 640, 3), np.nan, dtype=np.float32)
        with self.assertRaises(DataValidationError):
            ImageValidator.validate(image)


class TestFeatureValidator(unittest.TestCase):
    """Test cases for FeatureValidator."""
    
    def test_validate_valid_features(self):
        """Test validating valid feature vector."""
        features = np.random.randn(512).astype(np.float32)
        self.assertTrue(FeatureValidator.validate(features, 512))
    
    def test_validate_wrong_dimension(self):
        """Test validating wrong dimension raises error."""
        features = np.random.randn(256).astype(np.float32)
        with self.assertRaises(DataValidationError):
            FeatureValidator.validate(features, 512)
    
    def test_validate_nan_values(self):
        """Test validating features with NaN values raises error."""
        features = np.full(512, np.nan, dtype=np.float32)
        with self.assertRaises(DataValidationError):
            FeatureValidator.validate(features, 512)


class TestPredictionValidator(unittest.TestCase):
    """Test cases for PredictionValidator."""
    
    def test_validate_valid_prediction(self):
        """Test validating valid prediction."""
        prediction = np.array([1.0, 2.0, 3.0, 4.0])
        self.assertTrue(PredictionValidator.validate(prediction, 4))
    
    def test_validate_wrong_size(self):
        """Test validating wrong size raises error."""
        prediction = np.array([1.0, 2.0, 3.0])  # Size 3, expected 4
        with self.assertRaises(DataValidationError):
            PredictionValidator.validate(prediction, 4)
    
    def test_validate_nan_values(self):
        """Test validating prediction with NaN values raises error."""
        prediction = np.array([1.0, np.nan, 3.0, 4.0])
        with self.assertRaises(DataValidationError):
            PredictionValidator.validate(prediction, 4)


if __name__ == '__main__':
    unittest.main()

