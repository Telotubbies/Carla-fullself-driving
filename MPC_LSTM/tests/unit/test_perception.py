"""
Unit tests for perception module.
"""

import unittest
import numpy as np
import torch
from unittest.mock import Mock, patch, MagicMock

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from perception import ResNetEncoder
from core.exceptions import DataValidationError, ModelLoadError
from core.validators import ImageValidator


class TestResNetEncoder(unittest.TestCase):
    """Test cases for ResNetEncoder."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.encoder = ResNetEncoder(
            feature_dim=512,
            pretrained=False,  # Use False for faster tests
            freeze_backbone=True
        )
        self.test_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
    
    def test_encode_valid_image(self):
        """Test encoding valid image."""
        features = self.encoder.encode(self.test_image)
        self.assertEqual(features.shape[0], 512)
        self.assertFalse(np.any(np.isnan(features)))
        self.assertFalse(np.any(np.isinf(features)))
    
    def test_encode_invalid_image_none(self):
        """Test encoding None image raises error."""
        with self.assertRaises(DataValidationError):
            self.encoder.encode(None)
    
    def test_encode_invalid_image_shape(self):
        """Test encoding invalid image shape raises error."""
        invalid_image = np.random.randint(0, 255, (100,), dtype=np.uint8)
        with self.assertRaises(DataValidationError):
            self.encoder.encode(invalid_image)
    
    def test_get_feature_dim(self):
        """Test get_feature_dim method."""
        self.assertEqual(self.encoder.get_feature_dim(), 512)
    
    def test_encode_different_sizes(self):
        """Test encoding images of different sizes."""
        sizes = [(240, 320, 3), (480, 640, 3), (720, 1280, 3)]
        for h, w, c in sizes:
            image = np.random.randint(0, 255, (h, w, c), dtype=np.uint8)
            features = self.encoder.encode(image)
            self.assertEqual(features.shape[0], 512)


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
    
    def test_validate_wrong_channels(self):
        """Test validating wrong number of channels raises error."""
        image = np.random.randint(0, 255, (480, 640, 1), dtype=np.uint8)
        with self.assertRaises(DataValidationError):
            ImageValidator.validate(image)


if __name__ == '__main__':
    unittest.main()

