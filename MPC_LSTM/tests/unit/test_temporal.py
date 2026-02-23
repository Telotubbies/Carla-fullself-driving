"""
Unit tests for temporal module.
"""

import unittest
import numpy as np
import torch

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from temporal import LSTMPredictor, SequenceBuffer
from core.exceptions import DataValidationError, ModelLoadError
from core.validators import FeatureValidator, PredictionValidator


class TestLSTMPredictor(unittest.TestCase):
    """Test cases for LSTMPredictor."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.predictor = LSTMPredictor(
            input_size=512,
            hidden_size=256,
            num_layers=2,
            sequence_length=10,
            dropout=0.1
        )
        self.test_sequence = np.random.randn(10, 512).astype(np.float32)
    
    def test_predict_valid_sequence(self):
        """Test predicting with valid sequence."""
        prediction = self.predictor.predict(self.test_sequence)
        self.assertEqual(len(prediction), 4)
        self.assertFalse(np.any(np.isnan(prediction)))
        self.assertFalse(np.any(np.isinf(prediction)))
    
    def test_predict_invalid_sequence_none(self):
        """Test predicting with None sequence raises error."""
        with self.assertRaises(DataValidationError):
            self.predictor.predict(None)
    
    def test_predict_invalid_sequence_length(self):
        """Test predicting with wrong sequence length raises error."""
        invalid_sequence = np.random.randn(5, 512).astype(np.float32)
        with self.assertRaises(DataValidationError):
            self.predictor.predict(invalid_sequence)
    
    def test_predict_invalid_feature_dim(self):
        """Test predicting with wrong feature dimension raises error."""
        invalid_sequence = np.random.randn(10, 256).astype(np.float32)
        with self.assertRaises(DataValidationError):
            self.predictor.predict(invalid_sequence)
    
    def test_get_sequence_length(self):
        """Test get_sequence_length method."""
        self.assertEqual(self.predictor.get_sequence_length(), 10)


class TestSequenceBuffer(unittest.TestCase):
    """Test cases for SequenceBuffer."""
    
    def setUp(self):
        """Set up test fixtures."""
        self.buffer = SequenceBuffer(sequence_length=10, feature_dim=512)
        self.test_feature = np.random.randn(512).astype(np.float32)
    
    def test_add_feature(self):
        """Test adding feature to buffer."""
        self.buffer.add(self.test_feature)
        self.assertEqual(len(self.buffer.buffer), 1)
    
    def test_add_wrong_dimension(self):
        """Test adding feature with wrong dimension raises error."""
        wrong_feature = np.random.randn(256).astype(np.float32)
        with self.assertRaises(ValueError):
            self.buffer.add(wrong_feature)
    
    def test_get_sequence_not_ready(self):
        """Test getting sequence when buffer not ready."""
        for i in range(5):
            self.buffer.add(self.test_feature)
        self.assertIsNone(self.buffer.get_sequence())
    
    def test_get_sequence_ready(self):
        """Test getting sequence when buffer ready."""
        for i in range(10):
            self.buffer.add(self.test_feature)
        sequence = self.buffer.get_sequence()
        self.assertIsNotNone(sequence)
        self.assertEqual(sequence.shape, (10, 512))
    
    def test_is_ready(self):
        """Test is_ready method."""
        self.assertFalse(self.buffer.is_ready())
        for i in range(10):
            self.buffer.add(self.test_feature)
        self.assertTrue(self.buffer.is_ready())
    
    def test_reset(self):
        """Test reset method."""
        for i in range(10):
            self.buffer.add(self.test_feature)
        self.buffer.reset()
        self.assertEqual(len(self.buffer.buffer), 0)


if __name__ == '__main__':
    unittest.main()

