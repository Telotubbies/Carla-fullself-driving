"""
Unit tests that force CPU usage to avoid ROCm issues.
"""

import os
# Force CPU for tests
os.environ['CUDA_VISIBLE_DEVICES'] = ''
os.environ['HSA_OVERRIDE_GFX_VERSION'] = ''

import unittest
import numpy as np
import torch

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from perception import ResNetEncoder
from temporal import LSTMPredictor, SequenceBuffer
from core.validators import ImageValidator, FeatureValidator


class TestWithCPU(unittest.TestCase):
    """Tests that force CPU usage."""
    
    def test_resnet_encoder_cpu(self):
        """Test ResNetEncoder on CPU."""
        encoder = ResNetEncoder(
            feature_dim=512,
            pretrained=False,
            freeze_backbone=True
        )
        self.assertEqual(encoder.get_feature_dim(), 512)
        
        test_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        features = encoder.encode(test_image)
        self.assertEqual(features.shape[0], 512)
        self.assertFalse(np.any(np.isnan(features)))
    
    def test_lstm_predictor_cpu(self):
        """Test LSTMPredictor on CPU."""
        predictor = LSTMPredictor(
            input_size=512,
            hidden_size=256,
            num_layers=2,
            sequence_length=10,
            dropout=0.1
        )
        self.assertEqual(predictor.get_sequence_length(), 10)
        
        buffer = SequenceBuffer(sequence_length=10, feature_dim=512)
        for i in range(10):
            buffer.add(np.random.randn(512).astype(np.float32))
        
        sequence = buffer.get_sequence()
        self.assertIsNotNone(sequence)
        
        prediction = predictor.predict(sequence)
        self.assertEqual(len(prediction), 4)
        self.assertFalse(np.any(np.isnan(prediction)))


if __name__ == '__main__':
    unittest.main()

