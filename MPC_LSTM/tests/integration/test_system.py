"""
Integration tests for autonomous driving system.
"""

import unittest
import numpy as np
from unittest.mock import Mock, MagicMock, patch

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from core.system import AutonomousDrivingSystem
from core.config import ConfigManager
from core.exceptions import ConfigurationError, ModelLoadError


class TestAutonomousDrivingSystem(unittest.TestCase):
    """Integration tests for AutonomousDrivingSystem."""
    
    def setUp(self):
        """Set up test fixtures."""
        # Create mock config manager
        self.config_manager = Mock(spec=ConfigManager)
        self.config_manager._config = {
            'perception': {
                'model': 'resnet18',
                'feature_dim': 512,
                'pretrained': False,
                'freeze_backbone': True
            },
            'temporal': {
                'model': 'lstm',
                'input_size': 512,
                'hidden_size': 256,
                'num_layers': 2,
                'sequence_length': 10,
                'dropout': 0.1
            },
            'mpc': {
                'horizon': 10,
                'dt': 0.05
            },
            'visualization': {
                'window_width': 1280,
                'window_height': 720
            },
            'logging': {
                'save_trajectory': False,
                'save_controls': False,
                'save_predictions': False
            }
        }
        self.config_manager.get_section = Mock(side_effect=lambda x: self.config_manager._config.get(x, {}))
    
    @patch('core.factories.PerceptionFactory.create')
    @patch('core.factories.TemporalFactory.create')
    @patch('core.factories.ControlFactory.create')
    @patch('core.factories.VisualizationFactory.create')
    def test_initialization(self, mock_viz, mock_control, mock_temporal, mock_perception):
        """Test system initialization."""
        # Create mock components
        mock_perception.return_value = Mock()
        mock_perception.return_value.get_feature_dim.return_value = 512
        mock_temporal.return_value = Mock()
        mock_temporal.return_value.get_sequence_length.return_value = 10
        mock_control.return_value = Mock()
        mock_control.return_value.N = 10
        mock_viz.return_value = Mock()
        
        system = AutonomousDrivingSystem(self.config_manager)
        
        self.assertIsNotNone(system.perception)
        self.assertIsNotNone(system.temporal)
        self.assertIsNotNone(system.control)
        self.assertIsNotNone(system.visualization)
        self.assertIsNotNone(system.sequence_buffer)
    
    @patch('core.factories.PerceptionFactory.create')
    @patch('core.factories.TemporalFactory.create')
    @patch('core.factories.ControlFactory.create')
    @patch('core.factories.VisualizationFactory.create')
    def test_dependency_injection(self, mock_viz, mock_control, mock_temporal, mock_perception):
        """Test dependency injection."""
        # Create mock components
        mock_perception_instance = Mock()
        mock_perception_instance.get_feature_dim.return_value = 512
        mock_temporal_instance = Mock()
        mock_temporal_instance.get_sequence_length.return_value = 10
        mock_control_instance = Mock()
        mock_control_instance.N = 10
        mock_viz_instance = Mock()
        
        system = AutonomousDrivingSystem(
            self.config_manager,
            perception=mock_perception_instance,
            temporal=mock_temporal_instance,
            control=mock_control_instance,
            visualization=mock_viz_instance
        )
        
        self.assertEqual(system.perception, mock_perception_instance)
        self.assertEqual(system.temporal, mock_temporal_instance)
        self.assertEqual(system.control, mock_control_instance)
        self.assertEqual(system.visualization, mock_viz_instance)
        
        # Factories should not be called
        mock_perception.assert_not_called()
        mock_temporal.assert_not_called()
        mock_control.assert_not_called()
        mock_viz.assert_not_called()


if __name__ == '__main__':
    unittest.main()

