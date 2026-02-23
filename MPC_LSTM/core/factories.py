"""
Factory classes for creating system components.
"""

from typing import Dict, Any, Optional
from pathlib import Path
import torch
import numpy as np
import logging

from .interfaces import (
    IPerceptionModule,
    ITemporalModule,
    IControlModule,
    IVisualizationModule
)
from .exceptions import ModelLoadError, ConfigurationError
import sys
from pathlib import Path

# Setup logger
logger = logging.getLogger(__name__)

# Add project root to path for imports
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

from perception import ResNetEncoder
from temporal import LSTMPredictor
from control import MPCController
from visualization import VisualizationDisplay
from utils.device_utils import get_device


class PerceptionFactory:
    """Factory for creating perception modules."""
    
    @staticmethod
    def create(config: Dict[str, Any]) -> IPerceptionModule:
        """
        Create perception module based on configuration.
        
        Args:
            config: Perception configuration
        
        Returns:
            Perception module instance
        
        Raises:
            ConfigurationError: If configuration is invalid
            ModelLoadError: If model loading fails
        """
        model_type = config.get('model', 'resnet18')
        
        if model_type == 'resnet18':
            return PerceptionFactory._create_resnet(config)
        else:
            raise ConfigurationError(f"Unsupported perception model: {model_type}")
    
    @staticmethod
    def _create_resnet(config: Dict[str, Any]) -> ResNetEncoder:
        """Create ResNet encoder."""
        try:
            # Get model path if provided
            model_path = config.get('resnet_model_path')
            
            # ResNetEncoder loads model in __init__ if model_path is provided
            encoder = ResNetEncoder(
                feature_dim=config.get('feature_dim', 512),
                pretrained=config.get('pretrained', True),
                freeze_backbone=config.get('freeze_backbone', False),
                model_path=model_path if model_path and Path(model_path).exists() else None
            )
            
            return encoder
        except Exception as e:
            raise ModelLoadError(f"Failed to create ResNet encoder: {e}")


class TemporalFactory:
    """Factory for creating temporal prediction modules."""
    
    @staticmethod
    def create(config: Dict[str, Any]) -> ITemporalModule:
        """
        Create temporal module based on configuration.
        
        Args:
            config: Temporal configuration
        
        Returns:
            Temporal module instance
        
        Raises:
            ConfigurationError: If configuration is invalid
            ModelLoadError: If model loading fails
        """
        model_type = config.get('model', 'lstm')
        
        if model_type == 'lstm':
            return TemporalFactory._create_lstm(config)
        else:
            raise ConfigurationError(f"Unsupported temporal model: {model_type}")
    
    @staticmethod
    def _create_lstm(config: Dict[str, Any]):
        """Create LSTM predictor (supports both LSTMPredictor and AttentionLSTM)."""
        try:
            # Try to load trained model
            trained_model_path = config.get('trained_model_path')
            logger.info(f"Checking for trained model at: {trained_model_path}")
            if trained_model_path and Path(trained_model_path).exists():
                logger.info(f"✅ Found trained model, loading...")
                try:
                    from training.load_trained_model import load_trained_lstm
                    predictor, state_mean, state_std = load_trained_lstm(trained_model_path)
                    logger.info(f"✅ Loaded model type: {type(predictor).__name__}")
                    predictor.to(get_device())
                    # Store normalization params if needed
                    if hasattr(predictor, 'state_mean'):
                        predictor.state_mean = state_mean
                        predictor.state_std = state_std
                    return predictor
                except Exception as e:
                    logger.error(f"Failed to load trained model: {e}")
                    raise ModelLoadError(f"Failed to load LSTM model from {trained_model_path}: {e}")
            else:
                logger.info(f"⚠️  No trained model found at {trained_model_path}, creating new model")
            
            # Create new model
            predictor = LSTMPredictor(
                input_size=config.get('input_size', 512),
                hidden_size=config.get('hidden_size', 256),
                num_layers=config.get('num_layers', 2),
                sequence_length=config.get('sequence_length', 10),
                dropout=config.get('dropout', 0.1)
            )
            predictor.to(get_device())
            return predictor
        except Exception as e:
            raise ModelLoadError(f"Failed to create LSTM predictor: {e}")


class ControlFactory:
    """Factory for creating control modules."""
    
    @staticmethod
    def create(config: Dict[str, Any]) -> IControlModule:
        """
        Create control module based on configuration.
        
        Args:
            config: Full system configuration (needs mpc section)
        
        Returns:
            Control module instance
        
        Raises:
            ConfigurationError: If configuration is invalid
        """
        control_type = config.get('control', {}).get('type', 'mpc')
        
        if control_type == 'mpc':
            return ControlFactory._create_mpc(config)
        else:
            raise ConfigurationError(f"Unsupported control type: {control_type}")
    
    @staticmethod
    def _create_mpc(config: Dict[str, Any]) -> MPCController:
        """Create MPC controller."""
        try:
            return MPCController(config)
        except Exception as e:
            raise ConfigurationError(f"Failed to create MPC controller: {e}")


class VisualizationFactory:
    """Factory for creating visualization modules."""
    
    @staticmethod
    def create(config: Dict[str, Any]) -> IVisualizationModule:
        """
        Create visualization module based on configuration.
        
        Args:
            config: Visualization configuration
        
        Returns:
            Visualization module instance
        
        Raises:
            ConfigurationError: If configuration is invalid
        """
        viz_type = config.get('type', 'pygame')
        
        if viz_type == 'pygame':
            return VisualizationFactory._create_pygame(config)
        else:
            raise ConfigurationError(f"Unsupported visualization type: {viz_type}")
    
    @staticmethod
    def _create_pygame(config: Dict[str, Any]) -> VisualizationDisplay:
        """Create Pygame visualization."""
        try:
            return VisualizationDisplay(config)
        except Exception as e:
            raise ConfigurationError(f"Failed to create visualization: {e}")

