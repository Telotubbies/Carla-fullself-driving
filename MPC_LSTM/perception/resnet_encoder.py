"""
ResNet Feature Encoder for perception.

Extracts feature vectors from RGB images using pretrained ResNet18.
Implements IPerceptionModule interface.
"""

import torch
import torch.nn as nn
import torchvision.models as models
import torchvision.transforms as transforms
import numpy as np
import logging
from typing import Optional, Union, Tuple
import cv2
import sys
from pathlib import Path

# Add utils to path
sys.path.insert(0, str(Path(__file__).parent.parent))
from utils.device_utils import get_device
from core.interfaces import IPerceptionModule
from core.exceptions import ModelLoadError, DataValidationError
from core.validators import ImageValidator

logger = logging.getLogger(__name__)


class ResNetEncoder(nn.Module, IPerceptionModule):
    """ResNet18-based feature encoder for autonomous driving."""
    
    def __init__(
        self,
        feature_dim: int = 512,
        pretrained: bool = True,
        freeze_backbone: bool = True,
        model_path: Optional[str] = None
    ):
        """
        Initialize ResNet encoder.
        
        Args:
            feature_dim: Output feature dimension
            pretrained: Use pretrained weights
            freeze_backbone: Freeze ResNet backbone weights
            model_path: Path to fine-tuned model weights (optional)
        """
        super(ResNetEncoder, self).__init__()
        
        # Load pretrained ResNet18
        self.backbone = models.resnet18(pretrained=pretrained)
        
        # Remove final fully connected layer
        # ResNet18 output features: 512
        self.backbone = nn.Sequential(*list(self.backbone.children())[:-1])
        
        # Load fine-tuned weights if provided
        if model_path and Path(model_path).exists():
            try:
                state_dict = torch.load(model_path, map_location='cpu')
                # Handle different state_dict formats
                if isinstance(state_dict, dict):
                    # Check if it's a checkpoint with 'model_state_dict' or 'state_dict' key
                    if 'model_state_dict' in state_dict:
                        state_dict = state_dict['model_state_dict']
                    elif 'state_dict' in state_dict:
                        state_dict = state_dict['state_dict']
                    # Check if keys have 'backbone.' prefix
                    if any(k.startswith('backbone.') for k in state_dict.keys()):
                        # Remove 'backbone.' prefix
                        state_dict = {k.replace('backbone.', ''): v for k, v in state_dict.items()}
                
                # Try to load state dict (strict=False to ignore missing keys)
                try:
                    self.backbone.load_state_dict(state_dict, strict=False)
                    logger.info(f"✅ Loaded fine-tuned ResNet from {model_path}")
                except Exception as load_e:
                    logger.warning(f"Partial load failed: {load_e}, trying to match keys...")
                    # Try to match compatible keys
                    model_dict = self.backbone.state_dict()
                    compatible_dict = {k: v for k, v in state_dict.items() if k in model_dict and model_dict[k].shape == v.shape}
                    if len(compatible_dict) > 0:
                        model_dict.update(compatible_dict)
                        self.backbone.load_state_dict(model_dict)
                        logger.info(f"✅ Loaded {len(compatible_dict)}/{len(state_dict)} compatible weights")
                    else:
                        logger.warning(f"No compatible weights found, using pretrained")
            except Exception as e:
                logger.warning(f"Failed to load fine-tuned model: {e}, using pretrained")
        
        # Freeze backbone if requested
        if freeze_backbone:
            for param in self.backbone.parameters():
                param.requires_grad = False
            logger.info("✅ ResNet backbone frozen")
        
        # Optional projection layer to ensure exact feature_dim
        if feature_dim != 512:
            self.projection = nn.Linear(512, feature_dim)
        else:
            self.projection = nn.Identity()
        
        # Image preprocessing
        self.transform = transforms.Compose([
            transforms.ToPILImage(),
            transforms.Resize((224, 224)),
            transforms.ToTensor(),
            transforms.Normalize(
                mean=[0.485, 0.456, 0.406],
                std=[0.229, 0.224, 0.225]
            )
        ])
        
        self.feature_dim = feature_dim
        self.device = get_device()  # Supports ROCm/CUDA/CPU
        self.to(self.device)
        self.eval()
        
        logger.info(f"✅ ResNetEncoder initialized (feature_dim={feature_dim}, device={self.device})")
    
    def encode(self, image: np.ndarray) -> np.ndarray:
        """
        Encode image to feature vector.
        
        Implements IPerceptionModule.encode().
        
        Args:
            image: Input image as numpy array (H, W, 3) in RGB format, uint8
        
        Returns:
            Feature vector as numpy array (feature_dim,)
        
        Raises:
            DataValidationError: If image is invalid
            ModelLoadError: If model fails to process image
        """
        # Validate input image
        try:
            ImageValidator.validate(image)
        except DataValidationError as e:
            logger.error(f"Image validation failed: {e}")
            raise
        
        with torch.no_grad():
            # Preprocess image
            if isinstance(image, np.ndarray):
                # Ensure image is in correct format
                if image.dtype != np.uint8:
                    image = (image * 255).astype(np.uint8)
                
                # Convert to RGB if needed
                if len(image.shape) == 3 and image.shape[2] == 3:
                    # Already RGB
                    pass
                else:
                    logger.warning(f"Unexpected image shape: {image.shape}")
                
                # Apply transforms
                try:
                    image_tensor = self.transform(image)
                except Exception as e:
                    logger.error(f"Error in image transform: {e}")
                    # Fallback: convert to PIL manually
                    try:
                        from PIL import Image
                        if isinstance(image, np.ndarray):
                            image = Image.fromarray(image)
                        image_tensor = self.transform(image)
                    except Exception as e2:
                        logger.error(f"Fallback transform also failed: {e2}")
                        raise ModelLoadError(f"Failed to transform image: {e2}") from e2
                
                # Add batch dimension
                image_tensor = image_tensor.unsqueeze(0).to(self.device)
            else:
                image_tensor = image.to(self.device)
            
            # Forward pass
            try:
                features = self.backbone(image_tensor)
                features = features.squeeze()  # Remove spatial dimensions
                
                # Project to desired dimension
                features = self.projection(features)
                
                # Convert to numpy
                if isinstance(features, torch.Tensor):
                    features = features.cpu().numpy()
                
                # Validate output
                if features.shape[0] != self.feature_dim:
                    raise ModelLoadError(f"Feature dimension mismatch: expected {self.feature_dim}, got {features.shape[0]}")
                
                return features
            except Exception as e:
                logger.error(f"Forward pass failed: {e}", exc_info=True)
                raise ModelLoadError(f"Failed to encode image: {e}") from e
    
    def get_feature_dim(self) -> int:
        """
        Get feature dimension.
        
        Implements IPerceptionModule.get_feature_dim().
        
        Returns:
            Feature dimension
        """
        return self.feature_dim
    
    def forward(self, image: torch.Tensor) -> torch.Tensor:
        """
        Forward pass (for training).
        
        Args:
            image: Input image tensor (B, 3, H, W)
            
        Returns:
            Feature tensor (B, feature_dim)
        """
        features = self.backbone(image)
        features = features.view(features.size(0), -1)  # Flatten
        features = self.projection(features)
        return features

