"""
ResNet Feature Encoder for perception.

Extracts feature vectors from RGB images using pretrained ResNet18.
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

logger = logging.getLogger(__name__)


class ResNetEncoder(nn.Module):
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
                self.backbone.load_state_dict(state_dict)
                logger.info(f"✅ Loaded fine-tuned ResNet from {model_path}")
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
        
        Args:
            image: Input image as numpy array (H, W, 3) in RGB format, uint8
            
        Returns:
            Feature vector as numpy array (feature_dim,)
        """
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
                    from PIL import Image
                    if isinstance(image, np.ndarray):
                        image = Image.fromarray(image)
                    image_tensor = self.transform(image)
                
                # Add batch dimension
                image_tensor = image_tensor.unsqueeze(0).to(self.device)
            else:
                image_tensor = image.to(self.device)
            
            # Forward pass
            features = self.backbone(image_tensor)
            features = features.squeeze()  # Remove spatial dimensions
            
            # Project to desired dimension
            features = self.projection(features)
            
            # Convert to numpy
            if isinstance(features, torch.Tensor):
                features = features.cpu().numpy()
            
            return features
    
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

