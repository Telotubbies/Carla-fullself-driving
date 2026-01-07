"""
Vision Encoder Network for processing RGB + Depth images
Temporal reasoning with LSTM
Supports both CNN and ResNet architectures
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
from typing import Tuple, List, Optional

try:
    import torchvision.models as models
    TORCHVISION_AVAILABLE = True
except ImportError:
    TORCHVISION_AVAILABLE = False
    models = None

# YOLO removed - no longer used
    YOLO_AVAILABLE = False
    YOLOFeatureExtractor = None


class VisionEncoder(nn.Module):
    """
    CNN-based vision encoder for processing image sequences
    Processes RGB + Depth frames with temporal reasoning
    """
    
    def __init__(
        self,
        input_channels: int = 4,  # 3 RGB + 1 Depth
        sequence_length: int = 4,
        channels: List[int] = [32, 64, 128, 256],
        kernel_sizes: List[int] = [8, 4, 3, 3],
        strides: List[int] = [4, 2, 2, 1],
        activation: str = "relu",
        use_batch_norm: bool = True,
        image_size: Tuple[int, int] = (90, 160)  # (height, width)
    ):
        super().__init__()
        
        self.input_channels = input_channels
        self.sequence_length = sequence_length
        self.image_size = image_size
        
        # CNN layers for feature extraction
        conv_layers = []
        in_channels = input_channels
        
        for out_channels, kernel_size, stride in zip(channels, kernel_sizes, strides):
            conv_layers.append(
                nn.Conv2d(in_channels, out_channels, kernel_size, stride, padding=1)
            )
            if use_batch_norm:
                conv_layers.append(nn.BatchNorm2d(out_channels))
            
            if activation == "relu":
                conv_layers.append(nn.ReLU(inplace=True))
            elif activation == "elu":
                conv_layers.append(nn.ELU(inplace=True))
            else:
                conv_layers.append(nn.ReLU(inplace=True))
            
            in_channels = out_channels
        
        self.conv_encoder = nn.Sequential(*conv_layers)
        
        # Calculate feature map size after convolutions
        with torch.no_grad():
            dummy_input = torch.zeros(1, input_channels, image_size[0], image_size[1])
            dummy_output = self.conv_encoder(dummy_input)
            self.feature_size = dummy_output.numel() // dummy_output.size(0)
            self.feature_shape = dummy_output.shape[1:]  # (C, H, W)
        
        # Flatten layer
        self.flatten = nn.Flatten()
        
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        Forward pass
        
        Args:
            x: Input tensor of shape (batch, sequence_length, channels, height, width)
        
        Returns:
            Flattened features of shape (batch, sequence_length, feature_size)
        """
        batch_size, seq_len, channels, height, width = x.shape
        
        # Reshape to process all frames at once
        x = x.view(batch_size * seq_len, channels, height, width)
        
        # Extract features
        features = self.conv_encoder(x)
        
        # Flatten
        features = self.flatten(features)
        
        # Reshape back to sequence
        features = features.view(batch_size, seq_len, -1)
        
        return features


class ResNetEncoder(nn.Module):
    """
    ResNet-based vision encoder for processing image sequences
    Uses ResNet18/34 backbone with modified first layer for 4-channel input (RGB+Depth)
    Better feature extraction than simple CNN, especially for complex scenes
    """
    
    def __init__(
        self,
        input_channels: int = 4,  # 3 RGB + 1 Depth
        sequence_length: int = 4,
        resnet_type: str = "resnet18",  # "resnet18" or "resnet34"
        image_size: Tuple[int, int] = (90, 160),  # (height, width)
        pretrained: bool = False,  # Use ImageNet pre-trained weights
        freeze_early_layers: bool = False  # Freeze early layers for faster training
    ):
        super().__init__()
        
        self.input_channels = input_channels
        self.sequence_length = sequence_length
        self.image_size = image_size
        self.pretrained = pretrained
        self.freeze_early_layers = freeze_early_layers
        
        if not TORCHVISION_AVAILABLE:
            raise ImportError("torchvision is required for ResNet encoder")
        
        # Load ResNet backbone with optional pre-trained weights
        if resnet_type == "resnet18":
            if pretrained:
                # Load pre-trained ResNet18 from ImageNet
                resnet = models.resnet18(weights='IMAGENET1K_V1')
            else:
                resnet = models.resnet18(weights=None)
        elif resnet_type == "resnet34":
            if pretrained:
                resnet = models.resnet34(weights='IMAGENET1K_V1')
            else:
                resnet = models.resnet34(weights=None)
        else:
            raise ValueError(f"Unsupported ResNet type: {resnet_type}. Use 'resnet18' or 'resnet34'")
        
        # Replace first conv layer to accept 4 channels instead of 3
        original_conv1 = resnet.conv1
        # Initialize new conv1 layer
        # If pretrained, we'll copy weights from RGB channels and initialize depth channel
        new_conv1 = nn.Conv2d(
            input_channels,
            original_conv1.out_channels,
            kernel_size=original_conv1.kernel_size,
            stride=original_conv1.stride,
            padding=original_conv1.padding,
            bias=original_conv1.bias is not None
        )
        
        # If using pretrained, copy RGB weights and initialize depth channel
        if pretrained:
            with torch.no_grad():
                # Copy RGB weights (first 3 channels)
                new_conv1.weight[:, :3, :, :] = original_conv1.weight.clone()
                # Initialize depth channel (4th channel) as average of RGB channels
                new_conv1.weight[:, 3:4, :, :] = original_conv1.weight.mean(dim=1, keepdim=True)
                if original_conv1.bias is not None:
                    new_conv1.bias = original_conv1.bias.clone()
        
        resnet.conv1 = new_conv1
        
        # Remove final FC layer (we'll extract features before it)
        # ResNet structure: conv1 -> bn1 -> relu -> maxpool -> layer1 -> layer2 -> layer3 -> layer4 -> avgpool -> fc
        # We'll use everything up to avgpool
        self.resnet_backbone = nn.Sequential(
            resnet.conv1,
            resnet.bn1,
            resnet.relu,
            resnet.maxpool,
            resnet.layer1,
            resnet.layer2,
            resnet.layer3,
            resnet.layer4,
            resnet.avgpool
        )
        
        # Freeze early layers if requested (for faster training)
        if freeze_early_layers and pretrained:
            # Freeze conv1, bn1, maxpool, and layer1 (early feature extraction)
            for param in resnet.conv1.parameters():
                param.requires_grad = False
            for param in resnet.bn1.parameters():
                param.requires_grad = False
            for param in resnet.maxpool.parameters():
                param.requires_grad = False
            for param in resnet.layer1.parameters():
                param.requires_grad = False
            # layer2, layer3, layer4 will be fine-tuned
        
        # Calculate feature size
        with torch.no_grad():
            dummy_input = torch.zeros(1, input_channels, image_size[0], image_size[1])
            dummy_output = self.resnet_backbone(dummy_input)
            self.feature_size = dummy_output.numel() // dummy_output.size(0)
            self.feature_shape = dummy_output.shape[1:]  # (C, H, W) or flattened
        
        # Flatten layer
        self.flatten = nn.Flatten()
    
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        Forward pass
        
        Args:
            x: Input tensor of shape (batch, sequence_length, channels, height, width)
        
        Returns:
            Flattened features of shape (batch, sequence_length, feature_size)
        """
        batch_size, seq_len, channels, height, width = x.shape
        
        # Reshape to process all frames at once
        x = x.view(batch_size * seq_len, channels, height, width)
        
        # Extract features using ResNet
        features = self.resnet_backbone(x)
        
        # Flatten
        features = self.flatten(features)
        
        # Reshape back to sequence
        features = features.view(batch_size, seq_len, -1)
        
        return features


class TemporalEncoder(nn.Module):
    """
    LSTM-based temporal encoder for sequence processing
    """
    
    def __init__(
        self,
        input_size: int,
        hidden_size: int = 256,
        num_layers: int = 2,
        dropout: float = 0.1
    ):
        super().__init__()
        
        self.hidden_size = hidden_size
        self.num_layers = num_layers
        
        self.lstm = nn.LSTM(
            input_size,
            hidden_size,
            num_layers,
            batch_first=True,
            dropout=dropout if num_layers > 1 else 0.0
        )
    
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        Forward pass
        
        Args:
            x: Input tensor of shape (batch, sequence_length, feature_size)
        
        Returns:
            Last hidden state of shape (batch, hidden_size)
        """
        # LSTM forward
        lstm_out, (hidden, cell) = self.lstm(x)
        
        # Return last hidden state
        return hidden[-1]  # (batch, hidden_size)


class VisionTemporalEncoder(nn.Module):
    """
    Complete vision + temporal encoder
    Combines CNN feature extraction with LSTM temporal reasoning
    """
    
    def __init__(
        self,
        input_channels: int = 4,
        sequence_length: int = 4,
        vision_type: str = "CNN",  # "CNN" or "ResNet"
        vision_channels: List[int] = [32, 64, 128, 256],
        vision_kernels: List[int] = [8, 4, 3, 3],
        vision_strides: List[int] = [4, 2, 2, 1],
        resnet_type: str = "resnet18",  # "resnet18" or "resnet34"
        pretrained: bool = False,  # Use ImageNet pre-trained weights
        freeze_early_layers: bool = False,  # Freeze early layers for faster training
        temporal_hidden_size: int = 256,
        temporal_num_layers: int = 2,
        temporal_dropout: float = 0.1,
        image_size: Tuple[int, int] = (90, 160),
        activation: str = "relu",
        use_batch_norm: bool = True,
        use_yolo: bool = False,  # YOLO removed - no longer used (kept for backward compatibility)
        yolo_feature_dim: int = 0,  # YOLO removed - no longer used (kept for backward compatibility)
        device: str = 'cuda' if torch.cuda.is_available() else 'cpu'
    ):
        super().__init__()
        
        # YOLO removed - no longer used
        self.use_yolo = False
        self.device = device
        self.vision_type = vision_type
        
        # Vision encoder - choose between CNN and ResNet
        if vision_type == "ResNet":
            if not TORCHVISION_AVAILABLE:
                print("⚠️  torchvision not available. Falling back to CNN.")
                vision_type = "CNN"
            
            if vision_type == "ResNet":
                self.vision_encoder = ResNetEncoder(
                    input_channels=input_channels,
                    sequence_length=sequence_length,
                    resnet_type=resnet_type,
                    image_size=image_size,
                    pretrained=pretrained,
                    freeze_early_layers=freeze_early_layers
                )
                if pretrained:
                    print(f"✅ Using ResNet encoder: {resnet_type} (ImageNet pre-trained)")
                    if freeze_early_layers:
                        print(f"   ⚙️  Early layers frozen (conv1, bn1, maxpool, layer1)")
                else:
                    print(f"✅ Using ResNet encoder: {resnet_type}")
            else:
                # Fallback to CNN if ResNet failed
                self.vision_encoder = VisionEncoder(
                    input_channels=input_channels,
                    sequence_length=sequence_length,
                    channels=vision_channels,
                    kernel_sizes=vision_kernels,
                    strides=vision_strides,
                    activation=activation,
                    use_batch_norm=use_batch_norm,
                    image_size=image_size
                )
                print(f"✅ Using CNN encoder (ResNet fallback)")
        else:
            # vision_type == "CNN"
            self.vision_encoder = VisionEncoder(
                input_channels=input_channels,
                sequence_length=sequence_length,
                channels=vision_channels,
                kernel_sizes=vision_kernels,
                strides=vision_strides,
                activation=activation,
                use_batch_norm=use_batch_norm,
                image_size=image_size
            )
            print(f"✅ Using CNN encoder")
        
        # Calculate feature size
        feature_size = self.vision_encoder.feature_size
        
        # YOLO removed - no longer used
        self.yolo_extractor = None
        self.use_yolo = False
        
        # Temporal encoder
        self.temporal_encoder = TemporalEncoder(
            input_size=feature_size,
            hidden_size=temporal_hidden_size,
            num_layers=temporal_num_layers,
            dropout=temporal_dropout
        )
        
        self.output_size = temporal_hidden_size
    
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        """
        Forward pass
        
        Args:
            x: Input tensor of shape (batch, sequence_length, channels, height, width)
        
        Returns:
            Encoded features of shape (batch, hidden_size)
        """
        # Extract vision features
        vision_features = self.vision_encoder(x)  # (batch, seq_len, feature_size)
        
        # YOLO removed - no longer used
        
        # Process temporally
        temporal_features = self.temporal_encoder(vision_features)  # (batch, hidden_size)
        
        return temporal_features

