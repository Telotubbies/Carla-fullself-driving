import torch
import torch.nn as nn
import torch.nn.functional as F
import logging
from typing import Tuple, List, Optional
try:
    import torchvision.models as models
    TORCHVISION_AVAILABLE = True
except ImportError:
    TORCHVISION_AVAILABLE = False
    models = None
    YOLO_AVAILABLE = False
    YOLOFeatureExtractor = None
class VisionEncoder(nn.Module):
    
    def __init__(
        self,
        input_channels: int = 4,
        sequence_length: int = 4,
        channels: List[int] = [32, 64, 128, 256],
        kernel_sizes: List[int] = [8, 4, 3, 3],
        strides: List[int] = [4, 2, 2, 1],
        activation: str = "relu",
        use_batch_norm: bool = True,
        image_size: Tuple[int, int] = (90, 160)
    ):
        super().__init__()
        self.input_channels = input_channels
        self.sequence_length = sequence_length
        self.image_size = image_size
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
        with torch.no_grad():
            dummy_input = torch.zeros(1, input_channels, image_size[0], image_size[1])
            dummy_output = self.conv_encoder(dummy_input)
            self.feature_size = dummy_output.numel() // dummy_output.size(0)
            self.feature_shape = dummy_output.shape[1:]
        self.flatten = nn.Flatten()
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        
        batch_size, seq_len, channels, height, width = x.shape
        x = x.view(batch_size * seq_len, channels, height, width)
        input_device = torch.device('cpu')
        is_cuda = False
        try:
            device_attr = getattr(x, 'device', None)
            if device_attr is not None:
                try:
                    device_type = getattr(device_attr, 'type', 'cpu')
                    if device_type == 'cuda':
                        input_device = device_attr
                        is_cuda = True
                except (RuntimeError, AttributeError):
                    pass
        except (RuntimeError, AttributeError, Exception) as e:
            logging.debug(f"Device access failed, using CPU: {e}")
            input_device = torch.device('cpu')
            is_cuda = False
        if is_cuda:
            try:
                x_cpu = x.cpu()
            except (RuntimeError, AttributeError) as e:
                logging.debug(f"Failed to move tensor to CPU: {e}, assuming already on CPU")
                x_cpu = x
            features = self.conv_encoder(x_cpu)
            try:
                features = features.to(input_device)
            except (RuntimeError, AttributeError) as e:
                logging.debug(f"Failed to move features back to GPU, keeping on CPU: {e}")
                pass
        else:
            features = self.conv_encoder(x)
        features = self.flatten(features)
        features = features.view(batch_size, seq_len, -1)
        return features
class ResNetEncoder(nn.Module):
    
    def __init__(
        self,
        input_channels: int = 4,
        sequence_length: int = 4,
        resnet_type: str = "resnet18",
        image_size: Tuple[int, int] = (90, 160),
        pretrained: bool = False,
        freeze_early_layers: bool = False
    ):
        super().__init__()
        self.input_channels = input_channels
        self.sequence_length = sequence_length
        self.image_size = image_size
        self.pretrained = pretrained
        self.freeze_early_layers = freeze_early_layers
        if not TORCHVISION_AVAILABLE:
            raise ImportError("torchvision is required for ResNet encoder")
        if resnet_type == "resnet18":
            if pretrained:
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
        original_conv1 = resnet.conv1
        new_conv1 = nn.Conv2d(
            input_channels,
            original_conv1.out_channels,
            kernel_size=original_conv1.kernel_size,
            stride=original_conv1.stride,
            padding=original_conv1.padding,
            bias=original_conv1.bias is not None
        )
        if pretrained:
            with torch.no_grad():
                new_conv1.weight[:, :3, :, :] = original_conv1.weight.clone()
                new_conv1.weight[:, 3:4, :, :] = original_conv1.weight.mean(dim=1, keepdim=True)
                if original_conv1.bias is not None:
                    new_conv1.bias = original_conv1.bias.clone()
        resnet.conv1 = new_conv1
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
        if freeze_early_layers and pretrained:
            for param in resnet.conv1.parameters():
                param.requires_grad = False
            for param in resnet.bn1.parameters():
                param.requires_grad = False
            for param in resnet.maxpool.parameters():
                param.requires_grad = False
            for param in resnet.layer1.parameters():
                param.requires_grad = False
        with torch.no_grad():
            dummy_input = torch.zeros(1, input_channels, image_size[0], image_size[1])
            dummy_output = self.resnet_backbone(dummy_input)
            self.feature_size = dummy_output.numel() // dummy_output.size(0)
            self.feature_shape = dummy_output.shape[1:]
        self.flatten = nn.Flatten()
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        
        batch_size, seq_len, channels, height, width = x.shape
        x = x.view(batch_size * seq_len, channels, height, width)
        try:
            resnet_device = next(self.resnet_backbone.parameters()).device
            x = x.to(resnet_device)
        except (StopIteration, RuntimeError, AttributeError):
            try:
                model_device = next(self.parameters()).device
                x = x.to(model_device)
                resnet_device = model_device
            except (StopIteration, RuntimeError, AttributeError):
                if torch.cuda.is_available():
                    try:
                        x = x.to('cuda:0')
                        resnet_device = torch.device('cuda:0')
                    except:
                        logging.error("Could not determine device for ResNet, keeping on CPU (may cause error)")
                        resnet_device = torch.device('cpu')
                else:
                    resnet_device = torch.device('cpu')
        features = self.resnet_backbone(x)
        features = self.flatten(features)
        features = features.view(batch_size, seq_len, -1)
        return features
class TemporalEncoder(nn.Module):
    
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
        
        lstm_out, (hidden, cell) = self.lstm(x)
        return hidden[-1]
class VisionTemporalEncoder(nn.Module):
    
    def __init__(
        self,
        input_channels: int = 4,
        sequence_length: int = 4,
        vision_type: str = "CNN",
        vision_channels: List[int] = [32, 64, 128, 256],
        vision_kernels: List[int] = [8, 4, 3, 3],
        vision_strides: List[int] = [4, 2, 2, 1],
        resnet_type: str = "resnet18",
        pretrained: bool = False,
        freeze_early_layers: bool = False,
        temporal_hidden_size: int = 256,
        temporal_num_layers: int = 2,
        temporal_dropout: float = 0.1,
        image_size: Tuple[int, int] = (90, 160),
        activation: str = "relu",
        use_batch_norm: bool = True,
        use_yolo: bool = False,
        yolo_feature_dim: int = 0,
        device: str = 'cuda' if torch.cuda.is_available() else 'cpu'
    ):
        super().__init__()
        self.use_yolo = False
        self.device = device
        self.vision_type = vision_type
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
        feature_size = self.vision_encoder.feature_size
        self.yolo_extractor = None
        self.use_yolo = False
        self.temporal_encoder = TemporalEncoder(
            input_size=feature_size,
            hidden_size=temporal_hidden_size,
            num_layers=temporal_num_layers,
            dropout=temporal_dropout
        )
        self.output_size = temporal_hidden_size
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        
        if hasattr(self.vision_encoder, 'resnet_backbone'):
            try:
                vision_device = next(self.vision_encoder.resnet_backbone.parameters()).device
                if x.device != vision_device:
                    x = x.to(vision_device)
            except (StopIteration, RuntimeError, AttributeError):
                try:
                    vision_device = next(self.vision_encoder.parameters()).device
                    if x.device != vision_device:
                        x = x.to(vision_device)
                except (StopIteration, RuntimeError, AttributeError):
                    pass
        vision_features = self.vision_encoder(x)
        temporal_features = self.temporal_encoder(vision_features)
        return temporal_features