"""
Custom Policy Network for Stable-Baselines3
Vision-based policy with continuous actions
"""

import torch
import torch.nn as nn
from typing import Dict, List, Tuple, Type
import gymnasium as gym
from stable_baselines3.common.policies import ActorCriticPolicy
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor

try:
    from .vision_encoder import VisionTemporalEncoder
except ImportError:
    # Fallback for direct import
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).parent.parent))
    from models.vision_encoder import VisionTemporalEncoder


class VisionFeaturesExtractor(BaseFeaturesExtractor):
    """
    Custom features extractor for vision-based observations
    Wraps VisionTemporalEncoder for use with Stable-Baselines3
    Supports both Box (vision-only) and Dict (vision + GPS + goal) observation spaces
    """
    
    def __init__(
        self,
        observation_space: gym.Space,
        features_dim: int = 256,
        config: Dict = None
    ):
        super().__init__(observation_space, features_dim)
        
        if config is None:
            config = {}
        
        network_config = config.get('network', {})
        vision_config = network_config.get('vision_encoder', {})
        temporal_config = network_config.get('temporal', {})
        obs_config = config.get('observations', {})
        
        # Check if observation space is Dict (GPS/goal/waypoint/velocity enabled) or Box (vision-only)
        self.use_gps = obs_config.get('use_gps', False)
        self.use_goal = obs_config.get('use_goal', False)
        self.use_waypoint = obs_config.get('use_waypoint', True)  # NEW: Default True
        self.use_velocity = obs_config.get('use_velocity', True)  # NEW: Default True
        self.is_dict_obs = isinstance(observation_space, gym.spaces.Dict)
        
        if self.is_dict_obs:
            # Dict observation: extract vision shape
            vision_space = observation_space['vision']
            obs_shape = vision_space.shape  # (seq_len, H, W, C)
        else:
            # Box observation: vision-only
            obs_shape = observation_space.shape  # (seq_len, H, W, C)
        
        sequence_length, height, width, channels = obs_shape
        
        # YOLO removed - no longer used
        use_yolo = False
        yolo_feature_dim = 0
        
        # Device
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        
        # Create vision-temporal encoder
        self.encoder = VisionTemporalEncoder(
            input_channels=channels,
            sequence_length=sequence_length,
            vision_type=vision_config.get('type', 'CNN'),  # "CNN" or "ResNet"
            vision_channels=vision_config.get('channels', [32, 64, 128, 256]),
            vision_kernels=vision_config.get('kernel_sizes', [8, 4, 3, 3]),
            vision_strides=vision_config.get('strides', [4, 2, 2, 1]),
            resnet_type=vision_config.get('resnet_type', 'resnet18'),  # "resnet18" or "resnet34"
            pretrained=vision_config.get('pretrained', False),  # Transfer Learning: Use ImageNet pre-trained
            freeze_early_layers=vision_config.get('freeze_early_layers', False),  # Freeze early layers
            temporal_hidden_size=temporal_config.get('hidden_size', 256),
            temporal_num_layers=temporal_config.get('num_layers', 2),
            temporal_dropout=temporal_config.get('dropout', 0.1),
            image_size=(height, width),
            activation=vision_config.get('activation', 'relu'),
            use_batch_norm=vision_config.get('use_batch_norm', True),
            use_yolo=False,  # YOLO removed - no longer used
            yolo_feature_dim=0,  # YOLO removed - no longer used
            device=device
        )
        
        # GPS, Goal, Waypoint, and Velocity encoders (if enabled)
        if self.is_dict_obs:
            gps_dim = 0
            goal_dim = 0
            waypoint_dim = 0
            velocity_dim = 0
            
            if self.use_gps:
                # GPS encoder: 3 (x,y,z) -> 16
                self.gps_encoder = nn.Sequential(
                    nn.Linear(3, 16),
                    nn.ReLU(),
                    nn.Linear(16, 16)
                )
                gps_dim = 16
            else:
                self.gps_encoder = None
            
            if self.use_goal:
                # Goal encoder: 3 (x,y,z) + 1 (distance) -> 16
                self.goal_encoder = nn.Sequential(
                    nn.Linear(4, 16),  # 3 (goal) + 1 (distance)
                    nn.ReLU(),
                    nn.Linear(16, 16)
                )
                goal_dim = 16
            else:
                self.goal_encoder = None
            
            if self.use_waypoint:
                # Waypoint encoder: 8 features -> 16
                self.waypoint_encoder = nn.Sequential(
                    nn.Linear(8, 16),
                    nn.ReLU(),
                    nn.Linear(16, 16)
                )
                waypoint_dim = 16
            else:
                self.waypoint_encoder = None
            
            if self.use_velocity:
                # Velocity encoder: 5 features -> 16
                self.velocity_encoder = nn.Sequential(
                    nn.Linear(5, 16),
                    nn.ReLU(),
                    nn.Linear(16, 16)
                )
                velocity_dim = 16
            else:
                self.velocity_encoder = None
        else:
            self.gps_encoder = None
            self.goal_encoder = None
            self.waypoint_encoder = None
            self.velocity_encoder = None
            gps_dim = 0
            goal_dim = 0
            waypoint_dim = 0
            velocity_dim = 0
        
        # Projection layer to match features_dim
        encoder_output_size = self.encoder.output_size + gps_dim + goal_dim + waypoint_dim + velocity_dim
        if encoder_output_size != features_dim:
            self.projection = nn.Linear(encoder_output_size, features_dim)
        else:
            self.projection = nn.Identity()
    
    def forward(self, observations) -> torch.Tensor:
        """
        Extract features from observations
        
        Args:
            observations: Tensor of shape (batch, seq_len, H, W, C) for Box
                         or Dict with 'vision', 'gps', 'goal', 'distance_to_goal', 'waypoint', 'velocity' for Dict
        
        Returns:
            Features of shape (batch, features_dim)
        """
        if self.is_dict_obs:
            # Dict observation: extract vision, GPS, goal
            vision_obs = observations['vision']
            vision_obs = vision_obs.permute(0, 1, 4, 2, 3)  # (batch, seq_len, C, H, W)
            
            # Encode vision
            vision_features = self.encoder(vision_obs)  # (batch, vision_dim)
            
            # Encode GPS
            gps_features = None
            if self.use_gps and 'gps' in observations:
                gps_data = observations['gps']  # (batch, 3)
                gps_features = self.gps_encoder(gps_data)  # (batch, 16)
            
            # Encode Goal
            goal_features = None
            if self.use_goal and 'goal' in observations and 'distance_to_goal' in observations:
                goal_data = observations['goal']  # (batch, 3)
                distance_data = observations['distance_to_goal']  # (batch, 1)
                goal_input = torch.cat([goal_data, distance_data], dim=-1)  # (batch, 4)
                goal_features = self.goal_encoder(goal_input)  # (batch, 16)
            
            # Encode Waypoint (NEW: Priority 2)
            waypoint_features = None
            if self.use_waypoint and 'waypoint' in observations:
                waypoint_data = observations['waypoint']  # (batch, 8)
                waypoint_features = self.waypoint_encoder(waypoint_data)  # (batch, 16)
            
            # Encode Velocity (NEW: Priority 3)
            velocity_features = None
            if self.use_velocity and 'velocity' in observations:
                velocity_data = observations['velocity']  # (batch, 5)
                velocity_features = self.velocity_encoder(velocity_data)  # (batch, 16)
            
            # Concatenate all features
            feature_list = [vision_features]
            if gps_features is not None:
                feature_list.append(gps_features)
            if goal_features is not None:
                feature_list.append(goal_features)
            if waypoint_features is not None:
                feature_list.append(waypoint_features)
            if velocity_features is not None:
                feature_list.append(velocity_features)
            
            features = torch.cat(feature_list, dim=-1)
        else:
            # Box observation: vision-only (backward compatible)
            # Convert from (batch, seq_len, H, W, C) to (batch, seq_len, C, H, W)
            observations = observations.permute(0, 1, 4, 2, 3)
            
            # Encode
            features = self.encoder(observations)
        
        # Project to desired dimension
        features = self.projection(features)
        
        return features


class VisionActorCriticPolicy(ActorCriticPolicy):
    """
    Custom Actor-Critic Policy for vision-based RL
    Uses VisionFeaturesExtractor for processing image observations
    """
    
    def __init__(
        self,
        observation_space: gym.Space,
        action_space: gym.Space,
        lr_schedule,
        net_arch: List[int] = None,
        activation_fn: Type[nn.Module] = nn.Tanh,
        config: Dict = None,
        *args,
        **kwargs
    ):
        if config is None:
            config = {}
        
        network_config = config.get('network', {})
        policy_config = network_config.get('policy', {})
        value_config = network_config.get('value', {})
        
        # Default network architecture
        if net_arch is None:
            net_arch = policy_config.get('hidden_sizes', [512, 256, 128])
        
        # Features extractor
        features_extractor_class = VisionFeaturesExtractor
        features_extractor_kwargs = {
            'features_dim': 256,
            'config': config
        }
        
        super().__init__(
            observation_space,
            action_space,
            lr_schedule,
            net_arch=net_arch,
            activation_fn=activation_fn,
            features_extractor_class=features_extractor_class,
            features_extractor_kwargs=features_extractor_kwargs,
            *args,
            **kwargs
        )

