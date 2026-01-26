import torch
import torch.nn as nn
from typing import Dict, List, Tuple, Type
import gymnasium as gym
from stable_baselines3.common.policies import ActorCriticPolicy
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
try:
    from .vision_encoder import VisionTemporalEncoder
except ImportError:
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).parent.parent))
    from models.vision_encoder import VisionTemporalEncoder
class VisionFeaturesExtractor(BaseFeaturesExtractor):
    
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
        self.use_gps = obs_config.get('use_gps', False)
        self.use_goal = obs_config.get('use_goal', False)
        self.use_waypoint = obs_config.get('use_waypoint', True)
        self.use_velocity = obs_config.get('use_velocity', True)
        self.is_dict_obs = isinstance(observation_space, gym.spaces.Dict)
        if self.is_dict_obs:
            vision_space = observation_space['vision']
            obs_shape = vision_space.shape
        else:
            obs_shape = observation_space.shape
        sequence_length, height, width, channels = obs_shape
        use_yolo = False
        yolo_feature_dim = 0
        device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.encoder = VisionTemporalEncoder(
            input_channels=channels,
            sequence_length=sequence_length,
            vision_type=vision_config.get('type', 'CNN'),
            vision_channels=vision_config.get('channels', [32, 64, 128, 256]),
            vision_kernels=vision_config.get('kernel_sizes', [8, 4, 3, 3]),
            vision_strides=vision_config.get('strides', [4, 2, 2, 1]),
            resnet_type=vision_config.get('resnet_type', 'resnet18'),
            pretrained=vision_config.get('pretrained', False),
            freeze_early_layers=vision_config.get('freeze_early_layers', False),
            temporal_hidden_size=temporal_config.get('hidden_size', 256),
            temporal_num_layers=temporal_config.get('num_layers', 2),
            temporal_dropout=temporal_config.get('dropout', 0.1),
            image_size=(height, width),
            activation=vision_config.get('activation', 'relu'),
            use_batch_norm=vision_config.get('use_batch_norm', True),
            use_yolo=False,
            yolo_feature_dim=0,
            device=device
        )
        if self.is_dict_obs:
            gps_dim = 0
            goal_dim = 0
            waypoint_dim = 0
            velocity_dim = 0
            if self.use_gps:
                self.gps_encoder = nn.Sequential(
                    nn.Linear(3, 16),
                    nn.ReLU(),
                    nn.Linear(16, 16)
                )
                gps_dim = 16
            else:
                self.gps_encoder = None
            if self.use_goal:
                # goal: 4D (x, y, z, relative_angle) + distance_to_goal: 1D = 5D total
                self.goal_encoder = nn.Sequential(
                    nn.Linear(5, 16),
                    nn.ReLU(),
                    nn.Linear(16, 16)
                )
                goal_dim = 16
            else:
                self.goal_encoder = None
            if self.use_waypoint:
                self.waypoint_encoder = nn.Sequential(
                    nn.Linear(8, 16),
                    nn.ReLU(),
                    nn.Linear(16, 16)
                )
                waypoint_dim = 16
            else:
                self.waypoint_encoder = None
            if self.use_velocity:
                # Velocity observation: shape=(7,) [speed_kmh, vx, vy, vz, speed_ms, yaw, yaw_rate]
                self.velocity_encoder = nn.Sequential(
                    nn.Linear(7, 16),
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
        encoder_output_size = self.encoder.output_size + gps_dim + goal_dim + waypoint_dim + velocity_dim
        if encoder_output_size != features_dim:
            self.projection = nn.Linear(encoder_output_size, features_dim)
        else:
            self.projection = nn.Identity()
    def forward(self, observations) -> torch.Tensor:
        
        if self.is_dict_obs:
            try:
                device = next(self.encoder.parameters()).device
            except StopIteration:
                device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
            vision_obs = observations['vision']
            if not isinstance(vision_obs, torch.Tensor):
                vision_obs = torch.as_tensor(vision_obs)
            vision_obs = vision_obs.permute(0, 1, 4, 2, 3)
            if vision_obs.device != device:
                vision_obs = vision_obs.to(device)
            vision_features = self.encoder(vision_obs)
            gps_features = None
            if self.use_gps and 'gps' in observations:
                gps_data = observations['gps']
                if not isinstance(gps_data, torch.Tensor):
                    gps_data = torch.as_tensor(gps_data)
                if gps_data.device != device:
                    gps_data = gps_data.to(device)
                gps_features = self.gps_encoder(gps_data)
            goal_features = None
            if self.use_goal and 'goal' in observations and 'distance_to_goal' in observations:
                goal_data = observations['goal']
                distance_data = observations['distance_to_goal']
                if not isinstance(goal_data, torch.Tensor):
                    goal_data = torch.as_tensor(goal_data)
                if not isinstance(distance_data, torch.Tensor):
                    distance_data = torch.as_tensor(distance_data)
                if goal_data.device != device:
                    goal_data = goal_data.to(device)
                if distance_data.device != device:
                    distance_data = distance_data.to(device)
                goal_input = torch.cat([goal_data, distance_data], dim=-1)
                goal_features = self.goal_encoder(goal_input)
            waypoint_features = None
            if self.use_waypoint and 'waypoint' in observations:
                waypoint_data = observations['waypoint']
                if not isinstance(waypoint_data, torch.Tensor):
                    waypoint_data = torch.as_tensor(waypoint_data)
                if waypoint_data.device != device:
                    waypoint_data = waypoint_data.to(device)
                waypoint_features = self.waypoint_encoder(waypoint_data)
            velocity_features = None
            if self.use_velocity and 'velocity' in observations:
                velocity_data = observations['velocity']
                if not isinstance(velocity_data, torch.Tensor):
                    velocity_data = torch.as_tensor(velocity_data)
                if velocity_data.device != device:
                    velocity_data = velocity_data.to(device)
                velocity_features = self.velocity_encoder(velocity_data)
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
            observations = observations.permute(0, 1, 4, 2, 3)
            features = self.encoder(observations)
        features = self.projection(features)
        return features
class VisionActorCriticPolicy(ActorCriticPolicy):
    
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
        if net_arch is None:
            net_arch = policy_config.get('hidden_sizes', [512, 256, 128])
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