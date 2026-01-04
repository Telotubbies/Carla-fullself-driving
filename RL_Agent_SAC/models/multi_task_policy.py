import torch
import torch.nn as nn
from typing import Dict, List, Tuple, Type
import gymnasium as gym
from stable_baselines3.common.policies import ActorCriticPolicy
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
try:
    from .custom_policy import VisionFeaturesExtractor
except ImportError:
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).parent.parent))
    from models.custom_policy import VisionFeaturesExtractor
class MultiTaskPolicy(ActorCriticPolicy):
    
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
        multi_task_config = config.get('multi_task_learning', {})
        self.multi_task_enabled = multi_task_config.get('enabled', False)
        self.task_weights = multi_task_config.get('task_weights', {
            'lane_keeping': 1.0,
            'speed_control': 1.0,
            'obstacle_avoidance': 0.5
        })
        features_extractor_class = VisionFeaturesExtractor
        features_extractor_kwargs = {
            'features_dim': 256,
            'config': config
        }
        if net_arch is None:
            net_arch = [512, 256, 128]
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
        if self.multi_task_enabled:
            self._build_task_heads(net_arch, activation_fn)
    def _build_task_heads(self, net_arch: List[int], activation_fn: Type[nn.Module]):
        
        feature_dim = 256
        lane_keeping_layers = []
        prev_dim = feature_dim
        for hidden_dim in net_arch:
            lane_keeping_layers.append(nn.Linear(prev_dim, hidden_dim))
            lane_keeping_layers.append(activation_fn())
            prev_dim = hidden_dim
        lane_keeping_layers.append(nn.Linear(prev_dim, 1))
        self.lane_keeping_head = nn.Sequential(*lane_keeping_layers)
        speed_control_layers = []
        prev_dim = feature_dim
        for hidden_dim in net_arch:
            speed_control_layers.append(nn.Linear(prev_dim, hidden_dim))
            speed_control_layers.append(activation_fn())
            prev_dim = hidden_dim
        speed_control_layers.append(nn.Linear(prev_dim, 2))
        self.speed_control_head = nn.Sequential(*speed_control_layers)
        obstacle_avoidance_layers = []
        prev_dim = feature_dim
        for hidden_dim in net_arch[:2]:
            obstacle_avoidance_layers.append(nn.Linear(prev_dim, hidden_dim))
            obstacle_avoidance_layers.append(activation_fn())
            prev_dim = hidden_dim
        obstacle_avoidance_layers.append(nn.Linear(prev_dim, 1))
        obstacle_avoidance_layers.append(nn.Sigmoid())
        self.obstacle_avoidance_head = nn.Sequential(*obstacle_avoidance_layers)
    def forward(self, obs, deterministic: bool = False):
        
        features = self.extract_features(obs, self.features_extractor)
        latent_pi, latent_vf = self.mlp_extractor(features)
        values = self.value_net(latent_vf)
        distribution = self._get_action_dist_from_latent(latent_pi)
        actions = distribution.get_actions(deterministic=deterministic)
        log_prob = distribution.log_prob(actions)
        task_predictions = {}
        if self.multi_task_enabled:
            task_predictions['lane_keeping'] = self.lane_keeping_head(features)
            task_predictions['speed_control'] = self.speed_control_head(features)
            task_predictions['obstacle_avoidance'] = self.obstacle_avoidance_head(features)
        return actions, values, log_prob, task_predictions
    def evaluate_actions(self, obs, actions):
        
        features = self.extract_features(obs, self.features_extractor)
        latent_pi, latent_vf = self.mlp_extractor(features)
        distribution = self._get_action_dist_from_latent(latent_pi)
        log_prob = distribution.log_prob(actions)
        entropy = distribution.entropy()
        values = self.value_net(latent_vf)
        task_predictions = {}
        if self.multi_task_enabled:
            task_predictions['lane_keeping'] = self.lane_keeping_head(features)
            task_predictions['speed_control'] = self.speed_control_head(features)
            task_predictions['obstacle_avoidance'] = self.obstacle_avoidance_head(features)
        return values, log_prob, entropy, task_predictions
def compute_multi_task_loss(task_predictions: Dict, task_targets: Dict, task_weights: Dict) -> torch.Tensor:
    
    total_loss = 0.0
    if 'lane_keeping' in task_predictions and 'lane_keeping' in task_targets:
        lane_loss = nn.functional.mse_loss(
            task_predictions['lane_keeping'],
            task_targets['lane_keeping']
        )
        total_loss += task_weights.get('lane_keeping', 1.0) * lane_loss
    if 'speed_control' in task_predictions and 'speed_control' in task_targets:
        speed_loss = nn.functional.mse_loss(
            task_predictions['speed_control'],
            task_targets['speed_control']
        )
        total_loss += task_weights.get('speed_control', 1.0) * speed_loss
    if 'obstacle_avoidance' in task_predictions and 'obstacle_avoidance' in task_targets:
        obstacle_loss = nn.functional.binary_cross_entropy(
            task_predictions['obstacle_avoidance'],
            task_targets['obstacle_avoidance']
        )
        total_loss += task_weights.get('obstacle_avoidance', 0.5) * obstacle_loss
    return total_loss