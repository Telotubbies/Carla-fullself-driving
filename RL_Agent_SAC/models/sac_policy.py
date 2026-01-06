import torch
import torch.nn as nn
from typing import Dict, List, Tuple, Type
import gymnasium as gym
from stable_baselines3.sac.policies import SACPolicy
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor
try:
    from .custom_policy import VisionFeaturesExtractor
except ImportError:
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).parent.parent))
    from models.custom_policy import VisionFeaturesExtractor
class VisionSACPolicy(SACPolicy):
    
    def __init__(
        self,
        observation_space: gym.Space,
        action_space: gym.Space,
        lr_schedule,
        net_arch: List[int] = None,
        activation_fn: Type[nn.Module] = nn.ReLU,
        features_extractor_class: Type[BaseFeaturesExtractor] = None,
        features_extractor_kwargs: Dict = None,
        config: Dict = None,
        *args,
        **kwargs
    ):
        if config is None:
            config = {}
        network_config = config.get('network', {})
        policy_config = network_config.get('policy', {})
        if net_arch is None:
            net_arch = policy_config.get('hidden_sizes', [256, 256])
        if features_extractor_class is None:
            features_extractor_class = VisionFeaturesExtractor
        if features_extractor_kwargs is None:
            features_extractor_kwargs = {
                'features_dim': 256,
                'config': config
            }
        else:
            if 'config' not in features_extractor_kwargs:
                features_extractor_kwargs['config'] = config
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