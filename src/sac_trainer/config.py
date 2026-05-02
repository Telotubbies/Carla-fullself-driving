from ray.rllib.algorithms.sac import SACConfig
from ray.rllib.models import ModelCatalog
from typing import Dict, Any


def get_sac_config() -> Dict[str, Any]:
    """
    Get SAC configuration for CARLA training.
    
    Returns:
        Configuration dictionary for RLlib SAC algorithm
    """
    
    config = (
        SACConfig()
        .api_stack(
            enable_rl_module_and_learner=False,
            enable_env_runner_and_connector_v2=False,
        )
        .environment(
            env="carla_env",
            env_config={
                'host': 'localhost',
                'port': 2000,
                'timeout': 10.0,
                'map': 'Town01',
                'delta_seconds': 0.05,
                'max_episode_steps': 1000,
                'sensor_config': {
                    'lidar_channels': 64,
                    'lidar_range': 100,
                    'lidar_pps': 500000,
                    'lidar_freq': 20,
                    'lidar_upper_fov': 15,
                    'lidar_lower_fov': -25,
                    'camera_width': 640,
                    'camera_height': 480,
                    'camera_fov': 90,
                    'bev_range': 50.0,
                    'bev_resolution': 256,
                },
                'reward_config': {
                    'w_progress': 1.0,
                    'w_comfort': 0.1,
                    'w_collision': 200.0,
                    'w_lane_deviation': 0.5,
                    'w_speed': 0.2,
                    'target_speed': 30.0 / 3.6,  # 30 km/h in m/s
                }
            }
        )
        .framework("torch")
        .training(
            # SAC hyperparameters
            gamma=0.99,
            actor_lr=3e-4,
            critic_lr=3e-4,
            alpha_lr=3e-4,
            train_batch_size=256,
            target_network_update_freq=1,
            tau=0.005,
            
            # Replay buffer (legacy API)
            replay_buffer_config={
                "type": "MultiAgentReplayBuffer",
                "capacity": 100000,
            },
            
            # Initial alpha (entropy coefficient)
            initial_alpha=1.0,
            target_entropy="auto",
            
            # Number of steps before training starts
            num_steps_sampled_before_learning_starts=10000,
            
            # Twin Q-networks
            twin_q=True,
        )
        .env_runners(
            num_env_runners=4,
            num_envs_per_env_runner=1,
            num_cpus_per_env_runner=2,
            rollout_fragment_length=1,
            batch_mode="complete_episodes",
        )
        .resources(
            num_gpus=1,
        )
        .evaluation(
            evaluation_interval=10,
            evaluation_duration=10,
            evaluation_duration_unit="episodes",
            evaluation_num_env_runners=1,
            evaluation_config={
                "explore": False,
            }
        )
        .reporting(
            min_time_s_per_iteration=0,
            min_sample_timesteps_per_iteration=1000,
        )
        .debugging(
            log_level="INFO",
            seed=42,
        )
    )
    
    return config


def get_model_config() -> Dict[str, Any]:
    """
    Get model architecture configuration.
    
    Returns:
        Model configuration dictionary
    """
    return {
        "custom_model": "carla_sac_model",
        "custom_model_config": {
            # CNN for LiDAR BEV processing
            "conv_filters": [
                [32, [8, 8], 4],
                [64, [4, 4], 2],
                [64, [3, 3], 1],
            ],
            
            # MLP for ego state processing
            "ego_state_hiddens": [256, 256],
            
            # Combined feature processing
            "fcnet_hiddens": [512, 512],
            "fcnet_activation": "relu",
            
            # Post-processing
            "post_fcnet_hiddens": [256],
            "post_fcnet_activation": "relu",
        },
        
        # Frame stacking
        "framestack": True,
        "num_framestacks": 4,
        
        # Normalization
        "vf_share_layers": False,
    }


def get_training_config() -> Dict[str, Any]:
    """
    Get general training configuration.
    
    Returns:
        Training configuration dictionary
    """
    return {
        # Training
        "num_iterations": 1000,
        "checkpoint_freq": 10,
        "checkpoint_at_end": True,
        
        # Logging
        "log_dir": "data/logs",
        "tensorboard_dir": "data/tensorboard",
        
        # Checkpoints
        "checkpoint_dir": "data/checkpoints",
        "keep_checkpoints_num": 5,
        
        # Experiment
        "experiment_name": "carla_sac_training",
        "run_name": "sac_town01",
        
        # Wandb (optional)
        "use_wandb": False,
        "wandb_project": "carla-autonomous-driving",
        "wandb_entity": None,
    }
