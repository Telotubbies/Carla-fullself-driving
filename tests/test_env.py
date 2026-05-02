import pytest
import numpy as np
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent))

from src.carla_gym_env import CarlaEnv


@pytest.fixture
def env_config():
    """Fixture for environment configuration."""
    return {
        'host': 'localhost',
        'port': 2000,
        'timeout': 10.0,
        'map': 'Town01',
        'delta_seconds': 0.05,
        'max_episode_steps': 100,
        'sensor_config': {
            'lidar_channels': 32,
            'lidar_range': 50,
            'bev_range': 25.0,
            'bev_resolution': 128,
        },
        'reward_config': {
            'w_progress': 1.0,
            'w_comfort': 0.1,
            'w_collision': 200.0,
        }
    }


@pytest.fixture
def env(env_config):
    """Fixture for CARLA environment."""
    env = CarlaEnv(env_config)
    yield env
    env.close()


def test_env_creation(env):
    """Test environment creation."""
    assert env is not None
    assert env.observation_space is not None
    assert env.action_space is not None


def test_observation_space(env):
    """Test observation space."""
    obs_space = env.observation_space
    
    assert 'lidar_bev' in obs_space.spaces
    assert 'ego_state' in obs_space.spaces
    
    lidar_shape = obs_space['lidar_bev'].shape
    assert len(lidar_shape) == 3
    assert lidar_shape[2] == 3  # 3 channels


def test_action_space(env):
    """Test action space."""
    action_space = env.action_space
    
    assert action_space.shape == (3,)  # [steering, throttle, brake]
    assert np.all(action_space.low == np.array([-1.0, 0.0, 0.0]))
    assert np.all(action_space.high == np.array([1.0, 1.0, 1.0]))


def test_reset(env):
    """Test environment reset."""
    obs, info = env.reset()
    
    assert obs is not None
    assert 'lidar_bev' in obs
    assert 'ego_state' in obs
    assert info is not None
    assert 'episode_step' in info


def test_step(env):
    """Test environment step."""
    env.reset()
    
    # Random action
    action = env.action_space.sample()
    obs, reward, terminated, truncated, info = env.step(action)
    
    assert obs is not None
    assert isinstance(reward, (int, float))
    assert isinstance(terminated, bool)
    assert isinstance(truncated, bool)
    assert info is not None


def test_episode(env):
    """Test full episode."""
    obs, info = env.reset()
    
    done = False
    truncated = False
    total_reward = 0.0
    steps = 0
    max_steps = 50
    
    while not (done or truncated) and steps < max_steps:
        action = env.action_space.sample()
        obs, reward, done, truncated, info = env.step(action)
        total_reward += reward
        steps += 1
    
    assert steps > 0
    assert isinstance(total_reward, (int, float))


def test_observation_shape(env):
    """Test observation shapes."""
    obs, _ = env.reset()
    
    lidar_bev = obs['lidar_bev']
    ego_state = obs['ego_state']
    
    assert lidar_bev.shape == (256, 256, 3)
    assert ego_state.shape == (6,)


def test_action_clipping(env):
    """Test action clipping."""
    env.reset()
    
    # Test extreme actions
    action = np.array([2.0, 2.0, 2.0])  # Out of bounds
    obs, reward, done, truncated, info = env.step(action)
    
    # Should not crash
    assert obs is not None


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
