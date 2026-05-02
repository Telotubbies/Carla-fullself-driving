from ray.rllib.algorithms.callbacks import DefaultCallbacks
from ray.rllib.env import BaseEnv
from ray.rllib.evaluation import RolloutWorker
from ray.rllib.policy import Policy
from ray.rllib.utils.typing import EpisodeType
from typing import Dict, Optional
import numpy as np


class CarlaCallbacks(DefaultCallbacks):
    """Custom callbacks for CARLA SAC training."""
    
    def on_episode_start(
        self,
        *,
        worker: RolloutWorker,
        base_env: BaseEnv,
        policies: Dict[str, Policy],
        episode: EpisodeType,
        env_index: int,
        **kwargs
    ):
        """Called at the start of each episode."""
        episode.user_data["collision_count"] = 0
        episode.user_data["lane_deviation_sum"] = 0.0
        episode.user_data["speed_sum"] = 0.0
        episode.user_data["steps"] = 0
    
    def on_episode_step(
        self,
        *,
        worker: RolloutWorker,
        base_env: BaseEnv,
        policies: Optional[Dict[str, Policy]] = None,
        episode: EpisodeType,
        env_index: int,
        **kwargs
    ):
        """Called at each step of an episode."""
        info = episode.last_info_for()
        
        if info:
            # Track collision
            if info.get('collision', False):
                episode.user_data["collision_count"] += 1
            
            # Track speed
            speed = info.get('speed', 0.0)
            episode.user_data["speed_sum"] += speed
            
            episode.user_data["steps"] += 1
    
    def on_episode_end(
        self,
        *,
        worker: RolloutWorker,
        base_env: BaseEnv,
        policies: Dict[str, Policy],
        episode: EpisodeType,
        env_index: int,
        **kwargs
    ):
        """Called at the end of each episode."""
        # Calculate averages
        steps = episode.user_data["steps"]
        
        if steps > 0:
            avg_speed = episode.user_data["speed_sum"] / steps
            episode.custom_metrics["avg_speed"] = avg_speed
        
        # Log collision count
        episode.custom_metrics["collision_count"] = episode.user_data["collision_count"]
        
        # Log episode length
        episode.custom_metrics["episode_length"] = episode.length
        
        # Log success (completed without collision)
        success = episode.user_data["collision_count"] == 0 and episode.length > 100
        episode.custom_metrics["success"] = float(success)
    
    def on_train_result(
        self,
        *,
        algorithm,
        result: dict,
        **kwargs
    ):
        """Called after each training iteration."""
        # Log custom metrics
        if "custom_metrics" in result:
            custom = result["custom_metrics"]
            
            print("\nCustom Metrics:")
            if "avg_speed_mean" in custom:
                print(f"  Avg Speed: {custom['avg_speed_mean']:.2f} m/s")
            if "collision_count_mean" in custom:
                print(f"  Avg Collisions: {custom['collision_count_mean']:.2f}")
            if "success_mean" in custom:
                print(f"  Success Rate: {custom['success_mean']:.2%}")
    
    def on_sample_end(
        self,
        *,
        worker: RolloutWorker,
        samples,
        **kwargs
    ):
        """Called after sampling is done."""
        pass
    
    def on_learn_on_batch(
        self,
        *,
        policy: Policy,
        train_batch,
        result: dict,
        **kwargs
    ):
        """Called after learning on a batch."""
        pass
    
    def on_postprocess_trajectory(
        self,
        *,
        worker: RolloutWorker,
        episode: EpisodeType,
        agent_id: str,
        policy_id: str,
        policies: Dict[str, Policy],
        postprocessed_batch,
        original_batches: Dict[str, tuple],
        **kwargs
    ):
        """Called after trajectory post-processing."""
        pass
