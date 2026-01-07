"""
Hindsight Experience Replay (HER) for Goal-Based Tasks
เรียนรู้จาก failed episodes โดยสมมติ goal = สถานะสุดท้าย
"""

import numpy as np
from typing import Dict, List, Tuple, Optional
import logging


class HindsightExperienceReplay:
    """
    Hindsight Experience Replay Buffer
    สำหรับ goal-based navigation tasks
    """
    
    def __init__(
        self,
        capacity: int = 10000,
        goal_dim: int = 3,  # (x, y, z) for goal location
        k: int = 4  # Number of hindsight goals per transition
    ):
        """
        Initialize HER buffer
        
        Args:
            capacity: Maximum number of transitions
            goal_dim: Dimension of goal space
            k: Number of hindsight goals to sample per transition
        """
        self.capacity = capacity
        self.goal_dim = goal_dim
        self.k = k
        
        # Storage
        self.observations = []
        self.actions = []
        self.rewards = []
        self.next_observations = []
        self.dones = []
        self.goals = []  # Original goals
        self.achieved_goals = []  # Achieved states (for HER)
        
        self.size = 0
        self.position = 0
        
        logging.info(f"✅ HER Buffer initialized: capacity={capacity}, k={k}")
    
    def add(
        self,
        obs: np.ndarray,
        action: np.ndarray,
        reward: float,
        next_obs: np.ndarray,
        done: bool,
        goal: np.ndarray,
        achieved_goal: Optional[np.ndarray] = None
    ):
        """
        Add transition to buffer
        
        Args:
            obs: Current observation
            action: Action taken
            reward: Reward received
            next_obs: Next observation
            done: Whether episode ended
            goal: Goal state
            achieved_goal: Achieved state (if None, uses next_obs as achieved)
        """
        if achieved_goal is None:
            # Use next_obs as achieved goal (simplified)
            if isinstance(next_obs, dict) and 'goal' in next_obs:
                achieved_goal = next_obs['goal']
            else:
                # Extract goal from observation if possible
                achieved_goal = np.zeros(self.goal_dim)
        
        # Store transition
        if self.size < self.capacity:
            self.observations.append(obs)
            self.actions.append(action)
            self.rewards.append(reward)
            self.next_observations.append(next_obs)
            self.dones.append(done)
            self.goals.append(goal)
            self.achieved_goals.append(achieved_goal)
            self.size += 1
        else:
            # Overwrite oldest
            idx = self.position % self.capacity
            self.observations[idx] = obs
            self.actions[idx] = action
            self.rewards[idx] = reward
            self.next_observations[idx] = next_obs
            self.dones[idx] = done
            self.goals[idx] = goal
            self.achieved_goals[idx] = achieved_goal
            self.position += 1
    
    def sample(self, batch_size: int) -> Dict[str, np.ndarray]:
        """
        Sample batch with HER
        
        Args:
            batch_size: Number of transitions to sample
        
        Returns:
            Dictionary with observations, actions, rewards, etc.
        """
        # Sample indices
        indices = np.random.randint(0, self.size, size=batch_size)
        
        # Get original transitions
        obs_batch = [self.observations[i] for i in indices]
        action_batch = [self.actions[i] for i in indices]
        reward_batch = [self.rewards[i] for i in indices]
        next_obs_batch = [self.next_observations[i] for i in indices]
        done_batch = [self.dones[i] for i in indices]
        goal_batch = [self.goals[i] for i in indices]
        achieved_batch = [self.achieved_goals[i] for i in indices]
        
        # Apply HER: Replace some goals with achieved goals
        for i in range(batch_size):
            if np.random.random() < 0.5:  # 50% chance to use hindsight goal
                # Use achieved goal as new goal
                goal_batch[i] = achieved_batch[i].copy()
                # Recompute reward with new goal
                reward_batch[i] = self._compute_reward(obs_batch[i], goal_batch[i])
        
        return {
            'observations': np.array(obs_batch),
            'actions': np.array(action_batch),
            'rewards': np.array(reward_batch),
            'next_observations': np.array(next_obs_batch),
            'dones': np.array(done_batch),
            'goals': np.array(goal_batch)
        }
    
    def _compute_reward(self, obs: np.ndarray, goal: np.ndarray) -> float:
        """
        Compute reward for observation-goal pair
        
        Args:
            obs: Observation
            goal: Goal state
        
        Returns:
            Reward value
        """
        # Simplified reward: negative distance to goal
        if isinstance(obs, dict) and 'goal' in obs:
            current_goal = obs['goal']
        else:
            current_goal = np.zeros(self.goal_dim)
        
        distance = np.linalg.norm(current_goal - goal)
        reward = -distance  # Closer = higher reward
        
        # Bonus if goal reached
        if distance < 5.0:  # 5 meters threshold
            reward += 10.0
        
        return reward

