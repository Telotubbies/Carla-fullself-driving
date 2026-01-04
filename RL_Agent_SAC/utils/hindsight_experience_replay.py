import numpy as np
from typing import Dict, List, Tuple, Optional
import logging
class HindsightExperienceReplay:
    
    def __init__(
        self,
        capacity: int = 10000,
        goal_dim: int = 3,
        k: int = 4
    ):
        
        self.capacity = capacity
        self.goal_dim = goal_dim
        self.k = k
        self.observations = []
        self.actions = []
        self.rewards = []
        self.next_observations = []
        self.dones = []
        self.goals = []
        self.achieved_goals = []
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
        
        if achieved_goal is None:
            if isinstance(next_obs, dict) and 'goal' in next_obs:
                achieved_goal = next_obs['goal']
            else:
                achieved_goal = np.zeros(self.goal_dim)
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
        
        indices = np.random.randint(0, self.size, size=batch_size)
        obs_batch = [self.observations[i] for i in indices]
        action_batch = [self.actions[i] for i in indices]
        reward_batch = [self.rewards[i] for i in indices]
        next_obs_batch = [self.next_observations[i] for i in indices]
        done_batch = [self.dones[i] for i in indices]
        goal_batch = [self.goals[i] for i in indices]
        achieved_batch = [self.achieved_goals[i] for i in indices]
        for i in range(batch_size):
            if np.random.random() < 0.5:
                goal_batch[i] = achieved_batch[i].copy()
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
        
        if isinstance(obs, dict) and 'goal' in obs:
            current_goal = obs['goal']
        else:
            current_goal = np.zeros(self.goal_dim)
        distance = np.linalg.norm(current_goal - goal)
        reward = -distance
        if distance < 5.0:
            reward += 10.0
        return reward