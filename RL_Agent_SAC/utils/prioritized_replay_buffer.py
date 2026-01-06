import numpy as np
import torch
from collections import deque
from typing import Dict, Tuple, Optional, List, Any
import logging
class PrioritizedReplayBuffer:
    
    def __init__(
        self,
        capacity: int = 10000,
        alpha: float = 0.6,
        beta: float = 0.4,
        beta_increment: float = 0.001,
        epsilon: float = 1e-6
    ):
        
        self.capacity = capacity
        self.alpha = alpha
        self.beta = beta
        self.beta_increment = beta_increment
        self.epsilon = epsilon
        self.observations = deque(maxlen=capacity)
        self.actions = deque(maxlen=capacity)
        self.rewards = deque(maxlen=capacity)
        self.next_observations = deque(maxlen=capacity)
        self.dones = deque(maxlen=capacity)
        self.priorities = np.zeros(capacity, dtype=np.float32)
        self.position = 0
        self.size = 0
        self.tree_size = 1
        while self.tree_size < capacity:
            self.tree_size *= 2
        self.tree = np.zeros(2 * self.tree_size - 1, dtype=np.float32)
        logging.info(f"✅ PrioritizedReplayBuffer initialized: capacity={capacity}, alpha={alpha}, beta={beta}")
    def add(
        self,
        observation: np.ndarray,
        action: np.ndarray,
        reward: float,
        next_observation: np.ndarray,
        done: bool,
        priority: Optional[float] = None
    ):
        
        self.observations.append(observation.copy() if isinstance(observation, np.ndarray) else observation)
        self.actions.append(action.copy() if isinstance(action, np.ndarray) else action)
        self.rewards.append(reward)
        self.next_observations.append(next_observation.copy() if isinstance(next_observation, np.ndarray) else next_observation)
        self.dones.append(done)
        if priority is None:
            priority = self._get_max_priority()
        self._update_priority(self.position, priority)
        self.position = (self.position + 1) % self.capacity
        if self.size < self.capacity:
            self.size += 1
    def sample(self, batch_size: int) -> Tuple[Dict[str, np.ndarray], np.ndarray, np.ndarray]:
        
        if self.size < batch_size:
            batch_size = self.size
        indices = np.zeros(batch_size, dtype=np.int32)
        priorities = np.zeros(batch_size, dtype=np.float32)
        segment = self.tree[0] / batch_size
        for i in range(batch_size):
            a = segment * i
            b = segment * (i + 1)
            value = np.random.uniform(a, b)
            idx = self._retrieve(0, value)
            indices[i] = idx
            priorities[i] = self.priorities[idx]
        probabilities = priorities / self.tree[0]
        weights = np.power(self.size * probabilities, -self.beta)
        weights /= weights.max()
        self.beta = min(1.0, self.beta + self.beta_increment)
        batch = {
            'obs': np.array([self.observations[i] for i in indices]),
            'actions': np.array([self.actions[i] for i in indices]),
            'rewards': np.array([self.rewards[i] for i in indices], dtype=np.float32),
            'next_obs': np.array([self.next_observations[i] for i in indices]),
            'dones': np.array([self.dones[i] for i in indices], dtype=np.bool_)
        }
        return batch, indices, weights
    def update_priorities(self, indices: np.ndarray, td_errors: np.ndarray):
        
        for idx, td_error in zip(indices, td_errors):
            priority = (abs(td_error) + self.epsilon) ** self.alpha
            self._update_priority(idx, priority)
    def _update_priority(self, idx: int, priority: float):
        
        change = priority - self.priorities[idx]
        self.priorities[idx] = priority
        tree_idx = idx + self.tree_size - 1
        self.tree[tree_idx] = priority
        while tree_idx != 0:
            tree_idx = (tree_idx - 1) // 2
            self.tree[tree_idx] = self.tree[2 * tree_idx + 1] + self.tree[2 * tree_idx + 2]
    def _retrieve(self, idx: int, value: float) -> int:
        
        left = 2 * idx + 1
        right = left + 1
        if left >= len(self.tree):
            return idx - self.tree_size + 1
        if value <= self.tree[left]:
            return self._retrieve(left, value)
        else:
            return self._retrieve(right, value - self.tree[left])
    def _get_max_priority(self) -> float:
        
        if self.size == 0:
            return 1.0
        return self.priorities[:self.size].max()
    def __len__(self) -> int:
        
        return self.size
    def get_stats(self) -> Dict[str, Any]:
        
        stats = {
            'size': self.size,
            'capacity': self.capacity,
            'full': self.size >= self.capacity,
            'beta': self.beta
        }
        if self.size == 0:
            stats.update({
                'avg_priority': 0.0,
                'max_priority': 0.0,
                'min_priority': 0.0
            })
        else:
            priorities = self.priorities[:self.size]
            stats.update({
                'avg_priority': float(priorities.mean()),
                'max_priority': float(priorities.max()),
                'min_priority': float(priorities.min())
            })
        return stats