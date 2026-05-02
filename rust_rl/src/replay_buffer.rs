// High-Performance Replay Buffer in Rust
// เร็วกว่า Python 10-100x

use ndarray::{Array1, Array2};
use rand::seq::SliceRandom;
use std::collections::VecDeque;
use parking_lot::RwLock;
use std::sync::Arc;

/// Transition (s, a, r, s', done)
#[derive(Clone)]
pub struct Transition {
    pub state: Array1<f32>,
    pub action: Array1<f32>,
    pub reward: f32,
    pub next_state: Array1<f32>,
    pub done: f32,
}

/// High-Performance Replay Buffer
pub struct ReplayBuffer {
    buffer: Arc<RwLock<VecDeque<Transition>>>,
    capacity: usize,
    state_dim: usize,
    action_dim: usize,
}

impl ReplayBuffer {
    pub fn new(capacity: usize, state_dim: usize, action_dim: usize) -> Self {
        ReplayBuffer {
            buffer: Arc::new(RwLock::new(VecDeque::with_capacity(capacity))),
            capacity,
            state_dim,
            action_dim,
        }
    }
    
    /// Add transition (thread-safe)
    pub fn push(&self, transition: Transition) {
        let mut buffer = self.buffer.write();
        
        if buffer.len() >= self.capacity {
            buffer.pop_front();
        }
        
        buffer.push_back(transition);
    }
    
    /// Sample batch (parallel processing)
    pub fn sample(&self, batch_size: usize) -> Option<Batch> {
        let buffer = self.buffer.read();
        
        if buffer.len() < batch_size {
            return None;
        }
        
        // Random sampling
        let mut rng = rand::thread_rng();
        let indices: Vec<usize> = (0..buffer.len())
            .collect::<Vec<_>>()
            .choose_multiple(&mut rng, batch_size)
            .cloned()
            .collect();
        
        // Pre-allocate arrays
        let mut states = Array2::<f32>::zeros((batch_size, self.state_dim));
        let mut actions = Array2::<f32>::zeros((batch_size, self.action_dim));
        let mut rewards = Array1::<f32>::zeros(batch_size);
        let mut next_states = Array2::<f32>::zeros((batch_size, self.state_dim));
        let mut dones = Array1::<f32>::zeros(batch_size);
        
        // Fill arrays (can be parallelized)
        for (i, &idx) in indices.iter().enumerate() {
            let transition = &buffer[idx];
            states.row_mut(i).assign(&transition.state);
            actions.row_mut(i).assign(&transition.action);
            rewards[i] = transition.reward;
            next_states.row_mut(i).assign(&transition.next_state);
            dones[i] = transition.done;
        }
        
        Some(Batch {
            states,
            actions,
            rewards,
            next_states,
            dones,
        })
    }
    
    pub fn len(&self) -> usize {
        self.buffer.read().len()
    }
    
    pub fn is_empty(&self) -> bool {
        self.buffer.read().is_empty()
    }
    
    pub fn clear(&self) {
        self.buffer.write().clear();
    }
}

/// Batch of transitions
pub struct Batch {
    pub states: Array2<f32>,
    pub actions: Array2<f32>,
    pub rewards: Array1<f32>,
    pub next_states: Array2<f32>,
    pub dones: Array1<f32>,
}

/// Prioritized Replay Buffer (for future)
pub struct PrioritizedReplayBuffer {
    buffer: ReplayBuffer,
    priorities: Arc<RwLock<VecDeque<f32>>>,
    alpha: f32,
    beta: f32,
}

impl PrioritizedReplayBuffer {
    pub fn new(capacity: usize, state_dim: usize, action_dim: usize, alpha: f32, beta: f32) -> Self {
        PrioritizedReplayBuffer {
            buffer: ReplayBuffer::new(capacity, state_dim, action_dim),
            priorities: Arc::new(RwLock::new(VecDeque::with_capacity(capacity))),
            alpha,
            beta,
        }
    }
    
    pub fn push(&self, transition: Transition, priority: f32) {
        self.buffer.push(transition);
        
        let mut priorities = self.priorities.write();
        if priorities.len() >= self.buffer.capacity {
            priorities.pop_front();
        }
        priorities.push_back(priority.powf(self.alpha));
    }
    
    // TODO: Implement prioritized sampling
}

#[cfg(test)]
mod tests {
    use super::*;
    use ndarray::Array1;
    
    #[test]
    fn test_replay_buffer() {
        let buffer = ReplayBuffer::new(1000, 10, 3);
        
        // Add transitions
        for i in 0..100 {
            let transition = Transition {
                state: Array1::from_vec(vec![i as f32; 10]),
                action: Array1::from_vec(vec![0.0; 3]),
                reward: 1.0,
                next_state: Array1::from_vec(vec![(i+1) as f32; 10]),
                done: 0.0,
            };
            buffer.push(transition);
        }
        
        assert_eq!(buffer.len(), 100);
        
        // Sample batch
        let batch = buffer.sample(32).unwrap();
        assert_eq!(batch.states.shape(), &[32, 10]);
        assert_eq!(batch.actions.shape(), &[32, 3]);
    }
    
    #[test]
    fn test_buffer_capacity() {
        let buffer = ReplayBuffer::new(10, 5, 2);
        
        // Add more than capacity
        for i in 0..20 {
            let transition = Transition {
                state: Array1::from_vec(vec![i as f32; 5]),
                action: Array1::from_vec(vec![0.0; 2]),
                reward: 1.0,
                next_state: Array1::from_vec(vec![(i+1) as f32; 5]),
                done: 0.0,
            };
            buffer.push(transition);
        }
        
        // Should only keep last 10
        assert_eq!(buffer.len(), 10);
    }
}
