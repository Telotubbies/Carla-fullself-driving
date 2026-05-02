// Python Bindings for Rust RL
// ใช้ PyO3 เพื่อเรียก Rust จาก Python

use pyo3::prelude::*;
use pyo3::types::PyList;
use numpy::{PyArray1, PyArray2};
use ndarray::Array1;
use crate::sac::SACAgent;
use crate::replay_buffer::{ReplayBuffer, Transition};
use tch::Device;

/// Python wrapper for SAC Agent
#[pyclass]
pub struct RustSACAgent {
    agent: SACAgent,
}

#[pymethods]
impl RustSACAgent {
    #[new]
    fn new(
        obs_dim: i64,
        action_dim: i64,
        hidden_dim: i64,
        lr: f64,
        gamma: f64,
        tau: f64,
        alpha: f64,
        use_cuda: bool,
    ) -> Self {
        let device = if use_cuda && tch::Cuda::is_available() {
            Device::Cuda(0)
        } else {
            Device::Cpu
        };
        
        let agent = SACAgent::new(
            obs_dim,
            action_dim,
            hidden_dim,
            lr,
            gamma,
            tau,
            alpha,
            device,
        );
        
        RustSACAgent { agent }
    }
    
    /// Select action (numpy array input/output)
    fn select_action<'py>(
        &self,
        py: Python<'py>,
        obs: &PyArray1<f32>,
        deterministic: bool,
    ) -> &'py PyArray1<f32> {
        let obs_slice = unsafe { obs.as_slice().unwrap() };
        let obs_tensor = tch::Tensor::of_slice(obs_slice)
            .to_device(self.agent.device)
            .unsqueeze(0);
        
        let action_tensor = self.agent.select_action(&obs_tensor, deterministic);
        let action_vec: Vec<f32> = action_tensor.squeeze_dim(0).try_into().unwrap();
        
        PyArray1::from_vec(py, action_vec)
    }
    
    /// Update networks (numpy arrays input)
    fn update(
        &mut self,
        obs: &PyArray2<f32>,
        action: &PyArray2<f32>,
        reward: &PyArray1<f32>,
        next_obs: &PyArray2<f32>,
        done: &PyArray1<f32>,
    ) -> (f64, f64, f64, f64) {
        // Convert numpy to tensors
        let obs_tensor = numpy_to_tensor_2d(obs, self.agent.device);
        let action_tensor = numpy_to_tensor_2d(action, self.agent.device);
        let reward_tensor = numpy_to_tensor_1d(reward, self.agent.device).unsqueeze(-1);
        let next_obs_tensor = numpy_to_tensor_2d(next_obs, self.agent.device);
        let done_tensor = numpy_to_tensor_1d(done, self.agent.device).unsqueeze(-1);
        
        // Update
        self.agent.update(
            &obs_tensor,
            &action_tensor,
            &reward_tensor,
            &next_obs_tensor,
            &done_tensor,
        )
    }
    
    fn save(&self, path: String) -> PyResult<()> {
        self.agent.save(&path)
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyIOError, _>(e.to_string()))
    }
    
    fn load(&mut self, path: String) -> PyResult<()> {
        self.agent.load(&path)
            .map_err(|e| PyErr::new::<pyo3::exceptions::PyIOError, _>(e.to_string()))
    }
}

/// Python wrapper for Replay Buffer
#[pyclass]
pub struct RustReplayBuffer {
    buffer: ReplayBuffer,
}

#[pymethods]
impl RustReplayBuffer {
    #[new]
    fn new(capacity: usize, state_dim: usize, action_dim: usize) -> Self {
        RustReplayBuffer {
            buffer: ReplayBuffer::new(capacity, state_dim, action_dim),
        }
    }
    
    fn push(
        &self,
        state: &PyArray1<f32>,
        action: &PyArray1<f32>,
        reward: f32,
        next_state: &PyArray1<f32>,
        done: f32,
    ) {
        let state_vec = unsafe { state.as_slice().unwrap() };
        let action_vec = unsafe { action.as_slice().unwrap() };
        let next_state_vec = unsafe { next_state.as_slice().unwrap() };
        
        let transition = Transition {
            state: Array1::from_vec(state_vec.to_vec()),
            action: Array1::from_vec(action_vec.to_vec()),
            reward,
            next_state: Array1::from_vec(next_state_vec.to_vec()),
            done,
        };
        
        self.buffer.push(transition);
    }
    
    fn sample<'py>(
        &self,
        py: Python<'py>,
        batch_size: usize,
    ) -> Option<(
        &'py PyArray2<f32>,
        &'py PyArray2<f32>,
        &'py PyArray1<f32>,
        &'py PyArray2<f32>,
        &'py PyArray1<f32>,
    )> {
        let batch = self.buffer.sample(batch_size)?;
        
        Some((
            PyArray2::from_array(py, &batch.states),
            PyArray2::from_array(py, &batch.actions),
            PyArray1::from_array(py, &batch.rewards),
            PyArray2::from_array(py, &batch.next_states),
            PyArray1::from_array(py, &batch.dones),
        ))
    }
    
    fn __len__(&self) -> usize {
        self.buffer.len()
    }
    
    fn clear(&self) {
        self.buffer.clear();
    }
}

// Helper functions
fn numpy_to_tensor_2d(arr: &PyArray2<f32>, device: Device) -> tch::Tensor {
    let shape = arr.shape();
    let data: Vec<f32> = unsafe { arr.as_slice().unwrap().to_vec() };
    tch::Tensor::of_slice(&data)
        .reshape(&[shape[0] as i64, shape[1] as i64])
        .to_device(device)
}

fn numpy_to_tensor_1d(arr: &PyArray1<f32>, device: Device) -> tch::Tensor {
    let data: Vec<f32> = unsafe { arr.as_slice().unwrap().to_vec() };
    tch::Tensor::of_slice(&data).to_device(device)
}
