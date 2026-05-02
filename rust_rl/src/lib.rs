// CARLA SAC in Rust - High Performance RL
// แปลงจาก Python เป็น Rust เพื่อ performance สูงสุด

pub mod sac;
pub mod replay_buffer;
pub mod networks;
pub mod env_interface;
pub mod python_bindings;

use pyo3::prelude::*;

/// Python module สำหรับเรียกใช้จาก Python
#[pymodule]
fn carla_sac_rust(_py: Python, m: &PyModule) -> PyResult<()> {
    m.add_class::<python_bindings::RustSACAgent>()?;
    m.add_class::<python_bindings::RustReplayBuffer>()?;
    Ok(())
}
