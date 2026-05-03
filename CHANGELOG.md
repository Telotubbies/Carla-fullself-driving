# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [Unreleased]

### Added
- `docs/` directory consolidating all extended documentation
- `examples/` directory for demo and smoke-test scripts
- `scripts/train/` sub-directory for all training entry points
- `CHANGELOG.md` following Keep a Changelog standard

### Changed
- Reorganised repository layout to production standard
- Moved training scripts from root to `scripts/train/`
- Moved demo scripts from root to `examples/`
- Moved documentation `.md` files from root to `docs/`
- Moved visualisation and deployment helpers into `scripts/`
- Moved root-level `test_*.py` files into `tests/`
- Updated `Makefile` targets to reflect new paths
- Updated `pyproject.toml` URLs to point to actual GitHub repository
- Rewrote `README.md` with production-grade documentation

### Removed
- Redundant status/summary markdown files (`READY_TO_USE.md`, `SYSTEM_STATUS_TH.md`, etc.)
- Stale IEEE LaTeX report files
- `training_output.log` from version control (now in `.gitignore`)

---

## [0.1.0] - 2026-05-04

### Added
- SAC reinforcement learning agent for autonomous driving in CARLA
- Ray RLlib training pipeline with curriculum learning
- Ground-truth state builder (`src/gt_state/`) with configurable noise simulation
- LiDAR Bird's Eye View (BEV) grid processor
- ROS2 bridge for sensor publishing and control subscription
- MLflow experiment tracking integration
- Curriculum wrapper with automatic stage advancement
- Multi-modal observation support (BEV + kinematics + waypoints)
- Comprehensive reward shaping (progress, comfort, collision, lane, speed)
- TensorBoard logging
- RLlib checkpoint save/resume
- ROS2 colcon package (`ros2_ws/src/carla_sac_bridge`)
- Evaluation and deployment scripts
- Full pytest test suite

[Unreleased]: https://github.com/Telotubbies/Carla-fullself-driving/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/Telotubbies/Carla-fullself-driving/releases/tag/v0.1.0
