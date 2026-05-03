# 🏗️ Project Structure - Software Development Best Practices

**วันที่:** 10 เมษายน 2026  
**โครงสร้าง:** Clean Architecture + Domain-Driven Design

---

## 📁 โครงสร้างโปรเจคที่แนะนำ

```
carla_sac_ros2_training/
│
├── .github/                        # CI/CD workflows
│   ├── workflows/
│   │   ├── ci.yml                 # Continuous Integration
│   │   ├── tests.yml              # Automated tests
│   │   └── deploy.yml             # Deployment
│   └── ISSUE_TEMPLATE/
│
├── docs/                           # Documentation
│   ├── api/                       # API documentation
│   ├── guides/                    # User guides
│   │   ├── training.md
│   │   ├── perception.md
│   │   ├── ros2.md
│   │   └── rust.md
│   ├── architecture/              # Architecture diagrams
│   └── examples/                  # Example code
│
├── configs/                        # Configuration files
│   ├── environments/              # Environment configs
│   │   ├── carla.yaml
│   │   ├── training.yaml
│   │   └── curriculum.yaml
│   ├── models/                    # Model configs
│   │   ├── sac.yaml
│   │   └── perception.yaml
│   └── deployment/                # Deployment configs
│       ├── docker.yaml
│       └── kubernetes.yaml
│
├── src/                            # Source code (main package)
│   ├── carla_rl/                  # Main package
│   │   ├── __init__.py
│   │   │
│   │   ├── core/                  # Core domain logic
│   │   │   ├── __init__.py
│   │   │   ├── entities/          # Domain entities
│   │   │   │   ├── vehicle.py
│   │   │   │   ├── environment.py
│   │   │   │   └── trajectory.py
│   │   │   ├── interfaces/        # Abstract interfaces
│   │   │   │   ├── env_interface.py
│   │   │   │   ├── agent_interface.py
│   │   │   │   └── perception_interface.py
│   │   │   └── use_cases/         # Business logic
│   │   │       ├── train_agent.py
│   │   │       ├── evaluate_agent.py
│   │   │       └── collect_data.py
│   │   │
│   │   ├── infrastructure/        # External dependencies
│   │   │   ├── __init__.py
│   │   │   ├── carla/            # CARLA integration
│   │   │   │   ├── client.py
│   │   │   │   ├── sensors.py
│   │   │   │   └── world.py
│   │   │   ├── ros2/             # ROS2 integration
│   │   │   │   ├── bridge.py
│   │   │   │   ├── publishers.py
│   │   │   │   └── subscribers.py
│   │   │   └── storage/          # Data storage
│   │   │       ├── mlflow_store.py
│   │   │       └── hdf5_store.py
│   │   │
│   │   ├── application/           # Application services
│   │   │   ├── __init__.py
│   │   │   ├── training/         # Training services
│   │   │   │   ├── trainer.py
│   │   │   │   ├── curriculum.py
│   │   │   │   └── callbacks.py
│   │   │   ├── perception/       # Perception services
│   │   │   │   ├── lane_detection.py
│   │   │   │   ├── object_detection.py
│   │   │   │   ├── traffic_light.py
│   │   │   │   └── fusion.py
│   │   │   └── evaluation/       # Evaluation services
│   │   │       ├── metrics.py
│   │   │       └── benchmarks.py
│   │   │
│   │   ├── agents/                # RL Agents
│   │   │   ├── __init__.py
│   │   │   ├── sac/              # SAC implementation
│   │   │   │   ├── agent.py
│   │   │   │   ├── networks.py
│   │   │   │   ├── replay_buffer.py
│   │   │   │   └── config.py
│   │   │   ├── expert/           # Expert controller
│   │   │   │   ├── pid_controller.py
│   │   │   │   └── behavioral_cloning.py
│   │   │   └── base.py           # Base agent class
│   │   │
│   │   ├── environments/          # Gym environments
│   │   │   ├── __init__.py
│   │   │   ├── carla_env.py
│   │   │   ├── wrappers/
│   │   │   │   ├── perception_wrapper.py
│   │   │   │   ├── reward_wrapper.py
│   │   │   │   └── curriculum_wrapper.py
│   │   │   └── spaces/
│   │   │       ├── observation_space.py
│   │   │       └── action_space.py
│   │   │
│   │   ├── models/                # ML Models
│   │   │   ├── __init__.py
│   │   │   ├── perception/
│   │   │   │   ├── unet.py
│   │   │   │   ├── yolo.py
│   │   │   │   └── resnet.py
│   │   │   └── policy/
│   │   │       ├── actor.py
│   │   │       └── critic.py
│   │   │
│   │   └── utils/                 # Utilities
│   │       ├── __init__.py
│   │       ├── logging.py
│   │       ├── config_loader.py
│   │       ├── visualization.py
│   │       └── metrics.py
│   │
│   └── rust_rl/                   # Rust performance modules
│       ├── Cargo.toml
│       ├── src/
│       │   ├── lib.rs
│       │   ├── sac.rs
│       │   ├── replay_buffer.rs
│       │   └── python_bindings.rs
│       └── benches/
│
├── tests/                          # Test suite
│   ├── unit/                      # Unit tests
│   │   ├── test_agents/
│   │   ├── test_environments/
│   │   ├── test_perception/
│   │   └── test_utils/
│   ├── integration/               # Integration tests
│   │   ├── test_carla_integration.py
│   │   ├── test_ros2_integration.py
│   │   └── test_training_pipeline.py
│   ├── e2e/                       # End-to-end tests
│   │   └── test_full_training.py
│   └── fixtures/                  # Test fixtures
│       ├── mock_data/
│       └── test_configs/
│
├── scripts/                        # Executable scripts
│   ├── setup/                     # Setup scripts
│   │   ├── install_dependencies.sh
│   │   ├── setup_carla.sh
│   │   └── setup_ros2.sh
│   ├── training/                  # Training scripts
│   │   ├── train_sac.py
│   │   ├── train_with_curriculum.py
│   │   └── train_with_expert.py
│   ├── evaluation/                # Evaluation scripts
│   │   ├── evaluate_agent.py
│   │   └── benchmark.py
│   ├── data/                      # Data scripts
│   │   ├── collect_expert_data.py
│   │   └── preprocess_data.py
│   └── deployment/                # Deployment scripts
│       ├── deploy_model.py
│       └── serve_model.py
│
├── notebooks/                      # Jupyter notebooks
│   ├── exploratory/               # EDA notebooks
│   ├── experiments/               # Experiment notebooks
│   └── visualization/             # Visualization notebooks
│
├── data/                           # Data directory
│   ├── raw/                       # Raw data
│   ├── processed/                 # Processed data
│   ├── expert/                    # Expert demonstrations
│   ├── models/                    # Saved models
│   │   ├── checkpoints/
│   │   └── final/
│   └── logs/                      # Training logs
│       ├── tensorboard/
│       └── mlflow/
│
├── ros2_ws/                        # ROS2 workspace
│   └── src/
│       └── carla_rl_bridge/       # ROS2 package
│           ├── package.xml
│           ├── setup.py
│           ├── carla_rl_bridge/
│           │   ├── nodes/
│           │   ├── launch/
│           │   └── config/
│           └── test/
│
├── docker/                         # Docker files
│   ├── Dockerfile.training        # Training container
│   ├── Dockerfile.inference       # Inference container
│   ├── Dockerfile.ros2            # ROS2 container
│   ├── docker-compose.yml
│   └── .dockerignore
│
├── deployment/                     # Deployment configs
│   ├── kubernetes/
│   │   ├── deployment.yaml
│   │   └── service.yaml
│   └── terraform/
│
├── .vscode/                        # VSCode settings
│   ├── settings.json
│   ├── launch.json
│   └── tasks.json
│
├── .gitignore
├── .pre-commit-config.yaml        # Pre-commit hooks
├── .editorconfig
├── .flake8                        # Linting config
├── .pylintrc
├── mypy.ini                       # Type checking
│
├── pyproject.toml                 # Python project config
├── setup.py                       # Package setup
├── setup.cfg
├── requirements.txt               # Production dependencies
├── requirements-dev.txt           # Development dependencies
├── requirements-test.txt          # Test dependencies
│
├── Makefile                       # Build automation
├── README.md
├── CONTRIBUTING.md
├── LICENSE
├── CHANGELOG.md
└── VERSION
```

---

## 🎯 Design Principles

### 1. **Clean Architecture**
- **Core** - Domain logic (entities, use cases)
- **Application** - Application services
- **Infrastructure** - External dependencies
- **Interfaces** - Abstract interfaces

### 2. **Domain-Driven Design**
- Entities: Vehicle, Environment, Trajectory
- Value Objects: Position, Velocity, Action
- Aggregates: Training Session, Episode
- Repositories: Model storage, Data storage

### 3. **SOLID Principles**
- **S**ingle Responsibility
- **O**pen/Closed
- **L**iskov Substitution
- **I**nterface Segregation
- **D**ependency Inversion

### 4. **Separation of Concerns**
- Business logic ≠ Infrastructure
- Training ≠ Evaluation
- Data collection ≠ Processing

---

## 📦 Package Structure

### Main Package: `carla_rl`

```python
# src/carla_rl/__init__.py
from .agents import SACAgent, ExpertController
from .environments import CarlaEnv
from .application.training import Trainer
from .application.perception import PerceptionFusion

__version__ = "0.1.0"
__all__ = [
    "SACAgent",
    "ExpertController",
    "CarlaEnv",
    "Trainer",
    "PerceptionFusion"
]
```

### Installation:

```bash
# Development mode
pip install -e .

# Production
pip install .

# With extras
pip install -e ".[dev,test,ros2,rust]"
```

---

## 🔧 Configuration Management

### Hierarchical Config:

```
configs/
├── base.yaml              # Base configuration
├── environments/
│   ├── carla_dev.yaml    # Development
│   ├── carla_prod.yaml   # Production
│   └── carla_test.yaml   # Testing
└── models/
    ├── sac_default.yaml
    └── sac_optimized.yaml
```

### Config Loading:

```python
from carla_rl.utils import ConfigLoader

config = ConfigLoader.load(
    base="configs/base.yaml",
    environment="configs/environments/carla_dev.yaml",
    model="configs/models/sac_default.yaml"
)
```

---

## 🧪 Testing Structure

### Test Organization:

```python
# tests/unit/test_agents/test_sac_agent.py
import pytest
from carla_rl.agents import SACAgent

class TestSACAgent:
    def test_initialization(self):
        agent = SACAgent(obs_dim=10, action_dim=3)
        assert agent is not None
    
    def test_select_action(self):
        agent = SACAgent(obs_dim=10, action_dim=3)
        obs = np.random.randn(10)
        action = agent.select_action(obs)
        assert action.shape == (3,)
```

### Running Tests:

```bash
# All tests
pytest

# Unit tests only
pytest tests/unit/

# With coverage
pytest --cov=carla_rl --cov-report=html

# Specific test
pytest tests/unit/test_agents/test_sac_agent.py::TestSACAgent::test_initialization
```

---

## 🚀 CI/CD Pipeline

### GitHub Actions:

```yaml
# .github/workflows/ci.yml
name: CI

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      - name: Set up Python
        uses: actions/setup-python@v2
      - name: Install dependencies
        run: |
          pip install -r requirements-test.txt
      - name: Run tests
        run: pytest
      - name: Lint
        run: |
          flake8 src/
          mypy src/
```

---

## 🐳 Docker Structure

### Multi-stage Build:

```dockerfile
# docker/Dockerfile.training
FROM python:3.10-slim as base
WORKDIR /app
COPY requirements.txt .
RUN pip install -r requirements.txt

FROM base as training
COPY src/ ./src/
COPY configs/ ./configs/
COPY scripts/training/ ./scripts/
CMD ["python", "scripts/training/train_sac.py"]
```

### Docker Compose:

```yaml
# docker/docker-compose.yml
version: '3.8'
services:
  carla:
    image: carlasim/carla:0.9.16
    ports:
      - "2000:2000"
  
  training:
    build:
      context: ..
      dockerfile: docker/Dockerfile.training
    depends_on:
      - carla
    volumes:
      - ../data:/app/data
```

---

## 📝 Documentation Structure

### API Documentation:

```python
# src/carla_rl/agents/sac/agent.py
class SACAgent:
    """
    Soft Actor-Critic Agent
    
    Args:
        obs_dim: Observation dimension
        action_dim: Action dimension
        hidden_dim: Hidden layer dimension
        lr: Learning rate
        gamma: Discount factor
        tau: Target network update rate
    
    Example:
        >>> agent = SACAgent(obs_dim=10, action_dim=3)
        >>> action = agent.select_action(obs)
    """
```

### Sphinx Documentation:

```bash
# Generate docs
cd docs/
make html

# View docs
open _build/html/index.html
```

---

## 🔨 Build Automation

### Makefile:

```makefile
# Makefile
.PHONY: install test lint format clean

install:
	pip install -e ".[dev]"

test:
	pytest tests/ --cov=carla_rl

lint:
	flake8 src/
	mypy src/
	pylint src/

format:
	black src/ tests/
	isort src/ tests/

clean:
	find . -type d -name __pycache__ -exec rm -rf {} +
	find . -type f -name "*.pyc" -delete
	rm -rf .pytest_cache .mypy_cache .coverage htmlcov

docker-build:
	docker-compose -f docker/docker-compose.yml build

docker-up:
	docker-compose -f docker/docker-compose.yml up

docs:
	cd docs && make html
```

---

## 📊 Dependency Management

### pyproject.toml:

```toml
[build-system]
requires = ["setuptools>=45", "wheel", "setuptools_scm[toml]>=6.2"]
build-backend = "setuptools.build_meta"

[project]
name = "carla-rl"
version = "0.1.0"
description = "CARLA RL Training with ROS2 and Rust"
authors = [{name = "Your Name", email = "your.email@example.com"}]
license = {text = "MIT"}
requires-python = ">=3.8"

dependencies = [
    "numpy>=1.21.0",
    "torch>=2.0.0",
    "gymnasium>=0.28.0",
    "stable-baselines3>=2.0.0",
]

[project.optional-dependencies]
dev = [
    "pytest>=7.0.0",
    "black>=22.0.0",
    "flake8>=4.0.0",
    "mypy>=0.950",
]
ros2 = [
    "rclpy>=3.0.0",
]
rust = [
    "maturin>=0.14.0",
]

[tool.black]
line-length = 100
target-version = ['py38', 'py39', 'py310']

[tool.isort]
profile = "black"
line_length = 100

[tool.pytest.ini_options]
testpaths = ["tests"]
python_files = ["test_*.py"]
python_classes = ["Test*"]
python_functions = ["test_*"]
```

---

## 🎉 สรุป

**โครงสร้างที่ดีต้องมี:**

✅ **Separation of Concerns** - แยก business logic, infrastructure, application  
✅ **Clean Architecture** - Core → Application → Infrastructure  
✅ **Domain-Driven Design** - Entities, Use Cases, Repositories  
✅ **Testability** - Unit, Integration, E2E tests  
✅ **Documentation** - API docs, guides, examples  
✅ **CI/CD** - Automated testing, linting, deployment  
✅ **Containerization** - Docker, docker-compose  
✅ **Dependency Management** - pyproject.toml, requirements.txt  
✅ **Build Automation** - Makefile, scripts  

**โครงสร้างนี้รองรับ:**
- Scalability
- Maintainability
- Testability
- Collaboration
- Production deployment

ใช้โครงสร้างนี้เป็น template สำหรับจัดระเบียบโปรเจคครับ! 🏗️✨
