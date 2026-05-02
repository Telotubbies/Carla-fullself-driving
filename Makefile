# Makefile for CARLA SAC ROS2 Training Project
# ใช้สำหรับ build automation และ development workflow

.PHONY: help install install-dev install-all test lint format clean \
        carla-start carla-stop train-sac train-full demo-perception \
        ros2-build ros2-launch mlflow-ui tensorboard rust-build \
        docker-build docker-up docker-down docs

# Default target
.DEFAULT_GOAL := help

# Colors for output
BLUE := \033[0;34m
GREEN := \033[0;32m
YELLOW := \033[1;33m
RED := \033[0;31m
NC := \033[0m # No Color

# Project paths
PROJECT_ROOT := $(shell pwd)
CARLA_PATH := /home/supawich/Desktop/CARLA_0.9.16
ROS2_WS := $(PROJECT_ROOT)/ros2_ws
RUST_RL := $(PROJECT_ROOT)/rust_rl

##@ General

help: ## Display this help message
	@echo "$(BLUE)CARLA SAC ROS2 Training - Makefile Commands$(NC)"
	@echo ""
	@awk 'BEGIN {FS = ":.*##"; printf "Usage:\n  make $(GREEN)<target>$(NC)\n"} /^[a-zA-Z_-]+:.*?##/ { printf "  $(GREEN)%-20s$(NC) %s\n", $$1, $$2 } /^##@/ { printf "\n$(BLUE)%s$(NC)\n", substr($$0, 5) } ' $(MAKEFILE_LIST)

##@ Installation

install: ## Install production dependencies
	@echo "$(BLUE)Installing production dependencies...$(NC)"
	pip install -e .

install-dev: ## Install development dependencies
	@echo "$(BLUE)Installing development dependencies...$(NC)"
	pip install -e ".[dev,test]"

install-all: ## Install all dependencies (including ROS2, MLflow, Ray, Rust)
	@echo "$(BLUE)Installing all dependencies...$(NC)"
	pip install -e ".[all]"

##@ Development

test: ## Run all tests
	@echo "$(BLUE)Running tests...$(NC)"
	pytest tests/ -v --cov=src --cov-report=term-missing --cov-report=html

test-unit: ## Run unit tests only
	@echo "$(BLUE)Running unit tests...$(NC)"
	pytest tests/unit/ -v

test-integration: ## Run integration tests
	@echo "$(BLUE)Running integration tests...$(NC)"
	pytest tests/integration/ -v -m integration

test-e2e: ## Run end-to-end tests
	@echo "$(BLUE)Running E2E tests...$(NC)"
	pytest tests/e2e/ -v -m e2e

lint: ## Run linters (flake8, mypy, pylint)
	@echo "$(BLUE)Running linters...$(NC)"
	flake8 src/ tests/ --max-line-length=100
	mypy src/
	pylint src/ --max-line-length=100

format: ## Format code with black and isort
	@echo "$(BLUE)Formatting code...$(NC)"
	black src/ tests/ scripts/ --line-length=100
	isort src/ tests/ scripts/ --profile=black

format-check: ## Check code formatting without modifying
	@echo "$(BLUE)Checking code format...$(NC)"
	black src/ tests/ scripts/ --check --line-length=100
	isort src/ tests/ scripts/ --check-only --profile=black

clean: ## Clean up cache and temporary files
	@echo "$(BLUE)Cleaning up...$(NC)"
	find . -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null || true
	find . -type f -name "*.pyc" -delete
	find . -type f -name "*.pyo" -delete
	find . -type f -name "*.egg-info" -exec rm -rf {} + 2>/dev/null || true
	rm -rf .pytest_cache .mypy_cache .coverage htmlcov build dist
	@echo "$(GREEN)Cleanup complete!$(NC)"

##@ CARLA

carla-start: ## Start CARLA server
	@echo "$(BLUE)Starting CARLA server...$(NC)"
	cd $(CARLA_PATH) && ./CarlaUE4.sh -RenderOffScreen -carla-rpc-port=2000 &
	@echo "$(GREEN)CARLA server started on port 2000$(NC)"

carla-stop: ## Stop CARLA server
	@echo "$(BLUE)Stopping CARLA server...$(NC)"
	pkill -9 -f CarlaUE4 || true
	@echo "$(GREEN)CARLA server stopped$(NC)"

carla-restart: carla-stop carla-start ## Restart CARLA server

##@ Training

train-sac: ## Train SAC agent (simple version)
	@echo "$(BLUE)Starting SAC training...$(NC)"
	python train_sac_simple.py

train-full: ## Train with full system (Guidelines + Curriculum + Perception)
	@echo "$(BLUE)Starting full system training...$(NC)"
	python scripts/train_with_guidelines.py --episodes 100 --expert

train-curriculum: ## Train with curriculum learning
	@echo "$(BLUE)Starting curriculum training...$(NC)"
	python scripts/train_with_guidelines.py --episodes 1000

##@ Demo & Visualization

demo-perception: ## Run perception system demo
	@echo "$(BLUE)Running perception demo...$(NC)"
	python scripts/demo_perception.py

demo-drive: ## Run simple drive demo
	@echo "$(BLUE)Running drive demo...$(NC)"
	python demo_simple_drive.py

demo-autopilot: ## Run autopilot demo
	@echo "$(BLUE)Running autopilot demo...$(NC)"
	python demo_autopilot.py

visualize: ## Run advanced visualization
	@echo "$(BLUE)Running visualization...$(NC)"
	python visualize_advanced.py

##@ ROS2

ros2-build: ## Build ROS2 workspace
	@echo "$(BLUE)Building ROS2 workspace...$(NC)"
	cd $(ROS2_WS) && colcon build --symlink-install
	@echo "$(GREEN)ROS2 workspace built!$(NC)"

ros2-source: ## Source ROS2 workspace
	@echo "$(BLUE)Source ROS2 workspace:$(NC)"
	@echo "source $(ROS2_WS)/install/setup.bash"

ros2-launch: ## Launch ROS2 CARLA bridge
	@echo "$(BLUE)Launching ROS2 CARLA bridge...$(NC)"
	cd $(ROS2_WS) && source install/setup.bash && \
	ros2 launch carla_sac_bridge carla_system.launch.py

ros2-monitor: ## Launch ROS2 camera monitor
	@echo "$(BLUE)Launching camera monitor...$(NC)"
	cd $(ROS2_WS) && source install/setup.bash && \
	ros2 run carla_sac_bridge camera_monitor_node

##@ Tracking & Monitoring

mlflow-ui: ## Start MLflow UI
	@echo "$(BLUE)Starting MLflow UI...$(NC)"
	mlflow ui --port 5000 --backend-store-uri ./mlruns &
	@echo "$(GREEN)MLflow UI: http://localhost:5000$(NC)"

tensorboard: ## Start Tensorboard
	@echo "$(BLUE)Starting Tensorboard...$(NC)"
	tensorboard --logdir=data/tensorboard --port=6006 &
	@echo "$(GREEN)Tensorboard: http://localhost:6006$(NC)"

stop-tracking: ## Stop MLflow and Tensorboard
	@echo "$(BLUE)Stopping tracking services...$(NC)"
	pkill -f "mlflow ui" || true
	pkill -f "tensorboard" || true
	@echo "$(GREEN)Tracking services stopped$(NC)"

##@ Rust

rust-build: ## Build Rust RL modules
	@echo "$(BLUE)Building Rust modules...$(NC)"
	cd $(RUST_RL) && cargo build --release
	@echo "$(GREEN)Rust modules built!$(NC)"

rust-test: ## Test Rust modules
	@echo "$(BLUE)Testing Rust modules...$(NC)"
	cd $(RUST_RL) && cargo test

rust-bench: ## Benchmark Rust modules
	@echo "$(BLUE)Running Rust benchmarks...$(NC)"
	cd $(RUST_RL) && cargo bench

rust-python: ## Build Python bindings for Rust
	@echo "$(BLUE)Building Python bindings...$(NC)"
	cd $(RUST_RL) && maturin develop --release
	@echo "$(GREEN)Python bindings ready!$(NC)"

##@ Docker

docker-build: ## Build Docker images
	@echo "$(BLUE)Building Docker images...$(NC)"
	docker-compose -f docker/docker-compose.yml build

docker-up: ## Start Docker containers
	@echo "$(BLUE)Starting Docker containers...$(NC)"
	docker-compose -f docker/docker-compose.yml up -d
	@echo "$(GREEN)Containers started!$(NC)"

docker-down: ## Stop Docker containers
	@echo "$(BLUE)Stopping Docker containers...$(NC)"
	docker-compose -f docker/docker-compose.yml down
	@echo "$(GREEN)Containers stopped!$(NC)"

docker-logs: ## View Docker logs
	docker-compose -f docker/docker-compose.yml logs -f

##@ Documentation

docs: ## Build documentation
	@echo "$(BLUE)Building documentation...$(NC)"
	cd docs && make html
	@echo "$(GREEN)Documentation built! Open docs/_build/html/index.html$(NC)"

docs-serve: ## Serve documentation locally
	@echo "$(BLUE)Serving documentation...$(NC)"
	cd docs/_build/html && python -m http.server 8000

##@ Complete Workflows

setup: install-dev ros2-build rust-python ## Complete setup (install + build)
	@echo "$(GREEN)Setup complete!$(NC)"

full-train: carla-start mlflow-ui tensorboard train-full ## Full training workflow
	@echo "$(GREEN)Full training started!$(NC)"

demo-all: carla-start demo-perception ## Run all demos
	@echo "$(GREEN)Demos complete!$(NC)"

ci: format-check lint test ## Run CI checks (format, lint, test)
	@echo "$(GREEN)CI checks passed!$(NC)"

##@ Quick Commands

quick-train: ## Quick training (100 episodes)
	@echo "$(BLUE)Quick training...$(NC)"
	python scripts/train_with_guidelines.py --episodes 100

quick-test: ## Quick test (unit tests only)
	@echo "$(BLUE)Quick test...$(NC)"
	pytest tests/unit/ -v --tb=short

status: ## Show system status
	@echo "$(BLUE)System Status:$(NC)"
	@echo "CARLA Server: $$(pgrep -f CarlaUE4 > /dev/null && echo '$(GREEN)Running$(NC)' || echo '$(RED)Stopped$(NC)')"
	@echo "MLflow UI: $$(pgrep -f 'mlflow ui' > /dev/null && echo '$(GREEN)Running$(NC)' || echo '$(RED)Stopped$(NC)')"
	@echo "Tensorboard: $$(pgrep -f tensorboard > /dev/null && echo '$(GREEN)Running$(NC)' || echo '$(RED)Stopped$(NC)')"
	@echo "Python venv: $$([ -n "$$VIRTUAL_ENV" ] && echo '$(GREEN)Active$(NC)' || echo '$(YELLOW)Inactive$(NC)')"
