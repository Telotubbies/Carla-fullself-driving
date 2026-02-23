"""
Central Orchestrator for CARLA LSTM-MPC Pipeline.

Manages phase-based workflow execution with state tracking and error recovery.
"""

import logging
import json
import time
from pathlib import Path
from typing import Dict, List, Optional, Callable, Any
from enum import Enum
from datetime import datetime

from .exceptions import ProjectError, CARLAConnectionError
from .config import ConfigManager

logger = logging.getLogger(__name__)


class PhaseStatus(Enum):
    """Phase execution status."""
    PENDING = "pending"
    RUNNING = "running"
    COMPLETED = "completed"
    FAILED = "failed"
    SKIPPED = "skipped"


class Phase:
    """Represents a single phase in the pipeline."""
    
    def __init__(
        self,
        name: str,
        description: str,
        execute_fn: Callable[[], bool],
        dependencies: List[str] = None,
        skip_if: Callable[[], bool] = None
    ):
        """
        Initialize phase.
        
        Args:
            name: Phase name (unique identifier)
            description: Human-readable description
            execute_fn: Function to execute (returns True if success)
            dependencies: List of phase names that must complete first
            skip_if: Function that returns True if phase should be skipped
        """
        self.name = name
        self.description = description
        self.execute_fn = execute_fn
        self.dependencies = dependencies or []
        self.skip_if = skip_if
        self.status = PhaseStatus.PENDING
        self.start_time: Optional[float] = None
        self.end_time: Optional[float] = None
        self.error: Optional[str] = None
        self.output: Optional[Dict[str, Any]] = None
    
    def can_run(self, completed_phases: set) -> bool:
        """Check if phase can run (dependencies satisfied)."""
        return all(dep in completed_phases for dep in self.dependencies)
    
    def should_skip(self) -> bool:
        """Check if phase should be skipped."""
        if self.skip_if:
            try:
                return self.skip_if()
            except Exception as e:
                logger.warning(f"Error in skip_if for {self.name}: {e}")
                return False
        return False
    
    def execute(self) -> bool:
        """Execute phase."""
        self.status = PhaseStatus.RUNNING
        self.start_time = time.time()
        
        try:
            logger.info(f"▶ Starting phase: {self.name}")
            logger.info(f"   {self.description}")
            
            result = self.execute_fn()
            
            if result:
                self.status = PhaseStatus.COMPLETED
                self.end_time = time.time()
                duration = self.end_time - self.start_time
                logger.info(f"✅ Phase {self.name} completed in {duration:.1f}s")
                return True
            else:
                self.status = PhaseStatus.FAILED
                self.end_time = time.time()
                self.error = "Execution returned False"
                logger.error(f"❌ Phase {self.name} failed: {self.error}")
                return False
                
        except Exception as e:
            self.status = PhaseStatus.FAILED
            self.end_time = time.time()
            self.error = str(e)
            logger.error(f"❌ Phase {self.name} failed with exception: {e}", exc_info=True)
            return False


class PipelineOrchestrator:
    """
    Central orchestrator for managing pipeline phases.
    
    Features:
    - Phase-based execution
    - Dependency management
    - State persistence
    - Error recovery
    - Progress tracking
    """
    
    def __init__(self, config_path: str = "config.yaml", state_file: str = "logs/pipeline_state.json"):
        """
        Initialize orchestrator.
        
        Args:
            config_path: Path to configuration file
            state_file: Path to state persistence file
        """
        self.config_manager = ConfigManager(config_path)
        self.state_file = Path(state_file)
        self.state_file.parent.mkdir(parents=True, exist_ok=True)
        
        self.phases: Dict[str, Phase] = {}
        self.execution_order: List[str] = []
        self.state: Dict[str, Any] = self._load_state()
        
        logger.info("✅ PipelineOrchestrator initialized")
    
    def _load_state(self) -> Dict[str, Any]:
        """Load persisted state."""
        if self.state_file.exists():
            try:
                with open(self.state_file, 'r') as f:
                    return json.load(f)
            except Exception as e:
                logger.warning(f"Failed to load state: {e}")
        
        return {
            'phases': {},
            'last_updated': None,
            'current_phase': None
        }
    
    def _save_state(self):
        """Save current state."""
        try:
            self.state['last_updated'] = datetime.now().isoformat()
            with open(self.state_file, 'w') as f:
                json.dump(self.state, f, indent=2)
        except Exception as e:
            logger.warning(f"Failed to save state: {e}")
    
    def register_phase(
        self,
        name: str,
        description: str,
        execute_fn: Callable[[], bool],
        dependencies: List[str] = None,
        skip_if: Callable[[], bool] = None
    ):
        """
        Register a phase.
        
        Args:
            name: Phase name (unique)
            description: Human-readable description
            execute_fn: Function to execute
            dependencies: Required phases
            skip_if: Skip condition function
        """
        phase = Phase(name, description, execute_fn, dependencies, skip_if)
        self.phases[name] = phase
        logger.debug(f"Registered phase: {name}")
    
    def _resolve_order(self) -> List[str]:
        """Resolve execution order based on dependencies."""
        # Topological sort
        in_degree = {name: len(phase.dependencies) for name, phase in self.phases.items()}
        queue = [name for name, degree in in_degree.items() if degree == 0]
        order = []
        
        while queue:
            name = queue.pop(0)
            order.append(name)
            
            # Update dependencies
            for other_name, phase in self.phases.items():
                if name in phase.dependencies:
                    in_degree[other_name] -= 1
                    if in_degree[other_name] == 0:
                        queue.append(other_name)
        
        # Check for cycles
        if len(order) != len(self.phases):
            raise ProjectError("Circular dependency detected in phases")
        
        return order
    
    def execute(
        self,
        phases: Optional[List[str]] = None,
        resume: bool = True,
        stop_on_error: bool = True
    ) -> bool:
        """
        Execute pipeline phases.
        
        Args:
            phases: Specific phases to run (None = all)
            resume: Resume from last completed phase
            stop_on_error: Stop on first error
        
        Returns:
            True if all phases completed successfully
        """
        logger.info("🚀 Starting pipeline execution")
        
        # Resolve execution order
        self.execution_order = self._resolve_order()
        
        # Filter phases if specified
        if phases:
            self.execution_order = [p for p in self.execution_order if p in phases]
        
        # Track completed phases
        completed = set()
        if resume:
            # Load completed phases from state
            phase_states = self.state.get('phases', {})
            for name, phase_state in phase_states.items():
                if phase_state.get('status') == 'completed':
                    completed.add(name)
                    if name in self.phases:
                        self.phases[name].status = PhaseStatus.COMPLETED
        
        # Execute phases
        for phase_name in self.execution_order:
            phase = self.phases[phase_name]
            
            # Check if already completed
            if phase_name in completed:
                logger.info(f"⏭️  Skipping {phase_name} (already completed)")
                continue
            
            # Check dependencies
            if not phase.can_run(completed):
                logger.warning(f"⚠️  Cannot run {phase_name}: dependencies not met")
                if stop_on_error:
                    return False
                continue
            
            # Check skip condition
            if phase.should_skip():
                phase.status = PhaseStatus.SKIPPED
                logger.info(f"⏭️  Skipping {phase_name} (skip condition met)")
                continue
            
            # Execute phase
            self.state['current_phase'] = phase_name
            self._save_state()
            
            success = phase.execute()
            
            # Update state
            self.state['phases'][phase_name] = {
                'status': phase.status.value,
                'start_time': phase.start_time,
                'end_time': phase.end_time,
                'error': phase.error,
                'output': phase.output
            }
            self._save_state()
            
            if success:
                completed.add(phase_name)
            else:
                if stop_on_error:
                    logger.error(f"❌ Pipeline stopped at phase: {phase_name}")
                    return False
        
        logger.info("✅ Pipeline execution complete")
        return True
    
    def get_status(self) -> Dict[str, Any]:
        """Get current pipeline status."""
        return {
            'phases': {
                name: {
                    'status': phase.status.value,
                    'description': phase.description,
                    'start_time': phase.start_time,
                    'end_time': phase.end_time,
                    'duration': (phase.end_time - phase.start_time) if phase.end_time and phase.start_time else None,
                    'error': phase.error
                }
                for name, phase in self.phases.items()
            },
            'current_phase': self.state.get('current_phase'),
            'last_updated': self.state.get('last_updated')
        }
    
    def reset_phase(self, phase_name: str):
        """Reset a phase to allow re-execution."""
        if phase_name in self.phases:
            self.phases[phase_name].status = PhaseStatus.PENDING
            self.phases[phase_name].start_time = None
            self.phases[phase_name].end_time = None
            self.phases[phase_name].error = None
            
            if phase_name in self.state.get('phases', {}):
                del self.state['phases'][phase_name]
            self._save_state()
            logger.info(f"🔄 Reset phase: {phase_name}")
    
    def reset_all(self):
        """Reset all phases."""
        for phase in self.phases.values():
            phase.status = PhaseStatus.PENDING
            phase.start_time = None
            phase.end_time = None
            phase.error = None
        
        self.state['phases'] = {}
        self.state['current_phase'] = None
        self._save_state()
        logger.info("🔄 Reset all phases")

