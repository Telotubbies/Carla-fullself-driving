"""
Refactored autonomous driving system using factories and dependency injection.
"""

import logging
import time
import numpy as np
from typing import Dict, Any, Optional, Tuple
from pathlib import Path
from datetime import datetime
import csv

from .interfaces import IPerceptionModule, ITemporalModule, IControlModule, IVisualizationModule
from .factories import PerceptionFactory, TemporalFactory, ControlFactory, VisualizationFactory
from .config import ConfigManager
from .exceptions import (
    ProjectError, CARLAConnectionError, ModelLoadError, 
    DataValidationError, ControlError, ConfigurationError
)
from .validators import (
    ImageValidator, StateValidator, FeatureValidator, 
    PredictionValidator, ControlValidator
)

logger = logging.getLogger(__name__)


class AutonomousDrivingSystem:
    """
    Main autonomous driving system with dependency injection.
    
    Uses factories to create components and implements proper error handling.
    """
    
    def __init__(
        self,
        config_manager: ConfigManager,
        perception: Optional[IPerceptionModule] = None,
        temporal: Optional[ITemporalModule] = None,
        control: Optional[IControlModule] = None,
        visualization: Optional[IVisualizationModule] = None
    ):
        """
        Initialize autonomous driving system.
        
        Args:
            config_manager: Configuration manager instance
            perception: Optional perception module (created if None)
            temporal: Optional temporal module (created if None)
            control: Optional control module (created if None)
            visualization: Optional visualization module (created if None)
        
        Raises:
            ConfigurationError: If configuration is invalid
            ModelLoadError: If model loading fails
        """
        self.config_manager = config_manager
        self.config = config_manager._config
        
        # Initialize components using factories (dependency injection)
        logger.info("Initializing system components...")
        
        try:
            # Perception module
            if perception is None:
                perception_config = config_manager.get_section('perception')
                self.perception = PerceptionFactory.create(perception_config)
            else:
                self.perception = perception
            logger.info("✅ Perception module initialized")
            
            # Temporal module
            if temporal is None:
                temporal_config = config_manager.get_section('temporal')
                self.temporal = TemporalFactory.create(temporal_config)
            else:
                self.temporal = temporal
            logger.info("✅ Temporal module initialized")
            
            # Control module
            if control is None:
                self.control = ControlFactory.create(self.config)
            else:
                self.control = control
            logger.info("✅ Control module initialized")
            
            # Visualization module
            if visualization is None:
                viz_config = config_manager.get_section('visualization')
                self.visualization = VisualizationFactory.create(viz_config)
            else:
                self.visualization = visualization
            logger.info("✅ Visualization module initialized")
            
        except (ConfigurationError, ModelLoadError) as e:
            logger.error(f"Failed to initialize components: {e}")
            raise
        
        # CARLA client (will be initialized separately)
        self.carla_client = None
        self.camera = None
        self.lane_detector = None
        
        # Sequence buffer for temporal module
        import sys
        from pathlib import Path
        project_root = Path(__file__).parent.parent
        sys.path.insert(0, str(project_root))
        from temporal import SequenceBuffer
        self.sequence_buffer = SequenceBuffer(
            sequence_length=self.temporal.get_sequence_length(),
            feature_dim=self.perception.get_feature_dim()
        )
        
        # State tracking
        self.running = False
        self.step_count = 0
        self.trajectory_log = []
        self.control_log = []
        self.prediction_log = []
        
        # Validation error counters
        self.validation_errors = {
            'image': 0,
            'state': 0,
            'features': 0,
            'prediction': 0,
            'control': 0
        }
        
        logger.info("✅ System initialization complete")
    
    def set_carla_client(self, carla_client) -> None:
        """Set CARLA client (dependency injection)."""
        self.carla_client = carla_client
    
    def set_camera(self, camera) -> None:
        """Set camera sensor (dependency injection)."""
        self.camera = camera
    
    def set_lane_detector(self, lane_detector) -> None:
        """Set lane detector (dependency injection)."""
        self.lane_detector = lane_detector
    
    def run_inference_step(self) -> Tuple[bool, Optional[Dict[str, Any]]]:
        """
        Run single inference step.
        
        Returns:
            (should_continue, step_info) where step_info contains step data
        """
        if not self.running:
            return False, None
        
        try:
            # Tick world (synchronous mode)
            if self.carla_client:
                self.carla_client.tick()
            
            # Get camera image
            if not self.camera:
                return False, None
            
            image = self.camera.get_image()
            if image is None:
                return True, None
            
            # Validate image
            try:
                ImageValidator.validate(image)
            except DataValidationError as e:
                self.validation_errors['image'] += 1
                if self.validation_errors['image'] % 100 == 0:
                    logger.warning(f"Image validation failed {self.validation_errors['image']} times: {e}")
                return True, None
            
            # Get vehicle state
            if not self.carla_client:
                return False, None
            
            vehicle_state = self.carla_client.get_vehicle_state()
            try:
                StateValidator.validate(vehicle_state)
            except DataValidationError as e:
                self.validation_errors['state'] += 1
                if self.validation_errors['state'] % 100 == 0:
                    logger.warning(f"State validation failed {self.validation_errors['state']} times: {e}")
                return True, None
            
            # Encode image to features
            try:
                features = self.perception.encode(image)
                FeatureValidator.validate(features, self.perception.get_feature_dim())
                self.sequence_buffer.add(features)
            except (DataValidationError, ModelLoadError) as e:
                self.validation_errors['features'] += 1
                if self.validation_errors['features'] % 100 == 0:
                    logger.warning(f"Feature encoding failed {self.validation_errors['features']} times: {e}")
                # Use previous feature if available
                if len(self.sequence_buffer.buffer) == 0:
                    return True, None
            
            # Predict future state
            predicted_state = None
            if self.sequence_buffer.is_ready():
                sequence = self.sequence_buffer.get_sequence()
                if sequence is not None:
                    try:
                        predicted_state = self.temporal.predict(sequence)
                        PredictionValidator.validate(predicted_state)
                    except (DataValidationError, ModelLoadError) as e:
                        self.validation_errors['prediction'] += 1
                        logger.warning(f"Prediction failed: {e}")
                        # Use simple forward prediction as fallback
                        predicted_state = np.array([
                            vehicle_state['x'],
                            vehicle_state['y'],
                            vehicle_state['yaw'],
                            vehicle_state['velocity']
                        ])
            
            # Get waypoints from CARLA for lane following
            waypoints = None
            if self.carla_client:
                try:
                    waypoints = self.carla_client.get_waypoints(distance=3.0, num_waypoints=50)
                except Exception as e:
                    logger.debug(f"Could not get waypoints: {e}")
            
            # Get lane info from lane detector if available
            lane_info = None
            if self.lane_detector:
                try:
                    # Get camera transform for proper projection
                    camera_transform = None
                    if self.camera and hasattr(self.camera, 'sensor'):
                        camera_transform = self.camera.sensor.get_transform()
                    elif self.camera and hasattr(self.camera, 'config'):
                        # Construct from config
                        import carla
                        camera_transform = carla.Transform(
                            carla.Location(
                                x=self.camera.config.get('location', {}).get('x', 2.0),
                                y=self.camera.config.get('location', {}).get('y', 0.0),
                                z=self.camera.config.get('location', {}).get('z', 1.4)
                            ),
                            carla.Rotation(
                                pitch=self.camera.config.get('rotation', {}).get('pitch', 0.0),
                                yaw=self.camera.config.get('rotation', {}).get('yaw', 0.0),
                                roll=self.camera.config.get('rotation', {}).get('roll', 0.0)
                            )
                        )
                    
                    # detect_lanes returns (lane_mask, lane_features) tuple
                    if self.carla_client and self.carla_client.world and self.carla_client.vehicle:
                        lane_result = self.lane_detector.detect_lanes(
                            image, 
                            world=self.carla_client.world,
                            vehicle=self.carla_client.vehicle
                        )
                    else:
                        lane_result = self.lane_detector.detect_lanes(image)
                    
                    if lane_result is not None:
                        if isinstance(lane_result, tuple) and len(lane_result) == 2:
                            lane_mask, lane_features = lane_result
                            # Convert to dict format for visualization
                            lane_info = {
                                'lane_mask': lane_mask,
                                'lane_features': lane_features,
                                'center_line': None,
                                'left_lane': None,
                                'right_lane': None
                            }
                        elif isinstance(lane_result, dict):
                            lane_info = lane_result
                        else:
                            logger.warning(f"Unexpected lane_info format: {type(lane_result)}")
                except Exception as e:
                    logger.debug(f"Could not detect lanes: {e}")
            
            # Update MPC with waypoints and lane info
            if hasattr(self.control, 'set_waypoints'):
                self.control.set_waypoints(waypoints)
            if hasattr(self.control, 'set_lane_info'):
                self.control.set_lane_info(lane_info)
            
            # Compute control
            try:
                steering, throttle, brake = self.control.compute_control(
                    vehicle_state,
                    None  # Reference trajectory will be set by MPC internally
                )
                ControlValidator.validate(steering, throttle, brake)
            except (ControlError, DataValidationError) as e:
                self.validation_errors['control'] += 1
                logger.warning(f"Control computation failed: {e}")
                # Safe fallback
                steering, throttle, brake = 0.0, 0.0, 1.0
            
            # Apply control
            if self.carla_client:
                import carla
                control = carla.VehicleControl(
                    steer=steering,
                    throttle=throttle,
                    brake=brake
                )
                self.carla_client.apply_control(control)
            
            # Update visualization
            predicted_traj = None
            if predicted_state is not None:
                predicted_traj = self._state_to_trajectory(predicted_state, vehicle_state)
            
            mpc_horizon = None
            if hasattr(self.control, 'get_predicted_trajectory'):
                mpc_horizon = self.control.get_predicted_trajectory()
            
            # Prepare lane info with waypoints for visualization
            if lane_info is None:
                lane_info = {}
            if waypoints is not None:
                lane_info['waypoints'] = waypoints
            
            continue_running = self.visualization.update(
                image,
                vehicle_state,
                predicted_traj,
                mpc_horizon,
                lane_info
            )
            
            # Log step
            step_info = {
                'step': self.step_count,
                'vehicle_state': vehicle_state,
                'predicted_state': predicted_state,
                'control': (steering, throttle, brake)
            }
            self._log_step(vehicle_state, predicted_state, (steering, throttle, brake))
            
            self.step_count += 1
            return continue_running, step_info
            
        except Exception as e:
            logger.error(f"Error in inference step: {e}", exc_info=True)
            return True, None  # Continue despite error
    
    def _state_to_trajectory(self, state: np.ndarray, current_vehicle_state: Dict[str, Any]) -> np.ndarray:
        """Convert predicted state to reference trajectory for MPC."""
        if not hasattr(self.control, 'N'):
            return None
        
        N = self.control.N
        dt = getattr(self.control, 'dt', 0.05)
        
        pred_x = state[0] if len(state) > 0 else current_vehicle_state['x']
        pred_y = state[1] if len(state) > 1 else current_vehicle_state['y']
        pred_yaw = np.deg2rad(state[2]) if len(state) > 2 else np.deg2rad(current_vehicle_state['yaw'])
        pred_v = state[3] if len(state) > 3 else current_vehicle_state['velocity']
        
        current_x = current_vehicle_state['x']
        current_y = current_vehicle_state['y']
        current_yaw = np.deg2rad(current_vehicle_state['yaw'])
        current_v = current_vehicle_state['velocity']
        
        ref_traj = np.zeros((N + 1, 4))
        for i in range(N + 1):
            alpha = i / (N + 1)
            ref_traj[i, 0] = current_x + alpha * (pred_x - current_x)
            ref_traj[i, 1] = current_y + alpha * (pred_y - current_y)
            ref_traj[i, 2] = current_yaw + alpha * (pred_yaw - current_yaw)
            ref_traj[i, 3] = current_v + alpha * (pred_v - current_v)
        
        return ref_traj
    
    def _log_step(
        self,
        vehicle_state: Dict[str, Any],
        predicted_state: Optional[np.ndarray],
        control: Tuple[float, float, float]
    ) -> None:
        """Log step data."""
        if self.config.get('logging', {}).get('save_trajectory', False):
            self.trajectory_log.append({
                'step': self.step_count,
                'x': vehicle_state['x'],
                'y': vehicle_state['y'],
                'yaw': vehicle_state['yaw'],
                'velocity': vehicle_state['velocity']
            })
        
        if self.config.get('logging', {}).get('save_controls', False):
            self.control_log.append({
                'step': self.step_count,
                'steering': control[0],
                'throttle': control[1],
                'brake': control[2]
            })
        
        if self.config.get('logging', {}).get('save_predictions', False) and predicted_state is not None:
            self.prediction_log.append({
                'step': self.step_count,
                'pred_x': predicted_state[0] if len(predicted_state) > 0 else 0,
                'pred_y': predicted_state[1] if len(predicted_state) > 1 else 0,
                'pred_yaw': predicted_state[2] if len(predicted_state) > 2 else 0,
                'pred_velocity': predicted_state[3] if len(predicted_state) > 3 else 0
            })
    
    def cleanup(self) -> None:
        """Clean up resources."""
        logger.info("Cleaning up resources...")
        self.running = False
        
        # Log validation summary
        total_errors = sum(self.validation_errors.values())
        if total_errors > 0:
            logger.warning("Validation Summary:")
            for key, count in self.validation_errors.items():
                if count > 0:
                    logger.warning(f"  {key}: {count} errors")
        else:
            logger.info("✅ No validation errors detected")
        
        # Close visualization
        if self.visualization:
            self.visualization.close()
        
        logger.info("✅ Cleanup complete")

