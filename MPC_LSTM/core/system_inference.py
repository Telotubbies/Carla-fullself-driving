"""
Inference system module - handles autonomous driving control loop.

Extracted from main.py for better organization and separation of concerns.
"""

import logging
import time
import numpy as np
import carla
from typing import Dict, Any, Optional, Tuple
from pathlib import Path

from core.validators import (
    ImageValidator, StateValidator, FeatureValidator,
    PredictionValidator, ControlValidator
)
from core.exceptions import DataValidationError
from safety import SafetyOverride

logger = logging.getLogger(__name__)


class InferenceSystem:
    """Handles inference control loop for autonomous driving."""
    
    def __init__(self, system):
        """
        Initialize inference system.
        
        Args:
            system: Parent AutonomousDrivingSystem instance
        """
        self.system = system
        self.config = system.config
        self.running = False
        self.step_count = 0
        
        # Initialize safety override system
        self.safety_override = SafetyOverride(self.config.get('safety', {}))
        
        # Validation counters
        self.validation_errors = {
            'image': 0,
            'state': 0,
            'features': 0,
            'prediction': 0,
            'control': 0
        }
        
        logger.info("✅ InferenceSystem initialized")
    
    def run(self) -> None:
        """Run inference control loop."""
        logger.info("🚗 Starting autonomous driving inference...")
        self.running = True
        self.step_count = 0
        
        try:
            while self.running:
                # Tick world (synchronous mode)
                self.system.carla_client.tick()
                
                # Get camera image
                image = self._get_and_validate_image()
                if image is None:
                    continue
                
                # PHASE 2: UNet Segmentation (STEP 2.5 - SIM TEST)
                # Apply UNet to get segmentation mask
                segmentation_mask = None
                if self.system.lane_detector is not None and self.system.lane_detector.model is not None:
                    try:
                        # Use UNet for segmentation
                        lane_mask, _, _ = self.system.lane_detector.detect_lanes(image)
                        segmentation_mask = lane_mask
                        logger.debug(f"UNet segmentation applied: mask shape {segmentation_mask.shape}")
                    except Exception as e:
                        logger.warning(f"UNet segmentation failed: {e}, using original image")
                        segmentation_mask = None
                
                # Detect lanes (for MPC waypoints)
                lane_info, waypoints = self._detect_lanes(image)
                
                # Update MPC with lane info
                if lane_info is not None:
                    self.system.mpc_controller.set_lane_info(lane_info)
                if waypoints is not None:
                    self.system.mpc_controller.set_waypoints(waypoints)
                
                # Get vehicle state
                vehicle_state = self._get_and_validate_state()
                if vehicle_state is None:
                    continue
                
                # Encode image to features
                # Use segmentation mask if available, otherwise use original image
                # According to MASTER FLOW: UNet → ResNet → LSTM → MPC
                # We can either:
                # 1. Use segmentation mask as input to ResNet (if ResNet accepts grayscale)
                # 2. Concatenate mask with RGB
                # 3. Use mask to weight/guide ResNet features
                # For now, we'll use original image but store mask for future use
                features = self._encode_image(image, segmentation_mask)
                if features is None:
                    continue
                
                # Predict future state using LSTM
                predicted_state = self._predict_state(features, vehicle_state)
                
                # Compute control using MPC
                steering, throttle, brake = self._compute_control(
                    vehicle_state, predicted_state
                )
                
                # Apply safety overrides
                steering, throttle, brake = self.safety_override.apply_safety_override(
                    vehicle_state, steering, throttle, brake
                )
                
                # Apply control
                self._apply_control(steering, throttle, brake, vehicle_state)
                
                # Update visualization (include segmentation mask)
                if not self._update_visualization(image, vehicle_state, predicted_state, lane_info, segmentation_mask):
                    break
                
                # Log data
                self._log_step(vehicle_state, predicted_state, (steering, throttle, brake))
                
                self.step_count += 1
                
                # Limit frame rate
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            logger.info("Received interrupt signal, shutting down...")
        except Exception as e:
            logger.error(f"Error in inference loop: {e}", exc_info=True)
        finally:
            self._log_validation_summary()
            self.running = False
    
    def _get_and_validate_image(self) -> Optional[np.ndarray]:
        """Get and validate camera image."""
        try:
            image = self.system.camera.get_image()
            ImageValidator.validate(image)
            return image
        except DataValidationError:
            self.validation_errors['image'] += 1
            if self.validation_errors['image'] % 100 == 0:
                logger.warning(f"Image validation failed {self.validation_errors['image']} times")
            time.sleep(0.01)
            return None
        except Exception as e:
            logger.warning(f"Failed to get image: {e}")
            return None
    
    def _get_and_validate_state(self) -> Optional[Dict[str, Any]]:
        """Get and validate vehicle state."""
        try:
            vehicle_state = self.system.carla_client.get_vehicle_state()
            StateValidator.validate(vehicle_state)
            return vehicle_state
        except DataValidationError:
            self.validation_errors['state'] += 1
            if self.validation_errors['state'] % 100 == 0:
                logger.warning(f"State validation failed {self.validation_errors['state']} times")
            time.sleep(0.01)
            return None
        except Exception as e:
            logger.warning(f"Failed to get vehicle state: {e}")
            return None
    
    def _detect_lanes(self, image: np.ndarray) -> Tuple[Optional[Dict[str, Any]], Optional[list]]:
        """Detect lanes using geometric computation + CARLA waypoints."""
        lane_info = None
        waypoints = None

        world = self.system.carla_client.world
        vehicle = self.system.carla_client.vehicle
        if world is None or vehicle is None:
            return None, None

        # Always get CARLA waypoints for MPC
        try:
            carla_map = world.get_map()
            vehicle_tf = vehicle.get_transform()
            current_wp = carla_map.get_waypoint(vehicle_tf.location)
            if current_wp is not None:
                waypoints = []
                wp = current_wp
                for _ in range(self.system.mpc_controller.N + 10):
                    waypoints.append(wp)
                    nxt = wp.next(2.0)
                    if nxt and len(nxt) > 0:
                        wp = nxt[0]
                    else:
                        break
        except Exception as e:
            if self.step_count % 100 == 0:
                logger.debug(f"Waypoint extraction failed: {e}")

        # Geometric lane detection (primary - no ML)
        geo = getattr(self.system, 'geometric_lane_detector', None)
        if geo is not None:
            try:
                lane_mask, lane_features, lane_coords = geo.detect(world, vehicle, image)
                has_lanes = lane_mask is not None and np.sum(lane_mask > 0) > 50
                if has_lanes:
                    lane_center = self.system.mpc_controller.lane_planner.extract_lane_center_from_image(
                        lane_mask, image
                    )
                    lane_info = {
                        'lane_mask': lane_mask,
                        'center_line': lane_center,
                        'left_lane': None,
                        'right_lane': None,
                        'lane_coords': lane_coords,
                    }
            except Exception as e:
                if self.step_count % 100 == 0:
                    logger.debug(f"Geometric lane detection failed: {e}")

        return lane_info, waypoints

    def _encode_image(self, image: np.ndarray, segmentation_mask: Optional[np.ndarray] = None) -> Optional[np.ndarray]:
        """
        Encode image to features.
        
        Args:
            image: RGB image
            segmentation_mask: Optional segmentation mask from UNet
        
        Returns:
            Feature vector
        """
        try:
            # PHASE 2: UNet → ResNet flow
            # Currently ResNetEncoder expects RGB, so we use original image
            # Future enhancement: could use mask to guide feature extraction
            features = self.system.resnet_encoder.encode(image)
            FeatureValidator.validate(features, self.config['perception']['feature_dim'])
            self.system.sequence_buffer.add(features)
            return features
        except DataValidationError:
            self.validation_errors['features'] += 1
            if self.validation_errors['features'] % 100 == 0:
                logger.warning(f"Feature validation failed {self.validation_errors['features']} times")
            # Use last valid feature if available
            if len(self.system.sequence_buffer.buffer) > 0:
                return self.system.sequence_buffer.buffer[-1]
            return None
        except Exception as e:
            self.validation_errors['features'] += 1
            if self.validation_errors['features'] % 100 == 0:
                logger.warning(f"Feature encoding failed {self.validation_errors['features']} times: {e}")
            if len(self.system.sequence_buffer.buffer) == 0:
                return None
            return self.system.sequence_buffer.buffer[-1] if len(self.system.sequence_buffer.buffer) > 0 else None
    
    def _predict_state(self, features: np.ndarray, vehicle_state: Dict[str, Any]) -> Optional[np.ndarray]:
        """Predict future state using LSTM. Waypoint trajectory takes priority over LSTM."""
        predicted_state = None
        
        if self.system.sequence_buffer.is_ready():
            sequence = self.system.sequence_buffer.get_sequence()
            if sequence is None or sequence.shape[0] != self.config['temporal']['sequence_length']:
                logger.warning(f"Invalid sequence at step {self.step_count}")
                return None
            
            try:
                predicted_state = self.system.lstm_predictor.predict(sequence)
                
                if hasattr(self.system, 'state_mean') and self.system.state_mean is not None:
                    state_mean = np.array(self.system.state_mean)
                    state_std = np.array(self.system.state_std)
                    state_std = np.where(state_std < 1e-6, 1.0, state_std)
                    predicted_state = predicted_state * state_std + state_mean
                
                try:
                    PredictionValidator.validate(predicted_state)
                except DataValidationError:
                    self.validation_errors['prediction'] += 1
                    predicted_state = np.array([
                        vehicle_state['x'], vehicle_state['y'],
                        vehicle_state['yaw'], vehicle_state['velocity']
                    ])
            except Exception as e:
                self.validation_errors['prediction'] += 1
                if self.step_count % 100 == 0:
                    logger.warning(f"LSTM prediction failed: {e}")
                predicted_state = np.array([
                    vehicle_state['x'], vehicle_state['y'],
                    vehicle_state['yaw'], vehicle_state['velocity']
                ])
        
        # Waypoint-based trajectory takes priority (much more reliable than LSTM)
        # LSTM predicted_state is logged but waypoints drive MPC
        # MPC will use lane_planner.generate_reference_from_lanes() with waypoints
        # when no explicit reference_trajectory is set — which is exactly what we want.
        # So we intentionally do NOT set reference_trajectory here.
        # This lets MPC use CARLA waypoints via lane_planner for path following.
        
        return predicted_state
    
    def _compute_control(
        self,
        vehicle_state: Dict[str, Any],
        predicted_state: Optional[np.ndarray]
    ) -> Tuple[float, float, float]:
        """Compute control using MPC with waypoint-based trajectory."""
        try:
            steering, throttle, brake = self.system.mpc_controller.compute_control(
                vehicle_state, None
            )
            
            speed_kmh = vehicle_state['velocity'] * 3.6
            
            if not self.system.sequence_buffer.is_ready():
                brake = 0.0
                throttle = max(throttle, 0.5)
                steering = np.clip(steering, -0.3, 0.3)
                if self.step_count % 20 == 0:
                    logger.info(
                        f"Buffer filling ({len(self.system.sequence_buffer.buffer)}/"
                        f"{self.config['temporal']['sequence_length']}), "
                        f"steer={steering:.2f}, throttle={throttle:.2f}"
                    )
            elif speed_kmh < 5.0:
                brake = 0.0
                throttle = max(throttle, 0.6)
                if self.step_count % 30 == 0:
                    logger.info(
                        f"Low speed boost: {speed_kmh:.1f} km/h, "
                        f"steer={steering:.3f}, throttle={throttle:.3f}"
                    )
            elif speed_kmh < 20.0 and brake == 0.0:
                throttle = max(throttle, 0.3)
            
            try:
                ControlValidator.validate(steering, throttle, brake)
            except DataValidationError as e:
                self.validation_errors['control'] += 1
                logger.warning(f"Control validation failed: {e}")
                steering, throttle, brake = 0.0, 0.3, 0.0
            
            if self.step_count % 50 == 0:
                logger.info(
                    f"Control: steer={steering:.3f}, throttle={throttle:.3f}, "
                    f"brake={brake:.3f}, speed={speed_kmh:.1f} km/h"
                )
            
            return steering, throttle, brake
            
        except Exception as e:
            logger.error(f"MPC control failed: {e}", exc_info=True)
            if vehicle_state['velocity'] < 2.0:
                return 0.0, 0.5, 0.0
            else:
                return 0.0, 0.0, 0.5

    def _apply_control(
        self,
        steering: float,
        throttle: float,
        brake: float,
        vehicle_state: Dict[str, Any]
    ) -> None:
        """Apply control to vehicle."""
        try:
            control = carla.VehicleControl(
                steer=steering,
                throttle=throttle,
                brake=brake
            )
            self.system.carla_client.apply_control(control)
        except Exception as e:
            logger.error(f"Failed to apply control: {e}")
            # Emergency fallback
            if vehicle_state['velocity'] < 2.0:
                control = carla.VehicleControl(steer=0.0, throttle=0.6, brake=0.0)
                logger.warning(f"Emergency throttle (step={self.step_count})")
            else:
                control = carla.VehicleControl(steer=0.0, throttle=0.0, brake=1.0)
            self.system.carla_client.apply_control(control)
    
    def _update_visualization(
        self,
        image: np.ndarray,
        vehicle_state: Dict[str, Any],
        predicted_state: Optional[np.ndarray],
        lane_info: Optional[Dict[str, Any]],
        segmentation_mask: Optional[np.ndarray] = None
    ) -> bool:
        """Update visualization."""
        predicted_traj = None
        if predicted_state is not None:
            predicted_traj = self.system._state_to_trajectory(predicted_state, vehicle_state)
        
        mpc_horizon = self.system.mpc_controller.get_predicted_trajectory()
        
        # Update lane_info with segmentation mask if available
        if segmentation_mask is not None and lane_info is not None:
            lane_info['unet_segmentation'] = segmentation_mask
        
        return self.system.visualization.update(
            image,
            vehicle_state,
            predicted_traj,
            mpc_horizon,
            lane_info
        )
    
    def _log_step(
        self,
        vehicle_state: Dict[str, Any],
        predicted_state: Optional[np.ndarray],
        control: tuple
    ) -> None:
        """Log step data."""
        self.system._log_step(vehicle_state, predicted_state, control)
    
    def _log_validation_summary(self) -> None:
        """Log validation summary."""
        total_errors = sum(self.validation_errors.values())
        if total_errors > 0:
            logger.warning("Validation Summary:")
            for key, count in self.validation_errors.items():
                if count > 0:
                    logger.warning(f"  {key}: {count} errors")
        else:
            logger.info("✅ No validation errors detected")
