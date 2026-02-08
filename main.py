"""
Main entry point for CARLA LSTM-MPC Autonomous Driving System.

Supports both data collection and inference control modes.
"""

import sys
import os
import yaml
import logging
import argparse
import time
import numpy as np
import carla
from pathlib import Path
from typing import Dict, Any, Optional
import csv
from datetime import datetime

# Add project root to path
project_root = Path(__file__).parent
sys.path.insert(0, str(project_root))

from carla_env import CarlaClient, CameraSensor
from perception import ResNetEncoder
from perception.lane_detector import LaneDetector
from temporal import LSTMPredictor, SequenceBuffer
from control import MPCController
from visualization import VisualizationDisplay
from utils.database import DatabaseManager
from utils.device_utils import get_device_info, get_device

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('logs/main.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)


class AutonomousDrivingSystem:
    """Main autonomous driving system."""
    
    def __init__(self, config_path: str = "config.yaml"):
        """
        Initialize autonomous driving system.
        
        Args:
            config_path: Path to configuration YAML file
        """
        # Load configuration
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        
        # Initialize components
        logger.info("Initializing system components...")
        
        # Display device info
        device_info = get_device_info()
        logger.info(f"Device: {device_info['device_name']} ({device_info['device_type']})")
        
        # Database manager (optional)
        self.database = DatabaseManager(self.config.get('database', {'enabled': False}))
        
        # CARLA client
        self.carla_client = CarlaClient(self.config['carla'])
        
        # Perception
        self.resnet_encoder = ResNetEncoder(
            feature_dim=self.config['perception']['feature_dim'],
            pretrained=self.config['perception']['pretrained'],
            freeze_backbone=self.config['perception']['freeze_backbone']
        )
        
        # Temporal
        # Check if trained model path is provided
        trained_model_path = self.config.get('temporal', {}).get('trained_model_path', None)
        if trained_model_path and Path(trained_model_path).exists():
            logger.info(f"Loading trained LSTM model from {trained_model_path}")
            try:
                from training.load_trained_model import load_trained_lstm
                self.lstm_predictor, self.state_mean, self.state_std = load_trained_lstm(trained_model_path)
                self.lstm_predictor.to(get_device())
                logger.info("✅ Trained LSTM model loaded")
            except Exception as e:
                logger.warning(f"Failed to load trained model: {e}, using default")
                self.lstm_predictor = LSTMPredictor(
                    input_size=self.config['temporal']['input_size'],
                    hidden_size=self.config['temporal']['hidden_size'],
                    num_layers=self.config['temporal']['num_layers'],
                    sequence_length=self.config['temporal']['sequence_length'],
                    dropout=self.config['temporal']['dropout']
                )
                self.state_mean = None
                self.state_std = None
        else:
            self.lstm_predictor = LSTMPredictor(
                input_size=self.config['temporal']['input_size'],
                hidden_size=self.config['temporal']['hidden_size'],
                num_layers=self.config['temporal']['num_layers'],
                sequence_length=self.config['temporal']['sequence_length'],
                dropout=self.config['temporal']['dropout']
            )
            self.state_mean = None
            self.state_std = None
        self.sequence_buffer = SequenceBuffer(
            sequence_length=self.config['temporal']['sequence_length'],
            feature_dim=self.config['perception']['feature_dim']
        )
        
        # Control
        self.mpc_controller = MPCController(self.config)
        
        # Visualization
        self.visualization = VisualizationDisplay(self.config['visualization'])
        
        # Camera sensor (will be initialized after vehicle spawn)
        self.camera: Optional[CameraSensor] = None
        
        # Data collection
        self.data_collection_enabled = self.config['data_collection']['enabled']
        self.data_dir = Path(self.config['data_collection']['save_path'])
        self.data_dir.mkdir(exist_ok=True)
        
        # Logging
        self.log_dir = Path(self.config['logging']['log_dir'])
        self.log_dir.mkdir(exist_ok=True)
        
        # State tracking
        self.running = False
        self.step_count = 0
        self.trajectory_log = []
        self.control_log = []
        self.prediction_log = []
        
        logger.info("✅ System initialization complete")
    
    def initialize_carla(self) -> bool:
        """
        Initialize CARLA environment.
        
        Returns:
            True if successful
        """
        logger.info("Initializing CARLA environment...")
        
        # Connect to CARLA
        if not self.carla_client.connect():
            return False
        
        # Load world
        if not self.carla_client.load_world():
            return False
        
        # Spawn vehicle
        if not self.carla_client.spawn_vehicle():
            return False
        
        # Setup camera
        try:
            self.camera = CameraSensor(
                self.carla_client.world,
                self.carla_client.vehicle,
                self.config['camera']
            )
            logger.info("✅ Camera sensor initialized")
        except Exception as e:
            logger.error(f"❌ Failed to setup camera: {e}")
            return False
        
        # Initialize lane detector
        try:
            self.lane_detector = LaneDetector(use_carla=True)
            logger.info("✅ Lane detector initialized")
        except Exception as e:
            logger.warning(f"⚠️  Failed to initialize lane detector: {e}, continuing without lane detection")
            self.lane_detector = None
        
        logger.info("✅ CARLA environment initialized")
        return True
    
    def _validate_image(self, image: np.ndarray) -> bool:
        """Validate camera image."""
        if image is None:
            return False
        if not isinstance(image, np.ndarray):
            return False
        if image.shape[0] == 0 or image.shape[1] == 0:
            return False
        if len(image.shape) != 3 or image.shape[2] != 3:
            return False
        return True
    
    def _validate_vehicle_state(self, state: Dict[str, Any]) -> bool:
        """Validate vehicle state."""
        required_keys = ['x', 'y', 'yaw', 'velocity']
        if not all(key in state for key in required_keys):
            return False
        
        # Check for NaN or Inf
        for key in required_keys:
            value = state[key]
            if not isinstance(value, (int, float)) or np.isnan(value) or np.isinf(value):
                return False
        
        # Check reasonable ranges
        if abs(state['x']) > 10000 or abs(state['y']) > 10000:
            return False
        if abs(state['yaw']) > 360:
            return False
        if state['velocity'] < 0 or state['velocity'] > 100:  # m/s
            return False
        
        return True
    
    def _validate_features(self, features: np.ndarray) -> bool:
        """Validate feature vector."""
        if features is None:
            return False
        if not isinstance(features, np.ndarray):
            return False
        if features.shape[0] != self.config['perception']['feature_dim']:
            return False
        if np.any(np.isnan(features)) or np.any(np.isinf(features)):
            return False
        return True
    
    def _validate_prediction(self, prediction: np.ndarray) -> bool:
        """Validate LSTM prediction."""
        if prediction is None:
            return False
        if not isinstance(prediction, np.ndarray):
            return False
        if len(prediction) != 4:  # x, y, yaw, velocity
            return False
        if np.any(np.isnan(prediction)) or np.any(np.isinf(prediction)):
            return False
        return True
    
    def _validate_control(self, steering: float, throttle: float, brake: float) -> bool:
        """Validate control values."""
        if not all(isinstance(x, (int, float)) for x in [steering, throttle, brake]):
            return False
        if np.isnan(steering) or np.isnan(throttle) or np.isnan(brake):
            return False
        if not (-1.1 <= steering <= 1.1):
            return False
        if not (-0.1 <= throttle <= 1.1):
            return False
        if not (-0.1 <= brake <= 1.1):
            return False
        return True
    
    def run_inference(self) -> None:
        """Run inference control loop."""
        logger.info("Starting inference control loop...")
        self.running = True
        
        # Validation counters
        validation_errors = {
            'image': 0,
            'state': 0,
            'features': 0,
            'prediction': 0,
            'control': 0
        }
        
        try:
            while self.running:
                # Tick world (synchronous mode)
                self.carla_client.tick()
                
                # Get camera image
                image = self.camera.get_image()
                if not self._validate_image(image):
                    validation_errors['image'] += 1
                    if validation_errors['image'] % 100 == 0:
                        logger.warning(f"Image validation failed {validation_errors['image']} times")
                    time.sleep(0.01)
                    continue
                
                # Detect lanes
                lane_info = None
                waypoints = None
                if self.lane_detector is not None:
                    try:
                        world = self.carla_client.world
                        vehicle = self.carla_client.vehicle
                        if world is not None and vehicle is not None:
                            # Detect lanes using CARLA
                            lane_mask, lane_features = self.lane_detector.detect_lanes(
                                image, world=world, vehicle=vehicle
                            )
                            
                            # Extract lane center from mask
                            if lane_mask is not None:
                                lane_center = self.mpc_controller.lane_planner.extract_lane_center_from_image(
                                    lane_mask, image
                                )
                                
                                lane_info = {
                                    'lane_mask': lane_mask,
                                    'center_line': lane_center,
                                    'left_lane': None,  # Can be extracted from mask
                                    'right_lane': None
                                }
                            
                            # Get waypoints from CARLA
                            try:
                                carla_map = world.get_map()
                                vehicle_transform = vehicle.get_transform()
                                current_waypoint = carla_map.get_waypoint(vehicle_transform.location)
                                
                                if current_waypoint is not None:
                                    waypoints = []
                                    wp = current_waypoint
                                    for _ in range(self.mpc_controller.N + 1):
                                        waypoints.append(wp)
                                        next_wps = wp.next(2.0)  # 2 meters ahead
                                        if next_wps and len(next_wps) > 0:
                                            wp = next_wps[0]
                                        else:
                                            break
                            except Exception as e:
                                if self.step_count % 100 == 0:
                                    logger.debug(f"Waypoint extraction failed: {e}")
                    except Exception as e:
                        if self.step_count % 100 == 0:
                            logger.debug(f"Lane detection failed: {e}")
                
                # Update MPC with lane info
                if lane_info is not None:
                    self.mpc_controller.set_lane_info(lane_info)
                if waypoints is not None:
                    self.mpc_controller.set_waypoints(waypoints)
                
                # Get vehicle state
                vehicle_state = self.carla_client.get_vehicle_state()
                if not self._validate_vehicle_state(vehicle_state):
                    validation_errors['state'] += 1
                    if validation_errors['state'] % 100 == 0:
                        logger.warning(f"State validation failed {validation_errors['state']} times")
                    time.sleep(0.01)
                    continue
                
                # Encode image to features
                try:
                    features = self.resnet_encoder.encode(image)
                    if not self._validate_features(features):
                        validation_errors['features'] += 1
                        if validation_errors['features'] % 100 == 0:
                            logger.warning(f"Feature validation failed {validation_errors['features']} times")
                        # Still add to buffer even if validation fails (use previous)
                        if len(self.sequence_buffer.buffer) > 0:
                            features = self.sequence_buffer.buffer[-1]  # Use last valid feature
                        else:
                            continue
                    self.sequence_buffer.add(features)
                except Exception as e:
                    validation_errors['features'] += 1
                    if validation_errors['features'] % 100 == 0:
                        logger.warning(f"Feature encoding failed {validation_errors['features']} times: {e}")
                    # Continue anyway to keep control loop running
                    if len(self.sequence_buffer.buffer) == 0:
                        continue
                
                # Predict future state using LSTM
                predicted_state = None
                if self.sequence_buffer.is_ready():
                    sequence = self.sequence_buffer.get_sequence()
                    if sequence is None or sequence.shape[0] != self.config['temporal']['sequence_length']:
                        logger.warning(f"Invalid sequence at step {self.step_count}")
                        continue
                    
                    try:
                        predicted_state = self.lstm_predictor.predict(sequence)
                        
                        # Validate prediction
                        if not self._validate_prediction(predicted_state):
                            validation_errors['prediction'] += 1
                            logger.warning(f"Prediction validation failed at step {self.step_count}")
                            # Use simple forward prediction as fallback
                            predicted_state = np.array([
                                vehicle_state['x'],
                                vehicle_state['y'],
                                vehicle_state['yaw'],
                                vehicle_state['velocity']
                            ])
                        else:
                            # Set reference trajectory for MPC
                            # Convert predicted state to reference trajectory
                            ref_traj = self._state_to_trajectory(predicted_state, vehicle_state)
                            self.mpc_controller.set_reference_trajectory(ref_traj)
                    except Exception as e:
                        validation_errors['prediction'] += 1
                        logger.warning(f"LSTM prediction failed: {e}")
                        # Use simple forward prediction as fallback
                        predicted_state = np.array([
                            vehicle_state['x'],
                            vehicle_state['y'],
                            vehicle_state['yaw'],
                            vehicle_state['velocity']
                        ])
                
                # Compute control using MPC
                try:
                    # Use reference trajectory if set, otherwise let MPC generate default
                    ref_traj = None
                    if hasattr(self.mpc_controller, 'reference_trajectory') and self.mpc_controller.reference_trajectory is not None:
                        ref_traj = self.mpc_controller.reference_trajectory
                    
                    steering, throttle, brake = self.mpc_controller.compute_control(
                        vehicle_state,
                        ref_traj
                    )
                    
                    # CRITICAL FIX: Always ensure movement
                    # If buffer not ready, force startup
                    if not self.sequence_buffer.is_ready():
                        throttle = 0.6
                        steering = 0.0
                        brake = 0.0
                        if self.step_count % 20 == 0:
                            logger.info(f"Buffer filling ({len(self.sequence_buffer.buffer)}/{self.config['temporal']['sequence_length']}), startup throttle=0.6")
                    # If vehicle stopped or very slow, FORCE movement (override MPC)
                    elif vehicle_state['velocity'] < 2.0:
                        # Override MPC completely when stuck
                        throttle = 0.7
                        brake = 0.0
                        # Keep steering from MPC if reasonable, otherwise straight
                        if abs(steering) > 0.8:
                            steering = 0.0
                        if self.step_count % 10 == 0:
                            logger.warning(f"⚠️  Vehicle stuck ({vehicle_state['velocity']*3.6:.1f} km/h), OVERRIDING with throttle=0.7")
                    # Ensure minimum throttle for low speeds
                    elif vehicle_state['velocity'] < 5.0 and brake == 0.0:
                        if throttle < 0.4:
                            throttle = 0.4
                            if self.step_count % 50 == 0:
                                logger.debug(f"Boosting throttle for low speed: {vehicle_state['velocity']*3.6:.1f} km/h")
                    
                    # Safety constraints
                    if self.config['safety']['emergency_brake_enabled']:
                        speed_kmh = vehicle_state['velocity'] * 3.6
                        if speed_kmh > self.config['safety']['max_speed_kmh']:
                            throttle = 0.0
                            brake = 1.0
                            logger.warning(f"Speed limit exceeded: {speed_kmh:.1f} km/h, applying brake")
                    
                    # Log control values for debugging
                    if self.step_count % 50 == 0:
                        logger.info(f"Control: steer={steering:.3f}, throttle={throttle:.3f}, brake={brake:.3f}, speed={vehicle_state['velocity']*3.6:.1f} km/h")
                    
                    # Apply control
                    control = carla.VehicleControl(
                        steer=steering,
                        throttle=throttle,
                        brake=brake
                    )
                    self.carla_client.apply_control(control)
                    
                except Exception as e:
                    logger.error(f"MPC control failed: {e}", exc_info=True)
                    # Fallback: aggressive forward movement to get unstuck
                    if vehicle_state['velocity'] < 2.0:
                        control = carla.VehicleControl(steer=0.0, throttle=0.6, brake=0.0)
                        logger.warning(f"Emergency throttle (step={self.step_count}, speed={vehicle_state['velocity']*3.6:.1f} km/h)")
                    else:
                        control = carla.VehicleControl(steer=0.0, throttle=0.0, brake=1.0)
                    self.carla_client.apply_control(control)
                
                # Update visualization
                predicted_traj = None
                if predicted_state is not None:
                    predicted_traj = self._state_to_trajectory(predicted_state, vehicle_state)
                
                # Get MPC predicted trajectory
                mpc_horizon = self.mpc_controller.get_predicted_trajectory()
                
                continue_running = self.visualization.update(
                    image,
                    vehicle_state,
                    predicted_traj,
                    mpc_horizon,
                    lane_info
                )
                if not continue_running:
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
            # Log validation summary
            total_errors = sum(validation_errors.values())
            if total_errors > 0:
                logger.warning("Validation Summary:")
                for key, count in validation_errors.items():
                    if count > 0:
                        logger.warning(f"  {key}: {count} errors")
            else:
                logger.info("✅ No validation errors detected")
            
            self.cleanup()
    
    def run_data_collection(self) -> None:
        """Run data collection mode."""
        logger.info("Starting data collection mode...")
        self.running = True
        
        # Create data directories
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        run_dir = self.data_dir / f"run_{timestamp}"
        run_dir.mkdir(exist_ok=True)
        images_dir = run_dir / "images"
        images_dir.mkdir(exist_ok=True)
        
        # CSV file for states and controls
        csv_path = run_dir / "data.csv"
        csv_file = open(csv_path, 'w', newline='')
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow([
            'step', 'image_path', 'x', 'y', 'yaw', 'velocity',
            'steering', 'throttle', 'brake'
        ])
        
        try:
            while self.running:
                self.carla_client.tick()
                
                # Get image
                image = self.camera.get_image()
                if image is None:
                    time.sleep(0.01)
                    continue
                
                # Get vehicle state
                vehicle_state = self.carla_client.get_vehicle_state()
                
                # Save image
                image_path = None
                if self.config['data_collection']['save_images']:
                    image_path = images_dir / f"image_{self.step_count:06d}.png"
                    import cv2
                    cv2.imwrite(str(image_path), cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
                
                # Save data to CSV
                if self.config['data_collection']['save_states'] or self.config['data_collection']['save_controls']:
                    csv_writer.writerow([
                        self.step_count,
                        f"images/image_{self.step_count:06d}.png" if image_path else '',
                        vehicle_state['x'],
                        vehicle_state['y'],
                        vehicle_state['yaw'],
                        vehicle_state['velocity'],
                        vehicle_state['steering'],
                        vehicle_state['throttle'],
                        vehicle_state['brake']
                    ])
                
                # Save to database if enabled
                if self.database.enabled:
                    # Encode features
                    try:
                        features = self.resnet_encoder.encode(image)
                        features_list = features.tolist()
                    except Exception as e:
                        logger.warning(f"Feature encoding failed: {e}")
                        features_list = None
                    
                    self.database.save_data(
                        step=self.step_count,
                        image_path=str(image_path) if image_path else None,
                        vehicle_state=vehicle_state,
                        control=(vehicle_state['steering'], vehicle_state['throttle'], vehicle_state['brake']),
                        features=features_list,
                        metadata={'run_dir': str(run_dir)}
                    )
                
                # Update visualization
                self.visualization.update(image, vehicle_state)
                
                self.step_count += 1
                
                if self.step_count % 100 == 0:
                    logger.info(f"Collected {self.step_count} samples")
                
                time.sleep(0.01)
                
        except KeyboardInterrupt:
            logger.info("Data collection interrupted")
        finally:
            csv_file.close()
            logger.info(f"Data collection complete. Saved to {run_dir}")
            self.cleanup()
    
    def _state_to_trajectory(self, state: np.ndarray, current_vehicle_state: Dict[str, Any]) -> np.ndarray:
        """
        Convert predicted state to reference trajectory for MPC.
        
        Args:
            state: Predicted state [x, y, yaw, velocity]
            current_vehicle_state: Current vehicle state
            
        Returns:
            Reference trajectory (N+1, 4)
        """
        N = self.mpc_controller.N
        dt = self.mpc_controller.dt
        
        # Extract predicted state
        pred_x = state[0] if len(state) > 0 else current_vehicle_state['x']
        pred_y = state[1] if len(state) > 1 else current_vehicle_state['y']
        pred_yaw = np.deg2rad(state[2]) if len(state) > 2 else np.deg2rad(current_vehicle_state['yaw'])
        pred_v = state[3] if len(state) > 3 else current_vehicle_state['velocity']
        
        # Generate trajectory from current to predicted
        current_x = current_vehicle_state['x']
        current_y = current_vehicle_state['y']
        current_yaw = np.deg2rad(current_vehicle_state['yaw'])
        current_v = current_vehicle_state['velocity']
        
        ref_traj = np.zeros((N + 1, 4))
        for i in range(N + 1):
            alpha = i / (N + 1)  # Interpolation factor
            ref_traj[i, 0] = current_x + alpha * (pred_x - current_x)
            ref_traj[i, 1] = current_y + alpha * (pred_y - current_y)
            ref_traj[i, 2] = current_yaw + alpha * (pred_yaw - current_yaw)
            ref_traj[i, 3] = current_v + alpha * (pred_v - current_v)
        
        return ref_traj
    
    def _log_step(
        self,
        vehicle_state: Dict[str, Any],
        predicted_state: Optional[np.ndarray],
        control: tuple
    ) -> None:
        """Log step data."""
        if self.config['logging']['save_trajectory']:
            self.trajectory_log.append({
                'step': self.step_count,
                'x': vehicle_state['x'],
                'y': vehicle_state['y'],
                'yaw': vehicle_state['yaw'],
                'velocity': vehicle_state['velocity']
            })
        
        if self.config['logging']['save_controls']:
            self.control_log.append({
                'step': self.step_count,
                'steering': control[0],
                'throttle': control[1],
                'brake': control[2]
            })
        
        if self.config['logging']['save_predictions'] and predicted_state is not None:
            self.prediction_log.append({
                'step': self.step_count,
                'pred_x': predicted_state[0] if len(predicted_state) > 0 else 0,
                'pred_y': predicted_state[1] if len(predicted_state) > 1 else 0,
                'pred_yaw': predicted_state[2] if len(predicted_state) > 2 else 0,
                'pred_velocity': predicted_state[3] if len(predicted_state) > 3 else 0
            })
    
    def save_logs(self) -> None:
        """Save logged data to files."""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        if self.config['logging']['save_trajectory'] and self.trajectory_log:
            traj_path = self.log_dir / f"trajectory_{timestamp}.csv"
            with open(traj_path, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=['step', 'x', 'y', 'yaw', 'velocity'])
                writer.writeheader()
                writer.writerows(self.trajectory_log)
            logger.info(f"✅ Trajectory log saved to {traj_path}")
        
        if self.config['logging']['save_controls'] and self.control_log:
            control_path = self.log_dir / f"controls_{timestamp}.csv"
            with open(control_path, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=['step', 'steering', 'throttle', 'brake'])
                writer.writeheader()
                writer.writerows(self.control_log)
            logger.info(f"✅ Control log saved to {control_path}")
        
        if self.config['logging']['save_predictions'] and self.prediction_log:
            pred_path = self.log_dir / f"predictions_{timestamp}.csv"
            with open(pred_path, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=['step', 'pred_x', 'pred_y', 'pred_yaw', 'pred_velocity'])
                writer.writeheader()
                writer.writerows(self.prediction_log)
            logger.info(f"✅ Prediction log saved to {pred_path}")
    
    def cleanup(self) -> None:
        """Clean up resources."""
        logger.info("Cleaning up resources...")
        self.running = False
        
        # Save logs
        self.save_logs()
        
        # Destroy camera
        if self.camera is not None:
            self.camera.destroy()
        
        # Cleanup CARLA
        self.carla_client.cleanup()
        
        # Close visualization
        self.visualization.close()
        
        # Close database
        if self.database.enabled:
            self.database.close()
        
        logger.info("✅ Cleanup complete")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description='CARLA LSTM-MPC Autonomous Driving')
    parser.add_argument(
        '--mode',
        type=str,
        choices=['inference', 'collect'],
        default='inference',
        help='Run mode: inference (control) or collect (data collection)'
    )
    parser.add_argument(
        '--config',
        type=str,
        default='config.yaml',
        help='Path to configuration file'
    )
    
    args = parser.parse_args()
    
    # Initialize system
    system = AutonomousDrivingSystem(config_path=args.config)
    
    # Initialize CARLA
    if not system.initialize_carla():
        logger.error("Failed to initialize CARLA environment")
        return 1
    
    # Run based on mode
    if args.mode == 'inference':
        system.run_inference()
    elif args.mode == 'collect':
        system.run_data_collection()
    
    return 0


if __name__ == '__main__':
    sys.exit(main())

