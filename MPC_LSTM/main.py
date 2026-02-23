"""
Main entry point for CARLA LSTM-MPC Autonomous Driving System.

Supports both data collection and inference control modes.
"""

import sys
import os
import signal
import atexit
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
from perception.geometric_lane_detector import GeometricLaneDetector
from temporal import LSTMPredictor, SequenceBuffer
from control import MPCController
from visualization import VisualizationDisplay
from utils.database import DatabaseManager
from utils.device_utils import get_device_info, get_device
from core.system_inference import InferenceSystem
from core.system_collection import DataCollectionSystem

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
        
        # Initialize system modules
        self.inference_system = InferenceSystem(self)
        self.collection_system = DataCollectionSystem(self)
        
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
        
        # Initialize geometric lane detector (no ML, pure computation)
        try:
            self.geometric_lane_detector = GeometricLaneDetector(self.config['camera'])
            self.lane_detector = None
            logger.info("\u2705 Geometric lane detector initialized (no ML)")
        except Exception as e:
            logger.warning(f"Failed to init geometric lane detector: {e}")
            self.geometric_lane_detector = None
            self.lane_detector = None
        
        logger.info("✅ CARLA environment initialized")
        return True
    
    
    def run_inference(self) -> None:
        """Run inference control loop."""
        self.inference_system.run()
        self.cleanup()
    
    def run_data_collection(self) -> None:
        """Run data collection mode."""
        self.collection_system.run()
        self.cleanup()
    
    def _state_to_trajectory(self, state: np.ndarray, current_vehicle_state: Dict[str, Any]) -> np.ndarray:
        """
        Convert predicted state to reference trajectory for MPC.
        
        Uses predicted heading and velocity to project a forward trajectory
        from the current position, avoiding large jumps from absolute position prediction.
        
        Args:
            state: Predicted state [x, y, yaw, velocity]
            current_vehicle_state: Current vehicle state
            
        Returns:
            Reference trajectory (N+1, 4)
        """
        N = self.mpc_controller.N
        dt = self.mpc_controller.dt
        
        current_x = current_vehicle_state['x']
        current_y = current_vehicle_state['y']
        current_yaw = np.deg2rad(current_vehicle_state['yaw'])
        current_v = current_vehicle_state['velocity']
        
        pred_yaw = np.deg2rad(state[2]) if len(state) > 2 else current_yaw
        pred_v = max(2.0, state[3]) if len(state) > 3 else max(2.0, current_v)
        
        # Blend predicted heading with current heading for stability
        yaw_blend = 0.3
        target_yaw = current_yaw + yaw_blend * (pred_yaw - current_yaw)
        target_yaw = np.arctan2(np.sin(target_yaw), np.cos(target_yaw))
        
        target_v = max(5.0, min(pred_v, 12.0))
        
        ref_traj = np.zeros((N + 1, 4))
        x, y, yaw, v = current_x, current_y, current_yaw, current_v
        
        for i in range(N + 1):
            ref_traj[i, 0] = x
            ref_traj[i, 1] = y
            alpha = i / (N + 1)
            ref_traj[i, 2] = current_yaw + alpha * (target_yaw - current_yaw)
            ref_traj[i, 3] = current_v + alpha * (target_v - current_v)
            
            step_v = ref_traj[i, 3]
            step_yaw = ref_traj[i, 2]
            x += step_v * np.cos(step_yaw) * dt
            y += step_v * np.sin(step_yaw) * dt
        
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

