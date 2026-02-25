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
from temporal import LSTMPredictor, SequenceBuffer
from control import MPCController
from visualization import VisualizationDisplay
from utils.database import DatabaseManager
from utils.device_utils import get_device_info, get_device
from core.system_inference import InferenceSystem
from core.system_collection import DataCollectionSystem
from core.geometric_inference import GeometricInferenceSystem
from control.pure_pursuit_controller import PurePursuitController
from perception.geometric_lane_detector import GeometricLaneDetector

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
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        logger.info("Initializing system components...")
        device_info = get_device_info()
        logger.info(f"Device: {device_info['device_name']} ({device_info['device_type']})")
        self.database = DatabaseManager(self.config.get('database', {'enabled': False}))
        self.carla_client = CarlaClient(self.config['carla'])
        self.resnet_encoder = ResNetEncoder(
            feature_dim=self.config['perception']['feature_dim'],
            pretrained=self.config['perception']['pretrained'],
            freeze_backbone=self.config['perception']['freeze_backbone']
        )
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
        self.mpc_controller = MPCController(self.config)
        self.visualization = VisualizationDisplay(self.config['visualization'])
        self.camera = None
        self.data_collection_enabled = self.config['data_collection']['enabled']
        self.data_dir = Path(self.config['data_collection']['save_path'])
        self.data_dir.mkdir(exist_ok=True)
        self.log_dir = Path(self.config['logging']['log_dir'])
        self.log_dir.mkdir(exist_ok=True)
        self.running = False
        self.step_count = 0
        self.trajectory_log = []
        self.control_log = []
        self.prediction_log = []
        self.inference_system = InferenceSystem(self)
        self.collection_system = DataCollectionSystem(self)
        logger.info("✅ System initialization complete")

    def initialize_carla(self):
        logger.info("Initializing CARLA environment...")
        if not self.carla_client.connect() or not self.carla_client.load_world():
            return False
        self.carla_client.cleanup_old_actors()
        if not self.carla_client.spawn_vehicle():
            return False
        try:
            self.camera = CameraSensor(self.carla_client.world, self.carla_client.vehicle, self.config['camera'])
        except Exception as e:
            logger.error(f"Failed to setup camera: {e}")
            return False
        lane_type = self.config.get('perception', {}).get('lane_detection_type', 'geometric')
        if lane_type == 'geometric':
            try:
                self.geometric_lane_detector = GeometricLaneDetector(self.config)
                self.lane_detector = None
            except Exception as e:
                logger.warning(f"Failed to init geometric lane detector: {e}")
                self.geometric_lane_detector = None
                self.lane_detector = None
        else:
            self.geometric_lane_detector = None
            self._init_ml_lane_detector()
        logger.info("✅ CARLA environment initialized")
        return True

    def run_inference(self):
        self.inference_system.run()
        self.cleanup()

    def run_geometric(self):
        self.controller = PurePursuitController(self.config)
        self.geometric_inference = GeometricInferenceSystem(self)
        self.geometric_inference.run()
        self.cleanup()

    def run_data_collection(self):
        self.collection_system.run()
        self.cleanup()

    def _state_to_trajectory(self, state, current_vehicle_state):
        N = self.mpc_controller.N
        dt = self.mpc_controller.dt
        current_x = current_vehicle_state['x']
        current_y = current_vehicle_state['y']
        current_yaw = np.deg2rad(current_vehicle_state['yaw'])
        current_v = current_vehicle_state['velocity']
        pred_yaw = np.deg2rad(state[2]) if len(state) > 2 else current_yaw
        pred_v = max(2.0, state[3]) if len(state) > 3 else max(2.0, current_v)
        yaw_blend = 0.3
        target_yaw = current_yaw + yaw_blend * (pred_yaw - current_yaw)
        target_yaw = np.arctan2(np.sin(target_yaw), np.cos(target_yaw))
        target_v = max(5.0, min(pred_v, 12.0))
        ref_traj = np.zeros((N + 1, 4))
        x, y, yaw, v = current_x, current_y, current_yaw, current_v
        for i in range(N + 1):
            ref_traj[i, 0], ref_traj[i, 1] = x, y
            alpha = i / (N + 1)
            ref_traj[i, 2] = current_yaw + alpha * (target_yaw - current_yaw)
            ref_traj[i, 3] = current_v + alpha * (target_v - current_v)
            step_v, step_yaw = ref_traj[i, 3], ref_traj[i, 2]
            x += step_v * np.cos(step_yaw) * dt
            y += step_v * np.sin(step_yaw) * dt
        return ref_traj

    def _log_step(self, vehicle_state, predicted_state, control):
        if self.config['logging']['save_trajectory']:
            self.trajectory_log.append({'step': self.step_count, **{k: vehicle_state[k] for k in ['x', 'y', 'yaw', 'velocity']}})
        if self.config['logging']['save_controls']:
            self.control_log.append({'step': self.step_count, 'steering': control[0], 'throttle': control[1], 'brake': control[2]})
        if self.config['logging']['save_predictions'] and predicted_state is not None:
            self.prediction_log.append({'step': self.step_count, 'pred_x': predicted_state[0] if len(predicted_state) > 0 else 0, 'pred_y': predicted_state[1] if len(predicted_state) > 1 else 0, 'pred_yaw': predicted_state[2] if len(predicted_state) > 2 else 0, 'pred_velocity': predicted_state[3] if len(predicted_state) > 3 else 0})

    def save_logs(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        if self.config['logging']['save_trajectory'] and self.trajectory_log:
            with open(self.log_dir / f"trajectory_{timestamp}.csv", 'w', newline='') as f:
                csv.DictWriter(f, fieldnames=['step', 'x', 'y', 'yaw', 'velocity']).writeheader()
                csv.DictWriter(f, fieldnames=['step', 'x', 'y', 'yaw', 'velocity']).writerows(self.trajectory_log)
        if self.config['logging']['save_controls'] and self.control_log:
            with open(self.log_dir / f"controls_{timestamp}.csv", 'w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=['step', 'steering', 'throttle', 'brake'])
                w.writeheader()
                w.writerows(self.control_log)
        if self.config['logging']['save_predictions'] and self.prediction_log:
            with open(self.log_dir / f"predictions_{timestamp}.csv", 'w', newline='') as f:
                w = csv.DictWriter(f, fieldnames=['step', 'pred_x', 'pred_y', 'pred_yaw', 'pred_velocity'])
                w.writeheader()
                w.writerows(self.prediction_log)

    def cleanup(self):
        self.running = False
        self.save_logs()
        if self.camera:
            self.camera.destroy()
        self.carla_client.cleanup()
        self.visualization.close()
        if self.database.enabled:
            self.database.close()
        logger.info("✅ Cleanup complete")

    def _init_ml_lane_detector(self):
        pass


def main():
    parser = argparse.ArgumentParser(description='CARLA LSTM-MPC Autonomous Driving')
    parser.add_argument('--mode', type=str, choices=['inference', 'collect', 'geometric'], default='geometric')
    parser.add_argument('--config', type=str, default='config.yaml')
    args = parser.parse_args()
    system = AutonomousDrivingSystem(config_path=args.config)
    def _cleanup(signum=None, frame=None):
        try: system.cleanup()
        except Exception as exc: logger.error("Cleanup error: %s", exc)
        if signum: sys.exit(0)
    signal.signal(signal.SIGINT, _cleanup)
    signal.signal(signal.SIGTERM, _cleanup)
    atexit.register(_cleanup)
    if not system.initialize_carla():
        logger.error("Failed to initialize CARLA")
        return 1
    if args.mode == 'geometric': system.run_geometric()
    elif args.mode == 'inference': system.run_inference()
    elif args.mode == 'collect': system.run_data_collection()
    return 0


if __name__ == '__main__':
    sys.exit(main())
