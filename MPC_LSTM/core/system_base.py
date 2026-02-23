"""
Base system module - common functionality for AutonomousDrivingSystem.

Extracted from main.py for better organization.
"""

import logging
import numpy as np
import carla
from typing import Dict, Any, Optional
from pathlib import Path

logger = logging.getLogger(__name__)


class SystemBase:
    """Base class with common system functionality."""
    
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
    
    def _get_vehicle_state(self) -> Dict[str, Any]:
        """
        Get current vehicle state.
        
        Returns:
            Dictionary with x, y, yaw, velocity
        """
        transform = self.carla_client.vehicle.get_transform()
        velocity = self.carla_client.vehicle.get_velocity()
        
        # Calculate speed
        speed = np.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
        
        return {
            'x': transform.location.x,
            'y': transform.location.y,
            'yaw': np.radians(transform.rotation.yaw),
            'velocity': speed
        }
    
    def _log_step(
        self,
        vehicle_state: Dict[str, Any],
        predicted_traj: Optional[np.ndarray],
        steering: float,
        throttle: float,
        brake: float
    ):
        """Log step data."""
        self.trajectory_log.append({
            'step': self.step_count,
            'x': vehicle_state['x'],
            'y': vehicle_state['y'],
            'yaw': vehicle_state['yaw'],
            'velocity': vehicle_state['velocity']
        })
        
        self.control_log.append({
            'step': self.step_count,
            'steering': steering,
            'throttle': throttle,
            'brake': brake
        })
        
        if predicted_traj is not None and len(predicted_traj) > 0:
            self.prediction_log.append({
                'step': self.step_count,
                'pred_x': predicted_traj[0, 0] if len(predicted_traj) > 0 else 0,
                'pred_y': predicted_traj[0, 1] if len(predicted_traj) > 0 else 0,
                'pred_yaw': predicted_traj[0, 2] if len(predicted_traj) > 0 else 0,
                'pred_velocity': predicted_traj[0, 3] if len(predicted_traj) > 0 else 0
            })
    
    def save_logs(self):
        """Save logs to files."""
        import csv
        from datetime import datetime
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Save trajectory
        if self.trajectory_log:
            traj_path = self.log_dir / f"trajectory_{timestamp}.csv"
            with open(traj_path, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=['step', 'x', 'y', 'yaw', 'velocity'])
                writer.writeheader()
                writer.writerows(self.trajectory_log)
            logger.info(f"✅ Trajectory log saved to {traj_path}")
        
        # Save control
        if self.control_log:
            ctrl_path = self.log_dir / f"control_{timestamp}.csv"
            with open(ctrl_path, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=['step', 'steering', 'throttle', 'brake'])
                writer.writeheader()
                writer.writerows(self.control_log)
            logger.info(f"✅ Control log saved to {ctrl_path}")
        
        # Save predictions
        if self.prediction_log:
            pred_path = self.log_dir / f"predictions_{timestamp}.csv"
            with open(pred_path, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=['step', 'pred_x', 'pred_y', 'pred_yaw', 'pred_velocity'])
                writer.writeheader()
                writer.writerows(self.prediction_log)
            logger.info(f"✅ Prediction log saved to {pred_path}")

