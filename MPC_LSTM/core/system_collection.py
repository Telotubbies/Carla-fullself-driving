"""
Data collection system - collects driving data from CARLA autopilot.

Saves images, ResNet features, vehicle states, and controls
for training the LSTM temporal predictor.
"""

import logging
import csv
import time
import numpy as np
import cv2
from pathlib import Path
from datetime import datetime
from typing import Dict, Any, Optional

from core.validators import ImageValidator, StateValidator
from core.exceptions import DataValidationError

logger = logging.getLogger(__name__)


class DataCollectionSystem:
    """Collects data from CARLA autopilot for LSTM training."""

    def __init__(self, system):
        """
        Args:
            system: Parent AutonomousDrivingSystem instance
        """
        self.system = system
        self.config = system.config
        self.running = False
        self.step_count = 0

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.run_dir = Path(self.config['data_collection']['save_path']) / f"autopilot_{timestamp}"
        self.run_dir.mkdir(parents=True, exist_ok=True)

        self.images_dir = self.run_dir / "images"
        self.images_dir.mkdir(exist_ok=True)

        self.features_dir = self.run_dir / "features"
        self.features_dir.mkdir(exist_ok=True)

        self.csv_path = self.run_dir / "data.csv"
        self.csv_file = open(self.csv_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            'step', 'image_path', 'feature_path',
            'x', 'y', 'yaw', 'velocity',
            'steering', 'throttle', 'brake'
        ])

        logger.info(f"Data collection directory: {self.run_dir}")

    def run(self) -> None:
        """Run data collection loop with CARLA autopilot."""
        import carla
        logger.info("Starting data collection with CARLA autopilot...")
        self.running = True
        self.step_count = 0

        vehicle = self.system.carla_client.vehicle
        if vehicle is None:
            logger.error("No vehicle available for data collection")
            return

        vehicle.set_autopilot(True)
        logger.info("CARLA autopilot enabled")

        try:
            while self.running:
                self.system.carla_client.tick()

                image = self._get_and_validate_image()
                if image is None:
                    time.sleep(0.01)
                    continue

                vehicle_state = self._get_and_validate_state()
                if vehicle_state is None:
                    time.sleep(0.01)
                    continue

                # Skip if vehicle barely moving (first few frames)
                if self.step_count < 20 and vehicle_state['velocity'] < 0.5:
                    self.system.carla_client.tick()
                    continue

                # Save image
                img_rel = f"images/image_{self.step_count:06d}.png"
                img_path = self.run_dir / img_rel
                cv2.imwrite(str(img_path), cv2.cvtColor(image, cv2.COLOR_RGB2BGR))

                # Encode with ResNet and save features
                feat_rel = f"features/feat_{self.step_count:06d}.npy"
                feat_path = self.run_dir / feat_rel
                try:
                    features = self.system.resnet_encoder.encode(image)
                    np.save(str(feat_path), features)
                except Exception as e:
                    if self.step_count % 100 == 0:
                        logger.warning(f"Feature encoding failed: {e}")
                    feat_rel = ''

                # Write CSV row
                self.csv_writer.writerow([
                    self.step_count,
                    img_rel,
                    feat_rel,
                    f"{vehicle_state['x']:.4f}",
                    f"{vehicle_state['y']:.4f}",
                    f"{vehicle_state['yaw']:.4f}",
                    f"{vehicle_state['velocity']:.4f}",
                    f"{vehicle_state.get('steering', 0.0):.4f}",
                    f"{vehicle_state.get('throttle', 0.0):.4f}",
                    f"{vehicle_state.get('brake', 0.0):.4f}",
                ])

                # Visualization
                try:
                    self.system.visualization.update(
                        image, vehicle_state, None, None, None
                    )
                except Exception:
                    pass

                self.step_count += 1

                if self.step_count % 200 == 0:
                    logger.info(
                        f"Collected {self.step_count} samples | "
                        f"speed={vehicle_state['velocity']*3.6:.1f} km/h | "
                        f"steer={vehicle_state.get('steering', 0):.3f}"
                    )

                time.sleep(0.01)

        except KeyboardInterrupt:
            logger.info("Data collection stopped by user")
        except Exception as e:
            logger.error(f"Data collection error: {e}", exc_info=True)
        finally:
            vehicle.set_autopilot(False)
            self._cleanup()

    def _get_and_validate_image(self) -> Optional[np.ndarray]:
        """Get and validate camera image."""
        try:
            image = self.system.camera.get_image()
            ImageValidator.validate(image)
            return image
        except (DataValidationError, Exception):
            return None

    def _get_and_validate_state(self) -> Optional[Dict[str, Any]]:
        """Get and validate vehicle state with control info."""
        try:
            vehicle_state = self.system.carla_client.get_vehicle_state()
            StateValidator.validate(vehicle_state)

            control = self.system.carla_client.vehicle.get_control()
            vehicle_state['steering'] = control.steer
            vehicle_state['throttle'] = control.throttle
            vehicle_state['brake'] = control.brake

            return vehicle_state
        except (DataValidationError, Exception):
            return None

    def _cleanup(self) -> None:
        """Save and close resources."""
        self.running = False
        if hasattr(self, 'csv_file'):
            self.csv_file.close()
        logger.info(
            f"Data collection complete: {self.step_count} samples saved to {self.run_dir}"
        )
