"""
Collect training data using CARLA autopilot.

เก็บข้อมูล: Image, State (x,y,yaw,velocity), Control (steering,throttle,brake)
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
from typing import Dict, Any
import csv
from datetime import datetime
import cv2

# Add project root to path
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

from carla_env import CarlaClient, CameraSensor
from utils.device_utils import get_device_info

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class AutopilotDataCollector:
    """Collect data using CARLA autopilot."""
    
    def __init__(self, config_path: str = "config.yaml"):
        """Initialize data collector."""
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        
        self.carla_client = CarlaClient(self.config['carla'])
        self.camera: CameraSensor = None
        
        # Data storage
        self.data_dir = Path(self.config['data_collection']['save_path'])
        self.data_dir.mkdir(exist_ok=True)
        
        self.running = False
        self.step_count = 0
    
    def initialize_carla(self) -> bool:
        """Initialize CARLA environment."""
        logger.info("Initializing CARLA environment...")
        
        if not self.carla_client.connect():
            return False
        
        if not self.carla_client.load_world():
            return False
        
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
        
        # Enable autopilot with Traffic Manager
        try:
            # Get traffic manager
            traffic_manager = self.carla_client.client.get_trafficmanager()
            tm_port = traffic_manager.get_port()
            
            # Configure traffic manager
            traffic_manager.set_global_distance_to_leading_vehicle(2.5)
            traffic_manager.set_synchronous_mode(True)
            traffic_manager.set_random_device_seed(0)
            
            # Enable autopilot with traffic manager
            self.carla_client.vehicle.set_autopilot(True, tm_port)
            logger.info(f"✅ Autopilot enabled with traffic manager (port {tm_port})")
            
            # Wait for autopilot to initialize and check if vehicle moves
            logger.info("Waiting for autopilot to initialize...")
            moving = False
            for i in range(50):  # Wait up to 5 seconds
                self.carla_client.tick()
                vehicle_state = self.carla_client.get_vehicle_state()
                speed_kmh = vehicle_state['velocity'] * 3.6
                
                if speed_kmh > 1.0:
                    logger.info(f"✅ Vehicle is moving: {speed_kmh:.1f} km/h")
                    moving = True
                    break
                
                if i % 10 == 0:
                    logger.info(f"  Waiting... speed: {speed_kmh:.2f} km/h")
                
                time.sleep(0.1)
            
            if not moving:
                logger.warning("⚠️  Vehicle not moving after initialization")
                logger.warning("   Trying to re-enable autopilot...")
                # Try re-enabling
                self.carla_client.vehicle.set_autopilot(False)
                time.sleep(0.5)
                self.carla_client.vehicle.set_autopilot(True, tm_port)
                logger.warning("   Re-enabled autopilot, continuing...")
        except Exception as e:
            logger.error(f"❌ Failed to enable autopilot: {e}")
            # Try simple autopilot
            try:
                logger.info("Trying simple autopilot...")
                self.carla_client.vehicle.set_autopilot(True)
                logger.info("✅ Autopilot enabled (simple mode)")
                time.sleep(2.0)
            except Exception as e2:
                logger.error(f"❌ Simple autopilot failed: {e2}")
                return False
        
        return True
    
    def _validate_collected_data(self, vehicle_state: Dict[str, Any], image: np.ndarray) -> bool:
        """Validate collected data before saving."""
        # Validate vehicle state
        required_keys = ['x', 'y', 'yaw', 'velocity']
        if not all(key in vehicle_state for key in required_keys):
            return False
        
        # Check for NaN/Inf in state
        for key in required_keys:
            value = vehicle_state[key]
            if not isinstance(value, (int, float)) or np.isnan(value) or np.isinf(value):
                return False
        
        # Check reasonable ranges
        if abs(vehicle_state['x']) > 10000 or abs(vehicle_state['y']) > 10000:
            return False
        if abs(vehicle_state['yaw']) > 360:
            return False
        if vehicle_state['velocity'] < 0 or vehicle_state['velocity'] > 100:
            return False
        
        # Validate image
        if image is None:
            return False
        if not isinstance(image, np.ndarray):
            return False
        if image.shape[0] == 0 or image.shape[1] == 0:
            return False
        if len(image.shape) != 3 or image.shape[2] != 3:
            return False
        
        return True
    
    def collect_data(self, num_frames: int = 20000, output_dir: str = None) -> None:
        """
        Collect data using autopilot.
        
        Args:
            num_frames: Number of frames to collect
            output_dir: Output directory (default: data/autopilot_TIMESTAMP)
        """
        if output_dir is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_dir = self.data_dir / f"autopilot_{timestamp}"
        else:
            output_dir = Path(output_dir)
        
        output_dir.mkdir(exist_ok=True)
        images_dir = output_dir / "images"
        images_dir.mkdir(exist_ok=True)
        
        # CSV file
        csv_path = output_dir / "data.csv"
        csv_file = open(csv_path, 'w', newline='')
        csv_writer = csv.writer(csv_file)
        csv_writer.writerow([
            'step', 'image_path', 'x', 'y', 'yaw', 'velocity',
            'steering', 'throttle', 'brake'
        ])
        
        logger.info(f"Starting data collection: {num_frames} frames")
        logger.info(f"Output directory: {output_dir}")
        
        self.running = True
        start_time = time.time()
        
        try:
            while self.running and self.step_count < num_frames:
                self.carla_client.tick()
                
                # Get image
                image = self.camera.get_image()
                if image is None:
                    time.sleep(0.01)
                    continue
                
                # Get vehicle state
                vehicle_state = self.carla_client.get_vehicle_state()
                
                # Validate data before saving
                if not self._validate_collected_data(vehicle_state, image):
                    if self.step_count % 100 == 0:
                        logger.warning(f"Invalid data at step {self.step_count}, skipping...")
                    continue
                
                # Check if vehicle is moving (safety check)
                if self.step_count > 100 and self.step_count % 500 == 0:
                    speed_kmh = vehicle_state['velocity'] * 3.6
                    if speed_kmh < 0.5:
                        logger.warning(f"Vehicle appears stuck at step {self.step_count}, velocity: {speed_kmh:.2f} km/h")
                        # Try to re-enable autopilot with traffic manager
                        try:
                            traffic_manager = self.carla_client.client.get_trafficmanager()
                            tm_port = traffic_manager.get_port()
                            self.carla_client.vehicle.set_autopilot(False)
                            time.sleep(0.5)
                            self.carla_client.vehicle.set_autopilot(True, tm_port)
                            logger.info("Re-enabled autopilot with traffic manager")
                        except:
                            try:
                                self.carla_client.vehicle.set_autopilot(False)
                                time.sleep(0.5)
                                self.carla_client.vehicle.set_autopilot(True)
                                logger.info("Re-enabled autopilot (simple)")
                            except:
                                pass
                
                # Save image
                image_path = images_dir / f"image_{self.step_count:06d}.png"
                success = cv2.imwrite(str(image_path), cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
                
                # Validate image was saved
                if not success or not image_path.exists():
                    logger.warning(f"Failed to save image at step {self.step_count}")
                    continue
                
                # Save to CSV
                csv_writer.writerow([
                    self.step_count,
                    f"images/image_{self.step_count:06d}.png",
                    vehicle_state['x'],
                    vehicle_state['y'],
                    vehicle_state['yaw'],
                    vehicle_state['velocity'],
                    vehicle_state['steering'],
                    vehicle_state['throttle'],
                    vehicle_state['brake']
                ])
                
                self.step_count += 1
                
                if self.step_count % 100 == 0:
                    elapsed = time.time() - start_time
                    fps = self.step_count / elapsed
                    remaining = (num_frames - self.step_count) / fps if fps > 0 else 0
                    logger.info(
                        f"Collected {self.step_count}/{num_frames} frames "
                        f"({fps:.1f} fps, {remaining:.0f}s remaining)"
                    )
                
                time.sleep(0.01)
        
        except KeyboardInterrupt:
            logger.info("Data collection interrupted by user")
        finally:
            csv_file.close()
            elapsed = time.time() - start_time
            logger.info(f"✅ Data collection complete!")
            logger.info(f"   Total frames: {self.step_count}")
            logger.info(f"   Time: {elapsed:.1f}s")
            logger.info(f"   FPS: {self.step_count/elapsed:.1f}")
            logger.info(f"   Saved to: {output_dir}")
            self.cleanup()
    
    def cleanup(self) -> None:
        """Clean up resources."""
        if self.camera is not None:
            self.camera.destroy()
        self.carla_client.cleanup()


def main():
    parser = argparse.ArgumentParser(description='Collect training data using CARLA autopilot')
    parser.add_argument('--frames', type=int, default=50000, help='Number of frames to collect (default: 50000 for better training)')
    parser.add_argument('--output', type=str, default=None, help='Output directory')
    parser.add_argument('--config', type=str, default='config.yaml', help='Config file')
    
    args = parser.parse_args()
    
    collector = AutopilotDataCollector(config_path=args.config)
    
    if not collector.initialize_carla():
        logger.error("Failed to initialize CARLA")
        return 1
    
    collector.collect_data(num_frames=args.frames, output_dir=args.output)
    
    return 0


if __name__ == '__main__':
    sys.exit(main())

