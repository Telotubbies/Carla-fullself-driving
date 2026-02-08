"""
Collect diverse training data with multiple spawn points and routes.

Based on research papers:
- "End-to-End Learning for Self-Driving Cars" (Bojarski et al., 2016)
- "Learning to Drive in a Day" (Kendall et al., 2019)
- "CARLA: An Open Urban Driving Simulator" (Dosovitskiy et al., 2017)

Strategies:
1. Multiple spawn points (different locations)
2. Multiple routes (different paths)
3. Traffic Manager with random behaviors
4. Different weather conditions
5. Data balancing for steering distribution
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
from typing import Dict, Any, List, Tuple
import csv
from datetime import datetime
import cv2
import random

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


class DiverseDataCollector:
    """Collect diverse data using multiple strategies."""
    
    def __init__(self, config_path: str = "config.yaml"):
        """Initialize diverse data collector."""
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        
        self.carla_client = CarlaClient(self.config['carla'])
        self.camera: CameraSensor = None
        
        # Data storage
        self.data_dir = Path(self.config['data_collection']['save_path'])
        self.data_dir.mkdir(exist_ok=True)
        
        self.running = False
        self.step_count = 0
        
        # Diversity settings
        self.current_spawn_idx = 0
        self.current_route_idx = 0
        self.spawn_points_used = []
        self.steering_distribution = []  # Track steering for balancing
        
        # Statistics
        self.stats = {
            'total_frames': 0,
            'spawn_points_used': set(),
            'steering_left': 0,
            'steering_right': 0,
            'steering_straight': 0,
            'high_steering': 0,  # |steering| > 0.3
        }
    
    def get_all_spawn_points(self) -> List[carla.Transform]:
        """Get all available spawn points in the map."""
        spawn_points = self.carla_client.world.get_map().get_spawn_points()
        logger.info(f"Found {len(spawn_points)} spawn points")
        return spawn_points
    
    def select_diverse_spawn_points(self, num_points: int = 10) -> List[int]:
        """
        Select diverse spawn points.
        
        Strategy: Select points that are far apart to ensure route diversity.
        """
        all_spawns = self.get_all_spawn_points()
        
        if len(all_spawns) < num_points:
            # Use all available spawn points
            return list(range(len(all_spawns)))
        
        # Select diverse spawn points (spread out)
        selected = [0]  # Always include first spawn point
        remaining = list(range(1, len(all_spawns)))
        
        while len(selected) < num_points and remaining:
            # Find spawn point farthest from all selected points
            best_idx = None
            max_min_dist = -1
            
            for candidate in remaining:
                candidate_pos = all_spawns[candidate].location
                min_dist = float('inf')
                
                for selected_idx in selected:
                    selected_pos = all_spawns[selected_idx].location
                    dist = candidate_pos.distance(selected_pos)
                    min_dist = min(min_dist, dist)
                
                if min_dist > max_min_dist:
                    max_min_dist = min_dist
                    best_idx = candidate
            
            if best_idx is not None:
                selected.append(best_idx)
                remaining.remove(best_idx)
            else:
                break
        
        logger.info(f"Selected {len(selected)} diverse spawn points: {selected}")
        return selected
    
    def configure_traffic_manager_for_diversity(self, traffic_manager, aggressiveness: float = None):
        """
        Configure Traffic Manager for diverse behaviors.
        
        Args:
            traffic_manager: CARLA Traffic Manager
            aggressiveness: Random aggressiveness (0.0-1.0)
        """
        if aggressiveness is None:
            aggressiveness = random.uniform(0.3, 0.7)  # Random aggressiveness
        
        # Set diverse behaviors
        traffic_manager.set_global_distance_to_leading_vehicle(random.uniform(1.5, 3.5))
        traffic_manager.set_synchronous_mode(True)
        traffic_manager.set_random_device_seed(random.randint(0, 10000))
        
        # Set aggressiveness (higher = more aggressive)
        traffic_manager.vehicle_percentage_speed_difference(
            self.carla_client.vehicle,
            random.uniform(-20, 20)  # -20% to +20% speed variation
        )
        
        # Set ignore lights probability (for more diverse behavior)
        traffic_manager.ignore_lights_percentage(self.carla_client.vehicle, random.uniform(0, 30))
        
        logger.info(f"Traffic Manager configured: aggressiveness={aggressiveness:.2f}")
    
    def change_weather(self, weather_preset: str = None):
        """Change weather for diversity."""
        if weather_preset is None:
            # Random weather
            weathers = [
                carla.WeatherParameters.ClearNoon,
                carla.WeatherParameters.CloudyNoon,
                carla.WeatherParameters.WetNoon,
                carla.WeatherParameters.WetCloudyNoon,
                carla.WeatherParameters.MidRainyNoon,
                carla.WeatherParameters.HardRainNoon,
                carla.WeatherParameters.SoftRainNoon,
            ]
            weather = random.choice(weathers)
        else:
            weather_map = {
                'clear': carla.WeatherParameters.ClearNoon,
                'cloudy': carla.WeatherParameters.CloudyNoon,
                'wet': carla.WeatherParameters.WetNoon,
                'rain': carla.WeatherParameters.MidRainyNoon,
            }
            weather = weather_map.get(weather_preset, carla.WeatherParameters.ClearNoon)
        
        self.carla_client.world.set_weather(weather)
        logger.info(f"Weather changed to: {weather}")
    
    def initialize_carla(self, spawn_idx: int = None) -> bool:
        """Initialize CARLA environment with specific spawn point."""
        logger.info(f"Initializing CARLA environment (spawn_idx={spawn_idx})...")
        
        if not self.carla_client.connect():
            return False
        
        if not self.carla_client.load_world():
            return False
        
        # Change weather randomly
        if random.random() < 0.3:  # 30% chance to change weather
            self.change_weather()
        
        # Spawn vehicle at specific spawn point
        if spawn_idx is not None:
            spawn_points = self.get_all_spawn_points()
            if spawn_idx < len(spawn_points):
                # Temporarily override spawn point
                original_spawn = self.config['carla']['spawn_point_index']
                self.config['carla']['spawn_point_index'] = spawn_idx
                
                # Destroy existing vehicle if any
                if self.carla_client.vehicle is not None:
                    self.carla_client.vehicle.destroy()
                
                # Spawn at new location
                if not self.carla_client.spawn_vehicle():
                    # Restore original spawn
                    self.config['carla']['spawn_point_index'] = original_spawn
                    return False
                
                logger.info(f"✅ Vehicle spawned at spawn point {spawn_idx}")
            else:
                logger.warning(f"Spawn point {spawn_idx} not available, using default")
                if not self.carla_client.spawn_vehicle():
                    return False
        else:
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
        
        # Enable autopilot with Traffic Manager (diverse behavior)
        try:
            traffic_manager = self.carla_client.client.get_trafficmanager()
            tm_port = traffic_manager.get_port()
            
            # Configure for diversity
            self.configure_traffic_manager_for_diversity(traffic_manager)
            
            # Enable autopilot
            self.carla_client.vehicle.set_autopilot(True, tm_port)
            logger.info(f"✅ Autopilot enabled with diverse traffic manager (port {tm_port})")
            
            # Wait for vehicle to start moving
            logger.info("Waiting for vehicle to start moving...")
            moving = False
            for i in range(50):
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
                logger.warning("⚠️  Vehicle not moving, trying to re-enable...")
                self.carla_client.vehicle.set_autopilot(False)
                time.sleep(0.5)
                self.carla_client.vehicle.set_autopilot(True, tm_port)
                time.sleep(1.0)
        except Exception as e:
            logger.error(f"❌ Failed to enable autopilot: {e}")
            return False
        
        return True
    
    def should_switch_spawn_point(self, current_steering_std: float, min_frames_per_spawn: int = 2000) -> bool:
        """
        Decide if we should switch spawn point for more diversity.
        
        Strategy:
        - Switch if we've collected enough frames at current spawn
        - Switch if steering variance is too low (not enough diversity)
        """
        if self.step_count < min_frames_per_spawn:
            return False
        
        # Check if steering diversity is low
        if len(self.steering_distribution) > 100:
            recent_steering = self.steering_distribution[-100:]
            recent_std = np.std(recent_steering)
            
            if recent_std < 0.05:  # Very low steering variance
                logger.info(f"Low steering diversity (std={recent_std:.4f}), switching spawn point")
                return True
        
        # Switch after collecting enough frames
        if self.step_count % min_frames_per_spawn == 0:
            return True
        
        return False
    
    def _validate_collected_data(self, vehicle_state: Dict[str, Any], image: np.ndarray) -> bool:
        """Validate collected data before saving."""
        required_keys = ['x', 'y', 'yaw', 'velocity']
        if not all(key in vehicle_state for key in required_keys):
            return False
        
        for key in required_keys:
            value = vehicle_state[key]
            if not isinstance(value, (int, float)) or np.isnan(value) or np.isinf(value):
                return False
        
        if abs(vehicle_state['x']) > 10000 or abs(vehicle_state['y']) > 10000:
            return False
        if abs(vehicle_state['yaw']) > 360:
            return False
        if vehicle_state['velocity'] < 0 or vehicle_state['velocity'] > 100:
            return False
        
        if image is None or not isinstance(image, np.ndarray):
            return False
        if image.shape[0] == 0 or image.shape[1] == 0:
            return False
        if len(image.shape) != 3 or image.shape[2] != 3:
            return False
        
        return True
    
    def collect_diverse_data(
        self,
        num_frames: int = 50000,
        num_spawn_points: int = 10,
        frames_per_spawn: int = 5000,
        output_dir: str = None
    ) -> None:
        """
        Collect diverse data using multiple spawn points and routes.
        
        Args:
            num_frames: Total number of frames to collect
            num_spawn_points: Number of diverse spawn points to use
            frames_per_spawn: Frames to collect per spawn point before switching
            output_dir: Output directory
        """
        if output_dir is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            output_dir = self.data_dir / f"diverse_{timestamp}"
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
            'steering', 'throttle', 'brake', 'spawn_idx'
        ])
        
        logger.info(f"Starting DIVERSE data collection: {num_frames} frames")
        logger.info(f"  - Using {num_spawn_points} spawn points")
        logger.info(f"  - {frames_per_spawn} frames per spawn point")
        logger.info(f"Output directory: {output_dir}")
        
        self.running = True
        start_time = time.time()
        last_spawn_switch = 0
        
        try:
            # Initialize CARLA first (without spawn point to get world)
            if not self.carla_client.connect():
                logger.error("Failed to connect to CARLA")
                return
            if not self.carla_client.load_world():
                logger.error("Failed to load world")
                return
            
            # Now select diverse spawn points (world is available)
            spawn_indices = self.select_diverse_spawn_points(num_spawn_points)
            current_spawn_idx = 0
            
            # Initialize with first spawn point
            if not self.initialize_carla(spawn_idx=spawn_indices[current_spawn_idx]):
                logger.error("Failed to initialize CARLA")
                return
            
            self.stats['spawn_points_used'].add(spawn_indices[current_spawn_idx])
            
            while self.running and self.step_count < num_frames:
                self.carla_client.tick()
                
                # Get image
                image = self.camera.get_image()
                if image is None:
                    time.sleep(0.01)
                    continue
                
                # Get vehicle state
                vehicle_state = self.carla_client.get_vehicle_state()
                
                # Validate data
                if not self._validate_collected_data(vehicle_state, image):
                    if self.step_count % 100 == 0:
                        logger.warning(f"Invalid data at step {self.step_count}, skipping...")
                    continue
                
                # Track steering for diversity analysis
                steering = vehicle_state['steering']
                self.steering_distribution.append(steering)
                if len(self.steering_distribution) > 1000:
                    self.steering_distribution.pop(0)  # Keep last 1000
                
                # Update statistics
                if abs(steering) > 0.01:
                    if steering > 0:
                        self.stats['steering_right'] += 1
                    else:
                        self.stats['steering_left'] += 1
                else:
                    self.stats['steering_straight'] += 1
                
                if abs(steering) > 0.3:
                    self.stats['high_steering'] += 1
                
                # Check if we should switch spawn point
                frames_since_switch = self.step_count - last_spawn_switch
                if frames_since_switch >= frames_per_spawn and len(spawn_indices) > 1:
                    # Switch to next spawn point
                    current_spawn_idx = (current_spawn_idx + 1) % len(spawn_indices)
                    new_spawn_idx = spawn_indices[current_spawn_idx]
                    
                    logger.info(f"🔄 Switching to spawn point {new_spawn_idx} (collected {frames_since_switch} frames)")
                    
                    # Reinitialize with new spawn point
                    if self.initialize_carla(spawn_idx=new_spawn_idx):
                        self.stats['spawn_points_used'].add(new_spawn_idx)
                        last_spawn_switch = self.step_count
                        # Change weather occasionally
                        if random.random() < 0.2:  # 20% chance
                            self.change_weather()
                    else:
                        logger.warning(f"Failed to switch to spawn point {new_spawn_idx}, continuing...")
                
                # Check if vehicle is stuck
                if self.step_count > 100 and self.step_count % 500 == 0:
                    speed_kmh = vehicle_state['velocity'] * 3.6
                    if speed_kmh < 0.5:
                        logger.warning(f"Vehicle appears stuck, re-enabling autopilot...")
                        try:
                            traffic_manager = self.carla_client.client.get_trafficmanager()
                            tm_port = traffic_manager.get_port()
                            self.carla_client.vehicle.set_autopilot(False)
                            time.sleep(0.5)
                            self.configure_traffic_manager_for_diversity(traffic_manager)
                            self.carla_client.vehicle.set_autopilot(True, tm_port)
                        except:
                            pass
                
                # Save image
                image_path = images_dir / f"image_{self.step_count:06d}.png"
                success = cv2.imwrite(str(image_path), cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
                
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
                    vehicle_state['brake'],
                    spawn_indices[current_spawn_idx]
                ])
                
                self.step_count += 1
                self.stats['total_frames'] += 1
                
                # Progress logging
                if self.step_count % 500 == 0:
                    elapsed = time.time() - start_time
                    fps = self.step_count / elapsed
                    remaining = (num_frames - self.step_count) / fps if fps > 0 else 0
                    
                    # Steering statistics
                    if len(self.steering_distribution) > 0:
                        steering_std = np.std(self.steering_distribution[-500:])
                        steering_mean = np.mean(np.abs(self.steering_distribution[-500:]))
                    else:
                        steering_std = 0
                        steering_mean = 0
                    
                    logger.info(
                        f"Progress: {self.step_count}/{num_frames} frames "
                        f"({fps:.1f} fps, {remaining:.0f}s remaining) | "
                        f"Spawn: {spawn_indices[current_spawn_idx]} | "
                        f"Steering std: {steering_std:.4f}, mean_abs: {steering_mean:.4f}"
                    )
                
                time.sleep(0.01)
        
        except KeyboardInterrupt:
            logger.info("Data collection interrupted by user")
        finally:
            csv_file.close()
            elapsed = time.time() - start_time
            
            # Final statistics
            logger.info("=" * 60)
            logger.info("✅ Data collection complete!")
            logger.info(f"   Total frames: {self.step_count}")
            logger.info(f"   Time: {elapsed:.1f}s")
            logger.info(f"   FPS: {self.step_count/elapsed:.1f}")
            logger.info(f"   Saved to: {output_dir}")
            logger.info("")
            logger.info("📊 Diversity Statistics:")
            logger.info(f"   Spawn points used: {len(self.stats['spawn_points_used'])}")
            logger.info(f"   Steering left: {self.stats['steering_left']}")
            logger.info(f"   Steering right: {self.stats['steering_right']}")
            logger.info(f"   Steering straight: {self.stats['steering_straight']}")
            logger.info(f"   High steering (|>0.3|): {self.stats['high_steering']}")
            if len(self.steering_distribution) > 0:
                logger.info(f"   Steering std: {np.std(self.steering_distribution):.4f}")
                logger.info(f"   Steering mean abs: {np.mean(np.abs(self.steering_distribution)):.4f}")
            logger.info("=" * 60)
            
            self.cleanup()
    
    def cleanup(self) -> None:
        """Clean up resources."""
        if self.camera is not None:
            self.camera.destroy()
        self.carla_client.cleanup()


def main():
    parser = argparse.ArgumentParser(
        description='Collect diverse training data using multiple spawn points and routes'
    )
    parser.add_argument('--frames', type=int, default=50000, help='Total number of frames to collect')
    parser.add_argument('--spawn-points', type=int, default=10, help='Number of diverse spawn points')
    parser.add_argument('--frames-per-spawn', type=int, default=5000, help='Frames per spawn point')
    parser.add_argument('--output', type=str, default=None, help='Output directory')
    parser.add_argument('--config', type=str, default='config.yaml', help='Config file')
    
    args = parser.parse_args()
    
    collector = DiverseDataCollector(config_path=args.config)
    
    collector.collect_diverse_data(
        num_frames=args.frames,
        num_spawn_points=args.spawn_points,
        frames_per_spawn=args.frames_per_spawn,
        output_dir=args.output
    )
    
    return 0


if __name__ == '__main__':
    sys.exit(main())

