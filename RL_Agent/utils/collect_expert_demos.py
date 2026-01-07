"""
Collect Expert Demonstrations from CARLA Agents
ใช้ BasicAgent, BehaviorAgent, หรือ ConstantVelocityAgent เป็น expert
"""

import carla
import numpy as np
import cv2
import argparse
import os
import json
import time
import logging
from collections import deque
from typing import Dict, List, Tuple, Optional
import sys

# Add CARLA PythonAPI to path
# Try multiple possible paths
possible_paths = [
    '/home/a/Desktop/CARLA_0.9.16/PythonAPI',
    os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), '..', 'PythonAPI'),
    os.path.join(os.path.dirname(__file__), '..', '..', '..', 'PythonAPI'),
]

carla_api_path = None
for path in possible_paths:
    abs_path = os.path.abspath(path)
    if os.path.exists(abs_path):
        carla_api_path = abs_path
        break

if carla_api_path:
    sys.path.insert(0, carla_api_path)
    # Also add the examples path for agents
    examples_path = os.path.join(carla_api_path, 'examples')
    if os.path.exists(examples_path):
        sys.path.insert(0, examples_path)

try:
    if carla_api_path:
        # Add all necessary paths for agents to work
        agents_base = os.path.join(carla_api_path, 'carla', 'agents')
        agents_nav = os.path.join(agents_base, 'navigation')
        agents_tools = os.path.join(agents_base, 'tools')
        
        # Add to path in correct order
        sys.path.insert(0, agents_tools)
        sys.path.insert(0, agents_nav)
        sys.path.insert(0, agents_base)
        sys.path.insert(0, os.path.join(carla_api_path, 'carla'))
        
        # Now import
        from agents.navigation.basic_agent import BasicAgent
        from agents.navigation.behavior_agent import BehaviorAgent
        from agents.navigation.constant_velocity_agent import ConstantVelocityAgent
        
        AGENTS_AVAILABLE = True
    else:
        raise ImportError("CARLA API path not found")
        
except Exception as e:
    AGENTS_AVAILABLE = False
    logging.warning(f"CARLA agents not available: {e}")
    if carla_api_path:
        logging.warning(f"  CARLA API path: {carla_api_path}")
    logging.warning(f"  Make sure CARLA PythonAPI is installed and shapely is available")
    logging.warning(f"  Install shapely: pip install shapely")


class ExpertDemonstrationCollector:
    """
    Collect expert demonstrations from CARLA agents
    """
    
    def __init__(
        self,
        host: str = 'localhost',
        port: int = 2000,
        town: str = 'Town01_Opt',
        agent_type: str = 'Basic',
        image_size: Tuple[int, int] = (160, 90),
        num_episodes: int = 100,
        max_steps_per_episode: int = 1000,
        output_dir: str = 'expert_demonstrations'
    ):
        """
        Initialize collector
        
        Args:
            host: CARLA server host
            port: CARLA server port
            town: Town name
            agent_type: 'Basic', 'Behavior', or 'Constant'
            image_size: (width, height) for images
            num_episodes: Number of episodes to collect
            max_steps_per_episode: Maximum steps per episode
            output_dir: Output directory for demonstrations
        """
        if not AGENTS_AVAILABLE:
            raise ImportError("CARLA agents not available. Make sure CARLA PythonAPI is in path.")
        
        self.host = host
        self.port = port
        self.town = town
        self.agent_type = agent_type
        self.image_size = image_size
        self.num_episodes = num_episodes
        self.max_steps_per_episode = max_steps_per_episode
        self.output_dir = output_dir
        
        # Create output directory
        os.makedirs(output_dir, exist_ok=True)
        os.makedirs(os.path.join(output_dir, 'images'), exist_ok=True)
        
        # CARLA objects
        self.client = None
        self.world = None
        self.map = None
        self.vehicle = None
        self.agent = None
        self.rgb_camera = None
        self.depth_camera = None
        self.collision_sensor = None
        
        # Data collection
        self.demonstrations = []
        self.current_episode = []
        
        # Sensor data
        self.rgb_image = None
        self.depth_image = None
        self.collision_detected = False
        
        logging.basicConfig(level=logging.INFO)
        self.logger = logging.getLogger(__name__)
    
    def connect_to_carla(self):
        """Connect to CARLA server with retry logic"""
        self.logger.info(f"Connecting to CARLA at {self.host}:{self.port}")
        
        # Retry connection
        max_retries = 5
        for attempt in range(max_retries):
            try:
                self.client = carla.Client(self.host, self.port)
                self.client.set_timeout(10.0)  # Shorter timeout for faster retry
                
                # Get world first (faster than load_world)
                self.world = self.client.get_world()
                current_map = self.world.get_map().name
                
                # Check if we need to load different town
                target_map = f"Carla/Maps/{self.town}"
                if current_map != target_map:
                    self.logger.info(f"Current map: {current_map}, loading {self.town}...")
                    # Wait a bit for CARLA to be ready
                    import time
                    time.sleep(2)
                    try:
                        self.world = self.client.load_world(self.town, reset_settings=False)
                        self.logger.info(f"✅ Loaded town {self.town}")
                    except RuntimeError as e:
                        if "time-out" in str(e).lower():
                            self.logger.warning(f"Load world timeout (attempt {attempt + 1}/{max_retries}), retrying...")
                            if attempt < max_retries - 1:
                                time.sleep(5)
                                continue
                            else:
                                raise
                        else:
                            raise
                else:
                    self.logger.info(f"✅ Using current town: {self.town}")
                
                self.map = self.world.get_map()
                
                # Set synchronous mode
                settings = self.world.get_settings()
                settings.synchronous_mode = True
                settings.fixed_delta_seconds = 0.05  # 20 FPS
                self.world.apply_settings(settings)
                
                self.logger.info("✅ Connected to CARLA")
                return  # Success
                
            except RuntimeError as e:
                if "time-out" in str(e).lower() or "timeout" in str(e).lower():
                    if attempt < max_retries - 1:
                        wait_time = (attempt + 1) * 3
                        self.logger.warning(f"Connection timeout (attempt {attempt + 1}/{max_retries}), retrying in {wait_time}s...")
                        import time
                        time.sleep(wait_time)
                        continue
                    else:
                        self.logger.error(f"Failed to connect after {max_retries} attempts")
                        raise
                else:
                    raise
            except Exception as e:
                if attempt < max_retries - 1:
                    self.logger.warning(f"Connection error (attempt {attempt + 1}/{max_retries}): {e}, retrying...")
                    import time
                    time.sleep(3)
                    continue
                else:
                    raise
        
        raise RuntimeError(f"Failed to connect to CARLA after {max_retries} attempts")
    
    def setup_vehicle_and_agent(self):
        """Spawn vehicle and setup agent with retry logic"""
        # Get spawn points
        spawn_points = self.map.get_spawn_points()
        if len(spawn_points) == 0:
            raise RuntimeError("No spawn points available")
        
        # Spawn vehicle with retry logic
        blueprint_library = self.world.get_blueprint_library()
        vehicle_bp = blueprint_library.find('vehicle.tesla.model3')
        
        max_retries = 10
        for attempt in range(max_retries):
            try:
                spawn_point = np.random.choice(spawn_points)
                self.vehicle = self.world.spawn_actor(vehicle_bp, spawn_point)
                self.logger.info(f"Spawned vehicle: {self.vehicle.id} (attempt {attempt + 1})")
                break
            except RuntimeError as e:
                if "collision" in str(e).lower() or "spawn" in str(e).lower():
                    if attempt < max_retries - 1:
                        self.logger.warning(f"Spawn collision, retrying... (attempt {attempt + 1}/{max_retries})")
                        time.sleep(0.1)
                        continue
                    else:
                        raise RuntimeError(f"Failed to spawn vehicle after {max_retries} attempts: {e}")
                else:
                    raise
        
        # Create agent
        if self.agent_type == 'Basic':
            self.agent = BasicAgent(self.vehicle, target_speed=30.0)
            self.agent.follow_speed_limits(True)
        elif self.agent_type == 'Behavior':
            self.agent = BehaviorAgent(self.vehicle, behavior='normal')
        elif self.agent_type == 'Constant':
            self.agent = ConstantVelocityAgent(self.vehicle, target_speed=30.0)
            self.agent.follow_speed_limits(True)
        else:
            raise ValueError(f"Unknown agent type: {self.agent_type}")
        
        # Set destination
        destination = np.random.choice(spawn_points).location
        self.agent.set_destination(destination)
        self.logger.info(f"Agent destination set: {destination}")
    
    def setup_sensors(self):
        """Setup RGB, Depth cameras, and Collision sensor"""
        # RGB Camera
        rgb_bp = self.world.get_blueprint_library().find('sensor.camera.rgb')
        rgb_bp.set_attribute('image_size_x', str(self.image_size[0]))
        rgb_bp.set_attribute('image_size_y', str(self.image_size[1]))
        rgb_bp.set_attribute('fov', '110')
        
        rgb_transform = carla.Transform(carla.Location(x=2.5, z=0.7))
        self.rgb_camera = self.world.spawn_actor(rgb_bp, rgb_transform, attach_to=self.vehicle)
        self.rgb_camera.listen(lambda image: self._process_rgb_image(image))
        
        # Depth Camera
        depth_bp = self.world.get_blueprint_library().find('sensor.camera.depth')
        depth_bp.set_attribute('image_size_x', str(self.image_size[0]))
        depth_bp.set_attribute('image_size_y', str(self.image_size[1]))
        depth_bp.set_attribute('fov', '110')
        
        depth_transform = carla.Transform(carla.Location(x=2.5, z=0.7))
        self.depth_camera = self.world.spawn_actor(depth_bp, depth_transform, attach_to=self.vehicle)
        self.depth_camera.listen(lambda image: self._process_depth_image(image))
        
        # Collision Sensor
        collision_bp = self.world.get_blueprint_library().find('sensor.other.collision')
        collision_transform = carla.Transform()
        self.collision_sensor = self.world.spawn_actor(collision_bp, collision_transform, attach_to=self.vehicle)
        self.collision_sensor.listen(lambda event: self._on_collision(event))
        
        self.logger.info("✅ Sensors setup complete")
    
    def _on_collision(self, event):
        """Handle collision event"""
        self.collision_detected = True
        impulse = event.normal_impulse
        intensity = np.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        self.logger.warning(f"Collision detected: intensity={intensity:.2f}")
    
    def _process_rgb_image(self, image):
        """Process RGB image"""
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]  # Remove alpha
        array = array[:, :, ::-1]  # BGR to RGB
        self.rgb_image = array.astype(np.float32) / 255.0
    
    def _process_depth_image(self, image):
        """Process depth image"""
        array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
        array = np.reshape(array, (image.height, image.width, 4))
        array = array[:, :, :3]  # Use RGB as depth encoding
        # Convert to actual depth (meters)
        depth = (array[:, :, 0] + array[:, :, 1] * 256.0 + array[:, :, 2] * 256.0 * 256.0) / (256.0 * 256.0 * 256.0 - 1.0) * 1000.0
        self.depth_image = depth.astype(np.float32).reshape(depth.shape[0], depth.shape[1], 1) / 1000.0  # Normalize to [0, 1]
    
    def collect_episode(self, episode_idx: int) -> List[Dict]:
        """Collect one episode of demonstrations"""
        self.logger.info(f"Collecting episode {episode_idx + 1}/{self.num_episodes}")
        
        # Reset collision flag
        self.collision_detected = False
        
        # Reset
        try:
            self.setup_vehicle_and_agent()
            self.setup_sensors()
        except RuntimeError as e:
            self.logger.error(f"Failed to setup episode {episode_idx + 1}: {e}")
            return []  # Return empty episode
        
        episode_data = []
        step = 0
        done = False
        
        # Wait for sensors
        time.sleep(1.0)
        
        while step < self.max_steps_per_episode and not done:
            # Tick world
            self.world.tick()
            
            # Wait for sensor data
            wait_count = 0
            while (self.rgb_image is None or self.depth_image is None) and wait_count < 50:
                time.sleep(0.01)
                wait_count += 1
            
            if self.rgb_image is None or self.depth_image is None:
                self.logger.warning(f"Step {step}: Sensor data not available")
                break
            
            # Get agent action
            control = self.agent.run_step()
            action = np.array([
                control.steer,
                control.throttle,
                control.brake
            ], dtype=np.float32)
            
            # Get vehicle state (เพิ่มข้อมูลสำคัญ)
            transform = self.vehicle.get_transform()
            location = transform.location
            rotation = transform.rotation
            velocity = self.vehicle.get_velocity()
            acceleration = self.vehicle.get_acceleration()
            angular_velocity = self.vehicle.get_angular_velocity()
            
            speed = 3.6 * np.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2)
            
            # Get additional vehicle info
            vehicle_location = np.array([location.x, location.y, location.z], dtype=np.float32)
            vehicle_rotation = np.array([rotation.pitch, rotation.yaw, rotation.roll], dtype=np.float32)
            vehicle_velocity = np.array([velocity.x, velocity.y, velocity.z], dtype=np.float32)
            vehicle_acceleration = np.array([acceleration.x, acceleration.y, acceleration.z], dtype=np.float32)
            vehicle_angular_velocity = np.array([angular_velocity.x, angular_velocity.y, angular_velocity.z], dtype=np.float32)
            
            # Get action components (แยกเป็น individual values)
            steer = float(control.steer)
            throttle = float(control.throttle)
            brake = float(control.brake)
            
            # Check if done
            if self.agent.done():
                done = True
                self.logger.info(f"Episode {episode_idx + 1} completed: reached destination")
            
            # Check collision using collision sensor
            collision = self.collision_detected
            if collision:
                done = True
                self.logger.warning(f"Episode {episode_idx + 1} ended: collision")
            
            # Store data (เพิ่มข้อมูลสำคัญ)
            data = {
                'rgb': self.rgb_image.copy(),
                'depth': self.depth_image.copy(),
                'action': action,  # [steer, throttle, brake] array
                'steer': steer,  # พวงมาลัย (แยก)
                'throttle': throttle,  # ทอก (แยก)
                'brake': brake,  # เบรก (แยก)
                'speed': speed,  # ความเร็ว (km/h)
                'location': vehicle_location,  # ตำแหน่ง (x, y, z)
                'rotation': vehicle_rotation,  # การหมุน (pitch, yaw, roll)
                'velocity': vehicle_velocity,  # ความเร็วเวกเตอร์ (m/s)
                'acceleration': vehicle_acceleration,  # ความเร่ง (m/s²)
                'angular_velocity': vehicle_angular_velocity,  # ความเร็วเชิงมุม (rad/s)
                'done': done,
                'collision': collision
            }
            episode_data.append(data)
            
            # Apply control
            self.vehicle.apply_control(control)
            
            step += 1
        
        # Cleanup
        if self.collision_sensor:
            try:
                self.collision_sensor.destroy()
            except:
                pass
        if self.rgb_camera:
            try:
                self.rgb_camera.destroy()
            except:
                pass
        if self.depth_camera:
            try:
                self.depth_camera.destroy()
            except:
                pass
        if self.vehicle:
            try:
                self.vehicle.destroy()
            except:
                pass
        
        self.logger.info(f"Episode {episode_idx + 1} collected: {len(episode_data)} steps")
        return episode_data
    
    def collect_all(self):
        """Collect all demonstrations"""
        self.logger.info(f"Starting collection: {self.num_episodes} episodes")
        
        self.connect_to_carla()
        
        all_episodes = []
        for episode_idx in range(self.num_episodes):
            try:
                episode_data = self.collect_episode(episode_idx)
                all_episodes.append(episode_data)
                
                # Save periodically
                if (episode_idx + 1) % 10 == 0:
                    self.save_demonstrations(all_episodes, episode_idx + 1)
            except Exception as e:
                self.logger.error(f"Error in episode {episode_idx + 1}: {e}", exc_info=True)
                continue
        
        # Final save
        self.save_demonstrations(all_episodes, self.num_episodes)
        
        self.logger.info(f"✅ Collection complete: {len(all_episodes)} episodes")
    
    def save_demonstrations(self, episodes: List[List[Dict]], num_episodes: int):
        """Save demonstrations to disk"""
        # Save metadata
        metadata = {
            'num_episodes': num_episodes,
            'agent_type': self.agent_type,
            'image_size': self.image_size,
            'total_steps': sum(len(ep) for ep in episodes)
        }
        
        metadata_path = os.path.join(self.output_dir, 'metadata.json')
        with open(metadata_path, 'w') as f:
            json.dump(metadata, f, indent=2)
        
        # Save episodes (as numpy arrays for efficiency)
        episodes_path = os.path.join(self.output_dir, f'demonstrations_{num_episodes}episodes.npz')
        
        # Convert to arrays (เพิ่มข้อมูลสำคัญ)
        all_rgb = []
        all_depth = []
        all_actions = []
        all_steers = []
        all_throttles = []
        all_brakes = []
        all_speeds = []
        all_locations = []
        all_rotations = []
        all_velocities = []
        all_accelerations = []
        all_angular_velocities = []
        all_dones = []
        all_collisions = []
        
        for episode in episodes:
            for step_data in episode:
                all_rgb.append(step_data['rgb'])
                all_depth.append(step_data['depth'])
                all_actions.append(step_data['action'])
                # เก็บแยกเป็น individual values
                all_steers.append(step_data.get('steer', step_data['action'][0]))
                all_throttles.append(step_data.get('throttle', step_data['action'][1]))
                all_brakes.append(step_data.get('brake', step_data['action'][2]))
                all_speeds.append(step_data['speed'])
                # ข้อมูลเพิ่มเติม
                all_locations.append(step_data.get('location', np.array([0.0, 0.0, 0.0])))
                all_rotations.append(step_data.get('rotation', np.array([0.0, 0.0, 0.0])))
                all_velocities.append(step_data.get('velocity', np.array([0.0, 0.0, 0.0])))
                all_accelerations.append(step_data.get('acceleration', np.array([0.0, 0.0, 0.0])))
                all_angular_velocities.append(step_data.get('angular_velocity', np.array([0.0, 0.0, 0.0])))
                all_dones.append(step_data['done'])
                all_collisions.append(step_data['collision'])
        
        np.savez_compressed(
            episodes_path,
            rgb=np.array(all_rgb),
            depth=np.array(all_depth),
            actions=np.array(all_actions),  # [steer, throttle, brake] array
            steers=np.array(all_steers),  # พวงมาลัย (แยก)
            throttles=np.array(all_throttles),  # ทอก (แยก)
            brakes=np.array(all_brakes),  # เบรก (แยก)
            speeds=np.array(all_speeds),  # ความเร็ว (km/h)
            locations=np.array(all_locations),  # ตำแหน่ง (x, y, z)
            rotations=np.array(all_rotations),  # การหมุน (pitch, yaw, roll)
            velocities=np.array(all_velocities),  # ความเร็วเวกเตอร์ (m/s)
            accelerations=np.array(all_accelerations),  # ความเร่ง (m/s²)
            angular_velocities=np.array(all_angular_velocities),  # ความเร็วเชิงมุม (rad/s)
            dones=np.array(all_dones),
            collisions=np.array(all_collisions)
        )
        
        self.logger.info(f"✅ Saved {num_episodes} episodes to {episodes_path}")


def main():
    parser = argparse.ArgumentParser(description='Collect expert demonstrations from CARLA agents')
    parser.add_argument('--host', type=str, default='localhost', help='CARLA server host')
    parser.add_argument('--port', type=int, default=2000, help='CARLA server port')
    parser.add_argument('--town', type=str, default='Town01_Opt', help='Town name')
    parser.add_argument('--agent', type=str, default='Basic', choices=['Basic', 'Behavior', 'Constant'],
                       help='Agent type')
    parser.add_argument('--num-episodes', type=int, default=100, help='Number of episodes')
    parser.add_argument('--max-steps', type=int, default=1000, help='Max steps per episode')
    parser.add_argument('--output-dir', type=str, default='expert_demonstrations', help='Output directory')
    parser.add_argument('--image-size', type=int, nargs=2, default=[160, 90], help='Image size (width height)')
    
    args = parser.parse_args()
    
    collector = ExpertDemonstrationCollector(
        host=args.host,
        port=args.port,
        town=args.town,
        agent_type=args.agent,
        image_size=tuple(args.image_size),
        num_episodes=args.num_episodes,
        max_steps_per_episode=args.max_steps,
        output_dir=args.output_dir
    )
    
    collector.collect_all()


if __name__ == '__main__':
    main()

