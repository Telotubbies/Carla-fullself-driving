#!/usr/bin/env python3
"""
2D Demo Simulator for RL Agent
Lightweight 2D visualization using model predictions
"""

import numpy as np
import json
from pathlib import Path
import sys
import os

# Add parent directory to path
sys.path.insert(0, str(Path(__file__).parent.parent))

try:
    from stable_baselines3 import PPO
    import torch
except ImportError:
    print("Warning: stable_baselines3 not available, demo will use mock data")
    PPO = None
    torch = None


class Simple2DSimulator:
    """Simple 2D car simulator for demo"""
    
    def __init__(self, model_path=None, config_path=None):
        self.model = None
        self.model_path = model_path
        self.config_path = config_path
        self.device = None
        
        # Simple 2D state
        self.reset()
        
        # Track if using trained model for evaluation
        self.using_model = False
        
        # Load model if available (for evaluation)
        if model_path and PPO and os.path.exists(model_path):
            try:
                self.load_model(model_path)
                print(f"✅ Loaded trained RL model for evaluation: {model_path}")
            except Exception as e:
                print(f"⚠️ Warning: Could not load model for evaluation: {e}")
                print(f"   Will use Advanced AI logic instead")
                self.model = None
        else:
            if model_path:
                print(f"⚠️ Model path not found: {model_path}")
                print(f"   Will use Advanced AI logic instead")
            self.model = None
    
    def load_model(self, model_path):
        """Load trained model"""
        if not PPO:
            return
        
        try:
            # Try to determine device
            if torch and torch.cuda.is_available():
                self.device = torch.device('cuda:0')
            else:
                self.device = torch.device('cpu')
            
            # Load model (without env for inference only)
            self.model = PPO.load(model_path, device=self.device)
            print(f"✅ Model loaded from {model_path}")
        except Exception as e:
            print(f"❌ Failed to load model: {e}")
            self.model = None
    
    def reset(self):
        """Reset simulator state"""
        # Simple 2D state: [x, y, angle, speed]
        self.x = 0.0
        self.y = 0.0
        self.angle = 0.0  # radians
        self.speed = 0.0
        self.step_count = 0
        self.total_reward = 0.0
        self.done = False
        
        # Multi-lane road configuration
        self.num_lanes = 4  # 4 lanes (left to right: -1.5, -0.5, 0.5, 1.5)
        self.lane_width = 3.0  # Width of each lane (increased for better visualization)
        self.road_width = self.num_lanes * self.lane_width  # Total road width: 12.0
        self.road_length = 200.0  # Longer road for better demo
        
        # Lane centers (y positions) - adjusted for wider lanes
        self.lane_centers = [
            -4.5,  # Leftmost lane (y = -1.5 * 3)
            -1.5,  # Left center lane (y = -0.5 * 3)
            1.5,   # Right center lane (y = 0.5 * 3)
            4.5    # Rightmost lane (y = 1.5 * 3)
        ]
        
        # Current lane index (0-3)
        self.current_lane = 1  # Start in left center lane
        
        # Obstacles distributed across multiple lanes - more challenging
        import random
        random.seed(42)  # For reproducibility
        
        self.obstacles = []
        # Create obstacles in different lanes at various distances
        for i in range(12):
            lane_idx = random.randint(0, self.num_lanes - 1)
            x_pos = 20 + i * 15 + random.uniform(-3, 3)
            y_pos = self.lane_centers[lane_idx] + random.uniform(-0.5, 0.5)
            self.obstacles.append({
                'x': x_pos,
                'y': y_pos,
                'radius': 0.8,  # Smaller obstacles relative to lane width
                'lane': lane_idx
            })
    
    def get_observation(self):
        """Get current observation (simplified for 2D with multi-lane support)"""
        # Enhanced observation: distance to obstacles per lane, lane position, speed, angle
        obs = np.zeros(12, dtype=np.float32)
        
        # Find nearest obstacle in each lane ahead
        for lane_idx in range(self.num_lanes):
            min_dist = 50.0
            for obs_obj in self.obstacles:
                dx = obs_obj['x'] - self.x
                dy = obs_obj['y'] - self.y
                # Only consider obstacles ahead and in this lane
                if dx > 0 and abs(obs_obj['y'] - self.lane_centers[lane_idx]) < self.lane_width:
                    dist = np.sqrt(dx*dx + dy*dy)
                    if dist < min_dist:
                        min_dist = dist
            obs[lane_idx] = min_dist / 50.0  # Normalized distance per lane
        
        # Current lane position (which lane am I in?)
        obs[4] = (self.y + self.road_width/2) / self.road_width  # Normalized lane position
        
        # Road boundaries (left/right)
        obs[5] = (self.y + self.road_width/2) / self.road_width  # Left boundary
        obs[6] = (self.road_width/2 - self.y) / self.road_width  # Right boundary
        
        # Speed (normalized)
        obs[7] = self.speed / 20.0
        
        # Angle (normalized)
        obs[8] = self.angle / np.pi
        
        # Distance along road
        obs[9] = self.x / self.road_length
        
        # Simple features
        obs[10] = 1.0 if self.x > self.road_length * 0.5 else 0.0
        obs[11] = 1.0 if self.speed > 10.0 else 0.0
        
        return obs.reshape(1, -1)  # Add batch dimension
    
    def step(self, action=None):
        """Step simulation - Uses trained RL model for evaluation"""
        if self.done:
            return self.get_state()
        
        # Priority 1: Use trained RL model for evaluation (if available)
        if action is None and self.model:
            try:
                obs = self.get_observation()
                # Use model.predict() for evaluation (deterministic=True for consistent results)
                action, _ = self.model.predict(obs, deterministic=True)
                if isinstance(action, np.ndarray) and action.ndim > 1:
                    action = action[0]  # Remove batch dimension
                # Ensure action is array-like
                if not isinstance(action, (list, np.ndarray)):
                    action = [action] if hasattr(action, '__iter__') else [0.0, 0.5, 0.0]
                action = np.array(action, dtype=np.float32)
                # Ensure 3D action [steer, throttle, brake]
                if len(action) < 3:
                    action = np.pad(action, (0, 3 - len(action)), 'constant')
                # Mark that we're using the trained model
                self.using_model = True
            except Exception as e:
                # Fallback to Advanced AI if model fails
                print(f"⚠️ Model prediction failed: {e}, falling back to Advanced AI")
                self.using_model = False
                action = None  # Will trigger Advanced AI logic below
        else:
            self.using_model = False
        
        # Priority 2: Fallback to Advanced AI if no model or model failed
        if action is None:
            # Full Self-Driving AI: Advanced multi-lane autonomous driving logic
            # 1. Lane Detection: Determine current lane
            # 2. Lane Change Decision: Find best lane to avoid obstacles
            # 3. Lane Keeping: Stay in chosen lane
            # 4. Obstacle Avoidance: Steer around obstacles
            # 5. Speed Control: Adjust speed based on conditions
            
            # Determine current lane
            current_lane_idx = 0
            min_lane_dist = float('inf')
            for i, lane_center in enumerate(self.lane_centers):
                dist = abs(self.y - lane_center)
                if dist < min_lane_dist:
                    min_lane_dist = dist
                    current_lane_idx = i
            self.current_lane = current_lane_idx
            
            # Find obstacles ahead in each lane
            lane_obstacles = {}  # {lane_idx: min_distance}
            for lane_idx in range(self.num_lanes):
                min_dist = 100.0
                for obs_obj in self.obstacles:
                    dx = obs_obj['x'] - self.x
                    dy = obs_obj['y'] - self.y
                    # Only consider obstacles ahead and in this lane
                    if dx > 0 and dx < 30.0 and abs(obs_obj['y'] - self.lane_centers[lane_idx]) < self.lane_width * 0.8:
                        dist = np.sqrt(dx*dx + dy*dy)
                        if dist < min_dist:
                            min_dist = dist
                lane_obstacles[lane_idx] = min_dist
            
            # Lane change decision: find best lane (furthest obstacle)
            best_lane = current_lane_idx
            best_lane_dist = lane_obstacles[current_lane_idx]
            
            # Check adjacent lanes
            if current_lane_idx > 0:  # Can go left
                if lane_obstacles[current_lane_idx - 1] > best_lane_dist + 5.0:
                    best_lane = current_lane_idx - 1
                    best_lane_dist = lane_obstacles[current_lane_idx - 1]
            
            if current_lane_idx < self.num_lanes - 1:  # Can go right
                if lane_obstacles[current_lane_idx + 1] > best_lane_dist + 5.0:
                    best_lane = current_lane_idx + 1
                    best_lane_dist = lane_obstacles[current_lane_idx + 1]
            
            # If obstacle is very close in current lane, prioritize lane change
            if lane_obstacles[current_lane_idx] < 15.0:
                # Urgent lane change needed
                if current_lane_idx > 0 and lane_obstacles[current_lane_idx - 1] > lane_obstacles[current_lane_idx]:
                    best_lane = current_lane_idx - 1
                elif current_lane_idx < self.num_lanes - 1 and lane_obstacles[current_lane_idx + 1] > lane_obstacles[current_lane_idx]:
                    best_lane = current_lane_idx + 1
            
            # Lane keeping: steer towards target lane center
            target_lane_center = self.lane_centers[best_lane]
            y_offset = self.y - target_lane_center
            lane_keeping_steer = -y_offset * 0.4  # Proportional control
            
            # Obstacle avoidance: fine-tune steering if obstacle is very close
            min_dist = 50.0
            nearest_obs_x = 0
            nearest_obs_y = 0
            obstacle_ahead = False
            
            for obs_obj in self.obstacles:
                dx = obs_obj['x'] - self.x
                dy = obs_obj['y'] - self.y
                dist = np.sqrt(dx*dx + dy*dy)
                
                # Only consider obstacles ahead (within 45 degree cone)
                if dx > 0 and dx < 25.0 and abs(dy) < dx * 0.5:
                    if dist < min_dist:
                        min_dist = dist
                        nearest_obs_x = obs_obj['x']
                        nearest_obs_y = obs_obj['y']
                        obstacle_ahead = True
            
            # Fine-tune steering for immediate obstacle avoidance
            avoid_steer = 0.0
            if obstacle_ahead and min_dist < 12.0:
                # Emergency avoidance
                if nearest_obs_y > self.y:
                    avoid_steer = -0.8 * (1.0 - min_dist / 12.0)
                else:
                    avoid_steer = 0.8 * (1.0 - min_dist / 12.0)
            
            # Combine lane keeping and obstacle avoidance
            if obstacle_ahead and min_dist < 12.0:
                # Priority: immediate obstacle avoidance
                steer = avoid_steer * 0.6 + lane_keeping_steer * 0.4
            else:
                # Priority: lane keeping / lane change
                steer = lane_keeping_steer
            
            # Clamp steering
            steer = np.clip(steer, -1.0, 1.0)
            
            # Speed control: adaptive speed based on conditions
            # Base speed: maintain 10-15 m/s
            target_speed = 12.0
            
            # Slow down if obstacle is close in current lane
            if lane_obstacles[current_lane_idx] < 20.0:
                target_speed = max(6.0, lane_obstacles[current_lane_idx] * 0.4)
            
            # Slow down if changing lanes (smooth lane change)
            if best_lane != current_lane_idx:
                target_speed = max(8.0, target_speed * 0.8)
            
            # Speed control: throttle or brake to reach target speed
            speed_error = target_speed - self.speed
            if speed_error > 0.5:
                throttle = min(0.8, speed_error / 5.0)
                brake = 0.0
            elif speed_error < -0.5:
                throttle = 0.0
                brake = min(0.5, abs(speed_error) / 5.0)
            else:
                # Maintain speed
                throttle = 0.3
                brake = 0.0
            
            # Ensure minimum forward movement
            if self.speed < 2.0:
                throttle = max(throttle, 0.5)
            
            action = np.array([steer, throttle, brake], dtype=np.float32)
        
        # Simple physics
        steer = float(action[0])  # -1 to 1
        throttle = float(action[1])  # 0 to 1
        brake = float(action[2])  # 0 to 1
        
        # Update angle
        self.angle += steer * 0.1
        
        # Update speed
        if throttle > 0:
            self.speed = min(self.speed + throttle * 0.5, 20.0)
        if brake > 0:
            self.speed = max(self.speed - brake * 1.0, 0.0)
        if throttle == 0 and brake == 0:
            self.speed *= 0.95  # Friction
        
        # Update position
        self.x += self.speed * np.cos(self.angle) * 0.1
        self.y += self.speed * np.sin(self.angle) * 0.1
        
        # Simple reward
        reward = 0.1  # Base reward for moving
        if self.speed > 5.0:
            reward += 0.1  # Bonus for speed
        
        # Penalty for going off road
        if abs(self.y) > self.road_width / 2:
            reward -= 0.5
        
        # Bonus for staying in lane center
        current_lane_idx = 0
        min_lane_dist = float('inf')
        for i, lane_center in enumerate(self.lane_centers):
            dist = abs(self.y - lane_center)
            if dist < min_lane_dist:
                min_lane_dist = dist
                current_lane_idx = i
        
        if min_lane_dist < self.lane_width * 0.3:
            reward += 0.05  # Bonus for staying in lane center
        
        # Penalty for collision
        # Car size: width ~0.6 * lane_width, length ~1.8 * width
        car_width = self.lane_width * 0.6
        car_length = car_width * 1.8
        car_radius = max(car_width, car_length) / 2  # Approximate collision radius
        
        for obs_obj in self.obstacles:
            dx = obs_obj['x'] - self.x
            dy = obs_obj['y'] - self.y
            dist = np.sqrt(dx*dx + dy*dy)
            if dist < obs_obj['radius'] + car_radius:
                reward -= 10.0
                self.done = True
        
        # Check if reached end
        if self.x > self.road_length:
            reward += 10.0
            self.done = True
        
        self.total_reward += reward
        self.step_count += 1
        
        return self.get_state()
    
    def get_state(self):
        """Get current state for visualization"""
        return {
            'x': float(self.x),
            'y': float(self.y),
            'angle': float(self.angle),
            'speed': float(self.speed),
            'step': int(self.step_count),
            'reward': float(self.total_reward),
            'done': bool(self.done),
            'road_width': float(self.road_width),
            'road_length': float(self.road_length),
            'num_lanes': int(self.num_lanes),
            'lane_width': float(self.lane_width),
            'lane_centers': [float(lc) for lc in self.lane_centers],
            'current_lane': int(self.current_lane),
            'obstacles': self.obstacles.copy(),
            'using_model': bool(self.using_model)  # Indicate if using trained model
        }


# Global simulator instance
_simulator = None


def get_simulator():
    """Get or create simulator instance"""
    global _simulator
    if _simulator is None:
        try:
        # Try to find best model
        base_dir = Path(__file__).parent.parent
        checkpoint_dirs = [
            base_dir / "checkpoints_new" / "checkpoint",
            base_dir / "checkpoints" / "checkpoint"
        ]
        
        model_path = None
        for checkpoint_dir in checkpoint_dirs:
            if checkpoint_dir.exists():
                try:
                    zip_files = list(checkpoint_dir.glob("rl_model_*_steps.zip"))
                    if zip_files:
                        # Get latest by modification time (most recent trained model)
                        model_path = max(zip_files, key=lambda p: p.stat().st_mtime)
                        print(f"✅ Found trained RL model for evaluation: {model_path}")
                        # Extract timestep from filename for info
                        try:
                            import re
                            match = re.search(r'rl_model_(\d+)_steps', str(model_path))
                            if match:
                                timesteps = int(match.group(1))
                                print(f"   Model trained for {timesteps:,} timesteps")
                        except:
                            pass
                        break
                except Exception as e:
                    print(f"Error finding model: {e}")
        
        config_path = base_dir / "config" / "phase1_accelerated_learning.yaml"
        if not config_path.exists():
            config_path = base_dir / "config" / "phase1_config.yaml"
        
        try:
            _simulator = Simple2DSimulator(
                model_path=str(model_path) if model_path else None,
                config_path=str(config_path) if config_path.exists() else None
            )
            # Always reset on first creation
            _simulator.reset()
        except Exception as e:
                print(f"Error creating simulator with model: {e}")
            import traceback
            traceback.print_exc()
            # Create without model as fallback
            _simulator = Simple2DSimulator(model_path=None, config_path=None)
            _simulator.reset()
        except Exception as e:
            print(f"Critical error in get_simulator: {e}")
            import traceback
            traceback.print_exc()
            # Last resort: create minimal simulator
            try:
                _simulator = Simple2DSimulator(model_path=None, config_path=None)
                _simulator.reset()
            except Exception as e2:
                print(f"Failed to create fallback simulator: {e2}")
                raise
    
    return _simulator


def reset_demo():
    """Reset demo"""
    global _simulator
    try:
    if _simulator:
        _simulator.reset()
    else:
            _simulator = get_simulator()
            if _simulator:
                _simulator.reset()
        if _simulator:
            return _simulator.get_state()
        else:
            # Fallback: create a new simulator
            _simulator = Simple2DSimulator(model_path=None, config_path=None)
            _simulator.reset()
            return _simulator.get_state()
    except Exception as e:
        # If anything fails, create a minimal simulator
        print(f"Error in reset_demo: {e}")
        import traceback
        traceback.print_exc()
        _simulator = Simple2DSimulator(model_path=None, config_path=None)
        _simulator.reset()
        return _simulator.get_state()


def step_demo(action=None):
    """Step demo"""
    try:
    simulator = get_simulator()
        if simulator:
    simulator.step(action)
    return simulator.get_state()
        else:
            # Fallback: create new simulator
            _simulator = Simple2DSimulator(model_path=None, config_path=None)
            _simulator.reset()
            _simulator.step(action)
            return _simulator.get_state()
    except Exception as e:
        print(f"Error in step_demo: {e}")
        import traceback
        traceback.print_exc()
        # Return a minimal state on error
        return {
            'x': 0.0,
            'y': 0.0,
            'angle': 0.0,
            'speed': 0.0,
            'step': 0,
            'reward': 0.0,
            'done': True,
            'error': str(e)
        }

