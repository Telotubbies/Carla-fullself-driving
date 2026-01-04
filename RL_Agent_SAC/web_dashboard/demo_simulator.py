import numpy as np
import json
from pathlib import Path
import sys
import os
sys.path.insert(0, str(Path(__file__).parent.parent))
try:
    from stable_baselines3 import PPO
    import torch
except ImportError:
    print("Warning: stable_baselines3 not available, demo will use mock data")
    PPO = None
    torch = None
class Simple2DSimulator:
    
    def __init__(self, model_path=None, config_path=None):
        self.model = None
        self.model_path = model_path
        self.config_path = config_path
        self.device = None
        self.reset()
        self.using_model = False
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
        
        if not PPO:
            return
        try:
            if torch and torch.cuda.is_available():
                self.device = torch.device('cuda:0')
            else:
                self.device = torch.device('cpu')
            self.model = PPO.load(model_path, device=self.device)
            print(f"✅ Model loaded from {model_path}")
        except Exception as e:
            print(f"❌ Failed to load model: {e}")
            self.model = None
    def reset(self):
        
        self.x = 0.0
        self.y = 0.0
        self.angle = 0.0
        self.speed = 0.0
        self.step_count = 0
        self.total_reward = 0.0
        self.done = False
        self.num_lanes = 4
        self.lane_width = 3.0
        self.road_width = self.num_lanes * self.lane_width
        self.road_length = 200.0
        self.lane_centers = [
            -4.5,
            -1.5,
            1.5,
            4.5
        ]
        self.current_lane = 1
        import random
        random.seed(42)
        self.obstacles = []
        for i in range(12):
            lane_idx = random.randint(0, self.num_lanes - 1)
            x_pos = 20 + i * 15 + random.uniform(-3, 3)
            y_pos = self.lane_centers[lane_idx] + random.uniform(-0.5, 0.5)
            self.obstacles.append({
                'x': x_pos,
                'y': y_pos,
                'radius': 0.8,
                'lane': lane_idx
            })
    def get_observation(self):
        
        obs = np.zeros(12, dtype=np.float32)
        for lane_idx in range(self.num_lanes):
            min_dist = 50.0
            for obs_obj in self.obstacles:
                dx = obs_obj['x'] - self.x
                dy = obs_obj['y'] - self.y
                if dx > 0 and abs(obs_obj['y'] - self.lane_centers[lane_idx]) < self.lane_width:
                    dist = np.sqrt(dx*dx + dy*dy)
                    if dist < min_dist:
                        min_dist = dist
            obs[lane_idx] = min_dist / 50.0
        obs[4] = (self.y + self.road_width/2) / self.road_width
        obs[5] = (self.y + self.road_width/2) / self.road_width
        obs[6] = (self.road_width/2 - self.y) / self.road_width
        obs[7] = self.speed / 20.0
        obs[8] = self.angle / np.pi
        obs[9] = self.x / self.road_length
        obs[10] = 1.0 if self.x > self.road_length * 0.5 else 0.0
        obs[11] = 1.0 if self.speed > 10.0 else 0.0
        return obs.reshape(1, -1)
    def step(self, action=None):
        
        if self.done:
            return self.get_state()
        if action is None and self.model:
            try:
                obs = self.get_observation()
                action, _ = self.model.predict(obs, deterministic=True)
                if isinstance(action, np.ndarray) and action.ndim > 1:
                    action = action[0]
                if not isinstance(action, (list, np.ndarray)):
                    action = [action] if hasattr(action, '__iter__') else [0.0, 0.5, 0.0]
                action = np.array(action, dtype=np.float32)
                if len(action) < 3:
                    action = np.pad(action, (0, 3 - len(action)), 'constant')
                self.using_model = True
            except Exception as e:
                print(f"⚠️ Model prediction failed: {e}, falling back to Advanced AI")
                self.using_model = False
                action = None
        else:
            self.using_model = False
        if action is None:
            current_lane_idx = 0
            min_lane_dist = float('inf')
            for i, lane_center in enumerate(self.lane_centers):
                dist = abs(self.y - lane_center)
                if dist < min_lane_dist:
                    min_lane_dist = dist
                    current_lane_idx = i
            self.current_lane = current_lane_idx
            lane_obstacles = {}
            for lane_idx in range(self.num_lanes):
                min_dist = 100.0
                for obs_obj in self.obstacles:
                    dx = obs_obj['x'] - self.x
                    dy = obs_obj['y'] - self.y
                    if dx > 0 and dx < 30.0 and abs(obs_obj['y'] - self.lane_centers[lane_idx]) < self.lane_width * 0.8:
                        dist = np.sqrt(dx*dx + dy*dy)
                        if dist < min_dist:
                            min_dist = dist
                lane_obstacles[lane_idx] = min_dist
            best_lane = current_lane_idx
            best_lane_dist = lane_obstacles[current_lane_idx]
            if current_lane_idx > 0:
                if lane_obstacles[current_lane_idx - 1] > best_lane_dist + 5.0:
                    best_lane = current_lane_idx - 1
                    best_lane_dist = lane_obstacles[current_lane_idx - 1]
            if current_lane_idx < self.num_lanes - 1:
                if lane_obstacles[current_lane_idx + 1] > best_lane_dist + 5.0:
                    best_lane = current_lane_idx + 1
                    best_lane_dist = lane_obstacles[current_lane_idx + 1]
            if lane_obstacles[current_lane_idx] < 15.0:
                if current_lane_idx > 0 and lane_obstacles[current_lane_idx - 1] > lane_obstacles[current_lane_idx]:
                    best_lane = current_lane_idx - 1
                elif current_lane_idx < self.num_lanes - 1 and lane_obstacles[current_lane_idx + 1] > lane_obstacles[current_lane_idx]:
                    best_lane = current_lane_idx + 1
            target_lane_center = self.lane_centers[best_lane]
            y_offset = self.y - target_lane_center
            lane_keeping_steer = -y_offset * 0.4
            min_dist = 50.0
            nearest_obs_x = 0
            nearest_obs_y = 0
            obstacle_ahead = False
            for obs_obj in self.obstacles:
                dx = obs_obj['x'] - self.x
                dy = obs_obj['y'] - self.y
                dist = np.sqrt(dx*dx + dy*dy)
                if dx > 0 and dx < 25.0 and abs(dy) < dx * 0.5:
                    if dist < min_dist:
                        min_dist = dist
                        nearest_obs_x = obs_obj['x']
                        nearest_obs_y = obs_obj['y']
                        obstacle_ahead = True
            avoid_steer = 0.0
            if obstacle_ahead and min_dist < 12.0:
                if nearest_obs_y > self.y:
                    avoid_steer = -0.8 * (1.0 - min_dist / 12.0)
                else:
                    avoid_steer = 0.8 * (1.0 - min_dist / 12.0)
            if obstacle_ahead and min_dist < 12.0:
                steer = avoid_steer * 0.6 + lane_keeping_steer * 0.4
            else:
                steer = lane_keeping_steer
            steer = np.clip(steer, -1.0, 1.0)
            target_speed = 12.0
            if lane_obstacles[current_lane_idx] < 20.0:
                target_speed = max(6.0, lane_obstacles[current_lane_idx] * 0.4)
            if best_lane != current_lane_idx:
                target_speed = max(8.0, target_speed * 0.8)
            speed_error = target_speed - self.speed
            if speed_error > 0.5:
                throttle = min(0.8, speed_error / 5.0)
                brake = 0.0
            elif speed_error < -0.5:
                throttle = 0.0
                brake = min(0.5, abs(speed_error) / 5.0)
            else:
                throttle = 0.3
                brake = 0.0
            if self.speed < 2.0:
                throttle = max(throttle, 0.5)
            action = np.array([steer, throttle, brake], dtype=np.float32)
        steer = float(action[0])
        throttle = float(action[1])
        brake = float(action[2])
        self.angle += steer * 0.1
        if throttle > 0:
            self.speed = min(self.speed + throttle * 0.5, 20.0)
        if brake > 0:
            self.speed = max(self.speed - brake * 1.0, 0.0)
        if throttle == 0 and brake == 0:
            self.speed *= 0.95
        self.x += self.speed * np.cos(self.angle) * 0.1
        self.y += self.speed * np.sin(self.angle) * 0.1
        reward = 0.1
        if self.speed > 5.0:
            reward += 0.1
        if abs(self.y) > self.road_width / 2:
            reward -= 0.5
        current_lane_idx = 0
        min_lane_dist = float('inf')
        for i, lane_center in enumerate(self.lane_centers):
            dist = abs(self.y - lane_center)
            if dist < min_lane_dist:
                min_lane_dist = dist
                current_lane_idx = i
        if min_lane_dist < self.lane_width * 0.3:
            reward += 0.05
        car_width = self.lane_width * 0.6
        car_length = car_width * 1.8
        car_radius = max(car_width, car_length) / 2
        for obs_obj in self.obstacles:
            dx = obs_obj['x'] - self.x
            dy = obs_obj['y'] - self.y
            dist = np.sqrt(dx*dx + dy*dy)
            if dist < obs_obj['radius'] + car_radius:
                reward -= 10.0
                self.done = True
        if self.x > self.road_length:
            reward += 10.0
            self.done = True
        self.total_reward += reward
        self.step_count += 1
        return self.get_state()
    def get_state(self):
        
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
            'using_model': bool(self.using_model)
        }
_simulator = None
def get_simulator():
    
    global _simulator
    if _simulator is None:
        try:
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
                        model_path = max(zip_files, key=lambda p: p.stat().st_mtime)
                        print(f"✅ Found trained RL model for evaluation: {model_path}")
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
            _simulator.reset()
        except Exception as e:
                print(f"Error creating simulator with model: {e}")
            import traceback
            traceback.print_exc()
            _simulator = Simple2DSimulator(model_path=None, config_path=None)
            _simulator.reset()
        except Exception as e:
            print(f"Critical error in get_simulator: {e}")
            import traceback
            traceback.print_exc()
            try:
                _simulator = Simple2DSimulator(model_path=None, config_path=None)
                _simulator.reset()
            except Exception as e2:
                print(f"Failed to create fallback simulator: {e2}")
                raise
    return _simulator
def reset_demo():
    
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
            _simulator = Simple2DSimulator(model_path=None, config_path=None)
            _simulator.reset()
            return _simulator.get_state()
    except Exception as e:
        print(f"Error in reset_demo: {e}")
        import traceback
        traceback.print_exc()
        _simulator = Simple2DSimulator(model_path=None, config_path=None)
        _simulator.reset()
        return _simulator.get_state()
def step_demo(action=None):
    
    try:
    simulator = get_simulator()
        if simulator:
    simulator.step(action)
    return simulator.get_state()
        else:
            _simulator = Simple2DSimulator(model_path=None, config_path=None)
            _simulator.reset()
            _simulator.step(action)
            return _simulator.get_state()
    except Exception as e:
        print(f"Error in step_demo: {e}")
        import traceback
        traceback.print_exc()
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