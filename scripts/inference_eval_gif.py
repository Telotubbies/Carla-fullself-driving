#!/usr/bin/env python3
"""Eval trained SAC model with GIF recording and keyboard parameter control.

Usage:
    venv/bin/python scripts/inference_eval_gif.py \
        --checkpoint checkpoints/gt \
        --episodes 3 \
        --gif_fps 10 \
        --output_dir artifacts/eval_gifs

Keyboard Controls (during eval):
    W/S - Increase/Decrease target speed offset (+/- 5 m/s)
    A/D - Adjust steering bias (+/- 0.1)
    Q/E - Decrease/Increase safety distance for following
    R   - Reset control overrides
    P   - Pause/Resume
    ESC - Stop episode early

The script captures BEV frames and saves as animated GIF.
"""

from __future__ import annotations

import argparse
import sys
import time
import threading
from pathlib import Path
from typing import Optional, Dict, Any

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import numpy as np
import yaml
import pygame
from PIL import Image
import imageio

import ray
from ray.rllib.algorithms.sac import SACConfig
from ray.tune.registry import register_env

from src.carla_gym_env import CarlaEnv
from src.env_wrappers import CurriculumEnvWrapper, GTStateWrapper
from src.gt_state import DEFAULT_SCHEMA, StateBuilder, NoiseConfig

# Global state for keyboard control
CONTROL_STATE = {
    "speed_offset": 0.0,      # m/s offset to target speed
    "steering_bias": 0.0,   # steering bias
    "safety_dist": 10.0,    # target following distance
    "paused": False,
    "stop_episode": False,
    "manual_mode": False,   # if True, ignore model actions
}

FRAME_BUFFER: list = []
METRICS_BUFFER: list = []


def load_config(path: str) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def init_pygame_window(width: int = 800, height: int = 600):
    """Initialize pygame window for keyboard control display."""
    pygame.init()
    screen = pygame.display.set_mode((width, height))
    pygame.display.set_caption("CARLA Eval - Keyboard Control (W/S=speed, A/D=steer, R=reset, P=pause, ESC=stop)")
    font = pygame.font.SysFont("monospace", 14)
    return screen, font


def draw_control_panel(screen, font, obs_info: Dict[str, Any]):
    """Draw control state and vehicle info on pygame window."""
    screen.fill((20, 20, 20))
    
    lines = [
        "=== Keyboard Controls ===",
        "W/S : Speed offset (+/- 5 m/s)",
        "A/D : Steering bias (+/- 0.1)",
        "R   : Reset overrides",
        "P   : Pause/Resume",
        "ESC : Stop episode",
        "",
        "=== Control State ===",
        f"Speed offset:  {CONTROL_STATE['speed_offset']:+.1f} m/s",
        f"Steering bias: {CONTROL_STATE['steering_bias']:+.2f}",
        f"Safety dist:   {CONTROL_STATE['safety_dist']:.1f} m",
        f"Paused:        {CONTROL_STATE['paused']}",
        f"Manual mode:   {CONTROL_STATE['manual_mode']}",
        "",
        "=== Vehicle State ===",
    ]
    
    # Add observation info if available
    if obs_info:
        speed = obs_info.get("speed", 0.0)
        lat_offset = obs_info.get("lat_offset", 0.0)
        heading_err = obs_info.get("heading_error", 0.0)
        lines.extend([
            f"Speed:         {speed:.1f} m/s",
            f"Lat offset:    {lat_offset:.2f} m",
            f"Heading err:   {np.degrees(heading_err):.1f} deg",
        ])
    
    y = 10
    for line in lines:
        color = (200, 200, 200) if not line.startswith("===") else (100, 200, 255)
        text = font.render(line, True, color)
        screen.blit(text, (10, y))
        y += 18
    
    pygame.display.flip()


def handle_keyboard_events():
    """Process pygame keyboard events to update CONTROL_STATE."""
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            CONTROL_STATE["stop_episode"] = True
            return False
        
        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_w:
                CONTROL_STATE["speed_offset"] += 5.0
            elif event.key == pygame.K_s:
                CONTROL_STATE["speed_offset"] -= 5.0
            elif event.key == pygame.K_a:
                CONTROL_STATE["steering_bias"] -= 0.1
            elif event.key == pygame.K_d:
                CONTROL_STATE["steering_bias"] += 0.1
            elif event.key == pygame.K_r:
                CONTROL_STATE["speed_offset"] = 0.0
                CONTROL_STATE["steering_bias"] = 0.0
                CONTROL_STATE["manual_mode"] = False
            elif event.key == pygame.K_p:
                CONTROL_STATE["paused"] = not CONTROL_STATE["paused"]
            elif event.key == pygame.K_ESCAPE:
                CONTROL_STATE["stop_episode"] = True
            elif event.key == pygame.K_m:
                CONTROL_STATE["manual_mode"] = not CONTROL_STATE["manual_mode"]
    
    return True


def modify_action(action: np.ndarray, obs: np.ndarray) -> np.ndarray:
    """Apply keyboard control overrides to action."""
    modified = action.copy()
    
    # action: [steer, throttle, brake] typically in [-1, 1] or [0, 1]
    # Apply steering bias
    modified[0] = np.clip(modified[0] + CONTROL_STATE["steering_bias"], -1.0, 1.0)
    
    # Apply speed offset via throttle modification
    # This is approximate - real implementation would use target speed
    if CONTROL_STATE["speed_offset"] > 0:
        modified[1] = min(1.0, modified[1] + CONTROL_STATE["speed_offset"] / 30.0)
    elif CONTROL_STATE["speed_offset"] < 0:
        modified[2] = min(1.0, modified[2] + abs(CONTROL_STATE["speed_offset"]) / 30.0)
    
    return modified


def capture_frame(obs: Dict[str, Any]) -> Optional[np.ndarray]:
    """Extract BEV image from observation for GIF."""
    if isinstance(obs, dict) and "bev" in obs:
        bev = obs["bev"]
        # Convert to uint8 RGB if needed
        if bev.dtype != np.uint8:
            bev = (bev * 255).astype(np.uint8)
        # Ensure RGB format
        if len(bev.shape) == 3 and bev.shape[2] == 3:
            return bev
    return None


def save_gif(frames: list, output_path: Path, fps: int = 10):
    """Save collected frames as animated GIF."""
    if not frames:
        print(f"[eval] No frames to save for {output_path}")
        return
    
    output_path.parent.mkdir(parents=True, exist_ok=True)
    
    # Convert frames to PIL Images
    pil_frames = []
    for frame in frames:
        if frame is not None:
            # Ensure correct format
            if frame.shape[2] == 3:  # RGB
                pil_frames.append(Image.fromarray(frame, mode='RGB'))
    
    if pil_frames:
        pil_frames[0].save(
            output_path,
            save_all=True,
            append_images=pil_frames[1:],
            duration=int(1000 / fps),  # ms per frame
            loop=0
        )
        print(f"[eval] GIF saved: {output_path} ({len(pil_frames)} frames, {fps} fps)")
    else:
        print(f"[eval] No valid frames for {output_path}")


def env_factory(env_config: dict):
    """Create env with same wrappers as training."""
    cfg = load_config(env_config.get("config_path", "config/gt_state.yaml"))
    
    p = cfg["perception"]
    schema = DEFAULT_SCHEMA
    noise = NoiseConfig(
        latency_frames=int(p.get("latency_ms", 0) / 50),
        pos_sigma=p.get("pos_noise_std", 0.1),
        vel_sigma=0.1,
        ego_pos_sigma=0.0,
        ego_vel_sigma=0.0,
        dropout_p=p.get("dropout_p", 0.0),
        seed=None,
    )
    
    carla_cfg = {
        "host": env_config.get("host", "localhost"),
        "port": int(env_config.get("port", 2000)),
        "timeout": 30.0,
        "map": env_config.get("map", "Town10HD_Opt"),
        "delta_seconds": 0.05,
        "max_episode_steps": env_config.get("max_episode_steps", 1000),
        "use_camera": False,
        "use_fixed_spawn": False,
        "fixed_spawn_indices": [],
        "disable_perception_sensors": True,
        "sensor_config": {},
    }
    
    env = CarlaEnv(carla_cfg)
    env = CurriculumEnvWrapper(env)
    env = GTStateWrapper(
        env, schema=schema, noise=noise,
        jerk_penalty_weight=0.0,
        follow_spectator=True,
        spectator_every_n_steps=2,
    )
    
    return env


def extract_obs_info(obs) -> Dict[str, Any]:
    """Extract readable info from observation dict."""
    info = {}
    if isinstance(obs, dict) and "vector" in obs:
        vec = obs["vector"]
        # Assuming vector contains: [speed, lat_offset, heading_error, ...]
        if len(vec) >= 3:
            info["speed"] = vec[0]
            info["lat_offset"] = vec[1]
            info["heading_error"] = vec[2]
    return info


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--checkpoint", required=True, help="Path to RLlib checkpoint dir")
    ap.add_argument("--episodes", type=int, default=3)
    ap.add_argument("--steps", type=int, default=1000)
    ap.add_argument("--config", default="config/gt_state.yaml")
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--deterministic", action="store_true", help="Use deterministic actions")
    ap.add_argument("--output_dir", default="artifacts/eval_gifs")
    ap.add_argument("--gif_fps", type=int, default=10, help="FPS for output GIF")
    ap.add_argument("--capture_every", type=int, default=5, help="Capture frame every N steps")
    ap.add_argument("--no_control_window", action="store_true", help="Disable pygame control window")
    args = ap.parse_args()
    
    # Register env
    register_env("carla_gt_env", env_factory)
    
    # Initialize Ray
    ray.init(ignore_reinit_error=True)
    
    # Build config
    cfg = load_config(args.config)
    
    sac = (
        SACConfig()
        .environment(
            env="carla_gt_env",
            env_config={
                "config_path": args.config,
                "host": args.host,
                "port": args.port,
                "max_episode_steps": args.steps,
            },
        )
        .api_stack(
            enable_rl_module_and_learner=False,
            enable_env_runner_and_connector_v2=False,
        )
        .training(
            replay_buffer_config={
                "type": "MultiAgentPrioritizedReplayBuffer",
                "capacity": 1000,
                "prioritized_replay_alpha": 0.6,
                "prioritized_replay_beta": 0.4,
            },
        )
        .resources(num_gpus=0)
        .debugging(seed=42)
    )
    
    # Build algorithm and restore checkpoint
    print(f"[eval] Loading checkpoint from: {args.checkpoint}")
    algo = sac.build()
    algo.restore(args.checkpoint)
    print("[eval] Checkpoint loaded successfully!")
    
    # Init pygame control window if enabled
    screen, font = None, None
    if not args.no_control_window:
        try:
            screen, font = init_pygame_window()
            print("[eval] Pygame control window opened")
        except Exception as e:
            print(f"[eval] Warning: Could not open pygame window: {e}")
    
    # Run inference episodes
    total_reward = 0.0
    total_steps = 0
    
    for ep in range(args.episodes):
        print(f"\n[eval] Episode {ep+1}/{args.episodes}")
        
        # Reset control state
        CONTROL_STATE["speed_offset"] = 0.0
        CONTROL_STATE["steering_bias"] = 0.0
        CONTROL_STATE["paused"] = False
        CONTROL_STATE["stop_episode"] = False
        CONTROL_STATE["manual_mode"] = False
        
        # Clear frame buffer
        episode_frames = []
        
        # Get env from algorithm
        env_runner = algo.env_runner_group.local_env_runner
        obs, info = env_runner.env.reset()
        
        episode_reward = 0.0
        episode_steps = 0
        done = False
        
        step_count = 0
        
        while not done and episode_steps < args.steps and not CONTROL_STATE["stop_episode"]:
            # Handle pygame events
            if screen is not None:
                if not handle_keyboard_events():
                    break
                obs_info = extract_obs_info(obs)
                draw_control_panel(screen, font, obs_info)
            
            # Pause handling
            while CONTROL_STATE["paused"]:
                if screen is not None:
                    handle_keyboard_events()
                    obs_info = extract_obs_info(obs)
                    draw_control_panel(screen, font, obs_info)
                time.sleep(0.1)
            
            # Capture frame
            if step_count % args.capture_every == 0:
                frame = capture_frame(obs)
                if frame is not None:
                    episode_frames.append(frame)
            
            # Compute action from model
            if args.deterministic:
                action = algo.compute_single_action(obs, explore=False)
            else:
                action = algo.compute_single_action(obs, explore=True)
            
            # Apply keyboard overrides (unless in manual mode)
            if not CONTROL_STATE["manual_mode"]:
                action = modify_action(action, obs)
            else:
                # Manual mode - use keyboard to directly control
                action = np.array([
                    CONTROL_STATE["steering_bias"],
                    0.5 if CONTROL_STATE["speed_offset"] > 0 else 0.0,
                    0.5 if CONTROL_STATE["speed_offset"] < 0 else 0.0
                ])
            
            # Step environment
            obs, reward, terminated, truncated, info = env_runner.env.step(action)
            episode_reward += reward
            episode_steps += 1
            done = terminated or truncated
            step_count += 1
            
            # Visualization delay
            time.sleep(0.02)
        
        # Save episode GIF
        gif_path = Path(args.output_dir) / f"episode_{ep+1:02d}.gif"
        save_gif(episode_frames, gif_path, fps=args.gif_fps)
        
        total_reward += episode_reward
        total_steps += episode_steps
        
        print(f"  Episode {ep+1}: reward={episode_reward:+.2f}, steps={episode_steps}")
        if info.get("collision", False):
            print("  -> COLLISION")
        elif info.get("off_road", False):
            print("  -> OFF ROAD")
        else:
            print("  -> SUCCESS (no collision)")
    
    # Cleanup pygame
    if screen is not None:
        pygame.quit()
    
    avg_reward = total_reward / args.episodes
    avg_steps = total_steps / args.episodes
    
    print(f"\n[eval] Summary ({args.episodes} episodes):")
    print(f"  Average reward: {avg_reward:+.2f}")
    print(f"  Average steps:  {avg_steps:.1f}")
    print(f"  GIFs saved to:  {args.output_dir}/")
    
    algo.stop()
    ray.shutdown()
    print("[eval] Done!")


if __name__ == "__main__":
    main()
