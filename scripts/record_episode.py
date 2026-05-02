#!/usr/bin/env python3
"""Standalone script: run one episode and save third-person + BEV video.

Usage:
    python scripts/record_episode.py --policy random
    python scripts/record_episode.py --policy checkpoint --checkpoint path/to/ckpt
"""

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import numpy as np  # noqa: E402
import yaml  # noqa: E402

from src.carla_gym_env import CarlaEnv  # noqa: E402
from src.env_wrappers import EpisodeRecorderWrapper, GTStateWrapper  # noqa: E402
from src.gt_state import NoiseConfig, StateSchema  # noqa: E402


def _schema_and_noise(cfg):
    p = cfg["perception"]; b = cfg["bev"]; n = cfg["normalization"]; z = cfg["noise"]
    return (
        StateSchema(
            max_range=float(p["max_range"]), fov_deg=float(p["fov_deg"]),
            n_vehicles=int(p["n_vehicles"]), k_waypoints=int(p["k_waypoints"]),
            waypoint_spacing=float(p["waypoint_spacing"]),
            bev_size=int(b["size"]),
            bev_meters_per_cell=float(b["meters_per_cell"]),
            bev_forward_bias=float(b["forward_bias"]),
            max_speed=float(n["max_speed"]), max_accel=float(n["max_accel"]),
        ),
        NoiseConfig(
            pos_sigma=float(z["pos_sigma"]), vel_sigma=float(z["vel_sigma"]),
            ego_pos_sigma=float(z["ego_pos_sigma"]),
            ego_vel_sigma=float(z["ego_vel_sigma"]),
            latency_frames=int(z["latency_frames"]),
            dropout_p=float(z["dropout_p"]), seed=z.get("seed"),
        ),
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config/gt_state.yaml")
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--map", default="Town01")
    ap.add_argument("--max-steps", type=int, default=500)
    ap.add_argument("--output-dir", default="artifacts/episodes")
    ap.add_argument("--policy", choices=["random", "constant"], default="constant")
    args = ap.parse_args()

    with open(args.config, "r") as f:
        cfg = yaml.safe_load(f)
    schema, noise = _schema_and_noise(cfg)

    env = CarlaEnv({
        "host": args.host, "port": args.port, "map": args.map,
        "timeout": 10.0, "delta_seconds": 0.05,
        "max_episode_steps": args.max_steps,
        "use_camera": False, "use_fixed_spawn": True,
        "fixed_spawn_indices": [0], "sensor_config": {},
    })
    env = GTStateWrapper(env, schema=schema, noise=noise)
    env = EpisodeRecorderWrapper(
        env, output_dir=args.output_dir, every_n_episodes=1,
        fps=20, bev_upscale=3, log_to_mlflow=False,
    )

    obs, _ = env.reset()
    total_r = 0.0
    for t in range(args.max_steps):
        if args.policy == "random":
            a = env.action_space.sample()
        else:
            a = np.array([0.0, 0.3, 0.0], dtype=np.float32)  # steer, throttle, brake
        obs, r, term, trunc, info = env.step(a)
        total_r += float(r)
        if term or trunc:
            break
    print(f"episode reward={total_r:.2f} steps={t+1}")
    env.close()


if __name__ == "__main__":
    main()
