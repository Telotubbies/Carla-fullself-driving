#!/usr/bin/env python3
"""RLlib SAC training with GROUND-TRUTH state (no camera/lidar detection).

Complies with the spec in `.windsurf/plans/carla-gt-rl-system-ab5fe3.md`:
  - obs = Dict({'vector': ..., 'bev': ...}) built from CARLA ground truth
  - actor filter: nearest-N, range <= 50m, FOV = 120deg
  - noise: gaussian + latency + dropout
  - jerk penalty on action change
  - MLflow metrics + per-episode video artifact
  - curriculum stage advancement (uses existing CurriculumManager)

Run:
    python train_rllib_gt.py --iterations 100 --config config/gt_state.yaml
"""

import argparse
import atexit
import os
import signal
import sys
from pathlib import Path

ROOT = Path(__file__).parent.resolve()
sys.path.insert(0, str(ROOT))

import numpy as np  # noqa: E402
import yaml  # noqa: E402
import gymnasium as gym  # noqa: E402
from gymnasium import spaces  # noqa: E402

from src.carla_gym_env import CarlaEnv  # noqa: E402
from src.env_wrappers import (  # noqa: E402
    CurriculumController,
    CurriculumEnvWrapper,
    EpisodeRecorderWrapper,
    GTStateWrapper,
)
from src.gt_state import NoiseConfig, StateSchema  # noqa: E402
from src.mlflow_integration import MLflowTracker  # noqa: E402


def load_config(path: str) -> dict:
    with open(path, "r") as f:
        return yaml.safe_load(f)


def build_schema_and_noise(cfg: dict):
    p = cfg["perception"]
    b = cfg["bev"]
    n = cfg["normalization"]
    noise_cfg = cfg["noise"]
    schema = StateSchema(
        max_range=float(p["max_range"]),
        fov_deg=float(p["fov_deg"]),
        n_vehicles=int(p["n_vehicles"]),
        k_waypoints=int(p["k_waypoints"]),
        waypoint_spacing=float(p["waypoint_spacing"]),
        bev_size=int(b["size"]),
        bev_meters_per_cell=float(b["meters_per_cell"]),
        bev_forward_bias=float(b["forward_bias"]),
        max_speed=float(n["max_speed"]),
        max_accel=float(n["max_accel"]),
    )
    noise = NoiseConfig(
        pos_sigma=float(noise_cfg["pos_sigma"]),
        vel_sigma=float(noise_cfg["vel_sigma"]),
        ego_pos_sigma=float(noise_cfg["ego_pos_sigma"]),
        ego_vel_sigma=float(noise_cfg["ego_vel_sigma"]),
        latency_frames=int(noise_cfg["latency_frames"]),
        dropout_p=float(noise_cfg["dropout_p"]),
        seed=noise_cfg.get("seed"),
    )
    return schema, noise


def make_env_factory(cfg: dict):
    schema, noise = build_schema_and_noise(cfg)
    rec = cfg["recording"]
    jerk_w = float(cfg["reward"]["jerk_penalty_weight"])

    def _factory(env_config):
        carla_cfg = {
            "host": env_config.get("host", "localhost"),
            "port": env_config.get("port", 2000),
            "timeout": 30.0,
            "map": env_config.get("map", "Town10HD_Opt"),
            "delta_seconds": 0.05,
            "max_episode_steps": env_config.get("max_episode_steps", 1000),
            "use_camera": False,       # ground-truth only
            "use_fixed_spawn": False,  # random spawns avoid reuse collisions
            "fixed_spawn_indices": [],
            "disable_perception_sensors": True,  # skip LiDAR/RGB entirely
            "sensor_config": {},
        }
        env = CarlaEnv(carla_cfg)
        # Curriculum: worker reads the current stage from a shared JSON file
        # and rewrites the CarlaEnv spawn config on each reset.
        env = CurriculumEnvWrapper(env)
        env = GTStateWrapper(
            env, schema=schema, noise=noise,
            jerk_penalty_weight=jerk_w,
            follow_spectator=bool(env_config.get("follow_spectator", True)),
            spectator_every_n_steps=5,
        )
        if rec.get("enable", False):
            env = EpisodeRecorderWrapper(
                env,
                output_dir=rec.get("output_dir", "artifacts/episodes"),
                every_n_episodes=int(rec.get("every_n_episodes", 5)),
                fps=int(rec.get("fps", 20)),
                third_person_size=tuple(rec.get("third_person_size", [640, 360])),
                bev_upscale=int(rec.get("bev_upscale", 3)),
                log_to_mlflow=bool(rec.get("log_to_mlflow", True)),
            )
        return env

    return _factory


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", default="config/gt_state.yaml")
    ap.add_argument("--iterations", type=int, default=100)
    ap.add_argument("--host", default="localhost")
    ap.add_argument("--port", type=int, default=2000)
    ap.add_argument("--experiment", default="carla_rllib_gt_state")
    ap.add_argument("--run-name", default="sac_gt")
    ap.add_argument("--checkpoint-dir", default="./checkpoints/gt")
    ap.add_argument("--resume", action="store_true",
                    help="Resume from latest checkpoint in --checkpoint-dir")
    ap.add_argument("--restore", default=None,
                    help="Explicit checkpoint path to restore (overrides --resume)")
    args = ap.parse_args()

    cfg = load_config(args.config)

    # Lazy imports so the module imports cleanly without ray installed
    import ray  # noqa: E402
    from ray.rllib.algorithms.sac import SACConfig  # noqa: E402
    from ray.tune.registry import register_env  # noqa: E402

    print("=" * 80)
    print("RLlib SAC w/ Ground-Truth State (no camera/lidar detection)")
    print("=" * 80)

    ray.init(ignore_reinit_error=True)

    tracker = MLflowTracker(
        experiment_name=args.experiment, tracking_uri="./mlruns",
    )
    tracker.start_run(run_name=args.run_name)

    # Ensure the MLflow run is closed even on SIGTERM / os._exit from Ray.
    def _end_run_safely(*_):
        try:
            tracker.end_run()
        except Exception:
            pass
    atexit.register(_end_run_safely)
    for _sig in (signal.SIGTERM, signal.SIGINT):
        try:
            signal.signal(_sig, lambda *a: (_end_run_safely(), sys.exit(0)))
        except Exception:
            pass

    tracker.log_params({
        "algo": "SAC",
        "obs": "dict(vector+bev)",
        "config_file": args.config,
        "n_vehicles": cfg["perception"]["n_vehicles"],
        "k_waypoints": cfg["perception"]["k_waypoints"],
        "max_range": cfg["perception"]["max_range"],
        "fov_deg": cfg["perception"]["fov_deg"],
        "bev_size": cfg["bev"]["size"],
        "bev_mpc": cfg["bev"]["meters_per_cell"],
        "noise_pos_sigma": cfg["noise"]["pos_sigma"],
        "noise_vel_sigma": cfg["noise"]["vel_sigma"],
        "noise_latency_frames": cfg["noise"]["latency_frames"],
        "noise_dropout_p": cfg["noise"]["dropout_p"],
    })

    env_fn = make_env_factory(cfg)
    register_env("carla_gt_env", env_fn)

    sac = (
        SACConfig()
        .api_stack(
            enable_rl_module_and_learner=False,
            enable_env_runner_and_connector_v2=False,
        )
        .environment(env="carla_gt_env", env_config={"host": args.host, "port": args.port})
        .framework("torch")
        .training(
            actor_lr=3e-4,
            critic_lr=3e-4,
            tau=0.005,
            gamma=0.99,
            # Big batch on GPU for fast learning. Tunable via env var TRAIN_BATCH.
            train_batch_size=int(os.environ.get("TRAIN_BATCH", 256)),
            replay_buffer_config={
                "type": "MultiAgentReplayBuffer",
                "capacity": 25_000,
            },
        )
        # 1 remote env runner so a CARLA-side crash is isolated to that
        # worker; RLlib will auto-restart it instead of bringing down training.
        .env_runners(num_env_runners=1, num_envs_per_env_runner=1)
        .fault_tolerance(
            restart_failed_env_runners=True,
            ignore_env_runner_failures=True,
        )
        .resources(num_gpus=int(os.environ.get("TRAIN_NUM_GPUS", cfg.get("num_gpus", 1))))
        .debugging(seed=42)
    )
    algo = sac.build()

    # Resume from checkpoint if requested
    restore_path = args.restore
    if not restore_path and args.resume:
        # Find latest checkpoint in checkpoint-dir
        ck_dir = os.path.abspath(args.checkpoint_dir)
        if os.path.isdir(ck_dir):
            # Look for direct rllib_checkpoint.json or subdirs
            if os.path.exists(os.path.join(ck_dir, "rllib_checkpoint.json")):
                restore_path = ck_dir
            else:
                # Pick latest subdirectory containing a checkpoint
                subs = [
                    os.path.join(ck_dir, d) for d in os.listdir(ck_dir)
                    if os.path.isdir(os.path.join(ck_dir, d))
                    and os.path.exists(os.path.join(ck_dir, d, "rllib_checkpoint.json"))
                ]
                if subs:
                    restore_path = max(subs, key=os.path.getmtime)
    if restore_path:
        try:
            print(f"[resume] Restoring from: {restore_path}")
            algo.restore(restore_path)
            print(f"[resume] Restored successfully.")
            tracker.log_param("resumed_from", restore_path)
        except Exception as e:
            print(f"[resume] Restore failed: {e}. Starting fresh.")

    # Curriculum controller in the DRIVER process. Worker env reads stage via
    # shared JSON file (artifacts/curriculum_state.json). Advances when the
    # agent meets the reward threshold for N consecutive qualifying iters.
    curriculum = CurriculumController()
    tracker.log_param("curriculum_stages", len(curriculum.stages))

    def _get_metric(result, *keys, default=0.0):
        """Pull a metric from either the new Ray 2.x `env_runners` subdict or
        the legacy top-level keys, whichever is present."""
        er = result.get("env_runners", {}) or {}
        for k in keys:
            if k in er and er[k] is not None:
                return float(er[k])
            if k in result and result[k] is not None:
                return float(result[k])
        return default

    try:
        for it in range(args.iterations):
            result = algo.train()
            metrics = {
                "episode_reward_mean": _get_metric(
                    result, "episode_return_mean", "episode_reward_mean",
                ),
                "episode_reward_min": _get_metric(
                    result, "episode_return_min", "episode_reward_min",
                ),
                "episode_reward_max": _get_metric(
                    result, "episode_return_max", "episode_reward_max",
                ),
                "episode_len_mean": _get_metric(result, "episode_len_mean"),
                "num_episodes": _get_metric(
                    result, "num_episodes", "num_episodes_lifetime",
                ),
                "timesteps_total": _get_metric(
                    result,
                    "num_env_steps_sampled_lifetime",
                    "timesteps_total",
                ),
            }
            print(
                f"iter={it+1:3d} "
                f"R={metrics['episode_reward_mean']:+.1f} "
                f"len={metrics['episode_len_mean']:.0f} "
                f"eps={metrics['num_episodes']:.0f} "
                f"steps={metrics['timesteps_total']:.0f}"
            )
            tracker.log_metrics(metrics, step=it)

            # Feed the latest episode reward into the curriculum controller.
            # It decides when to advance stages and writes the shared JSON
            # file that workers read on reset.
            try:
                curriculum.record(metrics["episode_reward_mean"])
                tracker.log_metric(
                    "curriculum/stage_idx",
                    float(curriculum.current_idx),
                    step=it,
                )
                tracker.set_tag(
                    "curriculum/stage_name", curriculum.current.name,
                )
            except Exception as e:
                print(f"  curriculum update failed: {e}")

            if (it + 1) % 10 == 0:
                ck = algo.save(checkpoint_dir=args.checkpoint_dir)
                print(f"  checkpoint saved: {ck}")
    except KeyboardInterrupt:
        print("interrupted by user")
    finally:
        try:
            ck = algo.save(checkpoint_dir=args.checkpoint_dir + "/final")
            print(f"final checkpoint: {ck}")
        except Exception as e:
            print(f"checkpoint save failed: {e}")
        algo.stop()
        tracker.end_run()
        ray.shutdown()


if __name__ == "__main__":
    main()
