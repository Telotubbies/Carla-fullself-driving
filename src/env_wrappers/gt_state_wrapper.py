"""GTStateWrapper: override observation with ground-truth state.

Wraps any gym env that exposes `.world` and `.vehicle` (as CarlaEnv does)
and replaces obs with Dict({'vector', 'bev'}) from the StateBuilder.

Action space, reward, termination are passed through unchanged. A small
jerk penalty (on action change) is added so spec's "no harsh accel/brake"
requirement is honored without editing the base env.
"""

from __future__ import annotations

import math
from typing import Any, Dict, Optional, Tuple

import gymnasium as gym
import numpy as np
from gymnasium import spaces

try:
    import carla  # type: ignore
except Exception:
    carla = None

from src.bridge.direct_source import DirectCarlaSource
from src.gt_state import DEFAULT_SCHEMA, NoiseConfig, StateBuilder, StateSchema


class GTStateWrapper(gym.Wrapper):
    def __init__(
        self,
        env: gym.Env,
        schema: StateSchema = DEFAULT_SCHEMA,
        noise: Optional[NoiseConfig] = None,
        jerk_penalty_weight: float = 0.02,
        follow_spectator: bool = False,
        spectator_every_n_steps: int = 5,
    ):
        super().__init__(env)
        self.schema = schema
        self.builder = StateBuilder(schema=schema, noise=noise)
        self.jerk_penalty_weight = jerk_penalty_weight
        self.follow_spectator = follow_spectator
        self.spectator_every_n_steps = max(1, int(spectator_every_n_steps))
        self._step_count = 0

        c, h, w = schema.bev_shape()
        vec_dim = schema.vector_dim()
        # RLlib's default vision net expects HWC; we transpose at the edge.
        # Using explicit float32 inf bounds silences the "precision lowered"
        # warning from gymnasium.spaces.Box.
        self.observation_space = spaces.Dict({
            "vector": spaces.Box(
                low=np.full((vec_dim,), -np.inf, dtype=np.float32),
                high=np.full((vec_dim,), np.inf, dtype=np.float32),
                shape=(vec_dim,), dtype=np.float32,
            ),
            "bev": spaces.Box(
                low=0, high=255, shape=(h, w, c), dtype=np.uint8,
            ),
        })
        self._prev_action: Optional[np.ndarray] = None
        self._source: Optional[DirectCarlaSource] = None

    # ----- helpers -----
    def _ensure_source(self) -> DirectCarlaSource:
        base = self.env.unwrapped
        world = getattr(base, "world", None)
        vehicle = getattr(base, "vehicle", None)
        if world is None or vehicle is None:
            raise RuntimeError(
                "GTStateWrapper requires env.unwrapped to expose .world and .vehicle",
            )
        if self._source is None or self._source.ego is not vehicle:
            self._source = DirectCarlaSource(world, vehicle)
        return self._source

    def _build_obs(self) -> Dict[str, np.ndarray]:
        src = self._ensure_source()
        obs = self.builder.build(src)
        # Builder emits BEV as (C, H, W); RLlib expects HWC.
        obs["bev"] = np.ascontiguousarray(np.transpose(obs["bev"], (1, 2, 0)))
        return obs

    # ----- gym API -----
    def reset(self, **kwargs) -> Tuple[Dict[str, np.ndarray], Dict[str, Any]]:
        _, info = self.env.reset(**kwargs)
        self._source = None  # vehicle may have been respawned
        self.builder.reset()
        self._prev_action = None
        obs = self._build_obs()
        return obs, info

    def step(self, action):
        _, reward, terminated, truncated, info = self.env.step(action)
        obs = self._build_obs()

        # Jerk penalty (spec: penalize harsh accel/brake/steer changes)
        if self._prev_action is not None and self.jerk_penalty_weight > 0.0:
            da = np.asarray(action, dtype=np.float32) - self._prev_action
            reward = float(reward) - self.jerk_penalty_weight * float(np.linalg.norm(da))
        self._prev_action = np.asarray(action, dtype=np.float32)

        self._step_count += 1
        if (
            self.follow_spectator
            and self._step_count % self.spectator_every_n_steps == 0
        ):
            self._update_spectator()

        info["gt_state_vector_dim"] = self.schema.vector_dim()
        return obs, reward, terminated, truncated, info

    def _update_spectator(self) -> None:
        """Move the CARLA spectator to a chase-cam behind the ego."""
        if carla is None:
            return
        try:
            base = self.env.unwrapped
            world = base.world
            vehicle = base.vehicle
            tf = vehicle.get_transform()
            yaw_rad = math.radians(tf.rotation.yaw)
            offset = 8.0
            height = 4.0
            x = tf.location.x - offset * math.cos(yaw_rad)
            y = tf.location.y - offset * math.sin(yaw_rad)
            z = tf.location.z + height
            spectator = world.get_spectator()
            spectator.set_transform(carla.Transform(
                carla.Location(x=x, y=y, z=z),
                carla.Rotation(pitch=-15.0, yaw=tf.rotation.yaw, roll=0.0),
            ))
        except Exception:
            pass
