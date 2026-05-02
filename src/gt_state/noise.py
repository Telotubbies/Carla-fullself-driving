"""Observation noise: gaussian, latency buffer, random dropout.

Applied AFTER ego-centric transform but BEFORE vector/BEV rasterization,
so both representations are consistently degraded.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from typing import Deque, List, Optional

import numpy as np

from .actor_filter import ActorSnapshot


@dataclass
class NoiseConfig:
    pos_sigma: float = 0.0      # meters, per-axis gaussian on rel_x/rel_y
    vel_sigma: float = 0.0      # m/s, per-axis gaussian on rel_vx/rel_vy
    ego_pos_sigma: float = 0.0  # not applied to ego origin (always 0 in ego frame) but to ego_state.lat_offset
    ego_vel_sigma: float = 0.0  # m/s on ego speed
    latency_frames: int = 0     # >=0; 0 = disabled
    dropout_p: float = 0.0      # per-actor independent drop probability
    seed: Optional[int] = None


class StateNoise:
    """Stateful noise applicator (holds RNG and latency buffer)."""

    def __init__(self, config: NoiseConfig):
        self.cfg = config
        self._rng = np.random.default_rng(config.seed)
        self._buf: Deque[List[Optional[ActorSnapshot]]] = deque(
            maxlen=max(1, config.latency_frames + 1)
        )

    def reset(self) -> None:
        self._buf.clear()
        # Reseed RNG so episodes are reproducible when cfg.seed is set.
        if self.cfg.seed is not None:
            self._rng = np.random.default_rng(self.cfg.seed)

    def apply_actors(
        self, actors: List[Optional[ActorSnapshot]],
    ) -> List[Optional[ActorSnapshot]]:
        """Apply gaussian noise + dropout, then push through latency buffer."""
        noised: List[Optional[ActorSnapshot]] = []
        for a in actors:
            if a is None:
                noised.append(None)
                continue
            if self.cfg.dropout_p > 0.0 and self._rng.random() < self.cfg.dropout_p:
                noised.append(None)
                continue
            nx = a.rel_x
            ny = a.rel_y
            nvx = a.rel_vx
            nvy = a.rel_vy
            if self.cfg.pos_sigma > 0.0:
                nx += float(self._rng.normal(0.0, self.cfg.pos_sigma))
                ny += float(self._rng.normal(0.0, self.cfg.pos_sigma))
            if self.cfg.vel_sigma > 0.0:
                nvx += float(self._rng.normal(0.0, self.cfg.vel_sigma))
                nvy += float(self._rng.normal(0.0, self.cfg.vel_sigma))
            noised.append(ActorSnapshot(
                id=a.id,
                rel_x=nx, rel_y=ny,
                rel_vx=nvx, rel_vy=nvy,
                extent=a.extent, yaw_rel=a.yaw_rel,
            ))

        if self.cfg.latency_frames <= 0:
            return noised

        self._buf.append(noised)
        if len(self._buf) <= self.cfg.latency_frames:
            # Not enough history yet -> return oldest we have (approx: empty slots)
            return self._buf[0]
        return self._buf[0]

    def apply_ego_scalar(self, value: float, sigma: float) -> float:
        if sigma <= 0.0:
            return value
        return float(value + self._rng.normal(0.0, sigma))
