"""Filter & rank surrounding actors in ego frame."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Sequence

import numpy as np

from .ego_centric import in_fov


@dataclass
class ActorSnapshot:
    """Source-agnostic snapshot of a CARLA actor in ego frame."""
    id: int
    rel_x: float
    rel_y: float
    rel_vx: float
    rel_vy: float
    extent: tuple  # (length, width)
    yaw_rel: float  # radians, ego-relative

    @property
    def distance(self) -> float:
        return math.hypot(self.rel_x, self.rel_y)


def filter_and_rank(
    actors: Sequence[ActorSnapshot],
    max_range: float,
    fov_deg: float,
    top_n: int,
) -> List[ActorSnapshot]:
    """Keep actors inside range AND FOV, sort by distance, return top-N."""
    half_fov = math.radians(fov_deg) * 0.5
    out: List[ActorSnapshot] = []
    for a in actors:
        d = a.distance
        if d > max_range:
            continue
        if not in_fov(a.rel_x, a.rel_y, half_fov):
            continue
        out.append(a)
    out.sort(key=lambda x: x.distance)
    return out[:top_n]


def pad_to_n(
    actors: List[ActorSnapshot], n: int,
) -> List[ActorSnapshot | None]:
    padded: List[ActorSnapshot | None] = list(actors[:n])
    while len(padded) < n:
        padded.append(None)
    return padded
