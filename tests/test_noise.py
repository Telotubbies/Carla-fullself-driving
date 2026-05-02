"""Tests for the StateNoise module."""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from src.gt_state.actor_filter import ActorSnapshot
from src.gt_state.noise import NoiseConfig, StateNoise


def _mk(n=3):
    return [
        ActorSnapshot(i, float(i + 1), 0.0, 0.0, 0.0, (4, 2), 0.0)
        for i in range(n)
    ]


def test_gaussian_noise_perturbs_positions():
    noise = StateNoise(NoiseConfig(pos_sigma=0.5, seed=42))
    actors = _mk(3)
    out = noise.apply_actors(list(actors))
    diffs = [abs(o.rel_x - a.rel_x) for o, a in zip(out, actors) if o is not None]
    assert any(d > 1e-6 for d in diffs)


def test_latency_delays_output():
    noise = StateNoise(NoiseConfig(latency_frames=2, seed=0))
    frames = [_mk(1) for _ in range(5)]
    frames[0][0] = ActorSnapshot(0, 1.0, 0, 0, 0, (4, 2), 0.0)
    frames[1][0] = ActorSnapshot(0, 2.0, 0, 0, 0, (4, 2), 0.0)
    frames[2][0] = ActorSnapshot(0, 3.0, 0, 0, 0, (4, 2), 0.0)
    frames[3][0] = ActorSnapshot(0, 4.0, 0, 0, 0, (4, 2), 0.0)
    frames[4][0] = ActorSnapshot(0, 5.0, 0, 0, 0, (4, 2), 0.0)

    outs = [noise.apply_actors(list(f)) for f in frames]
    # Frames 0,1 return oldest buffered (== frame 0 after first push)
    # Frame 2 onwards: returns buffer head which equals the frame (N-latency) old
    # The exact behavior: push then return buf[0]. With maxlen=3 and latency=2:
    #   after push of frames 0..2, buf = [0,1,2], return buf[0] = frame 0
    #   after push of frame 3, buf = [1,2,3], return buf[0] = frame 1
    assert outs[2][0].rel_x == 1.0
    assert outs[3][0].rel_x == 2.0
    assert outs[4][0].rel_x == 3.0


def test_dropout_probability_zero_keeps_all():
    noise = StateNoise(NoiseConfig(dropout_p=0.0, seed=0))
    actors = _mk(5)
    out = noise.apply_actors(list(actors))
    assert all(o is not None for o in out)


def test_dropout_probability_one_drops_all():
    noise = StateNoise(NoiseConfig(dropout_p=1.0, seed=0))
    actors = _mk(5)
    out = noise.apply_actors(list(actors))
    assert all(o is None for o in out)
