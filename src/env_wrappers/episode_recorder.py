"""EpisodeRecorderWrapper: save third-person + BEV video per episode.

Uses a spectator-style third-person RGB camera attached to the ego
vehicle (for recording only -- NOT exposed in the observation) and a
colorized version of the BEV produced by StateBuilder. Saves MP4 files
under `output_dir/ep_{index}.mp4`.

If `mlflow` is available and a run is active, each mp4 is logged as an
artifact under the `videos/` path.
"""

from __future__ import annotations

import os
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

import gymnasium as gym
import numpy as np

try:
    import carla  # type: ignore
except Exception:
    carla = None

try:
    import imageio.v2 as imageio
except Exception:  # pragma: no cover
    imageio = None

try:
    import mlflow  # type: ignore
except Exception:  # pragma: no cover
    mlflow = None

import cv2


_BEV_COLORS = np.array([
    [80, 80, 80],     # road  (gray)
    [255, 255, 255],  # lane  (white)
    [0, 255, 0],      # ego   (green)
    [0, 128, 255],    # other vehicles (orange-blue)
    [0, 0, 255],      # traffic (red)
], dtype=np.uint8)


def colorize_bev(bev: np.ndarray) -> np.ndarray:
    """BEV (either CHW or HWC, uint8) -> (H,W,3) uint8."""
    if bev.ndim != 3:
        raise ValueError(f"expected 3D bev, got shape={bev.shape}")
    # Heuristic: channel-first if first dim is small (<=8) and last is larger
    if bev.shape[0] <= 8 and bev.shape[-1] > 8:
        chw = bev
    else:
        chw = np.transpose(bev, (2, 0, 1))
    c, h, w = chw.shape
    out = np.zeros((h, w, 3), dtype=np.uint8)
    for i in range(c):
        mask = chw[i] > 0
        if not mask.any():
            continue
        color = _BEV_COLORS[i % len(_BEV_COLORS)]
        out[mask] = color
    return out


class EpisodeRecorderWrapper(gym.Wrapper):
    def __init__(
        self,
        env: gym.Env,
        output_dir: str = "artifacts/episodes",
        every_n_episodes: int = 5,
        fps: int = 20,
        enable: bool = True,
        third_person_size: Tuple[int, int] = (640, 360),
        bev_upscale: int = 3,
        log_to_mlflow: bool = True,
    ):
        super().__init__(env)
        self.output_dir = output_dir
        self.every_n = max(1, int(every_n_episodes))
        self.fps = int(fps)
        self.enable = bool(enable) and imageio is not None
        print(
            f"[EpisodeRecorder] init enable={self.enable} "
            f"(requested={enable}, imageio_loaded={imageio is not None}) "
            f"output_dir={output_dir} every_n={self.every_n} fps={self.fps}"
        )
        self.third_person_size = third_person_size
        self.bev_upscale = max(1, int(bev_upscale))
        self.log_to_mlflow = log_to_mlflow and (mlflow is not None)

        os.makedirs(self.output_dir, exist_ok=True)
        self._episode_idx = -1
        self._recording = False
        self._writer = None
        self._video_path: Optional[str] = None
        self._latest_tp_frame: Optional[np.ndarray] = None
        self._tp_camera = None
        self._frames_written = 0
        self._write_err_once = True
        self._stepped_since_reset = False

    # ----- third-person camera lifecycle -----
    def _attach_camera(self) -> None:
        base = self.env.unwrapped
        world = getattr(base, "world", None)
        vehicle = getattr(base, "vehicle", None)
        if world is None or vehicle is None or carla is None:
            return
        try:
            bp = world.get_blueprint_library().find("sensor.camera.rgb")
            w, h = self.third_person_size
            bp.set_attribute("image_size_x", str(w))
            bp.set_attribute("image_size_y", str(h))
            bp.set_attribute("fov", "90")
            tf = carla.Transform(
                carla.Location(x=-6.0, z=3.0),
                carla.Rotation(pitch=-15.0),
            )
            cam = world.spawn_actor(bp, tf, attach_to=vehicle)
            cam.listen(self._on_camera)
            self._tp_camera = cam
        except Exception as e:
            print(f"[EpisodeRecorder] camera attach failed: {e}")
            self._tp_camera = None

    def _on_camera(self, image) -> None:
        try:
            w, h = self.third_person_size
            arr = np.frombuffer(image.raw_data, dtype=np.uint8).reshape(
                (h, w, 4),
            )[:, :, :3][:, :, ::-1].copy()  # BGRA -> RGB
            self._latest_tp_frame = arr
        except Exception:
            pass

    def _detach_camera(self) -> None:
        if self._tp_camera is not None:
            try:
                self._tp_camera.stop()
                self._tp_camera.destroy()
            except Exception:
                pass
            self._tp_camera = None

    # ----- frame composition -----
    def _compose_frame(self, obs: Dict[str, Any]) -> Optional[np.ndarray]:
        if not isinstance(obs, dict):
            if self._write_err_once:
                print(
                    f"[EpisodeRecorder] obs is not a dict (type={type(obs).__name__}"
                    f" shape={getattr(obs, 'shape', None)}); cannot compose BEV frame"
                )
                self._write_err_once = False
            return self._latest_tp_frame  # fall back to third-person only
        bev = obs.get("bev")
        if bev is None and self._write_err_once:
            print(f"[EpisodeRecorder] obs dict keys={list(obs.keys())} (no 'bev')")
            self._write_err_once = False
        bev_img = None
        if bev is not None:
            bev_img = colorize_bev(np.asarray(bev))
            if self.bev_upscale > 1:
                bev_img = cv2.resize(
                    bev_img,
                    (bev_img.shape[1] * self.bev_upscale,
                     bev_img.shape[0] * self.bev_upscale),
                    interpolation=cv2.INTER_NEAREST,
                )
        tp = self._latest_tp_frame
        if tp is None and bev_img is None:
            return None
        if tp is None:
            return bev_img
        if bev_img is None:
            return tp

        # Match heights, stack side-by-side
        th = tp.shape[0]
        bh = bev_img.shape[0]
        if bh != th:
            scale = th / bh
            bev_img = cv2.resize(
                bev_img,
                (int(bev_img.shape[1] * scale), th),
                interpolation=cv2.INTER_NEAREST,
            )
        return np.concatenate([tp, bev_img], axis=1)

    # ----- gym API -----
    def reset(self, **kwargs):
        # RLlib calls env.reset() multiple times during worker initialization
        # (validation + first episode). We advance episode_idx only for resets
        # that follow at least one step so the recorder's episode count matches
        # what the policy actually rolls out on.
        if self._stepped_since_reset:
            self._episode_idx += 1
        else:
            # No stepping happened, previous reset was a no-op validation reset.
            # Keep same episode_idx (start at 0 for the first real reset).
            if self._episode_idx < 0:
                self._episode_idx = 0
        self._stepped_since_reset = False
        self._finalize_writer()
        obs, info = self.env.reset(**kwargs)
        self._recording = self.enable and (self._episode_idx % self.every_n == 0)
        self._latest_tp_frame = None
        self._frames_written = 0
        self._write_err_once = True
        if self._recording:
            self._detach_camera()
            self._attach_camera()
            # Warm-up tick: the RGB camera is async; one tick after attach
            # makes the first frame available before the first step composes.
            try:
                base = self.env.unwrapped
                if hasattr(base, "world") and base.world is not None:
                    base.world.tick()
            except Exception:
                pass
            self._video_path = os.path.join(
                self.output_dir, f"ep_{self._episode_idx:05d}.mp4",
            )
            try:
                self._writer = imageio.get_writer(
                    self._video_path, fps=self.fps, codec="libx264",
                )
                print(
                    f"[EpisodeRecorder] recording episode {self._episode_idx} "
                    f"-> {self._video_path}"
                )
            except Exception as e:
                print(f"[EpisodeRecorder] writer open failed: {e}")
                self._writer = None
                self._recording = False
        return obs, info

    _DBG_STEP_COUNT = 0  # class-level, prints first N unconditionally

    def step(self, action):
        obs, reward, terminated, truncated, info = self.env.step(action)
        self._stepped_since_reset = True
        # UNCONDITIONAL debug for the first few steps so we can tell whether
        # step() is even entering and what the recorder state looks like.
        if EpisodeRecorderWrapper._DBG_STEP_COUNT < 3:
            EpisodeRecorderWrapper._DBG_STEP_COUNT += 1
            print(
                f"[EpisodeRecorder] STEP_ENTER#{EpisodeRecorderWrapper._DBG_STEP_COUNT} "
                f"self.enable={self.enable} "
                f"recording={self._recording} writer={'yes' if self._writer else 'no'} "
                f"episode_idx={self._episode_idx} "
                f"obs_type={type(obs).__name__} "
                f"keys={list(obs.keys()) if isinstance(obs, dict) else 'N/A'}",
                flush=True,
            )
        if self._recording and self._writer is not None:
            frame = self._compose_frame(obs)
            if frame is not None:
                try:
                    self._writer.append_data(frame)
                    self._frames_written += 1
                except Exception as e:
                    if self._write_err_once:
                        print(f"[EpisodeRecorder] append failed: {e}")
                        self._write_err_once = False
            else:
                # log once when frame is None so we can diagnose
                if self._frames_written == 0 and self._write_err_once:
                    print("[EpisodeRecorder] _compose_frame returned None")
                    self._write_err_once = False
        if terminated or truncated:
            print(
                f"[EpisodeRecorder] episode end (terminated={terminated} "
                f"truncated={truncated}) frames_written={self._frames_written}"
            )
            self._finalize_writer()
        return obs, reward, terminated, truncated, info

    def close(self):
        self._finalize_writer()
        self._detach_camera()
        return super().close()

    # ----- teardown -----
    def _finalize_writer(self) -> None:
        if self._writer is not None:
            path = self._video_path
            try:
                self._writer.close()
            except Exception as e:
                print(f"[EpisodeRecorder] close failed: {e}")
            self._writer = None
            self._video_path = None
            if path and os.path.exists(path):
                size = os.path.getsize(path)
                print(
                    f"[EpisodeRecorder] finalized {path} "
                    f"({size} bytes, {self._frames_written} frames)"
                )
                if self.log_to_mlflow and mlflow is not None:
                    try:
                        mlflow.log_artifact(path, artifact_path="videos")
                    except Exception:
                        pass
            else:
                print(f"[EpisodeRecorder] finalize: path missing or not created ({path})")
        self._detach_camera()
