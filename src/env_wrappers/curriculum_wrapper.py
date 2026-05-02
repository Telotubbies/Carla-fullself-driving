"""CurriculumEnvWrapper: spawn/reward difficulty driven by a shared JSON file.

The training driver process writes the current stage to
`artifacts/curriculum_state.json`. Ray worker processes read it on every
`reset()` and dynamically rewrite the underlying `CarlaEnv` spawn config
so the agent first learns on a single fixed spawn and only advances when
it has survived several episodes with sufficient return.
"""

from __future__ import annotations

import json
import os
import random
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional

import gymnasium as gym


DEFAULT_STATE_PATH = "artifacts/curriculum_state.json"


@dataclass
class Stage:
    """One curriculum stage."""
    name: str
    spawn_indices: List[int]       # empty list = random spawn
    reward_threshold: float        # avg reward per episode to pass
    min_episodes: int = 5          # # of consecutive qualifying episodes
    description: str = ""
    weather: str = "clear"         # weather preset name; "random" picks per-episode
    weather_pool: List[str] = field(default_factory=list)
    scenarios: List[str] = field(default_factory=list)  # cut_in, parking, pedestrian, brake_check, oncoming
    scenario_prob: float = 0.0     # P(spawn a scenario actor per reset)
    traffic_density: float = 0.0   # 0..1, fraction of map filled with auto-pilot traffic

    def is_random(self) -> bool:
        return len(self.spawn_indices) == 0


# ----------------------------------------------------------------------------
# Tesla-FSD-style curriculum.
#
# Each stage progressively introduces: more spawns -> traffic density ->
# weather variation -> accident scenarios (cut-in, brake-check, pedestrian,
# parking, oncoming). The final stages pick weather + scenario uniformly at
# random per episode, mimicking FSD's "all conditions" mixed training.
# ----------------------------------------------------------------------------
DEFAULT_STAGES: List["Stage"] = [
    Stage(
        name="fsd0_lane_keeping",
        spawn_indices=[0, 1, 2],
        reward_threshold=-1100.0,  # tuned to current policy (~-870)
        min_episodes=5,
        description="Lane keeping, clear weather, no traffic.",
        weather="clear",
        traffic_density=0.0,
    ),
    Stage(
        name="fsd1_curves_and_traffic",
        spawn_indices=[0, 1, 2, 3, 4, 5],
        reward_threshold=-1000.0,
        min_episodes=5,
        description="Curves + light traffic, clear weather.",
        weather="clear",
        traffic_density=0.15,
    ),
    Stage(
        name="fsd2_random_spawn",
        spawn_indices=[],
        reward_threshold=-900.0,
        min_episodes=8,
        description="Random spawn, moderate traffic, clear.",
        weather="clear",
        traffic_density=0.25,
    ),
    Stage(
        name="fsd3_weather_intro",
        spawn_indices=[],
        reward_threshold=-900.0,
        min_episodes=8,
        description="Random spawn + random weather (clear/cloudy/rain).",
        weather="random",
        weather_pool=["clear", "cloudy", "rain"],
        traffic_density=0.25,
    ),
    Stage(
        name="fsd4_cut_in",
        spawn_indices=[],
        reward_threshold=-1000.0,
        min_episodes=10,
        description="Cut-in vehicles + traffic, mixed weather.",
        weather="random",
        weather_pool=["clear", "cloudy", "rain"],
        scenarios=["cut_in"],
        scenario_prob=0.6,
        traffic_density=0.30,
    ),
    Stage(
        name="fsd5_brake_check",
        spawn_indices=[],
        reward_threshold=-1000.0,
        min_episodes=10,
        description="Lead vehicle brake-check / sudden stop.",
        weather="random",
        weather_pool=["clear", "cloudy", "rain", "fog"],
        scenarios=["brake_check", "cut_in"],
        scenario_prob=0.7,
        traffic_density=0.30,
    ),
    Stage(
        name="fsd6_pedestrian",
        spawn_indices=[],
        reward_threshold=-1100.0,
        min_episodes=10,
        description="Pedestrian crossings + cut-ins, mixed weather.",
        weather="random",
        weather_pool=["clear", "cloudy", "rain", "fog"],
        scenarios=["pedestrian", "cut_in", "brake_check"],
        scenario_prob=0.7,
        traffic_density=0.30,
    ),
    Stage(
        name="fsd7_parking_oncoming",
        spawn_indices=[],
        reward_threshold=-1100.0,
        min_episodes=10,
        description="Parked cars on shoulder + occasional oncoming.",
        weather="random",
        weather_pool=["clear", "cloudy", "rain", "fog", "night"],
        scenarios=["parking", "oncoming", "cut_in"],
        scenario_prob=0.8,
        traffic_density=0.30,
    ),
    Stage(
        name="fsd8_full_fsd",
        spawn_indices=[],
        reward_threshold=-1200.0,
        min_episodes=15,
        description="Full FSD mix: all weather + all scenarios + dense traffic.",
        weather="random",
        weather_pool=["clear", "cloudy", "rain", "heavy_rain", "fog", "night", "night_rain"],
        scenarios=["cut_in", "brake_check", "pedestrian", "parking", "oncoming"],
        scenario_prob=0.9,
        traffic_density=0.40,
    ),
]


def get_weather_for_stage(stage_name: str) -> str:
    """Get weather preset for legacy stage names."""
    weather_map = {
        "rainy_conditions": "rain",
        "heavy_rain": "heavy_rain",
        "fog_conditions": "fog",
        "night_driving": "night",
        "night_rain": "night_rain",
        "rain": "rain",
    }
    return weather_map.get(stage_name, "clear")


def write_stage_state(
    stage_idx: int,
    stage: Stage,
    path: str = DEFAULT_STATE_PATH,
) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    payload = {
        "stage_idx": stage_idx,
        "name": stage.name,
        "spawn_indices": list(stage.spawn_indices),
        "reward_threshold": float(stage.reward_threshold),
        "min_episodes": int(stage.min_episodes),
        "description": stage.description,
        "weather": stage.weather,
        "weather_pool": list(stage.weather_pool),
        "scenarios": list(stage.scenarios),
        "scenario_prob": float(stage.scenario_prob),
        "traffic_density": float(stage.traffic_density),
    }
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(payload, f)
    os.replace(tmp, path)  # atomic


def read_stage_state(path: str = DEFAULT_STATE_PATH) -> Optional[Dict[str, Any]]:
    if not os.path.exists(path):
        return None
    try:
        with open(path, "r") as f:
            return json.load(f)
    except Exception:
        return None


class CurriculumController:
    """Advances stages based on recent mean reward. Lives in the DRIVER process."""

    def __init__(
        self,
        stages: List[Stage] = None,
        state_path: str = DEFAULT_STATE_PATH,
    ):
        self.stages = stages or DEFAULT_STAGES
        self.state_path = state_path
        self.current_idx = 0
        self._recent_rewards: List[float] = []
        # Write initial stage so worker can read it on first reset.
        write_stage_state(self.current_idx, self.stages[0], self.state_path)
        print(
            f"[Curriculum] start at stage {self.current_idx} "
            f"({self.stages[0].name}) spawns={self.stages[0].spawn_indices}"
        )

    @property
    def current(self) -> Stage:
        return self.stages[self.current_idx]

    def record(self, episode_reward_mean: float) -> None:
        """Call once per training iteration with the latest mean reward."""
        if episode_reward_mean == 0.0:
            return  # no episode completed this iter
        self._recent_rewards.append(float(episode_reward_mean))
        # keep only last N
        if len(self._recent_rewards) > 50:
            self._recent_rewards = self._recent_rewards[-50:]

        stage = self.current
        need = stage.min_episodes
        if len(self._recent_rewards) < need:
            return
        recent = self._recent_rewards[-need:]
        if all(r >= stage.reward_threshold for r in recent):
            if self.current_idx + 1 < len(self.stages):
                self._advance()

    def _advance(self) -> None:
        old = self.current
        self.current_idx += 1
        new = self.current
        self._recent_rewards.clear()
        write_stage_state(self.current_idx, new, self.state_path)
        print(
            f"[Curriculum] ADVANCE stage {self.current_idx - 1} -> "
            f"{self.current_idx} ({old.name} -> {new.name}) "
            f"spawns={new.spawn_indices}"
        )


class CurriculumEnvWrapper(gym.Wrapper):
    """Reads curriculum state on each reset and adjusts CarlaEnv:
    - spawn config (fixed list or random)
    - weather (clear/rain/fog/night/...)
    - traffic density (auto-pilot vehicles)
    - accident scenarios (cut_in / brake_check / pedestrian / parking / oncoming)
    """

    def __init__(
        self,
        env: gym.Env,
        state_path: str = DEFAULT_STATE_PATH,
        default_stage: Optional[Stage] = None,
    ):
        super().__init__(env)
        self.state_path = state_path
        self.default_stage = default_stage or DEFAULT_STAGES[0]
        self._last_stage_idx: Optional[int] = None
        self._scenario_actors: List[Any] = []  # cleaned each reset
        self._traffic_actors: List[Any] = []
        self._tm_port = 8000  # traffic manager port

    # ------------------------------------------------------------------
    def _apply_stage(self) -> Dict[str, Any]:
        state = read_stage_state(self.state_path)
        if state is None:
            stage = self.default_stage
            spawns = list(stage.spawn_indices)
            stage_idx = -1
            weather = stage.weather
            weather_pool = list(stage.weather_pool)
            scenarios = list(stage.scenarios)
            scenario_prob = float(stage.scenario_prob)
            traffic_density = float(stage.traffic_density)
        else:
            spawns = list(state.get("spawn_indices", []))
            stage_idx = int(state.get("stage_idx", -1))
            weather = str(state.get("weather", "clear"))
            weather_pool = list(state.get("weather_pool", []))
            scenarios = list(state.get("scenarios", []))
            scenario_prob = float(state.get("scenario_prob", 0.0))
            traffic_density = float(state.get("traffic_density", 0.0))

        base = self.env.unwrapped
        if len(spawns) == 0:
            if hasattr(base, "use_fixed_spawn"):
                base.use_fixed_spawn = False
            if hasattr(base, "fixed_spawn_indices"):
                base.fixed_spawn_indices = []
        else:
            if hasattr(base, "use_fixed_spawn"):
                base.use_fixed_spawn = True
            if hasattr(base, "fixed_spawn_indices"):
                base.fixed_spawn_indices = spawns

        if stage_idx != self._last_stage_idx:
            print(
                f"[CurriculumEnvWrapper] stage {stage_idx} -> "
                f"spawns={spawns if spawns else 'RANDOM'} weather={weather} "
                f"traffic={traffic_density:.2f} scenarios={scenarios} "
                f"p={scenario_prob:.2f}"
            )
            self._last_stage_idx = stage_idx

        return {
            "weather": weather,
            "weather_pool": weather_pool,
            "scenarios": scenarios,
            "scenario_prob": scenario_prob,
            "traffic_density": traffic_density,
        }

    # ------------------------------------------------------------------
    def _cleanup_scenario_actors(self) -> None:
        for a in self._scenario_actors + self._traffic_actors:
            try:
                a.destroy()
            except Exception:
                pass
        self._scenario_actors.clear()
        self._traffic_actors.clear()

    # ------------------------------------------------------------------
    def _apply_weather(self, weather: str, weather_pool: List[str]) -> None:
        try:
            import carla  # noqa: F401
        except Exception:
            return
        base = self.env.unwrapped
        world = getattr(base, "world", None)
        if world is None:
            return

        if weather == "random" and weather_pool:
            weather = random.choice(weather_pool)

        try:
            from src.curriculum.weather_controller import WeatherController
            wc = WeatherController(world)
            wc.set_weather(weather)
        except Exception as e:
            print(f"[CurriculumEnvWrapper] weather set failed: {e}")

    # ------------------------------------------------------------------
    def _spawn_traffic(self, density: float) -> None:
        if density <= 0.0:
            return
        try:
            import carla
        except Exception:
            return
        base = self.env.unwrapped
        world = getattr(base, "world", None)
        client = getattr(base, "client", None)
        if world is None or client is None:
            return

        try:
            spawn_points = world.get_map().get_spawn_points()
            n = max(1, int(len(spawn_points) * density))
            blueprints = world.get_blueprint_library().filter("vehicle.*")
            blueprints = [bp for bp in blueprints if int(bp.get_attribute("number_of_wheels")) == 4]
            random.shuffle(spawn_points)

            tm = client.get_trafficmanager(self._tm_port)
            tm.set_synchronous_mode(True)
            tm.global_percentage_speed_difference(20.0)

            spawned = 0
            for sp in spawn_points:
                if spawned >= n:
                    break
                bp = random.choice(blueprints)
                if bp.has_attribute("color"):
                    bp.set_attribute("color", random.choice(bp.get_attribute("color").recommended_values))
                actor = world.try_spawn_actor(bp, sp)
                if actor is not None:
                    actor.set_autopilot(True, self._tm_port)
                    self._traffic_actors.append(actor)
                    spawned += 1
        except Exception as e:
            print(f"[CurriculumEnvWrapper] traffic spawn failed: {e}")

    # ------------------------------------------------------------------
    def _spawn_scenario(self, scenarios: List[str], prob: float) -> None:
        if not scenarios or prob <= 0.0:
            return
        if random.random() > prob:
            return
        try:
            import carla
        except Exception:
            return
        base = self.env.unwrapped
        world = getattr(base, "world", None)
        ego = getattr(base, "vehicle", None)
        if world is None or ego is None:
            return

        kind = random.choice(scenarios)
        try:
            if kind == "cut_in":
                self._spawn_cut_in(world, ego)
            elif kind == "brake_check":
                self._spawn_brake_check(world, ego)
            elif kind == "pedestrian":
                self._spawn_pedestrian(world, ego)
            elif kind == "parking":
                self._spawn_parked_cars(world, ego)
            elif kind == "oncoming":
                self._spawn_oncoming(world, ego)
            print(f"[CurriculumEnvWrapper] scenario={kind} spawned")
        except Exception as e:
            print(f"[CurriculumEnvWrapper] scenario {kind} failed: {e}")

    # --- scenario primitives ------------------------------------------
    def _vehicle_bp(self, world):
        bps = world.get_blueprint_library().filter("vehicle.*")
        bps = [bp for bp in bps if int(bp.get_attribute("number_of_wheels")) == 4]
        return random.choice(bps)

    def _walker_bp(self, world):
        return random.choice(world.get_blueprint_library().filter("walker.pedestrian.*"))

    def _spawn_cut_in(self, world, ego) -> None:
        import carla
        t = ego.get_transform()
        fwd = t.get_forward_vector()
        right = t.get_right_vector()
        loc = t.location + fwd * 18.0 + right * 3.5
        loc.z += 0.3
        sp = carla.Transform(loc, t.rotation)
        bp = self._vehicle_bp(world)
        actor = world.try_spawn_actor(bp, sp)
        if actor is not None:
            actor.apply_control(carla.VehicleControl(throttle=0.5, steer=-0.4))
            self._scenario_actors.append(actor)

    def _spawn_brake_check(self, world, ego) -> None:
        import carla
        t = ego.get_transform()
        fwd = t.get_forward_vector()
        loc = t.location + fwd * 12.0
        loc.z += 0.3
        sp = carla.Transform(loc, t.rotation)
        bp = self._vehicle_bp(world)
        actor = world.try_spawn_actor(bp, sp)
        if actor is not None:
            actor.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, hand_brake=True))
            self._scenario_actors.append(actor)

    def _spawn_pedestrian(self, world, ego) -> None:
        import carla
        t = ego.get_transform()
        fwd = t.get_forward_vector()
        right = t.get_right_vector()
        loc = t.location + fwd * 15.0 - right * 4.0
        loc.z += 0.5
        sp = carla.Transform(loc)
        walker_bp = self._walker_bp(world)
        if walker_bp.has_attribute("is_invincible"):
            walker_bp.set_attribute("is_invincible", "false")
        walker = world.try_spawn_actor(walker_bp, sp)
        if walker is None:
            return
        self._scenario_actors.append(walker)
        try:
            ctrl_bp = world.get_blueprint_library().find("controller.ai.walker")
            controller = world.spawn_actor(ctrl_bp, carla.Transform(), walker)
            self._scenario_actors.append(controller)
            world.tick()
            controller.start()
            target = walker.get_location() + right * 8.0
            controller.go_to_location(target)
            controller.set_max_speed(1.4)
        except Exception:
            pass

    def _spawn_parked_cars(self, world, ego) -> None:
        import carla
        t = ego.get_transform()
        fwd = t.get_forward_vector()
        right = t.get_right_vector()
        for i in range(3):
            loc = t.location + fwd * (15.0 + i * 6.0) + right * 3.0
            loc.z += 0.3
            sp = carla.Transform(loc, t.rotation)
            bp = self._vehicle_bp(world)
            actor = world.try_spawn_actor(bp, sp)
            if actor is not None:
                actor.apply_control(carla.VehicleControl(hand_brake=True))
                self._scenario_actors.append(actor)

    def _spawn_oncoming(self, world, ego) -> None:
        import carla
        t = ego.get_transform()
        fwd = t.get_forward_vector()
        loc = t.location + fwd * 35.0
        loc.z += 0.3
        # Flip yaw by 180 for oncoming
        rot = carla.Rotation(
            pitch=t.rotation.pitch,
            yaw=t.rotation.yaw + 180.0,
            roll=t.rotation.roll,
        )
        sp = carla.Transform(loc, rot)
        bp = self._vehicle_bp(world)
        actor = world.try_spawn_actor(bp, sp)
        if actor is not None:
            actor.apply_control(carla.VehicleControl(throttle=0.4))
            self._scenario_actors.append(actor)

    # ------------------------------------------------------------------
    def reset(self, **kwargs):
        self._cleanup_scenario_actors()
        cfg = self._apply_stage()
        out = self.env.reset(**kwargs)
        # Apply world-level effects AFTER reset so ego exists and world is loaded.
        self._apply_weather(cfg["weather"], cfg["weather_pool"])
        self._spawn_traffic(cfg["traffic_density"])
        self._spawn_scenario(cfg["scenarios"], cfg["scenario_prob"])
        return out

    def close(self):
        try:
            self._cleanup_scenario_actors()
        except Exception:
            pass
        return self.env.close()
