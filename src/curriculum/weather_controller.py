"""Weather and scenario controller for CARLA.

Controls:
- Weather conditions (rain, fog, night)
- Dynamic traffic scenarios (cut-in, parking, pedestrian)
- Time of day
"""

from __future__ import annotations

import random
from dataclasses import dataclass
from typing import Dict, Any, Optional, List
import carla


@dataclass
class WeatherConfig:
    """Weather configuration."""
    cloudiness: float = 5.0
    precipitation: float = 0.0
    precipitation_deposits: float = 0.0
    wind_intensity: float = 10.0
    sun_azimuth_angle: float = 45.0
    sun_altitude_angle: float = 75.0
    fog_density: float = 0.0
    fog_distance: float = 0.0
    fog_falloff: float = 0.0
    wetness: float = 0.0


class WeatherController:
    """Control CARLA weather dynamically."""
    
    PRESETS = {
        "clear": WeatherConfig(
            cloudiness=5.0, precipitation=0.0, precipitation_deposits=0.0,
            wind_intensity=10.0, sun_altitude_angle=75.0
        ),
        "cloudy": WeatherConfig(
            cloudiness=70.0, precipitation=0.0, precipitation_deposits=0.0,
            wind_intensity=20.0, sun_altitude_angle=60.0
        ),
        "rain": WeatherConfig(
            cloudiness=80.0, precipitation=60.0, precipitation_deposits=60.0,
            wind_intensity=30.0, sun_altitude_angle=45.0, wetness=60.0
        ),
        "heavy_rain": WeatherConfig(
            cloudiness=100.0, precipitation=100.0, precipitation_deposits=100.0,
            wind_intensity=50.0, sun_altitude_angle=35.0, wetness=100.0
        ),
        "fog": WeatherConfig(
            cloudiness=50.0, fog_density=80.0, fog_distance=25.0,
            fog_falloff=5.0, sun_altitude_angle=20.0
        ),
        "night": WeatherConfig(
            cloudiness=10.0, sun_altitude_angle=-30.0, sun_azimuth_angle=270.0
        ),
        "night_rain": WeatherConfig(
            cloudiness=80.0, precipitation=60.0, precipitation_deposits=60.0,
            wind_intensity=30.0, sun_altitude_angle=-20.0, wetness=60.0
        ),
    }
    
    def __init__(self, world: carla.World):
        self.world = world
        self.current_preset = "clear"
    
    def set_weather(self, preset_name: str) -> None:
        """Set weather by preset name."""
        if preset_name not in self.PRESETS:
            print(f"[Weather] Unknown preset: {preset_name}, using clear")
            preset_name = "clear"
        
        config = self.PRESETS[preset_name]
        weather = carla.WeatherParameters(
            cloudiness=config.cloudiness,
            precipitation=config.precipitation,
            precipitation_deposits=config.precipitation_deposits,
            wind_intensity=config.wind_intensity,
            sun_azimuth_angle=config.sun_azimuth_angle,
            sun_altitude_angle=config.sun_altitude_angle,
            fog_density=config.fog_density,
            fog_distance=config.fog_distance,
            fog_falloff=config.fog_falloff,
            wetness=config.wetness,
        )
        self.world.set_weather(weather)
        self.current_preset = preset_name
        print(f"[Weather] Set to: {preset_name}")
    
    def set_random_weather(self) -> str:
        """Set random weather."""
        preset = random.choice(list(self.PRESETS.keys()))
        self.set_weather(preset)
        return preset


class ScenarioController:
    """Control dynamic scenarios like cut-in, pedestrian crossing."""
    
    def __init__(self, world: carla.World, ego_vehicle: carla.Vehicle):
        self.world = world
        self.ego = ego_vehicle
        self.scenario_actors: List[carla.Actor] = []
        self.scenario_type: Optional[str] = None
    
    def clear_scenario(self) -> None:
        """Remove all scenario actors."""
        for actor in self.scenario_actors:
            try:
                actor.destroy()
            except Exception:
                pass
        self.scenario_actors.clear()
        self.scenario_type = None
    
    def spawn_cut_in_vehicle(self, distance: float = 30.0) -> Optional[carla.Vehicle]:
        """Spawn a vehicle that will cut in front of ego."""
        try:
            blueprint_library = self.world.get_blueprint_library()
            vehicle_bp = random.choice(blueprint_library.filter('vehicle.*'))
            
            # Get ego transform and spawn ahead in adjacent lane
            ego_transform = self.ego.get_transform()
            ego_location = ego_transform.location
            
            # Spawn slightly ahead and to the side
            spawn_location = carla.Location(
                x=ego_location.x + distance,
                y=ego_location.y + 3.5,  # Adjacent lane
                z=ego_location.z + 0.5
            )
            spawn_transform = carla.Transform(spawn_location, ego_transform.rotation)
            
            vehicle = self.world.spawn_actor(vehicle_bp, spawn_transform)
            if vehicle:
                self.scenario_actors.append(vehicle)
                self.scenario_type = "cut_in"
                
                # Apply control to move into ego's lane
                vehicle.apply_control(carla.VehicleControl(
                    throttle=0.4,
                    steer=-0.3,  # Steer toward ego's lane
                ))
                print(f"[Scenario] Spawned cut-in vehicle at {distance}m")
                return vehicle
        except Exception as e:
            print(f"[Scenario] Failed to spawn cut-in: {e}")
        return None
    
    def spawn_pedestrian_crossing(self, distance: float = 25.0) -> Optional[carla.Walker]:
        """Spawn a pedestrian crossing the road."""
        try:
            blueprint_library = self.world.get_blueprint_library()
            walker_bp = random.choice(blueprint_library.filter('walker.*'))
            
            ego_transform = self.ego.get_transform()
            ego_location = ego_transform.location
            
            # Spawn at crosswalk position
            spawn_location = carla.Location(
                x=ego_location.x + distance,
                y=ego_location.y - 5.0,  # Side of road
                z=ego_location.z + 0.5
            )
            spawn_transform = carla.Transform(spawn_location)
            
            walker = self.world.spawn_actor(walker_bp, spawn_transform)
            if walker:
                self.scenario_actors.append(walker)
                self.scenario_type = "pedestrian"
                
                # Control walker to cross the road
                walker_controller_bp = blueprint_library.find('controller.ai.walker')
                controller = self.world.spawn_actor(walker_controller_bp, carla.Transform(), walker)
                
                # Set destination across the road
                destination = carla.Location(
                    x=spawn_location.x,
                    y=spawn_location.y + 10.0,  # Cross the road
                    z=spawn_location.z
                )
                controller.start()
                controller.go_to_location(destination)
                controller.set_max_speed(1.5)  # Walking speed
                
                print(f"[Scenario] Spawned pedestrian crossing at {distance}m")
                return walker
        except Exception as e:
            print(f"[Scenario] Failed to spawn pedestrian: {e}")
        return None
    
    def spawn_parked_vehicles(self, count: int = 3) -> List[carla.Vehicle]:
        """Spawn parked vehicles on the side of the road."""
        spawned = []
        try:
            blueprint_library = self.world.get_blueprint_library()
            
            ego_transform = self.ego.get_transform()
            ego_location = ego_transform.location
            
            for i in range(count):
                vehicle_bp = random.choice(blueprint_library.filter('vehicle.*'))
                
                # Park on the side
                spawn_location = carla.Location(
                    x=ego_location.x + 20 + i * 8,
                    y=ego_location.y - 2.0,
                    z=ego_location.z + 0.5
                )
                spawn_transform = carla.Transform(spawn_location, ego_transform.rotation)
                
                vehicle = self.world.spawn_actor(vehicle_bp, spawn_transform)
                if vehicle:
                    vehicle.apply_control(carla.VehicleControl(hand_brake=True))
                    self.scenario_actors.append(vehicle)
                    spawned.append(vehicle)
            
            if spawned:
                self.scenario_type = "parking"
                print(f"[Scenario] Spawned {len(spawned)} parked vehicles")
        except Exception as e:
            print(f"[Scenario] Failed to spawn parked vehicles: {e}")
        return spawned


def get_weather_for_stage(stage_name: str) -> str:
    """Get appropriate weather for curriculum stage."""
    weather_map = {
        "rainy_conditions": "rain",
        "heavy_rain": "heavy_rain",
        "fog_conditions": "fog",
        "night_driving": "night",
        "night_rain": "night_rain",
    }
    return weather_map.get(stage_name, "clear")
