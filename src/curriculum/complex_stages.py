"""Complex curriculum stages with challenging scenarios.

Stages include:
- Cut-in vehicles
- Parking scenarios  
- Pedestrian crossings
- Various weather conditions (rain, fog, night)
- Multi-agent interactions
"""

from dataclasses import dataclass
from typing import List, Dict, Any, Optional
from .stages import StageConfig, Stage


def create_complex_stages() -> List[StageConfig]:
    """Create advanced curriculum with complex scenarios."""
    
    stages = [
        # Stage 0: Basic lane keeping (same as before)
        StageConfig(
            name="basic_lane_keeping",
            description="พื้นฐานการอยู่ในเลน - ถนนตรง ไม่มีสิ่งกีดขวาง",
            target_episodes=200,
            spawn_points=[0, 1, 2],
            traffic_density=0.0,
            reward_weights={
                'w_progress': 0.5,
                'w_comfort': 0.1,
                'w_collision': 200.0,
                'w_lane_deviation': 1.0,
                'w_speed': 0.3,
            },
            success_criteria={
                'success_rate': 0.80,
                'max_lane_deviation': 0.4,
            }
        ),
        
        # Stage 1: Gentle curves
        StageConfig(
            name="gentle_curves",
            description="เลี้ยวโค้งเบา - ทางโค้ง ไม่มีรถอื่น",
            target_episodes=300,
            spawn_points=[3, 4, 5, 6, 7],
            traffic_density=0.0,
            reward_weights={
                'w_progress': 0.6,
                'w_comfort': 0.15,
                'w_collision': 200.0,
                'w_lane_deviation': 0.8,
                'w_speed': 0.3,
            },
            success_criteria={
                'success_rate': 0.75,
                'max_lane_deviation': 0.6,
            }
        ),
        
        # Stage 2: Light traffic
        StageConfig(
            name="light_traffic",
            description="รถบนถนนน้อย - มีรถคันอื่น แต่ไม่กีดขวาง",
            target_episodes=400,
            spawn_points=list(range(10)),
            traffic_density=0.15,
            reward_weights={
                'w_progress': 0.7,
                'w_comfort': 0.2,
                'w_collision': 250.0,
                'w_lane_deviation': 0.6,
                'w_speed': 0.3,
            },
            success_criteria={
                'success_rate': 0.70,
                'max_lane_deviation': 0.7,
            }
        ),
        
        # Stage 3: Vehicle cut-in scenarios
        StageConfig(
            name="cut_in_scenarios",
            description="รถตัดหน้า - มีรถเปลี่ยนเลนกะทันหัน",
            target_episodes=500,
            spawn_points=list(range(15)),
            traffic_density=0.25,
            reward_weights={
                'w_progress': 0.6,
                'w_comfort': 0.25,
                'w_collision': 300.0,  # Higher penalty for collisions
                'w_lane_deviation': 0.5,
                'w_speed': 0.2,
            },
            success_criteria={
                'success_rate': 0.65,
                'max_lane_deviation': 0.8,
            }
        ),
        
        # Stage 4: Parking scenarios
        StageConfig(
            name="parking_scenarios",
            description="จอดรถ - หาที่จอดรถ หลบรถจอด",
            target_episodes=400,
            spawn_points=[20, 21, 22, 23, 24],  # Parking areas
            traffic_density=0.3,
            reward_weights={
                'w_progress': 0.4,
                'w_comfort': 0.2,
                'w_collision': 350.0,  # Very high penalty
                'w_lane_deviation': 0.4,
                'w_speed': 0.1,  # Lower speed priority
            },
            success_criteria={
                'success_rate': 0.60,
                'max_lane_deviation': 1.0,
            }
        ),
        
        # Stage 5: Pedestrian crossings
        StageConfig(
            name="pedestrian_crossings",
            description="คนเดินข้าม - มีคนเดินข้ามถนน",
            target_episodes=500,
            spawn_points=[10, 11, 12, 13, 14, 15],  # Crosswalk areas
            traffic_density=0.2,
            reward_weights={
                'w_progress': 0.5,
                'w_comfort': 0.3,
                'w_collision': 400.0,  # Extreme penalty for pedestrian collision
                'w_lane_deviation': 0.5,
                'w_speed': 0.15,
            },
            success_criteria={
                'success_rate': 0.70,
                'max_lane_deviation': 0.8,
            }
        ),
        
        # Stage 6: Rainy weather
        StageConfig(
            name="rainy_conditions",
            description="ฝนตก - ทัศนวิสัยลดลง ถนนลื่น",
            target_episodes=400,
            spawn_points=list(range(20)),
            traffic_density=0.2,
            reward_weights={
                'w_progress': 0.5,
                'w_comfort': 0.3,
                'w_collision': 350.0,
                'w_lane_deviation': 0.6,
                'w_speed': 0.1,  # Lower speed in rain
            },
            success_criteria={
                'success_rate': 0.65,
                'max_lane_deviation': 0.9,
            }
        ),
        
        # Stage 7: Night driving
        StageConfig(
            name="night_driving",
            description="กลางคืน - มองเห็นลำบาก ไฟทางน้อย",
            target_episodes=400,
            spawn_points=list(range(20)),
            traffic_density=0.2,
            reward_weights={
                'w_progress': 0.5,
                'w_comfort': 0.25,
                'w_collision': 350.0,
                'w_lane_deviation': 0.7,
                'w_speed': 0.15,
            },
            success_criteria={
                'success_rate': 0.65,
                'max_lane_deviation': 0.9,
            }
        ),
        
        # Stage 8: Fog conditions
        StageConfig(
            name="fog_conditions",
            description="หมอก - ทัศนวิสัยต่ำมาก",
            target_episodes=400,
            spawn_points=list(range(20)),
            traffic_density=0.15,
            reward_weights={
                'w_progress': 0.4,
                'w_comfort': 0.3,
                'w_collision': 400.0,
                'w_lane_deviation': 0.6,
                'w_speed': 0.1,
            },
            success_criteria={
                'success_rate': 0.60,
                'max_lane_deviation': 1.0,
            }
        ),
        
        # Stage 9: Mixed complex scenarios (final)
        StageConfig(
            name="master_scenarios",
            description="สถานการณ์ซับซ้อนรวม - ทุกอย่างรวมกัน",
            target_episodes=600,
            spawn_points=list(range(50)),  # All spawn points
            traffic_density=0.35,
            reward_weights={
                'w_progress': 0.5,
                'w_comfort': 0.3,
                'w_collision': 500.0,
                'w_lane_deviation': 0.5,
                'w_speed': 0.2,
            },
            success_criteria={
                'success_rate': 0.55,
                'max_lane_deviation': 1.2,
            }
        ),
    ]
    
    return stages


# Weather presets
WEATHER_PRESETS = {
    "clear": {
        "cloudiness": 5.0,
        "precipitation": 0.0,
        "precipitation_deposits": 0.0,
        "wind_intensity": 10.0,
        "sun_azimuth_angle": 45.0,
        "sun_altitude_angle": 75.0,
        "fog_density": 0.0,
        "fog_distance": 0.0,
    },
    "rain": {
        "cloudiness": 80.0,
        "precipitation": 60.0,
        "precipitation_deposits": 60.0,
        "wind_intensity": 30.0,
        "sun_azimuth_angle": 45.0,
        "sun_altitude_angle": 45.0,
        "fog_density": 0.0,
        "fog_distance": 0.0,
    },
    "heavy_rain": {
        "cloudiness": 100.0,
        "precipitation": 100.0,
        "precipitation_deposits": 100.0,
        "wind_intensity": 50.0,
        "sun_azimuth_angle": 45.0,
        "sun_altitude_angle": 30.0,
        "fog_density": 0.0,
        "fog_distance": 0.0,
    },
    "fog": {
        "cloudiness": 50.0,
        "precipitation": 0.0,
        "precipitation_deposits": 0.0,
        "wind_intensity": 5.0,
        "sun_azimuth_angle": 45.0,
        "sun_altitude_angle": 20.0,
        "fog_density": 80.0,
        "fog_distance": 25.0,
    },
    "night": {
        "cloudiness": 10.0,
        "precipitation": 0.0,
        "precipitation_deposits": 0.0,
        "wind_intensity": 5.0,
        "sun_azimuth_angle": 45.0,
        "sun_altitude_angle": -30.0,  # Night time
        "fog_density": 0.0,
        "fog_distance": 0.0,
    },
}
