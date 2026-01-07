"""
2D Topview Map Visualizer for CARLA
Creates a 2D topview map from CARLA world to reduce server load
"""

import carla
import numpy as np
import cv2
from typing import Tuple, Optional, List
import logging


class Map2DVisualizer:
    """Create 2D topview map from CARLA world"""
    
    def __init__(self, carla_world: carla.World, pixels_per_meter: float = 2.0):
        """
        Initialize 2D map visualizer
        
        Args:
            carla_world: CARLA world object
            pixels_per_meter: Resolution of the map (higher = more detailed)
        """
        self.world = carla_world
        self.map = carla_world.get_map()
        self.pixels_per_meter = pixels_per_meter
        
        # Calculate map bounds
        waypoints = self.map.generate_waypoints(2.0)
        if not waypoints:
            raise ValueError("No waypoints found in map")
        
        locations = [wp.transform.location for wp in waypoints]
        self.min_x = min(loc.x for loc in locations) - 50
        self.max_x = max(loc.x for loc in locations) + 50
        self.min_y = min(loc.y for loc in locations) - 50
        self.max_y = max(loc.y for loc in locations) + 50
        
        self.width = int((self.max_x - self.min_x) * pixels_per_meter)
        self.height = int((self.max_y - self.min_y) * pixels_per_meter)
        
        # Create base map image
        self.base_map = self._create_base_map()
        
        logging.info(f"✅ 2D Map created: {self.width}x{self.height} pixels "
                    f"({self.width/self.pixels_per_meter:.1f}m x {self.height/self.pixels_per_meter:.1f}m)")
    
    def _create_base_map(self) -> np.ndarray:
        """Create base 2D map from road network"""
        # Create blank image (white background)
        img = np.ones((self.height, self.width, 3), dtype=np.uint8) * 255
        
        # Draw roads
        waypoints = self.map.generate_waypoints(1.0)
        
        for waypoint in waypoints:
            # Get road boundaries
            left_waypoint = waypoint.get_left_lane_marking()
            right_waypoint = waypoint.get_right_lane_marking()
            
            # Convert to pixel coordinates
            x, y = self._world_to_pixel(waypoint.transform.location)
            
            # Draw road (gray)
            if 0 <= x < self.width and 0 <= y < self.height:
                # Draw road segment
                cv2.circle(img, (x, y), int(2 * self.pixels_per_meter), (200, 200, 200), -1)
                
                # Draw lane markings (white/yellow)
                if left_waypoint:
                    left_x, left_y = self._world_to_pixel(left_waypoint.transform.location)
                    if 0 <= left_x < self.width and 0 <= left_y < self.height:
                        cv2.circle(img, (left_x, left_y), 1, (255, 255, 0), -1)
                
                if right_waypoint:
                    right_x, right_y = self._world_to_pixel(right_waypoint.transform.location)
                    if 0 <= right_x < self.width and 0 <= right_y < self.height:
                        cv2.circle(img, (right_x, right_y), 1, (255, 255, 0), -1)
        
        return img
    
    def _world_to_pixel(self, location: carla.Location) -> Tuple[int, int]:
        """Convert CARLA world coordinates to pixel coordinates"""
        x = int((location.x - self.min_x) * self.pixels_per_meter)
        y = int((location.y - self.min_y) * self.pixels_per_meter)
        return x, y
    
    def draw_vehicle(self, img: np.ndarray, vehicle: carla.Vehicle, color: Tuple[int, int, int] = (0, 255, 0)) -> np.ndarray:
        """Draw vehicle on map"""
        transform = vehicle.get_transform()
        x, y = self._world_to_pixel(transform.location)
        
        if 0 <= x < self.width and 0 <= y < self.height:
            # Draw vehicle as rectangle
            angle = transform.rotation.yaw
            length = 4.5 * self.pixels_per_meter  # Vehicle length in meters
            width = 2.0 * self.pixels_per_meter   # Vehicle width in meters
            
            # Create rotated rectangle
            box = cv2.boxPoints(((x, y), (length, width), angle))
            box = np.int0(box)
            cv2.drawContours(img, [box], 0, color, -1)
            cv2.drawContours(img, [box], 0, (0, 0, 0), 2)
        
        return img
    
    def draw_waypoint(self, img: np.ndarray, waypoint: carla.Waypoint, color: Tuple[int, int, int] = (255, 0, 0)) -> np.ndarray:
        """Draw waypoint on map"""
        x, y = self._world_to_pixel(waypoint.transform.location)
        
        if 0 <= x < self.width and 0 <= y < self.height:
            cv2.circle(img, (x, y), 3, color, -1)
        
        return img
    
    def get_map_with_vehicle(self, vehicle: Optional[carla.Vehicle] = None) -> np.ndarray:
        """Get map image with vehicle drawn on it"""
        img = self.base_map.copy()
        
        if vehicle is not None:
            img = self.draw_vehicle(img, vehicle)
        
        return img
    
    def save_map(self, filepath: str, vehicle: Optional[carla.Vehicle] = None):
        """Save map to file"""
        img = self.get_map_with_vehicle(vehicle)
        cv2.imwrite(filepath, img)
        logging.info(f"💾 Saved 2D map to: {filepath}")


def create_2d_map_from_carla(world: carla.World, output_path: Optional[str] = None) -> Map2DVisualizer:
    """
    Create 2D topview map from CARLA world
    
    Args:
        world: CARLA world object
        output_path: Optional path to save map image
    
    Returns:
        Map2DVisualizer object
    """
    visualizer = Map2DVisualizer(world)
    
    if output_path:
        visualizer.save_map(output_path)
    
    return visualizer



