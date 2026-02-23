#!/usr/bin/env python3
"""
CARLA Autopilot with Lane Detection Demo

Real-time lane detection visualization using Ultra-Fast-Lane-Detection-v2
with focus on current lane (lane ที่รถอยู่) - Mark โหดๆ
"""

import sys
import yaml
import logging
import argparse
import time
import numpy as np
import carla
from pathlib import Path
from typing import Dict, Any, List, Tuple, Optional
import cv2

# Add project root to path
project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))

from carla_env import CarlaClient, CameraSensor
from perception.lane_detector import LaneDetector
from perception.far_field_lane_detector import FarFieldLaneDetector

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class AutopilotLaneDetectionDemo:
    """CARLA autopilot with real-time lane detection visualization."""
    
    def __init__(self, config_path: str = "config.yaml", destination: Optional[List[float]] = None):
        """Initialize demo."""
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        
        self.carla_client = CarlaClient(self.config['carla'])
        self.camera: CameraSensor = None
        self.vehicle: carla.Vehicle = None
        self.destination = destination
        
        # Load lane detector
        perception_config = self.config.get('perception', {})
        model_path = perception_config.get('lane_detection_model_path', None)
        model_type = perception_config.get('lane_detection_model_type', 'ultra_fast')
        enable_ipm = perception_config.get('enable_ipm', False)
        
        if model_path and Path(model_path).exists():
            # Try to use FarFieldLaneDetector if available
            try:
                dataset = "tusimple" if "tusimple" in str(model_path).lower() else "culane"
                backbone = "18" if "res18" in str(model_path).lower() else "34"
                
                self.far_field_detector = FarFieldLaneDetector(
                    model_path=model_path,
                    image_width=640,
                    image_height=480,
                    dataset=dataset,
                    backbone=backbone,
                    enable_ipm=enable_ipm,
                    enable_kalman=True,
                    enable_validation=True
                )
                logger.info(f"Far-Field Detector: IPM={enable_ipm}, Kalman=True, Validation=True")
                self.lane_detector = None
            except Exception as e:
                logger.warning(f"Failed to load FarFieldLaneDetector: {e}, using basic LaneDetector")
                self.far_field_detector = None
                self.lane_detector = LaneDetector(
                    model_path=model_path,
                    model_type=model_type,
                    use_carla=False
                )
        else:
            self.far_field_detector = None
            self.lane_detector = LaneDetector(use_carla=True)
        
        self.running = False
        self.step_count = 0
        
        # Display window
        self.window_name = "CARLA Autopilot - Lane Detection"
        try:
            cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
        except Exception as e:
            logger.warning(f"Failed to create window: {e}, trying alternative method")
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
    
    def initialize_carla(self) -> bool:
        """Initialize CARLA environment."""
        logger.info("Initializing CARLA environment...")
        
        if not self.carla_client.connect():
            return False
        
        if not self.carla_client.load_world():
            return False
        
        if not self.carla_client.spawn_vehicle():
            return False
        
        self.vehicle = self.carla_client.vehicle
        
        # Enable autopilot with Traffic Manager
        try:
            traffic_manager = self.carla_client.client.get_trafficmanager()
            tm_port = traffic_manager.get_port()
            
            traffic_manager.set_global_distance_to_leading_vehicle(2.5)
            traffic_manager.set_synchronous_mode(True)
            traffic_manager.set_random_device_seed(0)
            
            # Set destination if provided
            if self.destination:
                try:
                    dest_location = carla.Location(self.destination[0], self.destination[1], self.destination[2])
                    world = self.carla_client.world
                    map = world.get_map()
                    dest_waypoint = map.get_waypoint(dest_location)
                    
                    if dest_waypoint:
                        current_location = self.vehicle.get_location()
                        current_waypoint = map.get_waypoint(current_location)
                        route_waypoints = []
                        if current_waypoint:
                            route_waypoints = map.get_waypoints_between(current_waypoint, dest_waypoint, 2.0)
                        
                        if len(route_waypoints) > 0:
                            traffic_manager.set_path(self.vehicle, route_waypoints)
                            logger.info(f"✅ Destination set: ({dest_location.x:.1f}, {dest_location.y:.1f}, {dest_location.z:.1f})")
                except Exception as e:
                    logger.warning(f"Failed to set destination: {e}, using random route")
            
            self.vehicle.set_autopilot(True, tm_port)
            self.autopilot_enabled = True
            logger.info(f"✅ Autopilot enabled with traffic manager (port {tm_port})")
            
            # Wait for vehicle to start moving
            logger.info("Waiting for vehicle to start moving...")
            moving = False
            for i in range(50):
                self.carla_client.tick()
                velocity = self.vehicle.get_velocity()
                speed_kmh = np.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2) * 3.6
                
                if speed_kmh > 1.0:
                    logger.info(f"✅ Vehicle is moving: {speed_kmh:.1f} km/h")
                    moving = True
                    break
                
                if i % 10 == 0:
                    logger.info(f"  Waiting... speed: {speed_kmh:.2f} km/h")
                
                time.sleep(0.1)
            
            if not moving:
                logger.warning("⚠️  Vehicle not moving, trying to re-enable autopilot...")
                self.vehicle.set_autopilot(False)
                time.sleep(0.5)
                self.vehicle.set_autopilot(True, tm_port)
                time.sleep(1.0)
        except Exception as e:
            logger.error(f"❌ Failed to enable autopilot: {e}")
            return False
        
        # Setup camera
        try:
            self.camera = CameraSensor(
                self.carla_client.world,
                self.vehicle,
                self.config['camera']
            )
            logger.info("✅ Camera sensor initialized")
        except Exception as e:
            logger.error(f"❌ Failed to setup camera: {e}")
            return False
        
        return True
    
    def visualize_lanes(
        self,
        image: np.ndarray,
        lane_mask: np.ndarray,
        lane_coords: Optional[List[List[Tuple[int, int]]]] = None,
        detection_result: Optional[Dict[str, Any]] = None
    ) -> np.ndarray:
        """Visualize lanes with focus on current lane (lane ที่รถอยู่) - Mark โหดๆ."""
        vis = image.copy()
        h, w = vis.shape[:2]
        
        # Initialize lane_stats (must be outside if block)
        lane_stats = []
        
        # ============================================================
        # 1. Focus on Current Lane (Lane ที่รถอยู่) - Mark โหดๆ
        # ============================================================
        # Get vehicle position (center of image = vehicle position)
        vehicle_x = w // 2  # Vehicle is at center horizontally
        vehicle_y = h - 50  # Vehicle is near bottom of image
        
        # Get all detected lanes
        all_lanes = []
        if detection_result:
            # Use processed lanes from far_field_detector
            if 'left_lane_coords' in detection_result and detection_result['left_lane_coords']:
                all_lanes.append(('left', detection_result['left_lane_coords']))
            if 'right_lane_coords' in detection_result and detection_result['right_lane_coords']:
                all_lanes.append(('right', detection_result['right_lane_coords']))
        
        # Fallback: Use raw lane_coords
        if len(all_lanes) == 0 and lane_coords and len(lane_coords) > 0:
            for i, lane in enumerate(lane_coords):
                if len(lane) > 0:
                    all_lanes.append((f'lane_{i}', lane))
        
        # Find current lane (lane ที่รถอยู่)
        current_lane = None
        current_lane_type = None
        min_distance = float('inf')
        
        for lane_type, lane in all_lanes:
            if len(lane) < 2:
                continue
            
            # Find closest point to vehicle position (at bottom of image)
            for p in lane:
                try:
                    x, y = float(p[0]), float(p[1])
                    # Focus on points near bottom (where vehicle is)
                    if y > h * 0.6:  # Bottom 40% of image
                        dist = abs(x - vehicle_x)
                        if dist < min_distance:
                            min_distance = dist
                            current_lane = lane
                            current_lane_type = lane_type
                except (IndexError, TypeError, ValueError):
                    continue
        
        # Mark current lane โหดๆ (very prominent)
        if current_lane and len(current_lane) >= 2:
            # Convert to int coordinates
            current_points = []
            for p in current_lane:
                try:
                    x, y = float(p[0]), float(p[1])
                    x_int, y_int = int(x), int(y)
                    if 0 <= x_int < w and 0 <= y_int < h:
                        current_points.append((x_int, y_int))
                except (IndexError, TypeError, ValueError):
                    continue
            
            if len(current_points) >= 2:
                points = np.array(current_points, dtype=np.int32)
                
                # Mark โหดๆ: Thick line, bright color, glow effect
                current_lane_color = (0, 255, 0)  # Bright green
                
                # Draw glow effect (multiple layers)
                for thickness in [8, 6, 4]:
                    alpha = 0.3 if thickness == 8 else (0.5 if thickness == 6 else 0.7)
                    overlay = vis.copy()
                    cv2.polylines(overlay, [points.reshape((-1, 1, 2))], False, current_lane_color, thickness)
                    cv2.addWeighted(overlay, alpha, vis, 1 - alpha, 0, vis)
                
                # Main line (very thick)
                cv2.polylines(vis, [points.reshape((-1, 1, 2))], False, current_lane_color, 6)
                
                # Mark points (large, bright)
                for pt in current_points[::2]:  # Every 2nd point
                    # Outer glow
                    cv2.circle(vis, tuple(pt), 10, current_lane_color, 2)
                    # Main point
                    cv2.circle(vis, tuple(pt), 7, current_lane_color, -1)
                    # Inner highlight
                    cv2.circle(vis, tuple(pt), 3, (255, 255, 255), -1)
                
                # Draw lane fill (semi-transparent) to show current lane area
                if len(current_points) >= 3:
                    # Create polygon for lane area
                    lane_polygon = np.array(current_points, dtype=np.int32)
                    overlay = vis.copy()
                    cv2.fillPoly(overlay, [lane_polygon], (0, 255, 0))
                    cv2.addWeighted(overlay, 0.15, vis, 0.85, 0, vis)
        
        # Draw other lanes (faded, less prominent)
        other_lane_color = (100, 100, 100)  # Gray for other lanes
        for lane_type, lane in all_lanes:
            if lane == current_lane:  # Skip current lane (already drawn)
                continue
            
            if len(lane) >= 2:
                other_points = []
                for p in lane:
                    try:
                        x, y = float(p[0]), float(p[1])
                        x_int, y_int = int(x), int(y)
                        if 0 <= x_int < w and 0 <= y_int < h:
                            other_points.append((x_int, y_int))
                    except (IndexError, TypeError, ValueError):
                        continue
                
                if len(other_points) >= 2:
                    points = np.array(other_points, dtype=np.int32)
                    # Faded line (thin, gray)
                    cv2.polylines(vis, [points.reshape((-1, 1, 2))], False, other_lane_color, 1)
                    # Small points
                    for pt in other_points[::5]:  # Every 5th point
                        cv2.circle(vis, tuple(pt), 2, other_lane_color, -1)
        
        # Calculate lane statistics (only for current lane)
        if current_lane and len(current_lane) >= 2:
            x_coords = [p[0] for p in current_lane]
            y_coords = [p[1] for p in current_lane]
            curvature = 0.0
            if len(x_coords) > 3:
                try:
                    coeffs = np.polyfit(y_coords, x_coords, 2)
                    curvature = abs(coeffs[0]) * 1000
                except:
                    pass
            lane_length = sum([
                np.sqrt((current_lane[i+1][0] - current_lane[i][0])**2 + (current_lane[i+1][1] - current_lane[i][1])**2)
                for i in range(len(current_lane) - 1)
            ])
            confidence = min(100.0, len(current_lane) / 10.0 * 100.0)
            lane_stats.append({
                'points': len(current_lane),
                'curvature': curvature,
                'length': lane_length,
                'confidence': confidence,
                'label': 'CURRENT LANE'  # Highlight current lane
            })
        else:
            # Fallback: Draw from mask
            lane_pixels = np.where(lane_mask > 0)
            if len(lane_pixels[0]) > 0:
                # Draw as semi-transparent overlay
                overlay = vis.copy()
                overlay[lane_mask > 0] = [0, 255, 255]  # Cyan
                cv2.addWeighted(overlay, 0.3, vis, 0.7, 0, vis)
        
        # Draw reference grid
        horizon_y = int(h * 0.4)
        cv2.line(vis, (0, horizon_y), (w, horizon_y), (100, 100, 100), 1, cv2.LINE_AA)
        cv2.line(vis, (w // 2, 0), (w // 2, h), (100, 100, 100), 1, cv2.LINE_AA)
        
        # System metrics panel (top-left)
        panel_x = 10
        panel_y = 10
        line_height = 25
        
        cv2.rectangle(vis, (panel_x - 5, panel_y - 5), (panel_x + 250, panel_y + line_height * 6), (0, 0, 0), -1)
        cv2.rectangle(vis, (panel_x - 5, panel_y - 5), (panel_x + 250, panel_y + line_height * 6), (100, 100, 100), 2)
        
        cv2.putText(vis, "SYSTEM METRICS", 
                   (panel_x, panel_y + line_height), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        y_offset = panel_y + line_height * 2
        cv2.putText(vis, f"Frame: {self.step_count:06d}", 
                   (panel_x, y_offset), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        if self.vehicle:
            velocity = self.vehicle.get_velocity()
            speed_kmh = np.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2) * 3.6
            y_offset += line_height
            cv2.putText(vis, f"Speed: {speed_kmh:5.1f} km/h", 
                       (panel_x, y_offset), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        
        # Current Lane Statistics Panel (top-right)
        if len(lane_stats) > 0:
            stats_panel_x = w - 300
            stats_panel_y = 10
            stats_height = 120
            
            cv2.rectangle(vis, (stats_panel_x, stats_panel_y), (stats_panel_x + 290, stats_panel_y + stats_height), (20, 20, 20), -1)
            cv2.rectangle(vis, (stats_panel_x, stats_panel_y), (stats_panel_x + 290, stats_panel_y + stats_height), (0, 255, 0), 3)
            
            cv2.putText(vis, "CURRENT LANE", (stats_panel_x + 10, stats_panel_y + 28),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            stat = lane_stats[0]
            y_offset = stats_panel_y + 55
            cv2.putText(vis, f"Points: {stat['points']}", (stats_panel_x + 10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            y_offset += 25
            conf_color = (0, 255, 0) if stat['confidence'] > 70 else (0, 200, 255)
            cv2.putText(vis, f"Confidence: {stat['confidence']:.1f}%", (stats_panel_x + 10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, conf_color, 2)
            y_offset += 25
            cv2.putText(vis, f"Curvature: {stat['curvature']:.2f}", (stats_panel_x + 10, y_offset),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        return vis
    
    def run(self):
        """Run autopilot with lane detection."""
        if not self.initialize_carla():
            logger.error("Failed to initialize CARLA")
            return
        
        logger.info("=" * 60)
        logger.info("🚗 CARLA Autopilot + Lane Detection Demo")
        logger.info("=" * 60)
        logger.info("Press 'q' to quit")
        logger.info("")
        
        self.running = True
        self.step_count = 0
        
        try:
            while self.running:
                # Tick world (synchronous mode)
                self.carla_client.tick()
                
                # Get camera image
                image = None
                max_retries = 5
                for _ in range(max_retries):
                    image = self.camera.get_image()
                    if image is not None:
                        break
                    time.sleep(0.01)
                
                if image is None:
                    logger.warning("No image received")
                    continue
                
                # Detect lanes
                lane_mask = np.zeros((image.shape[0], image.shape[1]), dtype=np.uint8)
                lane_coords = None
                detection_result = None
                
                if self.far_field_detector:
                    try:
                        detection_result = self.far_field_detector.detect(image)
                        if detection_result:
                            lane_coords = detection_result.get('raw_lane_coords', [])
                            if detection_result.get('left_lane_coords'):
                                # Create mask from lane coords
                                for coords in [detection_result.get('left_lane_coords'), detection_result.get('right_lane_coords')]:
                                    if coords:
                                        for p in coords:
                                            try:
                                                x, y = int(p[0]), int(p[1])
                                                if 0 <= x < image.shape[1] and 0 <= y < image.shape[0]:
                                                    lane_mask[y, x] = 255
                                            except:
                                                pass
                    except Exception as e:
                        logger.debug(f"Far-field detection error: {e}")
                        detection_result = None
                
                elif self.lane_detector:
                    try:
                        result = self.lane_detector.detect_lanes(image)
                        if result:
                            if isinstance(result, tuple) and len(result) >= 2:
                                lane_mask, _ = result[0], result[1]
                                if len(result) >= 3:
                                    lane_coords = result[2]
                            elif isinstance(result, dict):
                                lane_mask = result.get('lane_mask', lane_mask)
                                lane_coords = result.get('lane_coords')
                    except Exception as e:
                        logger.debug(f"Lane detection error: {e}")
                
                # Visualize
                vis = self.visualize_lanes(image, lane_mask, lane_coords, detection_result)
                
                # Display
                cv2.imshow(self.window_name, cv2.cvtColor(vis, cv2.COLOR_RGB2BGR))
                
                # Check for quit
                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    logger.info("Quit requested")
                    break
                
                self.step_count += 1
                
                # Limit FPS
                time.sleep(0.05)
        
        except KeyboardInterrupt:
            logger.info("Interrupted by user")
        except Exception as e:
            logger.error(f"Error in main loop: {e}")
            import traceback
            logger.debug(traceback.format_exc())
        finally:
            self.cleanup()
    
    def cleanup(self):
        """Clean up resources."""
        logger.info("Cleaning up...")
        self.running = False
        
        if self.camera:
            self.camera.destroy()
        
        cv2.destroyAllWindows()
        
        if self.carla_client:
            self.carla_client.cleanup()
        
        logger.info("✅ Cleanup complete")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description="CARLA Autopilot + Lane Detection Demo")
    parser.add_argument("--destination", type=float, nargs=3, metavar=("X", "Y", "Z"),
                       help="Destination coordinates (x, y, z)")
    parser.add_argument("--config", type=str, default="config.yaml",
                       help="Path to config file")
    
    args = parser.parse_args()
    
    destination = args.destination if args.destination else None
    
    demo = AutopilotLaneDetectionDemo(config_path=args.config, destination=destination)
    demo.run()


if __name__ == "__main__":
    main()
