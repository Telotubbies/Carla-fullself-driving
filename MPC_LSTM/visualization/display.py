"""
Real-time visualization display using pygame.

Shows camera view, predicted trajectory, vehicle state, and live graphs.
"""

import pygame
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend
import matplotlib.pyplot as plt
import matplotlib.backends.backend_agg as agg
from typing import Optional, List, Dict, Any
import logging
from collections import deque
import time
import sys
from pathlib import Path

# Import core interfaces
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))
from core.interfaces import IVisualizationModule

logger = logging.getLogger(__name__)


class VisualizationDisplay(IVisualizationModule):
    """Real-time visualization display for autonomous driving."""
    
    def __init__(self, config: Dict[str, Any]):
        """
        Initialize visualization display.
        
        Args:
            config: Visualization configuration dictionary
        """
        self.config = config
        self.width = config.get('window_width', 1280)
        self.height = config.get('window_height', 720)
        self.update_rate = config.get('update_rate', 20)
        self.graph_history = config.get('graph_history', 1000)
        
        # Initialize pygame
        pygame.init()
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("CARLA LSTM-MPC Autonomous Driving")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.Font(None, 36)
        self.small_font = pygame.font.Font(None, 24)
        
        # Data buffers for graphs
        self.time_buffer = deque(maxlen=self.graph_history)
        self.speed_buffer = deque(maxlen=self.graph_history)
        self.steering_buffer = deque(maxlen=self.graph_history)
        self.start_time = time.time()
        
        # Matplotlib figure for graphs
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(6, 4))
        self.fig.tight_layout(pad=2.0)
        
        logger.info(f"✅ VisualizationDisplay initialized ({self.width}x{self.height})")
    
    def update(
        self,
        image: np.ndarray,
        vehicle_state: Dict[str, Any],
        predicted_trajectory: Optional[np.ndarray] = None,
        mpc_horizon: Optional[np.ndarray] = None,
        lane_info: Optional[Dict[str, Any]] = None
    ) -> bool:
        """
        Update visualization.
        
        Implements IVisualizationModule.update().
        
        Args:
            image: Camera image (H, W, 3)
            vehicle_state: Current vehicle state
            predicted_trajectory: LSTM predicted trajectory (optional)
            mpc_horizon: MPC predicted horizon (optional)
            lane_info: Lane detection information (optional)
        
        Returns:
            True if should continue, False to stop
        """
        camera_image = image  # Keep for backward compatibility
        # Handle events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
        
        # Clear screen
        self.screen.fill((30, 30, 30))
        
        # Update data buffers
        current_time = time.time() - self.start_time
        self.time_buffer.append(current_time)
        self.speed_buffer.append(vehicle_state.get('velocity', 0.0) * 3.6)  # Convert to km/h
        self.steering_buffer.append(vehicle_state.get('steering', 0.0))
        
        # Layout: Camera on left, info and graphs on right
        camera_width = self.width // 2
        camera_height = self.height // 2
        
        # Display camera image
        if camera_image is not None and self.config.get('show_camera', True):
            camera_surface = self._image_to_surface(camera_image, camera_width, camera_height)
            self.screen.blit(camera_surface, (0, 0))
            
            # Overlay lane visualization if available
            if lane_info is not None and self.config.get('show_trajectory', True):
                self._draw_lane_visualization(lane_info, camera_width, camera_height)
            
            # Overlay predicted trajectory if available
            if predicted_trajectory is not None and self.config.get('show_trajectory', True):
                self._draw_trajectory_overlay(predicted_trajectory, camera_width, camera_height)
            
            # Overlay MPC horizon if available
            if mpc_horizon is not None and self.config.get('show_trajectory', True):
                self._draw_mpc_horizon(mpc_horizon, camera_width, camera_height)
        
        # Display vehicle state info
        info_x = camera_width + 20
        info_y = 20
        self._draw_vehicle_info(vehicle_state, info_x, info_y)
        
        # Display graphs
        if self.config.get('show_graphs', True) and len(self.time_buffer) > 1:
            graph_surface = self._render_graphs()
            self.screen.blit(graph_surface, (camera_width + 20, camera_height + 20))
        
        # Update display
        pygame.display.flip()
        self.clock.tick(self.update_rate)
        
        return True
    
    def _image_to_surface(self, image: np.ndarray, target_width: int, target_height: int) -> pygame.Surface:
        """
        Convert numpy image to pygame surface.
        
        Args:
            image: Image array (H, W, 3)
            target_width: Target width
            target_height: Target height
            
        Returns:
            Pygame surface
        """
        # Resize if needed
        if image.shape[1] != target_width or image.shape[0] != target_height:
            import cv2
            image = cv2.resize(image, (target_width, target_height))
        
        # Convert to RGB if needed
        if image.shape[2] == 3:
            # Already RGB
            image_rgb = image
        else:
            image_rgb = image[:, :, :3]
        
        # Ensure uint8
        if image_rgb.dtype != np.uint8:
            image_rgb = (image_rgb * 255).astype(np.uint8)
        
        # Convert to pygame surface
        surface = pygame.surfarray.make_surface(np.swapaxes(image_rgb, 0, 1))
        return surface
    
    def _draw_vehicle_info(self, state: Dict[str, Any], x: int, y: int) -> None:
        """Draw vehicle state information."""
        info_lines = [
            f"Speed: {state.get('velocity', 0.0) * 3.6:.1f} km/h",
            f"Steering: {state.get('steering', 0.0):.3f}",
            f"Throttle: {state.get('throttle', 0.0):.3f}",
            f"Brake: {state.get('brake', 0.0):.3f}",
            f"Position: ({state.get('x', 0.0):.1f}, {state.get('y', 0.0):.1f})",
            f"Yaw: {state.get('yaw', 0.0):.1f}°",
        ]
        
        for i, line in enumerate(info_lines):
            text = self.small_font.render(line, True, (255, 255, 255))
            self.screen.blit(text, (x, y + i * 30))
    
    def _draw_lane_visualization(self, lane_info: Dict[str, Any], width: int, height: int) -> None:
        """
        Draw lane visualization (lines and boxes) on camera view for lane following.
        
        Args:
            lane_info: Lane information dict with 'left_lane', 'right_lane', 'center_line', 'lane_mask', 'waypoints'
            width: Camera view width
            height: Camera view height
        """
        # Draw lane mask overlay (semi-transparent)
        lane_mask = lane_info.get('lane_mask')
        if lane_mask is not None:
            # Convert mask to surface
            mask_surface = pygame.surfarray.make_surface(
                np.swapaxes(lane_mask.astype(np.uint8) * 255, 0, 1)
            )
            mask_surface.set_alpha(80)  # More transparent
            mask_surface = pygame.transform.scale(mask_surface, (width, height))
            self.screen.blit(mask_surface, (0, 0))
        
        # Draw waypoints if available (from CARLA map)
        waypoints = lane_info.get('waypoints')
        if waypoints is not None and len(waypoints) > 0:
            self._draw_waypoints(waypoints, width, height)
        
        # Draw center line (bright green, thicker)
        center_line = lane_info.get('center_line')
        if center_line is not None and len(center_line) > 0:
            points = []
            for point in center_line[:30]:  # Show more points
                if len(point) >= 2:
                    x = int(np.clip(point[0], 0, width))
                    y = int(np.clip(point[1], 0, height))
                    points.append((x, y))
            
            if len(points) > 1:
                # Draw center line
                pygame.draw.lines(self.screen, (0, 255, 0), False, points, 5)
                # Draw boxes along center line for lane following
                for i in range(0, len(points), 2):
                    x, y = points[i]
                    box_size = 10
                    
                    # Calculate box position with boundary checking
                    box_x = int(np.clip(x - box_size//2, 0, width - box_size))
                    box_y = int(np.clip(y - box_size//2, 0, height - box_size))
                    
                    # Only draw if box is within screen bounds
                    if 0 <= box_x < width and 0 <= box_y < height:
                        # Draw filled box
                        pygame.draw.rect(self.screen, (0, 255, 0), 
                                        (box_x, box_y, box_size, box_size))
                        pygame.draw.rect(self.screen, (0, 180, 0), 
                                        (box_x, box_y, box_size, box_size), 2)
        
        # Draw left lane (yellow, thicker)
        left_lane = lane_info.get('left_lane')
        if left_lane is not None and len(left_lane) > 0:
            points = []
            for point in left_lane[:30]:
                if len(point) >= 2:
                    x = int(np.clip(point[0], 0, width))
                    y = int(np.clip(point[1], 0, height))
                    points.append((x, y))
            
            if len(points) > 1:
                pygame.draw.lines(self.screen, (255, 255, 0), False, points, 4)
                # Draw boxes
                for i in range(0, len(points), 3):
                    x, y = points[i]
                    box_size = 8
                    
                    # Calculate box position with boundary checking
                    box_x = int(np.clip(x - box_size//2, 0, width - box_size))
                    box_y = int(np.clip(y - box_size//2, 0, height - box_size))
                    
                    # Only draw if box is within screen bounds
                    if 0 <= box_x < width and 0 <= box_y < height:
                        pygame.draw.rect(self.screen, (255, 255, 0), 
                                        (box_x, box_y, box_size, box_size), 2)
        
        # Draw right lane (cyan, thicker)
        right_lane = lane_info.get('right_lane')
        if right_lane is not None and len(right_lane) > 0:
            points = []
            for point in right_lane[:30]:
                if len(point) >= 2:
                    x = int(np.clip(point[0], 0, width))
                    y = int(np.clip(point[1], 0, height))
                    points.append((x, y))
            
            if len(points) > 1:
                pygame.draw.lines(self.screen, (0, 255, 255), False, points, 4)
                # Draw boxes
                for i in range(0, len(points), 3):
                    x, y = points[i]
                    box_size = 8
                    
                    # Calculate box position with boundary checking
                    box_x = int(np.clip(x - box_size//2, 0, width - box_size))
                    box_y = int(np.clip(y - box_size//2, 0, height - box_size))
                    
                    # Only draw if box is within screen bounds
                    if 0 <= box_x < width and 0 <= box_y < height:
                        pygame.draw.rect(self.screen, (0, 255, 255), 
                                        (box_x, box_y, box_size, box_size), 2)
        
        # Draw lane following area (box between left and right lanes)
        if left_lane is not None and right_lane is not None:
            self._draw_lane_following_area(left_lane, right_lane, width, height)
    
    def _draw_trajectory_overlay(self, trajectory: np.ndarray, width: int, height: int) -> None:
        """
        Draw predicted trajectory overlay on camera view as lines and boxes.
        
        Args:
            trajectory: Trajectory points (N, 2) or (N, 4) where first 2 are x, y
            width: Camera view width
            height: Camera view height
        """
        if trajectory is None or len(trajectory) == 0:
            return
        
        # Extract x, y coordinates
        if trajectory.shape[1] >= 2:
            traj_points = trajectory[:, :2]
        else:
            return
        
        # Project trajectory to camera view
        # Draw from bottom center (vehicle position) going forward
        points = []
        center_x = width // 2
        bottom_y = height - 20
        
        for i in range(min(len(traj_points), 20)):  # Show up to 20 points
            # Project relative to vehicle position
            # x offset from center, y goes up (forward)
            x_offset = traj_points[i, 0] * 15  # Scale factor for visibility
            y_offset = -i * 8  # Move up (forward in image)
            
            x_screen = int(center_x + x_offset)
            y_screen = int(bottom_y + y_offset)
            
            x_screen = np.clip(x_screen, 0, width)
            y_screen = np.clip(y_screen, 0, height)
            points.append((x_screen, y_screen))
        
        # Draw trajectory line (bright blue, thicker)
        if len(points) > 1:
            # Draw main trajectory line
            pygame.draw.lines(self.screen, (100, 200, 255), False, points, 5)
            
            # Draw boxes along trajectory (every point)
            for i, point in enumerate(points):
                box_size = 8 if i % 3 == 0 else 6  # Larger boxes every 3rd point
                color = (150, 220, 255) if i % 3 == 0 else (100, 200, 255)
                border_color = (50, 150, 255)  # Darker blue border for better contrast
                
                # Calculate box position with boundary checking
                x = int(np.clip(point[0] - box_size//2, 0, width - box_size))
                y = int(np.clip(point[1] - box_size//2, 0, height - box_size))
                
                # Only draw if box is within screen bounds
                if 0 <= x < width and 0 <= y < height:
                    # Draw filled box with border
                    pygame.draw.rect(self.screen, color, 
                                    (x, y, box_size, box_size))
                    pygame.draw.rect(self.screen, border_color, 
                                    (x, y, box_size, box_size), 2)
    
    def _draw_mpc_horizon(self, mpc_horizon: np.ndarray, width: int, height: int) -> None:
        """
        Draw MPC horizon path on camera view with lines and boxes for future path.
        
        Args:
            mpc_horizon: MPC predicted trajectory (N+1, 4) [x, y, yaw, v]
            width: Camera view width
            height: Camera view height
        """
        if mpc_horizon is None or len(mpc_horizon) == 0:
            return
        
        # Extract x, y coordinates
        if mpc_horizon.shape[1] >= 2:
            horizon_points = mpc_horizon[:, :2]
        else:
            return
        
        # Project to camera view
        center_x = width // 2
        bottom_y = height - 20
        points = []
        
        horizon = self.config.get('mpc', {}).get('horizon', 10)
        for i in range(min(len(horizon_points), horizon + 1)):
            # Project relative to vehicle
            x_offset = horizon_points[i, 0] * 12  # Scale for visibility
            y_offset = -i * 7  # Move up (forward)
            
            x_screen = int(center_x + x_offset)
            y_screen = int(bottom_y + y_offset)
            
            x_screen = np.clip(x_screen, 0, width)
            y_screen = np.clip(y_screen, 0, height)
            points.append((x_screen, y_screen))
        
        # Draw MPC horizon line (magenta/pink, thicker)
        if len(points) > 1:
            # Draw main horizon line
            pygame.draw.lines(self.screen, (255, 100, 255), False, points, 4)
            
            # Draw boxes for future path points
            for i, point in enumerate(points):
                # Box size increases with distance (future points are larger)
                box_size = 6 + (i // 3)  # Increase size for further points
                box_size = min(box_size, 12)  # Cap at 12
                
                # Color intensity based on distance (keep minimum brightness)
                alpha_factor = max(0.6, 1.0 - (i / len(points)) * 0.4)  # Fade but stay visible
                color = (
                    int(255 * alpha_factor),
                    int(100 * alpha_factor),
                    int(255 * alpha_factor)
                )
                border_color = (200, 0, 200)  # Darker magenta border for better contrast
                
                # Calculate box position with boundary checking
                x = int(np.clip(point[0] - box_size//2, 0, width - box_size))
                y = int(np.clip(point[1] - box_size//2, 0, height - box_size))
                
                # Only draw if box is within screen bounds
                if 0 <= x < width and 0 <= y < height:
                    # Draw filled box
                    pygame.draw.rect(self.screen, color, 
                                    (x, y, box_size, box_size))
                    # Draw border
                    pygame.draw.rect(self.screen, border_color, 
                                    (x, y, box_size, box_size), 2)
    
    def _draw_waypoints(self, waypoints: list, width: int, height: int) -> None:
        """
        Draw CARLA waypoints as boxes for lane following.
        
        Args:
            waypoints: List of CARLA waypoint objects
            width: Camera view width
            height: Camera view height
        """
        if not waypoints or len(waypoints) == 0:
            return
        
        # Project waypoints to screen (simplified projection)
        # In real implementation, would use proper camera projection
        points = []
        center_x = width // 2
        bottom_y = height - 20
        
        for i, wp in enumerate(waypoints[:30]):  # Show first 30 waypoints
            try:
                wp_loc = wp.transform.location
                # Simple projection: assume waypoint is in front
                # Map to screen coordinates
                x_offset = (wp_loc.x % 100) * 0.1  # Simplified
                y_offset = -i * 6  # Move up
                
                x_screen = int(center_x + x_offset * 10)
                y_screen = int(bottom_y + y_offset)
                
                x_screen = np.clip(x_screen, 0, width)
                y_screen = np.clip(y_screen, 0, height)
                points.append((x_screen, y_screen))
            except Exception:
                continue
        
        # Draw waypoint line (orange)
        if len(points) > 1:
            pygame.draw.lines(self.screen, (255, 165, 0), False, points, 3)
        
        # Draw boxes for waypoints
        for i, point in enumerate(points[::2]):  # Every other waypoint
            box_size = 7
            
            # Calculate box position with boundary checking
            x = int(np.clip(point[0] - box_size//2, 0, width - box_size))
            y = int(np.clip(point[1] - box_size//2, 0, height - box_size))
            
            # Only draw if box is within screen bounds
            if 0 <= x < width and 0 <= y < height:
                # Draw filled box
                pygame.draw.rect(self.screen, (255, 165, 0), 
                                (x, y, box_size, box_size))
                pygame.draw.rect(self.screen, (255, 140, 0), 
                                (x, y, box_size, box_size), 2)
    
    def _draw_lane_following_area(self, left_lane: list, right_lane: list, width: int, height: int) -> None:
        """
        Draw lane following area as boxes between left and right lanes.
        
        Args:
            left_lane: Left lane points
            right_lane: Right lane points
            width: Camera view width
            height: Camera view height
        """
        if not left_lane or not right_lane:
            return
        
        # Draw boxes in the lane following area
        min_len = min(len(left_lane), len(right_lane))
        for i in range(0, min_len, 5):  # Every 5th point
            if i < len(left_lane) and i < len(right_lane):
                left_point = left_lane[i]
                right_point = right_lane[i]
                
                if len(left_point) >= 2 and len(right_point) >= 2:
                    # Calculate center point
                    center_x = int((left_point[0] + right_point[0]) / 2)
                    center_y = int((left_point[1] + right_point[1]) / 2)
                    
                    # Calculate lane width
                    lane_width = np.sqrt(
                        (left_point[0] - right_point[0])**2 + 
                        (left_point[1] - right_point[1])**2
                    )
                    
                    # Draw box representing lane following area
                    box_width = int(min(lane_width * 0.6, 30))
                    box_height = 8
                    
                    center_x = np.clip(center_x, box_width//2, width - box_width//2)
                    center_y = np.clip(center_y, box_height//2, height - box_height//2)
                    
                    # Draw semi-transparent box
                    box_surface = pygame.Surface((box_width, box_height))
                    box_surface.set_alpha(60)
                    box_surface.fill((0, 200, 255))  # Light blue
                    self.screen.blit(box_surface, 
                                    (center_x - box_width//2, center_y - box_height//2))
                    
                    # Draw border
                    pygame.draw.rect(self.screen, (0, 150, 255), 
                                    (center_x - box_width//2, center_y - box_height//2, 
                                     box_width, box_height), 1)
    
    def _render_graphs(self) -> pygame.Surface:
        """
        Render matplotlib graphs to pygame surface.
        
        Returns:
            Pygame surface with graphs
        """
        # Clear axes
        self.ax1.clear()
        self.ax2.clear()
        
        # Plot speed vs time
        if len(self.time_buffer) > 1:
            times = list(self.time_buffer)
            speeds = list(self.speed_buffer)
            
            self.ax1.plot(times, speeds, 'b-', linewidth=2)
            self.ax1.set_xlabel('Time (s)')
            self.ax1.set_ylabel('Speed (km/h)')
            self.ax1.set_title('Speed vs Time')
            self.ax1.grid(True, alpha=0.3)
            self.ax1.set_ylim([0, max(35, max(speeds) * 1.1) if speeds else 35])
        
        # Plot steering vs time
        if len(self.time_buffer) > 1:
            times = list(self.time_buffer)
            steerings = list(self.steering_buffer)
            
            self.ax2.plot(times, steerings, 'r-', linewidth=2)
            self.ax2.set_xlabel('Time (s)')
            self.ax2.set_ylabel('Steering')
            self.ax2.set_title('Steering vs Time')
            self.ax2.grid(True, alpha=0.3)
            self.ax2.set_ylim([-1.1, 1.1])
        
        # Render to buffer
        canvas = agg.FigureCanvasAgg(self.fig)
        canvas.draw()
        renderer = canvas.get_renderer()
        
        # Handle different matplotlib versions
        try:
            raw_data = renderer.tostring_rgb()
        except AttributeError:
            # Newer matplotlib versions
            try:
                raw_data = renderer.buffer_rgba()
                # Convert RGBA to RGB
                import numpy as np
                arr = np.frombuffer(raw_data, dtype=np.uint8).reshape((*canvas.get_width_height()[::-1], 4))
                raw_data = arr[:, :, :3].tobytes()
            except AttributeError:
                # Fallback: use tostring_argb and convert
                raw_data = renderer.tostring_argb()
                import numpy as np
                arr = np.frombuffer(raw_data, dtype=np.uint8).reshape((*canvas.get_width_height()[::-1], 4))
                raw_data = arr[:, :, [2, 1, 0]].tobytes()  # ARGB to RGB
        
        # Convert to pygame surface
        size = canvas.get_width_height()
        surface = pygame.image.fromstring(raw_data, size, "RGB")
        
        return surface
    
    def close(self) -> None:
        """Close visualization display."""
        pygame.quit()
        plt.close(self.fig)
        logger.info("✅ Visualization display closed")

