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

logger = logging.getLogger(__name__)


class VisualizationDisplay:
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
        camera_image: Optional[np.ndarray],
        vehicle_state: Dict[str, Any],
        predicted_trajectory: Optional[np.ndarray] = None,
        mpc_horizon: Optional[np.ndarray] = None,
        lane_info: Optional[Dict[str, Any]] = None
    ) -> bool:
        """
        Update display with new data.
        
        Args:
            camera_image: Camera image (H, W, 3)
            vehicle_state: Current vehicle state dictionary
            predicted_trajectory: Predicted trajectory from LSTM (optional)
            mpc_horizon: MPC horizon path (optional)
            
        Returns:
            True if window should continue, False if closed
        """
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
        Draw lane visualization (lines and boxes) on camera view.
        
        Args:
            lane_info: Lane information dict with 'left_lane', 'right_lane', 'center_line', 'lane_mask'
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
            mask_surface.set_alpha(100)  # Semi-transparent
            mask_surface = pygame.transform.scale(mask_surface, (width, height))
            self.screen.blit(mask_surface, (0, 0))
        
        # Draw center line (green)
        center_line = lane_info.get('center_line')
        if center_line is not None and len(center_line) > 0:
            points = []
            for point in center_line[:20]:  # Show first 20 points
                if len(point) >= 2:
                    x = int(np.clip(point[0], 0, width))
                    y = int(np.clip(point[1], 0, height))
                    points.append((x, y))
            
            if len(points) > 1:
                pygame.draw.lines(self.screen, (0, 255, 0), False, points, 4)
                # Draw boxes along center line
                for i in range(0, len(points), 3):
                    x, y = points[i]
                    box_size = 8
                    pygame.draw.rect(self.screen, (0, 255, 0), 
                                    (x - box_size//2, y - box_size//2, box_size, box_size), 2)
        
        # Draw left lane (yellow)
        left_lane = lane_info.get('left_lane')
        if left_lane is not None and len(left_lane) > 0:
            points = []
            for point in left_lane[:20]:
                if len(point) >= 2:
                    x = int(np.clip(point[0], 0, width))
                    y = int(np.clip(point[1], 0, height))
                    points.append((x, y))
            
            if len(points) > 1:
                pygame.draw.lines(self.screen, (255, 255, 0), False, points, 3)
        
        # Draw right lane (cyan)
        right_lane = lane_info.get('right_lane')
        if right_lane is not None and len(right_lane) > 0:
            points = []
            for point in right_lane[:20]:
                if len(point) >= 2:
                    x = int(np.clip(point[0], 0, width))
                    y = int(np.clip(point[1], 0, height))
                    points.append((x, y))
            
            if len(points) > 1:
                pygame.draw.lines(self.screen, (0, 255, 255), False, points, 3)
    
    def _draw_trajectory_overlay(self, trajectory: np.ndarray, width: int, height: int) -> None:
        """
        Draw predicted trajectory overlay on camera view.
        
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
        
        # Project trajectory to camera view (simplified: just draw as path)
        # In real implementation, would need proper camera projection
        # For now, draw as a simple path indicator
        points = []
        for i in range(min(len(traj_points), 10)):  # Show first 10 points
            # Simple projection: assume trajectory is in front of vehicle
            # Map to bottom center of image
            x_screen = int(width // 2 + traj_points[i, 0] * 10)  # Scale factor
            y_screen = int(height - 50 - i * 5)  # Draw from bottom
            
            x_screen = np.clip(x_screen, 0, width)
            y_screen = np.clip(y_screen, 0, height)
            points.append((x_screen, y_screen))
        
        # Draw trajectory line (blue)
        if len(points) > 1:
            pygame.draw.lines(self.screen, (0, 150, 255), False, points, 4)
            # Draw boxes along trajectory
            for i, point in enumerate(points[::2]):  # Every other point
                box_size = 6
                pygame.draw.rect(self.screen, (0, 150, 255), 
                                (point[0] - box_size//2, point[1] - box_size//2, 
                                 box_size, box_size), 2)
    
    def _draw_mpc_horizon(self, mpc_horizon: np.ndarray, width: int, height: int) -> None:
        """
        Draw MPC horizon path on camera view.
        
        Args:
            mpc_horizon: MPC predicted trajectory (N+1, 4)
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
        points = []
        for i in range(min(len(horizon_points), self.config.get('mpc', {}).get('horizon', 10) + 1)):
            # Project to screen (simplified)
            x_screen = int(width // 2 + horizon_points[i, 0] * 8)
            y_screen = int(height - 30 - i * 4)
            
            x_screen = np.clip(x_screen, 0, width)
            y_screen = np.clip(y_screen, 0, height)
            points.append((x_screen, y_screen))
        
        # Draw MPC horizon (magenta)
        if len(points) > 1:
            pygame.draw.lines(self.screen, (255, 0, 255), False, points, 3)
            # Draw boxes
            for i, point in enumerate(points[::2]):
                box_size = 5
                pygame.draw.rect(self.screen, (255, 0, 255), 
                                (point[0] - box_size//2, point[1] - box_size//2, 
                                 box_size, box_size), 1)
    
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

