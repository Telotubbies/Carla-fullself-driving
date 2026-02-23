"""
Sensor management for CARLA environment.

Handles camera and other sensor setup and data collection.
"""

import carla
import numpy as np
import logging
from typing import Optional, Callable, Any
import queue
import threading

logger = logging.getLogger(__name__)


class CameraSensor:
    """RGB Camera sensor for CARLA."""
    
    def __init__(
        self,
        world: carla.World,
        vehicle: carla.Vehicle,
        config: dict,
        callback: Optional[Callable] = None
    ):
        """
        Initialize camera sensor.
        
        Args:
            world: CARLA world object
            vehicle: CARLA vehicle to attach camera to
            config: Camera configuration dictionary
            callback: Optional callback function for image data
        """
        self.world = world
        self.vehicle = vehicle
        self.config = config
        self.callback = callback
        
        self.sensor: Optional[carla.Sensor] = None
        self.image_queue = queue.Queue(maxsize=10)
        self.latest_image: Optional[np.ndarray] = None
        
        self._setup_camera()
    
    def _setup_camera(self) -> None:
        """Setup and attach camera to vehicle."""
        try:
            # Get camera blueprint
            bp_library = self.world.get_blueprint_library()
            camera_bp = bp_library.find('sensor.camera.rgb')
            
            # Set camera attributes
            camera_bp.set_attribute('image_size_x', str(self.config['width']))
            camera_bp.set_attribute('image_size_y', str(self.config['height']))
            camera_bp.set_attribute('fov', str(self.config['fov']))
            camera_bp.set_attribute('sensor_tick', str(1.0 / self.config['fps']))
            
            # Camera transform
            camera_location = carla.Location(
                x=self.config['location']['x'],
                y=self.config['location']['y'],
                z=self.config['location']['z']
            )
            camera_rotation = carla.Rotation(
                pitch=self.config['rotation']['pitch'],
                yaw=self.config['rotation']['yaw'],
                roll=self.config['rotation']['roll']
            )
            camera_transform = carla.Transform(camera_location, camera_rotation)
            
            # Spawn camera
            self.sensor = self.world.spawn_actor(
                camera_bp,
                camera_transform,
                attach_to=self.vehicle
            )
            
            # Set callback
            self.sensor.listen(self._on_image)
            
            logger.info("✅ Camera sensor created and attached")
            
        except Exception as e:
            logger.error(f"❌ Failed to setup camera: {e}")
            raise
    
    def _on_image(self, image: carla.Image) -> None:
        """
        Callback for camera image data.
        
        Args:
            image: CARLA Image object
        """
        try:
            # Convert to numpy array
            array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
            array = np.reshape(array, (image.height, image.width, 4))
            array = array[:, :, :3]  # Remove alpha channel
            array = array[:, :, ::-1]  # Convert BGRA to RGB
            
            # Update latest image
            self.latest_image = array.copy()
            
            # Add to queue (non-blocking)
            try:
                self.image_queue.put_nowait(array)
            except queue.Full:
                # Remove oldest if queue is full
                try:
                    self.image_queue.get_nowait()
                    self.image_queue.put_nowait(array)
                except queue.Empty:
                    pass
            
            # Call external callback if provided
            if self.callback is not None:
                self.callback(array)
                
        except Exception as e:
            logger.error(f"Error processing image: {e}")
    
    def get_image(self) -> Optional[np.ndarray]:
        """
        Get latest camera image.
        
        Returns:
            Latest image as numpy array (H, W, 3) or None
        """
        return self.latest_image
    
    def get_image_from_queue(self, timeout: float = 0.1) -> Optional[np.ndarray]:
        """
        Get image from queue (blocking).
        
        Args:
            timeout: Timeout in seconds
            
        Returns:
            Image as numpy array or None if timeout
        """
        try:
            return self.image_queue.get(timeout=timeout)
        except queue.Empty:
            return None
    
    def destroy(self) -> None:
        """Destroy camera sensor."""
        if self.sensor is not None:
            try:
                self.sensor.destroy()
                logger.info("✅ Camera sensor destroyed")
            except Exception as e:
                logger.warning(f"Error destroying camera: {e}")

