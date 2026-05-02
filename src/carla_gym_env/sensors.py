import carla
import numpy as np
import weakref
from typing import Dict, Callable, Optional
import queue


class SensorManager:
    """Manages all sensors attached to the ego vehicle."""
    
    def __init__(self, world: carla.World, vehicle: carla.Actor, config: Dict):
        self.world = world
        self.vehicle = vehicle
        self.config = config
        
        # Sensor actors
        self.sensors = {}
        
        # Data queues
        self.lidar_queue = queue.Queue()
        self.camera_queue = queue.Queue()
        self.collision_queue = queue.Queue()
        
        # Callbacks
        self.collision_callback = None
        
        # Latest data
        self.latest_lidar_data = None
        self.latest_camera_data = None
    
    def setup_sensors(self):
        """Setup sensors.

        Collision is always attached (needed for termination). LiDAR and RGB
        camera are skipped when `disable_perception_sensors=True`; this is
        used when an upstream wrapper supplies ground-truth observations and
        perception sensors would only add cost.
        """
        if not self.config.get('disable_perception_sensors', False):
            self._setup_lidar()
            self._setup_camera()
        self._setup_collision_sensor()
    
    def _setup_lidar(self):
        """Setup LiDAR sensor."""
        blueprint_library = self.world.get_blueprint_library()
        lidar_bp = blueprint_library.find('sensor.lidar.ray_cast')
        
        # Configure LiDAR
        lidar_bp.set_attribute('channels', str(self.config.get('lidar_channels', 64)))
        lidar_bp.set_attribute('range', str(self.config.get('lidar_range', 100)))
        lidar_bp.set_attribute('points_per_second', str(self.config.get('lidar_pps', 500000)))
        lidar_bp.set_attribute('rotation_frequency', str(self.config.get('lidar_freq', 20)))
        lidar_bp.set_attribute('upper_fov', str(self.config.get('lidar_upper_fov', 15)))
        lidar_bp.set_attribute('lower_fov', str(self.config.get('lidar_lower_fov', -25)))
        
        # Spawn LiDAR
        lidar_transform = carla.Transform(carla.Location(x=0.0, z=2.5))
        lidar = self.world.spawn_actor(lidar_bp, lidar_transform, attach_to=self.vehicle)
        
        # Register callback
        weak_self = weakref.ref(self)
        lidar.listen(lambda data: SensorManager._lidar_callback(weak_self, data))
        
        self.sensors['lidar'] = lidar
    
    def _setup_camera(self):
        """Setup RGB camera."""
        blueprint_library = self.world.get_blueprint_library()
        camera_bp = blueprint_library.find('sensor.camera.rgb')
        
        # Configure camera
        camera_bp.set_attribute('image_size_x', str(self.config.get('camera_width', 640)))
        camera_bp.set_attribute('image_size_y', str(self.config.get('camera_height', 480)))
        camera_bp.set_attribute('fov', str(self.config.get('camera_fov', 90)))
        
        # Spawn camera
        camera_transform = carla.Transform(
            carla.Location(x=1.5, z=2.4),
            carla.Rotation(pitch=-15)
        )
        camera = self.world.spawn_actor(camera_bp, camera_transform, attach_to=self.vehicle)
        
        # Register callback
        weak_self = weakref.ref(self)
        camera.listen(lambda data: SensorManager._camera_callback(weak_self, data))
        
        self.sensors['camera'] = camera
    
    def _setup_collision_sensor(self):
        """Setup collision sensor."""
        blueprint_library = self.world.get_blueprint_library()
        collision_bp = blueprint_library.find('sensor.other.collision')
        
        # Spawn collision sensor
        collision_sensor = self.world.spawn_actor(
            collision_bp,
            carla.Transform(),
            attach_to=self.vehicle
        )
        
        # Register callback
        weak_self = weakref.ref(self)
        collision_sensor.listen(lambda event: SensorManager._collision_callback(weak_self, event))
        
        self.sensors['collision'] = collision_sensor
    
    @staticmethod
    def _lidar_callback(weak_self, data):
        """Callback for LiDAR data."""
        self = weak_self()
        if not self:
            return
        
        self.latest_lidar_data = data
        try:
            self.lidar_queue.put(data, block=False)
        except queue.Full:
            pass
    
    @staticmethod
    def _camera_callback(weak_self, data):
        """Callback for camera data."""
        self = weak_self()
        if not self:
            return
        
        self.latest_camera_data = data
        try:
            self.camera_queue.put(data, block=False)
        except queue.Full:
            pass
    
    @staticmethod
    def _collision_callback(weak_self, event):
        """Callback for collision events."""
        self = weak_self()
        if not self:
            return
        
        try:
            self.collision_queue.put(event, block=False)
        except queue.Full:
            pass
        
        if self.collision_callback is not None:
            self.collision_callback(event)
    
    def register_collision_callback(self, callback: Callable):
        """Register external collision callback."""
        self.collision_callback = callback
    
    def get_lidar_bev(self) -> np.ndarray:
        """Convert LiDAR point cloud to BEV occupancy grid."""
        if self.latest_lidar_data is None:
            return np.zeros((256, 256, 3), dtype=np.uint8)
        
        # Get point cloud
        points = np.frombuffer(self.latest_lidar_data.raw_data, dtype=np.float32)
        points = np.reshape(points, (int(points.shape[0] / 4), 4))
        
        # Extract x, y, z, intensity
        x = points[:, 0]
        y = points[:, 1]
        z = points[:, 2]
        intensity = points[:, 3]
        
        # Create BEV grid
        bev_range = self.config.get('bev_range', 50.0)  # meters
        bev_resolution = self.config.get('bev_resolution', 256)
        
        # Convert to grid coordinates
        pixel_per_meter = bev_resolution / (2 * bev_range)
        x_img = (-y * pixel_per_meter + bev_resolution / 2).astype(np.int32)
        y_img = (-x * pixel_per_meter + bev_resolution / 2).astype(np.int32)
        
        # Filter points within grid
        mask = (x_img >= 0) & (x_img < bev_resolution) & \
               (y_img >= 0) & (y_img < bev_resolution)
        
        x_img = x_img[mask]
        y_img = y_img[mask]
        z_vals = z[mask]
        intensity_vals = intensity[mask]
        
        # Create multi-channel BEV
        bev = np.zeros((bev_resolution, bev_resolution, 3), dtype=np.uint8)
        
        if len(x_img) > 0:
            # Channel 0: Height (normalized)
            z_normalized = np.clip((z_vals + 2.0) / 4.0, 0, 1) * 255
            bev[y_img, x_img, 0] = z_normalized.astype(np.uint8)
            
            # Channel 1: Intensity
            intensity_normalized = np.clip(intensity_vals * 255, 0, 255)
            bev[y_img, x_img, 1] = intensity_normalized.astype(np.uint8)
            
            # Channel 2: Density (count of points)
            # Cast to int to avoid uint8 overflow in the `+ 10` before min().
            for i in range(len(x_img)):
                bev[y_img[i], x_img[i], 2] = min(
                    int(bev[y_img[i], x_img[i], 2]) + 10, 255,
                )
        
        return bev
    
    def get_camera_image(self) -> Optional[np.ndarray]:
        """Get latest camera image as numpy array."""
        if self.latest_camera_data is None:
            return None
        
        # Convert to numpy array
        array = np.frombuffer(self.latest_camera_data.raw_data, dtype=np.uint8)
        array = np.reshape(array, (self.latest_camera_data.height, self.latest_camera_data.width, 4))
        array = array[:, :, :3]  # Remove alpha channel
        
        return array
    
    def destroy(self):
        """Destroy all sensors. stop() before destroy() to cleanly close the
        stream listener and avoid CARLA's 'Invalid session: no stream available'
        log spam (which can fill disk with millions of error lines)."""
        for sensor in self.sensors.values():
            if sensor is not None and sensor.is_alive:
                try:
                    sensor.stop()
                except Exception:
                    pass
                try:
                    sensor.destroy()
                except Exception:
                    pass
        
        self.sensors.clear()
        
        # Clear queues
        while not self.lidar_queue.empty():
            try:
                self.lidar_queue.get_nowait()
            except queue.Empty:
                break
        
        while not self.camera_queue.empty():
            try:
                self.camera_queue.get_nowait()
            except queue.Empty:
                break
        
        while not self.collision_queue.empty():
            try:
                self.collision_queue.get_nowait()
            except queue.Empty:
                break
