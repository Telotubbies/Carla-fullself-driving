import carla
import logging
import time
from typing import Optional, Tuple
class CarlaConnectionManager:
    
    def __init__(self, host: str = 'localhost', port: int = 2000, timeout: float = 120.0):
        
        self.host = host
        self.port = port
        self.timeout = timeout
        self.client: Optional[carla.Client] = None
        self.world: Optional[carla.World] = None
    def connect(self, max_retries: int = 20, town: Optional[str] = None) -> Tuple[carla.Client, carla.World]:
        
        for attempt in range(max_retries):
            try:
                self.client = carla.Client(self.host, self.port)
                self.client.set_timeout(self.timeout)
                if town:
                    self.world = self._load_world(town)
                else:
                    self.world = self.client.get_world()
                logging.info(f"Connected to CARLA at {self.host}:{self.port}")
                return self.client, self.world
            except RuntimeError as e:
                if attempt == max_retries - 1:
                    logging.error(f"Failed to connect after {max_retries} attempts")
                    raise
                wait_time = min(5 + (attempt * 2), 15)
                logging.warning(f"Connection failed ({e}). Retrying in {wait_time}s...")
                time.sleep(wait_time)
        raise RuntimeError("Connection failed after all retries")
    def _load_world(self, town: str) -> carla.World:
        
        try:
            world = self.client.load_world(town, reset_settings=False)
            logging.info(f"Loaded town: {town}")
            return world
        except RuntimeError:
            world = self.client.get_world()
            current_map = world.get_map().name
            if current_map != town:
                logging.info(f"Loading requested town {town}...")
                world = self.client.load_world(town, reset_settings=False)
            else:
                logging.info(f"Using current town: {town}")
            return world
    def disconnect(self) -> None:
        
        if self.world:
            try:
                settings = self.world.get_settings()
                settings.synchronous_mode = False
                self.world.apply_settings(settings)
            except Exception as e:
                logging.debug(f"Error resetting world settings: {e}")
        self.client = None
        self.world = None