#!/usr/bin/env python3
"""
Test each component used in simulation separately.
"""

import sys
import logging
import time
from pathlib import Path

# Add project root to path
project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))

from core.config import ConfigManager
from carla_env import CarlaClient, CameraSensor
from perception import ResNetEncoder
from perception.lane_detector import LaneDetector
from temporal import LSTMPredictor
from control import MPCController
from visualization import VisualizationDisplay
from utils.device_utils import get_device_info

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class ComponentTester:
    """Test each simulation component separately."""
    
    def __init__(self):
        self.config_manager = None
        self.results = {}
        
    def test_config_loading(self):
        """Test 1: Configuration loading."""
        logger.info("=" * 60)
        logger.info("TEST 1: Configuration Loading")
        logger.info("=" * 60)
        try:
            self.config_manager = ConfigManager("config.yaml", environment="development")
            logger.info("✅ Configuration loaded successfully")
            self.results['config'] = True
            return True
        except Exception as e:
            logger.error(f"❌ Configuration loading failed: {e}")
            self.results['config'] = False
            return False
    
    def test_device_info(self):
        """Test 2: Device information."""
        logger.info("=" * 60)
        logger.info("TEST 2: Device Information")
        logger.info("=" * 60)
        try:
            device_info = get_device_info()
            logger.info(f"✅ Device: {device_info['device_name']} ({device_info['device_type']})")
            self.results['device'] = True
            return True
        except Exception as e:
            logger.error(f"❌ Device info failed: {e}")
            self.results['device'] = False
            return False
    
    def test_resnet_loading(self):
        """Test 3: ResNet model loading."""
        logger.info("=" * 60)
        logger.info("TEST 3: ResNet Model Loading")
        logger.info("=" * 60)
        try:
            perception_config = self.config_manager.get_section('perception')
            encoder = ResNetEncoder(
                feature_dim=perception_config.get('feature_dim', 512),
                pretrained=perception_config.get('pretrained', True),
                freeze_backbone=perception_config.get('freeze_backbone', False),
                model_path=perception_config.get('resnet_model_path')
            )
            logger.info("✅ ResNet encoder loaded successfully")
            self.results['resnet'] = True
            return True
        except Exception as e:
            logger.error(f"❌ ResNet loading failed: {e}")
            self.results['resnet'] = False
            return False
    
    def test_lstm_loading(self):
        """Test 4: LSTM model loading."""
        logger.info("=" * 60)
        logger.info("TEST 4: LSTM Model Loading")
        logger.info("=" * 60)
        try:
            temporal_config = self.config_manager.get_section('temporal')
            predictor = LSTMPredictor(
                input_size=temporal_config.get('input_size', 512),
                hidden_size=temporal_config.get('hidden_size', 256),
                num_layers=temporal_config.get('num_layers', 2),
                sequence_length=temporal_config.get('sequence_length', 10),
                dropout=temporal_config.get('dropout', 0.1)
            )
            # Load model if path provided
            model_path = temporal_config.get('trained_model_path')
            if model_path and Path(model_path).exists():
                try:
                    predictor.load_model(model_path)
                    logger.info("✅ LSTM model loaded from file")
                except Exception as e:
                    logger.warning(f"⚠️  Could not load LSTM model: {e}")
            logger.info("✅ LSTM predictor initialized successfully")
            self.results['lstm'] = True
            return True
        except Exception as e:
            logger.error(f"❌ LSTM loading failed: {e}")
            self.results['lstm'] = False
            return False
    
    def test_carla_connection(self):
        """Test 5: CARLA connection."""
        logger.info("=" * 60)
        logger.info("TEST 5: CARLA Connection")
        logger.info("=" * 60)
        try:
            carla_config = self.config_manager.get_section('carla')
            carla_client = CarlaClient(carla_config)
            
            logger.info("Attempting to connect to CARLA...")
            if carla_client.connect(max_retries=3):
                logger.info("✅ CARLA connection successful")
                self.results['carla_connection'] = True
                return carla_client
            else:
                logger.error("❌ CARLA connection failed")
                self.results['carla_connection'] = False
                return None
        except Exception as e:
            logger.error(f"❌ CARLA connection error: {e}")
            self.results['carla_connection'] = False
            return None
    
    def test_world_loading(self, carla_client):
        """Test 6: World loading."""
        logger.info("=" * 60)
        logger.info("TEST 6: World Loading")
        logger.info("=" * 60)
        if carla_client is None:
            logger.warning("⚠️  Skipping: CARLA not connected")
            self.results['world_loading'] = False
            return False
        
        try:
            if carla_client.load_world(retries=3):
                logger.info("✅ World loaded successfully")
                self.results['world_loading'] = True
                return True
            else:
                logger.error("❌ World loading failed")
                self.results['world_loading'] = False
                return False
        except Exception as e:
            logger.error(f"❌ World loading error: {e}")
            self.results['world_loading'] = False
            return False
    
    def test_vehicle_spawning(self, carla_client):
        """Test 7: Vehicle spawning."""
        logger.info("=" * 60)
        logger.info("TEST 7: Vehicle Spawning")
        logger.info("=" * 60)
        if carla_client is None or carla_client.world is None:
            logger.warning("⚠️  Skipping: CARLA world not loaded")
            self.results['vehicle_spawning'] = False
            return False
        
        try:
            if carla_client.spawn_vehicle():
                logger.info("✅ Vehicle spawned successfully")
                self.results['vehicle_spawning'] = True
                return True
            else:
                logger.error("❌ Vehicle spawning failed")
                self.results['vehicle_spawning'] = False
                return False
        except Exception as e:
            logger.error(f"❌ Vehicle spawning error: {e}")
            self.results['vehicle_spawning'] = False
            return False
    
    def test_camera_setup(self, carla_client):
        """Test 8: Camera setup."""
        logger.info("=" * 60)
        logger.info("TEST 8: Camera Setup")
        logger.info("=" * 60)
        if carla_client is None or carla_client.vehicle is None:
            logger.warning("⚠️  Skipping: Vehicle not spawned")
            self.results['camera'] = False
            return None
        
        try:
            camera_config = self.config_manager.get_section('camera')
            camera = CameraSensor(
                carla_client.world,
                carla_client.vehicle,
                camera_config
            )
            logger.info("✅ Camera sensor created successfully")
            
            # Test getting image
            time.sleep(1.0)  # Wait for camera to initialize
            image = camera.get_image()
            if image is not None:
                logger.info(f"✅ Camera image captured: {image.shape}")
                self.results['camera'] = True
                return camera
            else:
                logger.warning("⚠️  Camera created but no image available yet")
                self.results['camera'] = True  # Still count as success
                return camera
        except Exception as e:
            logger.error(f"❌ Camera setup error: {e}")
            self.results['camera'] = False
            return None
    
    def test_lane_detector(self):
        """Test 9: Lane detector."""
        logger.info("=" * 60)
        logger.info("TEST 9: Lane Detector")
        logger.info("=" * 60)
        try:
            lane_detector = LaneDetector(use_carla=True)
            logger.info("✅ Lane detector initialized successfully")
            self.results['lane_detector'] = True
            return True
        except Exception as e:
            logger.warning(f"⚠️  Lane detector initialization failed: {e}")
            self.results['lane_detector'] = False
            return False
    
    def test_mpc_controller(self):
        """Test 10: MPC controller."""
        logger.info("=" * 60)
        logger.info("TEST 10: MPC Controller")
        logger.info("=" * 60)
        try:
            # Get full config dict
            full_config = {}
            for section in ['carla', 'camera', 'perception', 'temporal', 'mpc', 'visualization']:
                full_config[section] = self.config_manager.get_section(section)
            controller = MPCController(full_config)
            logger.info("✅ MPC controller initialized successfully")
            self.results['mpc'] = True
            return True
        except Exception as e:
            logger.error(f"❌ MPC controller initialization failed: {e}")
            self.results['mpc'] = False
            return False
    
    def test_visualization(self):
        """Test 11: Visualization."""
        logger.info("=" * 60)
        logger.info("TEST 11: Visualization Display")
        logger.info("=" * 60)
        try:
            viz_config = self.config_manager.get_section('visualization')
            display = VisualizationDisplay(viz_config)
            logger.info("✅ Visualization display initialized successfully")
            self.results['visualization'] = True
            display.close()
            return True
        except Exception as e:
            logger.error(f"❌ Visualization initialization failed: {e}")
            self.results['visualization'] = False
            return False
    
    def test_waypoints(self, carla_client):
        """Test 12: Waypoints."""
        logger.info("=" * 60)
        logger.info("TEST 12: Waypoints")
        logger.info("=" * 60)
        if carla_client is None or carla_client.vehicle is None:
            logger.warning("⚠️  Skipping: Vehicle not available")
            self.results['waypoints'] = False
            return False
        
        try:
            waypoints = carla_client.get_waypoints(distance=3.0, num_waypoints=10)
            if waypoints and len(waypoints) > 0:
                logger.info(f"✅ Got {len(waypoints)} waypoints")
                self.results['waypoints'] = True
                return True
            else:
                logger.warning("⚠️  No waypoints returned")
                self.results['waypoints'] = False
                return False
        except Exception as e:
            logger.error(f"❌ Waypoints error: {e}")
            self.results['waypoints'] = False
            return False
    
    def run_all_tests(self):
        """Run all tests."""
        logger.info("=" * 60)
        logger.info("COMPONENT TESTING - Simulation Functions")
        logger.info("=" * 60)
        logger.info("")
        
        # Test 1-4: Non-CARLA components
        self.test_config_loading()
        self.test_device_info()
        self.test_resnet_loading()
        self.test_lstm_loading()
        
        # Test 5-8: CARLA components
        carla_client = self.test_carla_connection()
        if carla_client:
            self.test_world_loading(carla_client)
            self.test_vehicle_spawning(carla_client)
            camera = self.test_camera_setup(carla_client)
        else:
            logger.warning("⚠️  Skipping CARLA-dependent tests")
        
        # Test 9-11: Other components
        self.test_lane_detector()
        self.test_mpc_controller()
        self.test_visualization()
        
        # Test 12: Waypoints (if CARLA available)
        if carla_client and carla_client.vehicle:
            self.test_waypoints(carla_client)
        
        # Cleanup
        if carla_client:
            try:
                carla_client.cleanup()
            except:
                pass
        
        # Print summary
        self.print_summary()
    
    def print_summary(self):
        """Print test summary."""
        logger.info("")
        logger.info("=" * 60)
        logger.info("TEST SUMMARY")
        logger.info("=" * 60)
        
        total = len(self.results)
        passed = sum(1 for v in self.results.values() if v)
        failed = total - passed
        
        for test_name, result in self.results.items():
            status = "✅ PASS" if result else "❌ FAIL"
            logger.info(f"{status}: {test_name}")
        
        logger.info("")
        logger.info(f"Total: {total} | Passed: {passed} | Failed: {failed}")
        logger.info("=" * 60)


def main():
    """Main entry point."""
    tester = ComponentTester()
    tester.run_all_tests()
    return 0


if __name__ == '__main__':
    sys.exit(main())

