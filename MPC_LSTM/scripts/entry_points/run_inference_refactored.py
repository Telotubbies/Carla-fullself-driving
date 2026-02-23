#!/usr/bin/env python3
"""
Run inference using refactored system with AI engineering improvements.
"""

import sys
import logging
import time
from pathlib import Path

# Add project root to path
project_root = Path(__file__).parent.parent.parent
sys.path.insert(0, str(project_root))

from core.config import ConfigManager
from core.system import AutonomousDrivingSystem
from core.exceptions import ConfigurationError, ModelLoadError, CARLAConnectionError
from carla_env import CarlaClient, CameraSensor
from perception.lane_detector import LaneDetector
from utils.device_utils import get_device_info

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler('logs/inference_refactored.log'),
        logging.StreamHandler()
    ]
)
logger = logging.getLogger(__name__)


def main():
    """Main entry point."""
    logger.info("=" * 60)
    logger.info("CARLA LSTM-MPC Autonomous Driving - Refactored System")
    logger.info("=" * 60)
    
    try:
        # Load configuration
        logger.info("Loading configuration...")
        config_manager = ConfigManager("config.yaml", environment="development")
        logger.info("✅ Configuration loaded")
        
        # Display device info
        device_info = get_device_info()
        logger.info(f"Device: {device_info['device_name']} ({device_info['device_type']})")
        
        # Create system using factories (dependency injection ready)
        logger.info("Initializing autonomous driving system...")
        system = AutonomousDrivingSystem(config_manager)
        logger.info("✅ System initialized")
        
        # Initialize CARLA
        logger.info("Initializing CARLA environment...")
        logger.info("💡 Waiting for CARLA to be ready (this may take 10-30 seconds)...")
        carla_client = CarlaClient(config_manager.get_section('carla'))
        
        # Try connecting with retries
        max_connection_retries = 10
        connected = False
        for i in range(max_connection_retries):
            if carla_client.connect(max_retries=3):
                connected = True
                break
            else:
                if i < max_connection_retries - 1:
                    wait_time = min(5 + (i * 2), 15)
                    logger.info(f"CARLA not ready yet, waiting {wait_time}s before retry...")
                    time.sleep(wait_time)
        
        if not connected:
            raise CARLAConnectionError(f"Failed to connect to CARLA after {max_connection_retries} attempts. Make sure CARLA is running.")
        
        if not carla_client.load_world(retries=5):
            raise CARLAConnectionError("Failed to load CARLA world after multiple attempts")
        
        if not carla_client.spawn_vehicle():
            raise CARLAConnectionError("Failed to spawn vehicle")
        
        logger.info("✅ CARLA environment initialized")
        
        # Setup camera
        try:
            camera = CameraSensor(
                carla_client.world,
                carla_client.vehicle,
                config_manager.get_section('camera')
            )
            logger.info("✅ Camera sensor initialized")
        except Exception as e:
            raise CARLAConnectionError(f"Failed to setup camera: {e}")
        
        # Setup lane detector from config
        try:
            perception_config = config_manager.get_section('perception')
            model_path = perception_config.get('lane_detection_model_path', None)
            model_type = perception_config.get('lane_detection_model_type', 'unet')
            use_carla = perception_config.get('use_carla_lane_detection', True)
            
            if model_path and Path(model_path).exists():
                logger.info(f"Loading {model_type} model from {model_path}")
                lane_detector = LaneDetector(
                    model_path=model_path,
                    model_type=model_type,
                    use_carla=False
                )
                logger.info(f"✅ Lane detector initialized with {model_type} model")
            else:
                lane_detector = LaneDetector(use_carla=use_carla)
                if use_carla:
                    logger.info("✅ Lane detector initialized with CARLA detection")
                else:
                    logger.warning("⚠️  No model found, using CARLA detection")
        except Exception as e:
            logger.warning(f"⚠️  Failed to initialize lane detector: {e}, continuing without lane detection")
            lane_detector = None
        
        # Inject dependencies
        system.set_carla_client(carla_client)
        system.set_camera(camera)
        system.set_lane_detector(lane_detector)
        
        # Run inference loop
        logger.info("Starting inference loop...")
        logger.info("Press Ctrl+C to stop")
        logger.info("")
        
        system.running = True
        step_count = 0
        
        try:
            while system.running:
                continue_running, step_info = system.run_inference_step()
                
                if not continue_running:
                    logger.info("Visualization window closed")
                    break
                
                step_count += 1
                
                # Log progress every 100 steps
                if step_count % 100 == 0:
                    if step_info:
                        state = step_info.get('vehicle_state', {})
                        speed_kmh = state.get('velocity', 0) * 3.6
                        logger.info(f"Step {step_count}: Speed={speed_kmh:.1f} km/h")
                
                # Limit frame rate
                time.sleep(0.01)
        
        except KeyboardInterrupt:
            logger.info("Received interrupt signal, shutting down...")
        except Exception as e:
            logger.error(f"Error in inference loop: {e}", exc_info=True)
        finally:
            # Cleanup
            system.cleanup()
            
            # Destroy camera
            if camera:
                camera.destroy()
            
            # Cleanup CARLA
            carla_client.cleanup()
            
            logger.info(f"✅ Inference complete. Total steps: {step_count}")
            logger.info("=" * 60)
    
    except ConfigurationError as e:
        logger.error(f"Configuration error: {e}")
        return 1
    except ModelLoadError as e:
        logger.error(f"Model loading error: {e}")
        return 1
    except CARLAConnectionError as e:
        logger.error(f"CARLA connection error: {e}")
        logger.info("💡 Make sure CARLA is running: ./CarlaUE4.sh")
        return 1
    except Exception as e:
        logger.error(f"Unexpected error: {e}", exc_info=True)
        return 1
    
    return 0


if __name__ == '__main__':
    sys.exit(main())

