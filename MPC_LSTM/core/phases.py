"""
Phase definitions for the CARLA LSTM-MPC pipeline.

Each phase represents a distinct step in the workflow.
"""

import sys
import logging
from pathlib import Path
from typing import Dict, Any, Optional

# Add project root to path
PROJECT_ROOT = Path(__file__).parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

logger = logging.getLogger(__name__)


def create_phase_functions(config_manager, project_root: Path) -> Dict[str, Any]:
    """
    Create phase execution functions.
    
    Args:
        config_manager: ConfigManager instance
        project_root: Project root directory
    
    Returns:
        Dictionary of phase functions
    """
    phases = {}
    
    # Phase 1: Data Collection
    def phase_collect_data() -> bool:
        """Phase 1: Collect training data from CARLA."""
        try:
            from training.collect_autopilot_data import collect_autopilot_data
            
            config = config_manager.get_section('data_collection')
            frames = config.get('frames', 20000)
            output_dir = project_root / "data"
            
            logger.info(f"Collecting {frames} frames...")
            result = collect_autopilot_data(
                frames=frames,
                output_dir=str(output_dir)
            )
            
            return result is not None
        except Exception as e:
            logger.error(f"Data collection failed: {e}")
            return False
    
    phases['collect_data'] = phase_collect_data
    
    # Phase 2: Create Lane Labels
    def phase_create_lane_labels() -> bool:
        """Phase 2: Create lane masks from CARLA."""
        try:
            from training.create_lane_labels import create_lane_labels_from_carla
            from carla_env.carla_client import CarlaClient
            
            # Find latest data directory
            data_dirs = sorted((project_root / "data").glob("autopilot_*"), reverse=True)
            if not data_dirs:
                logger.error("No data directory found")
                return False
            
            latest_data = data_dirs[0]
            images_dir = latest_data / "images"
            masks_dir = latest_data / "lane_masks"
            
            if not images_dir.exists():
                logger.error(f"Images directory not found: {images_dir}")
                return False
            
            masks_dir.mkdir(exist_ok=True)
            
            # Connect to CARLA
            client = CarlaClient()
            if not client.connect():
                logger.error("Failed to connect to CARLA")
                return False
            
            if not client.load_world():
                logger.error("Failed to load CARLA world")
                return False
            
            # Create labels
            create_lane_labels_from_carla(
                str(images_dir),
                str(masks_dir),
                client.world,
                client.vehicle
            )
            
            return True
        except Exception as e:
            logger.error(f"Lane label creation failed: {e}")
            return False
    
    phases['create_lane_labels'] = phase_create_lane_labels
    
    # Phase 3: Fine-tune ResNet
    def phase_finetune_resnet() -> bool:
        """Phase 3: Fine-tune ResNet for lane detection."""
        try:
            from training.finetune_resnet_lane import finetune_resnet_lane
            
            # Find latest data directory
            data_dirs = sorted((project_root / "data").glob("autopilot_*"), reverse=True)
            if not data_dirs:
                logger.error("No data directory found")
                return False
            
            latest_data = data_dirs[0]
            masks_dir = latest_data / "lane_masks"
            
            if not masks_dir.exists() or len(list(masks_dir.glob("*.png"))) < 1000:
                logger.error("Insufficient lane masks")
                return False
            
            config = config_manager.get_section('training')
            resnet_config = config.get('resnet', {})
            
            result = finetune_resnet_lane(
                data_dir=str(latest_data),
                epochs=resnet_config.get('epochs', 300),
                batch_size=resnet_config.get('batch_size', 16),
                lr=resnet_config.get('learning_rate', 0.0003),
                masks_dir=str(masks_dir)
            )
            
            return result is not None
        except Exception as e:
            logger.error(f"ResNet fine-tuning failed: {e}")
            return False
    
    phases['finetune_resnet'] = phase_finetune_resnet
    
    # Phase 4: Extract Features
    def phase_extract_features() -> bool:
        """Phase 4: Extract features using ResNet."""
        try:
            from training.extract_features import extract_features
            
            # Find latest data directory
            data_dirs = sorted((project_root / "data").glob("autopilot_*"), reverse=True)
            if not data_dirs:
                logger.error("No data directory found")
                return False
            
            latest_data = data_dirs[0]
            
            # Get ResNet model path
            resnet_model = latest_data / "resnet_lane_model" / "resnet_lane_final.pth"
            if not resnet_model.exists():
                logger.warning("ResNet model not found, using pretrained")
                resnet_model = None
            
            result = extract_features(
                data_dir=str(latest_data),
                model_path=str(resnet_model) if resnet_model else None
            )
            
            return result
        except Exception as e:
            logger.error(f"Feature extraction failed: {e}")
            return False
    
    phases['extract_features'] = phase_extract_features
    
    # Phase 5: Train LSTM
    def phase_train_lstm() -> bool:
        """Phase 5: Train LSTM model."""
        try:
            from training.train_lstm import train_lstm
            
            # Find latest data directory
            data_dirs = sorted((project_root / "data").glob("autopilot_*"), reverse=True)
            if not data_dirs:
                logger.error("No data directory found")
                return False
            
            latest_data = data_dirs[0]
            
            config = config_manager.get_section('training')
            lstm_config = config.get('lstm', {})
            temporal_config = config_manager.get_section('temporal')
            
            result = train_lstm(
                data_dir=str(latest_data),
                epochs=lstm_config.get('epochs', 200),
                batch_size=lstm_config.get('batch_size', 64),
                learning_rate=lstm_config.get('learning_rate', 0.0005),
                hidden_size=temporal_config.get('hidden_size', 256),
                num_layers=temporal_config.get('num_layers', 2),
                use_attention=lstm_config.get('use_attention', True),
                use_advanced_loss=lstm_config.get('use_advanced_loss', True),
                early_stopping_patience=lstm_config.get('early_stopping', 20)
            )
            
            return result is not None
        except Exception as e:
            logger.error(f"LSTM training failed: {e}")
            return False
    
    phases['train_lstm'] = phase_train_lstm
    
    # Phase 6: Update Config
    def phase_update_config() -> bool:
        """Phase 6: Update config.yaml with trained models."""
        try:
            import yaml
            
            # Find latest data directory
            data_dirs = sorted((project_root / "data").glob("autopilot_*"), reverse=True)
            if not data_dirs:
                logger.error("No data directory found")
                return False
            
            latest_data = data_dirs[0]
            
            # Load config
            config_path = project_root / "config.yaml"
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            
            # Update paths
            resnet_model = latest_data / "resnet_lane_model" / "resnet_lane_final.pth"
            lstm_model = latest_data / "lstm_model" / "best_model.pth"
            
            if resnet_model.exists():
                config['perception']['resnet_model_path'] = str(resnet_model.relative_to(project_root))
            
            if lstm_model.exists():
                config['temporal']['trained_model_path'] = str(lstm_model.relative_to(project_root))
            
            # Save config
            with open(config_path, 'w') as f:
                yaml.dump(config, f, default_flow_style=False)
            
            logger.info("✅ Config updated")
            return True
        except Exception as e:
            logger.error(f"Config update failed: {e}")
            return False
    
    phases['update_config'] = phase_update_config
    
    # Phase 7: Run Inference
    def phase_run_inference() -> bool:
        """Phase 7: Run inference with trained models."""
        try:
            from scripts.entry_points.run_inference_refactored import main as run_inference
            
            # This will run the inference loop
            # Note: This might run indefinitely, so we might want to add timeout
            run_inference()
            return True
        except KeyboardInterrupt:
            logger.info("Inference interrupted by user")
            return True
        except Exception as e:
            logger.error(f"Inference failed: {e}")
            return False
    
    phases['run_inference'] = phase_run_inference
    
    return phases

