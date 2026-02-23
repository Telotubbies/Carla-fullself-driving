"""
Centralized Status Logger for the entire pipeline.

Provides a single file to check all system status.
"""

import json
import logging
from pathlib import Path
from datetime import datetime
from typing import Dict, Any, Optional
import yaml
import torch
import numpy as np

logger = logging.getLogger(__name__)


class StatusLogger:
    """Centralized status logger for the entire pipeline."""
    
    def __init__(self, status_file: str = "logs/system_status.json"):
        """
        Initialize status logger.
        
        Args:
            status_file: Path to status JSON file
        """
        self.status_file = Path(status_file)
        self.status_file.parent.mkdir(parents=True, exist_ok=True)
        self.status = self._load_status()
    
    def _load_status(self) -> Dict[str, Any]:
        """Load existing status or create new."""
        if self.status_file.exists():
            try:
                with open(self.status_file, 'r') as f:
                    return json.load(f)
            except Exception as e:
                logger.warning(f"Failed to load status: {e}")
        
        return {
            'last_updated': None,
            'data_pipeline': {},
            'models': {},
            'training': {},
            'system': {},
            'metrics': {}
        }
    
    def _save_status(self):
        """Save status to file."""
        self.status['last_updated'] = datetime.now().isoformat()
        try:
            with open(self.status_file, 'w') as f:
                json.dump(self.status, f, indent=2)
        except Exception as e:
            logger.error(f"Failed to save status: {e}")
    
    def update_data_pipeline(self, **kwargs):
        """Update data pipeline status."""
        self.status['data_pipeline'].update({
            **kwargs,
            'updated_at': datetime.now().isoformat()
        })
        self._save_status()
    
    def update_model(self, model_name: str, **kwargs):
        """Update model status."""
        if 'models' not in self.status:
            self.status['models'] = {}
        
        if model_name not in self.status['models']:
            self.status['models'][model_name] = {}
        
        self.status['models'][model_name].update({
            **kwargs,
            'updated_at': datetime.now().isoformat()
        })
        self._save_status()
    
    def update_training(self, training_name: str, **kwargs):
        """Update training status."""
        if 'training' not in self.status:
            self.status['training'] = {}
        
        if training_name not in self.status['training']:
            self.status['training'][training_name] = {
                'started_at': datetime.now().isoformat(),
                'status': 'running'
            }
        
        self.status['training'][training_name].update({
            **kwargs,
            'updated_at': datetime.now().isoformat()
        })
        self._save_status()
    
    def update_metrics(self, **kwargs):
        """Update metrics."""
        self.status['metrics'].update(kwargs)
        self.status['metrics']['updated_at'] = datetime.now().isoformat()
        self._save_status()
    
    def update_system(self, **kwargs):
        """Update system status."""
        self.status['system'].update(kwargs)
        self.status['system']['updated_at'] = datetime.now().isoformat()
        self._save_status()
    
    def get_status(self) -> Dict[str, Any]:
        """Get current status."""
        return self.status
    
    def get_summary(self) -> str:
        """Get human-readable summary."""
        lines = []
        lines.append("=" * 60)
        lines.append("SYSTEM STATUS SUMMARY")
        lines.append("=" * 60)
        lines.append(f"Last Updated: {self.status.get('last_updated', 'Never')}")
        lines.append("")
        
        # Data Pipeline
        lines.append("📊 DATA PIPELINE:")
        dp = self.status.get('data_pipeline', {})
        lines.append(f"   Images: {dp.get('image_count', 'N/A')}")
        lines.append(f"   Lane Masks: {dp.get('mask_count', 'N/A')}")
        lines.append(f"   Features: {dp.get('feature_size', 'N/A')}")
        lines.append("")
        
        # Models
        lines.append("🤖 MODELS:")
        models = self.status.get('models', {})
        for name, info in models.items():
            status_icon = "✅" if info.get('exists', False) else "❌"
            lines.append(f"   {status_icon} {name}:")
            if info.get('exists'):
                lines.append(f"      Size: {info.get('size', 'N/A')}")
                lines.append(f"      Trained: {info.get('trained_at', 'N/A')}")
                if 'val_loss' in info:
                    lines.append(f"      Val Loss: {info.get('val_loss', 'N/A')}")
        lines.append("")
        
        # Training
        lines.append("⏳ TRAINING:")
        training = self.status.get('training', {})
        for name, info in training.items():
            status = info.get('status', 'unknown')
            status_icon = "⏳" if status == 'running' else "✅" if status == 'completed' else "❌"
            lines.append(f"   {status_icon} {name}: {status}")
            if 'current_epoch' in info:
                lines.append(f"      Epoch: {info.get('current_epoch')}/{info.get('total_epochs', 'N/A')}")
            if 'val_loss' in info:
                lines.append(f"      Val Loss: {info.get('val_loss', 'N/A')}")
        lines.append("")
        
        # Metrics
        lines.append("📈 METRICS:")
        metrics = self.status.get('metrics', {})
        for key, value in metrics.items():
            if key != 'updated_at':
                lines.append(f"   {key}: {value}")
        lines.append("")
        
        lines.append("=" * 60)
        return "\n".join(lines)


def scan_system_status(status_logger: StatusLogger):
    """Scan and update system status."""
    project_root = Path(__file__).parent.parent
    
    # Scan data pipeline
    latest_data = None
    data_dirs = sorted(project_root.glob("data/autopilot_*"), reverse=True)
    if data_dirs:
        latest_data = data_dirs[0]
        
        images_dir = latest_data / "images"
        masks_dir = latest_data / "lane_masks"
        features_file = latest_data / "features.npy"
        
        image_count = len(list(images_dir.glob("*.png"))) if images_dir.exists() else 0
        mask_count = len(list(masks_dir.glob("*_lane.png"))) if masks_dir.exists() else 0
        feature_size = None
        if features_file.exists():
            try:
                size_mb = features_file.stat().st_size / (1024 * 1024)
                feature_size = f"{size_mb:.1f}M"
            except:
                pass
        
        status_logger.update_data_pipeline(
            image_count=image_count,
            mask_count=mask_count,
            feature_size=feature_size,
            data_dir=str(latest_data)
        )
    
    # Scan models
    if latest_data:
        # LSTM Model
        lstm_model = latest_data / "lstm_model" / "best_model.pth"
        if lstm_model.exists():
            try:
                size_mb = lstm_model.stat().st_size / (1024 * 1024)
                trained_at = datetime.fromtimestamp(lstm_model.stat().st_mtime).isoformat()
                
                # Try to load training history
                history_file = latest_data / "lstm_model" / "training_history.json"
                val_loss = None
                if history_file.exists():
                    try:
                        with open(history_file, 'r') as f:
                            history = json.load(f)
                            val_losses = history.get('val_losses', [])
                            if val_losses:
                                val_loss = min(val_losses)
                    except:
                        pass
                
                status_logger.update_model(
                    'lstm',
                    exists=True,
                    size=f"{size_mb:.1f}M",
                    trained_at=trained_at,
                    val_loss=val_loss,
                    path=str(lstm_model)
                )
            except Exception as e:
                logger.warning(f"Failed to scan LSTM model: {e}")
        
        # ResNet Model
        resnet_model = latest_data / "resnet_lane_model" / "resnet_lane_final.pth"
        if resnet_model.exists():
            try:
                size_mb = resnet_model.stat().st_size / (1024 * 1024)
                trained_at = datetime.fromtimestamp(resnet_model.stat().st_mtime).isoformat()
                
                # Try to get val loss from log
                val_loss = None
                log_files = sorted(project_root.glob("logs/finetune_resnet_*.log"), reverse=True)
                if log_files:
                    try:
                        with open(log_files[0], 'r') as f:
                            content = f.read()
                            # Find best val loss
                            import re
                            matches = re.findall(r'val_loss=([0-9.]+)', content)
                            if matches:
                                val_loss = min([float(m) for m in matches])
                    except:
                        pass
                
                status_logger.update_model(
                    'resnet_lane',
                    exists=True,
                    size=f"{size_mb:.1f}M",
                    trained_at=trained_at,
                    val_loss=val_loss,
                    path=str(resnet_model)
                )
            except Exception as e:
                logger.warning(f"Failed to scan ResNet model: {e}")
    
    # Scan training processes
    import subprocess
    try:
        # Check ResNet training
        result = subprocess.run(['pgrep', '-f', 'finetune_resnet'], 
                              capture_output=True, text=True)
        if result.returncode == 0:
            # Try to get progress from log
            log_files = sorted(project_root.glob("logs/finetune_resnet_300ep_*.log"), reverse=True)
            if log_files:
                try:
                    with open(log_files[0], 'r') as f:
                        content = f.read()
                        # Find current epoch
                        import re
                        epoch_matches = re.findall(r'Epoch (\d+)/300', content)
                        if epoch_matches:
                            current_epoch = int(epoch_matches[-1])
                            val_loss_matches = re.findall(r'Val Loss: ([0-9.]+)', content)
                            val_loss = float(val_loss_matches[-1]) if val_loss_matches else None
                            
                            status_logger.update_training(
                                'resnet_300ep',
                                status='running',
                                current_epoch=current_epoch,
                                total_epochs=300,
                                val_loss=val_loss
                            )
                except:
                    status_logger.update_training('resnet_300ep', status='running')
    except:
        pass
    
    # Scan config
    config_file = project_root / "config.yaml"
    if config_file.exists():
        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
                status_logger.update_system(
                    config_loaded=True,
                    lstm_path=config.get('temporal', {}).get('trained_model_path', 'Not set'),
                    resnet_path=config.get('perception', {}).get('resnet_model_path', 'Not set')
                )
        except:
            pass


if __name__ == '__main__':
    # CLI interface
    import argparse
    parser = argparse.ArgumentParser(description='Status Logger')
    parser.add_argument('--scan', action='store_true', help='Scan and update status')
    parser.add_argument('--summary', action='store_true', help='Show summary')
    parser.add_argument('--file', type=str, default='logs/system_status.json', help='Status file path')
    
    args = parser.parse_args()
    
    status_logger = StatusLogger(args.file)
    
    if args.scan:
        scan_system_status(status_logger)
        print("✅ Status updated")
    
    if args.summary:
        print(status_logger.get_summary())

