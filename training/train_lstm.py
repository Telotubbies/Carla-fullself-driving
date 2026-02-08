"""
Train LSTM to predict future vehicle state from feature sequence.

STEP 3: feature sequence (10 frames) → next state
"""

import sys
import os
import yaml
import logging
import argparse
import numpy as np
import pandas as pd
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
from pathlib import Path
from tqdm import tqdm
import json

# Add project root to path
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

from temporal import LSTMPredictor
from utils.device_utils import get_device
from training.data_preprocessing import DataPreprocessor
from training.advanced_training import (
    AttentionLSTM, HuberLoss, WeightedMSELoss, CombinedLoss,
    AdvancedOptimizer, EarlyStopping, TrainingMonitor
)
from utils.status_logger import StatusLogger

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class LSTMSequenceDataset(Dataset):
    """Dataset for LSTM training."""
    
    def __init__(self, features: np.ndarray, states: np.ndarray, sequence_length: int = 10):
        """
        Initialize dataset.
        
        Args:
            features: Feature array (N, feature_dim)
            states: State array (N, 4) [x, y, yaw, velocity]
            sequence_length: Length of input sequence
        """
        self.sequence_length = sequence_length
        self.features = features
        self.states = states
        
        # Create sequences
        self.sequences = []
        self.targets = []
        
        for i in range(len(features) - sequence_length):
            seq = features[i:i+sequence_length]
            target = states[i+sequence_length]  # Predict next state
            self.sequences.append(seq)
            self.targets.append(target)
        
        logger.info(f"Created {len(self.sequences)} sequences")
    
    def __len__(self):
        return len(self.sequences)
    
    def __getitem__(self, idx):
        return (
            torch.FloatTensor(self.sequences[idx]),
            torch.FloatTensor(self.targets[idx])
        )


def load_training_data(data_dir: str, use_processed: bool = True):
    """
    Load features and states from data directory.
    
    Args:
        data_dir: Directory containing features.npy and data
        use_processed: Use processed data if available
        
    Returns:
        features: (N, feature_dim)
        states: (N, 4) [x, y, yaw, velocity] (normalized)
        state_mean: Normalization mean
        state_std: Normalization std
    """
    data_dir = Path(data_dir)
    
    # Load features
    features_path = data_dir / "features.npy"
    if not features_path.exists():
        raise FileNotFoundError(f"features.npy not found in {data_dir}")
    
    features = np.load(features_path)
    logger.info(f"Loaded features: {features.shape}")
    
    # Load states - prefer processed data
    if use_processed:
        csv_path = data_dir / "processed" / "data_processed.csv"
        stats_path = data_dir / "processed" / "statistics.json"
        
        if csv_path.exists() and stats_path.exists():
            logger.info("Using processed data")
            df = pd.read_csv(csv_path)
            import json
            with open(stats_path, 'r') as f:
                stats = json.load(f)
            state_mean = np.array(stats['state_mean'])
            state_std = np.array(stats['state_std'])
        else:
            logger.info("Processed data not found, using raw data")
            csv_path = data_dir / "data_valid.csv"
            if not csv_path.exists():
                csv_path = data_dir / "data.csv"
            df = pd.read_csv(csv_path)
            
            # Compute statistics
            preprocessor = DataPreprocessor()
            stats = preprocessor.compute_statistics(df)
            state_mean = np.array(stats['state_mean'])
            state_std = np.array(stats['state_std'])
    else:
        csv_path = data_dir / "data_valid.csv"
        if not csv_path.exists():
            csv_path = data_dir / "data.csv"
        df = pd.read_csv(csv_path)
        
        # Compute statistics
        preprocessor = DataPreprocessor()
        stats = preprocessor.compute_statistics(df)
        state_mean = np.array(stats['state_mean'])
        state_std = np.array(stats['state_std'])
    
    # Extract states: [x, y, yaw, velocity]
    states = df[['x', 'y', 'yaw', 'velocity']].values
    logger.info(f"Loaded states: {states.shape}")
    
    # Normalize states
    states_normalized = (states - state_mean) / state_std
    logger.info(f"State normalization:")
    logger.info(f"  Mean: {state_mean}")
    logger.info(f"  Std:  {state_std}")
    
    return features, states_normalized, state_mean, state_std


def train_lstm(
    data_dir: str,
    output_dir: str = None,
    sequence_length: int = 10,
    hidden_size: int = 256,
    num_layers: int = 2,
    batch_size: int = 64,
    epochs: int = 100,
    learning_rate: float = 0.001,
    train_split: float = 0.8,
    use_processed: bool = True,
    use_attention: bool = True,
    use_advanced_loss: bool = True,
    gradient_clip: float = 1.0,
    early_stopping_patience: int = 15
):
    """
    Train LSTM model.
    
    Args:
        data_dir: Directory containing training data
        output_dir: Output directory for model and stats
        sequence_length: Input sequence length
        hidden_size: LSTM hidden size
        num_layers: Number of LSTM layers
        batch_size: Batch size
        epochs: Number of epochs
        learning_rate: Learning rate
        train_split: Train/validation split ratio
    """
    data_dir = Path(data_dir)
    
    if output_dir is None:
        output_dir = data_dir / "lstm_model"
    else:
        output_dir = Path(output_dir)
    output_dir.mkdir(exist_ok=True)
    
    # Load data
    logger.info("Loading training data...")
    features, states, state_mean, state_std = load_training_data(data_dir, use_processed=use_processed)
    
    # Ensure same length
    min_len = min(len(features), len(states))
    features = features[:min_len]
    states = states[:min_len]
    
    # Validate data
    logger.info("Validating training data...")
    if len(features) < sequence_length + 1:
        raise ValueError(f"Insufficient data: need at least {sequence_length + 1} samples, got {len(features)}")
    
    # Check for NaN/Inf
    if np.any(np.isnan(features)) or np.any(np.isinf(features)):
        raise ValueError("Features contain NaN or Inf values")
    if np.any(np.isnan(states)) or np.any(np.isinf(states)):
        raise ValueError("States contain NaN or Inf values")
    
    logger.info("✅ Data validation passed")
    
    # Split train/val
    split_idx = int(len(features) * train_split)
    train_features = features[:split_idx]
    train_states = states[:split_idx]
    val_features = features[split_idx:]
    val_states = states[split_idx:]
    
    logger.info(f"Train: {len(train_features)}, Val: {len(val_features)}")
    
    # Create datasets
    train_dataset = LSTMSequenceDataset(train_features, train_states, sequence_length)
    val_dataset = LSTMSequenceDataset(val_features, val_states, sequence_length)
    
    train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True)
    val_loader = DataLoader(val_dataset, batch_size=batch_size, shuffle=False)
    
    # Initialize model
    # Try GPU first, fallback to CPU if ROCm has issues
    device = get_device()
    
    # Test GPU with a simple operation
    try:
        test_tensor = torch.zeros(1).to(device)
        _ = test_tensor + 1
        logger.info(f"✅ GPU device verified: {device}")
    except Exception as e:
        logger.warning(f"⚠️  GPU test failed: {e}, falling back to CPU")
        device = torch.device('cpu')
        logger.info("✅ Using CPU for training")
    
    # Use Attention LSTM if requested
    if use_attention:
        logger.info("Using Attention LSTM")
        model = AttentionLSTM(
            input_size=features.shape[1],
            hidden_size=hidden_size,
            num_layers=num_layers,
            output_size=4
        ).to(device)
    else:
        model = LSTMPredictor(
            input_size=features.shape[1],
            hidden_size=hidden_size,
            num_layers=num_layers,
            sequence_length=sequence_length,
            output_size=4
        ).to(device)
    
    # Advanced training setup
    if use_advanced_loss:
        logger.info("Using Combined Loss (MSE + Huber + Regularization)")
        criterion = CombinedLoss(mse_weight=0.5, huber_weight=0.3, reg_weight=0.2)
    else:
        criterion = nn.MSELoss()
    
    # Advanced optimizer with scheduling
    adv_optimizer = AdvancedOptimizer(
        model,
        learning_rate=learning_rate,
        weight_decay=1e-5,
        use_adamw=True
    )
    adv_optimizer.set_scheduler(
        'warmup_cosine',
        warmup_epochs=10,
        total_epochs=epochs
    )
    
    # Early stopping
    early_stopping = EarlyStopping(
        patience=early_stopping_patience,
        min_delta=1e-6,
        mode='min'
    )
    
    # Training monitor
    monitor = TrainingMonitor(log_interval=10)
    
    # Status logger
    status_logger = StatusLogger()
    status_logger.update_training(
        'lstm',
        status='running',
        total_epochs=epochs,
        started_at=datetime.now().isoformat()
    )
    
    # Training loop
    best_val_loss = float('inf')
    train_losses = []
    val_losses = []
    
    logger.info(f"Starting training for {epochs} epochs with advanced techniques...")
    logger.info(f"  - Attention: {use_attention}")
    logger.info(f"  - Advanced Loss: {use_advanced_loss}")
    logger.info(f"  - Gradient Clipping: {gradient_clip}")
    logger.info(f"  - Early Stopping: {early_stopping_patience} patience")
    
    for epoch in range(epochs):
        # Train
        model.train()
        train_loss = 0.0
        for sequences, targets in tqdm(train_loader, desc=f"Epoch {epoch+1}/{epochs}"):
            sequences = sequences.to(device)
            targets = targets.to(device)
            
            adv_optimizer.zero_grad()
            outputs = model(sequences)
            
            if use_advanced_loss:
                loss, loss_dict = criterion(outputs, targets, model.parameters())
            else:
                loss = criterion(outputs, targets)
                loss_dict = {}
            
            loss.backward()
            
            # Gradient clipping
            if gradient_clip > 0:
                adv_optimizer.clip_gradients(gradient_clip)
            
            adv_optimizer.step()
            
            train_loss += loss.item() if isinstance(loss, torch.Tensor) else loss
        
        avg_train_loss = train_loss / len(train_loader)
        train_losses.append(avg_train_loss)
        
        # Validate
        model.eval()
        val_loss = 0.0
        with torch.no_grad():
            for sequences, targets in val_loader:
                sequences = sequences.to(device)
                targets = targets.to(device)
                
                outputs = model(sequences)
                if use_advanced_loss:
                    loss, _ = criterion(outputs, targets, model.parameters())
                else:
                    loss = criterion(outputs, targets)
                val_loss += loss.item() if isinstance(loss, torch.Tensor) else loss
        
        avg_val_loss = val_loss / len(val_loader)
        val_losses.append(avg_val_loss)
        
        # Get current learning rate
        current_lr = adv_optimizer.optimizer.param_groups[0]['lr']
        
        # Calculate gradient norm
        grad_norm = 0.0
        if gradient_clip > 0:
            for param in model.parameters():
                if param.grad is not None:
                    grad_norm += param.grad.data.norm(2).item() ** 2
            grad_norm = grad_norm ** 0.5
        
        # Learning rate scheduling
        adv_optimizer.scheduler_step(avg_val_loss)
        
        # Monitor training
        monitor.log(epoch+1, avg_train_loss, avg_val_loss, current_lr, grad_norm)
        
        # Update status logger
        status_logger.update_training(
            'lstm',
            current_epoch=epoch+1,
            train_loss=avg_train_loss,
            val_loss=avg_val_loss,
            learning_rate=current_lr,
            best_val_loss=best_val_loss
        )
        
        # Early stopping
        if early_stopping(avg_val_loss):
            logger.info(f"Early stopping at epoch {epoch+1}")
            logger.info(f"Best validation loss: {best_val_loss:.6f} at epoch {monitor.get_best_epoch()+1}")
            status_logger.update_training('lstm', status='stopped_early', stopped_at_epoch=epoch+1)
            break
        
        # Save best model
        if avg_val_loss < best_val_loss:
            best_val_loss = avg_val_loss
            model_path = output_dir / "best_model.pth"
            torch.save({
                'epoch': epoch,
                'model_state_dict': model.state_dict(),
                'optimizer_state_dict': adv_optimizer.optimizer.state_dict(),
                'train_loss': avg_train_loss,
                'val_loss': avg_val_loss,
                'state_mean': state_mean.tolist() if state_mean is not None else None,
                'state_std': state_std.tolist() if state_std is not None else None,
                'config': {
                    'input_size': features.shape[1],
                    'hidden_size': hidden_size,
                    'num_layers': num_layers,
                    'sequence_length': sequence_length,
                    'output_size': 4
                }
            }, model_path)
            logger.info(f"✅ Saved best model (val_loss={avg_val_loss:.6f})")
    
    # Save training history
    history = {
        'train_losses': train_losses,
        'val_losses': val_losses,
        'learning_rates': monitor.metrics.get('learning_rate', []),
        'grad_norms': monitor.metrics.get('grad_norm', []),
        'best_epoch': monitor.get_best_epoch() + 1,
        'best_val_loss': best_val_loss,
        'total_epochs': len(train_losses),
        'config': {
            'epochs': epochs,
            'batch_size': batch_size,
            'learning_rate': learning_rate,
            'hidden_size': hidden_size,
            'num_layers': num_layers,
            'sequence_length': sequence_length,
            'use_attention': use_attention,
            'use_advanced_loss': use_advanced_loss
        }
    }
    history_path = output_dir / "training_history.json"
    with open(history_path, 'w') as f:
        json.dump(history, f, indent=2)
    
    # Update status logger
    status_logger.update_training(
        'lstm',
        status='completed',
        completed_at=datetime.now().isoformat(),
        best_val_loss=best_val_loss,
        final_train_loss=train_losses[-1] if train_losses else None,
        final_val_loss=val_losses[-1] if val_losses else None
    )
    
    # Update model status
    model_path = output_dir / "best_model.pth"
    if model_path.exists():
        size_mb = model_path.stat().st_size / (1024 * 1024)
        status_logger.update_model(
            'lstm',
            exists=True,
            size=f"{size_mb:.1f}M",
            trained_at=datetime.now().isoformat(),
            val_loss=best_val_loss,
            path=str(model_path)
        )
    
    logger.info(f"✅ Training complete!")
    logger.info(f"   Best validation loss: {best_val_loss:.6f}")
    logger.info(f"   Model saved to: {output_dir}")
    logger.info(f"   Training history saved to: {history_path}")


def main():
    parser = argparse.ArgumentParser(description='Train LSTM model')
    parser.add_argument('data_dir', type=str, help='Directory containing features.npy and data.csv')
    parser.add_argument('--output', type=str, default=None, help='Output directory')
    parser.add_argument('--sequence-length', type=int, default=10, help='Sequence length')
    parser.add_argument('--hidden-size', type=int, default=256, help='LSTM hidden size')
    parser.add_argument('--num-layers', type=int, default=2, help='Number of LSTM layers')
    parser.add_argument('--batch-size', type=int, default=64, help='Batch size')
    parser.add_argument('--epochs', type=int, default=100, help='Number of epochs')
    parser.add_argument('--use-attention', action='store_true', help='Use Attention LSTM')
    parser.add_argument('--use-advanced-loss', action='store_true', help='Use advanced loss function')
    parser.add_argument('--gradient-clip', type=float, default=1.0, help='Gradient clipping norm')
    parser.add_argument('--early-stopping', type=int, default=15, help='Early stopping patience')
    parser.add_argument('--lr', type=float, default=0.001, help='Learning rate')
    parser.add_argument('--train-split', type=float, default=0.8, help='Train/val split')
    parser.add_argument('--no-preprocess', action='store_true', help='Skip using processed data')
    
    args = parser.parse_args()
    
    train_lstm(
        args.data_dir,
        args.output,
        args.sequence_length,
        args.hidden_size,
        args.num_layers,
        args.batch_size,
        args.epochs,
        args.lr,
        args.train_split,
        use_processed=not args.no_preprocess,
        use_attention=args.use_attention,
        use_advanced_loss=args.use_advanced_loss,
        gradient_clip=args.gradient_clip,
        early_stopping_patience=args.early_stopping
    )
    
    return 0


if __name__ == '__main__':
    sys.exit(main())

