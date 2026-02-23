"""
Fine-tune LSTM model from existing checkpoint.

Loads existing model and continues training with lower learning rate.
"""

import sys
import argparse
import logging
import torch
import numpy as np
from pathlib import Path
from tqdm import tqdm

# Add project root to path
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

from training.train_lstm import (
    load_training_data, LSTMSequenceDataset, train_lstm
)
from training.load_trained_model import load_trained_lstm
from training.advanced_training import (
    AttentionLSTM, CombinedLoss, AdvancedOptimizer, 
    EarlyStopping, TrainingMonitor
)
from temporal import LSTMPredictor
from utils.device_utils import get_device
from torch.utils.data import DataLoader
from datetime import datetime

# Setup logging with file handler
def setup_logging(log_dir: Path = None):
    """Setup logging to both console and file."""
    if log_dir is None:
        log_dir = project_root / "logs" / "training"
    log_dir.mkdir(parents=True, exist_ok=True)
    
    # Create log file with timestamp
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_file = log_dir / f"finetune_lstm_{timestamp}.log"
    
    # Configure logging
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        datefmt='%Y-%m-%d %H:%M:%S'
    )
    
    # Console handler
    console_handler = logging.StreamHandler()
    console_handler.setLevel(logging.INFO)
    console_handler.setFormatter(formatter)
    
    # File handler
    file_handler = logging.FileHandler(log_file, encoding='utf-8')
    file_handler.setLevel(logging.INFO)
    file_handler.setFormatter(formatter)
    
    # Setup root logger
    root_logger = logging.getLogger()
    root_logger.setLevel(logging.INFO)
    root_logger.handlers.clear()  # Clear existing handlers
    root_logger.addHandler(console_handler)
    root_logger.addHandler(file_handler)
    
    return log_file

# Setup logging
log_file = setup_logging()
logger = logging.getLogger(__name__)
logger.info(f"Logging to file: {log_file}")


def finetune_lstm(
    data_dir: str,
    checkpoint_path: str,
    output_dir: str = None,
    epochs: int = 50,
    batch_size: int = 64,
    learning_rate: float = 0.0001,  # Lower LR for fine-tuning
    train_split: float = 0.8,
    gradient_clip: float = 1.0,
    early_stopping_patience: int = 15
):
    """
    Fine-tune LSTM model from checkpoint.
    
    Args:
        data_dir: Directory containing training data
        checkpoint_path: Path to existing model checkpoint
        output_dir: Output directory for fine-tuned model
        epochs: Number of additional epochs
        batch_size: Batch size
        learning_rate: Learning rate (lower than initial training)
        train_split: Train/validation split ratio
        gradient_clip: Gradient clipping value
        early_stopping_patience: Early stopping patience
    """
    data_dir = Path(data_dir)
    checkpoint_path = Path(checkpoint_path)
    
    if not checkpoint_path.exists():
        raise FileNotFoundError(f"Checkpoint not found: {checkpoint_path}")
    
    if output_dir is None:
        output_dir = data_dir / "lstm_model"
    else:
        output_dir = Path(output_dir)
    output_dir.mkdir(exist_ok=True)
    
    # Load existing model
    logger.info(f"Loading checkpoint: {checkpoint_path}")
    model, state_mean, state_std = load_trained_lstm(str(checkpoint_path))
    device = get_device()
    model = model.to(device)
    model.train()  # Set to training mode
    
    logger.info("✅ Model loaded successfully")
    
    # Load training data
    logger.info("Loading training data...")
    features, states, _, _ = load_training_data(data_dir, use_processed=True)
    
    # Ensure same length
    min_len = min(len(features), len(states))
    features = features[:min_len]
    states = states[:min_len]
    
    # Validate data
    sequence_length = 10  # Default, can be extracted from model if needed
    if len(features) < sequence_length + 1:
        raise ValueError(f"Insufficient data: need at least {sequence_length + 1} samples")
    
    logger.info(f"✅ Data loaded: {len(features)} samples")
    
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
    
    # Loss function
    criterion = CombinedLoss(mse_weight=0.5, huber_weight=0.3, reg_weight=0.2)
    
    # Optimizer with lower learning rate for fine-tuning
    adv_optimizer = AdvancedOptimizer(
        model,
        learning_rate=learning_rate,
        weight_decay=1e-5,
        use_adamw=True
    )
    adv_optimizer.set_scheduler(
        'warmup_cosine',
        warmup_epochs=5,  # Shorter warmup for fine-tuning
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
    
    # Training loop
    best_val_loss = float('inf')
    best_model_state = None
    
    logger.info(f"Starting fine-tuning for {epochs} epochs...")
    logger.info(f"Learning rate: {learning_rate} (fine-tuning rate)")
    
    for epoch in range(epochs):
        # Training phase
        model.train()
        train_loss = 0.0
        train_count = 0
        total_grad_norm = 0.0
        
        pbar = tqdm(train_loader, desc=f"Epoch {epoch+1}/{epochs} [Train]")
        for batch_features, batch_states in pbar:
            batch_features = batch_features.to(device)
            batch_states = batch_states.to(device)
            
            # Forward pass
            adv_optimizer.zero_grad()
            outputs = model(batch_features)
            loss_result = criterion(outputs, batch_states, model.parameters())
            
            # Handle tuple return from CombinedLoss
            if isinstance(loss_result, tuple):
                loss, loss_dict = loss_result
            else:
                loss = loss_result
                loss_dict = {}
            
            # Backward pass
            loss.backward()
            
            # Calculate gradient norm before clipping
            grad_norm = torch.nn.utils.clip_grad_norm_(model.parameters(), gradient_clip)
            total_grad_norm += grad_norm.item()
            
            adv_optimizer.step()
            
            train_loss += loss.item()
            train_count += 1
            pbar.set_postfix({'loss': f'{loss.item():.4f}'})
        
        avg_train_loss = train_loss / train_count
        avg_grad_norm = total_grad_norm / train_count
        
        # Validation phase
        model.eval()
        val_loss = 0.0
        val_count = 0
        
        with torch.no_grad():
            for batch_features, batch_states in val_loader:
                batch_features = batch_features.to(device)
                batch_states = batch_states.to(device)
                
                outputs = model(batch_features)
                loss_result = criterion(outputs, batch_states, model.parameters())
                
                # Handle tuple return from CombinedLoss
                if isinstance(loss_result, tuple):
                    loss, _ = loss_result
                else:
                    loss = loss_result
                
                val_loss += loss.item()
                val_count += 1
        
        avg_val_loss = val_loss / val_count
        
        # Update learning rate
        adv_optimizer.scheduler_step()
        current_lr = adv_optimizer.optimizer.param_groups[0]['lr']
        
        # Logging
        logger.info(
            f"Epoch {epoch+1}/{epochs} - "
            f"Train Loss: {avg_train_loss:.6f}, "
            f"Val Loss: {avg_val_loss:.6f}, "
            f"LR: {current_lr:.6f}"
        )
        
        # Monitor (use log method like train_lstm.py)
        monitor.log(epoch+1, avg_train_loss, avg_val_loss, current_lr, avg_grad_norm)
        
        # Save best model
        if avg_val_loss < best_val_loss:
            best_val_loss = avg_val_loss
            best_model_state = model.state_dict().copy()
            logger.info(f"✅ New best validation loss: {best_val_loss:.6f}")
        
        # Early stopping
        should_stop = early_stopping(avg_val_loss)
        if should_stop:
            logger.info(f"Early stopping triggered at epoch {epoch+1}")
            break
    
    # Save fine-tuned model
    if best_model_state is not None:
        model.load_state_dict(best_model_state)
    
    # Save checkpoint
    checkpoint = {
        'model_state_dict': model.state_dict(),
        'state_mean': state_mean,
        'state_std': state_std,
        'config': {
            'input_size': features.shape[1],
            'hidden_size': 256,  # Default, should match model
            'num_layers': 2,  # Default, should match model
            'sequence_length': sequence_length,
            'output_size': 4
        },
        'best_val_loss': best_val_loss,
        'epochs_trained': epochs,
        'fine_tuned_from': str(checkpoint_path)
    }
    
    # Save as best_model.pth
    best_model_path = output_dir / "best_model.pth"
    torch.save(checkpoint, best_model_path)
    logger.info(f"✅ Saved fine-tuned model: {best_model_path}")
    
    # Also save as fine_tuned_model.pth
    fine_tuned_path = output_dir / "fine_tuned_model.pth"
    torch.save(checkpoint, fine_tuned_path)
    logger.info(f"✅ Saved as: {fine_tuned_path}")
    
    logger.info(f"✅ Fine-tuning complete!")
    logger.info(f"   Best validation loss: {best_val_loss:.6f}")
    logger.info(f"   Fine-tuned from: {checkpoint_path}")
    
    return best_model_path


def main():
    parser = argparse.ArgumentParser(description='Fine-tune LSTM model from checkpoint')
    parser.add_argument('data_dir', type=str, help='Directory containing training data')
    parser.add_argument('--checkpoint', type=str, required=True,
                        help='Path to existing model checkpoint')
    parser.add_argument('--output', type=str, default=None,
                        help='Output directory (default: data_dir/lstm_model)')
    parser.add_argument('--epochs', type=int, default=50,
                        help='Number of epochs (default: 50)')
    parser.add_argument('--batch-size', type=int, default=64,
                        help='Batch size (default: 64)')
    parser.add_argument('--lr', type=float, default=0.0001,
                        help='Learning rate for fine-tuning (default: 0.0001)')
    parser.add_argument('--train-split', type=float, default=0.8,
                        help='Train/validation split (default: 0.8)')
    parser.add_argument('--gradient-clip', type=float, default=1.0,
                        help='Gradient clipping value (default: 1.0)')
    parser.add_argument('--early-stopping', type=int, default=15,
                        help='Early stopping patience (default: 15)')
    
    args = parser.parse_args()
    
    finetune_lstm(
        data_dir=args.data_dir,
        checkpoint_path=args.checkpoint,
        output_dir=args.output,
        epochs=args.epochs,
        batch_size=args.batch_size,
        learning_rate=args.lr,
        train_split=args.train_split,
        gradient_clip=args.gradient_clip,
        early_stopping_patience=args.early_stopping
    )


if __name__ == '__main__':
    main()

