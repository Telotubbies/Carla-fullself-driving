"""
LSTM Training Loop.

Handles the training loop with advanced techniques.
"""

import logging
import torch
import torch.nn as nn
from torch.utils.data import DataLoader
from tqdm import tqdm
from typing import Dict, Any, Optional
from datetime import datetime

from training.advanced_training import (
    AttentionLSTM, CombinedLoss, AdvancedOptimizer,
    EarlyStopping, TrainingMonitor
)
from temporal import LSTMPredictor
from utils.device_utils import get_device
from utils.status_logger import StatusLogger

logger = logging.getLogger(__name__)


class LSTMTrainer:
    """Handles LSTM training with advanced techniques."""
    
    def __init__(
        self,
        model: nn.Module,
        device: torch.device,
        use_advanced_loss: bool = True,
        learning_rate: float = 0.0005,
        weight_decay: float = 1e-5,
        gradient_clip: float = 1.0,
        early_stopping_patience: int = 20
    ):
        """
        Initialize trainer.
        
        Args:
            model: LSTM model
            device: Training device
            use_advanced_loss: Use combined loss
            learning_rate: Learning rate
            weight_decay: Weight decay
            gradient_clip: Gradient clipping value
            early_stopping_patience: Early stopping patience
        """
        self.model = model
        self.device = device
        self.gradient_clip = gradient_clip
        
        # Loss function
        if use_advanced_loss:
            self.criterion = CombinedLoss(mse_weight=0.5, huber_weight=0.3, reg_weight=0.2)
        else:
            self.criterion = nn.MSELoss()
        
        # Optimizer
        self.optimizer = AdvancedOptimizer(
            model,
            learning_rate=learning_rate,
            weight_decay=weight_decay,
            use_adamw=True
        )
        
        # Early stopping
        self.early_stopping = EarlyStopping(
            patience=early_stopping_patience,
            min_delta=1e-6,
            mode='min'
        )
        
        # Monitor
        self.monitor = TrainingMonitor(log_interval=10)
        
        # Status logger
        self.status_logger = StatusLogger()
    
    def train_epoch(self, train_loader: DataLoader) -> float:
        """
        Train for one epoch.
        
        Args:
            train_loader: Training data loader
        
        Returns:
            Average training loss
        """
        self.model.train()
        train_loss = 0.0
        
        for sequences, targets in train_loader:
            sequences = sequences.to(self.device)
            targets = targets.to(self.device)
            
            self.optimizer.zero_grad()
            outputs = self.model(sequences)
            
            if isinstance(self.criterion, CombinedLoss):
                loss, loss_dict = self.criterion(outputs, targets, self.model.parameters())
            else:
                loss = self.criterion(outputs, targets)
            
            loss.backward()
            
            # Gradient clipping
            if self.gradient_clip > 0:
                self.optimizer.clip_gradients(self.gradient_clip)
            
            self.optimizer.step()
            
            train_loss += loss.item() if isinstance(loss, torch.Tensor) else loss
        
        return train_loss / len(train_loader)
    
    def validate(self, val_loader: DataLoader) -> float:
        """
        Validate model.
        
        Args:
            val_loader: Validation data loader
        
        Returns:
            Average validation loss
        """
        self.model.eval()
        val_loss = 0.0
        
        with torch.no_grad():
            for sequences, targets in val_loader:
                sequences = sequences.to(self.device)
                targets = targets.to(self.device)
                
                outputs = self.model(sequences)
                
                if isinstance(self.criterion, CombinedLoss):
                    loss, _ = self.criterion(outputs, targets, self.model.parameters())
                else:
                    loss = self.criterion(outputs, targets)
                
                val_loss += loss.item() if isinstance(loss, torch.Tensor) else loss
        
        return val_loss / len(val_loader)
    
    def train(
        self,
        train_loader: DataLoader,
        val_loader: DataLoader,
        epochs: int,
        output_dir: str
    ) -> Dict[str, Any]:
        """
        Train model.
        
        Args:
            train_loader: Training data loader
            val_loader: Validation data loader
            epochs: Number of epochs
            output_dir: Output directory
        
        Returns:
            Training history dictionary
        """
        # Setup optimizer scheduler
        self.optimizer.set_scheduler(
            'warmup_cosine',
            warmup_epochs=10,
            total_epochs=epochs
        )
        
        # Update status
        self.status_logger.update_training(
            'lstm',
            status='running',
            total_epochs=epochs,
            started_at=datetime.now().isoformat()
        )
        
        # Training history
        train_losses = []
        val_losses = []
        best_val_loss = float('inf')
        output_dir = Path(output_dir)
        output_dir.mkdir(parents=True, exist_ok=True)
        
        logger.info(f"Starting training for {epochs} epochs...")
        
        for epoch in range(epochs):
            # Train
            train_loss = self.train_epoch(train_loader)
            train_losses.append(train_loss)
            
            # Validate
            val_loss = self.validate(val_loader)
            val_losses.append(val_loss)
            
            # Update learning rate
            self.optimizer.step_scheduler(val_loss)
            
            # Early stopping check
            self.early_stopping(val_loss)
            
            # Log progress
            self.monitor.log_metrics(epoch, train_loss, val_loss)
            
            # Save best model
            if val_loss < best_val_loss:
                best_val_loss = val_loss
                best_model_path = output_dir / "best_model.pth"
                torch.save(self.model.state_dict(), best_model_path)
                logger.info(f"✅ Saved best model (val_loss={val_loss:.6f})")
            
            # Check early stopping
            if self.early_stopping.early_stop:
                logger.info(f"Early stopping at epoch {epoch+1}")
                break
        
        # Update status
        self.status_logger.update_training(
            'lstm',
            status='completed',
            total_epochs=epochs,
            completed_epochs=len(train_losses),
            best_val_loss=best_val_loss,
            completed_at=datetime.now().isoformat()
        )
        
        return {
            'train_losses': train_losses,
            'val_losses': val_losses,
            'best_val_loss': best_val_loss,
            'epochs_trained': len(train_losses)
        }


