"""
Advanced training techniques and algorithms for LSTM.

Includes:
- Attention mechanisms
- Advanced loss functions (Huber, Smooth L1)
- Gradient clipping
- Learning rate scheduling
- Early stopping
- Mixed precision training
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
from typing import Optional, Dict, Any
import logging

logger = logging.getLogger(__name__)


class AttentionLSTM(nn.Module):
    """
    LSTM with attention mechanism for better temporal modeling.
    
    Uses attention to focus on important frames in the sequence.
    """
    
    def __init__(self, input_size: int, hidden_size: int, num_layers: int, output_size: int = 4):
        super().__init__()
        self.hidden_size = hidden_size
        self.num_layers = num_layers
        
        # LSTM layers
        self.lstm = nn.LSTM(input_size, hidden_size, num_layers, batch_first=True, dropout=0.1)
        
        # Attention mechanism
        self.attention = nn.MultiheadAttention(hidden_size, num_heads=4, batch_first=True)
        
        # Output layers
        self.fc1 = nn.Linear(hidden_size, hidden_size // 2)
        self.fc2 = nn.Linear(hidden_size // 2, output_size)
        self.dropout = nn.Dropout(0.2)
        
    def forward(self, x):
        # x: (batch, seq_len, input_size)
        lstm_out, (h_n, c_n) = self.lstm(x)
        
        # Apply attention
        attn_out, attn_weights = self.attention(lstm_out, lstm_out, lstm_out)
        
        # Use last output
        last_out = attn_out[:, -1, :]
        
        # Fully connected layers
        out = F.relu(self.fc1(last_out))
        out = self.dropout(out)
        out = self.fc2(out)
        
        return out


class HuberLoss(nn.Module):
    """
    Huber Loss (smooth L1) - less sensitive to outliers than MSE.
    
    Good for autonomous driving where some predictions may be noisy.
    """
    
    def __init__(self, delta: float = 1.0):
        super().__init__()
        self.delta = delta
    
    def forward(self, pred, target):
        error = pred - target
        is_small = torch.abs(error) < self.delta
        squared_loss = 0.5 * error ** 2
        linear_loss = self.delta * (torch.abs(error) - 0.5 * self.delta)
        return torch.where(is_small, squared_loss, linear_loss).mean()


class WeightedMSELoss(nn.Module):
    """
    Weighted MSE Loss - different weights for different state components.
    
    Can prioritize position accuracy over velocity, etc.
    """
    
    def __init__(self, weights: Optional[torch.Tensor] = None):
        super().__init__()
        if weights is None:
            # Default: equal weights
            weights = torch.ones(4)
        self.register_buffer('weights', weights)
    
    def forward(self, pred, target):
        error = pred - target
        weighted_error = error ** 2 * self.weights
        return weighted_error.mean()


class CombinedLoss(nn.Module):
    """
    Combined loss function: MSE + Huber + Regularization.
    
    Balances different aspects of prediction accuracy.
    """
    
    def __init__(self, mse_weight: float = 0.5, huber_weight: float = 0.3, reg_weight: float = 0.2):
        super().__init__()
        self.mse_weight = mse_weight
        self.huber_weight = huber_weight
        self.reg_weight = reg_weight
        self.mse = nn.MSELoss()
        self.huber = HuberLoss(delta=1.0)
    
    def forward(self, pred, target, model_params=None):
        mse_loss = self.mse(pred, target)
        huber_loss = self.huber(pred, target)
        
        # L2 regularization
        reg_loss = 0.0
        if model_params is not None:
            for param in model_params:
                reg_loss += torch.norm(param) ** 2
        
        total_loss = (self.mse_weight * mse_loss + 
                     self.huber_weight * huber_loss + 
                     self.reg_weight * reg_loss)
        
        return total_loss, {
            'mse': mse_loss.item(),
            'huber': huber_loss.item(),
            'reg': reg_loss.item() if isinstance(reg_loss, torch.Tensor) else reg_loss
        }


class AdvancedOptimizer:
    """
    Advanced optimizer with learning rate scheduling and gradient clipping.
    """
    
    def __init__(
        self,
        model: nn.Module,
        learning_rate: float = 0.001,
        weight_decay: float = 1e-5,
        use_adamw: bool = True
    ):
        if use_adamw:
            self.optimizer = torch.optim.AdamW(
                model.parameters(),
                lr=learning_rate,
                weight_decay=weight_decay,
                betas=(0.9, 0.999),
                eps=1e-8
            )
        else:
            self.optimizer = torch.optim.Adam(
                model.parameters(),
                lr=learning_rate,
                weight_decay=weight_decay
            )
        
        self.scheduler = None
    
    def set_scheduler(self, scheduler_type: str = 'cosine', **kwargs):
        """Set learning rate scheduler."""
        if scheduler_type == 'cosine':
            self.scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
                self.optimizer,
                T_max=kwargs.get('T_max', 100),
                eta_min=kwargs.get('eta_min', 1e-6)
            )
        elif scheduler_type == 'plateau':
            self.scheduler = torch.optim.lr_scheduler.ReduceLROnPlateau(
                self.optimizer,
                mode='min',
                factor=kwargs.get('factor', 0.5),
                patience=kwargs.get('patience', 5),
                verbose=True
            )
        elif scheduler_type == 'warmup_cosine':
            # Warmup + Cosine annealing
            from torch.optim.lr_scheduler import LambdaLR
            warmup_epochs = kwargs.get('warmup_epochs', 10)
            total_epochs = kwargs.get('total_epochs', 100)
            
            def lr_lambda(epoch):
                if epoch < warmup_epochs:
                    return epoch / warmup_epochs
                else:
                    return 0.5 * (1 + np.cos(np.pi * (epoch - warmup_epochs) / (total_epochs - warmup_epochs)))
            
            self.scheduler = LambdaLR(self.optimizer, lr_lambda)
    
    def step(self, closure=None):
        """Optimizer step."""
        return self.optimizer.step(closure)
    
    def zero_grad(self):
        """Zero gradients."""
        return self.optimizer.zero_grad()
    
    def clip_gradients(self, max_norm: float = 1.0):
        """Clip gradients to prevent explosion."""
        torch.nn.utils.clip_grad_norm_(self.optimizer.param_groups[0]['params'], max_norm)
    
    def scheduler_step(self, metric=None):
        """Step scheduler."""
        if self.scheduler is None:
            return
        
        if isinstance(self.scheduler, torch.optim.lr_scheduler.ReduceLROnPlateau):
            if metric is not None:
                self.scheduler.step(metric)
        else:
            self.scheduler.step()


class EarlyStopping:
    """
    Early stopping to prevent overfitting.
    """
    
    def __init__(self, patience: int = 10, min_delta: float = 0.0, mode: str = 'min'):
        self.patience = patience
        self.min_delta = min_delta
        self.mode = mode
        self.counter = 0
        self.best_score = None
        self.early_stop = False
        
    def __call__(self, score: float) -> bool:
        """
        Check if should stop early.
        
        Returns:
            True if should stop
        """
        if self.best_score is None:
            self.best_score = score
        elif self._is_better(score, self.best_score):
            self.best_score = score
            self.counter = 0
        else:
            self.counter += 1
            if self.counter >= self.patience:
                self.early_stop = True
        
        return self.early_stop
    
    def _is_better(self, current: float, best: float) -> bool:
        """Check if current score is better."""
        if self.mode == 'min':
            return current < best - self.min_delta
        else:
            return current > best + self.min_delta


class TrainingMonitor:
    """
    Monitor training progress and log metrics.
    """
    
    def __init__(self, log_interval: int = 100):
        self.log_interval = log_interval
        self.metrics = {
            'train_loss': [],
            'val_loss': [],
            'learning_rate': [],
            'grad_norm': []
        }
    
    def log(self, epoch: int, train_loss: float, val_loss: float, lr: float, grad_norm: float = 0.0):
        """Log metrics."""
        self.metrics['train_loss'].append(train_loss)
        self.metrics['val_loss'].append(val_loss)
        self.metrics['learning_rate'].append(lr)
        self.metrics['grad_norm'].append(grad_norm)
        
        if epoch % self.log_interval == 0:
            logger.info(
                f"Epoch {epoch} - Train: {train_loss:.6f}, Val: {val_loss:.6f}, "
                f"LR: {lr:.6f}, Grad: {grad_norm:.4f}"
            )
    
    def get_best_epoch(self) -> int:
        """Get epoch with best validation loss."""
        if len(self.metrics['val_loss']) == 0:
            return 0
        return np.argmin(self.metrics['val_loss'])

