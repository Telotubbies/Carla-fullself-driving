"""
Behavioral Cloning
Neural network model และ trainer สำหรับ imitation learning
"""

import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
from typing import Dict, Tuple, Optional
from pathlib import Path


class BehavioralCloningModel(nn.Module):
    """
    BC Model สำหรับ CARLA
    Input: LiDAR BEV (256x256x3) + Ego State (6)
    Output: Actions (3) [steering, throttle, brake]
    """
    
    def __init__(
        self,
        lidar_shape: Tuple[int, int, int] = (256, 256, 3),
        ego_state_dim: int = 6,
        action_dim: int = 3,
        hidden_dim: int = 256
    ):
        super().__init__()
        
        self.lidar_shape = lidar_shape
        self.ego_state_dim = ego_state_dim
        self.action_dim = action_dim
        
        # CNN สำหรับ LiDAR BEV
        self.lidar_encoder = nn.Sequential(
            nn.Conv2d(3, 32, kernel_size=8, stride=4, padding=2),
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=4, stride=2, padding=1),
            nn.ReLU(),
            nn.Conv2d(64, 128, kernel_size=3, stride=2, padding=1),
            nn.ReLU(),
            nn.Conv2d(128, 256, kernel_size=3, stride=2, padding=1),
            nn.ReLU(),
            nn.Flatten()
        )
        
        # คำนวณ output size ของ CNN
        with torch.no_grad():
            dummy_input = torch.zeros(1, 3, lidar_shape[0], lidar_shape[1])
            cnn_output_size = self.lidar_encoder(dummy_input).shape[1]
        
        # MLP สำหรับ ego state
        self.ego_encoder = nn.Sequential(
            nn.Linear(ego_state_dim, 64),
            nn.ReLU(),
            nn.Linear(64, 128),
            nn.ReLU()
        )
        
        # Fusion layer
        fusion_input_size = cnn_output_size + 128
        self.fusion = nn.Sequential(
            nn.Linear(fusion_input_size, hidden_dim),
            nn.ReLU(),
            nn.Dropout(0.2),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Dropout(0.2),
        )
        
        # Output heads
        self.steering_head = nn.Linear(hidden_dim, 1)
        self.throttle_head = nn.Linear(hidden_dim, 1)
        self.brake_head = nn.Linear(hidden_dim, 1)
    
    def forward(self, lidar_bev: torch.Tensor, ego_state: torch.Tensor) -> torch.Tensor:
        """
        Forward pass
        
        Args:
            lidar_bev: (batch, 256, 256, 3)
            ego_state: (batch, 6)
            
        Returns:
            actions: (batch, 3) [steering, throttle, brake]
        """
        # Transpose LiDAR to (batch, 3, 256, 256)
        if lidar_bev.dim() == 4 and lidar_bev.shape[-1] == 3:
            lidar_bev = lidar_bev.permute(0, 3, 1, 2)
        
        # Normalize LiDAR to [0, 1]
        lidar_bev = lidar_bev.float() / 255.0
        
        # Encode LiDAR
        lidar_features = self.lidar_encoder(lidar_bev)
        
        # Encode ego state
        ego_features = self.ego_encoder(ego_state)
        
        # Fusion
        combined = torch.cat([lidar_features, ego_features], dim=1)
        features = self.fusion(combined)
        
        # Output
        steering = torch.tanh(self.steering_head(features))
        throttle = torch.sigmoid(self.throttle_head(features))
        brake = torch.sigmoid(self.brake_head(features))
        
        actions = torch.cat([steering, throttle, brake], dim=1)
        return actions
    
    def predict(self, observation: Dict[str, np.ndarray]) -> np.ndarray:
        """
        Predict action จาก observation
        
        Args:
            observation: dict with 'lidar_bev' and 'ego_state'
            
        Returns:
            action: (3,) [steering, throttle, brake]
        """
        self.eval()
        with torch.no_grad():
            lidar_bev = torch.from_numpy(observation['lidar_bev']).unsqueeze(0)
            ego_state = torch.from_numpy(observation['ego_state']).unsqueeze(0).float()
            
            action = self.forward(lidar_bev, ego_state)
            return action.squeeze(0).cpu().numpy()


class BCTrainer:
    """Trainer สำหรับ Behavioral Cloning"""
    
    def __init__(
        self,
        model: BehavioralCloningModel,
        learning_rate: float = 1e-4,
        weight_decay: float = 1e-5,
        device: str = 'cuda' if torch.cuda.is_available() else 'cpu'
    ):
        self.model = model.to(device)
        self.device = device
        
        self.optimizer = torch.optim.Adam(
            model.parameters(),
            lr=learning_rate,
            weight_decay=weight_decay
        )
        
        self.criterion = nn.MSELoss()
        
        self.train_losses = []
        self.val_losses = []
    
    def train_step(
        self,
        lidar_bev: torch.Tensor,
        ego_state: torch.Tensor,
        actions: torch.Tensor
    ) -> float:
        """Training step"""
        self.model.train()
        
        lidar_bev = lidar_bev.to(self.device)
        ego_state = ego_state.to(self.device)
        actions = actions.to(self.device)
        
        # Forward pass
        predicted_actions = self.model(lidar_bev, ego_state)
        
        # Compute loss
        loss = self.criterion(predicted_actions, actions)
        
        # Backward pass
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()
        
        return loss.item()
    
    def validate(
        self,
        lidar_bev: torch.Tensor,
        ego_state: torch.Tensor,
        actions: torch.Tensor
    ) -> float:
        """Validation step"""
        self.model.eval()
        
        with torch.no_grad():
            lidar_bev = lidar_bev.to(self.device)
            ego_state = ego_state.to(self.device)
            actions = actions.to(self.device)
            
            predicted_actions = self.model(lidar_bev, ego_state)
            loss = self.criterion(predicted_actions, actions)
        
        return loss.item()
    
    def train_epoch(
        self,
        train_loader: torch.utils.data.DataLoader,
        val_loader: Optional[torch.utils.data.DataLoader] = None
    ) -> Dict[str, float]:
        """Train one epoch"""
        train_losses = []
        
        for batch in train_loader:
            lidar_bev, ego_state, actions = batch
            loss = self.train_step(lidar_bev, ego_state, actions)
            train_losses.append(loss)
        
        avg_train_loss = np.mean(train_losses)
        self.train_losses.append(avg_train_loss)
        
        results = {'train_loss': avg_train_loss}
        
        if val_loader is not None:
            val_losses = []
            for batch in val_loader:
                lidar_bev, ego_state, actions = batch
                loss = self.validate(lidar_bev, ego_state, actions)
                val_losses.append(loss)
            
            avg_val_loss = np.mean(val_losses)
            self.val_losses.append(avg_val_loss)
            results['val_loss'] = avg_val_loss
        
        return results
    
    def save_checkpoint(self, filepath: str):
        """Save model checkpoint"""
        checkpoint = {
            'model_state_dict': self.model.state_dict(),
            'optimizer_state_dict': self.optimizer.state_dict(),
            'train_losses': self.train_losses,
            'val_losses': self.val_losses,
        }
        
        torch.save(checkpoint, filepath)
        print(f"✅ Checkpoint saved to {filepath}")
    
    def load_checkpoint(self, filepath: str):
        """Load model checkpoint"""
        checkpoint = torch.load(filepath, map_location=self.device)
        
        self.model.load_state_dict(checkpoint['model_state_dict'])
        self.optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
        self.train_losses = checkpoint.get('train_losses', [])
        self.val_losses = checkpoint.get('val_losses', [])
        
        print(f"✅ Checkpoint loaded from {filepath}")


class ExpertDataset(torch.utils.data.Dataset):
    """Dataset สำหรับ expert demonstrations"""
    
    def __init__(self, demonstrations: list):
        """
        Args:
            demonstrations: list of episode dicts from DataCollector
        """
        self.lidar_bevs = []
        self.ego_states = []
        self.actions = []
        
        # Flatten all episodes
        for episode in demonstrations:
            for obs, action in zip(episode['observations'], episode['actions']):
                self.lidar_bevs.append(obs['lidar_bev'])
                self.ego_states.append(obs['ego_state'])
                self.actions.append(action)
        
        self.lidar_bevs = np.array(self.lidar_bevs)
        self.ego_states = np.array(self.ego_states)
        self.actions = np.array(self.actions)
        
        print(f"✅ Dataset created: {len(self)} samples")
    
    def __len__(self):
        return len(self.actions)
    
    def __getitem__(self, idx):
        lidar_bev = torch.from_numpy(self.lidar_bevs[idx])
        ego_state = torch.from_numpy(self.ego_states[idx]).float()
        action = torch.from_numpy(self.actions[idx]).float()
        
        return lidar_bev, ego_state, action


def train_bc_model(
    demonstrations: list,
    num_epochs: int = 100,
    batch_size: int = 32,
    learning_rate: float = 1e-4,
    val_split: float = 0.2,
    save_dir: str = "models",
    device: str = 'cuda' if torch.cuda.is_available() else 'cpu'
) -> BehavioralCloningModel:
    """
    Train BC model จาก demonstrations
    
    Args:
        demonstrations: list of episodes from DataCollector
        num_epochs: จำนวน epochs
        batch_size: batch size
        learning_rate: learning rate
        val_split: validation split ratio
        save_dir: directory สำหรับบันทึก model
        device: device to use
        
    Returns:
        Trained BC model
    """
    # สร้าง dataset
    dataset = ExpertDataset(demonstrations)
    
    # Split train/val
    val_size = int(len(dataset) * val_split)
    train_size = len(dataset) - val_size
    
    train_dataset, val_dataset = torch.utils.data.random_split(
        dataset, [train_size, val_size]
    )
    
    # สร้าง data loaders
    train_loader = torch.utils.data.DataLoader(
        train_dataset, batch_size=batch_size, shuffle=True
    )
    val_loader = torch.utils.data.DataLoader(
        val_dataset, batch_size=batch_size, shuffle=False
    )
    
    # สร้าง model และ trainer
    model = BehavioralCloningModel()
    trainer = BCTrainer(model, learning_rate=learning_rate, device=device)
    
    print(f"\n{'='*80}")
    print(f"🎓 TRAINING BEHAVIORAL CLONING MODEL")
    print(f"{'='*80}")
    print(f"Device: {device}")
    print(f"Train samples: {train_size}")
    print(f"Val samples: {val_size}")
    print(f"Batch size: {batch_size}")
    print(f"Epochs: {num_epochs}")
    print(f"{'='*80}\n")
    
    # Training loop
    best_val_loss = float('inf')
    save_path = Path(save_dir)
    save_path.mkdir(parents=True, exist_ok=True)
    
    for epoch in range(num_epochs):
        results = trainer.train_epoch(train_loader, val_loader)
        
        train_loss = results['train_loss']
        val_loss = results.get('val_loss', 0)
        
        print(f"Epoch {epoch+1}/{num_epochs}: "
              f"train_loss={train_loss:.4f}, val_loss={val_loss:.4f}")
        
        # Save best model
        if val_loss < best_val_loss:
            best_val_loss = val_loss
            trainer.save_checkpoint(str(save_path / "bc_model_best.pth"))
    
    # Save final model
    trainer.save_checkpoint(str(save_path / "bc_model_final.pth"))
    
    print(f"\n✅ Training complete! Best val loss: {best_val_loss:.4f}")
    
    return model
