"""
Imitation Learning (Behavioral Cloning) Training
Pre-train policy from expert demonstrations
"""

import torch
import torch.nn as nn
import torch.optim as optim
import numpy as np
import argparse
import os
import json
import logging
from typing import Dict, List, Tuple
from torch.utils.data import Dataset, DataLoader
import yaml
import sys
from pathlib import Path

# Add project root to path
sys.path.insert(0, str(Path(__file__).parent.parent))

from models.custom_policy import VisionActorCriticPolicy
from models.vision_encoder import VisionTemporalEncoder
import gymnasium as gym


class ExpertDataset(Dataset):
    """Dataset for expert demonstrations"""
    
    def __init__(self, demonstrations_path: str, sequence_length: int = 4):
        """
        Load expert demonstrations
        
        Args:
            demonstrations_path: Path to .npz file with demonstrations
            sequence_length: Number of frames to stack
        """
        data = np.load(demonstrations_path)
        
        self.rgb = data['rgb']  # (N, H, W, 3)
        self.depth = data['depth']  # (N, H, W, 1)
        self.actions = data['actions']  # (N, 3) - [steer, throttle, brake]
        self.speeds = data['speeds']
        self.dones = data['dones']
        self.collisions = data['collisions']
        
        # Stack RGB + Depth
        self.observations = np.concatenate([self.rgb, self.depth], axis=-1)  # (N, H, W, 4)
        
        # Normalize to [0, 1] if not already
        if self.observations.max() > 1.0:
            self.observations = self.observations.astype(np.float32) / 255.0
        
        # Create sequences
        self.sequences = []
        self.sequence_actions = []
        
        episode_start = 0
        for i in range(len(self.dones)):
            if self.dones[i] or i == len(self.dones) - 1:
                episode_end = i + 1
                episode_obs = self.observations[episode_start:episode_end]
                episode_actions = self.actions[episode_start:episode_end]
                
                # Create sequences for this episode
                for j in range(len(episode_obs) - sequence_length + 1):
                    seq_obs = episode_obs[j:j+sequence_length]  # (seq_len, H, W, 4)
                    seq_action = episode_actions[j+sequence_length-1]  # Action for last frame
                    self.sequences.append(seq_obs)
                    self.sequence_actions.append(seq_action)
                
                episode_start = episode_end
        
        self.sequences = np.array(self.sequences)  # (M, seq_len, H, W, 4)
        self.sequence_actions = np.array(self.sequence_actions)  # (M, 3)
        
        logging.info(f"✅ Loaded {len(self.sequences)} sequences from {len(self.dones)} steps")
    
    def __len__(self):
        return len(self.sequences)
    
    def __getitem__(self, idx):
        obs = self.sequences[idx]  # (seq_len, H, W, 4)
        action = self.sequence_actions[idx]  # (3,)
        
        # Convert to torch tensors
        obs = torch.from_numpy(obs).float()  # (seq_len, H, W, 4)
        action = torch.from_numpy(action).float()  # (3,)
        
        # Permute to (seq_len, C, H, W) for CNN
        # From (seq_len, H, W, 4) to (seq_len, 4, H, W)
        obs = obs.permute(0, 3, 1, 2)  # (seq_len, 4, H, W)
        
        # Verify shape: should be (seq_len, 4, 90, 160) for image_size (90, 160)
        # Vision encoder expects: (batch, seq_len, channels, height, width)
        # DataLoader will add batch dimension: (batch, seq_len, 4, H, W)
        
        return obs, action


class ImitationLearningTrainer:
    """Train policy using Behavioral Cloning"""
    
    def __init__(
        self,
        config_path: str,
        demonstrations_path: str,
        output_dir: str = 'checkpoints/il_pretrained',
        batch_size: int = 32,
        learning_rate: float = 1e-4,
        num_epochs: int = 50,
        device: str = None
    ):
        """
        Initialize IL trainer
        
        Args:
            config_path: Path to config YAML
            demonstrations_path: Path to expert demonstrations .npz file
            output_dir: Output directory for saved models
            batch_size: Batch size for training
            learning_rate: Learning rate
            num_epochs: Number of training epochs
            device: Device ('cuda' or 'cpu')
        """
        # Load config
        with open(config_path, 'r') as f:
            self.config = yaml.safe_load(f)
        
        # Setup
        self.demonstrations_path = demonstrations_path
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)
        
        # Device - prefer CPU if GPU memory is full (RL training may be using GPU)
        if device is None:
            if torch.cuda.is_available():
                # Check GPU memory
                try:
                    torch.cuda.empty_cache()
                    # Try to allocate a small tensor to check if GPU is available
                    test_tensor = torch.zeros(1).cuda()
                    del test_tensor
                    torch.cuda.empty_cache()
                    device = 'cuda'
                    logging.info("GPU available, using CUDA")
                except RuntimeError as e:
                    if "out of memory" in str(e).lower():
                        logging.warning("GPU memory full, falling back to CPU")
                        device = 'cpu'
                    else:
                        device = 'cuda'
            else:
                device = 'cpu'
        self.device = torch.device(device)
        logging.info(f"Using device: {self.device}")
        
        # Create observation and action spaces
        obs_config = self.config.get('observations', {})
        image_size = obs_config.get('image_size', [160, 90])
        sequence_length = obs_config.get('stack_frames', 4)
        use_depth = obs_config.get('use_depth', True)
        
        if use_depth:
            obs_shape = (sequence_length, image_size[1], image_size[0], 4)  # (seq_len, H, W, 4)
        else:
            obs_shape = (sequence_length, image_size[1], image_size[0], 3)  # (seq_len, H, W, 3)
        
        observation_space = gym.spaces.Box(
            low=0.0,
            high=1.0,
            shape=obs_shape,
            dtype=np.float32
        )
        
        action_space = gym.spaces.Box(
            low=np.array([-1.0, 0.0, 0.0]),
            high=np.array([1.0, 1.0, 1.0]),
            dtype=np.float32
        )
        
        # Create policy (actor only, no critic needed for BC)
        # For BC, we need a simpler network architecture
        # ActorCriticPolicy creates: features -> mlp_extractor -> latent_pi -> action_net -> actions
        # MlpExtractor: if net_arch=[], latent_dim_pi = feature_dim (256)
        #               if net_arch=[128], latent_dim_pi = 128
        # action_net expects latent_dim_pi as input
        # So we need to ensure net_arch matches what we want
        # Let's use net_arch=[] to get latent_dim_pi = 256, then action_net will be Linear(256, 3)
        # But config may override this, so let's explicitly set it
        # IMPORTANT: Pass net_arch=[] directly to override any config values
        import copy
        config_override = copy.deepcopy(self.config)
        # Remove network.policy.hidden_sizes from config to prevent override
        if 'network' in config_override and 'policy' in config_override['network']:
            if 'hidden_sizes' in config_override['network']['policy']:
                del config_override['network']['policy']['hidden_sizes']
        
        self.policy = VisionActorCriticPolicy(
            observation_space=observation_space,
            action_space=action_space,
            lr_schedule=lambda _: learning_rate,
            net_arch=[],  # Direct: features_dim (256) -> action_dim (3) - FORCE empty list
            config=config_override  # Use overridden config
        ).to(self.device)
        
        # Verify the policy structure
        logging.info(f"✅ Policy created:")
        logging.info(f"   mlp_extractor.latent_dim_pi: {self.policy.mlp_extractor.latent_dim_pi}")
        # Check action_net structure
        if hasattr(self.policy.action_net, '__getitem__'):
            first_layer = self.policy.action_net[0]
            if hasattr(first_layer, 'in_features'):
                logging.info(f"   action_net[0].in_features: {first_layer.in_features}")
                logging.info(f"   action_net[0].out_features: {first_layer.out_features}")
        else:
            # action_net might be a single layer or distribution
            if hasattr(self.policy.action_net, 'in_features'):
                logging.info(f"   action_net.in_features: {self.policy.action_net.in_features}")
                logging.info(f"   action_net.out_features: {self.policy.action_net.out_features}")
        
        # Verify dimensions match
        if self.policy.mlp_extractor.latent_dim_pi != 256:
            logging.warning(f"⚠️  WARNING: latent_dim_pi={self.policy.mlp_extractor.latent_dim_pi}, expected 256!")
        
        # Create dataset and dataloader
        self.dataset = ExpertDataset(demonstrations_path, sequence_length=sequence_length)
        self.dataloader = DataLoader(
            self.dataset,
            batch_size=batch_size,
            shuffle=True,
            num_workers=0,  # Set to 0 to avoid multiprocessing issues
            pin_memory=True if self.device.type == 'cuda' else False
        )
        
        # Optimizer
        self.optimizer = optim.Adam(self.policy.parameters(), lr=learning_rate)
        
        # Loss function (MSE for continuous actions)
        self.criterion = nn.MSELoss()
        
        # Training parameters
        self.num_epochs = num_epochs
        self.batch_size = batch_size
        
        logging.info(f"✅ IL Trainer initialized: {len(self.dataset)} sequences, {num_epochs} epochs")
    
    def train(self):
        """Train policy using Behavioral Cloning"""
        logging.info("Starting Imitation Learning training...")
        
        best_loss = float('inf')
        
        for epoch in range(self.num_epochs):
            epoch_losses = []
            
            self.policy.train()
            
            for batch_idx, (observations, actions) in enumerate(self.dataloader):
                # Move to device
                # observations from DataLoader: (batch, seq_len, C, H, W)
                # But extract_features expects: (batch, seq_len, H, W, C)
                # So we need to permute back: (batch, seq_len, C, H, W) -> (batch, seq_len, H, W, C)
                observations = observations.to(self.device)  # (batch, seq_len, C, H, W)
                observations = observations.permute(0, 1, 3, 4, 2)  # (batch, seq_len, H, W, C)
                actions = actions.to(self.device)  # (batch, 3)
                
                # Forward pass
                # Get action distribution from policy
                # extract_features will permute back to (batch, seq_len, C, H, W) internally
                features = self.policy.extract_features(observations)  # (batch, features_dim=256)
                
                # For Behavioral Cloning, we need to go through mlp_extractor first
                # ActorCriticPolicy structure:
                #   features -> mlp_extractor -> latent_pi -> action_net -> actions
                # So we need: features -> mlp_extractor -> action_net
                latent_pi, _ = self.policy.mlp_extractor(features)  # (batch, latent_dim_pi)
                predicted_actions = self.policy.action_net(latent_pi)  # (batch, 3)
                
                # Compute loss
                loss = self.criterion(predicted_actions, actions)
                
                # Backward pass
                self.optimizer.zero_grad()
                loss.backward()
                self.optimizer.step()
                
                epoch_losses.append(loss.item())
                
                if batch_idx % 100 == 0:
                    logging.info(f"Epoch {epoch+1}/{self.num_epochs}, Batch {batch_idx}/{len(self.dataloader)}, Loss: {loss.item():.4f}")
            
            avg_loss = np.mean(epoch_losses)
            logging.info(f"Epoch {epoch+1}/{self.num_epochs} completed, Average Loss: {avg_loss:.4f}")
            
            # Save checkpoint
            if avg_loss < best_loss:
                best_loss = avg_loss
                self.save_checkpoint(epoch, avg_loss, is_best=True)
            
            # Save periodic checkpoint
            if (epoch + 1) % 10 == 0:
                self.save_checkpoint(epoch, avg_loss, is_best=False)
        
        logging.info(f"✅ Training complete! Best loss: {best_loss:.4f}")
    
    def save_checkpoint(self, epoch: int, loss: float, is_best: bool = False):
        """Save model checkpoint"""
        checkpoint = {
            'epoch': epoch,
            'loss': loss,
            'policy_state_dict': self.policy.state_dict(),
            'optimizer_state_dict': self.optimizer.state_dict(),
            'config': self.config
        }
        
        if is_best:
            path = os.path.join(self.output_dir, 'best_il_model.pt')
        else:
            path = os.path.join(self.output_dir, f'il_model_epoch_{epoch+1}.pt')
        
        torch.save(checkpoint, path)
        logging.info(f"Saved checkpoint: {path} (loss: {loss:.4f})")


def main():
    parser = argparse.ArgumentParser(description='Train policy using Imitation Learning')
    parser.add_argument('--config', type=str, required=True, help='Path to config YAML')
    parser.add_argument('--demonstrations', type=str, required=True, help='Path to expert demonstrations .npz file')
    parser.add_argument('--output-dir', type=str, default='checkpoints/il_pretrained', help='Output directory')
    parser.add_argument('--batch-size', type=int, default=16, help='Batch size (reduced for GPU memory)')
    parser.add_argument('--learning-rate', type=float, default=1e-4, help='Learning rate')
    parser.add_argument('--num-epochs', type=int, default=50, help='Number of epochs')
    parser.add_argument('--device', type=str, default=None, help='Device (cuda/cpu). Default: cpu if GPU memory full')
    
    args = parser.parse_args()
    
    # Setup logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    # Create trainer
    trainer = ImitationLearningTrainer(
        config_path=args.config,
        demonstrations_path=args.demonstrations,
        output_dir=args.output_dir,
        batch_size=args.batch_size,
        learning_rate=args.learning_rate,
        num_epochs=args.num_epochs,
        device=args.device
    )
    
    # Train
    trainer.train()


if __name__ == '__main__':
    main()

