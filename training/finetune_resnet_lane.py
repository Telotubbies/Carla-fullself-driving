"""
Fine-tune ResNet for lane detection.

Train ResNet to extract better lane features from CARLA images.
"""

import sys
import os
import yaml
import logging
import argparse
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
import numpy as np
import pandas as pd
from pathlib import Path
from tqdm import tqdm
import cv2
from PIL import Image
import torchvision.transforms as transforms
from typing import Tuple, Optional

sys.path.insert(0, str(Path(__file__).parent.parent))
from perception.resnet_encoder import ResNetEncoder
from perception.lane_detector import LaneDetector, create_lane_labels_from_carla
from utils.device_utils import get_device
from utils.status_logger import StatusLogger
from datetime import datetime

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class LaneDataset(Dataset):
    """Dataset for lane detection fine-tuning."""
    
    def __init__(self, images_dir: str, masks_dir: str = None, transform=None, augment: bool = True):
        """
        Initialize dataset.
        
        Args:
            images_dir: Directory with images
            masks_dir: Directory with lane masks (REQUIRED - must use CARLA labels)
            transform: Image transforms
            augment: Enable data augmentation
        """
        self.images_dir = Path(images_dir)
        self.masks_dir = Path(masks_dir) if masks_dir else None
        self.augment = augment
        
        # Load images
        self.image_files = sorted(list(self.images_dir.glob("*.png")))
        
        # Filter images that have corresponding masks
        if self.masks_dir:
            valid_files = []
            for img_path in self.image_files:
                mask_path = self.masks_dir / f"{img_path.stem}_lane.png"
                if mask_path.exists():
                    valid_files.append(img_path)
            self.image_files = valid_files
            logger.info(f"Found {len(self.image_files)} images with lane masks")
        else:
            logger.error("❌ masks_dir is REQUIRED! Use create_lane_labels.py first.")
            raise ValueError("masks_dir is required for training")
        
        if len(self.image_files) == 0:
            logger.error("❌ No images with masks found!")
            raise ValueError("No valid image-mask pairs found")
        
        if transform is None:
            self.transform = transforms.Compose([
                transforms.ToPILImage(),
                transforms.Resize((224, 224)),
                transforms.ToTensor(),
                transforms.Normalize(
                    mean=[0.485, 0.456, 0.406],
                    std=[0.229, 0.224, 0.225]
                )
            ])
        else:
            self.transform = transform
        
        logger.info(f"✅ Loaded {len(self.image_files)} image-mask pairs")
    
    def __len__(self):
        return len(self.image_files)
    
    def __getitem__(self, idx):
        # Load image
        img_path = self.image_files[idx]
        image = cv2.imread(str(img_path))
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        
        # Load mask (REQUIRED)
        mask_path = self.masks_dir / f"{img_path.stem}_lane.png"
        if not mask_path.exists():
            raise FileNotFoundError(f"Mask not found: {mask_path}")
        
        mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
        if mask is None:
            mask = np.zeros((image.shape[0], image.shape[1]), dtype=np.uint8)
        
        # Data augmentation
        if self.augment and np.random.rand() > 0.5:
            # Horizontal flip
            if np.random.rand() > 0.5:
                image = cv2.flip(image, 1)
                mask = cv2.flip(mask, 1)
            
            # Brightness adjustment
            if np.random.rand() > 0.5:
                alpha = np.random.uniform(0.8, 1.2)
                image = cv2.convertScaleAbs(image, alpha=alpha, beta=0)
        
        # Resize to model input size
        image_resized = cv2.resize(image, (224, 224))
        mask_resized = cv2.resize(mask, (224, 224), interpolation=cv2.INTER_NEAREST)
        
        # Normalize mask
        mask_resized = (mask_resized > 127).astype(np.float32)
        
        # Transform image
        image_tensor = self.transform(image_resized)
        
        return image_tensor, torch.FloatTensor(mask_resized)


class LaneDetectionHead(nn.Module):
    """Lane detection head on top of ResNet features."""
    
    def __init__(self, feature_dim: int = 512, output_size: Tuple[int, int] = (224, 224)):
        super(LaneDetectionHead, self).__init__()
        self.feature_dim = feature_dim
        self.output_size = output_size
        
        # Decoder
        self.decoder = nn.Sequential(
            nn.ConvTranspose2d(feature_dim, 256, 4, stride=2, padding=1),
            nn.ReLU(inplace=True),
            nn.ConvTranspose2d(256, 128, 4, stride=2, padding=1),
            nn.ReLU(inplace=True),
            nn.ConvTranspose2d(128, 64, 4, stride=2, padding=1),
            nn.ReLU(inplace=True),
            nn.ConvTranspose2d(64, 32, 4, stride=2, padding=1),
            nn.ReLU(inplace=True),
            nn.Conv2d(32, 1, 3, padding=1),
            nn.Sigmoid()
        )
    
    def forward(self, features: torch.Tensor) -> torch.Tensor:
        """
        Forward pass.
        
        Args:
            features: ResNet features (B, feature_dim, 7, 7) or (B, feature_dim)
            
        Returns:
            Lane mask (B, 1, H, W)
        """
        if features.dim() == 2:
            # Reshape to spatial
            B = features.size(0)
            features = features.view(B, self.feature_dim, 7, 7)
        
        return self.decoder(features)


def finetune_resnet_lane(
    data_dir: str,
    epochs: int = 20,
    batch_size: int = 16,
    lr: float = 0.001,
    freeze_backbone: bool = False,
    output_dir: str = None,
    masks_dir: str = None
):
    """
    Fine-tune ResNet for lane detection.
    
    Args:
        data_dir: Directory with images
        epochs: Number of epochs
        batch_size: Batch size
        lr: Learning rate
        freeze_backbone: Whether to freeze ResNet backbone
        output_dir: Output directory for model
    """
    data_dir = Path(data_dir)
    images_dir = data_dir / "images"
    
    if not images_dir.exists():
        logger.error(f"Images directory not found: {images_dir}")
        return
    
    # Check for lane masks
    if masks_dir is None:
        masks_dir = data_dir / "lane_masks"
    else:
        masks_dir = Path(masks_dir)
    
    if not masks_dir.exists() or len(list(masks_dir.glob("*_lane.png"))) == 0:
        logger.error(f"❌ Lane masks not found in {masks_dir}!")
        logger.error("   Please run: python3 training/create_lane_labels.py --images-dir <images> --output-dir <masks>")
        return
    
    # Setup output
    if output_dir is None:
        output_dir = data_dir / "resnet_lane_model"
    else:
        output_dir = Path(output_dir)
    output_dir.mkdir(exist_ok=True)
    
    # Device
    device = get_device()
    logger.info(f"Using device: {device}")
    
    # Create full dataset first to get valid indices
    full_dataset = LaneDataset(str(images_dir), str(masks_dir), augment=True)
    
    # Split dataset: 80% train, 20% validation (CRITICAL: ต้องมี validation)
    all_indices = list(range(len(full_dataset)))
    np.random.seed(42)
    np.random.shuffle(all_indices)
    split_idx = int(len(all_indices) * 0.8)
    train_indices = all_indices[:split_idx]
    val_indices = all_indices[split_idx:]
    
    # ตรวจสอบว่ามี validation data เพียงพอ
    if len(val_indices) < 10:
        logger.warning(f"⚠️  Validation set too small ({len(val_indices)} samples), using 90/10 split")
        split_idx = int(len(all_indices) * 0.9)
        train_indices = all_indices[:split_idx]
        val_indices = all_indices[split_idx:]
    
    # Create train/val subsets
    train_dataset = torch.utils.data.Subset(full_dataset, train_indices)
    val_dataset = torch.utils.data.Subset(full_dataset, val_indices)
    
    train_loader = DataLoader(train_dataset, batch_size=batch_size, shuffle=True, num_workers=2)
    val_loader = DataLoader(val_dataset, batch_size=batch_size, shuffle=False, num_workers=2)
    
    logger.info(f"📊 Dataset split: Train={len(train_indices)} images, Val={len(val_indices)} images")
    logger.info(f"   Train batches: {len(train_loader)}, Val batches: {len(val_loader)}")
    
    if len(val_loader) == 0:
        logger.error("❌ No validation data! Cannot train with validation.")
        return
    
    # Model
    resnet = ResNetEncoder(feature_dim=512, pretrained=True, freeze_backbone=freeze_backbone)
    lane_head = LaneDetectionHead(feature_dim=512)
    
    # Combine model
    class LaneResNet(nn.Module):
        def __init__(self, resnet, lane_head):
            super().__init__()
            self.resnet = resnet
            self.lane_head = lane_head
        
        def forward(self, x):
            # Get ResNet features (spatial)
            with torch.set_grad_enabled(not freeze_backbone):
                features = self.resnet.backbone(x)  # (B, 512, 7, 7)
            mask = self.lane_head(features)
            return mask
    
    model = LaneResNet(resnet, lane_head).to(device)
    
    # Loss function: Dice Loss + BCE Loss (better for segmentation)
    class DiceBCELoss(nn.Module):
        def __init__(self):
            super().__init__()
            self.bce = nn.BCELoss()
        
        def forward(self, pred, target):
            # BCE Loss
            bce_loss = self.bce(pred, target)
            
            # Dice Loss
            pred_flat = pred.view(-1)
            target_flat = target.view(-1)
            intersection = (pred_flat * target_flat).sum()
            dice_loss = 1 - (2. * intersection + 1e-6) / (pred_flat.sum() + target_flat.sum() + 1e-6)
            
            return bce_loss + dice_loss
    
    criterion = DiceBCELoss()
    optimizer = optim.Adam(model.parameters(), lr=lr)
    scheduler = optim.lr_scheduler.ReduceLROnPlateau(optimizer, mode='min', factor=0.5, patience=3, verbose=True)
    
    # Training loop with validation
    logger.info(f"Starting fine-tuning for {epochs} epochs...")
    best_val_loss = float('inf')
    
    train_losses = []
    val_losses = []
    for epoch in range(epochs):
        # Training phase
        model.train()
        train_loss = 0.0
        
        pbar = tqdm(train_loader, desc=f"Epoch {epoch+1}/{epochs} [Train]")
        for images, masks in pbar:
            images = images.to(device)
            masks = masks.unsqueeze(1).to(device)  # (B, 1, H, W)
            
            # Forward
            optimizer.zero_grad()
            pred_masks = model(images)
            
            # Resize to match
            if pred_masks.shape[2:] != masks.shape[2:]:
                pred_masks = F.interpolate(pred_masks, size=masks.shape[2:], mode='bilinear', align_corners=False)
            
            # Loss
            loss = criterion(pred_masks, masks)
            
            # Backward
            loss.backward()
            optimizer.step()
            
            train_loss += loss.item()
            pbar.set_postfix({'loss': f'{loss.item():.4f}'})
        
        avg_train_loss = train_loss / len(train_loader)
        train_losses.append(avg_train_loss)
        
        # Validation phase (CRITICAL: ต้องใช้ validation จริงๆ)
        model.eval()
        val_loss = 0.0
        val_samples = 0
        
        if len(val_loader) > 0:  # ตรวจสอบว่ามี validation data
            with torch.no_grad():
                val_pbar = tqdm(val_loader, desc=f"Epoch {epoch+1}/{epochs} [Val]", leave=False)
                for images, masks in val_pbar:
                    images = images.to(device)
                    masks = masks.unsqueeze(1).to(device)
                    
                    pred_masks = model(images)
                    if pred_masks.shape[2:] != masks.shape[2:]:
                        pred_masks = F.interpolate(pred_masks, size=masks.shape[2:], mode='bilinear', align_corners=False)
                    
                    loss = criterion(pred_masks, masks)
                    val_loss += loss.item()
                    val_samples += images.size(0)
                    val_pbar.set_postfix({'val_loss': f'{loss.item():.4f}'})
            
            avg_val_loss = val_loss / len(val_loader) if len(val_loader) > 0 else float('inf')
            val_losses.append(avg_val_loss)
            
            # Learning rate scheduling (ใช้ validation loss)
            scheduler.step(avg_val_loss)
            
            # Update status logger
            status_logger.update_training(
                'resnet_lane',
                current_epoch=epoch+1,
                train_loss=avg_train_loss,
                val_loss=avg_val_loss,
                best_val_loss=best_val_loss,
                learning_rate=scheduler.optimizer.param_groups[0]['lr'] if hasattr(scheduler, 'optimizer') else lr
            )
            
            logger.info(f"Epoch {epoch+1}/{epochs} - Train Loss: {avg_train_loss:.4f}, Val Loss: {avg_val_loss:.4f} ({val_samples} samples)")
            
            # Save best model based on validation loss (CRITICAL)
            if avg_val_loss < best_val_loss:
                best_val_loss = avg_val_loss
                best_path = output_dir / "resnet_lane_best.pth"
                torch.save(model.resnet.state_dict(), best_path)
                logger.info(f"✅ Saved best model (val_loss={avg_val_loss:.4f}): {best_path}")
        else:
            logger.warning("⚠️  No validation data! Using train loss for scheduling")
            avg_val_loss = avg_train_loss
            val_losses.append(avg_val_loss)
            scheduler.step(avg_train_loss)
            logger.info(f"Epoch {epoch+1}/{epochs} - Train Loss: {avg_train_loss:.4f} (No validation)")
        
        # Save checkpoint
        if (epoch + 1) % 5 == 0:
            checkpoint_path = output_dir / f"checkpoint_epoch_{epoch+1}.pth"
            torch.save({
                'epoch': epoch,
                'model_state_dict': model.state_dict(),
                'optimizer_state_dict': optimizer.state_dict(),
                'train_loss': avg_train_loss,
                'val_loss': avg_val_loss,
            }, checkpoint_path)
            logger.info(f"Saved checkpoint: {checkpoint_path}")
    
    # Save final model
    final_path = output_dir / "resnet_lane_final.pth"
    torch.save(model.resnet.state_dict(), final_path)
    logger.info(f"✅ Saved fine-tuned ResNet: {final_path}")
    
    # Also copy best model as final if it exists
    best_path = output_dir / "resnet_lane_best.pth"
    if best_path.exists():
        import shutil
        shutil.copy(best_path, final_path)
        logger.info(f"✅ Using best model as final (val_loss={best_val_loss:.4f})")
    
    # Save training history
    history = {
        'train_losses': train_losses,
        'val_losses': val_losses,
        'best_val_loss': best_val_loss,
        'best_epoch': val_losses.index(best_val_loss) + 1 if val_losses else None,
        'total_epochs': len(train_losses),
        'config': {
            'epochs': epochs,
            'batch_size': batch_size,
            'learning_rate': lr,
            'freeze_backbone': freeze_backbone
        }
    }
    history_path = output_dir / "training_history.json"
    import json
    with open(history_path, 'w') as f:
        json.dump(history, f, indent=2)
    logger.info(f"✅ Training history saved: {history_path}")
    
    # Update status logger
    status_logger.update_training(
        'resnet_lane',
        status='completed',
        completed_at=datetime.now().isoformat(),
        best_val_loss=best_val_loss,
        final_train_loss=train_losses[-1] if train_losses else None,
        final_val_loss=val_losses[-1] if val_losses else None
    )
    
    # Update model status
    if final_path.exists():
        size_mb = final_path.stat().st_size / (1024 * 1024)
        status_logger.update_model(
            'resnet_lane',
            exists=True,
            size=f"{size_mb:.1f}M",
            trained_at=datetime.now().isoformat(),
            val_loss=best_val_loss,
            path=str(final_path)
        )
    
    return final_path


def main():
    parser = argparse.ArgumentParser(description='Fine-tune ResNet for lane detection')
    parser.add_argument('--data-dir', type=str, required=True, help='Data directory with images')
    parser.add_argument('--epochs', type=int, default=10, help='Number of epochs')
    parser.add_argument('--batch-size', type=int, default=16, help='Batch size')
    parser.add_argument('--lr', type=float, default=0.001, help='Learning rate')
    parser.add_argument('--freeze', action='store_true', help='Freeze ResNet backbone')
    parser.add_argument('--output-dir', type=str, default=None, help='Output directory')
    parser.add_argument('--masks-dir', type=str, default=None, help='Lane masks directory (default: data_dir/lane_masks)')
    
    args = parser.parse_args()
    
    finetune_resnet_lane(
        data_dir=args.data_dir,
        epochs=args.epochs,
        batch_size=args.batch_size,
        lr=args.lr,
        freeze_backbone=args.freeze,
        output_dir=args.output_dir,
        masks_dir=args.masks_dir
    )


if __name__ == '__main__':
    main()

