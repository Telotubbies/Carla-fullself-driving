"""
Train U-Net for lane detection.

Create lane labels from CARLA and train U-Net model.
"""

import sys
import argparse
import logging
import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import Dataset, DataLoader
import numpy as np
from pathlib import Path
from tqdm import tqdm
import cv2

sys.path.insert(0, str(Path(__file__).parent.parent))
from perception.lane_detector import LaneUNet, LaneDetector
from utils.device_utils import get_device

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class LaneUNetDataset(Dataset):
    """Dataset for U-Net lane detection training."""
    
    def __init__(self, images_dir: str, masks_dir: str, transform=None):
        self.images_dir = Path(images_dir)
        self.masks_dir = Path(masks_dir)
        
        self.image_files = sorted(list(self.images_dir.glob("*.png")))
        logger.info(f"Loaded {len(self.image_files)} images")
    
    def __len__(self):
        return len(self.image_files)
    
    def __getitem__(self, idx):
        # Load image
        img_path = self.image_files[idx]
        image = cv2.imread(str(img_path))
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = cv2.resize(image, (256, 256))
        image = image.astype(np.float32) / 255.0
        image = torch.FloatTensor(image).permute(2, 0, 1)
        
        # Load mask
        mask_path = self.masks_dir / f"{img_path.stem}_lane.png"
        if mask_path.exists():
            mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
        else:
            mask = np.zeros((256, 256), dtype=np.uint8)
        
        mask = cv2.resize(mask, (256, 256), interpolation=cv2.INTER_NEAREST)
        mask = (mask > 127).astype(np.float32)
        mask = torch.FloatTensor(mask).unsqueeze(0)
        
        return image, mask


def train_lane_unet(
    images_dir: str,
    masks_dir: str,
    epochs: int = 20,
    batch_size: int = 8,
    lr: float = 0.0001,
    output_dir: str = None
):
    """Train U-Net for lane detection."""
    device = get_device()
    logger.info(f"Using device: {device}")
    
    # Dataset
    dataset = LaneUNetDataset(images_dir, masks_dir)
    dataloader = DataLoader(dataset, batch_size=batch_size, shuffle=True, num_workers=2)
    
    # Model
    model = LaneUNet(in_channels=3, num_classes=2).to(device)
    
    # Loss and optimizer
    criterion = nn.CrossEntropyLoss()
    optimizer = optim.Adam(model.parameters(), lr=lr)
    
    # Training
    logger.info(f"Training U-Net for {epochs} epochs...")
    
    for epoch in range(epochs):
        model.train()
        total_loss = 0.0
        
        pbar = tqdm(dataloader, desc=f"Epoch {epoch+1}/{epochs}")
        for images, masks in pbar:
            images = images.to(device)
            masks = masks.squeeze(1).long().to(device)  # (B, H, W)
            
            optimizer.zero_grad()
            outputs = model(images)  # (B, 2, H, W)
            loss = criterion(outputs, masks)
            loss.backward()
            optimizer.step()
            
            total_loss += loss.item()
            pbar.set_postfix({'loss': f'{loss.item():.4f}'})
        
        avg_loss = total_loss / len(dataloader)
        logger.info(f"Epoch {epoch+1}/{epochs} - Loss: {avg_loss:.4f}")
        
        # Save checkpoint
        if (epoch + 1) % 5 == 0:
            if output_dir is None:
                output_dir = Path(images_dir).parent / "lane_unet_model"
            else:
                output_dir = Path(output_dir)
            output_dir.mkdir(exist_ok=True)
            
            checkpoint_path = output_dir / f"unet_epoch_{epoch+1}.pth"
            torch.save(model.state_dict(), checkpoint_path)
            logger.info(f"Saved: {checkpoint_path}")
    
    # Save final model
    if output_dir is None:
        output_dir = Path(images_dir).parent / "lane_unet_model"
    else:
        output_dir = Path(output_dir)
    output_dir.mkdir(exist_ok=True)
    
    final_path = output_dir / "lane_unet_final.pth"
    torch.save(model.state_dict(), final_path)
    logger.info(f"✅ Saved U-Net model: {final_path}")
    
    return final_path


def main():
    parser = argparse.ArgumentParser(description='Train U-Net for lane detection')
    parser.add_argument('--images-dir', type=str, required=True, help='Images directory')
    parser.add_argument('--masks-dir', type=str, required=True, help='Lane masks directory')
    parser.add_argument('--epochs', type=int, default=20, help='Number of epochs')
    parser.add_argument('--batch-size', type=int, default=8, help='Batch size')
    parser.add_argument('--lr', type=float, default=0.0001, help='Learning rate')
    parser.add_argument('--output-dir', type=str, default=None, help='Output directory')
    
    args = parser.parse_args()
    
    train_lane_unet(
        images_dir=args.images_dir,
        masks_dir=args.masks_dir,
        epochs=args.epochs,
        batch_size=args.batch_size,
        lr=args.lr,
        output_dir=args.output_dir
    )


if __name__ == '__main__':
    main()

