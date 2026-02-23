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
from datetime import datetime

logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(name)s - %(levelname)s - %(message)s')
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
        try:
            image = cv2.imread(str(img_path))
            if image is None:
                raise ValueError(f"Failed to load image: {img_path}")
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            image = cv2.resize(image, (256, 256))
            image = image.astype(np.float32) / 255.0
            image = torch.FloatTensor(image).permute(2, 0, 1)
        except Exception as e:
            # Return black image if loading fails
            image = torch.zeros((3, 256, 256), dtype=torch.float32)
        
        # Load mask
        mask_path = self.masks_dir / f"{img_path.stem}_lane.png"
        try:
            if mask_path.exists():
                mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
                if mask is None:
                    mask = np.zeros((256, 256), dtype=np.uint8)
            else:
                mask = np.zeros((256, 256), dtype=np.uint8)
        except Exception:
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
    
    # Force GPU only - no CPU fallback
    if device.type != 'cuda':
        raise RuntimeError("GPU is required but not available. Please ensure GPU is properly configured.")
    
    # Test GPU with a simple operation before training
    try:
        test_tensor = torch.zeros(1).to(device)
        _ = test_tensor + 1
        torch.cuda.synchronize()  # Ensure GPU is ready
        logger.info(f"✅ GPU device verified: {device}")
    except Exception as e:
        logger.error(f"❌ GPU test failed: {e}")
        raise RuntimeError(f"GPU verification failed: {e}. Cannot proceed with GPU-only training.")
    
    # Dataset
    dataset = LaneUNetDataset(images_dir, masks_dir)
    # Use num_workers=0 to avoid multiprocessing issues, or use pin_memory=True for GPU
    dataloader = DataLoader(dataset, batch_size=batch_size, shuffle=True, num_workers=0, pin_memory=True if device.type == 'cuda' else False)
    
    # Model
    model = LaneUNet(in_channels=3, num_classes=2).to(device)
    
    # Configure PyTorch for ROCm stability
    torch.backends.cudnn.benchmark = False  # Disable for stability
    torch.backends.cudnn.deterministic = True
    
    # Warm up GPU with a small forward pass
    logger.info("Warming up GPU with small forward pass...")
    try:
        model.eval()
        with torch.no_grad():
            warmup_input = torch.randn(1, 3, 64, 64).to(device)  # Smaller size for warmup
            _ = model(warmup_input)
            torch.cuda.synchronize()  # Wait for GPU to finish
        model.train()
        logger.info("✅ GPU warmup successful")
    except Exception as e:
        logger.error(f"❌ GPU warmup failed: {e}")
        raise RuntimeError(f"GPU warmup failed: {e}. Cannot proceed with GPU-only training.")
    
    # Loss and optimizer
    criterion = nn.CrossEntropyLoss()
    optimizer = optim.Adam(model.parameters(), lr=lr)
    
    # Training
    logger.info(f"Training U-Net for {epochs} epochs...")
    logger.info(f"Dataset size: {len(dataset)}, Batches per epoch: {len(dataloader)}")
    
    for epoch in range(epochs):
        model.train()
        total_loss = 0.0
        
        logger.info(f"Starting Epoch {epoch+1}/{epochs}")
        pbar = tqdm(dataloader, desc=f"Epoch {epoch+1}/{epochs}")
        for batch_idx, (images, masks) in enumerate(pbar):
            if batch_idx == 0:
                logger.info(f"Epoch {epoch+1}: First batch loaded, shape: {images.shape}, device: {images.device}")
            images = images.to(device)
            masks = masks.squeeze(1).long().to(device)  # (B, H, W)
            
            if batch_idx == 0:
                logger.info(f"Epoch {epoch+1}: Data moved to GPU, images.device: {images.device}, masks.device: {masks.device}")
            
            optimizer.zero_grad()
            
            # GPU-only training - no fallback
            try:
                # Forward pass
                outputs = model(images)  # (B, 2, H, W)
                loss = criterion(outputs, masks)
                
                # Backward pass
                loss.backward()
                
                # Optimizer step
                optimizer.step()
                
                # Synchronize GPU after first batch to ensure it's working
                if batch_idx == 0:
                    torch.cuda.synchronize()
                    logger.info(f"✅ First batch completed successfully on GPU, Loss: {loss.item():.4f}")
                    
            except RuntimeError as e:
                error_str = str(e)
                if 'HIP error' in error_str or 'rocBLAS' in error_str or 'invalid device function' in error_str:
                    logger.error(f"❌ ROCm GPU error: {e}")
                    logger.error("This is a ROCm compatibility issue. Possible solutions:")
                    logger.error("1. Check HSA_OVERRIDE_GFX_VERSION=11.0.0 is set")
                    logger.error("2. Verify ROCm installation and PyTorch ROCm compatibility")
                    logger.error("3. Try reducing batch size")
                    raise RuntimeError(f"ROCm GPU error: {e}. Cannot proceed with GPU-only training.")
                else:
                    logger.error(f"❌ GPU training error: {e}", exc_info=True)
                    raise
            except Exception as e:
                logger.error(f"❌ Error in training step (batch {batch_idx}): {e}", exc_info=True)
                raise
            
            # Force flush every 10 batches to ensure log updates
            if batch_idx % 10 == 0:
                import sys
                sys.stdout.flush()
                sys.stderr.flush()
            
            total_loss += loss.item()
            if batch_idx % 100 == 0:
                logger.info(f"Epoch {epoch+1}/{epochs}, Batch {batch_idx}/{len(dataloader)}, Loss: {loss.item():.4f}")
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
    parser.add_argument('--log-dir', type=str, default='logs', help='Log directory')
    
    args = parser.parse_args()
    
    # Create log directory if it doesn't exist
    log_dir = Path(args.log_dir)
    log_dir.mkdir(exist_ok=True)
    
    # Generate timestamp for log file
    from datetime import datetime
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_file = log_dir / f"train_unet_{timestamp}.log"
    
    # Setup logging to file
    import logging
    file_handler = logging.FileHandler(log_file)
    file_handler.setLevel(logging.INFO)
    file_handler.setFormatter(logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s'))
    logger.addHandler(file_handler)
    
    logger.info(f"Logging to: {log_file}")
    
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

