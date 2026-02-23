"""
Extract features from images using ResNet + Lane Detector.

STEP 2: image → ResNet (512) + Lane Features (128) → combined feature vector (640)
"""

import sys
import os
import yaml
import logging
import argparse
import numpy as np
import pandas as pd
from pathlib import Path
from tqdm import tqdm
import cv2

# Add project root to path
project_root = Path(__file__).parent.parent
sys.path.insert(0, str(project_root))

from perception import ResNetEncoder
from perception.lane_detector import LaneDetector
from training.data_preprocessing import preprocess_dataset
from datetime import datetime

# Setup logging with file handler
def setup_logging(log_dir: Path = None):
    """Setup logging to both console and file."""
    if log_dir is None:
        log_dir = project_root / "logs" / "training"
    log_dir.mkdir(parents=True, exist_ok=True)
    
    # Create log file with timestamp
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_file = log_dir / f"extract_features_{timestamp}.log"
    
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
    file_handler = logging.FileHandler(log_file)
    file_handler.setLevel(logging.INFO)
    file_handler.setFormatter(formatter)
    
    # Root logger
    root_logger = logging.getLogger()
    root_logger.setLevel(logging.INFO)
    root_logger.handlers = []  # Clear existing handlers
    root_logger.addHandler(console_handler)
    root_logger.addHandler(file_handler)
    
    logger = logging.getLogger(__name__)
    logger.info(f"Logging to file: {log_file}")
    
    return log_file

# Setup logging
log_file = setup_logging()
logger = logging.getLogger(__name__)


def extract_features(data_dir: str, output_path: str = None, batch_size: int = 32, preprocess: bool = True, resnet_model: str = None, use_lane_features: bool = True, lane_model_path: str = None):
    """
    Extract features from images (ResNet + Lane Features).
    
    Args:
        data_dir: Directory containing data.csv and images/
        output_path: Output path for features.npy (default: data_dir/features.npy)
        batch_size: Batch size for processing
        preprocess: Whether to preprocess data first
        resnet_model: Path to fine-tuned ResNet model (optional)
        use_lane_features: Whether to include lane features (default: True)
        lane_model_path: Path to U-Net lane detection model (optional, uses CARLA if None)
    """
    data_dir = Path(data_dir)
    
    if output_path is None:
        output_path = data_dir / "features.npy"
    else:
        output_path = Path(output_path)
    
    # Preprocess data first (if requested)
    if preprocess:
        logger.info("Preprocessing data...")
        try:
            preprocess_dataset(data_dir)
            # Use processed data
            csv_path = data_dir / "processed" / "data_processed.csv"
            if not csv_path.exists():
                csv_path = data_dir / "data.csv"
        except Exception as e:
            logger.warning(f"Preprocessing failed: {e}, using raw data")
            csv_path = data_dir / "data.csv"
    else:
        csv_path = data_dir / "data.csv"
    
    if not csv_path.exists():
        logger.error(f"data.csv not found in {data_dir}")
        return
    
    df = pd.read_csv(csv_path)
    logger.info(f"Loaded {len(df)} records from {csv_path}")
    
    # Initialize ResNet encoder
    logger.info("Initializing ResNet encoder...")
    if resnet_model and Path(resnet_model).exists():
        logger.info(f"✅ Using fine-tuned ResNet: {resnet_model}")
        encoder = ResNetEncoder(feature_dim=512, pretrained=True, freeze_backbone=False, model_path=resnet_model)
    else:
        encoder = ResNetEncoder(feature_dim=512, pretrained=True, freeze_backbone=True)
    
    # Initialize Lane Detector (if needed)
    lane_detector = None
    if use_lane_features:
        logger.info("Initializing Lane Detector...")
        try:
            if lane_model_path and Path(lane_model_path).exists():
                logger.info(f"✅ Using U-Net lane detector: {lane_model_path}")
                lane_detector = LaneDetector(model_path=lane_model_path, use_carla=False)
            else:
                logger.info("✅ Using CARLA lane detection (no U-Net model)")
                lane_detector = LaneDetector(use_carla=True)
        except Exception as e:
            logger.warning(f"Failed to initialize lane detector: {e}, continuing without lane features")
            use_lane_features = False
    
    # Extract features
    features_list = []
    valid_indices = []
    
    logger.info(f"Extracting features (ResNet: 512, Lane: {128 if use_lane_features else 0})...")
    logger.info(f"Total images: {len(df)}")
    logger.info(f"Batch size: {batch_size} (for GPU efficiency)")
    
    # Progress tracking
    save_interval = 1000  # Save every 1000 images
    last_save_count = 0
    
    # Process in batches for better GPU utilization
    import torch
    from torch.utils.data import Dataset, DataLoader
    
    class ImageDataset(Dataset):
        def __init__(self, df, data_dir):
            self.df = df
            self.data_dir = data_dir
            self.valid_indices = []
            for idx, row in df.iterrows():
                image_path = data_dir / row['image_path']
                if image_path.exists():
                    self.valid_indices.append(idx)
        
        def __len__(self):
            return len(self.valid_indices)
        
        def __getitem__(self, i):
            idx = self.valid_indices[i]
            row = self.df.iloc[idx]
            image_path = self.data_dir / row['image_path']
            image = cv2.imread(str(image_path))
            if image is not None:
                image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
                # Return as numpy array (H, W, 3)
                return idx, image.astype(np.uint8)
            return idx, None
    
    dataset = ImageDataset(df, data_dir)
    # Custom collate function to handle tuple returns
    def collate_fn(batch):
        """Custom collate function for (idx, image) tuples."""
        indices = []
        images = []
        for idx, img in batch:
            if img is not None:
                indices.append(idx)
                images.append(img)
        return indices, images
    
    # Use num_workers=0 to avoid multiprocessing issues with custom collate
    dataloader = DataLoader(dataset, batch_size=batch_size, shuffle=False, num_workers=0, collate_fn=collate_fn)
    
    logger.info(f"Processing {len(dataset)} valid images in batches of {batch_size}")
    
    import torch
    from torchvision import transforms
    from PIL import Image
    
    # Transform for batch processing
    transform = transforms.Compose([
        transforms.ToPILImage(),
        transforms.Resize((224, 224)),
        transforms.ToTensor(),
        transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
    ])
    
    device = encoder.device
    
    # Use tqdm with file output for logging
    import sys
    tqdm_file = sys.stdout if logger.handlers else None
    
    logger.info("Starting feature extraction loop...")
    batch_count = 0
    
    for batch_idx, batch_data in enumerate(tqdm(dataloader, desc="Extracting features", file=tqdm_file, mininterval=1.0)):
        batch_count += 1
        if batch_idx == 0:
            logger.info(f"First batch received: type={type(batch_data)}, len={len(batch_data) if isinstance(batch_data, (list, tuple)) else 'N/A'}")
        
        # Handle batch data format
        if isinstance(batch_data, (list, tuple)) and len(batch_data) == 2:
            indices_batch, images_batch = batch_data
        else:
            logger.error(f"Unexpected batch format: {type(batch_data)}")
            continue
        
        # Custom collate_fn returns (indices, images) as lists
        valid_batch = []
        valid_indices_batch = []
        
        if batch_idx == 0:
            logger.info(f"Batch 0: indices type={type(indices_batch)}, len={len(indices_batch) if isinstance(indices_batch, (list, tuple)) else 'N/A'}")
            logger.info(f"Batch 0: images type={type(images_batch)}, len={len(images_batch) if isinstance(images_batch, (list, tuple)) else 'N/A'}")
        
        for i in range(len(images_batch)):
            img = images_batch[i]
            idx = indices_batch[i] if i < len(indices_batch) else None
            
            # Images are already numpy arrays from collate_fn
            if img is not None and isinstance(img, np.ndarray) and len(img.shape) == 3 and img.shape[2] == 3:
                valid_batch.append(img)
                valid_indices_batch.append(idx)
        
        if len(valid_batch) == 0:
            if batch_idx == 0:
                images_len = len(images_batch) if isinstance(images_batch, (list, tuple)) else (images_batch.shape[0] if hasattr(images_batch, 'shape') else 0)
                logger.warning(f"First batch has no valid images. Batch format: indices={type(indices_batch)}, images={type(images_batch)}, len={images_len}")
            continue
        
        if batch_idx == 0:
            logger.info(f"Batch 0: {len(valid_batch)} valid images found, starting processing...")
        
        try:
            if batch_idx == 0:
                logger.info(f"Processing batch {batch_idx}: {len(valid_batch)} images")
            
            # Process ResNet features in batch (GPU)
            batch_images = []
            for img in valid_batch:
                # img is numpy array (H, W, 3) uint8
                try:
                    img_tensor = transform(img)
                    batch_images.append(img_tensor)
                except Exception as e:
                    logger.warning(f"Transform error: {e}")
                    continue
            
            if len(batch_images) == 0:
                if batch_idx == 0:
                    logger.warning("No images after transform")
                continue
            
            if batch_idx == 0:
                logger.info(f"Stacking {len(batch_images)} tensors...")
                
            batch_tensor = torch.stack(batch_images).to(device)
            
            if batch_idx == 0:
                logger.info(f"Running ResNet forward pass on batch shape: {batch_tensor.shape}...")
            
            # Batch forward pass
            with torch.no_grad():
                encoder.eval()
                resnet_features = encoder.forward(batch_tensor).cpu().numpy()
            
            if batch_idx == 0:
                logger.info(f"ResNet features extracted: shape={resnet_features.shape}")
            
            # Process lane features individually (U-Net)
            for i, (idx, img) in enumerate(zip(valid_indices_batch, valid_batch)):
                resnet_feature = resnet_features[i]
                
                # Validate ResNet feature
                if resnet_feature is None or resnet_feature.shape[0] != 512:
                    continue
                if np.any(np.isnan(resnet_feature)) or np.any(np.isinf(resnet_feature)):
                    continue
                
                # Extract lane features (if enabled)
                if use_lane_features and lane_detector is not None:
                    try:
                        lane_result = lane_detector.detect_lanes(img)
                        if isinstance(lane_result, tuple):
                            lane_mask, lane_features = lane_result
                        else:
                            lane_mask = lane_result
                            lane_features = lane_detector._extract_lane_features(lane_mask)
                        
                        if lane_features is None or len(lane_features) != 128:
                            lane_features = np.zeros(128)
                        if np.any(np.isnan(lane_features)) or np.any(np.isinf(lane_features)):
                            lane_features = np.zeros(128)
                        
                        combined_feature = np.concatenate([resnet_feature, lane_features])
                    except Exception as e:
                        lane_features = np.zeros(128)
                        combined_feature = np.concatenate([resnet_feature, lane_features])
                else:
                    combined_feature = resnet_feature
                
                # Validate combined feature
                expected_dim = 640 if use_lane_features else 512
                if combined_feature.shape[0] != expected_dim:
                    continue
                if np.any(np.isnan(combined_feature)) or np.any(np.isinf(combined_feature)):
                    continue
                
                features_list.append(combined_feature)
                valid_indices.append(idx)
            
            # Periodic save and logging
            if len(features_list) >= save_interval and len(features_list) > last_save_count:
                current_count = len(features_list)
                logger.info(f"Progress: {current_count}/{len(dataset)} images processed ({current_count*100/len(dataset):.1f}%)")
                temp_features = np.array(features_list)
                temp_path = output_path.parent / f"{output_path.stem}_temp.npy"
                np.save(temp_path, temp_features)
                logger.info(f"💾 Saved intermediate features: {temp_path} ({temp_features.shape})")
                last_save_count = current_count
            
        except Exception as e:
            logger.warning(f"Error processing batch {batch_idx}: {e}")
            continue
    
    # Save features
    if len(features_list) == 0:
        logger.error(f"❌ No features extracted! Check batch processing logic.")
        return
    
    features_array = np.array(features_list)
    np.save(output_path, features_array)
    logger.info(f"✅ Saved {len(features_list)} features to {output_path}")
    logger.info(f"   Feature shape: {features_array.shape}")
    if len(features_array.shape) > 1:
        if use_lane_features:
            logger.info(f"   Combined: ResNet (512) + Lane (128) = {features_array.shape[1]}")
        else:
            logger.info(f"   ResNet only: {features_array.shape[1]}")
    
    # Save valid indices
    valid_indices_path = output_path.parent / "valid_indices.npy"
    np.save(valid_indices_path, np.array(valid_indices))
    logger.info(f"✅ Saved valid indices to {valid_indices_path}")
    
    # Update data.csv with valid indices
    df_valid = df.iloc[valid_indices].reset_index(drop=True)
    df_valid_path = data_dir / "data_valid.csv"
    df_valid.to_csv(df_valid_path, index=False)
    logger.info(f"✅ Saved valid data to {df_valid_path}")


def main():
    parser = argparse.ArgumentParser(description='Extract features from images (ResNet + Lane)')
    parser.add_argument('--data-dir', type=str, required=True, help='Directory containing data.csv and images/')
    parser.add_argument('--output', type=str, default=None, help='Output path for features.npy')
    parser.add_argument('--batch-size', type=int, default=32, help='Batch size (not used yet)')
    parser.add_argument('--preprocess', action='store_true', help='Preprocess data before extraction')
    parser.add_argument('--resnet-model', type=str, default=None, help='Path to fine-tuned ResNet model')
    parser.add_argument('--use-lane-features', action='store_true', default=True, help='Include lane features (default: True)')
    parser.add_argument('--no-lane-features', dest='use_lane_features', action='store_false', help='Disable lane features')
    parser.add_argument('--lane-model', type=str, default=None, help='Path to U-Net lane detection model (uses CARLA if not provided)')
    
    args = parser.parse_args()
    
    extract_features(
        data_dir=args.data_dir,
        output_path=args.output,
        batch_size=args.batch_size,
        preprocess=args.preprocess,
        resnet_model=args.resnet_model,
        use_lane_features=args.use_lane_features,
        lane_model_path=args.lane_model
    )
    
    return 0


if __name__ == '__main__':
    sys.exit(main())

