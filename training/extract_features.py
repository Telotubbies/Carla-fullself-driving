"""
Extract features from images using ResNet.

STEP 2: image → ResNet → feature vector
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
from training.data_preprocessing import preprocess_dataset

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def extract_features(data_dir: str, output_path: str = None, batch_size: int = 32, preprocess: bool = True, resnet_model: str = None):
    """
    Extract features from images.
    
    Args:
        data_dir: Directory containing data.csv and images/
        output_path: Output path for features.npy (default: data_dir/features.npy)
        batch_size: Batch size for processing
        preprocess: Whether to preprocess data first
        resnet_model: Path to fine-tuned ResNet model (optional)
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
    
    # Extract features
    features_list = []
    valid_indices = []
    
    logger.info("Extracting features...")
    for idx, row in tqdm(df.iterrows(), total=len(df)):
        image_path = data_dir / row['image_path']
        
        if not image_path.exists():
            logger.warning(f"Image not found: {image_path}")
            continue
        
        try:
            # Load image
            image = cv2.imread(str(image_path))
            if image is None:
                continue
            
            # Convert BGR to RGB
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            
            # Extract feature
            feature = encoder.encode(image)
            
            # Validate feature
            if feature is None or feature.shape[0] != 512:
                logger.warning(f"Invalid feature for {image_path}")
                continue
            if np.any(np.isnan(feature)) or np.any(np.isinf(feature)):
                logger.warning(f"Feature contains NaN/Inf for {image_path}")
                continue
            
            features_list.append(feature)
            valid_indices.append(idx)
            
        except Exception as e:
            logger.warning(f"Error processing {image_path}: {e}")
            continue
    
    # Save features
    features_array = np.array(features_list)
    np.save(output_path, features_array)
    logger.info(f"✅ Saved {len(features_list)} features to {output_path}")
    logger.info(f"   Feature shape: {features_array.shape}")
    
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
    parser = argparse.ArgumentParser(description='Extract features from images')
    parser.add_argument('--data-dir', type=str, required=True, help='Directory containing data.csv and images/')
    parser.add_argument('--output', type=str, default=None, help='Output path for features.npy')
    parser.add_argument('--batch-size', type=int, default=32, help='Batch size (not used yet)')
    parser.add_argument('--preprocess', action='store_true', help='Preprocess data before extraction')
    parser.add_argument('--resnet-model', type=str, default=None, help='Path to fine-tuned ResNet model')
    
    args = parser.parse_args()
    
    extract_features(
        data_dir=args.data_dir,
        output_path=args.output,
        batch_size=args.batch_size,
        preprocess=args.preprocess,
        resnet_model=args.resnet_model
    )
    
    return 0


if __name__ == '__main__':
    sys.exit(main())

