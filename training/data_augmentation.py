"""
Data Augmentation for Steering Diversity.

Based on research papers:
- "End-to-End Learning for Self-Driving Cars" (Bojarski et al., 2016)
- "Learning to Drive in a Day" (Kendall et al., 2019)

Augmentation strategies:
1. Horizontal flip (mirror images and negate steering)
2. Brightness/contrast adjustment
3. Steering angle augmentation (add noise)
4. Balancing steering distribution
"""

import numpy as np
import pandas as pd
import cv2
from pathlib import Path
import logging
from typing import Tuple, Dict, Any
import argparse
from PIL import Image, ImageEnhance
import random

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class DataAugmenter:
    """Augment data to increase steering diversity."""
    
    def __init__(self):
        """Initialize augmenter."""
        self.augmentation_stats = {
            'flipped': 0,
            'brightness': 0,
            'contrast': 0,
            'steering_noise': 0,
        }
    
    def flip_image(self, image: np.ndarray) -> np.ndarray:
        """Horizontal flip image."""
        return cv2.flip(image, 1)
    
    def adjust_brightness(self, image: np.ndarray, factor: float = None) -> np.ndarray:
        """
        Adjust image brightness.
        
        Args:
            image: Input image (H, W, 3)
            factor: Brightness factor (1.0 = no change, <1.0 = darker, >1.0 = brighter)
        """
        if factor is None:
            factor = random.uniform(0.7, 1.3)
        
        pil_image = Image.fromarray(image)
        enhancer = ImageEnhance.Brightness(pil_image)
        enhanced = enhancer.enhance(factor)
        return np.array(enhanced)
    
    def adjust_contrast(self, image: np.ndarray, factor: float = None) -> np.ndarray:
        """
        Adjust image contrast.
        
        Args:
            image: Input image (H, W, 3)
            factor: Contrast factor (1.0 = no change, <1.0 = less contrast, >1.0 = more contrast)
        """
        if factor is None:
            factor = random.uniform(0.8, 1.2)
        
        pil_image = Image.fromarray(image)
        enhancer = ImageEnhance.Contrast(pil_image)
        enhanced = enhancer.enhance(factor)
        return np.array(enhanced)
    
    def add_steering_noise(self, steering: float, noise_std: float = 0.05) -> float:
        """
        Add noise to steering angle.
        
        Args:
            steering: Original steering angle
            noise_std: Standard deviation of noise
        """
        noise = np.random.normal(0, noise_std)
        new_steering = steering + noise
        return np.clip(new_steering, -1.0, 1.0)
    
    def augment_sample(
        self,
        image: np.ndarray,
        steering: float,
        apply_flip: bool = True,
        apply_brightness: bool = True,
        apply_contrast: bool = True,
        apply_steering_noise: bool = True
    ) -> Tuple[np.ndarray, float]:
        """
        Augment a single sample.
        
        Args:
            image: Input image
            steering: Steering angle
            apply_flip: Whether to apply horizontal flip
            apply_brightness: Whether to adjust brightness
            apply_contrast: Whether to adjust contrast
            apply_steering_noise: Whether to add steering noise
        
        Returns:
            Augmented image and steering
        """
        aug_image = image.copy()
        aug_steering = steering
        
        # Horizontal flip (mirror and negate steering)
        if apply_flip and random.random() < 0.5:
            aug_image = self.flip_image(aug_image)
            aug_steering = -aug_steering
            self.augmentation_stats['flipped'] += 1
        
        # Brightness adjustment
        if apply_brightness and random.random() < 0.3:
            aug_image = self.adjust_brightness(aug_image)
            self.augmentation_stats['brightness'] += 1
        
        # Contrast adjustment
        if apply_contrast and random.random() < 0.3:
            aug_image = self.adjust_contrast(aug_image)
            self.augmentation_stats['contrast'] += 1
        
        # Steering noise
        if apply_steering_noise and random.random() < 0.2:
            aug_steering = self.add_steering_noise(aug_steering)
            self.augmentation_stats['steering_noise'] += 1
        
        return aug_image, aug_steering
    
    def balance_steering_distribution(
        self,
        df: pd.DataFrame,
        target_distribution: Dict[str, float] = None
    ) -> pd.DataFrame:
        """
        Balance steering distribution by oversampling underrepresented angles.
        
        Args:
            df: Input dataframe
            target_distribution: Target distribution (e.g., {'left': 0.3, 'right': 0.3, 'straight': 0.4})
        """
        if target_distribution is None:
            target_distribution = {
                'left': 0.3,      # steering < -0.1
                'right': 0.3,    # steering > 0.1
                'straight': 0.4   # -0.1 <= steering <= 0.1
            }
        
        # Classify samples
        df['steering_class'] = 'straight'
        df.loc[df['steering'] < -0.1, 'steering_class'] = 'left'
        df.loc[df['steering'] > 0.1, 'steering_class'] = 'right'
        
        # Count current distribution
        class_counts = df['steering_class'].value_counts()
        total = len(df)
        current_dist = {
            'left': class_counts.get('left', 0) / total,
            'right': class_counts.get('right', 0) / total,
            'straight': class_counts.get('straight', 0) / total
        }
        
        logger.info(f"Current distribution: {current_dist}")
        logger.info(f"Target distribution: {target_distribution}")
        
        # Calculate oversampling factors
        oversample_factors = {}
        for cls in ['left', 'right', 'straight']:
            if current_dist[cls] > 0:
                oversample_factors[cls] = target_distribution[cls] / current_dist[cls]
            else:
                oversample_factors[cls] = 10.0  # Heavy oversampling if missing
        
        # Oversample
        augmented_samples = []
        for cls in ['left', 'right', 'straight']:
            cls_df = df[df['steering_class'] == cls].copy()
            if len(cls_df) == 0:
                continue
            
            num_samples = int(len(cls_df) * oversample_factors[cls])
            if num_samples > len(cls_df):
                # Oversample with replacement
                oversampled = cls_df.sample(n=num_samples, replace=True, random_state=42)
            else:
                oversampled = cls_df
            
            augmented_samples.append(oversampled)
        
        balanced_df = pd.concat(augmented_samples, ignore_index=True)
        balanced_df = balanced_df.sample(frac=1, random_state=42).reset_index(drop=True)  # Shuffle
        
        logger.info(f"Balanced dataset: {len(balanced_df)} samples")
        logger.info(f"New distribution: {balanced_df['steering_class'].value_counts(normalize=True).to_dict()}")
        
        return balanced_df.drop('steering_class', axis=1)


def augment_dataset(
    data_dir: str,
    output_dir: str = None,
    augmentation_factor: float = 2.0,
    balance_steering: bool = True
) -> None:
    """
    Augment entire dataset.
    
    Args:
        data_dir: Input data directory
        output_dir: Output directory (default: data_dir + '_augmented')
        augmentation_factor: How many times to augment (2.0 = double the dataset)
        balance_steering: Whether to balance steering distribution
    """
    data_dir = Path(data_dir)
    
    if output_dir is None:
        output_dir = data_dir.parent / f"{data_dir.name}_augmented"
    else:
        output_dir = Path(output_dir)
    
    output_dir.mkdir(exist_ok=True)
    output_images_dir = output_dir / "images"
    output_images_dir.mkdir(exist_ok=True)
    
    # Load data
    csv_path = data_dir / "data.csv"
    if not csv_path.exists():
        raise FileNotFoundError(f"data.csv not found in {data_dir}")
    
    df = pd.read_csv(csv_path)
    logger.info(f"Loaded {len(df)} samples from {csv_path}")
    
    # Balance steering if requested
    if balance_steering:
        augmenter = DataAugmenter()
        df = augmenter.balance_steering_distribution(df)
    
    # Augment samples
    augmenter = DataAugmenter()
    augmented_data = []
    
    num_augmented = int(len(df) * augmentation_factor)
    logger.info(f"Augmenting {len(df)} samples to {num_augmented} samples...")
    
    for idx, row in df.iterrows():
        # Load original image
        image_path = data_dir / row['image_path']
        if not image_path.exists():
            logger.warning(f"Image not found: {image_path}")
            continue
        
        image = cv2.imread(str(image_path))
        if image is None:
            logger.warning(f"Failed to load image: {image_path}")
            continue
        
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        steering = row['steering']
        
        # Original sample
        output_image_path = output_images_dir / f"image_{len(augmented_data):06d}.png"
        cv2.imwrite(str(output_image_path), cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
        
        augmented_data.append({
            'step': len(augmented_data),
            'image_path': f"images/{output_image_path.name}",
            'x': row['x'],
            'y': row['y'],
            'yaw': row['yaw'],
            'velocity': row['velocity'],
            'steering': steering,
            'throttle': row['throttle'],
            'brake': row['brake'],
        })
        
        # Augmented samples
        num_aug = int(augmentation_factor) - 1
        for _ in range(num_aug):
            aug_image, aug_steering = augmenter.augment_sample(image, steering)
            
            output_image_path = output_images_dir / f"image_{len(augmented_data):06d}.png"
            cv2.imwrite(str(output_image_path), cv2.cvtColor(aug_image, cv2.COLOR_RGB2BGR))
            
            augmented_data.append({
                'step': len(augmented_data),
                'image_path': f"images/{output_image_path.name}",
                'x': row['x'],
                'y': row['y'],
                'yaw': row['yaw'],
                'velocity': row['velocity'],
                'steering': aug_steering,
                'throttle': row['throttle'],
                'brake': row['brake'],
            })
        
        if (idx + 1) % 100 == 0:
            logger.info(f"Augmented {idx + 1}/{len(df)} samples...")
    
    # Save augmented data
    aug_df = pd.DataFrame(augmented_data)
    output_csv = output_dir / "data.csv"
    aug_df.to_csv(output_csv, index=False)
    
    logger.info("=" * 60)
    logger.info("✅ Augmentation complete!")
    logger.info(f"   Original samples: {len(df)}")
    logger.info(f"   Augmented samples: {len(aug_df)}")
    logger.info(f"   Output directory: {output_dir}")
    logger.info("")
    logger.info("📊 Augmentation Statistics:")
    for key, value in augmenter.augmentation_stats.items():
        logger.info(f"   {key}: {value}")
    logger.info("=" * 60)


def main():
    parser = argparse.ArgumentParser(description='Augment dataset for steering diversity')
    parser.add_argument('--data-dir', type=str, required=True, help='Input data directory')
    parser.add_argument('--output-dir', type=str, default=None, help='Output directory')
    parser.add_argument('--factor', type=float, default=2.0, help='Augmentation factor (2.0 = double)')
    parser.add_argument('--balance', action='store_true', help='Balance steering distribution')
    
    args = parser.parse_args()
    
    augment_dataset(
        data_dir=args.data_dir,
        output_dir=args.output_dir,
        augmentation_factor=args.factor,
        balance_steering=args.balance
    )
    
    return 0


if __name__ == '__main__':
    import sys
    sys.exit(main())

