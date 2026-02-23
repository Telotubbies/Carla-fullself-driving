"""
UNet Validation Module.

Implements STEP 2.4 requirements:
- IoU (Intersection over Union) calculation
- Pixel accuracy calculation
- Inference speed measurement
"""

import logging
import numpy as np
import torch
import time
from typing import Dict, Any, List, Tuple, Optional
from pathlib import Path
import cv2
from tqdm import tqdm

logger = logging.getLogger(__name__)


class UNetValidator:
    """Validates UNet model performance."""
    
    @staticmethod
    def calculate_iou(pred_mask: np.ndarray, gt_mask: np.ndarray) -> float:
        """
        Calculate Intersection over Union (IoU).
        
        Args:
            pred_mask: Predicted binary mask (H, W) 0 or 255
            gt_mask: Ground truth binary mask (H, W) 0 or 255
        
        Returns:
            IoU score (0.0 to 1.0)
        """
        # Convert to binary
        pred_binary = (pred_mask > 127).astype(np.uint8)
        gt_binary = (gt_mask > 127).astype(np.uint8)
        
        # Calculate intersection and union
        intersection = np.logical_and(pred_binary, gt_binary).sum()
        union = np.logical_or(pred_binary, gt_binary).sum()
        
        if union == 0:
            return 1.0 if intersection == 0 else 0.0
        
        iou = intersection / union
        return float(iou)
    
    @staticmethod
    def calculate_pixel_accuracy(pred_mask: np.ndarray, gt_mask: np.ndarray) -> float:
        """
        Calculate pixel accuracy.
        
        Args:
            pred_mask: Predicted binary mask (H, W) 0 or 255
            gt_mask: Ground truth binary mask (H, W) 0 or 255
        
        Returns:
            Pixel accuracy (0.0 to 1.0)
        """
        # Convert to binary
        pred_binary = (pred_mask > 127).astype(np.uint8)
        gt_binary = (gt_mask > 127).astype(np.uint8)
        
        # Calculate correct pixels
        correct = (pred_binary == gt_binary).sum()
        total = pred_binary.size
        
        accuracy = correct / total if total > 0 else 0.0
        return float(accuracy)
    
    @staticmethod
    def measure_inference_speed(
        model,
        input_size: Tuple[int, int] = (256, 256),
        num_runs: int = 100,
        warmup: int = 10
    ) -> Dict[str, float]:
        """
        Measure inference speed of UNet model.
        
        Args:
            model: UNet model
            input_size: Input image size (H, W)
            num_runs: Number of inference runs
            warmup: Number of warmup runs
        
        Returns:
            Dictionary with timing statistics
        """
        device = next(model.parameters()).device
        
        # Create dummy input
        dummy_input = torch.randn(1, 3, input_size[0], input_size[1]).to(device)
        
        # Warmup
        model.eval()
        with torch.no_grad():
            for _ in range(warmup):
                _ = model(dummy_input)
        
        # Synchronize GPU if CUDA
        if device.type == 'cuda':
            torch.cuda.synchronize()
        
        # Measure inference time
        times = []
        with torch.no_grad():
            for _ in range(num_runs):
                if device.type == 'cuda':
                    torch.cuda.synchronize()
                
                start_time = time.time()
                _ = model(dummy_input)
                
                if device.type == 'cuda':
                    torch.cuda.synchronize()
                
                elapsed = time.time() - start_time
                times.append(elapsed)
        
        times = np.array(times)
        
        return {
            'mean_ms': float(times.mean() * 1000),
            'std_ms': float(times.std() * 1000),
            'min_ms': float(times.min() * 1000),
            'max_ms': float(times.max() * 1000),
            'fps': float(1.0 / times.mean()),
            'num_runs': num_runs
        }
    
    @staticmethod
    def validate_on_dataset(
        model,
        images_dir: Path,
        masks_dir: Path,
        device: Optional[torch.device] = None
    ) -> Dict[str, Any]:
        """
        Validate UNet on dataset.
        
        Args:
            model: UNet model
            images_dir: Directory with test images
            masks_dir: Directory with ground truth masks
            device: Device to run on
        
        Returns:
            Dictionary with validation metrics
        """
        if device is None:
            device = next(model.parameters()).device
        
        model.eval()
        
        # Get image files
        image_files = sorted(list(images_dir.glob("*.png")))
        if len(image_files) == 0:
            logger.error(f"No images found in {images_dir}")
            return {}
        
        ious = []
        pixel_accs = []
        inference_times = []
        
        logger.info(f"Validating on {len(image_files)} images...")
        
        with torch.no_grad():
            for img_path in tqdm(image_files, desc="Validating"):
                # Load image
                img = cv2.imread(str(img_path))
                if img is None:
                    continue
                img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                
                # Load ground truth mask
                mask_path = masks_dir / f"{img_path.stem}_lane.png"
                if not mask_path.exists():
                    continue
                gt_mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
                if gt_mask is None:
                    continue
                
                # Preprocess image
                h, w = img_rgb.shape[:2]
                img_tensor = torch.FloatTensor(img_rgb).permute(2, 0, 1).unsqueeze(0) / 255.0
                img_tensor = img_tensor.to(device)
                
                # Resize to model input
                import torch.nn.functional as F
                img_tensor_resized = F.interpolate(
                    img_tensor, size=(256, 256), mode='bilinear', align_corners=False
                )
                
                # Inference
                start_time = time.time()
                output = model(img_tensor_resized)
                if device.type == 'cuda':
                    torch.cuda.synchronize()
                inference_time = time.time() - start_time
                inference_times.append(inference_time)
                
                # Postprocess
                pred_mask = torch.argmax(output, dim=1).squeeze().cpu().numpy()
                pred_mask = cv2.resize(
                    pred_mask.astype(np.uint8),
                    (w, h),
                    interpolation=cv2.INTER_NEAREST
                )
                pred_mask = (pred_mask * 255).astype(np.uint8)
                
                # Calculate metrics
                iou = UNetValidator.calculate_iou(pred_mask, gt_mask)
                pixel_acc = UNetValidator.calculate_pixel_accuracy(pred_mask, gt_mask)
                
                ious.append(iou)
                pixel_accs.append(pixel_acc)
        
        if len(ious) == 0:
            logger.error("No valid samples found for validation")
            return {}
        
        # Calculate statistics
        ious = np.array(ious)
        pixel_accs = np.array(pixel_accs)
        inference_times = np.array(inference_times)
        
        results = {
            'num_samples': len(ious),
            'iou': {
                'mean': float(ious.mean()),
                'std': float(ious.std()),
                'min': float(ious.min()),
                'max': float(ious.max())
            },
            'pixel_accuracy': {
                'mean': float(pixel_accs.mean()),
                'std': float(pixel_accs.std()),
                'min': float(pixel_accs.min()),
                'max': float(pixel_accs.max())
            },
            'inference_speed': {
                'mean_ms': float(inference_times.mean() * 1000),
                'std_ms': float(inference_times.std() * 1000),
                'fps': float(1.0 / inference_times.mean())
            }
        }
        
        return results
    
    @staticmethod
    def print_validation_results(results: Dict[str, Any]) -> None:
        """
        Print validation results.
        
        Args:
            results: Validation results dictionary
        """
        logger.info("=" * 60)
        logger.info("UNET VALIDATION RESULTS")
        logger.info("=" * 60)
        logger.info(f"Number of samples: {results.get('num_samples', 0)}")
        logger.info("")
        
        # IoU
        iou = results.get('iou', {})
        logger.info("IoU (Intersection over Union):")
        logger.info(f"  Mean: {iou.get('mean', 0.0):.4f}")
        logger.info(f"  Std:  {iou.get('std', 0.0):.4f}")
        logger.info(f"  Min:  {iou.get('min', 0.0):.4f}")
        logger.info(f"  Max:  {iou.get('max', 0.0):.4f}")
        logger.info("")
        
        # Pixel Accuracy
        pixel_acc = results.get('pixel_accuracy', {})
        logger.info("Pixel Accuracy:")
        logger.info(f"  Mean: {pixel_acc.get('mean', 0.0):.4f}")
        logger.info(f"  Std:  {pixel_acc.get('std', 0.0):.4f}")
        logger.info(f"  Min:  {pixel_acc.get('min', 0.0):.4f}")
        logger.info(f"  Max:  {pixel_acc.get('max', 0.0):.4f}")
        logger.info("")
        
        # Inference Speed
        speed = results.get('inference_speed', {})
        logger.info("Inference Speed:")
        logger.info(f"  Mean: {speed.get('mean_ms', 0.0):.2f} ms")
        logger.info(f"  Std:  {speed.get('std_ms', 0.0):.2f} ms")
        logger.info(f"  FPS:  {speed.get('fps', 0.0):.2f}")
        logger.info("")
        logger.info("=" * 60)

