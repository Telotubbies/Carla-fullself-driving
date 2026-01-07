"""
Data Augmentation for CARLA RL Training
เพิ่ม robustness และ generalization
"""

import numpy as np
import cv2
from typing import Optional, Tuple
import logging


class DataAugmentation:
    """
    Data Augmentation for vision observations
    """
    
    def __init__(self, config: dict):
        """
        Initialize data augmentation
        
        Args:
            config: Augmentation configuration from yaml
        """
        self.enabled = config.get('enabled', False)
        if not self.enabled:
            return
        
        self.config = config
        self.methods = config.get('methods', [])
        
        # Augmentation probabilities
        self.color_jitter_prob = config.get('color_jitter_prob', 0.5)
        self.gaussian_noise_prob = config.get('gaussian_noise_prob', 0.3)
        self.motion_blur_prob = config.get('motion_blur_prob', 0.3)
        self.random_erasing_prob = config.get('random_erasing_prob', 0.1)
        
        # Augmentation parameters
        self.color_jitter_brightness = config.get('color_jitter_brightness', 0.2)
        self.color_jitter_contrast = config.get('color_jitter_contrast', 0.2)
        self.color_jitter_saturation = config.get('color_jitter_saturation', 0.2)
        self.gaussian_noise_std = config.get('gaussian_noise_std', 0.1)
        self.motion_blur_kernel = config.get('motion_blur_kernel', 5)
        self.random_erasing_area = config.get('random_erasing_area', 0.1)
        
        logging.info(f"✅ Data Augmentation initialized: enabled={self.enabled}, methods={self.methods}")
    
    def augment_image(self, image: np.ndarray) -> np.ndarray:
        """
        Apply augmentations to image
        
        Args:
            image: Image array (H, W, C) in range [0, 1]
        
        Returns:
            Augmented image
        """
        if not self.enabled or len(self.methods) == 0:
            return image
        
        # Convert to uint8 for OpenCV operations
        if image.dtype == np.float32 or image.dtype == np.float64:
            img_uint8 = (image * 255).astype(np.uint8)
        else:
            img_uint8 = image.copy()
        
        # Apply augmentations
        if 'color_jitter' in self.methods and np.random.random() < self.color_jitter_prob:
            img_uint8 = self._color_jitter(img_uint8)
        
        if 'gaussian_noise' in self.methods and np.random.random() < self.gaussian_noise_prob:
            img_uint8 = self._gaussian_noise(img_uint8)
        
        if 'motion_blur' in self.methods and np.random.random() < self.motion_blur_prob:
            img_uint8 = self._motion_blur(img_uint8)
        
        if 'random_erasing' in self.methods and np.random.random() < self.random_erasing_prob:
            img_uint8 = self._random_erasing(img_uint8)
        
        # Convert back to float32 [0, 1]
        if image.dtype == np.float32 or image.dtype == np.float64:
            return (img_uint8.astype(np.float32) / 255.0).clip(0.0, 1.0)
        else:
            return img_uint8
    
    def _color_jitter(self, image: np.ndarray) -> np.ndarray:
        """Apply color jitter (brightness, contrast, saturation)"""
        # Brightness
        brightness_factor = 1.0 + np.random.uniform(-self.color_jitter_brightness, self.color_jitter_brightness)
        image = cv2.convertScaleAbs(image, alpha=1.0, beta=(brightness_factor - 1.0) * 127)
        
        # Contrast
        contrast_factor = 1.0 + np.random.uniform(-self.color_jitter_contrast, self.color_jitter_contrast)
        image = cv2.convertScaleAbs(image, alpha=contrast_factor, beta=0)
        
        # Saturation (convert to HSV)
        if len(image.shape) == 3 and image.shape[2] == 3:
            hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
            saturation_factor = 1.0 + np.random.uniform(-self.color_jitter_saturation, self.color_jitter_saturation)
            hsv[:, :, 1] = cv2.multiply(hsv[:, :, 1], saturation_factor).clip(0, 255)
            image = cv2.cvtColor(hsv, cv2.COLOR_HSV2RGB)
        
        return image
    
    def _gaussian_noise(self, image: np.ndarray) -> np.ndarray:
        """Add Gaussian noise"""
        noise = np.random.normal(0, self.gaussian_noise_std * 255, image.shape).astype(np.int16)
        noisy_image = image.astype(np.int16) + noise
        return noisy_image.clip(0, 255).astype(np.uint8)
    
    def _motion_blur(self, image: np.ndarray) -> np.ndarray:
        """Apply motion blur"""
        kernel_size = self.motion_blur_kernel
        # Create motion blur kernel
        kernel = np.zeros((kernel_size, kernel_size))
        kernel[int((kernel_size-1)/2), :] = np.ones(kernel_size)
        kernel = kernel / kernel_size
        
        # Apply blur
        blurred = cv2.filter2D(image, -1, kernel)
        return blurred
    
    def _random_erasing(self, image: np.ndarray) -> np.ndarray:
        """Apply random erasing (cutout)"""
        h, w = image.shape[:2]
        area = h * w * self.random_erasing_area
        
        # Random rectangle
        h_erase = int(np.sqrt(area * np.random.uniform(0.3, 1.0)))
        w_erase = int(area / h_erase)
        
        if h_erase < h and w_erase < w:
            y = np.random.randint(0, h - h_erase)
            x = np.random.randint(0, w - w_erase)
            
            if len(image.shape) == 3:
                image[y:y+h_erase, x:x+w_erase, :] = 0
            else:
                image[y:y+h_erase, x:x+w_erase] = 0
        
        return image

