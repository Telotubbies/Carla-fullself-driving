"""
Create lane masks from images using edge detection.
Alternative to CARLA-based lane label creation.
"""

import cv2
import numpy as np
from pathlib import Path
from tqdm import tqdm
import argparse
import sys

def create_lane_mask_from_image(image_path: Path, output_path: Path) -> bool:
    """Create lane mask from image using edge detection."""
    try:
        # Read image
        img = cv2.imread(str(image_path))
        if img is None:
            return False
        
        # Convert to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # Apply Gaussian blur
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Canny edge detection
        edges = cv2.Canny(blurred, 50, 150)
        
        # Hough lines for lane detection
        lines = cv2.HoughLinesP(edges, 1, np.pi/180, 50, minLineLength=50, maxLineGap=10)
        
        # Create mask
        mask = np.zeros_like(gray)
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                # Filter horizontal lines (likely lanes)
                if abs(y2 - y1) < abs(x2 - x1):  # More horizontal than vertical
                    cv2.line(mask, (x1, y1), (x2, y2), 255, 3)
        
        # Morphological operations to clean up
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        
        # Save mask
        cv2.imwrite(str(output_path), mask)
        return True
        
    except Exception as e:
        print(f"Error processing {image_path}: {e}")
        return False


def main():
    parser = argparse.ArgumentParser(description='Create lane masks from images')
    parser.add_argument('--images-dir', type=str, required=True, help='Images directory')
    parser.add_argument('--output-dir', type=str, required=True, help='Output masks directory')
    parser.add_argument('--max-images', type=int, default=1000, help='Maximum images to process')
    
    args = parser.parse_args()
    
    images_dir = Path(args.images_dir)
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Get image files
    image_files = sorted(list(images_dir.glob("*.png")))[:args.max_images]
    print(f"Processing {len(image_files)} images...")
    
    success_count = 0
    for img_path in tqdm(image_files, desc="Creating lane masks"):
        mask_path = output_dir / f"{img_path.stem}_lane.png"
        if create_lane_mask_from_image(img_path, mask_path):
            success_count += 1
    
    print(f"✅ Created {success_count}/{len(image_files)} lane masks")
    print(f"   Output: {output_dir}")


if __name__ == '__main__':
    main()

