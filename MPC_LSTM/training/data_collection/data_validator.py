"""
Data Validation for Collected Data.

Validates vehicle state and images before saving.
Implements STEP 1.3 requirements:
- Check missing frames
- Check synchronization
- Check corrupted files
- Print dataset statistics
"""

import logging
import numpy as np
import cv2
from typing import Dict, Any, List, Tuple, Optional
from pathlib import Path
import pandas as pd
from datetime import datetime

logger = logging.getLogger(__name__)


class DataValidator:
    """Validates collected data with comprehensive checks."""
    
    @staticmethod
    def validate_vehicle_state(vehicle_state: Dict[str, Any]) -> bool:
        """
        Validate vehicle state.
        
        Args:
            vehicle_state: Dictionary with x, y, yaw, velocity
        
        Returns:
            True if valid
        """
        required_keys = ['x', 'y', 'yaw', 'velocity']
        if not all(key in vehicle_state for key in required_keys):
            return False
        
        for key in required_keys:
            value = vehicle_state[key]
            if not isinstance(value, (int, float)) or np.isnan(value) or np.isinf(value):
                return False
        
        # Check reasonable ranges
        if abs(vehicle_state['x']) > 10000 or abs(vehicle_state['y']) > 10000:
            return False
        if abs(vehicle_state['yaw']) > 360:
            return False
        if vehicle_state['velocity'] < 0 or vehicle_state['velocity'] > 100:  # m/s
            return False
        
        return True
    
    @staticmethod
    def validate_image(image: np.ndarray) -> bool:
        """
        Validate camera image.
        
        Args:
            image: Image array (H, W, 3)
        
        Returns:
            True if valid
        """
        if image is None:
            return False
        if not isinstance(image, np.ndarray):
            return False
        if len(image.shape) != 3 or image.shape[2] != 3:
            return False
        if image.shape[0] == 0 or image.shape[1] == 0:
            return False
        if np.any(np.isnan(image)) or np.any(np.isinf(image)):
            return False
        return True
    
    @staticmethod
    def check_corrupted_file(file_path: Path) -> bool:
        """
        Check if image file is corrupted.
        
        Args:
            file_path: Path to image file
        
        Returns:
            True if file is valid (not corrupted)
        """
        try:
            if not file_path.exists():
                return False
            
            # Try to read image
            img = cv2.imread(str(file_path))
            if img is None:
                return False
            
            # Check if image can be decoded properly
            img_array = np.asarray(img)
            if img_array.size == 0:
                return False
            
            return True
        except Exception as e:
            logger.debug(f"File corruption check failed for {file_path}: {e}")
            return False
    
    @staticmethod
    def check_missing_frames(
        data_dir: Path,
        expected_start: int = 0,
        expected_end: Optional[int] = None
    ) -> List[int]:
        """
        Check for missing frames in dataset.
        
        Args:
            data_dir: Data directory containing images
            expected_start: Expected starting frame number
            expected_end: Expected ending frame number (None = check all)
        
        Returns:
            List of missing frame numbers
        """
        images_dir = data_dir / "images"
        if not images_dir.exists():
            logger.warning(f"Images directory not found: {images_dir}")
            return []
        
        # Get all image files
        image_files = sorted(images_dir.glob("image_*.png"))
        if len(image_files) == 0:
            logger.warning("No image files found")
            return []
        
        # Extract frame numbers
        frame_numbers = []
        for img_file in image_files:
            try:
                # Extract number from filename like "image_000123.png"
                frame_num = int(img_file.stem.split('_')[1])
                frame_numbers.append(frame_num)
            except (ValueError, IndexError):
                continue
        
        if len(frame_numbers) == 0:
            return []
        
        # Determine expected range
        if expected_end is None:
            expected_end = max(frame_numbers)
        
        # Find missing frames
        expected_frames = set(range(expected_start, expected_end + 1))
        actual_frames = set(frame_numbers)
        missing_frames = sorted(list(expected_frames - actual_frames))
        
        return missing_frames
    
    @staticmethod
    def check_synchronization(
        data_dir: Path,
        csv_path: Optional[Path] = None
    ) -> Dict[str, Any]:
        """
        Check synchronization between images and CSV data.
        
        Args:
            data_dir: Data directory
            csv_path: Path to CSV file (default: data_dir/data.csv)
        
        Returns:
            Dictionary with synchronization results
        """
        if csv_path is None:
            csv_path = data_dir / "data.csv"
        
        if not csv_path.exists():
            logger.warning(f"CSV file not found: {csv_path}")
            return {
                'synchronized': False,
                'image_count': 0,
                'csv_rows': 0,
                'mismatches': []
            }
        
        # Load CSV
        try:
            df = pd.read_csv(csv_path)
        except Exception as e:
            logger.error(f"Failed to load CSV: {e}")
            return {
                'synchronized': False,
                'error': str(e)
            }
        
        # Get image files
        images_dir = data_dir / "images"
        image_files = sorted(images_dir.glob("image_*.png")) if images_dir.exists() else []
        
        # Check synchronization
        csv_rows = len(df)
        image_count = len(image_files)
        
        mismatches = []
        
        # Check if CSV has image_path column
        if 'image_path' in df.columns:
            # Check if all CSV rows have corresponding images
            for idx, row in df.iterrows():
                if pd.isna(row.get('image_path', '')):
                    continue
                img_path = data_dir / row['image_path']
                if not img_path.exists():
                    mismatches.append({
                        'csv_row': idx,
                        'expected_image': row['image_path'],
                        'status': 'missing'
                    })
        else:
            # Check by frame number
            for idx in range(min(csv_rows, image_count)):
                expected_img = images_dir / f"image_{idx:06d}.png"
                if not expected_img.exists():
                    mismatches.append({
                        'csv_row': idx,
                        'expected_image': f"image_{idx:06d}.png",
                        'status': 'missing'
                    })
        
        synchronized = (csv_rows == image_count) and (len(mismatches) == 0)
        
        return {
            'synchronized': synchronized,
            'image_count': image_count,
            'csv_rows': csv_rows,
            'mismatches': mismatches[:10],  # First 10 mismatches
            'mismatch_count': len(mismatches)
        }
    
    @staticmethod
    def generate_statistics(data_dir: Path) -> Dict[str, Any]:
        """
        Generate dataset statistics.
        
        Implements STEP 1.3 requirement: Print dataset statistics.
        
        Args:
            data_dir: Data directory
        
        Returns:
            Dictionary with dataset statistics
        """
        stats = {
            'data_dir': str(data_dir),
            'timestamp': datetime.now().isoformat(),
            'images': {},
            'data': {},
            'synchronization': {},
            'corruption': {}
        }
        
        # Image statistics
        images_dir = data_dir / "images"
        if images_dir.exists():
            image_files = list(images_dir.glob("*.png"))
            stats['images']['count'] = len(image_files)
            
            if len(image_files) > 0:
                # Check first image for dimensions
                try:
                    img = cv2.imread(str(image_files[0]))
                    if img is not None:
                        stats['images']['shape'] = list(img.shape)
                        stats['images']['dtype'] = str(img.dtype)
                except Exception as e:
                    logger.warning(f"Failed to read sample image: {e}")
                
                # Check for corrupted files
                corrupted_count = 0
                for img_file in image_files[:100]:  # Sample first 100
                    if not DataValidator.check_corrupted_file(img_file):
                        corrupted_count += 1
                
                stats['corruption']['sampled_files'] = min(100, len(image_files))
                stats['corruption']['corrupted_count'] = corrupted_count
                stats['corruption']['corruption_rate'] = corrupted_count / min(100, len(image_files)) if len(image_files) > 0 else 0.0
        else:
            stats['images']['count'] = 0
        
        # CSV data statistics
        csv_path = data_dir / "data.csv"
        if csv_path.exists():
            try:
                df = pd.read_csv(csv_path)
                stats['data']['rows'] = len(df)
                stats['data']['columns'] = list(df.columns)
                
                # Statistics for numeric columns
                numeric_cols = ['x', 'y', 'yaw', 'velocity', 'steering', 'throttle', 'brake']
                for col in numeric_cols:
                    if col in df.columns:
                        stats['data'][col] = {
                            'mean': float(df[col].mean()) if df[col].dtype in ['float64', 'int64'] else None,
                            'std': float(df[col].std()) if df[col].dtype in ['float64', 'int64'] else None,
                            'min': float(df[col].min()) if df[col].dtype in ['float64', 'int64'] else None,
                            'max': float(df[col].max()) if df[col].dtype in ['float64', 'int64'] else None,
                            'null_count': int(df[col].isna().sum())
                        }
            except Exception as e:
                logger.error(f"Failed to load CSV for statistics: {e}")
                stats['data']['error'] = str(e)
        else:
            stats['data']['rows'] = 0
        
        # Synchronization check
        sync_result = DataValidator.check_synchronization(data_dir, csv_path)
        stats['synchronization'] = sync_result
        
        # Missing frames check
        missing_frames = DataValidator.check_missing_frames(data_dir)
        stats['missing_frames'] = {
            'count': len(missing_frames),
            'frames': missing_frames[:20]  # First 20 missing frames
        }
        
        return stats
    
    @staticmethod
    def print_statistics(data_dir: Path) -> None:
        """
        Print dataset statistics to console.
        
        Args:
            data_dir: Data directory
        """
        stats = DataValidator.generate_statistics(data_dir)
        
        logger.info("=" * 60)
        logger.info("DATASET STATISTICS")
        logger.info("=" * 60)
        logger.info(f"Data Directory: {stats['data_dir']}")
        logger.info(f"Timestamp: {stats['timestamp']}")
        logger.info("")
        
        # Images
        logger.info("IMAGES:")
        logger.info(f"  Count: {stats['images'].get('count', 0)}")
        if 'shape' in stats['images']:
            logger.info(f"  Shape: {stats['images']['shape']}")
        if 'corruption' in stats:
            logger.info(f"  Corruption Rate: {stats['corruption'].get('corruption_rate', 0.0):.2%}")
        logger.info("")
        
        # Data
        logger.info("DATA (CSV):")
        logger.info(f"  Rows: {stats['data'].get('rows', 0)}")
        logger.info(f"  Columns: {stats['data'].get('columns', [])}")
        
        # Numeric statistics
        numeric_cols = ['x', 'y', 'yaw', 'velocity', 'steering', 'throttle', 'brake']
        for col in numeric_cols:
            if col in stats['data']:
                col_stats = stats['data'][col]
                logger.info(f"  {col.upper()}:")
                if col_stats.get('mean') is not None:
                    logger.info(f"    Mean: {col_stats['mean']:.3f}")
                    logger.info(f"    Std:  {col_stats['std']:.3f}")
                    logger.info(f"    Min:  {col_stats['min']:.3f}")
                    logger.info(f"    Max:  {col_stats['max']:.3f}")
                logger.info(f"    Null: {col_stats['null_count']}")
        logger.info("")
        
        # Synchronization
        logger.info("SYNCHRONIZATION:")
        sync = stats['synchronization']
        logger.info(f"  Synchronized: {sync.get('synchronized', False)}")
        logger.info(f"  Image Count: {sync.get('image_count', 0)}")
        logger.info(f"  CSV Rows: {sync.get('csv_rows', 0)}")
        logger.info(f"  Mismatches: {sync.get('mismatch_count', 0)}")
        logger.info("")
        
        # Missing frames
        logger.info("MISSING FRAMES:")
        missing = stats['missing_frames']
        logger.info(f"  Count: {missing['count']}")
        if missing['count'] > 0:
            logger.info(f"  First 20: {missing['frames']}")
        logger.info("")
        
        logger.info("=" * 60)
