"""
Data Preprocessing and Cleaning for Training Pipeline.

ตามหลัก Data Analysis:
- Data Cleaning
- Normalization
- Outlier Detection
- Data Validation
"""

import numpy as np
import pandas as pd
from pathlib import Path
import logging
from typing import Tuple, Dict, Any
import json

logger = logging.getLogger(__name__)


class DataPreprocessor:
    """Data preprocessing and cleaning."""
    
    def __init__(self):
        """Initialize preprocessor."""
        self.state_stats = None
        self.control_stats = None
        
    def clean_data(self, df: pd.DataFrame) -> pd.DataFrame:
        """
        Clean data: remove invalid, outliers, etc.
        
        Args:
            df: Input dataframe
            
        Returns:
            Cleaned dataframe
        """
        logger.info(f"Original data: {len(df)} rows")
        
        # Remove rows with missing values
        df_clean = df.dropna()
        logger.info(f"After removing NaN: {len(df_clean)} rows")
        
        # Remove invalid image paths
        if 'image_path' in df_clean.columns:
            df_clean = df_clean[df_clean['image_path'].notna()]
            df_clean = df_clean[df_clean['image_path'] != '']
        
        # Remove outliers in states
        if 'velocity' in df_clean.columns:
            # Velocity should be reasonable (0-50 m/s = 0-180 km/h)
            df_clean = df_clean[(df_clean['velocity'] >= 0) & (df_clean['velocity'] <= 50)]
        
        if 'x' in df_clean.columns and 'y' in df_clean.columns:
            # Position should be finite
            df_clean = df_clean[
                np.isfinite(df_clean['x']) & 
                np.isfinite(df_clean['y'])
            ]
        
        if 'yaw' in df_clean.columns:
            # Yaw should be in reasonable range (-180 to 180)
            df_clean = df_clean[(df_clean['yaw'] >= -180) & (df_clean['yaw'] <= 180)]
        
        # Remove outliers in controls
        if 'steering' in df_clean.columns:
            # Steering: -1 to 1
            df_clean = df_clean[(df_clean['steering'] >= -1.5) & (df_clean['steering'] <= 1.5)]
        
        if 'throttle' in df_clean.columns:
            # Throttle: 0 to 1
            df_clean = df_clean[(df_clean['throttle'] >= -0.1) & (df_clean['throttle'] <= 1.1)]
        
        if 'brake' in df_clean.columns:
            # Brake: 0 to 1
            df_clean = df_clean[(df_clean['brake'] >= -0.1) & (df_clean['brake'] <= 1.1)]
        
        # Remove duplicate steps
        if 'step' in df_clean.columns:
            df_clean = df_clean.drop_duplicates(subset=['step'], keep='first')
        
        logger.info(f"After cleaning: {len(df_clean)} rows ({len(df_clean)/len(df)*100:.1f}% retained)")
        
        return df_clean.reset_index(drop=True)
    
    def compute_statistics(self, df: pd.DataFrame) -> Dict[str, Any]:
        """
        Compute statistics for normalization.
        
        Args:
            df: Dataframe with states and controls
            
        Returns:
            Dictionary with statistics
        """
        stats = {}
        
        # State statistics
        state_cols = ['x', 'y', 'yaw', 'velocity']
        if all(col in df.columns for col in state_cols):
            states = df[state_cols].values
            stats['state_mean'] = states.mean(axis=0).tolist()
            stats['state_std'] = states.std(axis=0).tolist()
            stats['state_min'] = states.min(axis=0).tolist()
            stats['state_max'] = states.max(axis=0).tolist()
            
            # Avoid division by zero
            stats['state_std'] = [max(s, 1e-8) for s in stats['state_std']]
            
            logger.info("State statistics:")
            logger.info(f"  Mean: {stats['state_mean']}")
            logger.info(f"  Std:  {stats['state_std']}")
        
        # Control statistics
        control_cols = ['steering', 'throttle', 'brake']
        if all(col in df.columns for col in control_cols):
            controls = df[control_cols].values
            stats['control_mean'] = controls.mean(axis=0).tolist()
            stats['control_std'] = controls.std(axis=0).tolist()
            stats['control_min'] = controls.min(axis=0).tolist()
            stats['control_max'] = controls.max(axis=0).tolist()
            
            stats['control_std'] = [max(s, 1e-8) for s in stats['control_std']]
            
            logger.info("Control statistics:")
            logger.info(f"  Mean: {stats['control_mean']}")
            logger.info(f"  Std:  {stats['control_std']}")
        
        self.state_stats = stats.get('state_mean'), stats.get('state_std')
        self.control_stats = stats.get('control_mean'), stats.get('control_std')
        
        return stats
    
    def normalize_states(self, states: np.ndarray, stats: Dict[str, Any] = None) -> Tuple[np.ndarray, Dict[str, Any]]:
        """
        Normalize states.
        
        Args:
            states: State array (N, 4) [x, y, yaw, velocity]
            stats: Statistics dictionary (if None, compute from data)
            
        Returns:
            Normalized states, statistics
        """
        if stats is None:
            if self.state_stats is None:
                raise ValueError("Statistics not computed. Call compute_statistics first.")
            mean = np.array(self.state_stats[0])
            std = np.array(self.state_stats[1])
        else:
            mean = np.array(stats['state_mean'])
            std = np.array(stats['state_std'])
        
        # Normalize
        states_norm = (states - mean) / std
        
        return states_norm, {'state_mean': mean.tolist(), 'state_std': std.tolist()}
    
    def denormalize_states(self, states_norm: np.ndarray, stats: Dict[str, Any]) -> np.ndarray:
        """
        Denormalize states.
        
        Args:
            states_norm: Normalized state array
            stats: Statistics dictionary
            
        Returns:
            Denormalized states
        """
        mean = np.array(stats['state_mean'])
        std = np.array(stats['state_std'])
        
        states = states_norm * std + mean
        
        return states
    
    def detect_outliers(self, df: pd.DataFrame, method: str = 'iqr') -> pd.DataFrame:
        """
        Detect and remove outliers.
        
        Args:
            df: Dataframe
            method: 'iqr' (Interquartile Range) or 'zscore'
            
        Returns:
            Dataframe without outliers
        """
        df_clean = df.copy()
        
        if method == 'iqr':
            # IQR method
            numeric_cols = ['x', 'y', 'yaw', 'velocity', 'steering', 'throttle', 'brake']
            numeric_cols = [col for col in numeric_cols if col in df_clean.columns]
            
            for col in numeric_cols:
                Q1 = df_clean[col].quantile(0.25)
                Q3 = df_clean[col].quantile(0.75)
                IQR = Q3 - Q1
                lower_bound = Q1 - 1.5 * IQR
                upper_bound = Q3 + 1.5 * IQR
                
                before = len(df_clean)
                df_clean = df_clean[(df_clean[col] >= lower_bound) & (df_clean[col] <= upper_bound)]
                removed = before - len(df_clean)
                if removed > 0:
                    logger.info(f"Removed {removed} outliers from {col}")
        
        elif method == 'zscore':
            # Z-score method
            from scipy import stats
            numeric_cols = ['x', 'y', 'yaw', 'velocity', 'steering', 'throttle', 'brake']
            numeric_cols = [col for col in numeric_cols if col in df_clean.columns]
            
            z_scores = np.abs(stats.zscore(df_clean[numeric_cols]))
            df_clean = df_clean[(z_scores < 3).all(axis=1)]
        
        logger.info(f"After outlier removal: {len(df_clean)} rows")
        
        return df_clean.reset_index(drop=True)
    
    def validate_data(self, df: pd.DataFrame) -> bool:
        """
        Validate data quality.
        
        Args:
            df: Dataframe to validate
            
        Returns:
            True if valid
        """
        issues = []
        
        # Check required columns
        required_cols = ['x', 'y', 'yaw', 'velocity']
        missing = [col for col in required_cols if col not in df.columns]
        if missing:
            issues.append(f"Missing columns: {missing}")
        
        # Check data types
        numeric_cols = ['x', 'y', 'yaw', 'velocity']
        for col in numeric_cols:
            if col in df.columns:
                if not pd.api.types.is_numeric_dtype(df[col]):
                    issues.append(f"{col} is not numeric")
                # Check for NaN/Inf
                nan_count = df[col].isna().sum()
                if nan_count > 0:
                    issues.append(f"{col} has {nan_count} NaN values")
                inf_count = np.isinf(df[col]).sum()
                if inf_count > 0:
                    issues.append(f"{col} has {inf_count} Inf values")
        
        # Check for sufficient data
        if len(df) < 100:
            issues.append(f"Insufficient data: only {len(df)} rows")
        
        # Check for sequences
        if 'step' in df.columns:
            steps = df['step'].values
            if len(np.unique(np.diff(steps))) > 1:
                issues.append("Non-sequential steps detected")
        
        # Check data quality metrics
        if 'velocity' in df.columns:
            moving_frames = (df['velocity'] > 1.0).sum()
            if moving_frames < len(df) * 0.1:  # Less than 10% moving
                issues.append(f"Low movement: only {moving_frames}/{len(df)} frames moving")
        
        if 'x' in df.columns and 'y' in df.columns:
            distance = ((df['x'].diff()**2 + df['y'].diff()**2)**0.5).sum()
            if distance < 10:  # Less than 10m traveled
                issues.append(f"Low distance traveled: only {distance:.2f}m")
        
        if issues:
            logger.warning("Data validation issues:")
            for issue in issues:
                logger.warning(f"  - {issue}")
            return False
        
        logger.info("✅ Data validation passed")
        return True


def preprocess_dataset(data_dir: str, output_dir: str = None) -> Dict[str, Any]:
    """
    Complete preprocessing pipeline.
    
    Args:
        data_dir: Directory containing data.csv
        output_dir: Output directory (default: data_dir/processed)
        
    Returns:
        Statistics dictionary
    """
    data_dir = Path(data_dir)
    
    if output_dir is None:
        output_dir = data_dir / "processed"
    else:
        output_dir = Path(output_dir)
    output_dir.mkdir(exist_ok=True)
    
    # Load data
    csv_path = data_dir / "data.csv"
    if not csv_path.exists():
        raise FileNotFoundError(f"data.csv not found in {data_dir}")
    
    logger.info(f"Loading data from {csv_path}")
    df = pd.read_csv(csv_path)
    logger.info(f"Loaded {len(df)} rows")
    
    # Initialize preprocessor
    preprocessor = DataPreprocessor()
    
    # Step 1: Clean data
    logger.info("Step 1: Cleaning data...")
    df_clean = preprocessor.clean_data(df)
    
    # Step 2: Detect outliers
    logger.info("Step 2: Detecting outliers...")
    df_clean = preprocessor.detect_outliers(df_clean, method='iqr')
    
    # Step 3: Validate
    logger.info("Step 3: Validating data...")
    if not preprocessor.validate_data(df_clean):
        logger.warning("Data validation failed, but continuing...")
    
    # Step 4: Compute statistics
    logger.info("Step 4: Computing statistics...")
    stats = preprocessor.compute_statistics(df_clean)
    
    # Save processed data
    output_csv = output_dir / "data_processed.csv"
    df_clean.to_csv(output_csv, index=False)
    logger.info(f"✅ Saved processed data to {output_csv}")
    
    # Save statistics
    stats_path = output_dir / "statistics.json"
    with open(stats_path, 'w') as f:
        json.dump(stats, f, indent=2)
    logger.info(f"✅ Saved statistics to {stats_path}")
    
    # Summary
    logger.info("=" * 50)
    logger.info("Preprocessing Summary:")
    logger.info(f"  Original rows: {len(df)}")
    logger.info(f"  Processed rows: {len(df_clean)}")
    logger.info(f"  Retention: {len(df_clean)/len(df)*100:.1f}%")
    logger.info("=" * 50)
    
    return stats

