"""
LSTM Validation Module.

Implements STEP 4.4 requirements:
- Trajectory MSE calculation
- Stability across time validation
"""

import logging
import numpy as np
from typing import Dict, Any, List, Tuple, Optional
from pathlib import Path
import pandas as pd

logger = logging.getLogger(__name__)


class LSTMValidator:
    """Validates LSTM predictor performance."""
    
    @staticmethod
    def calculate_trajectory_mse(
        predictions: np.ndarray,
        ground_truth: np.ndarray
    ) -> Dict[str, float]:
        """
        Calculate trajectory Mean Squared Error (MSE).
        
        Args:
            predictions: Predicted states (N, 4) [x, y, yaw, velocity]
            ground_truth: Ground truth states (N, 4) [x, y, yaw, velocity]
        
        Returns:
            Dictionary with MSE metrics
        """
        if predictions.shape != ground_truth.shape:
            raise ValueError(f"Shape mismatch: predictions {predictions.shape} vs ground_truth {ground_truth.shape}")
        
        # Calculate MSE for each component
        mse_x = np.mean((predictions[:, 0] - ground_truth[:, 0]) ** 2)
        mse_y = np.mean((predictions[:, 1] - ground_truth[:, 1]) ** 2)
        mse_yaw = np.mean((predictions[:, 2] - ground_truth[:, 2]) ** 2)
        mse_velocity = np.mean((predictions[:, 3] - ground_truth[:, 3]) ** 2)
        
        # Overall MSE
        mse_overall = np.mean((predictions - ground_truth) ** 2)
        
        # RMSE
        rmse_x = np.sqrt(mse_x)
        rmse_y = np.sqrt(mse_y)
        rmse_yaw = np.sqrt(mse_yaw)
        rmse_velocity = np.sqrt(mse_velocity)
        rmse_overall = np.sqrt(mse_overall)
        
        return {
            'mse': {
                'x': float(mse_x),
                'y': float(mse_y),
                'yaw': float(mse_yaw),
                'velocity': float(mse_velocity),
                'overall': float(mse_overall)
            },
            'rmse': {
                'x': float(rmse_x),
                'y': float(rmse_y),
                'yaw': float(rmse_yaw),
                'velocity': float(rmse_velocity),
                'overall': float(rmse_overall)
            }
        }
    
    @staticmethod
    def calculate_stability_metrics(
        predictions: np.ndarray,
        window_size: int = 10
    ) -> Dict[str, float]:
        """
        Calculate stability metrics across time.
        
        Args:
            predictions: Predicted states (N, 4)
            window_size: Window size for stability calculation
        
        Returns:
            Dictionary with stability metrics
        """
        if len(predictions) < window_size:
            logger.warning(f"Not enough predictions ({len(predictions)}) for stability calculation (need {window_size})")
            return {}
        
        # Calculate variance in sliding windows
        variances = []
        for i in range(len(predictions) - window_size + 1):
            window = predictions[i:i+window_size]
            var = np.var(window, axis=0)
            variances.append(var)
        
        variances = np.array(variances)
        
        # Average variance per component
        avg_variance = np.mean(variances, axis=0)
        
        # Stability score (lower variance = more stable)
        # Normalize by component ranges
        stability_scores = 1.0 / (1.0 + avg_variance)
        
        return {
            'variance': {
                'x': float(avg_variance[0]),
                'y': float(avg_variance[1]),
                'yaw': float(avg_variance[2]),
                'velocity': float(avg_variance[3])
            },
            'stability_score': {
                'x': float(stability_scores[0]),
                'y': float(stability_scores[1]),
                'yaw': float(stability_scores[2]),
                'velocity': float(stability_scores[3])
            },
            'overall_stability': float(np.mean(stability_scores))
        }
    
    @staticmethod
    def validate_on_logs(
        prediction_log_path: Path,
        trajectory_log_path: Path
    ) -> Dict[str, Any]:
        """
        Validate LSTM predictions from log files.
        
        Args:
            prediction_log_path: Path to prediction log CSV
            trajectory_log_path: Path to trajectory log CSV
        
        Returns:
            Dictionary with validation results
        """
        try:
            pred_df = pd.read_csv(prediction_log_path)
            traj_df = pd.read_csv(trajectory_log_path)
        except Exception as e:
            logger.error(f"Failed to load log files: {e}")
            return {}
        
        # Align by step
        merged = pd.merge(
            pred_df, traj_df,
            on='step',
            suffixes=('_pred', '_gt')
        )
        
        if len(merged) == 0:
            logger.error("No matching steps found between prediction and trajectory logs")
            return {}
        
        # Extract predictions and ground truth
        predictions = merged[['pred_x', 'pred_y', 'pred_yaw', 'pred_velocity']].values
        ground_truth = merged[['x', 'y', 'yaw', 'velocity']].values
        
        # Calculate MSE
        mse_results = LSTMValidator.calculate_trajectory_mse(predictions, ground_truth)
        
        # Calculate stability
        stability_results = LSTMValidator.calculate_stability_metrics(predictions)
        
        return {
            'num_samples': len(merged),
            'mse': mse_results,
            'stability': stability_results
        }
    
    @staticmethod
    def print_validation_results(results: Dict[str, Any]) -> None:
        """
        Print validation results.
        
        Args:
            results: Validation results dictionary
        """
        logger.info("=" * 60)
        logger.info("LSTM VALIDATION RESULTS")
        logger.info("=" * 60)
        logger.info(f"Number of samples: {results.get('num_samples', 0)}")
        logger.info("")
        
        # MSE
        mse = results.get('mse', {})
        logger.info("Trajectory MSE:")
        if 'mse' in mse:
            logger.info(f"  X:        {mse['mse']['x']:.4f}")
            logger.info(f"  Y:        {mse['mse']['y']:.4f}")
            logger.info(f"  Yaw:      {mse['mse']['yaw']:.4f}")
            logger.info(f"  Velocity: {mse['mse']['velocity']:.4f}")
            logger.info(f"  Overall:  {mse['mse']['overall']:.4f}")
        logger.info("")
        
        # RMSE
        if 'rmse' in mse:
            logger.info("Trajectory RMSE:")
            logger.info(f"  X:        {mse['rmse']['x']:.4f}")
            logger.info(f"  Y:        {mse['rmse']['y']:.4f}")
            logger.info(f"  Yaw:      {mse['rmse']['yaw']:.4f}")
            logger.info(f"  Velocity: {mse['rmse']['velocity']:.4f}")
            logger.info(f"  Overall:  {mse['rmse']['overall']:.4f}")
        logger.info("")
        
        # Stability
        stability = results.get('stability', {})
        if stability:
            logger.info("Stability Metrics:")
            if 'variance' in stability:
                logger.info(f"  Variance X:        {stability['variance']['x']:.4f}")
                logger.info(f"  Variance Y:        {stability['variance']['y']:.4f}")
                logger.info(f"  Variance Yaw:      {stability['variance']['yaw']:.4f}")
                logger.info(f"  Variance Velocity: {stability['variance']['velocity']:.4f}")
            if 'stability_score' in stability:
                logger.info(f"  Stability Score:   {stability.get('overall_stability', 0.0):.4f}")
        logger.info("")
        logger.info("=" * 60)

