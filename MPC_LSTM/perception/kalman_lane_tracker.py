"""
Kalman Filter for Temporal Lane Smoothing

Tracks polynomial coefficients over time to ensure smooth, stable lane detection.
"""

import numpy as np
from typing import Optional, Tuple
import logging

logger = logging.getLogger(__name__)


class KalmanLaneTracker:
    """
    Kalman filter for tracking lane polynomial coefficients.
    
    State vector: x = [a, b, c]ᵀ (polynomial coefficients)
    """
    
    def __init__(
        self,
        polynomial_order: int = 2,
        process_noise: Optional[np.ndarray] = None,
        measurement_noise: Optional[np.ndarray] = None,
        initial_covariance: Optional[np.ndarray] = None
    ):
        """
        Initialize Kalman filter for lane tracking.
        
        Args:
            polynomial_order: Order of polynomial (2 for quadratic)
            process_noise: Process noise covariance matrix (Q)
            measurement_noise: Measurement noise covariance matrix (R)
            initial_covariance: Initial state covariance (P_0)
        """
        self.state_dim = polynomial_order + 1  # [a, b, c] for order=2
        
        # State vector: x = [a, b, c]ᵀ
        self.x = np.zeros(self.state_dim)
        
        # State covariance
        if initial_covariance is None:
            self.P = np.eye(self.state_dim) * 10.0  # Large initial uncertainty
        else:
            self.P = initial_covariance.copy()
        
        # State transition matrix (constant model: x_k = x_{k-1} + noise)
        self.F = np.eye(self.state_dim)
        
        # Observation matrix (we observe coefficients directly)
        self.H = np.eye(self.state_dim)
        
        # Process noise covariance (Q)
        if process_noise is None:
            # Default: small changes expected
            self.Q = np.diag([0.0001, 0.01, 0.1])  # [σ_a², σ_b², σ_c²]
        else:
            self.Q = process_noise.copy()
        
        # Measurement noise covariance (R)
        if measurement_noise is None:
            # Default: measurement uncertainty
            self.R = np.diag([0.001, 0.05, 0.2])  # [σ_meas_a², σ_meas_b², σ_meas_c²]
        else:
            self.R = measurement_noise.copy()
        
        # Track initialization status
        self.initialized = False
        
        logger.info(f"KalmanLaneTracker initialized: state_dim={self.state_dim}")
    
    def predict(self) -> np.ndarray:
        """
        Prediction step: x̂_{k|k-1} = F · x̂_{k-1|k-1}
        
        Returns:
            Predicted state vector
        """
        if not self.initialized:
            return self.x
        
        # State prediction
        self.x = self.F @ self.x
        
        # Covariance prediction: P_{k|k-1} = F · P_{k-1|k-1} · Fᵀ + Q
        self.P = self.F @ self.P @ self.F.T + self.Q
        
        return self.x.copy()
    
    def update(
        self,
        measurement: np.ndarray,
        measurement_covariance: Optional[np.ndarray] = None
    ) -> np.ndarray:
        """
        Update step: x̂_k = x̂_{k|k-1} + K · (z_k - H·x̂_{k|k-1})
        
        Args:
            measurement: Observed coefficients z_k = [a, b, c]ᵀ
            measurement_covariance: Optional measurement noise (overrides R)
            
        Returns:
            Updated state vector
        """
        if len(measurement) != self.state_dim:
            logger.warning(f"Measurement dimension mismatch: {len(measurement)} != {self.state_dim}")
            return self.x
        
        # Use provided measurement covariance or default R
        R = measurement_covariance if measurement_covariance is not None else self.R
        
        # Innovation: y = z_k - H·x̂_{k|k-1}
        y = measurement - self.H @ self.x
        
        # Innovation covariance: S = H·P_{k|k-1}·Hᵀ + R
        S = self.H @ self.P @ self.H.T + R
        
        # Kalman gain: K = P_{k|k-1}·Hᵀ·S⁻¹
        try:
            K = self.P @ self.H.T @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            logger.warning("Kalman gain computation failed (singular matrix)")
            return self.x
        
        # State update: x̂_k = x̂_{k|k-1} + K·y
        self.x = self.x + K @ y
        
        # Covariance update: P_{k|k} = (I - K·H)·P_{k|k-1}
        I = np.eye(self.state_dim)
        self.P = (I - K @ self.H) @ self.P
        
        # Mark as initialized after first update
        if not self.initialized:
            self.initialized = True
        
        return self.x.copy()
    
    def get_state(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Get current state and covariance.
        
        Returns:
            Tuple of (state_vector, covariance_matrix)
        """
        return self.x.copy(), self.P.copy()
    
    def set_state(
        self,
        state: np.ndarray,
        covariance: Optional[np.ndarray] = None
    ):
        """
        Set state and covariance (for initialization or reset).
        
        Args:
            state: State vector [a, b, c]ᵀ
            covariance: Optional covariance matrix
        """
        if len(state) != self.state_dim:
            logger.warning(f"State dimension mismatch: {len(state)} != {self.state_dim}")
            return
        
        self.x = state.copy()
        
        if covariance is not None:
            if covariance.shape != (self.state_dim, self.state_dim):
                logger.warning(f"Covariance shape mismatch: {covariance.shape}")
                return
            self.P = covariance.copy()
        
        self.initialized = True
    
    def reset(self):
        """Reset filter to uninitialized state."""
        self.x = np.zeros(self.state_dim)
        self.P = np.eye(self.state_dim) * 10.0
        self.initialized = False
    
    def get_uncertainty(self) -> np.ndarray:
        """
        Get uncertainty (standard deviation) for each coefficient.
        
        Returns:
            Array of standard deviations [σ_a, σ_b, σ_c]
        """
        return np.sqrt(np.diag(self.P))
    
    def is_initialized(self) -> bool:
        """Check if filter is initialized."""
        return self.initialized


class DualLaneTracker:
    """
    Tracks both left and right lane boundaries using separate Kalman filters.
    """
    
    def __init__(
        self,
        polynomial_order: int = 2,
        process_noise: Optional[np.ndarray] = None,
        measurement_noise: Optional[np.ndarray] = None
    ):
        """
        Initialize dual lane tracker.
        
        Args:
            polynomial_order: Order of polynomial
            process_noise: Process noise covariance
            measurement_noise: Measurement noise covariance
        """
        self.left_tracker = KalmanLaneTracker(
            polynomial_order=polynomial_order,
            process_noise=process_noise,
            measurement_noise=measurement_noise
        )
        self.right_tracker = KalmanLaneTracker(
            polynomial_order=polynomial_order,
            process_noise=process_noise,
            measurement_noise=measurement_noise
        )
        
        logger.info("DualLaneTracker initialized")
    
    def predict(self) -> Tuple[np.ndarray, np.ndarray]:
        """
        Predict both lanes.
        
        Returns:
            Tuple of (left_state, right_state)
        """
        left_state = self.left_tracker.predict()
        right_state = self.right_tracker.predict()
        return left_state, right_state
    
    def update(
        self,
        left_measurement: Optional[np.ndarray],
        right_measurement: Optional[np.ndarray],
        left_covariance: Optional[np.ndarray] = None,
        right_covariance: Optional[np.ndarray] = None
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Update both lanes.
        
        Args:
            left_measurement: Left lane coefficients (None to skip update)
            right_measurement: Right lane coefficients (None to skip update)
            left_covariance: Optional left measurement covariance
            right_covariance: Optional right measurement covariance
            
        Returns:
            Tuple of (left_state, right_state)
        """
        if left_measurement is not None:
            self.left_tracker.update(left_measurement, left_covariance)
        
        if right_measurement is not None:
            self.right_tracker.update(right_measurement, right_covariance)
        
        return self.left_tracker.get_state()[0], self.right_tracker.get_state()[0]
    
    def get_states(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """
        Get current states and covariances.
        
        Returns:
            Tuple of (left_state, left_cov, right_state, right_cov)
        """
        left_state, left_cov = self.left_tracker.get_state()
        right_state, right_cov = self.right_tracker.get_state()
        return left_state, left_cov, right_state, right_cov
    
    def reset(self):
        """Reset both trackers."""
        self.left_tracker.reset()
        self.right_tracker.reset()

