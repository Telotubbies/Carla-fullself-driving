"""
Model Predictive Control (MPC) for autonomous vehicle control.

Uses kinematic bicycle model and CasADi for optimization.
Implements IControlModule interface.
"""

import casadi as ca
import numpy as np
import logging
from typing import Dict, Any, Optional, Tuple
import time

logger = logging.getLogger(__name__)

# Import lane path planner
try:
    from .lane_path_planner import LanePathPlanner
except ImportError:
    from lane_path_planner import LanePathPlanner

# Import core interfaces and exceptions
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent))
from core.interfaces import IControlModule
from core.exceptions import ControlError, DataValidationError
from core.validators import StateValidator, ControlValidator


class MPCController(IControlModule):
    """MPC controller using kinematic bicycle model."""
    
    def __init__(self, config: Dict[str, Any]):
        """
        Initialize MPC controller.
        
        Args:
            config: MPC configuration dictionary
        """
        self.config = config
        self.mpc_config = config.get('mpc', {})
        self.vehicle_config = self.mpc_config.get('vehicle', {})
        self.weights = self.mpc_config.get('weights', {})
        
        # Vehicle parameters
        self.L = self.vehicle_config.get('wheelbase', 2.875)  # wheelbase (m)
        self.max_steer = self.vehicle_config.get('max_steer', 0.5)  # max steering angle (rad)
        self.max_accel = self.vehicle_config.get('max_accel', 3.0)  # max acceleration (m/s²)
        self.max_decel = self.vehicle_config.get('max_decel', -3.0)  # max deceleration (m/s²)
        
        # MPC parameters
        self.N = self.mpc_config.get('horizon', 10)  # prediction horizon
        self.dt = self.mpc_config.get('dt', 0.05)  # time step (s)
        
        # Cost weights
        self.w_lateral = self.weights.get('lateral_error', 5.0)
        self.w_heading = self.weights.get('heading_error', 8.0)
        self.w_velocity = self.weights.get('velocity_error', 1.0)
        self.w_steer = self.weights.get('steering', 20.0)
        self.w_steer_rate = self.weights.get('steer_rate', 50.0)
        self.w_accel_rate = self.weights.get('accel_rate', 5.0)
        
        # Constraints
        self.max_speed = self.mpc_config.get('constraints', {}).get('max_speed', 30.0)  # km/h
        self.min_speed = self.mpc_config.get('constraints', {}).get('min_speed', 0.0)  # km/h
        
        # Convert max_speed from km/h to m/s
        # Increased max speed for faster driving
        self.max_speed = min(self.max_speed, 50.0)  # Cap at 50 km/h (13.9 m/s)
        self.max_speed_ms = self.max_speed / 3.6
        self.min_speed_ms = self.min_speed / 3.6
        
        # Reference trajectory (will be set dynamically)
        self.reference_trajectory: Optional[np.ndarray] = None
        
        # Previous control for rate limiting
        self.prev_steer = 0.0
        self.prev_accel = 0.0
        
        # Lane path planner
        self.lane_planner = LanePathPlanner(config)
        
        # Lane information cache
        self.lane_info: Optional[Dict[str, Any]] = None
        self.waypoints: Optional[list] = None
        
        # Setup optimization problem
        try:
            self._setup_optimizer()
            logger.info(f"✅ MPCController initialized (N={self.N}, dt={self.dt})")
        except Exception as e:
            logger.error(f"⚠️  MPC optimizer setup failed: {e}, will use simple control fallback")
            self.opti = None  # Mark as not initialized
    
    def set_waypoints(self, waypoints: Optional[list]) -> None:
        """Set waypoints for lane following."""
        self.waypoints = waypoints
    
    def set_lane_info(self, lane_info: Optional[Dict[str, Any]]) -> None:
        """Set lane information for lane following."""
        self.lane_info = lane_info
    
    def _setup_optimizer(self) -> None:
        """Setup CasADi optimization problem."""
        # State variables: [x, y, yaw, v]
        # Control variables: [delta (steer), accel]
        
        # Optimization variables
        opti = ca.Opti()
        
        # State trajectory
        X = opti.variable(4, self.N + 1)  # [x, y, yaw, v] for N+1 steps
        # Control trajectory
        U = opti.variable(2, self.N)  # [delta, accel] for N steps
        
        # Parameters: initial state, reference trajectory, and previous controls
        x0 = opti.parameter(4)  # initial state
        ref_traj = opti.parameter(4, self.N + 1)  # reference trajectory
        u_prev = opti.parameter(2)  # previous control [steer, accel]
        
        # Cost function
        cost = 0
        
        # Initial state constraint
        opti.subject_to(X[:, 0] == x0)
        
        # Dynamics constraints (kinematic bicycle model)
        for k in range(self.N):
            x_k = X[:, k]
            u_k = U[:, k]
            
            # Kinematic bicycle model
            x = x_k[0]
            y = x_k[1]
            yaw = x_k[2]
            v = x_k[3]
            delta = u_k[0]  # steering angle
            accel = u_k[1]  # acceleration
            
            # State derivatives
            x_dot = v * ca.cos(yaw)
            y_dot = v * ca.sin(yaw)
            yaw_dot = (v / self.L) * ca.tan(delta)
            v_dot = accel
            
            # Euler integration
            x_next = x + self.dt * x_dot
            y_next = y + self.dt * y_dot
            yaw_next = yaw + self.dt * yaw_dot
            v_next = v + self.dt * v_dot
            
            opti.subject_to(X[:, k + 1] == ca.vertcat(x_next, y_next, yaw_next, v_next))
            
            # Reference tracking cost
            ref_k = ref_traj[:, k]
            lateral_error = ca.sqrt((x - ref_k[0])**2 + (y - ref_k[1])**2)
            heading_error = ca.fabs(yaw - ref_k[2])
            velocity_error = ca.fabs(v - ref_k[3])
            
            cost += self.w_lateral * lateral_error**2
            cost += self.w_heading * heading_error**2
            cost += self.w_velocity * velocity_error**2
            
            # Absolute steering cost (penalize large steering angles)
            cost += self.w_steer * u_k[0]**2
            
            # Control rate cost (smoothness)
            if k == 0:
                cost += self.w_steer_rate * (U[0, 0] - u_prev[0])**2
                cost += self.w_accel_rate * (U[1, 0] - u_prev[1])**2
            else:
                cost += self.w_steer_rate * (U[0, k] - U[0, k-1])**2
                cost += self.w_accel_rate * (U[1, k] - U[1, k-1])**2
        
        # Terminal cost
        ref_N = ref_traj[:, -1]
        x_N = X[:, -1]
        lateral_error_N = ca.sqrt((x_N[0] - ref_N[0])**2 + (x_N[1] - ref_N[1])**2)
        heading_error_N = ca.fabs(x_N[2] - ref_N[2])
        velocity_error_N = ca.fabs(x_N[3] - ref_N[3])
        cost += 2.0 * (self.w_lateral * lateral_error_N**2 + 
                      self.w_heading * heading_error_N**2 + 
                      self.w_velocity * velocity_error_N**2)
        
        # Control constraints
        opti.subject_to(opti.bounded(-self.max_steer, U[0, :], self.max_steer))  # steering
        opti.subject_to(opti.bounded(self.max_decel, U[1, :], self.max_accel))  # acceleration
        
        # State constraints (relaxed for feasibility)
        # Use wider bounds to avoid infeasibility
        min_v = max(0.0, self.min_speed_ms - 1.0)  # Allow slightly negative for feasibility
        max_v = min(20.0, self.max_speed_ms + 2.0)  # Allow slightly higher
        opti.subject_to(opti.bounded(min_v, X[3, :], max_v))  # velocity
        
        # Set cost function
        opti.minimize(cost)
        
        # Solver options (more robust settings)
        opts = {
            'ipopt.print_level': 0,
            'ipopt.sb': 'yes',
            'print_time': 0,
            'ipopt.max_iter': 100,  # Limit iterations
            'ipopt.tol': 1e-3,  # Relax tolerance
            'ipopt.acceptable_tol': 1e-2,  # Accept suboptimal solutions
            'ipopt.acceptable_iter': 5,  # Accept after 5 iterations
            'ipopt.mu_init': 1e-3,  # Initial barrier parameter
            'ipopt.warm_start_init_point': 'yes',  # Use warm start
        }
        opti.solver('ipopt', opts)
        
        self.opti = opti
        self.X = X
        self.U = U
        self.x0_param = x0
        self.ref_traj_param = ref_traj
        self.u_prev_param = u_prev
    
    def _validate_state(self, state: Dict[str, float]) -> bool:
        """Validate vehicle state."""
        required = ['x', 'y', 'yaw', 'velocity']
        if not all(key in state for key in required):
            return False
        for key in required:
            value = state[key]
            if not isinstance(value, (int, float)) or np.isnan(value) or np.isinf(value):
                return False
        return True
    
    def _validate_trajectory(self, traj: np.ndarray) -> bool:
        """Validate reference trajectory."""
        if traj is None:
            return False
        if not isinstance(traj, np.ndarray):
            return False
        if traj.shape != (self.N + 1, 4):
            return False
        if np.any(np.isnan(traj)) or np.any(np.isinf(traj)):
            return False
        return True
    
    def compute_control(
        self,
        current_state: Dict[str, Any],
        reference_trajectory: Optional[np.ndarray] = None
    ) -> Tuple[float, float, float]:
        """
        Compute control output.
        
        Implements IControlModule.compute_control().
        
        Args:
            current_state: Current vehicle state dictionary
            reference_trajectory: Optional reference trajectory (N+1, 4)
        
        Returns:
            (steering, throttle, brake)
        
        Raises:
            ControlError: If control computation fails
            DataValidationError: If state or trajectory is invalid
        """
        try:
            # Validate state
            try:
                StateValidator.validate(current_state)
            except DataValidationError as e:
                logger.error(f"Invalid state: {e}")
                raise ControlError(f"Invalid vehicle state: {e}") from e
            
            # Extract current state
            x = current_state.get('x', 0.0)
            y = current_state.get('y', 0.0)
            yaw = np.deg2rad(current_state.get('yaw', 0.0))  # Convert to radians
            v = current_state.get('velocity', 0.0)  # m/s
            
            x0 = np.array([x, y, yaw, v])
            
            # Validate reference trajectory if provided
            if reference_trajectory is not None:
                if not self._validate_trajectory(reference_trajectory):
                    logger.warning("Invalid reference trajectory, generating default")
                    reference_trajectory = None
            
            # Generate reference trajectory if not provided
            if reference_trajectory is None:
                # Try to use lane-based trajectory
                reference_trajectory = self.lane_planner.generate_reference_from_lanes(
                    x0,
                    lane_info=self.lane_info,
                    waypoints=self.waypoints
                )
                # Fallback to default if lane planner fails
                if reference_trajectory is None or reference_trajectory.shape[0] != self.N + 1:
                    reference_trajectory = self._generate_reference_trajectory(x0)
            
            # Validate and fix reference trajectory
            reference_trajectory = self._validate_and_fix_trajectory(reference_trajectory, x0)
            
            # Check if optimizer is initialized
            if not hasattr(self, 'opti') or self.opti is None:
                logger.error("MPC optimizer not initialized, falling back to simple control")
                # Fallback: simple proportional control
                return self._simple_control(x0, reference_trajectory)
            
            # Set parameters
            try:
                self.opti.set_value(self.x0_param, x0)
                self.opti.set_value(self.ref_traj_param, reference_trajectory.T)
                self.opti.set_value(self.u_prev_param, np.array([self.prev_steer, self.prev_accel]))
            except Exception as e:
                logger.error(f"Failed to set MPC parameters: {e}, falling back to simple control")
                return self._simple_control(x0, reference_trajectory)
            
            # Initial guess (warm start or simple forward prediction)
            if hasattr(self, 'prev_solution_X') and self.prev_solution_X is not None:
                try:
                    self.opti.set_initial(self.X, self.prev_solution_X)
                    self.opti.set_initial(self.U, self.prev_solution_U)
                except Exception as e:
                    logger.debug(f"Could not set initial guess: {e}")
                    # Fallback: simple forward prediction
                    self._set_simple_initial_guess(x0, reference_trajectory)
            else:
                # First time: use simple forward prediction
                self._set_simple_initial_guess(x0, reference_trajectory)
            
            # Solve optimization problem
            try:
                sol = self.opti.solve()
                
                # Extract control
                u_opt = sol.value(self.U)
                steering = float(u_opt[0, 0])
                accel = float(u_opt[1, 0])
                
                # Store solution for warm start
                self.prev_solution_X = sol.value(self.X)
                self.prev_solution_U = sol.value(self.U)
                
                # Update previous controls
                self.prev_steer = steering
                self.prev_accel = accel
                
            except Exception as e:
                logger.warning(f"MPC optimization failed: {e}, using simple control fallback")
                return self._simple_control(x0, reference_trajectory)
            
            # Convert to CARLA control format
            # CARLA steering [-1, 1] maps to ~[-70, 70] degrees (1.22 rad) for Tesla Model 3
            # MPC outputs steering angle in radians bounded by max_steer
            carla_max_steer_rad = np.deg2rad(70.0)
            steering_normalized = np.clip(steering / carla_max_steer_rad, -1.0, 1.0)
            
            # Throttle and brake from acceleration
            if accel > 0:
                throttle = np.clip(accel / self.max_accel, 0.0, 1.0)
                brake = 0.0
            else:
                throttle = 0.0
            
            # Throttle and brake from acceleration (complete the logic)
            if accel <= 0:
                brake = np.clip(-accel / abs(self.max_decel), 0.0, 1.0)
            
            # Validate control output
            try:
                ControlValidator.validate(steering_normalized, throttle, brake)
            except DataValidationError as e:
                logger.error(f"Invalid control output: {e}")
                raise ControlError(f"Invalid control values: {e}") from e
            
            return steering_normalized, throttle, brake
            
        except Exception as e:
            logger.error(f"Error in MPC control computation: {e}")
            # Return safe default
            return 0.0, 0.0, 1.0  # No steering, no throttle, brake
    
    def _simple_control(self, current_state: np.ndarray, reference_trajectory: Optional[np.ndarray] = None) -> Tuple[float, float, float]:
        """
        Simple fallback control when MPC optimization fails.
        
        Uses proportional control to follow reference trajectory.
        
        Args:
            current_state: Current state [x, y, yaw, v]
            reference_trajectory: Optional reference trajectory (N+1, 4)
        
        Returns:
            (steering, throttle, brake)
        """
        # Simple proportional control
        # Get target speed from config or use max_speed_ms
        target_speed = getattr(self, 'max_speed_ms', 8.0)
        
        # If we have reference trajectory, use first point
        if reference_trajectory is not None and len(reference_trajectory) > 0:
            ref_point = reference_trajectory[0]
            # Calculate desired yaw towards reference point
            dx = ref_point[0] - current_state[0]
            dy = ref_point[1] - current_state[1]
            desired_yaw = np.arctan2(dy, dx)
            yaw_error = desired_yaw - current_state[2]
            # Normalize yaw error to [-pi, pi]
            yaw_error = np.arctan2(np.sin(yaw_error), np.cos(yaw_error))
            steering = np.clip(yaw_error * 0.5, -self.max_steer, self.max_steer)
        else:
            # No reference, go straight
            steering = 0.0
        
        # Speed control
        speed_error = target_speed - current_state[3]
        if speed_error > 0.5:
            throttle = min(0.5, speed_error * 0.3)
            brake = 0.0
        elif speed_error < -0.5:
            throttle = 0.0
            brake = min(0.5, -speed_error * 0.3)
        else:
            throttle = 0.1  # Maintain speed
            brake = 0.0
        
        # Normalize steering to [-1, 1]
        steering_normalized = np.clip(steering / self.max_steer, -1.0, 1.0)
        
        return float(steering_normalized), float(throttle), float(brake)
    
    def _validate_and_fix_trajectory(self, traj: np.ndarray, current_state: np.ndarray) -> np.ndarray:
        """
        Validate and fix reference trajectory to ensure feasibility.
        
        Args:
            traj: Reference trajectory (N+1, 4)
            current_state: Current state [x, y, yaw, v]
        
        Returns:
            Fixed trajectory (N+1, 4)
        """
        if traj is None or traj.shape[0] != self.N + 1:
            return self._generate_reference_trajectory(current_state)
        
        # Fix NaN/Inf values
        traj = np.nan_to_num(traj, nan=0.0, posinf=100.0, neginf=-100.0)
        
        # Ensure reasonable velocity bounds
        traj[:, 3] = np.clip(traj[:, 3], 0.0, self.max_speed_ms + 1.0)
        
        # Ensure trajectory starts near current state
        if np.linalg.norm(traj[0, :2] - current_state[:2]) > 5.0:
            traj[0, :2] = current_state[:2]
        
        # Smooth trajectory to avoid sharp changes
        for i in range(1, len(traj)):
            # Limit change in position
            dx = traj[i, 0] - traj[i-1, 0]
            dy = traj[i, 1] - traj[i-1, 1]
            dist = np.sqrt(dx**2 + dy**2)
            max_dist = traj[i-1, 3] * self.dt * 2.0  # Max distance based on speed
            if dist > max_dist:
                scale = max_dist / (dist + 1e-6)
                traj[i, 0] = traj[i-1, 0] + dx * scale
                traj[i, 1] = traj[i-1, 1] + dy * scale
        
        return traj
    
    def _set_simple_initial_guess(self, x0: np.ndarray, ref_traj: np.ndarray) -> None:
        """
        Set simple initial guess for optimization variables.
        
        Args:
            x0: Initial state [x, y, yaw, v]
            ref_traj: Reference trajectory (N+1, 4)
        """
        try:
            # Simple forward prediction for states
            X_init = np.zeros((4, self.N + 1))
            X_init[:, 0] = x0
            
            for k in range(1, self.N + 1):
                if k < len(ref_traj):
                    # Use reference trajectory as initial guess
                    X_init[:, k] = ref_traj[k]
                else:
                    # Simple forward prediction
                    x_prev = X_init[:, k-1]
                    v = max(0.1, x_prev[3])  # Minimum speed
                    yaw = x_prev[2]
                    X_init[0, k] = x_prev[0] + v * np.cos(yaw) * self.dt
                    X_init[1, k] = x_prev[1] + v * np.sin(yaw) * self.dt
                    X_init[2, k] = yaw
                    X_init[3, k] = v
            
            # Simple control guess (zero steering, small acceleration)
            U_init = np.zeros((2, self.N))
            U_init[1, :] = 0.1  # Small acceleration
            
            self.opti.set_initial(self.X, X_init)
            self.opti.set_initial(self.U, U_init)
        except Exception as e:
            logger.debug(f"Could not set simple initial guess: {e}")
    
    def _generate_reference_trajectory(self, current_state: np.ndarray) -> np.ndarray:
        """
        Generate simple reference trajectory (straight ahead with constant speed).
        
        Args:
            current_state: Current state [x, y, yaw, v]
            
        Returns:
            Reference trajectory (N+1, 4)
        """
        x, y, yaw, v = current_state
        
        # Target speed: increased for faster driving
        # Start at 8 m/s, can reach up to 12 m/s (43.2 km/h)
        if v < 0.5:
            # Starting from stop: accelerate to 8 m/s
            target_v = 8.0  # Target 8 m/s (28.8 km/h)
        elif v < 10.0:
            # Accelerate gradually to 12 m/s
            target_v = min(12.0, v + 0.5)  # Gradually increase
        else:
            # Maintain 12 m/s (43.2 km/h)
            target_v = min(self.max_speed_ms, 12.0)  # Cap at max_speed_ms or 12 m/s
        
        # Generate straight-ahead trajectory
        ref_traj = np.zeros((self.N + 1, 4))
        for i in range(self.N + 1):
            t = i * self.dt
            ref_traj[i, 0] = x + target_v * np.cos(yaw) * t  # x
            ref_traj[i, 1] = y + target_v * np.sin(yaw) * t  # y
            ref_traj[i, 2] = yaw  # yaw (straight)
            # Gradually increase velocity towards target
            ref_traj[i, 3] = min(target_v, v + (target_v - v) * (i / (self.N + 1)))  # velocity
        
        return ref_traj
    
    def set_reference_trajectory(self, trajectory: np.ndarray) -> None:
        """
        Set reference trajectory from LSTM prediction.
        
        Args:
            trajectory: Reference trajectory (N+1, 4) or (4,) for single point
        """
        if trajectory.ndim == 1:
            # Single point, extend to full horizon
            self.reference_trajectory = np.tile(trajectory, (self.N + 1, 1))
        else:
            self.reference_trajectory = trajectory.copy()
    
    def get_predicted_trajectory(self) -> Optional[np.ndarray]:
        """
        Get predicted trajectory from last MPC solution.
        
        Returns:
            Predicted trajectory (N+1, 4) or None
        """
        if hasattr(self, 'prev_solution_X'):
            return self.prev_solution_X.T  # Transpose to (N+1, 4)
        return None

