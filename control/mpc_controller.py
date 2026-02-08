"""
Model Predictive Control (MPC) for autonomous vehicle control.

Uses kinematic bicycle model and CasADi for optimization.
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


class MPCController:
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
        self.w_lateral = self.weights.get('lateral_error', 10.0)
        self.w_heading = self.weights.get('heading_error', 5.0)
        self.w_velocity = self.weights.get('velocity_error', 2.0)
        self.w_steer_rate = self.weights.get('steer_rate', 1.0)
        self.w_accel_rate = self.weights.get('accel_rate', 1.0)
        
        # Constraints
        self.max_speed = self.mpc_config.get('constraints', {}).get('max_speed', 30.0)  # km/h
        self.min_speed = self.mpc_config.get('constraints', {}).get('min_speed', 0.0)  # km/h
        
        # Convert max_speed from km/h to m/s
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
        self._setup_optimizer()
        
        logger.info(f"✅ MPCController initialized (N={self.N}, dt={self.dt})")
    
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
        
        # Parameters: initial state and reference trajectory
        x0 = opti.parameter(4)  # initial state
        ref_traj = opti.parameter(4, self.N + 1)  # reference trajectory
        
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
            
            # Control rate cost (smoothness)
            if k > 0:
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
        
        # State constraints
        opti.subject_to(opti.bounded(self.min_speed_ms, X[3, :], self.max_speed_ms))  # velocity
        
        # Set cost function
        opti.minimize(cost)
        
        # Solver options
        opts = {
            'ipopt.print_level': 0,
            'ipopt.sb': 'yes',
            'print_time': 0
        }
        opti.solver('ipopt', opts)
        
        self.opti = opti
        self.X = X
        self.U = U
        self.x0_param = x0
        self.ref_traj_param = ref_traj
    
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
        current_state: Dict[str, float],
        reference_trajectory: Optional[np.ndarray] = None
    ) -> Tuple[float, float, float]:
        """
        Compute control action using MPC.
        
        Args:
            current_state: Current vehicle state dictionary
            reference_trajectory: Reference trajectory (N+1, 4) or None to use default
            
        Returns:
            Tuple of (steering, throttle, brake)
        """
        try:
            # Validate state
            if not self._validate_state(current_state):
                logger.warning("Invalid state in MPC, using safe defaults")
                return 0.0, 0.0, 1.0  # Brake
            
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
            
            # Set parameters
            self.opti.set_value(self.x0_param, x0)
            self.opti.set_value(self.ref_traj_param, reference_trajectory.T)
            
            # Initial guess (warm start)
            if hasattr(self, 'prev_solution_X'):
                self.opti.set_initial(self.X, self.prev_solution_X)
                self.opti.set_initial(self.U, self.prev_solution_U)
            
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
                logger.warning(f"MPC optimization failed: {e}, using previous control")
                steering = self.prev_steer
                accel = self.prev_accel
            
            # Convert to CARLA control format
            # Steering: [-1, 1] in CARLA, we have [-max_steer, max_steer] in radians
            steering_normalized = np.clip(steering / self.max_steer, -1.0, 1.0)
            
            # Throttle and brake from acceleration
            if accel > 0:
                throttle = np.clip(accel / self.max_accel, 0.0, 1.0)
                brake = 0.0
            else:
                throttle = 0.0
                brake = np.clip(-accel / abs(self.max_decel), 0.0, 1.0)
            
            return steering_normalized, throttle, brake
            
        except Exception as e:
            logger.error(f"Error in MPC control computation: {e}")
            # Return safe default
            return 0.0, 0.0, 1.0  # No steering, no throttle, brake
    
    def _generate_reference_trajectory(self, current_state: np.ndarray) -> np.ndarray:
        """
        Generate simple reference trajectory (straight ahead with constant speed).
        
        Args:
            current_state: Current state [x, y, yaw, v]
            
        Returns:
            Reference trajectory (N+1, 4)
        """
        x, y, yaw, v = current_state
        
        # Target speed: if stopped, accelerate to 5 m/s, otherwise maintain or reach 8 m/s
        if v < 0.5:
            # Starting from stop: accelerate
            target_v = 5.0  # Target 5 m/s (18 km/h)
        else:
            # Already moving: maintain or slightly increase
            target_v = min(self.max_speed_ms, max(v, 8.0))  # Target 8 m/s (28.8 km/h)
        
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
    
    def set_lane_info(self, lane_info: Dict[str, Any]) -> None:
        """
        Set lane information for path planning.
        
        Args:
            lane_info: Lane information dict with 'left_lane', 'right_lane', 'center_line'
        """
        self.lane_info = lane_info
    
    def set_waypoints(self, waypoints: list) -> None:
        """
        Set CARLA waypoints for path planning.
        
        Args:
            waypoints: List of CARLA waypoints
        """
        self.waypoints = waypoints
    
    def get_predicted_trajectory(self) -> Optional[np.ndarray]:
        """
        Get predicted trajectory from last MPC solution.
        
        Returns:
            Predicted trajectory (N+1, 4) or None
        """
        if hasattr(self, 'prev_solution_X'):
            return self.prev_solution_X.T  # Transpose to (N+1, 4)
        return None

