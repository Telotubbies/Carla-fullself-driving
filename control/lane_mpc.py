#!/usr/bin/env python3
"""
Model Predictive Controller for Lane Following.
Uses kinematic bicycle model with CasADi/IPOPT optimization.
"""
import math
import numpy as np
import casadi as ca
from dataclasses import dataclass

@dataclass
class MPCConfig:
    N: int = 15
    dt: float = 0.1
    L: float = 2.875
    max_steer: float = 0.7
    max_steer_rate: float = 0.3
    max_accel: float = 3.0
    min_accel: float = -5.0
    w_cte: float = 50.0
    w_heading: float = 30.0
    w_vel: float = 5.0
    w_steer: float = 100.0
    w_accel: float = 10.0
    w_steer_rate: float = 200.0
    w_accel_rate: float = 10.0

class LaneMPC:
    def __init__(self, cfg: MPCConfig = None):
        self.cfg = cfg or MPCConfig()
        self._solver = None
        self._prev_u = np.zeros(2)
        self._build_solver()

    def _build_solver(self):
        c = self.cfg
        N = c.N
        n_states = 4
        n_controls = 2
        X = ca.MX.sym("X", n_states, N + 1)
        U = ca.MX.sym("U", n_controls, N)
        P = ca.MX.sym("P", 10)
        cost = 0.0
        constraints = []
        lb_g = []
        ub_g = []
        for i in range(n_states):
            constraints.append(X[i, 0] - P[i])
            lb_g.append(0.0)
            ub_g.append(0.0)
        v_ref = P[4]
        u_prev = P[8:10]
        for k in range(N):
            x_k = X[:, k]
            u_k = U[:, k]
            x_next = ca.vertcat(
                x_k[0] + x_k[3] * ca.cos(x_k[2]) * c.dt,
                x_k[1] + x_k[3] * ca.sin(x_k[2]) * c.dt,
                x_k[2] + (x_k[3] / c.L) * ca.tan(u_k[0]) * c.dt,
                x_k[3] + u_k[1] * c.dt,
            )
            for i in range(n_states):
                constraints.append(X[i, k + 1] - x_next[i])
                lb_g.append(0.0)
                ub_g.append(0.0)
            cost += c.w_cte * x_k[1]**2 + c.w_heading * x_k[2]**2 + c.w_vel * (x_k[3] - v_ref)**2
            cost += c.w_steer * u_k[0]**2 + c.w_accel * u_k[1]**2
            if k == 0:
                cost += c.w_steer_rate * (u_k[0] - u_prev[0])**2 + c.w_accel_rate * (u_k[1] - u_prev[1])**2
            else:
                cost += c.w_steer_rate * (u_k[0] - U[0, k - 1])**2 + c.w_accel_rate * (u_k[1] - U[1, k - 1])**2
        cost += 2 * c.w_cte * X[1, N]**2 + 2 * c.w_heading * X[2, N]**2
        opt_vars = ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1))
        n_x_vars = n_states * (N + 1)
        n_u_vars = n_controls * N
        lb_x = [-1e6] * n_x_vars
        ub_x = [1e6] * n_x_vars
        for k in range(N + 1):
            lb_x[k * n_states + 3] = 0.0
            ub_x[k * n_states + 3] = 20.0
        lb_u = []
        ub_u = []
        for k in range(N):
            lb_u.extend([-c.max_steer, c.min_accel])
            ub_u.extend([c.max_steer, c.max_accel])
        nlp = {"x": opt_vars, "f": cost, "g": ca.vertcat(*constraints), "p": P}
        opts = {"ipopt.print_level": 0, "ipopt.max_iter": 50, "print_time": 0}
        self._solver = ca.nlpsol("mpc", "ipopt", nlp, opts)
        self._lb = lb_x + lb_u
        self._ub = ub_x + ub_u
        self._lb_g = lb_g
        self._ub_g = ub_g
        self._n_states = n_states
        self._n_controls = n_controls
        self._N = N

    def solve(self, x0, y0, psi0, v0, v_ref, cte, heading_err, curvature=0.0):
        p = np.array([x0, y0, psi0, v0, v_ref, cte, heading_err, curvature, self._prev_u[0], self._prev_u[1]])
        n_x = self._n_states * (self._N + 1)
        n_u = self._n_controls * self._N
        x_init = np.zeros(n_x + n_u)
        try:
            sol = self._solver(x0=x_init, lbx=self._lb, ubx=self._ub, lbg=self._lb_g, ubg=self._ub_g, p=p)
            opt = sol["x"].full().flatten()
            u0 = opt[n_x:n_x + 2]
            steer = float(np.clip(u0[0], -self.cfg.max_steer, self.cfg.max_steer))
            accel = float(np.clip(u0[1], self.cfg.min_accel, self.cfg.max_accel))
            self._prev_u = np.array([steer, accel])
            return steer, accel
        except Exception:
            steer = -0.5 * cte - 0.3 * heading_err
            steer = float(np.clip(steer, -self.cfg.max_steer, self.cfg.max_steer))
            accel = 0.5 * (v_ref - v0)
            accel = float(np.clip(accel, self.cfg.min_accel, self.cfg.max_accel))
            return steer, accel
