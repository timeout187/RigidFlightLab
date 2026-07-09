"""Numerical integrators for the 6-DOF trajectory propagation.

Provides a fixed-step classical RK4 integrator and a wrapper around
scipy.integrate.solve_ivp for adaptive-step methods (RK45, DOP853, ...).

Academic simulation component - numerical methods education only.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.integrate import solve_ivp

from src.simulator.dynamics import DynamicsModel, hit_ground_event


@dataclass
class TrajectoryResult:
    t: np.ndarray            # (N,) time samples, s
    state: np.ndarray        # (N, 12) state history
    mach: np.ndarray         # (N,) Mach number history
    alpha_deg: np.ndarray    # (N,) total angle of attack, deg
    success: bool
    message: str

    def as_dict(self) -> dict:
        return {
            "t": self.t,
            "x": self.state[:, 0],
            "y": self.state[:, 1],
            "z": self.state[:, 2],
            "u": self.state[:, 3],
            "v": self.state[:, 4],
            "w": self.state[:, 5],
            "phi_deg": np.degrees(self.state[:, 6]),
            "theta_deg": np.degrees(self.state[:, 7]),
            "psi_deg": np.degrees(self.state[:, 8]),
            "p_rps": self.state[:, 9] / (2 * np.pi),
            "q_deg_s": np.degrees(self.state[:, 10]),
            "r_deg_s": np.degrees(self.state[:, 11]),
            "mach": self.mach,
            "alpha_deg": self.alpha_deg,
        }


def _post_process(model: DynamicsModel, t: np.ndarray, state: np.ndarray):
    import math

    from src.simulator.atmosphere import standard_atmosphere

    mach = np.zeros(len(t))
    alpha_deg = np.zeros(len(t))
    for i in range(len(t)):
        s = state[i]
        atmo = standard_atmosphere(s[2])
        vel_body = s[3:6]
        vel_mag = max(np.linalg.norm(vel_body), 1e-6)
        mach[i] = vel_mag / atmo.speed_of_sound_mps
        cos_a = np.clip(vel_body[0] / vel_mag, -1.0, 1.0)
        alpha_deg[i] = math.degrees(math.acos(cos_a))
    return mach, alpha_deg


def run_rk4(model: DynamicsModel, t_max: float, dt: float) -> TrajectoryResult:
    """Fixed-step classical 4th-order Runge-Kutta integrator, stopping
    when the projectile reaches the ground altitude."""
    ground = model.case.solver.ground_altitude_m
    y = model.initial_state()
    t = 0.0
    ts = [t]
    states = [y.copy()]

    while t < t_max:
        h = min(dt, t_max - t)
        k1 = model.state_derivative(t, y)
        k2 = model.state_derivative(t + h / 2, y + h / 2 * k1)
        k3 = model.state_derivative(t + h / 2, y + h / 2 * k2)
        k4 = model.state_derivative(t + h, y + h * k3)
        y_next = y + (h / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
        t_next = t + h

        if y_next[2] <= ground and y[2] > ground:
            frac = (y[2] - ground) / (y[2] - y_next[2]) if y[2] != y_next[2] else 1.0
            t_next = t + h * frac
            y_next = y + (y_next - y) * frac
            ts.append(t_next)
            states.append(y_next.copy())
            t, y = t_next, y_next
            break

        t, y = t_next, y_next
        ts.append(t)
        states.append(y.copy())

    t_arr = np.array(ts)
    state_arr = np.array(states)
    mach, alpha_deg = _post_process(model, t_arr, state_arr)
    return TrajectoryResult(t_arr, state_arr, mach, alpha_deg, True, "RK4 fixed-step integration complete")


def run_solve_ivp(model: DynamicsModel, t_max: float, method: str = "RK45",
                   max_step: float = 0.01, rtol: float = 1e-8, atol: float = 1e-8) -> TrajectoryResult:
    """Adaptive-step integration via scipy.integrate.solve_ivp."""
    ground = model.case.solver.ground_altitude_m
    y0 = model.initial_state()

    def event(t, y):
        return hit_ground_event(t, y, ground)
    event.terminal = True
    event.direction = -1

    sol = solve_ivp(
        model.state_derivative,
        (0.0, t_max),
        y0,
        method=method,
        max_step=max_step,
        rtol=rtol,
        atol=atol,
        events=event,
        dense_output=False,
    )

    t_arr = sol.t
    state_arr = sol.y.T
    mach, alpha_deg = _post_process(model, t_arr, state_arr)
    return TrajectoryResult(t_arr, state_arr, mach, alpha_deg, sol.success, sol.message)


def run_trajectory(model: DynamicsModel) -> TrajectoryResult:
    """Dispatch to the configured solver method in model.case.solver."""
    solver = model.case.solver
    if solver.method.upper() == "RK4":
        return run_rk4(model, solver.t_max_s, solver.fixed_step_s)
    return run_solve_ivp(
        model, solver.t_max_s, method=solver.method,
        max_step=solver.max_step_s, rtol=solver.rtol, atol=solver.atol,
    )
