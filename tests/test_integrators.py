import numpy as np

from src.data.default_case import default_case
from src.simulator.dynamics import DynamicsModel
from src.simulator.integrators import run_rk4, run_solve_ivp, run_trajectory


def test_rk4_and_solve_ivp_agree_over_a_short_early_flight_segment():
    """RK4 (fixed-step) and solve_ivp (adaptive RK45) should agree
    closely before the projectile has traveled far, without paying the
    cost of integrating the full multi-hundred-second flight."""
    case = default_case()
    model = DynamicsModel(case)

    rk4_result = run_rk4(model, t_max=5.0, dt=0.002)
    ivp_result = run_solve_ivp(model, t_max=5.0, method="RK45", max_step=0.01, rtol=1e-7, atol=1e-7)

    rk4_range = rk4_result.state[-1, 0]
    ivp_range = ivp_result.state[-1, 0]
    assert rk4_range > 0
    assert ivp_range > 0
    assert abs(rk4_range - ivp_range) / max(rk4_range, ivp_range) < 0.05


def test_run_trajectory_dispatches_on_method():
    case = default_case()
    case.solver.method = "RK4"
    case.solver.fixed_step_s = 0.002
    case.solver.t_max_s = 5.0
    model = DynamicsModel(case)
    result = run_trajectory(model)
    assert result.success
    assert result.t[-1] > 0


def test_trajectory_terminates_at_ground():
    case = default_case()
    model = DynamicsModel(case)
    result = run_trajectory(model)
    assert result.state[-1, 2] <= case.solver.ground_altitude_m + 1.0
