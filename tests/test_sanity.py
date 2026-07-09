"""Output sanity checks against physically plausible bounds for the
published 155 mm academic example case (muzzle velocity 684.3 m/s,
elevation 44 deg, spin rate 175.48 rps).

Note: the bundled aerodynamic coefficient table is a representative
academic approximation, not a validated match to any specific shell
variant, so these bounds check gross physical plausibility (finite,
correctly-signed, right order of magnitude) rather than an exact
published-range reproduction. See docs/model.md for limitations.
"""
import math

import numpy as np

from src.data.default_case import default_case
from src.simulator.dynamics import DynamicsModel
from src.simulator.integrators import run_trajectory


_CACHE = {}


def _nominal_result():
    if "result" not in _CACHE:
        case = default_case()
        model = DynamicsModel(case)
        _CACHE["result"] = (run_trajectory(model), case)
    return _CACHE["result"]


def test_range_is_finite_and_positive():
    result, _ = _nominal_result()
    range_m = result.state[-1, 0]
    assert 0 < range_m < 100_000


def test_time_of_flight_is_positive_and_bounded():
    result, _ = _nominal_result()
    tof = result.t[-1]
    assert 5 < tof < 300


def test_spin_rate_decays_due_to_spin_damping():
    result, _ = _nominal_result()
    spin_rps = result.state[:, 9] / (2 * math.pi)
    assert spin_rps[0] > spin_rps[-1]
    assert spin_rps[-1] > 0  # should not reverse sign over the flight


def test_altitude_starts_at_launch_altitude():
    result, case = _nominal_result()
    assert math.isclose(result.state[0, 2], case.initial_conditions.launch_altitude_m, abs_tol=1e-6)


def test_trajectory_terminates_at_or_near_ground():
    result, case = _nominal_result()
    assert result.state[-1, 2] <= case.solver.ground_altitude_m + 1.0


def test_max_altitude_is_positive_and_plausible():
    result, _ = _nominal_result()
    max_alt = np.max(result.state[:, 2])
    assert 100 < max_alt < 20_000


def test_no_nan_or_inf_in_state_history():
    result, _ = _nominal_result()
    assert np.all(np.isfinite(result.state))
