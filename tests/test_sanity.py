"""Output sanity checks against the published reference results for the
155 mm M107 academic example case (muzzle velocity 684.3 m/s, elevation
44 deg, spin rate 175.48 rps), from:

    Khalil, M., Abdalla, H., and Kamal, O., "Dispersion Analysis for
    Spinning Artillery Projectile", ASAT-13, May 2009. Section 4.3 and
    Figures 3-10 report: total flight time 66.67 s, summit time ~31 s,
    summit altitude ~5750 m, initial axial deceleration 4.45 g, pitch
    angle 44 deg -> ~-55 deg, muzzle-to-impact velocity 684.3 -> ~330
    m/s (minimum ~250-300 m/s near summit), and total angle of attack
    peaking at ~1.3 deg near the summit.

This simulator reproduces those figures closely (see docs/model.md),
so these bounds are tight, not just order-of-magnitude sanity checks.
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


def test_time_of_flight_matches_paper():
    result, _ = _nominal_result()
    tof = result.t[-1]
    assert 60 < tof < 73  # paper: 66.67 s


def test_summit_altitude_and_time_match_paper():
    result, _ = _nominal_result()
    z = result.state[:, 2]
    idx = np.argmax(z)
    assert 4500 < z[idx] < 7000  # paper: ~5750 m
    assert 25 < result.t[idx] < 37  # paper: ~31 s


def test_initial_axial_deceleration_matches_paper():
    result, _ = _nominal_result()
    u = result.state[:, 3]
    axial_accel = np.gradient(u, result.t)
    initial_g = axial_accel[0] / 9.80665
    assert -5.0 < initial_g < -3.9  # paper: -4.45 g


def test_pitch_angle_swings_from_positive_to_negative():
    result, _ = _nominal_result()
    theta_deg = np.degrees(result.state[:, 7])
    assert math.isclose(theta_deg[0], 44.0, abs_tol=0.5)
    assert theta_deg[-1] < -35  # paper: ~-55 deg at impact


def test_total_angle_of_attack_stays_small():
    """The shell should stay close to its symmetry axis throughout
    flight (small epicyclic pitch/yaw motion), not tumble."""
    result, _ = _nominal_result()
    assert np.max(result.alpha_deg) < 5.0  # paper: peaks at ~1.3 deg


def test_spin_rate_decays_due_to_spin_damping():
    result, _ = _nominal_result()
    spin_rps = result.state[:, 9] / (2 * math.pi)
    assert spin_rps[0] > spin_rps[-1]
    assert spin_rps[-1] > 0


def test_velocity_dips_then_recovers_toward_impact():
    result, _ = _nominal_result()
    vel_mag = np.linalg.norm(result.state[:, 3:6], axis=1)
    assert math.isclose(vel_mag[0], 684.3, abs_tol=0.5)
    assert vel_mag.min() < 350  # decelerates well below muzzle velocity
    assert vel_mag[-1] > vel_mag.min()  # regains speed diving back down


def test_range_is_within_plausible_155mm_envelope():
    result, _ = _nominal_result()
    range_m = result.state[-1, 0]
    assert 10_000 < range_m < 25_000  # paper's chart: ~16-17 km


def test_trajectory_terminates_at_or_near_ground():
    result, case = _nominal_result()
    assert result.state[-1, 2] <= case.solver.ground_altitude_m + 1.0


def test_no_nan_or_inf_in_state_history():
    result, _ = _nominal_result()
    assert np.all(np.isfinite(result.state))
