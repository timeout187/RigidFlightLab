import numpy as np

from src.data.default_case import default_case
from src.simulator.dispersion import run_dispersion_analysis


def test_dispersion_defaults_match_paper_table_2():
    disp = default_case().dispersion
    assert disp.elevation_angle_std_deg == 0.4
    assert disp.mass_std_pct == 1.0
    assert disp.axial_inertia_std_pct == 2.0
    assert disp.lateral_inertia_std_pct == 2.0
    assert disp.muzzle_velocity_std_pct == 2.0
    assert disp.spin_rate_std_pct == 2.0
    assert disp.wind_speed_std_mps == 2.0
    assert disp.wind_direction_std_deg == 2.0


def test_dispersion_analysis_runs_and_produces_spread():
    case = default_case()
    case.solver.t_max_s = 5.0  # keep the test fast; only need early-flight spread
    case.dispersion.n_samples = 5
    result = run_dispersion_analysis(case)

    assert len(result.impact_x) == 5
    assert np.all(np.isfinite(result.impact_x))
    assert np.all(np.isfinite(result.impact_y))

    summary = result.summary()
    assert summary["n_samples"] == 5
    assert summary["std_range_m"] >= 0.0
