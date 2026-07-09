import numpy as np

from src.data.default_case import default_case
from src.simulator.dispersion import run_dispersion_analysis


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
