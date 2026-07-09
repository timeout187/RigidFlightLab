"""Dispersion sensitivity example: runs a Monte Carlo sweep over the
uncertainty parameters (muzzle velocity, launch angles, mass, drag
coefficient, wind) for the default academic 155 mm case, and reports
the impact-point spread.

Academic simulation - published benchmark reproduction and numerical-
methods education only. This is a dispersion *sensitivity* study, not
a targeting, aim-correction, or fire-control tool.

Usage:
    python -m examples.dispersion_example
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import pandas as pd  # noqa: E402

from src.data.default_case import default_case  # noqa: E402
from src.simulator.dispersion import run_dispersion_analysis  # noqa: E402


def main() -> None:
    case = default_case()
    case.dispersion.n_samples = 100
    result = run_dispersion_analysis(case)
    summary = result.summary()

    print("Dispersion sensitivity summary:")
    for key, value in summary.items():
        print(f"  {key}: {value}")

    out_path = Path(__file__).with_name("dispersion_example_output.csv")
    pd.DataFrame({
        "impact_x_m": result.impact_x,
        "impact_y_m": result.impact_y,
        "time_of_flight_s": result.time_of_flight_s,
    }).to_csv(out_path, index=False)
    print(f"Dispersion samples written to {out_path}")


if __name__ == "__main__":
    main()
