"""Nominal educational simulation case: runs the default 155 mm
academic example (from the published "Dispersion Analysis for Spinning
Artillery Projectile" paper) and prints/exports the trajectory.

Academic simulation - published benchmark reproduction and numerical-
methods education only. Not for operational use.

Usage:
    python -m examples.nominal_run
"""
from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import numpy as np  # noqa: E402
import pandas as pd  # noqa: E402

from src.data.default_case import default_case  # noqa: E402
from src.simulator.dynamics import DynamicsModel  # noqa: E402
from src.simulator.integrators import run_trajectory  # noqa: E402


def main() -> None:
    case = default_case()
    model = DynamicsModel(case)
    result = run_trajectory(model)
    data = result.as_dict()

    print(f"Solver message: {result.message}")
    print(f"Time of flight: {data['t'][-1]:.2f} s")
    print(f"Range (downrange): {data['x'][-1]:.1f} m")
    print(f"Cross-range: {data['y'][-1]:.2f} m")
    print(f"Max altitude: {np.max(data['z']):.1f} m")
    print(f"Muzzle spin rate: {data['p_rps'][0]:.2f} rev/s -> impact spin rate: {data['p_rps'][-1]:.2f} rev/s")

    out_path = Path(__file__).with_name("nominal_run_output.csv")
    pd.DataFrame(data).to_csv(out_path, index=False)
    print(f"Full trajectory written to {out_path}")


if __name__ == "__main__":
    main()
