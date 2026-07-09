"""Monte Carlo dispersion-sensitivity analysis.

Perturbs muzzle velocity, launch angles, mass, drag coefficient, and
wind within user-specified uncertainty ranges and reports the spread
of impact points. Academic reproduction of published dispersion-
analysis methodology - NOT a targeting or fire-control tool, and does
not compute or suggest any aim/fire-control correction.
"""
from __future__ import annotations

import copy
from dataclasses import dataclass

import numpy as np

from src.data.default_case import SimulationCase
from src.simulator.dynamics import DynamicsModel
from src.simulator.integrators import run_trajectory


@dataclass
class DispersionResult:
    impact_x: np.ndarray
    impact_y: np.ndarray
    range_m: np.ndarray
    time_of_flight_s: np.ndarray

    def summary(self) -> dict:
        return {
            "mean_range_m": float(np.mean(self.range_m)),
            "std_range_m": float(np.std(self.range_m)),
            "mean_cross_range_m": float(np.mean(self.impact_y)),
            "std_cross_range_m": float(np.std(self.impact_y)),
            "cep_50_m": float(np.percentile(
                np.hypot(self.impact_x - np.mean(self.impact_x),
                         self.impact_y - np.mean(self.impact_y)), 50)),
            "n_samples": int(len(self.range_m)),
        }


def run_dispersion_analysis(base_case: SimulationCase) -> DispersionResult:
    disp = base_case.dispersion
    rng = np.random.default_rng(disp.random_seed)

    n = disp.n_samples
    impact_x = np.zeros(n)
    impact_y = np.zeros(n)
    tof = np.zeros(n)

    dv = rng.normal(0.0, disp.muzzle_velocity_std_mps, n)
    delev = rng.normal(0.0, disp.elevation_angle_std_deg, n)
    dazim = rng.normal(0.0, disp.azimuth_angle_std_deg, n)
    dmass = rng.normal(0.0, disp.mass_std_kg, n)
    dcd = rng.normal(0.0, disp.cd0_std_frac, n)
    dwind_n = rng.normal(0.0, disp.wind_std_mps, n)
    dwind_e = rng.normal(0.0, disp.wind_std_mps, n)

    for i in range(n):
        case = copy.deepcopy(base_case)
        case.initial_conditions.muzzle_velocity_mps += dv[i]
        case.initial_conditions.elevation_angle_deg += delev[i]
        case.initial_conditions.azimuth_angle_deg += dazim[i]
        case.projectile.mass_kg += dmass[i]
        case.aero_table.cd0 = case.aero_table.cd0 * (1.0 + dcd[i])
        case.wind.wind_north_mps += dwind_n[i]
        case.wind.wind_east_mps += dwind_e[i]

        model = DynamicsModel(case)
        result = run_trajectory(model)
        final = result.state[-1]
        impact_x[i] = final[0]
        impact_y[i] = final[1]
        tof[i] = result.t[-1]

    return DispersionResult(impact_x, impact_y, impact_x.copy(), tof)
