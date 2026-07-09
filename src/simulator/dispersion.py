"""Monte Carlo dispersion-sensitivity analysis.

Perturbs the eight uncertainty parameters from Table 2 of Khalil,
Abdalla & Kamal (2009) - firing pitch angle, projectile mass, axial
and lateral moments of inertia, muzzle velocity, muzzle spin rate, and
wind speed/direction at zero altitude - and reports the spread of
impact points. Academic reproduction of published dispersion-analysis
methodology - NOT a targeting or fire-control tool, and does not
compute or suggest any aim/fire-control correction. See
`DispersionSettings` and docs/model.md for how this project's joint
Monte Carlo sweep relates to the paper's own individual-parameter
sweeps (Figures 11-18).
"""
from __future__ import annotations

import copy
import math
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

    v0 = base_case.initial_conditions.muzzle_velocity_mps
    mass0 = base_case.projectile.mass_kg
    ixx0 = base_case.projectile.ixx_kg_m2
    iyy0 = base_case.projectile.iyy_kg_m2
    izz0 = base_case.projectile.izz_kg_m2
    spin0 = base_case.initial_conditions.muzzle_spin_rate_rps
    wind_n0 = base_case.wind.wind_north_mps
    wind_e0 = base_case.wind.wind_east_mps
    wind_speed0 = math.hypot(wind_n0, wind_e0)
    wind_dir0 = math.atan2(wind_e0, wind_n0)

    dv = rng.normal(0.0, disp.muzzle_velocity_std_pct / 100.0, n) * v0
    delev = rng.normal(0.0, disp.elevation_angle_std_deg, n)
    dazim = rng.normal(0.0, disp.azimuth_angle_std_deg, n)
    dmass = rng.normal(0.0, disp.mass_std_pct / 100.0, n) * mass0
    daxial = rng.normal(0.0, disp.axial_inertia_std_pct / 100.0, n)
    dlateral = rng.normal(0.0, disp.lateral_inertia_std_pct / 100.0, n)
    dspin = rng.normal(0.0, disp.spin_rate_std_pct / 100.0, n) * spin0
    dcd = rng.normal(0.0, disp.cd0_std_frac, n)
    dwind_speed = rng.normal(0.0, disp.wind_speed_std_mps, n)
    dwind_dir = rng.normal(0.0, math.radians(disp.wind_direction_std_deg), n)

    for i in range(n):
        case = copy.deepcopy(base_case)
        case.initial_conditions.muzzle_velocity_mps += dv[i]
        case.initial_conditions.elevation_angle_deg += delev[i]
        case.initial_conditions.azimuth_angle_deg += dazim[i]
        case.initial_conditions.muzzle_spin_rate_rps += dspin[i]
        case.projectile.mass_kg += dmass[i]
        case.projectile.ixx_kg_m2 = ixx0 * (1.0 + daxial[i])
        case.projectile.iyy_kg_m2 = iyy0 * (1.0 + dlateral[i])
        case.projectile.izz_kg_m2 = izz0 * (1.0 + dlateral[i])
        case.aero_table.cd0 = case.aero_table.cd0 * (1.0 + dcd[i])

        wind_speed = wind_speed0 + dwind_speed[i]
        wind_dir = wind_dir0 + dwind_dir[i]
        case.wind.wind_north_mps = wind_speed * math.cos(wind_dir)
        case.wind.wind_east_mps = wind_speed * math.sin(wind_dir)

        model = DynamicsModel(case)
        result = run_trajectory(model)
        final = result.state[-1]
        impact_x[i] = final[0]
        impact_y[i] = final[1]
        tof[i] = result.t[-1]

    return DispersionResult(impact_x, impact_y, impact_x.copy(), tof)
