"""Default academic example data for the 155 mm spin-stabilized
projectile, taken from the published paper "Dispersion Analysis for
Spinning Artillery Projectile".

Academic simulation data - published benchmark reproduction only.
Not validated for real-world fire-control use.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from src.simulator.aero import AeroTable, default_155mm_aero_table
from src.simulator.atmosphere import WindModel


@dataclass
class ProjectileProperties:
    caliber_m: float = 0.155
    length_m: float = 0.698
    mass_kg: float = 43.0
    cg_from_nose_m: float = 0.459
    ixx_kg_m2: float = 0.144   # axial (roll) moment of inertia
    iyy_kg_m2: float = 1.216   # transverse moment of inertia
    izz_kg_m2: float = 1.216   # transverse moment of inertia (Iyy == Izz)


@dataclass
class InitialConditions:
    muzzle_velocity_mps: float = 684.3
    muzzle_spin_rate_rps: float = 175.48
    elevation_angle_deg: float = 44.0
    azimuth_angle_deg: float = 0.0
    launch_altitude_m: float = 0.0


@dataclass
class SolverSettings:
    method: str = "RK45"       # "RK4" (fixed-step) or a scipy solve_ivp method
    fixed_step_s: float = 0.002
    max_step_s: float = 0.02
    rtol: float = 1e-6
    atol: float = 1e-6
    t_max_s: float = 300.0
    ground_altitude_m: float = 0.0


@dataclass
class DispersionSettings:
    """Uncertainty ranges (as +/- fractions or absolute values) used for
    Monte Carlo dispersion sensitivity sweeps."""

    n_samples: int = 200
    muzzle_velocity_std_mps: float = 2.0
    elevation_angle_std_deg: float = 0.05
    azimuth_angle_std_deg: float = 0.05
    mass_std_kg: float = 0.1
    cd0_std_frac: float = 0.03
    wind_std_mps: float = 2.0
    random_seed: int = 42


@dataclass
class SimulationCase:
    projectile: ProjectileProperties = field(default_factory=ProjectileProperties)
    initial_conditions: InitialConditions = field(default_factory=InitialConditions)
    solver: SolverSettings = field(default_factory=SolverSettings)
    dispersion: DispersionSettings = field(default_factory=DispersionSettings)
    wind: WindModel = field(default_factory=WindModel)
    aero_table: AeroTable = field(default_factory=default_155mm_aero_table)


def default_case() -> SimulationCase:
    """Return the nominal academic example case from the paper."""
    return SimulationCase()
