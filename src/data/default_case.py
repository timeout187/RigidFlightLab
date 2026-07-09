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
    t_max_s: float = 90.0
    ground_altitude_m: float = 0.0


@dataclass
class DispersionSettings:
    """Uncertainty parameters for the Monte Carlo dispersion sensitivity
    sweep. Defaults are Table 2 of Khalil, Abdalla & Kamal (2009),
    "Dispersion Analysis for Spinning Artillery Projectile" - the
    paper's own eight uncertainty parameters and ranges:

        1. Firing pitch angle:          +/- 0.4 deg
        2. Projectile mass:              +/- 1.0 %
        3. Axial moment of inertia:      +/- 2.0 %
        4. Lateral moment of inertia:    +/- 2.0 %
        5. Projectile muzzle velocity:   +/- 2.0 %
        6. Projectile muzzle spin rate:  +/- 2.0 %
        7. Wind speed at zero altitude:  +/- 2.0 m/s
        8. Wind direction at zero altitude: +/- 2.0 deg

    The paper itself sweeps each parameter individually (deterministic,
    one at a time) and plots the resulting range/drift/radial error -
    see Figures 11-18. This project instead draws all eight
    simultaneously from independent Gaussians for a joint Monte Carlo
    sweep (the same general method used in one of the paper's own
    references [8]), treating the paper's stated range as an
    approximate one-standard-deviation width. This is a deliberate
    methodological choice, not a reproduction of Figures 11-18
    specifically - see docs/model.md.

    `azimuth_angle_std_deg` and `cd0_std_frac` are not part of the
    paper's Table 2; they are provided as optional extra uncertainty
    sources and default to 0 (disabled).
    """

    n_samples: int = 200
    muzzle_velocity_std_pct: float = 2.0
    elevation_angle_std_deg: float = 0.4
    mass_std_pct: float = 1.0
    axial_inertia_std_pct: float = 2.0
    lateral_inertia_std_pct: float = 2.0
    spin_rate_std_pct: float = 2.0
    wind_speed_std_mps: float = 2.0
    wind_direction_std_deg: float = 2.0
    azimuth_angle_std_deg: float = 0.0
    cd0_std_frac: float = 0.0
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
