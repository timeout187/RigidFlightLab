"""Aerodynamic coefficient tables and Mach/angle-of-attack interpolation
for the 155 mm M107 spin-stabilized projectile.

The default table (`default_155mm_aero_table`) is **Table 1** from the
published paper this project reproduces:

    Khalil, M., Abdalla, H., and Kamal, O., "Dispersion Analysis for
    Spinning Artillery Projectile", 13th International Conference on
    Aerospace Sciences & Aviation Technology (ASAT-13), Military
    Technical College, Cairo, Egypt, May 2009. Table 1 (p. 6/12),
    computed by the authors using the SPINNER-98 aeroprediction code.

Column mapping from the paper's nomenclature to this module's field
names (see paper's Nomenclature, p. 1-2/12):

    CA         -> cd0        (total/zero-yaw axial force coefficient)
    CA_alpha2  -> cd_alpha2  (second-order axial force coefficient)
    CN_alpha   -> cl_alpha   (normal force coefficient derivative)
    C_Ypalpha  -> cmag_f     (Magnus force coefficient derivative)
    Clp        -> cspin      (roll damping coefficient derivative)
    Cm_alpha   -> cm_alpha   (pitching moment coefficient derivative)
    Cmq        -> cmq        (pitch damping coefficient derivative)
    Cnpalpha   -> cnpalpha_table (Magnus moment coefficient, tabulated
                  vs. Mach *and* total angle of attack - the paper
                  gives this at alpha = 0, 2, 5, 10 deg rather than as
                  a single per-radian derivative)

CN_alpha and C_Ypalpha are tabulated as negative numbers in the paper's
own body-axis sign convention; this module uses their magnitude, since
the physical force direction is reconstructed geometrically (from the
relative-wind/symmetry-axis cross product) in `dynamics.py` rather than
from a raw body-axis component. Cm_alpha, Cmq, Clp, and Cnpalpha keep
their published sign, which is physically meaningful (e.g. Cm_alpha is
*positive*, i.e. aerodynamically destabilizing/overturning - expected
for a projectile that relies on gyroscopic, not aerodynamic, static
stability).
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class AeroTable:
    """Mach-indexed aerodynamic coefficient table, plus a Mach x total-
    angle-of-attack table for the Magnus moment coefficient. All 1-D
    coefficients are linearly interpolated (clamped at the ends) in
    Mach number; the Magnus moment coefficient is bilinearly
    interpolated in (Mach, total angle of attack in degrees).
    """

    mach: np.ndarray
    cd0: np.ndarray        # CA: zero-yaw axial force coefficient
    cd_alpha2: np.ndarray  # CA_alpha2: second-order axial force coefficient
    cl_alpha: np.ndarray   # |CN_alpha|: normal force coefficient derivative
    cmq: np.ndarray        # Cmq: pitch damping moment coefficient derivative
    cm_alpha: np.ndarray   # Cm_alpha: overturning moment coefficient derivative
    cspin: np.ndarray      # Clp: roll damping coefficient derivative
    cmag_f: np.ndarray     # |C_Ypalpha|: Magnus force coefficient derivative
    alpha_grid_deg: np.ndarray        # total angle-of-attack breakpoints, deg
    cnpalpha_table: np.ndarray        # shape (len(mach), len(alpha_grid_deg))

    def _interp(self, table: np.ndarray, mach: float) -> float:
        return float(np.interp(mach, self.mach, table))

    def _cnpalpha(self, mach: float, alpha_deg: float) -> float:
        alpha_deg = float(np.clip(alpha_deg, self.alpha_grid_deg[0], self.alpha_grid_deg[-1]))
        # Interpolate each alpha breakpoint's Mach-column to the given Mach,
        # then interpolate across alpha to the given angle of attack.
        column_at_mach = np.array([
            np.interp(mach, self.mach, self.cnpalpha_table[:, j])
            for j in range(len(self.alpha_grid_deg))
        ])
        return float(np.interp(alpha_deg, self.alpha_grid_deg, column_at_mach))

    def coefficients(self, mach: float, alpha_deg: float = 0.0) -> dict:
        return {
            "cd0": self._interp(self.cd0, mach),
            "cd_alpha2": self._interp(self.cd_alpha2, mach),
            "cl_alpha": self._interp(self.cl_alpha, mach),
            "cmq": self._interp(self.cmq, mach),
            "cm_alpha": self._interp(self.cm_alpha, mach),
            "cspin": self._interp(self.cspin, mach),
            "cmag_f": self._interp(self.cmag_f, mach),
            "cnpalpha": self._cnpalpha(mach, alpha_deg),
        }


def default_155mm_aero_table() -> AeroTable:
    """Table 1 from Khalil, Abdalla & Kamal (2009), "Dispersion Analysis
    for Spinning Artillery Projectile" - the 155 mm M107 aerodynamic
    coefficients and derivatives computed with SPINNER-98."""
    mach = np.array([0.01, 0.60, 0.80, 0.90, 0.95, 1.00, 1.05, 1.10, 1.20, 1.35, 1.50, 1.75, 2.00])
    cd0 = np.array([.144, .144, .146, .167, .221, .327, .383, .381, .370, .353, .338, .314, .294])
    cd_alpha2 = np.array([2.343, 2.343, 2.847, 3.372, 3.73, 4.180, 4.691, 5.209, 5.702, 5.130, 4.561, 3.970, 3.460])
    cl_alpha = np.abs(np.array([-1.763, -1.763, -1.783, -1.827, -2.038, -2.153, -2.207, -2.255, -2.325, -2.442, -2.556, -2.692, -2.747]))
    cmag_f = np.abs(np.array([-0.767, -0.767, -0.767, -0.857, -1.082, -0.992, -0.902, -0.857, -0.767, -0.767, -0.767, -0.767, -0.767]))
    cspin = np.array([-.023, -.023, -.022, -.021, -.020, -.020, -.020, -.019, -.020, -.020, -.020, -.020, -.021])
    cm_alpha = np.array([3.355, 3.378, 3.571, 3.957, 3.886, 3.682, 3.415, 3.384, 3.424, 3.278, 3.264, 3.201, 3.013])
    cmq = np.array([-5.1, -5.1, -5.1, -7.4, -9.9, -13.8, -13.3, -14.6, -15.8, -15.6, -15.3, -15.3, -15.3])

    alpha_grid_deg = np.array([0.0, 2.0, 5.0, 10.0])
    cnpalpha_table = np.array([
        [-0.500, 0.005, 0.294, 0.58],
        [-0.500, 0.005, 0.294, 0.58],
        [-0.355, 0.078, 0.366, 0.65],
        [-0.112, 0.172, 0.415, 0.86],
        [0.085, 0.292, 0.500, 1.12],
        [0.198, 0.388, 0.482, 0.72],
        [0.293, 0.430, 0.465, 0.55],
        [0.334, 0.432, 0.456, 0.54],
        [0.352, 0.424, 0.438, 0.51],
        [0.366, 0.424, 0.438, 0.51],
        [0.373, 0.424, 0.438, 0.51],
        [0.381, 0.431, 0.438, 0.51],
        [0.388, 0.431, 0.438, 0.51],
    ])

    return AeroTable(mach, cd0, cd_alpha2, cl_alpha, cmq, cm_alpha, cspin, cmag_f,
                      alpha_grid_deg, cnpalpha_table)
