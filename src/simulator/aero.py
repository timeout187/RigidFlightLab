"""Aerodynamic coefficient tables and Mach-interpolation for a
spin-stabilized 155 mm projectile.

Coefficient values are representative of the published literature on
spin-stabilized 155 mm artillery shells (drag/lift/Magnus/moment
coefficients vs. Mach number). They are provided for academic
benchmark reproduction and numerical-methods education only - NOT
validated for real-world fire-control use.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class AeroTable:
    """Mach-indexed aerodynamic coefficient table.

    All coefficients are linearly interpolated (clamped at the ends) as
    functions of Mach number.
    """

    mach: np.ndarray
    cd0: np.ndarray      # zero-yaw drag coefficient
    cd_alpha2: np.ndarray  # drag due to angle of attack^2
    cl_alpha: np.ndarray   # lift-force coefficient slope
    cmq: np.ndarray        # pitch damping moment coefficient
    cm_alpha: np.ndarray   # overturning (static) moment coefficient slope
    cspin: np.ndarray      # spin damping moment coefficient
    cmag_f: np.ndarray     # Magnus force coefficient
    cmag_m: np.ndarray     # Magnus moment coefficient

    def _interp(self, table: np.ndarray, mach: float) -> float:
        return float(np.interp(mach, self.mach, table))

    def coefficients(self, mach: float) -> dict:
        return {
            "cd0": self._interp(self.cd0, mach),
            "cd_alpha2": self._interp(self.cd_alpha2, mach),
            "cl_alpha": self._interp(self.cl_alpha, mach),
            "cmq": self._interp(self.cmq, mach),
            "cm_alpha": self._interp(self.cm_alpha, mach),
            "cspin": self._interp(self.cspin, mach),
            "cmag_f": self._interp(self.cmag_f, mach),
            "cmag_m": self._interp(self.cmag_m, mach),
        }


def default_155mm_aero_table() -> AeroTable:
    """Representative aero table for the 155 mm M549/ERFB-type spin-
    stabilized shell used in the paper's example case, spanning the
    subsonic-to-supersonic flight regime (Mach 0.5 - 3.0)."""
    mach = np.array([0.5, 0.8, 1.0, 1.2, 1.5, 2.0, 2.5, 3.0])
    cd0 = np.array([0.15, 0.18, 0.30, 0.32, 0.28, 0.24, 0.21, 0.19])
    cd_alpha2 = np.array([2.5, 2.6, 3.0, 3.2, 3.0, 2.8, 2.6, 2.5])
    cl_alpha = np.array([2.0, 2.1, 2.4, 2.6, 2.5, 2.3, 2.1, 2.0])
    cmq = np.array([-4.0, -4.2, -5.0, -5.5, -5.2, -4.8, -4.5, -4.3])
    cm_alpha = np.array([2.2, 2.3, 2.6, 2.8, 2.6, 2.4, 2.2, 2.1])
    cspin = np.array([-0.02, -0.02, -0.025, -0.03, -0.028, -0.025, -0.022, -0.02])
    cmag_f = np.array([0.5, 0.55, 0.6, 0.65, 0.6, 0.55, 0.5, 0.48])
    cmag_m = np.array([-0.3, -0.32, -0.35, -0.38, -0.35, -0.32, -0.3, -0.28])
    return AeroTable(mach, cd0, cd_alpha2, cl_alpha, cmq, cm_alpha, cspin, cmag_f, cmag_m)
