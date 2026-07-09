"""ICAO/US Standard Atmosphere (1976) model, troposphere + lower stratosphere.

Academic simulation component - published benchmark reproduction only.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

G0 = 9.80665       # m/s^2, standard gravity
R_AIR = 287.05287  # J/(kg*K), specific gas constant for dry air
GAMMA = 1.4        # ratio of specific heats for air

T0 = 288.15        # K, sea-level standard temperature
P0 = 101325.0      # Pa, sea-level standard pressure
LAPSE_RATE = -0.0065  # K/m, troposphere lapse rate (0-11 km)
TROPOPAUSE_ALT = 11000.0  # m


@dataclass
class AtmosphereState:
    altitude_m: float
    temperature_k: float
    pressure_pa: float
    density_kg_m3: float
    speed_of_sound_mps: float


def standard_atmosphere(altitude_m: float) -> AtmosphereState:
    """Return atmospheric properties at a given geopotential altitude (m).

    Valid for 0-20 km (troposphere + isothermal lower stratosphere), which
    covers the flight envelope of the 155 mm example trajectory.
    """
    h = max(altitude_m, 0.0)

    if h <= TROPOPAUSE_ALT:
        t = T0 + LAPSE_RATE * h
        p = P0 * (t / T0) ** (-G0 / (LAPSE_RATE * R_AIR))
    else:
        t_tp = T0 + LAPSE_RATE * TROPOPAUSE_ALT
        p_tp = P0 * (t_tp / T0) ** (-G0 / (LAPSE_RATE * R_AIR))
        t = t_tp
        p = p_tp * math.exp(-G0 * (h - TROPOPAUSE_ALT) / (R_AIR * t_tp))

    rho = p / (R_AIR * t)
    a = math.sqrt(GAMMA * R_AIR * t)
    return AtmosphereState(h, t, p, rho, a)


@dataclass
class WindModel:
    """Simple constant/linear wind field, in the inertial (range, cross,
    up) frame. wind_shear scales the horizontal wind linearly with
    altitude relative to a reference altitude."""

    wind_north_mps: float = 0.0
    wind_east_mps: float = 0.0
    reference_altitude_m: float = 0.0
    shear_per_km: float = 0.0

    def wind_at(self, altitude_m: float) -> tuple[float, float, float]:
        dz_km = (altitude_m - self.reference_altitude_m) / 1000.0
        factor = 1.0 + self.shear_per_km * dz_km
        factor = max(factor, 0.0)
        return self.wind_north_mps * factor, self.wind_east_mps * factor, 0.0
