import math

from src.simulator.atmosphere import WindModel, standard_atmosphere


def test_sea_level_conditions():
    atmo = standard_atmosphere(0.0)
    assert math.isclose(atmo.temperature_k, 288.15, rel_tol=1e-6)
    assert math.isclose(atmo.pressure_pa, 101325.0, rel_tol=1e-6)
    assert atmo.density_kg_m3 > 1.2 and atmo.density_kg_m3 < 1.3


def test_density_decreases_with_altitude():
    atmo_low = standard_atmosphere(0.0)
    atmo_high = standard_atmosphere(5000.0)
    assert atmo_high.density_kg_m3 < atmo_low.density_kg_m3
    assert atmo_high.temperature_k < atmo_low.temperature_k


def test_stratosphere_isothermal_above_tropopause():
    atmo_11 = standard_atmosphere(11000.0)
    atmo_15 = standard_atmosphere(15000.0)
    assert math.isclose(atmo_11.temperature_k, atmo_15.temperature_k, rel_tol=1e-9)
    assert atmo_15.pressure_pa < atmo_11.pressure_pa


def test_speed_of_sound_positive_and_reasonable():
    atmo = standard_atmosphere(0.0)
    assert 330 < atmo.speed_of_sound_mps < 345


def test_wind_model_shear():
    wind = WindModel(wind_north_mps=10.0, wind_east_mps=0.0, reference_altitude_m=0.0, shear_per_km=0.1)
    wn0, we0, _ = wind.wind_at(0.0)
    wn1, _, _ = wind.wind_at(1000.0)
    assert math.isclose(wn0, 10.0)
    assert wn1 > wn0
