import numpy as np

from src.simulator.aero import default_155mm_aero_table


def test_interpolation_matches_table_points():
    table = default_155mm_aero_table()
    for i, m in enumerate(table.mach):
        coeffs = table.coefficients(float(m))
        assert np.isclose(coeffs["cd0"], table.cd0[i])
        assert np.isclose(coeffs["cm_alpha"], table.cm_alpha[i])


def test_interpolation_between_points_is_bounded():
    table = default_155mm_aero_table()
    mid_mach = (table.mach[2] + table.mach[3]) / 2.0
    coeffs = table.coefficients(mid_mach)
    lo, hi = sorted([table.cd0[2], table.cd0[3]])
    assert lo <= coeffs["cd0"] <= hi


def test_clamping_outside_table_range():
    table = default_155mm_aero_table()
    coeffs_low = table.coefficients(0.1)
    coeffs_high = table.coefficients(10.0)
    assert np.isclose(coeffs_low["cd0"], table.cd0[0])
    assert np.isclose(coeffs_high["cd0"], table.cd0[-1])
