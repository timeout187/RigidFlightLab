import numpy as np

from src.simulator.aero import default_155mm_aero_table


def test_interpolation_matches_table_points():
    table = default_155mm_aero_table()
    for i, m in enumerate(table.mach):
        coeffs = table.coefficients(float(m), alpha_deg=0.0)
        assert np.isclose(coeffs["cd0"], table.cd0[i])
        assert np.isclose(coeffs["cm_alpha"], table.cm_alpha[i])
        assert np.isclose(coeffs["cnpalpha"], table.cnpalpha_table[i, 0])


def test_interpolation_between_mach_points_is_bounded():
    table = default_155mm_aero_table()
    mid_mach = (table.mach[2] + table.mach[3]) / 2.0
    coeffs = table.coefficients(mid_mach, alpha_deg=0.0)
    lo, hi = sorted([table.cd0[2], table.cd0[3]])
    assert lo <= coeffs["cd0"] <= hi


def test_clamping_outside_mach_range():
    table = default_155mm_aero_table()
    coeffs_low = table.coefficients(0.001, alpha_deg=0.0)
    coeffs_high = table.coefficients(10.0, alpha_deg=0.0)
    assert np.isclose(coeffs_low["cd0"], table.cd0[0])
    assert np.isclose(coeffs_high["cd0"], table.cd0[-1])


def test_cnpalpha_varies_with_alpha_and_is_clamped():
    table = default_155mm_aero_table()
    mach = 1.00  # matches a table row exactly
    row = 5  # index of Mach=1.00 in the table
    assert np.isclose(table.coefficients(mach, alpha_deg=0.0)["cnpalpha"], table.cnpalpha_table[row, 0])
    assert np.isclose(table.coefficients(mach, alpha_deg=2.0)["cnpalpha"], table.cnpalpha_table[row, 1])
    assert np.isclose(table.coefficients(mach, alpha_deg=10.0)["cnpalpha"], table.cnpalpha_table[row, 3])
    # clamped beyond the tabulated alpha range
    assert np.isclose(table.coefficients(mach, alpha_deg=20.0)["cnpalpha"], table.cnpalpha_table[row, 3])
    assert np.isclose(table.coefficients(mach, alpha_deg=-5.0)["cnpalpha"], table.cnpalpha_table[row, 0])


def test_cd0_shows_transonic_drag_rise():
    """Table 1's CA column should show the expected transonic drag rise
    (a jump around Mach 1) rather than a smooth monotonic curve."""
    table = default_155mm_aero_table()
    subsonic_cd0 = table.coefficients(0.60, alpha_deg=0.0)["cd0"]
    transonic_cd0 = table.coefficients(1.05, alpha_deg=0.0)["cd0"]
    assert transonic_cd0 > subsonic_cd0 * 2
