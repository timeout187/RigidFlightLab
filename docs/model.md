# Theoretical Model

RigidFlightLab is an **academic simulation** for **published-benchmark
reproduction and numerical-methods education only**. It is **not
validated for real-world fire-control use** and is not a targeting or
operational artillery tool.

## Source paper

Khalil, M., Abdalla, H., and Kamal, O., *"Dispersion Analysis for
Spinning Artillery Projectile"*, 13th International Conference on
Aerospace Sciences & Aviation Technology (ASAT-13), Paper
ASAT-13-FM-03, Military Technical College, Cairo, Egypt, May 2009.

The default case (155 mm M107 projectile: 43 kg, 698 mm, CG 0.459 m
from the nose, Ixx = 0.144 kg.m^2, Iyy = Izz = 1.216 kg.m^2, muzzle
velocity 684.3 m/s, muzzle spin rate 175.48 rps, 44 deg elevation) and the
**aerodynamic coefficient table are Table 1 of that paper**, computed
by the authors with the SPINNER-98 aeroprediction code - not
independently-invented placeholder values.

## Reference frames

- **Inertial frame**: z-up (x = downrange, y = cross-range, z =
  altitude), fixed to the launch point.
- **Non-rolling (aeroballistic) frame**: x-axis aligned with the
  projectile's symmetry axis, but the frame itself does not roll with
  the body. This is the standard formulation used throughout exterior
  ballistics texts (e.g. McCoy, *Modern Exterior Ballistics*) for
  axisymmetric spinning projectiles, because it is mathematically
  equivalent to a fully body-fixed frame (for `Iyy == Izz`, as here)
  while avoiding numerically stiff, spin-frequency coning artifacts in
  the transverse velocity components that a fully body-fixed frame
  would otherwise force onto the integrator. Roll angle (pure spin
  about the symmetry axis) is tracked as a decoupled scalar, since
  aerodynamics are axisymmetric and do not depend on roll orientation.
  The paper's own equations (1)-(4) are given in general body-fixed
  axes (with cross moments of inertia); this project specializes them
  to the non-rolling, `Iyy = Izz` case for tractable integration.

## State vector

12 states: inertial position (3), non-rolling-frame velocity (3), roll
angle + frame pitch/yaw (3), spin rate + frame transverse angular
rates (3).

The translational and rotational equations were verified against the
standard transport theorem (`dV/dt|frame = F/m - Omega x V`) and
Euler's equations (`dH/dt|frame + Omega x H = M`) for a symmetric top,
and the resulting flight independently checked against the paper's
published figures (see Validation below) - this project's first
implementation had two sign errors here (Coriolis terms, and the
overturning-moment direction) that produced an unstable, tumbling
trajectory before being caught by that comparison.

## Aerodynamics

Forces and moments are computed from the total angle of attack between
the relative-wind vector and the symmetry axis, using the paper's
Mach-indexed coefficient table (linearly interpolated in Mach; the
Magnus moment coefficient is additionally tabulated - and bilinearly
interpolated - against total angle of attack, per Table 1's `Cnpalpha`
columns at 0/2/5/10 deg):

- **Axial force (drag)**: `CA` (zero-yaw) + `CA_alpha2 * sin^2(alpha)`.
- **Normal force**: `|CN_alpha| * sin(alpha)`, directed geometrically
  from the relative-wind vector toward the symmetry axis.
- **Magnus force**: `|C_Ypalpha| * (p*d/2V) * sin(alpha)`, perpendicular
  to both the symmetry axis and the relative wind.
- **Overturning (static) moment**: `Cm_alpha * sin(alpha)`, directed to
  *increase* alpha (Cm_alpha is positive in the paper - the shell is
  aerodynamically destabilizing/overturning and relies on gyroscopic,
  not aerodynamic, static stability).
- **Magnus moment**: `Cnpalpha(Mach, alpha) * (p*d/2V)` (no extra
  alpha factor - the paper's table already tabulates the coefficient's
  alpha-dependence directly).
- **Pitch damping moment**: `Cmq * (q or r)`, opposing transverse rates.
- **Spin damping moment**: `Clp * (p*d/2V)`, reduces spin rate over time.

`CN_alpha` and `C_Ypalpha` are negative in the paper's own body-axis
sign convention; this project uses their magnitude, since the physical
force direction is reconstructed geometrically (see
`src/simulator/aero.py` docstring) rather than from a raw body-axis
component. `Cm_alpha`, `Cmq`, `Clp`, and `Cnpalpha` keep their
published sign.

## Atmosphere

US Standard Atmosphere 1976 (troposphere + isothermal lower
stratosphere, 0-20 km), with an optional constant/linearly-sheared
wind field.

## Numerical integration

Two integrator options are provided:

- **RK4**: classical fixed-step 4th-order Runge-Kutta.
- **solve_ivp**: adaptive-step methods from `scipy.integrate`
  (RK45, DOP853, Radau, ...).

Both terminate via a ground-impact event at the configured ground
altitude. The default step size (0.02 s / max_step) resolves the
projectile's fast epicyclic (nutation) mode; a much coarser step will
alias that mode and can produce spurious, unstable-looking results.

## Dispersion sensitivity analysis

A Monte Carlo sweep perturbs muzzle velocity, launch angles, mass,
drag coefficient, and wind within user-specified uncertainty ranges
(Table 2 of the paper), and reports the spread of impact points (mean,
standard deviation, CEP-50). This reproduces the *sensitivity-analysis
methodology* described in the paper; it does **not** compute or
suggest any aim/fire-control correction.

## Validation against the published results

With the default case and Table 1 aero data, this simulator reproduces
the paper's Section 4.3 / Figures 3-10 closely:

| Quantity | Paper | This simulator |
|---|---|---|
| Time of flight | 66.67 s | ~66.4 s |
| Summit altitude / time | ~5750 m / ~31 s | ~5630 m / ~30.5 s |
| Initial axial deceleration | -4.45 g | ~-4.47 g |
| Pitch angle, launch -> impact | 44 deg -> ~-55 deg | 44 deg -> ~-58 deg |
| Max total angle of attack | ~1.3 deg | ~1.7 deg |
| Muzzle -> min -> impact velocity | 684 -> ~250-300 -> ~330 m/s | 684 -> ~253 -> ~329 m/s |

Range (paper's 3D trajectory plot: ~16-17 km) is reproduced to within
about 10-15%; exact agreement isn't expected since the paper's own
figures are read off charts rather than published as tables, and this
project's aerodynamic model (small differences in how `CA_alpha2`,
`Cnpalpha`, and the normal-force direction are combined - see above)
is a defensible but not certified-identical reconstruction of the
paper's own body-fixed-axes equations (1)-(2).

## Limitations

- The aerodynamic coefficients are the paper's own published Table 1
  for a 155 mm M107 shell - not independently validated by this
  project against any other source or real firing data.
- The model does not include Coriolis/Eotvos effects from Earth's
  rotation, projectile flexibility, or base-drag variation with base
  bleed/rocket assist (the paper's own equations (3)-(4) include
  Earth-rotation terms that this project omits for simplicity).
- Range/impact values are close to, but not exact reproductions of,
  the paper's own charts (see Validation table above).
- This tool is for **numerical methods education and published-
  benchmark reproduction only** and must not be used for real-world
  fire-control, targeting, or weapon-deployment purposes.
