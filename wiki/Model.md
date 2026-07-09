# Theoretical Model

RigidFlightLab is an **academic simulation** for **published-benchmark
reproduction and numerical-methods education only**. It is **not
validated for real-world fire-control use** and is not a targeting or
operational artillery tool.

## Scope

The simulator reproduces the theoretical 6-DOF rigid-body model
described in the published paper *"Dispersion Analysis for Spinning
Artillery Projectile"* (see citation in the README), applied to a
representative 155 mm spin-stabilized shell.

## Reference frames

- **Inertial frame**: z-up (x = downrange, y = cross-range, z =
  altitude), fixed to the launch point.
- **Non-rolling (aeroballistic) frame**: x-axis aligned with the
  projectile's symmetry axis, but the frame itself does not roll with
  the body. This is the standard formulation used throughout exterior
  ballistics texts (e.g. McCoy, *Modern Exterior Ballistics*) for
  axisymmetric spinning projectiles, because it is mathematically
  equivalent to a fully body-fixed frame (for `Iyy == Izz`) while
  avoiding numerically stiff, spin-frequency coning artifacts in the
  transverse velocity components. Roll angle (pure spin about the
  symmetry axis) is tracked as a decoupled scalar, since aerodynamics
  are axisymmetric and do not depend on roll orientation.

## State vector

12 states: inertial position (3), non-rolling-frame velocity (3), roll
angle + frame pitch/yaw (3), spin rate + frame transverse angular
rates (3).

## Aerodynamics

Forces and moments are computed from the total angle of attack between
the relative-wind vector and the symmetry axis, using a Mach-indexed
coefficient table (linearly interpolated):

- **Axial force (drag)**: zero-yaw drag plus yaw-induced drag.
- **Normal force (lift)**: proportional to `sin(alpha_total)`.
- **Magnus force/moment**: proportional to spin rate and angle of
  attack, perpendicular to both the symmetry axis and the relative
  wind.
- **Overturning (static) moment**: destabilizing/restoring moment
  proportional to angle of attack.
- **Pitch damping moment**: opposes transverse angular rates.
- **Spin damping moment**: reduces spin rate over time.

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
altitude.

## Dispersion sensitivity analysis

A Monte Carlo sweep perturbs muzzle velocity, launch angles, mass,
drag coefficient, and wind within user-specified uncertainty ranges,
and reports the spread of impact points (mean, standard deviation,
CEP-50). This reproduces the *sensitivity-analysis methodology*
described in the paper; it does **not** compute or suggest any
aim/fire-control correction.

## Limitations

- The bundled aerodynamic coefficient table is a representative
  academic approximation for a 155 mm spin-stabilized shell, not a
  validated match to any specific fielded round.
- The model does not include Coriolis/Eotvos effects from Earth's
  rotation, projectile flexibility, or base-drag variation with base
  bleed/rocket assist.
- Absolute range and impact-point values should be treated as
  illustrative of the underlying numerical methods and qualitative
  physical trends (spin decay, epicyclic pitch/yaw motion, angle-of-
  attack behavior), not as an exact reproduction of any specific
  published trajectory table.
- This tool is for **numerical methods education and published-
  benchmark reproduction only** and must not be used for real-world
  fire-control, targeting, or weapon-deployment purposes.
