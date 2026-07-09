# RigidFlightLab

**six-dof-spin-projectile-simulator**

An academic simulation of a 155 mm spin-stabilized artillery
projectile, producing full 6-DOF (six degrees of freedom) rigid-body
trajectory calculations. Built for **published-benchmark reproduction**
and **numerical-methods education only** — see
[docs/safety.md](docs/safety.md).

> **Not for operational use. Not validated for real-world fire-
> control.** This project has no target-coordinate input, aim
> correction, weapon-deployment advice, live-fire recommendation,
> artillery table generation, or range/accuracy optimization.

## Purpose

This tool reproduces the theoretical 6-DOF model, example parameters,
**and aerodynamic coefficient table (Table 1)** described in the
published paper *"Dispersion Analysis for Spinning Artillery
Projectile"* (see [Citation](#citation)), for PhD-level study of
rigid-body flight dynamics and numerical integration methods. With the
default case, this simulator reproduces the paper's own published
flight-time, summit altitude, deceleration, and pitch/velocity curves
closely - see [Validation](docs/model.md#validation-against-the-published-results).

## Demo

A static, non-interactive demo of the nominal example case (charts
only, no input forms) is in [docs/demo.html](docs/demo.html) —
open it directly in a browser, or enable GitHub Pages
(Settings -> Pages -> deploy from `/docs`) to host it at
`https://timeout187.github.io/RigidFlightLab/demo.html`.

## Screenshots

_placeholder — run the GUI locally (`streamlit run src/gui/app.py`) to
see the full input forms; the 3D trajectory and time-history plots
match [docs/demo.html](docs/demo.html)._

## Installation

```bash
git clone https://github.com/timeout187/RigidFlightLab.git
cd RigidFlightLab
pip install -r requirements.txt
```

Requires Python 3.10+.

## Running the GUI

```bash
streamlit run src/gui/app.py
```

The GUI loads the default academic example case (155 mm, 684.3 m/s
muzzle velocity, 175.48 rev/s spin rate, 44 deg elevation) and lets you
edit:

- Projectile physical properties (caliber, length, mass, CG, moments
  of inertia)
- Initial conditions (muzzle velocity, spin rate, elevation/azimuth,
  launch altitude)
- Aerodynamic coefficient table (Mach-indexed, editable)
- Atmosphere/wind model
- Numerical solver settings (RK4 or scipy `solve_ivp` methods)
- Dispersion/uncertainty parameters

Click **Run simulation** to integrate the trajectory and view:

- 3D trajectory path
- Altitude, velocity, axial/normal acceleration, pitch angle, spin
  rate, and total angle of attack vs. flight time
- Impact point summary
- Optional Monte Carlo dispersion sensitivity plots

Results can be exported as CSV or JSON.

## Running the examples

```bash
python -m examples.nominal_run          # nominal academic example case
python -m examples.dispersion_example   # dispersion sensitivity sweep
```

## Running the tests

```bash
python -m pytest tests/ -q
```

## Model overview

See [docs/model.md](docs/model.md) for the full theoretical model:
reference frames, equations of motion, aerodynamic model, atmosphere
model, and numerical integration methods.

## Repository layout

```
src/simulator/   equations of motion, atmosphere, aero interpolation, integrators
src/gui/         Streamlit GUI (input forms, plots, run button, export)
src/data/        default academic example data
docs/            theoretical model, equation references, safety statement, static demo page
wiki/            wiki-page content, staged here until the GitHub Wiki is enabled
tests/           unit tests for atmosphere, aero interpolation, integration, sanity
examples/        nominal case and dispersion sensitivity example scripts
```

## Limitations

- The aerodynamic coefficient table is the paper's own published
  Table 1 for a 155 mm M107 shell, not independently validated by this
  project against any other source or real firing data.
- Range/impact values are close to, but not exact reproductions of,
  the paper's own charts - see [docs/model.md](docs/model.md#validation-against-the-published-results).
- See [docs/model.md](docs/model.md#limitations) for the full list.

## Citation

If you use this software, please cite the source paper:

> Khalil, M., Abdalla, H., and Kamal, O., "Dispersion Analysis for
> Spinning Artillery Projectile", 13th International Conference on
> Aerospace Sciences & Aviation Technology (ASAT-13), Paper
> ASAT-13-FM-03, Military Technical College, Cairo, Egypt, May 2009.

This project reproduces that paper's theoretical 6-DOF model, 155 mm
M107 example case, and Table 1 aerodynamic coefficients.

## Safety disclaimer

This is an **academic simulation** for **published-benchmark
reproduction** and **numerical methods education only**. It is **not a
fire-control system**, **not a targeting tool**, and **not an
operational artillery calculator**. See [docs/safety.md](docs/safety.md)
for the full statement.

## License

[MIT](LICENSE).
