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
and educational output plots described in the published paper
*"Dispersion Analysis for Spinning Artillery Projectile"* (see
[Citation](#citation)), for PhD-level study of rigid-body flight
dynamics and numerical integration methods.

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

- The bundled aerodynamic coefficient table is a representative
  academic approximation, not a validated match to any specific
  fielded 155 mm round.
- Absolute range/impact values are illustrative of the underlying
  numerical methods and qualitative physics, not an exact reproduction
  of any specific published firing table.
- See [docs/model.md](docs/model.md#limitations) for the full list.

## Citation

If you use this software, please cite the source paper:

> "Dispersion Analysis for Spinning Artillery Projectile" (uploaded
> reference paper providing the theoretical 6-DOF model, example
> parameters, and aerodynamic coefficient data reproduced here).

## Safety disclaimer

This is an **academic simulation** for **published-benchmark
reproduction** and **numerical methods education only**. It is **not a
fire-control system**, **not a targeting tool**, and **not an
operational artillery calculator**. See [docs/safety.md](docs/safety.md)
for the full statement.

## License

[MIT](LICENSE).
