# RigidFlightLab

**A 6-DOF spin-stabilized artillery projectile simulator.**

> **Academic simulation only.** Built for numerical-methods education
> and reproducing a published research paper — **not** a fire-control
> system, targeting tool, or operational artillery calculator. No
> target-coordinate input, aim correction, or weapon-deployment
> capability. Not validated for real-world use. See
> [docs/safety.md](docs/safety.md).

## What is this?

RigidFlightLab simulates the full 6-degree-of-freedom flight of a
155&nbsp;mm spin-stabilized shell — position, velocity, spin, and
tumble (pitch/yaw) — from muzzle to impact. It reproduces the model
and results of a real published paper (see [Credits](#credits)), and
comes with a browser-based GUI so you can change the inputs and watch
the trajectory update.

With the default inputs, it matches the paper's own published numbers
closely: **66.4 s flight time** (paper: 66.67 s), **5630 m summit
altitude** (paper: ~5750 m), **-4.47 g initial deceleration** (paper:
-4.45 g). See [docs/model.md](docs/model.md#validation-against-the-published-results)
for the full comparison.

## Quick start

```bash
git clone https://github.com/timeout187/RigidFlightLab.git
cd RigidFlightLab
pip install -r requirements.txt
streamlit run src/gui/app.py
```

That opens a browser tab with the full GUI: edit the projectile,
launch conditions, aerodynamics table, wind, and solver settings, hit
**Run simulation**, and get a 3D trajectory plus time-history plots you
can export as CSV/JSON.

A static, no-install preview of the default trajectory is in
[docs/demo.html](docs/demo.html) — open it in any browser.

## What you can do with it

- Run the paper's own 155 mm example case, or change any input
  (caliber, mass, muzzle velocity, spin rate, elevation angle, wind...)
- Edit the aerodynamic coefficient table directly in the GUI
- Choose between a fixed-step RK4 integrator or adaptive
  `scipy.integrate.solve_ivp` methods
- Run a Monte Carlo dispersion sweep over 8 uncertainty parameters
  (muzzle velocity, mass, inertia, wind, etc.) to see impact-point
  spread
- Export any run as CSV or JSON

## Learn more

- [docs/model.md](docs/model.md) - reference frames, aerodynamics,
  numerical integration, and the validation table above in full.
- [docs/equations.md](docs/equations.md) - the paper's actual
  equations of motion, transcribed, with the exact mapping to the code.
- [docs/safety.md](docs/safety.md) - the full academic-use statement.

## Project layout

```
src/simulator/   the physics: equations of motion, atmosphere, aero data, integrators
src/gui/         the Streamlit app
src/data/        default 155 mm example case
docs/            model write-up, safety notes, static demo page
wiki/            same docs, staged for the GitHub Wiki
tests/           automated tests (25, all passing)
examples/        two ready-to-run example scripts
```

## Limitations

- The aerodynamic data is the paper's own published table for one
  specific shell — not independently re-validated against other
  sources or live firing data.
- Range/impact numbers are close to, but not an exact match for, the
  paper's own charts (see the validation table linked above).
- Full details in [docs/model.md](docs/model.md#limitations).

## Credits

**Based on the published paper:**

> Khalil, M., Abdalla, H., and Kamal, O., *"Dispersion Analysis for
> Spinning Artillery Projectile"*, 13th International Conference on
> Aerospace Sciences & Aviation Technology (ASAT-13), Paper
> ASAT-13-FM-03, Military Technical College, Cairo, Egypt, May 2009.

This project reimplements that paper's 6-DOF equations of motion,
155&nbsp;mm M107 example case, and Table 1 aerodynamic coefficients as
open-source, runnable code with an interactive GUI — full credit for
the underlying research and data to Mostafa Khalil, H. Abdalla, and
Osama Kamal.

**Built with:** [Streamlit](https://streamlit.io),
[Plotly](https://plotly.com), [SciPy](https://scipy.org), and
[NumPy](https://numpy.org).

**Project by [Hasan Ahmed](https://github.com/timeout187)**, built
with Claude.

## License

[MIT](LICENSE) — see the license file for the full text and copyright.
