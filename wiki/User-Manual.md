# RigidFlightLab — User Manual

**For PhD, MSc, and research students.** This manual covers the theory
behind the simulator, how to use it, a complete reference of every
module/class/function in the codebase, and how to modify the project
and contribute changes back through GitHub.

> **Academic simulation only.** Published-benchmark reproduction and
> numerical-methods education. Not a fire-control system, not a
> targeting tool, not validated for real-world use. See
> [safety.md](safety.md).

---

## Table of contents

1. [Scope and audience](#1-scope-and-audience)
2. [Theory](#2-theory)
   - [2.1 Why 6-DOF, and why this projectile](#21-why-6-dof-and-why-this-projectile)
   - [2.2 Reference frames](#22-reference-frames)
   - [2.3 Equations of motion](#23-equations-of-motion)
   - [2.4 Aerodynamics](#24-aerodynamics)
   - [2.5 Atmosphere model](#25-atmosphere-model)
   - [2.6 Numerical integration](#26-numerical-integration)
   - [2.7 Dispersion / Monte Carlo sensitivity analysis](#27-dispersion--monte-carlo-sensitivity-analysis)
   - [2.8 Validation against the published paper](#28-validation-against-the-published-paper)
3. [Using the simulator](#3-using-the-simulator)
   - [3.1 Installation](#31-installation)
   - [3.2 The GUI, section by section](#32-the-gui-section-by-section)
   - [3.3 Reading the outputs](#33-reading-the-outputs)
   - [3.4 Running from the command line](#34-running-from-the-command-line)
   - [3.5 Running the tests](#35-running-the-tests)
4. [Code reference](#4-code-reference)
   - [4.1 Repository map](#41-repository-map)
   - [4.2 `src/simulator/atmosphere.py`](#42-srcsimulatoratmospherepy)
   - [4.3 `src/simulator/aero.py`](#43-srcsimulatoraeropy)
   - [4.4 `src/simulator/dynamics.py`](#44-srcsimulatordynamicspy)
   - [4.5 `src/simulator/integrators.py`](#45-srcsimulatorintegratorspy)
   - [4.6 `src/simulator/dispersion.py`](#46-srcsimulatordispersionpy)
   - [4.7 `src/data/default_case.py`](#47-srcdatadefault_casepy)
   - [4.8 `src/gui/app.py`](#48-srcguiapppy)
5. [Modifying the project](#5-modifying-the-project)
   - [5.1 Local workflow with git and GitHub](#51-local-workflow-with-git-and-github)
   - [5.2 Recipe: change the aerodynamic table](#52-recipe-change-the-aerodynamic-table)
   - [5.3 Recipe: add a new output plot](#53-recipe-add-a-new-output-plot)
   - [5.4 Recipe: simulate a different projectile](#54-recipe-simulate-a-different-projectile)
   - [5.5 Recipe: add a new integrator method](#55-recipe-add-a-new-integrator-method)
   - [5.6 Recipe: add a new dispersion parameter](#56-recipe-add-a-new-dispersion-parameter)
   - [5.7 Writing and running tests for your change](#57-writing-and-running-tests-for-your-change)
6. [Glossary](#6-glossary)

---

## 1. Scope and audience

This project reproduces the 6-DOF (six degree of freedom) rigid-body
flight dynamics model and results of:

> Khalil, M., Abdalla, H., and Kamal, O., *"Dispersion Analysis for
> Spinning Artillery Projectile"*, 13th International Conference on
> Aerospace Sciences & Aviation Technology (ASAT-13), Paper
> ASAT-13-FM-03, Military Technical College, Cairo, Egypt, May 2009.

It is written for graduate students and researchers who want to:

- **Understand** how a spin-stabilized projectile's 6-DOF flight is
  modeled and numerically integrated (this manual's Theory section).
- **Use** the simulator to run the paper's own case, or their own
  variations, through a GUI or scripts (Using the simulator).
- **Read and modify** the actual implementation — every function is
  documented here with what it does and why (Code reference,
  Modifying the project).

It assumes undergraduate-level rigid-body dynamics and ODE numerical
methods; it does not assume prior Python or git experience — Section 5
walks through both from first principles.

---

## 2. Theory

### 2.1 Why 6-DOF, and why this projectile

A spin-stabilized artillery shell does not fly like a point mass. Its
nose wobbles slightly around the velocity vector throughout flight (an
**epicyclic** motion made of a fast and a slow oscillation), and this
wobble changes the drag and lift the shell experiences, which changes
the trajectory. A 6-DOF model tracks all of this explicitly:

- 3 **translational** degrees of freedom: position (and hence
  velocity) of the center of gravity.
- 3 **rotational** degrees of freedom: the body's orientation (and
  hence angular rates) — roll (spin), pitch, and yaw.

A simpler **point-mass** or **modified point-mass** model (used for
most firing tables) approximates the epicyclic effect with a
correction factor. A full 6-DOF model computes it directly from the
projectile's mass properties and aerodynamic coefficients, which is
why the paper — and this project — uses one.

The specific projectile is a **155 mm M107** high-explosive shell,
because that's the paper's own worked example, with real,
published aerodynamic data (not a generic approximation) to validate
the whole simulator against.

### 2.2 Reference frames

Three frames matter here:

1. **Inertial (Earth-fixed) frame**: `x` = downrange, `y` = cross-range,
   `z` = altitude (positive up). Fixed to the launch point. A flat,
   non-rotating Earth is assumed (see §2.8/limitations — the paper's
   own model includes Earth's rotation and ellipsoidal shape; this
   project omits that for the range/altitude regime of the example
   case).

2. **Body-fixed frame**: rotates *with* the projectile, including its
   roll. This is what the paper's own equations (1)-(2) use.

3. **Non-rolling ("aeroballistic") frame**: this project's actual
   working frame. Its `x`-axis tracks the projectile's *symmetry axis*
   (nose direction) — so it pitches and yaws with the body — but it
   does **not roll** with it. Roll (pure spin about the symmetry axis)
   is tracked as an independent number instead.

**Why not just use the body-fixed frame like the paper?** For an
axisymmetric shell (`Iyy = Izz`, true here), the non-rolling frame is
*mathematically exactly equivalent* to the body-fixed frame — nothing
is lost. But the body-fixed frame's translational equations pick up
extremely fast terms from the roll rate (this shell spins at
175 revolutions **per second** at launch — about 1100 rad/s). A
fixed-step integrator would need a timestep well under a millisecond
just to avoid aliasing that spin frequency, which is needlessly slow.
The non-rolling frame removes the roll rate from the translational
equations entirely (see the derivation in §2.3), so the integrator
only has to resolve the much slower ~20 Hz epicyclic (nutation) mode —
about 50-100x more forgiving.

### 2.3 Equations of motion

The paper's own equations (1)-(4) are transcribed verbatim, with the
code's exact implementation and any sign/convention differences, in
**[docs/equations.md](equations.md)** — read that alongside this
section for the full derivation. Summary:

**Translational** (non-rolling frame), from the transport theorem:

```
dV/dt |frame = F/m - Omega x V,     Omega = (0, q, r),  V = (u, v, w)
```

expanding the cross product gives:

```
u_dot = -q*w + r*v + Fx/m
v_dot = -r*u        + Fy/m
w_dot =  q*u         + Fz/m
```

Note there is **no `p`** (spin rate) in these — that's the payoff of
the non-rolling frame described above.

**Rotational** (Euler's equations for a symmetric top, `Iyy = Izz`,
non-rolling frame angular velocity `Omega = (0, q, r)`, true angular
velocity `(p, q, r)`):

```
p_dot = Mx / Ixx
q_dot = (My - Ixx*p*r) / Iyy
r_dot = (Mz + Ixx*p*q) / Iyy
```

The `Ixx*p*r` / `Ixx*p*q` terms are the **gyroscopic coupling** — they
are what turn an aerodynamically *destabilizing* moment (see §2.4)
into bounded precession instead of a runaway tumble, provided the spin
rate is high enough (the **gyroscopic stability factor**, `Sg > 1`).
This coupling is the mathematical heart of why spin-stabilization
works at all, and getting its sign right was the single most important
correctness issue in building this simulator (see §2.8).

**Kinematics** (how the frame's own orientation, `theta` = pitch,
`psi` = yaw, evolves from `q, r`; the frame doesn't roll so there's no
`phi` kinematics equation for it — roll is just `phi_dot = p`):

```
theta_dot = -q
psi_dot   =  r / cos(theta)
```

All of this lives in `src/simulator/dynamics.py` — see §4.4.

### 2.4 Aerodynamics

At each integration step, the simulator computes the **total angle of
attack** `alpha` — the angle between the relative wind and the
symmetry axis — and looks up Mach-indexed coefficients from
**Table 1** of the source paper (see §4.3) to compute:

| Force/moment | Formula | Physical meaning |
|---|---|---|
| Axial force (drag) | `q_dyn * A * (CA + CA_alpha2*sin^2(alpha))` | opposes motion; grows with yaw |
| Normal force | `q_dyn * A * |CN_alpha| * sin(alpha)` | "lift" from angle of attack |
| Magnus force | `q_dyn * A * |C_Ypalpha| * (p*d/2V) * sin(alpha)` | sideways force from spin + yaw |
| Overturning moment | `-q_dyn * A * d * Cm_alpha * sin(alpha)` | tries to *increase* alpha (destabilizing) |
| Magnus moment | `-q_dyn * A * d * Cnpalpha(Mach, alpha) * (p*d/2V)` | spin-induced moment |
| Pitch damping moment | `q_dyn * A * d^2/(2V) * Cmq * (q or r)` | opposes rotation rate |
| Spin damping moment | `q_dyn * A * d * (p*d/2V) * Clp` | slows the spin over time |

where `q_dyn = 0.5 * rho * V^2` (dynamic pressure), `A` is the
projectile's cross-sectional area, `d` is the caliber.

**The key physical fact**: `Cm_alpha` is **positive** in the paper —
meaning the shell is *aerodynamically unstable* on its own (any angle
of attack tends to grow, not shrink). It only flies straight because
of gyroscopic stabilization (§2.3). This is completely normal and
expected for a spin-stabilized (as opposed to fin-stabilized)
projectile, and it's why this whole exercise is nontrivial: get the
gyroscopic-coupling sign wrong, and the simulated shell will tumble
instead of fly (again, see §2.8).

### 2.5 Atmosphere model

The **US Standard Atmosphere (1976)** model gives temperature,
pressure, density, and speed of sound as functions of altitude (the
troposphere's linear lapse rate below 11 km, isothermal above). An
optional wind field can be constant or linearly sheared with altitude.
See §4.2.

### 2.6 Numerical integration

Two integrators are available:

- **RK4**: classical fixed-step 4th-order Runge-Kutta.
- **`scipy.integrate.solve_ivp`**: adaptive-step methods (RK45,
  DOP853, Radau, ...).

**Why the timestep matters so much here**: the shell's fast epicyclic
(nutation) mode has a natural frequency of roughly `Ixx*p / Iyy`
radians/second — around 130 rad/s (≈20 Hz) at launch. An integrator
step that's too coarse relative to that frequency doesn't just lose
accuracy, it can go numerically unstable and produce a trajectory that
*looks* like a real physical tumble but is actually an artifact of
under-resolving that mode. The default settings (`max_step = 0.02 s`,
`rtol = atol = 1e-6`) were chosen specifically to resolve this mode
correctly — see the runnable check in §5.7.

### 2.7 Dispersion / Monte Carlo sensitivity analysis

The source paper's **Table 2** lists eight production/launch
uncertainty parameters (firing pitch angle, projectile mass, axial and
lateral moments of inertia, muzzle velocity, muzzle spin rate, wind
speed and direction) and their expected ranges. The paper's own method
sweeps each parameter *individually* (holding the rest fixed) and
plots the resulting range/drift error (its Figures 11-18).

This project instead draws all eight **simultaneously** from
independent Gaussians (the paper's own stated range used as an
approximate one-standard-deviation width) for a **joint Monte Carlo**
sweep, and reports the resulting impact-point spread (mean, standard
deviation, CEP-50). This is a legitimate and commonly used dispersion
methodology (one of the paper's own cited references uses it), but it
does not reproduce the paper's individual-parameter figures curve for
curve — see `src/simulator/dispersion.py` and §4.6.

### 2.8 Validation against the published paper

With the default 155 mm case, this simulator reproduces the paper's
Section 4.3 published results closely:

| Quantity | Paper | This simulator |
|---|---|---|
| Time of flight | 66.67 s | ~66.4 s |
| Summit altitude / time | ~5750 m / ~31 s | ~5630 m / ~30.5 s |
| Initial axial deceleration | -4.45 g | ~-4.47 g |
| Pitch angle, launch → impact | 44° → ~-55° | 44° → ~-58° |
| Max total angle of attack | ~1.3° | ~1.7° |
| Muzzle → min → impact velocity | 684 → ~250-300 → ~330 m/s | 684 → ~253 → ~329 m/s |

**A worked note on debugging this**, useful reading for anyone
modifying the dynamics: the first implementation of this simulator had
**two sign errors** — the translational Coriolis terms (§2.3) were
backwards, and the overturning moment (§2.4) was applied in the
aerodynamically *restoring* direction rather than the paper's
destabilizing one. Both bugs individually looked physically plausible
(the trajectory integrated fine, gave numbers, didn't crash), and
together they made the shell tumble uncontrollably after 30-50 seconds
of flight instead of flying a normal arc. They were only caught by
building the exact comparison table above and noticing the flight time
was off by a factor of 3 and the trajectory shape was wrong. **The
takeaway for anyone extending this code**: a 6-DOF simulation that
runs without crashing and produces plausible-looking individual
numbers is not the same as a *correct* one — always validate against
an independent published result if one exists, not just against your
own intuition about "does this look right."

---

## 3. Using the simulator

### 3.1 Installation

```bash
git clone https://github.com/timeout187/RigidFlightLab.git
cd RigidFlightLab
pip install -r requirements.txt
streamlit run src/gui/app.py
```

Requires Python 3.10+. This opens the GUI in your browser at
`http://localhost:8501`.

### 3.2 The GUI, section by section

The sidebar has six input groups (all map directly to
`src/data/default_case.py` dataclasses, §4.7):

1. **Projectile physical properties** — caliber, length, mass, center
   of gravity, and moments of inertia (`ProjectileProperties`).
2. **Initial conditions** — muzzle velocity, spin rate, elevation and
   azimuth angles, launch altitude (`InitialConditions`).
3. **Atmosphere / wind** — constant or altitude-sheared wind
   (`WindModel`).
4. **Numerical solver settings** — integrator choice (RK4 or a
   `solve_ivp` method), step size, tolerance, max flight time
   (`SolverSettings`). See §2.6 before loosening these.
5. **Dispersion / uncertainty parameters** — the eight Table 2
   parameters for the Monte Carlo sweep (`DispersionSettings`), plus a
   checkbox to run it (it's off by default since it runs the full
   trajectory many times and is much slower than a single run).
6. **Aerodynamic coefficient table** — the main panel has an editable
   table of the Mach-indexed coefficients from Table 1 (§2.4), so you
   can substitute your own projectile's data.

Click **Run simulation**. The GUI then integrates the trajectory and
shows the outputs described next.

### 3.3 Reading the outputs

- **3D trajectory**: the flight path in downrange/cross-range/altitude
  space. Note the **drift** (nonzero cross-range) — this is the real,
  expected sideways deflection from gyroscopic precession, not a bug.
- **Altitude, velocity, axial/normal acceleration, pitch angle, spin
  rate, total angle of attack vs. flight time**: the same six
  quantities the source paper plots (its Figures 4-10), for direct
  comparison.
- **Impact point summary**: range, cross-range, time of flight.
- **Dispersion scatter plot** (if enabled): each point is one Monte
  Carlo sample's impact point; the summary reports mean/std/CEP-50.

All of the above can be exported as **CSV or JSON** from the GUI.

### 3.4 Running from the command line

```bash
python -m examples.nominal_run          # the paper's own example case
python -m examples.dispersion_example   # a dispersion sweep, 100 samples
```

Or script it directly:

```python
from src.data.default_case import default_case
from src.simulator.dynamics import DynamicsModel
from src.simulator.integrators import run_trajectory

case = default_case()
case.initial_conditions.elevation_angle_deg = 30.0   # change anything
model = DynamicsModel(case)
result = run_trajectory(model)
print(result.t[-1], result.state[-1, 0])  # time of flight, range
```

### 3.5 Running the tests

```bash
python -m pytest tests/ -q
```

25 tests, covering atmosphere, aero interpolation, integrators,
dispersion, and sanity/validation checks against the paper's published
numbers (§2.8). Takes about 45 seconds — most of that is genuinely
integrating full ~66 s trajectories, not test overhead.

---

## 4. Code reference

### 4.1 Repository map

```
src/simulator/atmosphere.py   US Standard Atmosphere 1976 + wind model
src/simulator/aero.py         Table 1 aero coefficients + interpolation
src/simulator/dynamics.py     the 6-DOF equations of motion (the physics core)
src/simulator/integrators.py  RK4 and solve_ivp wrappers, ground-impact event
src/simulator/dispersion.py   Monte Carlo dispersion sweep
src/data/default_case.py      all input dataclasses + the paper's default values
src/gui/app.py                the Streamlit GUI
tests/                        unit + validation tests
examples/                     two ready-to-run scripts
docs/                         this manual, model.md, equations.md, safety.md
```

### 4.2 `src/simulator/atmosphere.py`

| Name | Kind | Purpose |
|---|---|---|
| `standard_atmosphere(altitude_m)` | function | Returns an `AtmosphereState` (temperature, pressure, density, speed of sound) for a given altitude, per the US Standard Atmosphere 1976 (troposphere + isothermal lower stratosphere). |
| `AtmosphereState` | dataclass | Plain container for the four values above. |
| `WindModel` | dataclass | `wind_north_mps`, `wind_east_mps`, `reference_altitude_m`, `shear_per_km`. |
| `WindModel.wind_at(altitude_m)` | method | Returns `(wind_north, wind_east, wind_down)` at a given altitude, linearly scaling the horizontal wind by `shear_per_km` relative to `reference_altitude_m`. |

### 4.3 `src/simulator/aero.py`

| Name | Kind | Purpose |
|---|---|---|
| `AeroTable` | dataclass | Holds the Mach-indexed 1-D coefficient arrays (`cd0`, `cd_alpha2`, `cl_alpha`, `cmq`, `cm_alpha`, `cspin`, `cmag_f`) plus the 2-D `cnpalpha_table` (Mach × angle-of-attack grid). |
| `AeroTable.coefficients(mach, alpha_deg)` | method | Returns a `dict` of all coefficients interpolated at the given Mach number (and, for `cnpalpha`, bilinearly at the given angle of attack too). This is the **only** entry point `dynamics.py` uses. |
| `AeroTable._interp` / `_cnpalpha` | private methods | 1-D linear and bilinear interpolation helpers, respectively. |
| `default_155mm_aero_table()` | function | Returns an `AeroTable` populated with the source paper's actual Table 1 data. **This is the function to copy/modify if you want to simulate a different projectile** — see §5.2. |

### 4.4 `src/simulator/dynamics.py`

This is the physics core — read §2.2-2.3 first.

| Name | Kind | Purpose |
|---|---|---|
| `body_to_inertial_dcm(phi, theta, psi)` | function | The 3-2-1 direction cosine matrix for a z-up, nose-up-positive-pitch frame. Used both for the full body-fixed orientation (with roll `phi`) and, with `phi` fixed at 0, for the non-rolling frame. |
| `non_rolling_frame_dcm(theta, psi)` | function | Convenience wrapper: `body_to_inertial_dcm(0, theta, psi)`. |
| `DynamicsModel` | dataclass | Wraps a `SimulationCase` (§4.7) and exposes the two functions `solve_ivp`/RK4 actually need. |
| `DynamicsModel.state_derivative(t, state)` | method | **The entire equations of motion.** Given the current 12-element state, computes: atmosphere and wind at the current altitude, relative wind and Mach number, angle of attack and the `m_dir`/`lift_dir` geometric axes, all aerodynamic forces/moments (§2.4), gravity in the frame, the Coriolis-corrected translational accelerations, the gyroscopic-coupled angular accelerations, and the Euler kinematics — and returns the 12-element derivative. This is the function to read line-by-line to understand the model, and the one to modify to change the physics. |
| `DynamicsModel.initial_state()` | method | Builds the 12-element initial state vector from `SimulationCase.initial_conditions`. |
| `hit_ground_event(t, state, ground_altitude_m)` | function | A `solve_ivp`-style event function (zero-crossing at ground impact, `terminal=True`, `direction=-1`) also used to end the fixed-step RK4 loop. |

**State vector layout** (12 elements, all functions agree on this
order): `[x, y, z, u, v, w, phi, theta, psi, p, q, r]` — inertial
position, non-rolling-frame velocity, roll angle + frame pitch/yaw,
spin rate + frame transverse angular rates. See the module docstring
for full units and sign conventions.

### 4.5 `src/simulator/integrators.py`

| Name | Kind | Purpose |
|---|---|---|
| `TrajectoryResult` | dataclass | `t`, `state` (N×12 array), `mach`, `alpha_deg` time histories, plus `success`/`message` from the solver. |
| `TrajectoryResult.as_dict()` | method | Unpacks the raw state array into a friendly `dict` of named 1-D arrays (`x, y, z, u, v, w, phi_deg, theta_deg, ..., p_rps, q_deg_s, r_deg_s, mach, alpha_deg`) — what the GUI and demo page actually plot. |
| `run_rk4(model, t_max, dt)` | function | Fixed-step classical RK4, with linear interpolation to land exactly on the ground-impact altitude rather than overshooting by up to one timestep. |
| `run_solve_ivp(model, t_max, method, max_step, rtol, atol)` | function | Wraps `scipy.integrate.solve_ivp` with the ground-impact event attached. |
| `run_trajectory(model)` | function | Dispatches to one of the above based on `model.case.solver.method` (`"RK4"` vs. any `solve_ivp` method name). **This is the one entry point the GUI, examples, and tests actually call.** |
| `_post_process(model, t, state)` | private function | Recomputes Mach number and angle of attack along a finished trajectory (used by both integrators so the logic lives in one place). |

### 4.6 `src/simulator/dispersion.py`

| Name | Kind | Purpose |
|---|---|---|
| `DispersionResult` | dataclass | Arrays of impact `x`, `y`, range, and time-of-flight across all Monte Carlo samples. |
| `DispersionResult.summary()` | method | Returns mean/std range and cross-range, CEP-50, and sample count as a `dict`. |
| `run_dispersion_analysis(base_case)` | function | Draws all eight Table 2 uncertainty parameters (§2.7) as independent Gaussians, runs a full trajectory per sample via `run_trajectory`, and collects the impact points. `base_case.dispersion.n_samples` controls how many; each sample is a full, independent 6-DOF integration, so this scales linearly in cost. |

### 4.7 `src/data/default_case.py`

All the input dataclasses, and the paper's own default values.

| Name | Kind | Purpose |
|---|---|---|
| `ProjectileProperties` | dataclass | `caliber_m, length_m, mass_kg, cg_from_nose_m, ixx_kg_m2, iyy_kg_m2, izz_kg_m2`. Defaults = the paper's 155 mm M107 values. |
| `InitialConditions` | dataclass | `muzzle_velocity_mps, muzzle_spin_rate_rps, elevation_angle_deg, azimuth_angle_deg, launch_altitude_m`. |
| `SolverSettings` | dataclass | `method, fixed_step_s, max_step_s, rtol, atol, t_max_s, ground_altitude_m`. See §2.6 before changing `max_step_s`. |
| `DispersionSettings` | dataclass | The eight Table 2 parameters (§2.7) plus `n_samples` and `random_seed`. |
| `SimulationCase` | dataclass | Bundles all of the above plus `WindModel` and `AeroTable` into the one object every function in `simulator/` actually takes. |
| `default_case()` | function | Returns a `SimulationCase()` with every field at its paper-matched default. **The starting point for almost everything** — copy it, then mutate the fields you want to change (see the CLI example in §3.4). |

### 4.8 `src/gui/app.py`

Not a library — a single Streamlit script, read top to bottom:

1. Sidebar `st.number_input`/`st.selectbox`/`st.checkbox` widgets for
   every field in §4.7's dataclasses.
2. `build_case()` — assembles a `SimulationCase` from the current
   widget values.
3. An editable `st.data_editor` table for the aero coefficients
   (§4.3), including the four `Cnpalpha` alpha-columns.
4. On **Run simulation**: builds the case, calls `run_trajectory`,
   unpacks `result.as_dict()`, and renders the 3D trajectory + six
   time-history Plotly charts, plus CSV/JSON export buttons.
5. If the dispersion checkbox is on: calls `run_dispersion_analysis`
   and renders the impact-point scatter plot and summary.

---

## 5. Modifying the project

### 5.1 Local workflow with git and GitHub

If you're new to git, here is the complete loop:

```bash
# 1. Get the code
git clone https://github.com/timeout187/RigidFlightLab.git
cd RigidFlightLab

# 2. Make a branch for your change (never commit straight to main)
git checkout -b my-change

# 3. Edit files, then check what you changed
git status
git diff

# 4. Run the tests before committing (see §3.5)
python -m pytest tests/ -q

# 5. Stage and commit
git add path/to/changed_file.py
git commit -m "Short, specific description of the change"

# 6. Push your branch to GitHub
git push -u origin my-change
```

Then on **github.com/timeout187/RigidFlightLab**, GitHub will show a
banner offering to open a **Pull Request** from your branch. Open one,
describe what you changed and why (link back to the relevant section
of this manual if useful), and it can be reviewed and merged into
`main` from the GitHub web UI — no further command-line steps needed.

If you'd rather edit directly on GitHub without cloning: open any file
→ pencil (edit) icon → GitHub will automatically create a branch and a
pull request for you when you commit. This works fine for markdown/doc
changes; for code changes you'll want to run the tests locally first
(GitHub's web editor can't do that for you).

### 5.2 Recipe: change the aerodynamic table

To simulate a different projectile's aerodynamics:

1. Open `src/simulator/aero.py`.
2. Either edit the arrays inside `default_155mm_aero_table()` directly,
   or write a new function (e.g. `my_projectile_aero_table()`) that
   returns an `AeroTable` with your own `mach`, `cd0`, `cd_alpha2`,
   `cl_alpha`, `cmq`, `cm_alpha`, `cspin`, `cmag_f` arrays and your own
   `alpha_grid_deg` / `cnpalpha_table`.
3. In `src/data/default_case.py`, change `SimulationCase.aero_table`'s
   `default_factory` to call your new function (or just pass
   `aero_table=my_projectile_aero_table()` when constructing a case in
   your own script — you don't have to change the shipped default).
4. **Check your sign conventions against §2.4** before trusting
   results — a positive vs. negative `Cm_alpha` is the difference
   between a stable and a tumbling shell.

The GUI's aero table editor also lets you do this interactively for
one-off experiments, without touching any code.

### 5.3 Recipe: add a new output plot

1. Make sure the quantity you want is in `TrajectoryResult.as_dict()`
   (§4.5) — if not, add it there (and to the state derivative in
   `dynamics.py` if it's a genuinely new physical quantity, not just a
   derived one).
2. In `src/gui/app.py`, add another `(title, x, y, xlabel, ylabel)`
   tuple to the `plots` list, following the existing pattern.
3. If you also want it on the static demo page, add the equivalent
   entry to the `channels` array in `docs/demo.html`'s `<script>`.

### 5.4 Recipe: simulate a different projectile

Combine §5.2 (aero table) with new values in `ProjectileProperties`
and `InitialConditions` (§4.7) — either edit `default_case()`'s
defaults directly, or construct your own `SimulationCase` in a script:

```python
from src.data.default_case import SimulationCase, ProjectileProperties, InitialConditions
from src.simulator.aero import my_projectile_aero_table  # your function from §5.2

case = SimulationCase(
    projectile=ProjectileProperties(caliber_m=0.105, mass_kg=15.0, ...),
    initial_conditions=InitialConditions(muzzle_velocity_mps=800.0, ...),
    aero_table=my_projectile_aero_table(),
)
```

### 5.5 Recipe: add a new integrator method

`run_trajectory` (§4.5) already dispatches any `solver.method` string
that isn't `"RK4"` straight to `scipy.integrate.solve_ivp(method=...)`
— so any of scipy's built-in methods (`"BDF"`, `"LSODA"`, ...) already
work with no code changes; just set `case.solver.method`. To add a
genuinely custom integrator, write a new `run_my_method(model, t_max,
...)` function in `integrators.py` following the same signature
pattern as `run_rk4`, and add a branch for it in `run_trajectory`.

### 5.6 Recipe: add a new dispersion parameter

1. Add a new field (with a sensible default) to `DispersionSettings`
   in `default_case.py`.
2. In `run_dispersion_analysis` (`dispersion.py`), draw a random sample
   for it (`rng.normal(0.0, disp.your_new_field, n)`) and apply it to
   the per-sample `case` before calling `run_trajectory`.
3. Add the corresponding input widget in the GUI's "Dispersion /
   uncertainty parameters" sidebar section.

### 5.7 Writing and running tests for your change

Tests live in `tests/`, one file per module (`test_atmosphere.py`,
`test_aero.py`, `test_integrators.py`, `test_dispersion.py`,
`test_sanity.py`). Add new test functions (`def test_...():`) to the
relevant file, or a new file following the same `test_*.py` pattern —
`pytest` picks them up automatically.

If you change anything in `dynamics.py`, at minimum re-run
`test_sanity.py`'s validation tests (§2.8) — they will catch a
regression back toward instability/tumbling immediately (that's
exactly what they're there for; see the worked debugging note in
§2.8). A quick manual check for "is my change still numerically
stable":

```python
from src.data.default_case import default_case
from src.simulator.dynamics import DynamicsModel
from src.simulator.integrators import run_trajectory
import numpy as np

result = run_trajectory(DynamicsModel(default_case()))
assert np.max(result.alpha_deg) < 5.0   # should stay small (paper: ~1.3 deg peak)
assert 60 < result.t[-1] < 73           # should land around 66-67 s
```

---

## 6. Glossary

| Term | Meaning |
|---|---|
| **6-DOF** | Six degrees of freedom: 3 translational (position) + 3 rotational (orientation). |
| **Angle of attack (alpha)** | Angle between the relative wind and the projectile's symmetry (nose) axis. |
| **Epicyclic motion** | The combined fast + slow oscillation of a spinning projectile's nose around its velocity vector. |
| **Gyroscopic stability factor (Sg)** | A dimensionless number; `Sg > 1` means spin is enough to keep an aerodynamically-unstable shell's motion bounded. |
| **Non-rolling / aeroballistic frame** | A reference frame that pitches/yaws with the projectile's nose but does not roll with it — see §2.2. |
| **Magnus effect** | A sideways force/moment on a spinning body moving through air at an angle of attack. |
| **Yaw of repose** | The small, persistent equilibrium angle of attack a spin-stabilized shell settles into, which causes it to drift sideways (in the spin direction) over a long flight. |
| **CEP-50** | Circular Error Probable at 50%: the radius of a circle, centered on the mean impact point, containing half the impact points in a dispersion sample. |
| **Dynamic pressure (q_dyn)** | `0.5 * rho * V^2` — scales all aerodynamic forces and moments. |
