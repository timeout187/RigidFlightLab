# FAQ

**Is this a fire-control or targeting tool?**
No. See [Safety](Safety.md). It has no target-coordinate input, aim
correction, or weapon-deployment capability, and is not validated for
real-world use.

**Why don't my ranges match a real 155 mm firing table?**
The bundled aerodynamic coefficient table is a representative academic
approximation, not a validated match to any specific fielded round.
See [Model Overview — Limitations](Model.md#limitations).

**Why a "non-rolling frame" instead of full body-fixed axes?**
For an axisymmetric spinning projectile (`Iyy == Izz`), the non-rolling
(aeroballistic) frame is mathematically equivalent to full body-fixed
axes but avoids numerically resolving the very high-frequency (~spin
rate) coning of the transverse velocity components, which would
otherwise force an impractically small integration step. This is the
standard approach in the exterior-ballistics literature (e.g. McCoy,
*Modern Exterior Ballistics*).

**Can I use a different aero coefficient table?**
Yes — the Streamlit GUI's aerodynamic coefficient table is editable,
or you can construct your own `AeroTable` in
`src/simulator/aero.py`.

**Which integrators are available?**
Fixed-step classical RK4, and any `scipy.integrate.solve_ivp` method
(RK45, DOP853, Radau, ...) via the solver-settings dropdown in the GUI.
