# Equations of Motion

This page is the mathematical reference for RigidFlightLab: the
paper's own published equations, and how this project's code
implements them. Notation follows the paper's own nomenclature
(Section 1, p. 1-2/12).

**Source**: Khalil, M., Abdalla, H., and Kamal, O., *"Dispersion
Analysis for Spinning Artillery Projectile"*, ASAT-13-FM-03, Military
Technical College, Cairo, Egypt, May 2009. Equations (1)-(4), p. 4/12.

## Nomenclature (paper's notation)

| Symbol | Meaning |
|---|---|
| `[u v w]` | body-frame velocity components, m/s |
| `p, q, r` | body-frame roll, pitch, yaw rates, rad/s |
| `phi, theta, psi` | roll (bank), pitch (inclination), yaw (azimuth) angles |
| `[Tx Ty Tz]` | resultant external force in the body-fixed frame, N |
| `Ix, Iy, Iz` | axial and transverse moments of inertia, kg.m^2 |
| `Ixy, Iyz, Izx` | products of inertia, kg.m^2 |
| `alpha` | angle of attack |
| `beta` | angle of sideslip |
| `L, M, N` | roll, pitch, yaw moments |
| `g` | normal gravity |
| `CA` | total axial force coefficient |
| `CA_alpha2` | second-order axial force coefficient |
| `CN_alpha` | normal force coefficient derivative with angle of attack |
| `C_Ypalpha` | Magnus force coefficient derivative |
| `Clp` | roll-damping coefficient derivative |
| `Cm_alpha` | pitching moment coefficient derivative with angle of attack |
| `Cmq` | pitching moment coefficient derivative with pitch rate |
| `Cnpalpha` | Magnus moment coefficient derivative |
| `M` (italic) | Mach number |

## Equation (1) - translational (trajectory)

```
[ u_dot ]       [ Tx - A_axial ]        [   -sin(theta)     ]   [ p_B^E + p ]   [ u ]
[ v_dot ] = 1/m [ Ty + A_side  ] + g *  [ cos(theta)sin(phi) ] + [ q_B^E + q ] x [ v ]
[ w_dot ]       [ Tz - A_normal]        [ cos(theta)cos(phi) ]   [ r_B^E + r ]   [ w ]
```

`A_axial`, `A_side`, `A_normal` are the aerodynamic force components;
`[p_B^E q_B^E r_B^E]` is the Earth's angular velocity resolved into the
body frame (Earth-rotation effect - see equation (3)).

**This project**: implements the same physics in the *non-rolling*
frame instead of the full body-fixed frame (see docs/model.md for why),
which for an axisymmetric body (`Iy = Iz`) is an exact reformulation.
The transport-theorem form used in `src/simulator/dynamics.py` is:

```
dV/dt |non-rolling frame = F/m - Omega x V,   Omega = (0, q, r)
```

Earth's rotation terms are omitted (see Limitations in docs/model.md).

## Equation (2) - rotational (Euler's equations)

```
p_dot = L/Ix + Izx(r_dot + p.q)/Ix + (Iy - Iz).q.r / Ix
q_dot = M/Iy + [Izx(r^2 - p^2) + (Iz - Ix).r.p] / Iy
r_dot = N/Iz + Izx(p_dot - q.r)/Iz + (Ix - Iy).p.q / Iz
```

For this project's axisymmetric case (`Izx = 0`, `Iy = Iz`), Omega =
(0, q, r) for the non-rolling frame, and the general rigid-body law
`dH/dt|frame + Omega x H = M` (with `H = (Ix.p, Iy.q, Iy.r)`) reduces
to exactly:

```
p_dot = Mx / Ix
q_dot = (My - Ix.p.r) / Iy
r_dot = (Mz + Ix.p.q) / Iy
```

which is what `src/simulator/dynamics.py` implements.

## Equation (3) - Earth's rotation effect

```
[P]   [p]         [ (omega_E + mu_dot).cos(phi)  ]
[Q] = [q] - L_BE * [        -lambda_dot           ]
[R]   [r]         [ -(omega_E + mu_dot).sin(phi)  ]
```

Not implemented in this project (see Limitations) - the simulator uses
a flat, non-rotating Earth, appropriate for the range/altitude regime
of this example case.

## Equation (4) - body-to-Earth transformation matrix

```
        [ cos(theta)cos(psi)                                   cos(theta)sin(psi)                                  -sin(theta)       ]
L_BE  = [ sin(phi)sin(theta)cos(psi) - cos(phi)sin(psi)          sin(phi)sin(theta)sin(psi) + cos(phi)cos(psi)        sin(phi)cos(theta) ]
        [ cos(phi)sin(theta)cos(psi) + sin(phi)sin(psi)          cos(phi)sin(theta)sin(psi) - sin(phi)cos(psi)        cos(phi)cos(theta) ]
```

**This project**: `body_to_inertial_dcm()` in `src/simulator/dynamics.py`
implements the equivalent transform for a z-up (altitude-positive)
inertial frame with a nose-up-positive pitch convention, which flips
some signs relative to the paper's own (implicitly NED-style, z-down)
convention above. See the docstring there for the exact derivation.

## Aerodynamic forces and moments

**Important provenance note**: the paper's own equations (1)-(2)
leave the aerodynamic terms `A_axial`, `A_side`, `A_normal`, `L`, `M`,
`N` as generic symbols - it does not spell out how they're computed
from the Table 1 coefficients (`CA`, `CN_alpha`, `Cm_alpha`, etc.).
Table 1 (Section 4.2) gives the *coefficient data*; the algebraic
formulas below, connecting that data to actual forces and moments, are
**standard aeroballistics practice (e.g. McCoy's textbook treatment),
not text or formulas printed in the paper itself**. Only the
coefficient *values* in Table 1 and the projectile/launch parameters
in Section 4.1 are taken directly from the paper.

Given the total angle of attack `alpha` (angle between the relative
wind and the symmetry axis) and dynamic pressure `q_dyn = 0.5 * rho *
V^2`, reference area `A = (pi/4) d^2`, reference length `d` (caliber):

```
Axial force        = q_dyn * A * [ CA + CA_alpha2 * sin^2(alpha) ]
Normal force        = q_dyn * A * |CN_alpha| * sin(alpha)
Magnus force        = q_dyn * A * |C_Ypalpha| * (p.d / 2V) * sin(alpha)
Overturning moment   = -q_dyn * A * d * Cm_alpha * sin(alpha)          [see sign note below]
Magnus moment        = -q_dyn * A * d * Cnpalpha(M, alpha) * (p.d / 2V)
Pitch damping moment  = q_dyn * A * d^2/(2V) * Cmq * (q or r)
Spin damping moment   = q_dyn * A * d * (p.d / 2V) * Clp
```

`Cnpalpha` is looked up by bilinear interpolation over the paper's own
(Mach, alpha) grid (Table 1's four alpha columns: 0, 2, 5, 10 deg)
rather than treated as a constant per-radian slope, matching how the
paper itself tabulates it.

**Sign note**: `CN_alpha` and `C_Ypalpha` are negative in the paper's
own body-axis convention; this project uses their magnitude since the
force direction is reconstructed geometrically (see `aero.py`
docstring). The overturning and Magnus moments carry an explicit minus
sign here because of how this project's moment-axis vector (`e_x` cross
`e_v`) is defined - the important physical fact, preserved exactly, is
that `Cm_alpha`'s published *positive* sign is aerodynamically
destabilizing (the shell relies on gyroscopic, not aerodynamic, static
stability), which is what makes the gyroscopic-precession terms in
Euler's equations above load-bearing for flight stability.

## Where the numbers come from

- Default projectile/launch parameters: paper Section 4.1 -
  `src/data/default_case.py`.
- Aerodynamic coefficient table: paper Table 1, Section 4.2 -
  `src/simulator/aero.py`.
- Dispersion uncertainty parameters: paper Table 2, Section 4.4 -
  `DispersionSettings` in `src/data/default_case.py`.
