"""6-DOF rigid-body equations of motion for a spin-stabilized
projectile, following the standard aeroballistic formulation (axial
drag, normal-force lift, Magnus force/moment, overturning moment, pitch
and spin damping moments) used in the published dispersion-analysis
literature this project reproduces.

Academic simulation component - published benchmark reproduction and
numerical-methods education only. NOT a fire-control or targeting tool
and NOT validated for real-world operational use.

The rotational/translational equations are expressed in the standard
"non-rolling" (aeroballistic) reference frame used throughout the
exterior-ballistics literature for symmetric spinning projectiles
(e.g. McCoy, "Modern Exterior Ballistics"): the frame's x-axis tracks
the projectile's symmetry axis (pitch/yaw only), but the frame itself
does not roll with the body. This is mathematically equivalent to the
full body-fixed-axes formulation for an axisymmetric body (Iyy = Izz)
but avoids resolving the very high-frequency (~ spin rate) coning of
the transverse velocity components that a fully body-fixed frame would
introduce, which otherwise forces an impractically small integration
step. Roll angle (pure spin about the symmetry axis) is tracked as an
independent, decoupled scalar since it does not feed back into the
aerodynamics of an axisymmetric body.

State vector (12 elements), all in SI units:
    [x, y, z,          inertial position (x=downrange, y=cross-range, z=altitude, up+)
     u, v, w,          velocity components in the non-rolling frame
     phi, theta, psi,  body roll angle (phi, decoupled) and non-rolling
                       frame orientation (theta=pitch, psi=yaw), radians
     p, q, r]          spin rate (p) and non-rolling-frame transverse
                       angular rates (q, r), rad/s
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from src.simulator.atmosphere import WindModel, standard_atmosphere
from src.data.default_case import ProjectileProperties, SimulationCase

STATE_SIZE = 12
GRAVITY_MPS2 = 9.80665


def body_to_inertial_dcm(phi: float, theta: float, psi: float) -> np.ndarray:
    """3-2-1 (yaw-pitch-roll) direction cosine matrix, frame -> inertial,
    for a z-up inertial frame (x=downrange, y=cross-range, z=altitude).

    theta is nose-up pitch angle (positive theta tilts the x-axis
    toward +z, i.e. climbing flight). phi is the roll angle; for the
    non-rolling frame itself phi is always passed as 0 (see
    `non_rolling_frame_dcm`) while the body's true roll is tracked
    separately as a decoupled state.
    """
    cphi, sphi = math.cos(phi), math.sin(phi)
    cth, sth = math.cos(theta), math.sin(theta)
    cpsi, spsi = math.cos(psi), math.sin(psi)

    return np.array([
        [cth * cpsi, -cpsi * sth * sphi - spsi * cphi, -cpsi * sth * cphi + spsi * sphi],
        [cth * spsi, -spsi * sth * sphi + cpsi * cphi, -spsi * sth * cphi - cpsi * sphi],
        [sth, cth * sphi, cth * cphi],
    ])


def non_rolling_frame_dcm(theta: float, psi: float) -> np.ndarray:
    """DCM for the non-rolling (aeroballistic) frame -> inertial, i.e.
    `body_to_inertial_dcm` with roll fixed at zero."""
    return body_to_inertial_dcm(0.0, theta, psi)


@dataclass
class DynamicsModel:
    case: SimulationCase

    def state_derivative(self, t: float, state: np.ndarray) -> np.ndarray:
        proj: ProjectileProperties = self.case.projectile
        aero = self.case.aero_table
        wind: WindModel = self.case.wind

        x, y, z = state[0:3]
        u, v, w = state[3:6]
        phi, theta, psi = state[6:9]
        p, q, r = state[9:12]

        dcm = non_rolling_frame_dcm(theta, psi)
        atmo = standard_atmosphere(z)

        wind_n, wind_e, wind_d = wind.wind_at(z)
        v_inertial = dcm @ np.array([u, v, w])
        v_air_inertial = v_inertial - np.array([wind_n, wind_e, -wind_d])
        v_air_frame = dcm.T @ v_air_inertial

        vel_mag = float(np.linalg.norm(v_air_frame))
        vel_mag_safe = max(vel_mag, 1e-3)
        mach = vel_mag / atmo.speed_of_sound_mps
        coeffs = aero.coefficients(mach)

        e_v = v_air_frame / vel_mag_safe
        e_x = np.array([1.0, 0.0, 0.0])
        cos_alpha = float(np.clip(np.dot(e_x, e_v), -1.0, 1.0))
        alpha_total = math.acos(cos_alpha)

        cross_xv = np.cross(e_x, e_v)
        cross_norm = float(np.linalg.norm(cross_xv))
        if cross_norm > 1e-9:
            m_dir = cross_xv / cross_norm  # magnus/overturning moment axis
            lift_dir = np.cross(e_v, cross_xv) / cross_norm  # in-plane, toward nose
        else:
            m_dir = np.zeros(3)
            lift_dir = np.zeros(3)

        d = proj.caliber_m
        area = math.pi / 4.0 * d ** 2
        q_dyn = 0.5 * atmo.density_kg_m3 * vel_mag_safe ** 2

        cd = coeffs["cd0"] + coeffs["cd_alpha2"] * math.sin(alpha_total) ** 2
        drag_force = -q_dyn * area * cd * e_v

        lift_force = q_dyn * area * coeffs["cl_alpha"] * math.sin(alpha_total) * lift_dir

        spin_factor = p * d / (2.0 * vel_mag_safe)
        magnus_force = q_dyn * area * coeffs["cmag_f"] * spin_factor * math.sin(alpha_total) * m_dir

        gravity_inertial = np.array([0.0, 0.0, -GRAVITY_MPS2])
        gravity_frame = dcm.T @ gravity_inertial

        aero_force_frame = drag_force + lift_force + magnus_force
        total_force_frame = aero_force_frame + proj.mass_kg * gravity_frame

        overturning_moment = q_dyn * area * d * coeffs["cm_alpha"] * math.sin(alpha_total) * m_dir
        magnus_moment = q_dyn * area * d * coeffs["cmag_m"] * spin_factor * math.sin(alpha_total) * m_dir
        spin_damping_moment = np.array([
            q_dyn * area * d * (d * p / (2.0 * vel_mag_safe)) * coeffs["cspin"], 0.0, 0.0,
        ])
        pitch_damp_coeff = q_dyn * area * d ** 2 / (2.0 * vel_mag_safe) * coeffs["cmq"]
        pitch_damping_moment = np.array([0.0, pitch_damp_coeff * q, pitch_damp_coeff * r])

        total_moment_frame = (
            overturning_moment + magnus_moment + spin_damping_moment + pitch_damping_moment
        )

        m = proj.mass_kg
        # Coriolis terms use the non-rolling frame's own angular velocity
        # (0, q, r) - the spin rate p does not enter the translational
        # equations directly in this frame.
        u_dot = q * w - r * v + total_force_frame[0] / m
        v_dot = r * u + total_force_frame[1] / m
        w_dot = -q * u + total_force_frame[2] / m

        ixx, iyy = proj.ixx_kg_m2, proj.iyy_kg_m2
        # Symmetric-top (Iyy == Izz) gyroscopic equations in the
        # non-rolling frame.
        p_dot = total_moment_frame[0] / ixx
        q_dot = (total_moment_frame[1] - ixx * p * r) / iyy
        r_dot = (total_moment_frame[2] + ixx * p * q) / iyy

        pos_dot = dcm @ np.array([u, v, w])
        # Frame orientation (theta, psi) evolves from the transverse
        # rates only; body roll angle phi accumulates spin independently.
        cth = math.cos(theta)
        if abs(cth) < 1e-6:
            cth = math.copysign(1e-6, cth) if cth != 0 else 1e-6
        euler_dot = np.array([p, -q, r / cth])

        deriv = np.empty(STATE_SIZE)
        deriv[0:3] = pos_dot
        deriv[3:6] = [u_dot, v_dot, w_dot]
        deriv[6:9] = euler_dot
        deriv[9:12] = [p_dot, q_dot, r_dot]
        return deriv

    def initial_state(self) -> np.ndarray:
        ic = self.case.initial_conditions
        elev = math.radians(ic.elevation_angle_deg)
        azim = math.radians(ic.azimuth_angle_deg)

        v0 = ic.muzzle_velocity_mps
        state = np.zeros(STATE_SIZE)
        state[0:3] = [0.0, 0.0, ic.launch_altitude_m]
        state[3:6] = [v0, 0.0, 0.0]  # velocity along the symmetry axis at launch (no initial AoA)
        state[6:9] = [0.0, elev, azim]
        state[9:12] = [2.0 * math.pi * ic.muzzle_spin_rate_rps, 0.0, 0.0]
        return state


def hit_ground_event(t: float, state: np.ndarray, ground_altitude_m: float = 0.0) -> float:
    return state[2] - ground_altitude_m


hit_ground_event.terminal = True
hit_ground_event.direction = -1
