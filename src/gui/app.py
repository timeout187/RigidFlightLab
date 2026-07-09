"""Streamlit GUI for the 6-DOF spin-stabilized projectile simulator.

ACADEMIC SIMULATION - published benchmark reproduction and numerical-
methods education only. NOT for operational use. NOT validated for
real-world fire-control. Contains no target-coordinate input, aim
correction, weapon-deployment advice, or artillery table generation.
"""
from __future__ import annotations

import io
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd
import plotly.graph_objects as go
import streamlit as st

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from src.data.default_case import (  # noqa: E402
    DispersionSettings, InitialConditions, ProjectileProperties, SimulationCase, SolverSettings,
)
from src.simulator.aero import default_155mm_aero_table  # noqa: E402
from src.simulator.atmosphere import WindModel  # noqa: E402
from src.simulator.dispersion import run_dispersion_analysis  # noqa: E402
from src.simulator.dynamics import DynamicsModel  # noqa: E402
from src.simulator.integrators import run_trajectory  # noqa: E402

st.set_page_config(page_title="RigidFlightLab - 6-DOF Spin Projectile Simulator", layout="wide")

st.title("RigidFlightLab: 6-DOF Spin-Stabilized Projectile Simulator")
st.caption(
    "Academic simulation for published benchmark reproduction and numerical-methods "
    "education only. Not for operational use. Not validated for real-world fire-control. "
    "This tool has no target-coordinate input, aim-correction, or fire-control capability."
)

with st.sidebar:
    st.header("Projectile physical properties")
    caliber_mm = st.number_input("Caliber (mm)", value=155.0, step=1.0)
    length_mm = st.number_input("Length (mm)", value=698.0, step=1.0)
    mass_kg = st.number_input("Mass (kg)", value=43.0, step=0.1)
    cg_m = st.number_input("CG from nose tip (m)", value=0.459, step=0.001, format="%.3f")
    ixx = st.number_input("Ixx - axial moment of inertia (kg.m^2)", value=0.144, step=0.001, format="%.3f")
    iyy = st.number_input("Iyy = Izz - transverse moment of inertia (kg.m^2)", value=1.216, step=0.001, format="%.3f")

    st.header("Initial conditions")
    v0 = st.number_input("Muzzle velocity (m/s)", value=684.3, step=1.0)
    spin0_rps = st.number_input("Muzzle spin rate (rev/s)", value=175.48, step=0.1)
    elev_deg = st.number_input("Firing elevation angle (deg)", value=44.0, step=0.1)
    azim_deg = st.number_input("Azimuth angle (deg)", value=0.0, step=0.1)
    launch_alt = st.number_input("Launch altitude (m)", value=0.0, step=1.0)

    st.header("Atmosphere / wind")
    wind_n = st.number_input("Wind - north component (m/s)", value=0.0, step=0.5)
    wind_e = st.number_input("Wind - east component (m/s)", value=0.0, step=0.5)
    wind_shear = st.number_input("Wind shear (per km altitude)", value=0.0, step=0.05)

    st.header("Numerical solver settings")
    method = st.selectbox("Integration method", ["RK45", "RK4", "DOP853", "Radau"], index=0)
    t_max = st.number_input("Max flight time (s)", value=90.0, step=5.0)
    max_step = st.number_input("Max/fixed step size (s)", value=0.02, step=0.001, format="%.4f")
    rtol = st.number_input("Relative tolerance", value=1e-6, format="%.1e")

    st.header("Dispersion / uncertainty parameters")
    run_dispersion = st.checkbox("Run dispersion sensitivity sweep", value=False)
    n_samples = st.number_input("Monte Carlo samples", value=200, step=10, min_value=10)
    v0_std = st.number_input("Muzzle velocity std dev (m/s)", value=2.0, step=0.1)
    elev_std = st.number_input("Elevation angle std dev (deg)", value=0.05, step=0.01, format="%.3f")


def build_case() -> SimulationCase:
    return SimulationCase(
        projectile=ProjectileProperties(
            caliber_m=caliber_mm / 1000.0,
            length_m=length_mm / 1000.0,
            mass_kg=mass_kg,
            cg_from_nose_m=cg_m,
            ixx_kg_m2=ixx,
            iyy_kg_m2=iyy,
            izz_kg_m2=iyy,
        ),
        initial_conditions=InitialConditions(
            muzzle_velocity_mps=v0,
            muzzle_spin_rate_rps=spin0_rps,
            elevation_angle_deg=elev_deg,
            azimuth_angle_deg=azim_deg,
            launch_altitude_m=launch_alt,
        ),
        solver=SolverSettings(
            method=method,
            fixed_step_s=max_step,
            max_step_s=max_step,
            rtol=rtol,
            atol=rtol,
            t_max_s=t_max,
        ),
        dispersion=DispersionSettings(
            n_samples=int(n_samples),
            muzzle_velocity_std_mps=v0_std,
            elevation_angle_std_deg=elev_std,
        ),
        wind=WindModel(wind_north_mps=wind_n, wind_east_mps=wind_e, shear_per_km=wind_shear),
        aero_table=default_155mm_aero_table(),
    )


st.subheader("Aerodynamic coefficient table (Mach-indexed, default academic values)")
st.caption(
    "Default values are Table 1 from Khalil, Abdalla & Kamal (2009), \"Dispersion "
    "Analysis for Spinning Artillery Projectile\" (155 mm M107, computed with "
    "SPINNER-98) - see docs/model.md for the column mapping and sign conventions."
)
aero_default = default_155mm_aero_table()
aero_df = pd.DataFrame({
    "Mach": aero_default.mach,
    "CA (Cd0)": aero_default.cd0,
    "CA_alpha2 (Cd_alpha2)": aero_default.cd_alpha2,
    "CN_alpha (Cl_alpha)": aero_default.cl_alpha,
    "Cmq": aero_default.cmq,
    "Cm_alpha": aero_default.cm_alpha,
    "Clp (Cspin)": aero_default.cspin,
    "CYpalpha (Cmag_force)": aero_default.cmag_f,
    "Cnpalpha_0deg": aero_default.cnpalpha_table[:, 0],
    "Cnpalpha_2deg": aero_default.cnpalpha_table[:, 1],
    "Cnpalpha_5deg": aero_default.cnpalpha_table[:, 2],
    "Cnpalpha_10deg": aero_default.cnpalpha_table[:, 3],
})
edited_aero_df = st.data_editor(aero_df, num_rows="dynamic")

run_clicked = st.button("Run simulation", type="primary")

if run_clicked:
    case = build_case()
    case.aero_table.mach = edited_aero_df["Mach"].to_numpy()
    case.aero_table.cd0 = edited_aero_df["CA (Cd0)"].to_numpy()
    case.aero_table.cd_alpha2 = edited_aero_df["CA_alpha2 (Cd_alpha2)"].to_numpy()
    case.aero_table.cl_alpha = edited_aero_df["CN_alpha (Cl_alpha)"].to_numpy()
    case.aero_table.cmq = edited_aero_df["Cmq"].to_numpy()
    case.aero_table.cm_alpha = edited_aero_df["Cm_alpha"].to_numpy()
    case.aero_table.cspin = edited_aero_df["Clp (Cspin)"].to_numpy()
    case.aero_table.cmag_f = edited_aero_df["CYpalpha (Cmag_force)"].to_numpy()
    case.aero_table.cnpalpha_table = edited_aero_df[
        ["Cnpalpha_0deg", "Cnpalpha_2deg", "Cnpalpha_5deg", "Cnpalpha_10deg"]
    ].to_numpy()

    model = DynamicsModel(case)
    with st.spinner("Integrating 6-DOF equations of motion..."):
        result = run_trajectory(model)
    data = result.as_dict()

    st.success(
        f"Integration finished ({result.message}). "
        f"Time of flight: {data['t'][-1]:.2f} s, range: {data['x'][-1]:.1f} m."
    )

    col1, col2 = st.columns(2)
    with col1:
        st.metric("Time of flight (s)", f"{data['t'][-1]:.2f}")
        st.metric("Range (m)", f"{data['x'][-1]:.1f}")
    with col2:
        st.metric("Cross-range (m)", f"{data['y'][-1]:.2f}")
        st.metric("Max altitude (m)", f"{np.max(data['z']):.1f}")

    st.subheader("3D trajectory")
    fig3d = go.Figure(data=[go.Scatter3d(
        x=data["x"], y=data["y"], z=data["z"], mode="lines", line=dict(width=4),
    )])
    fig3d.update_layout(scene=dict(xaxis_title="Downrange (m)", yaxis_title="Cross-range (m)",
                                    zaxis_title="Altitude (m)"), height=600)
    st.plotly_chart(fig3d, use_container_width=True)

    vel_mag = np.sqrt(data["u"] ** 2 + data["v"] ** 2 + data["w"] ** 2)
    accel_axial = np.gradient(data["u"], data["t"])
    accel_normal = np.gradient(np.sqrt(data["v"] ** 2 + data["w"] ** 2), data["t"])

    plots = [
        ("Altitude vs flight time", data["t"], data["z"], "Time (s)", "Altitude (m)"),
        ("Velocity magnitude vs flight time", data["t"], vel_mag, "Time (s)", "Velocity (m/s)"),
        ("Axial acceleration vs flight time", data["t"], accel_axial, "Time (s)", "Axial accel (m/s^2)"),
        ("Normal acceleration vs flight time", data["t"], accel_normal, "Time (s)", "Normal accel (m/s^2)"),
        ("Pitch angle vs flight time", data["t"], data["theta_deg"], "Time (s)", "Pitch angle (deg)"),
        ("Spin rate vs flight time", data["t"], data["p_rps"], "Time (s)", "Spin rate (rev/s)"),
        ("Total angle of attack vs flight time", data["t"], data["alpha_deg"], "Time (s)", "Alpha total (deg)"),
    ]
    cols = st.columns(2)
    for i, (title, xdat, ydat, xlabel, ylabel) in enumerate(plots):
        fig = go.Figure(data=go.Scatter(x=xdat, y=ydat, mode="lines"))
        fig.update_layout(title=title, xaxis_title=xlabel, yaxis_title=ylabel, height=350)
        cols[i % 2].plotly_chart(fig, use_container_width=True)

    st.subheader("Impact point summary")
    st.write(
        f"Impact at downrange = {data['x'][-1]:.1f} m, cross-range = {data['y'][-1]:.2f} m, "
        f"time of flight = {data['t'][-1]:.2f} s. (Educational reproduction only - not a "
        "fire-control solution.)"
    )

    df_export = pd.DataFrame(data)
    csv_buf = io.StringIO()
    df_export.to_csv(csv_buf, index=False)
    st.download_button("Export trajectory as CSV", csv_buf.getvalue(), file_name="trajectory.csv")
    st.download_button(
        "Export trajectory as JSON",
        json.dumps({k: v.tolist() for k, v in data.items()}),
        file_name="trajectory.json",
    )

    if run_dispersion:
        st.subheader("Dispersion sensitivity analysis (Monte Carlo)")
        with st.spinner("Running dispersion sweep..."):
            disp_result = run_dispersion_analysis(case)
        summary = disp_result.summary()
        st.json(summary)

        fig_disp = go.Figure(data=go.Scatter(
            x=disp_result.impact_x, y=disp_result.impact_y, mode="markers",
            marker=dict(size=5, opacity=0.6),
        ))
        fig_disp.update_layout(
            title="Impact point dispersion (downrange vs cross-range)",
            xaxis_title="Downrange (m)", yaxis_title="Cross-range (m)", height=500,
        )
        st.plotly_chart(fig_disp, use_container_width=True)

        disp_df = pd.DataFrame({
            "impact_x_m": disp_result.impact_x,
            "impact_y_m": disp_result.impact_y,
            "time_of_flight_s": disp_result.time_of_flight_s,
        })
        disp_csv = io.StringIO()
        disp_df.to_csv(disp_csv, index=False)
        st.download_button("Export dispersion samples as CSV", disp_csv.getvalue(), file_name="dispersion.csv")

st.divider()
st.caption(
    "RigidFlightLab is an offline educational simulator for reproducing published academic "
    "6-DOF results and teaching numerical methods. It is not a fire-control system, targeting "
    "tool, or operational artillery calculator, and provides no aim solutions or weapon-"
    "deployment guidance."
)
