# This is the main simulation page for PhysiKit
# It is identical to your previous app.py

import streamlit as st
import matplotlib.pyplot as plt
import plotly.graph_objs as go
import importlib
import numpy as np

def reset_nbody_defaults():
    st.session_state["n"] = 2
    for i in range(5):
        st.session_state[f"mass_{i}"] = 1.0
        st.session_state[f"x_{i}"] = 10.0 * i
        st.session_state[f"y_{i}"] = 0.0
        st.session_state[f"vx_{i}"] = 0.0
        st.session_state[f"vy_{i}"] = 0.0

def reset_defaults(system):
    if system == "Spring-Mass":
        st.session_state["m"] = 1.0
        st.session_state["k"] = 1.0
        st.session_state["x0"] = 1.0
        st.session_state["v0"] = 0.0
        st.session_state["c"] = 0.0
    elif system == "Double Pendulum":
        st.session_state["m1"] = 1.0
        st.session_state["m2"] = 1.0
        st.session_state["l1"] = 1.0
        st.session_state["l2"] = 1.0
        st.session_state["theta1"] = 90.0
        st.session_state["theta2"] = 90.0
        st.session_state["z1"] = 0.0
        st.session_state["z2"] = 0.0
        st.session_state["g"] = 9.81
    elif system == "Projectile Motion":
        st.session_state["v0"] = 10.0
        st.session_state["angle"] = 45.0
        st.session_state["mass"] = 1.0
        st.session_state["drag"] = 0.1
        st.session_state["g"] = 9.81
    elif system == "N-body Simulation":
        st.session_state["n"] = 2
        for i in range(5):
            st.session_state[f"mass_{i}"] = 1.0
            st.session_state[f"x_{i}"] = 10.0 * i
            st.session_state[f"y_{i}"] = 0.0
            st.session_state[f"vx_{i}"] = 0.0
            st.session_state[f"vy_{i}"] = 0.0
        st.session_state["G"] = 6.67430e-11
    elif system == "Rigidbody Collisions":
        st.session_state["restitution"] = 1.0
    elif system == "Rotational Dynamics":
        st.session_state["I"] = 1.0
        st.session_state["omega0"] = 10.0
        st.session_state["torque"] = 0.0
    elif system == "Orbital Mechanics":
        st.session_state["m1"] = 1.0
        st.session_state["m2"] = 1.0
        st.session_state["r1"] = 1.0
        st.session_state["r2"] = 2.0
        st.session_state["v1"] = 1.0
        st.session_state["G"] = 6.67430e-11

SYSTEMS = {
    "Spring-Mass": {
        "module": "systems.spring_mass",
        "simulate": "simulate_spring_mass",
        "sidebar": lambda st: {
            "m": st.sidebar.number_input("Mass (kg)", value=1.0, step=0.0001, format="%.4f"),
            "k": st.sidebar.number_input("Spring Constant k (N/m)", value=1.0, step=0.0001, format="%.4f"),
            "x0": st.sidebar.number_input("Initial Position (m)", value=1.0, step=0.0001, format="%.4f"),
            "v0": st.sidebar.number_input("Initial Velocity (m/s)", value=0.0, step=0.0001, format="%.4f"),
            "c": st.sidebar.number_input("Damping Coefficient c (Ns/m)", value=0.0, step=0.0001, format="%.4f")
        },
        "plot": lambda result, st: plot_spring_mass(result, st)
    },
    "Double Pendulum": {
        "module": "systems.double_pendulum",
        "simulate": "simulate_double_pendulum",
        "sidebar": lambda st: {
            "m1": st.sidebar.number_input("Mass 1 (kg)", value=1.0, step=0.0001, format="%.4f"),
            "m2": st.sidebar.number_input("Mass 2 (kg)", value=1.0, step=0.0001, format="%.4f"),
            "l1": st.sidebar.number_input("Rod Length 1 (m)", value=1.0, step=0.0001, format="%.4f"),
            "l2": st.sidebar.number_input("Rod Length 2 (m)", value=1.0, step=0.0001, format="%.4f"),
            "theta1": np.deg2rad(st.sidebar.number_input("Initial Angle 1 (deg)", value=90.0, step=0.0001, format="%.4f")),
            "theta2": np.deg2rad(st.sidebar.number_input("Initial Angle 2 (deg)", value=90.0, step=0.0001, format="%.4f")),
            "z1": st.sidebar.number_input("Initial Angular Velocity 1 (rad/s)", value=0.0, step=0.0001, format="%.4f"),
            "z2": st.sidebar.number_input("Initial Angular Velocity 2 (rad/s)", value=0.0, step=0.0001, format="%.4f"),
            "g": st.sidebar.number_input("Gravity (m/s²)", value=9.81, step=0.0001, format="%.4f")
        },
        "plot": lambda result, st: plot_double_pendulum(result, st)
    },
    "Projectile Motion": {
        "module": "systems.projectile_motion",
        "simulate": "simulate_projectile_motion",
        "sidebar": lambda st: {
            "v0": st.sidebar.number_input("Initial Velocity (m/s)", value=10.0, step=0.0001, format="%.4f"),
            "angle": st.sidebar.number_input("Launch Angle (deg)", value=45.0, step=0.0001, format="%.4f"),
            "mass": st.sidebar.number_input("Mass (kg)", value=1.0, step=0.0001, format="%.4f"),
            "drag": st.sidebar.number_input("Drag Coefficient", value=0.1, step=0.0001, format="%.4f"),
            "g": st.sidebar.number_input("Gravity (m/s²)", value=9.81, step=0.0001, format="%.4f")
        },
        "plot": lambda result, st: plot_projectile_motion(result, st)
    },
    "N-body Simulation": {
        "module": "systems.nbody",
        "simulate": "simulate_nbody",
        "sidebar": lambda st: (
            lambda n: {
                "n": n,
                "G": st.sidebar.number_input("Gravitational Constant", value=6.67430e-11, step=1e-14, format="%.14f"),
                "masses": np.array([
                    st.sidebar.number_input(f"Mass {i+1} (kg)", value=1.0, step=0.0001, format="%.4f", key=f"mass_{i}")
                    for i in range(n)
                ]),
                "positions": np.array([
                    [
                        st.sidebar.number_input(f"x{i+1} (m)", value=10.0 * i, step=0.0001, format="%.4f", key=f"x_{i}"),
                        st.sidebar.number_input(f"y{i+1} (m)", value=0.0, step=0.0001, format="%.4f", key=f"y_{i}")
                    ] for i in range(n)
                ]),
                "velocities": np.array([
                    [
                        st.sidebar.number_input(f"vx{i+1} (m/s)", value=0.0, step=0.0001, format="%.4f", key=f"vx_{i}"),
                        st.sidebar.number_input(f"vy{i+1} (m/s)", value=0.0, step=0.0001, format="%.4f", key=f"vy_{i}")
                    ] for i in range(n)
                ])
            }
        )(st.sidebar.number_input("Number of Bodies", value=2, step=1, min_value=2, max_value=5)),
        "plot": lambda result, st: plot_nbody(result, st)
    },
    "Rigidbody Collisions": {
        "module": "systems.rigidbody_collisions",
        "simulate": "simulate_rigidbody_collisions",
        "sidebar": lambda st: {
            "restitution": st.sidebar.number_input("Restitution Coefficient", value=1.0, step=0.0001, format="%.4f")
        },
        "plot": lambda result, st: st.info("Rigidbody collisions plot coming soon!")
    },
    "Rotational Dynamics": {
        "module": "systems.rotational_dynamics",
        "simulate": "simulate_rotational_dynamics",
        "sidebar": lambda st: {
            "I": st.sidebar.number_input("Moment of Inertia (kg·m²)", value=1.0, step=0.0001, format="%.4f"),
            "omega0": st.sidebar.number_input("Initial Angular Velocity (rad/s)", value=10.0, step=0.0001, format="%.4f"),
            "torque": st.sidebar.number_input("Applied Torque (N·m)", value=0.0, step=0.0001, format="%.4f")
        },
        "plot": lambda result, st: st.info("Rotational dynamics plot coming soon!")
    },
    "Orbital Mechanics": {
        "module": "systems.orbital_mechanics",
        "simulate": "simulate_orbital_mechanics",
        "sidebar": lambda st: {
            "m1": st.sidebar.number_input("Primary Mass (kg)", value=1.0, step=0.0001, format="%.4f"),
            "m2": st.sidebar.number_input("Satellite Mass (kg)", value=1.0, step=0.0001, format="%.4f"),
            "r1": st.sidebar.number_input("Initial Orbit Radius (m)", value=1.0, step=0.0001, format="%.4f"),
            "r2": st.sidebar.number_input("Final Orbit Radius (m)", value=2.0, step=0.0001, format="%.4f"),
            "v1": st.sidebar.number_input("Initial Velocity (m/s)", value=1.0, step=0.0001, format="%.4f"),
            "G": st.sidebar.number_input("Gravitational Constant", value=6.67430e-11, step=1e-14, format="%.14f")
        },
        "plot": lambda result, st: st.info("Orbital mechanics plot coming soon!")
    }
}

def plot_projectile_motion(result, st):
    st.subheader("Projectile Motion: Trajectory")
    fig = go.Figure()
    fig.add_trace(go.Scatter(x=result['x'], y=result['y'], mode='lines+markers', name='Trajectory'))
    fig.update_layout(xaxis_title="X (m)", yaxis_title="Y (m)", title="Projectile Trajectory", dragmode='pan')
    st.plotly_chart(fig, use_container_width=True)

    if st.checkbox("Show velocity vs time"):
        st.subheader("Velocity vs Time")
        fig2 = go.Figure()
        fig2.add_trace(go.Scatter(x=result['time'], y=np.sqrt(result['vx']**2 + result['vy']**2), mode='lines', name='Speed'))
        fig2.update_layout(xaxis_title="Time (s)", yaxis_title="Speed (m/s)", title="Speed vs Time", dragmode='pan')
        st.plotly_chart(fig2, use_container_width=True)

    st.subheader("Download Simulation Data")
    csv = result.to_csv(index=False).encode('utf-8')
    st.download_button(
        label="Download CSV",
        data=csv,
        file_name='projectile_motion_simulation.csv',
        mime='text/csv',
    )

def plot_spring_mass(result, st):
    st.subheader("Spring-Mass: Position vs Time")
    fig = go.Figure()
    fig.add_trace(go.Scatter(x=result['time'], y=result['position'], mode='lines', name='Position'))
    fig.update_layout(xaxis_title="Time (s)", yaxis_title="Position (m)", title="Position vs Time", dragmode='pan')
    st.plotly_chart(fig, use_container_width=True)

    if st.checkbox("Show velocity plot"):
        st.subheader("Velocity vs Time")
        fig2 = go.Figure()
        fig2.add_trace(go.Scatter(x=result['time'], y=result['velocity'], mode='lines', name='Velocity', line=dict(color='orange')))
        fig2.update_layout(xaxis_title="Time (s)", yaxis_title="Velocity (m/s)", title="Velocity vs Time", dragmode='pan')
        st.plotly_chart(fig2, use_container_width=True)

    if st.checkbox("Show energy plots"):
        st.subheader("Energy vs Time")
        fig3 = go.Figure()
        fig3.add_trace(go.Scatter(x=result['time'], y=result['KE'], mode='lines', name='Kinetic Energy'))
        fig3.add_trace(go.Scatter(x=result['time'], y=result['PE'], mode='lines', name='Potential Energy'))
        fig3.add_trace(go.Scatter(x=result['time'], y=result['TE'], mode='lines', name='Total Energy', line=dict(dash='dash', color='black')))
        fig3.update_layout(xaxis_title="Time (s)", yaxis_title="Energy (Joules)", title="Energy vs Time", dragmode='pan')
        st.plotly_chart(fig3, use_container_width=True)

    st.subheader("Download Simulation Data")
    csv = result.to_csv(index=False).encode('utf-8')
    st.download_button(
        label="Download CSV",
        data=csv,
        file_name='spring_mass_simulation.csv',
        mime='text/csv',
    )

def plot_double_pendulum(result, st):
    st.subheader("Double Pendulum: Trajectory")
    fig = go.Figure()
    fig.add_trace(go.Scatter(x=result['x1'], y=result['y1'], mode='lines', name='Mass 1'))
    fig.add_trace(go.Scatter(x=result['x2'], y=result['y2'], mode='lines', name='Mass 2'))
    fig.update_layout(xaxis_title="X (m)", yaxis_title="Y (m)", title="Pendulum Trajectories", dragmode='pan')
    st.plotly_chart(fig, use_container_width=True)

    if st.checkbox("Show angles vs time"):
        st.subheader("Angles vs Time")
        fig2 = go.Figure()
        fig2.add_trace(go.Scatter(x=result['time'], y=np.rad2deg(result['theta1']), mode='lines', name='Theta 1 (deg)'))
        fig2.add_trace(go.Scatter(x=result['time'], y=np.rad2deg(result['theta2']), mode='lines', name='Theta 2 (deg)'))
        fig2.update_layout(xaxis_title="Time (s)", yaxis_title="Angle (deg)", title="Angles vs Time", dragmode='pan')
        st.plotly_chart(fig2, use_container_width=True)

    if st.checkbox("Show velocity vs time"):
        st.subheader("Angular Velocity vs Time")
        fig3 = go.Figure()
        fig3.add_trace(go.Scatter(x=result['time'], y=result['z1'], mode='lines', name='Angular Velocity 1 (rad/s)'))
        fig3.add_trace(go.Scatter(x=result['time'], y=result['z2'], mode='lines', name='Angular Velocity 2 (rad/s)'))
        fig3.update_layout(xaxis_title="Time (s)", yaxis_title="Angular Velocity (rad/s)", title="Angular Velocity vs Time", dragmode='pan')
        st.plotly_chart(fig3, use_container_width=True)

    if st.checkbox("Show theta1 vs theta2"):
        st.subheader("Theta1 vs Theta2")
        fig4 = go.Figure()
        fig4.add_trace(go.Scatter(x=np.rad2deg(result['theta1']), y=np.rad2deg(result['theta2']), mode='lines', name='Theta1 vs Theta2'))
        fig4.update_layout(xaxis_title="Theta 1 (deg)", yaxis_title="Theta 2 (deg)", title="Phase Space: Theta1 vs Theta2", dragmode='pan')
        st.plotly_chart(fig4, use_container_width=True)

    st.subheader("Download Simulation Data")
    csv = result.to_csv(index=False).encode('utf-8')
    st.download_button(
        label="Download CSV",
        data=csv,
        file_name='double_pendulum_simulation.csv',
        mime='text/csv',
    )

def plot_nbody(result, st):
    st.subheader("N-body Simulation: Trajectories")
    fig = go.Figure()
    n = (len([col for col in result.columns if col.startswith('x')]))
    for i in range(n):
        fig.add_trace(go.Scatter(x=result[f'x{i+1}'], y=result[f'y{i+1}'], mode='lines', name=f'Body {i+1}'))
    fig.update_layout(xaxis_title="X (m)", yaxis_title="Y (m)", title="N-body Trajectories", dragmode='pan')
    st.plotly_chart(fig, use_container_width=True)

    if st.checkbox("Show velocity magnitude vs time for each body"):
        st.subheader("Velocity Magnitude vs Time")
        fig2 = go.Figure()
        for i in range(n):
                speed = np.sqrt(result[f'vx{i+1}']**2 + result[f'vy{i+1}']**2)
                fig2.add_trace(go.Scatter(x=result['time'], y=speed, mode='lines', name=f'Body {i+1} Speed'))
        fig2.update_layout(xaxis_title="Time (s)", yaxis_title="Speed (m/s)", title="Velocity Magnitude vs Time", dragmode='pan')
        st.plotly_chart(fig2, use_container_width=True)

    if st.checkbox("Show pairwise distance vs time"):
        st.subheader("Pairwise Distance vs Time")
        fig3 = go.Figure()
        for i in range(n):
            for j in range(i+1, n):
                dist = np.sqrt((result[f'x{i+1}'] - result[f'x{j+1}'])**2 + (result[f'y{i+1}'] - result[f'y{j+1}'])**2)
                fig3.add_trace(go.Scatter(x=result['time'], y=dist, mode='lines', name=f'Dist Body {i+1}-{j+1}'))
        fig3.update_layout(xaxis_title="Time (s)", yaxis_title="Distance (m)", title="Pairwise Distance vs Time", dragmode='pan')
        st.plotly_chart(fig3, use_container_width=True)

    st.subheader("Download Simulation Data")
    csv = result.to_csv(index=False).encode('utf-8')
    st.download_button(
        label="Download CSV",
        data=csv,
        file_name='nbody_simulation.csv',
        mime='text/csv',
    )

st.title("PhysiKit Simulations")
st.sidebar.markdown("## ⚡ Choose Physical System")
system_names = list(SYSTEMS.keys())
system = st.sidebar.selectbox("System", system_names)

if st.sidebar.button("Reset to Defaults"):
    reset_defaults(system)

st.sidebar.header("Simulation Parameters")
dt = 0.01
t_max = st.sidebar.number_input("Simulation Time (s)", value=10.0, step=0.1, format="%.2f")

params = SYSTEMS[system]["sidebar"](st)
params["t_max"] = t_max
params["dt"] = dt

mod = importlib.import_module(SYSTEMS[system]["module"])
simulate_func = getattr(mod, SYSTEMS[system]["simulate"])
result = simulate_func(**params)

SYSTEMS[system]["plot"](result, st)
