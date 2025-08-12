# This is the main simulation page for PhysiKit
# It is identical to your previous app.py

import streamlit as st
import matplotlib.pyplot as plt
import plotly.graph_objs as go
import importlib
import numpy as np

SYSTEMS = {
    "Spring-Mass": {
        "module": "systems.spring_mass",
        "simulate": "simulate_spring_mass",
        "sidebar": lambda st: {
            "m": st.sidebar.number_input("Mass (kg)", value=1.0, step=0.1, format="%.2f"),
            "k": st.sidebar.number_input("Spring Constant k (N/m)", value=1.0, step=0.1, format="%.2f"),
            "x0": st.sidebar.number_input("Initial Position (m)", value=1.0, step=0.1, format="%.2f"),
            "v0": st.sidebar.number_input("Initial Velocity (m/s)", value=0.0, step=0.1, format="%.2f"),
            "c": st.sidebar.number_input("Damping Coefficient c (Ns/m)", value=0.0, step=0.1, format="%.2f")
        },
        "plot": lambda result, st: plot_spring_mass(result, st)
    },
    "Double Pendulum": {
        "module": "systems.double_pendulum",
        "simulate": "simulate_double_pendulum",
        "sidebar": lambda st: {
            "m1": st.sidebar.number_input("Mass 1 (kg)", value=1.0, step=0.1, format="%.2f"),
            "m2": st.sidebar.number_input("Mass 2 (kg)", value=1.0, step=0.1, format="%.2f"),
            "l1": st.sidebar.number_input("Rod Length 1 (m)", value=1.0, step=0.1, format="%.2f"),
            "l2": st.sidebar.number_input("Rod Length 2 (m)", value=1.0, step=0.1, format="%.2f"),
            "theta1": np.deg2rad(st.sidebar.number_input("Initial Angle 1 (deg)", value=90.0, step=1.0, format="%.1f")),
            "theta2": np.deg2rad(st.sidebar.number_input("Initial Angle 2 (deg)", value=90.0, step=1.0, format="%.1f")),
            "z1": st.sidebar.number_input("Initial Angular Velocity 1 (rad/s)", value=0.0, step=0.1, format="%.2f"),
            "z2": st.sidebar.number_input("Initial Angular Velocity 2 (rad/s)", value=0.0, step=0.1, format="%.2f"),
            "g": st.sidebar.number_input("Gravity (m/s²)", value=9.81, step=0.01, format="%.2f")
        },
        "plot": lambda result, st: plot_double_pendulum(result, st)
    }
}

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

st.title("PhysiKit Simulations")
st.sidebar.markdown("## ⚡ Choose Physical System")
system_names = list(SYSTEMS.keys())
system = st.sidebar.selectbox("System", system_names)

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
