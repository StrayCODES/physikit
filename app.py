import streamlit as st
import matplotlib.pyplot as plt
from systems.spring_mass import simulate_spring_mass

st.title("PhysiKit — Spring-Mass System")

# Sidebar for inputs
st.sidebar.header("Simulation Parameters")
mass = st.sidebar.slider("Mass (kg)", 0.1, 10.0, 1.0)
spring_const = st.sidebar.slider("Spring Constant k (N/m)", 0.1, 10.0, 1.0)
x0 = st.sidebar.slider("Initial Position (m)", -5.0, 5.0, 1.0)
v0 = st.sidebar.slider("Initial Velocity (m/s)", -10.0, 10.0, 0.0)
c = st.sidebar.slider("Damping Coefficient c (Ns/m)", 0.0, 5.0, 0.0, step=0.1)
t_max = st.sidebar.slider("Simulation Time (s)", 1, 20, 10)
dt = 0.01

# Run simulation
result = simulate_spring_mass(mass, spring_const, c, x0, v0, t_max, dt)

# Plotting
st.subheader("Position vs Time")
fig, ax = plt.subplots()
ax.plot(result['time'], result['position'])
ax.set_xlabel("Time (s)")
ax.set_ylabel("Position (m)")
ax.grid()
st.pyplot(fig)

# Optional: Show velocity plot
if st.checkbox("Show velocity plot"):
    st.subheader("Velocity vs Time")
    fig2, ax2 = plt.subplots()
    ax2.plot(result['time'], result['velocity'], color='orange')
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Velocity (m/s)")
    ax2.grid()
    st.pyplot(fig2)

# Energy plot
if st.checkbox("Show energy plots"):
    st.subheader("Energy vs Time")
    fig3, ax3 = plt.subplots()
    ax3.plot(result['time'], result['KE'], label='Kinetic Energy')
    ax3.plot(result['time'], result['PE'], label='Potential Energy')
    ax3.plot(result['time'], result['TE'], label='Total Energy', linestyle='--', color='black')
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Energy (Joules)")
    ax3.grid()
    ax3.legend()
    st.pyplot(fig3)

# CSV download
st.subheader("Download Simulation Data")
csv = result.to_csv(index=False).encode('utf-8')
st.download_button(
    label="Download CSV",
    data=csv,
    file_name='spring_mass_simulation.csv',
    mime='text/csv',
)