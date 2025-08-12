"""
Rotational Dynamics: Gyroscope and top motion simulations.
"""
import numpy as np
import pandas as pd

def simulate_rotational_dynamics(I=1.0, omega0=10.0, torque=0.0, t_max=10.0, dt=0.01):
    """
    Simulate rotational dynamics (gyroscope/top).
    Parameters: moment of inertia, initial angular velocity, applied torque, simulation time, time step.
    Returns: DataFrame with time, angle, omega
    """
    # Input validation
    if t_max <= 0 or dt <= 0:
        return pd.DataFrame({'time': [], 'angle': [], 'omega': []})
    steps = int(t_max / dt)
    times = np.linspace(0, t_max, steps)
    omega = np.zeros(steps)
    angle = np.zeros(steps)
    omega[0] = omega0
    angle[0] = 0.0
    for k in range(1, steps):
        # omega(t+dt) = omega(t) + (torque/I)*dt
        omega[k] = omega[k-1] + (torque / I) * dt
        # angle(t+dt) = angle(t) + omega(t)*dt
        angle[k] = angle[k-1] + omega[k-1] * dt
    return pd.DataFrame({'time': times, 'angle': angle, 'omega': omega})
