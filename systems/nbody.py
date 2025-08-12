"""
N-body Gravitational Simulation: Simulate planetary systems.
"""
import numpy as np
import pandas as pd

def simulate_nbody(n=2, masses=None, positions=None, velocities=None, G=6.67430e-11, t_max=10.0, dt=0.01):
    """
    Simulate N-body gravitational system.
    Parameters: number of bodies, masses, initial positions, velocities, gravitational constant, simulation time, time step.
    Returns: DataFrame with time, positions, velocities
    """
    # Input validation
    if t_max <= 0 or dt <= 0 or n < 2:
        return pd.DataFrame({'time': [], 'positions': [], 'velocities': []})

    # Default masses, positions, velocities
    if masses is None:
        masses = np.ones(n)
    if positions is None:
        positions = np.random.rand(n, 2) * 10  # 2D positions
    if velocities is None:
        velocities = np.zeros((n, 2))

    steps = int(t_max / dt)
    times = np.linspace(0, t_max, steps)
    pos_hist = np.zeros((steps, n, 2))
    vel_hist = np.zeros((steps, n, 2))

    pos = positions.copy()
    vel = velocities.copy()

    def compute_acc(pos):
        acc = np.zeros_like(pos)
        for i in range(n):
            for j in range(n):
                if i != j:
                    r = pos[j] - pos[i]
                    dist = np.linalg.norm(r) + 1e-9  # avoid div by zero
                    acc[i] += G * masses[j] * r / dist**3
        return acc

    for k, t in enumerate(times):
        pos_hist[k] = pos
        vel_hist[k] = vel
        acc = compute_acc(pos)
        # Velocity Verlet
        pos = pos + vel * dt + 0.5 * acc * dt**2
        acc_new = compute_acc(pos)
        vel = vel + 0.5 * (acc + acc_new) * dt

    # Flatten positions and velocities for DataFrame
    data = {
        'time': times
    }
    for i in range(n):
        data[f'x{i+1}'] = pos_hist[:, i, 0]
        data[f'y{i+1}'] = pos_hist[:, i, 1]
        data[f'vx{i+1}'] = vel_hist[:, i, 0]
        data[f'vy{i+1}'] = vel_hist[:, i, 1]
    return pd.DataFrame(data)
