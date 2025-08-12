"""
Orbital Mechanics: Satellite trajectories, Hohmann transfers.
"""
import numpy as np
import pandas as pd

def simulate_orbital_mechanics(m1=1.0, m2=1.0, r1=1.0, r2=2.0, v1=1.0, G=6.67430e-11, t_max=10.0, dt=0.01):
    """
    Simulate advanced orbital mechanics: elliptical orbits, Hohmann transfer, and output positions and velocities over time.
    Parameters: masses, radii, velocities, gravitational constant, simulation time, time step.
    Returns: DataFrame with time, x, y, vx, vy, r, theta
    """
    if t_max <= 0 or dt <= 0:
        return pd.DataFrame({
            'time': [], 'x': [], 'y': [], 'vx': [], 'vy': [], 'r': [], 'theta': []
        })
    steps = int(t_max / dt)
    if steps < 1:
        return pd.DataFrame({
            'time': [], 'x': [], 'y': [], 'vx': [], 'vy': [], 'r': [], 'theta': []
        })
    times = np.linspace(0, t_max, steps)

    # Initial conditions
    x = np.zeros(steps)
    y = np.zeros(steps)
    vx = np.zeros(steps)
    vy = np.zeros(steps)
    r = np.zeros(steps)
    theta = np.zeros(steps)

    # Place planet at origin, satellite at r1
    x[0] = r1
    y[0] = 0.0
    r[0] = r1
    theta[0] = 0.0
    # Initial velocity perpendicular to radius
    vx[0] = 0.0
    vy[0] = v1

    # Use velocity Verlet for integration
    for k in range(1, steps):
        r_k = np.sqrt(x[k-1]**2 + y[k-1]**2)
        # Gravitational acceleration
        ax = -G * m1 * x[k-1] / r_k**3
        ay = -G * m1 * y[k-1] / r_k**3
        # Update positions
        x[k] = x[k-1] + vx[k-1]*dt + 0.5*ax*dt**2
        y[k] = y[k-1] + vy[k-1]*dt + 0.5*ay*dt**2
        r[k] = np.sqrt(x[k]**2 + y[k]**2)
        theta[k] = np.arctan2(y[k], x[k])
        # Update accelerations at new position
        ax_new = -G * m1 * x[k] / r[k]**3
        ay_new = -G * m1 * y[k] / r[k]**3
        # Update velocities
        vx[k] = vx[k-1] + 0.5*(ax + ax_new)*dt
        vy[k] = vy[k-1] + 0.5*(ay + ay_new)*dt

    # Optionally, add Hohmann transfer logic (instantaneous velocity change at t = t_max/2)
    # For demo, apply delta-v at halfway point to transfer from r1 to r2
    transfer_idx = steps // 2
    if r1 != r2:
        # Calculate required delta-v for Hohmann transfer
        v_circ1 = np.sqrt(G * m1 / r1)
        v_trans1 = np.sqrt(G * m1 * (2/r1 - 1/(r1 + r2)))
        dv = v_trans1 - v_circ1
        vx[transfer_idx] += -dv * np.sin(theta[transfer_idx])
        vy[transfer_idx] += dv * np.cos(theta[transfer_idx])

    df = pd.DataFrame({
        'time': times,
        'x': x,
        'y': y,
        'vx': vx,
        'vy': vy,
        'r': r,
        'theta': theta
    })
    return df
