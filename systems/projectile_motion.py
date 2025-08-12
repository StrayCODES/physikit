"""
Projectile Motion with Air Resistance: Realistic trajectory simulation.
"""
import numpy as np
import pandas as pd


def simulate_projectile_motion(v0=10.0, angle=45.0, mass=1.0, drag=0.1, g=9.81, t_max=10.0, dt=0.01):
    """
    Simulate projectile motion with air resistance (linear drag).
    Parameters: initial velocity, launch angle (deg), mass, drag coefficient, gravity, simulation time, time step.
    Returns: DataFrame with time, x, y, vx, vy
    """
    angle_rad = np.deg2rad(angle)
    vx0 = v0 * np.cos(angle_rad)
    vy0 = v0 * np.sin(angle_rad)

    def derivatives(t, y):
        x, y_pos, vx, vy = y
        v = np.sqrt(vx**2 + vy**2)
        dxdt = vx
        dydt = vy
        dvxdt = -drag * vx / mass
        dvydt = -g - drag * vy / mass
        return [dxdt, dydt, dvxdt, dvydt]

    from scipy.integrate import solve_ivp
    t_span = (0, t_max)
    # Ensure t_eval is strictly within t_span
    t_eval = np.arange(0, t_max, dt)
    y0 = [0, 0, vx0, vy0]
    sol = solve_ivp(derivatives, t_span, y0, t_eval=t_eval, method="RK45")

    # Input validation
    if t_max <= 0 or dt <= 0:
        return pd.DataFrame(columns=['time', 'x', 'y', 'vx', 'vy'])

    # Stop at ground (y >= 0)
    x = sol.y[0]
    y_pos = sol.y[1]
    vx = sol.y[2]
    vy = sol.y[3]
    mask = y_pos >= 0
    result = pd.DataFrame({
        'time': sol.t[mask],
        'x': x[mask],
        'y': y_pos[mask],
        'vx': vx[mask],
        'vy': vy[mask]
    })
    return result
