import numpy as np
from scipy.integrate import solve_ivp
import pandas as pd

def double_pendulum_ode(t, y, m1, m2, l1, l2, g):
    theta1, z1, theta2, z2 = y
    delta = theta2 - theta1
    denom1 = (m1 + m2) * l1 - m2 * l1 * np.cos(delta) ** 2
    denom2 = (l2 / l1) * denom1

    dtheta1dt = z1
    dtheta2dt = z2

    dz1dt = (
        m2 * l1 * z1 ** 2 * np.sin(delta) * np.cos(delta)
        + m2 * g * np.sin(theta2) * np.cos(delta)
        + m2 * l2 * z2 ** 2 * np.sin(delta)
        - (m1 + m2) * g * np.sin(theta1)
    ) / denom1

    dz2dt = (
        -m2 * l2 * z2 ** 2 * np.sin(delta) * np.cos(delta)
        + (m1 + m2) * g * np.sin(theta1) * np.cos(delta)
        - (m1 + m2) * l1 * z1 ** 2 * np.sin(delta)
        - (m1 + m2) * g * np.sin(theta2)
    ) / denom2

    return [dtheta1dt, dz1dt, dtheta2dt, dz2dt]

def simulate_double_pendulum(
    m1=1.0, m2=1.0, l1=1.0, l2=1.0, theta1=np.pi/2, theta2=np.pi/2, z1=0.0, z2=0.0, g=9.81, t_max=10, dt=0.01
):
    t_span = (0, t_max)
    t_eval = np.arange(0, t_max, dt)
    y0 = [theta1, z1, theta2, z2]

    # Input validation
    if t_max <= 0 or dt <= 0:
        return pd.DataFrame(columns=['time', 'theta1', 'theta2', 'z1', 'z2', 'x1', 'y1', 'x2', 'y2'])

    sol = solve_ivp(
        double_pendulum_ode,
        t_span,
        y0,
        args=(m1, m2, l1, l2, g),
        t_eval=t_eval,
        method="RK45"
    )


    theta1 = sol.y[0]
    z1 = sol.y[1]
    theta2 = sol.y[2]
    z2 = sol.y[3]

    # Cartesian coordinates for visualization
    x1 = l1 * np.sin(theta1)
    y1 = -l1 * np.cos(theta1)
    x2 = x1 + l2 * np.sin(theta2)
    y2 = y1 - l2 * np.cos(theta2)

    result = pd.DataFrame({
        'time': sol.t,
        'theta1': theta1,
        'z1': z1,
        'theta2': theta2,
        'z2': z2,
        'x1': x1,
        'y1': y1,
        'x2': x2,
        'y2': y2
    })
    return result
