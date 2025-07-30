import numpy as np
from scipy.integrate import solve_ivp
import pandas as pd

def spring_mass_ode(t, y, k, m, c):
    x, v = y
    dxdt = v
    dvdt = - (k / m) * x - (c / m) * v  # Adding damping term
    return [dxdt, dvdt]

def simulate_spring_mass(m=1.0, k=1.0,c=0.0, x0=1.0, v0=0.0, t_max=10, dt=0.01):
    t_span = (0, t_max)
    t_eval = np.arange(0, t_max, dt)
    y0 = [x0, v0]
    
    sol = solve_ivp(
        spring_mass_ode,
        t_span,
        y0,
        args=(k, m, c),
        t_eval=t_eval,
        method="RK45"
    )
    
    x = sol.y[0]
    v = sol.y[1]
    
    # Energy calculations
    KE = 0.5 * m * v**2
    PE = 0.5 * k * x**2
    TE = KE + PE
    
    result = pd.DataFrame({
        'time': sol.t,
        'position': x,
        'velocity': v,
        'KE': KE,
        'PE': PE,
        'TE': TE
    })
    return result

