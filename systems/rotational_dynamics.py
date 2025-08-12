"""
Rotational Dynamics: Gyroscope and top motion simulations.
"""
import numpy as np
import pandas as pd

def simulate_rotational_dynamics(I=1.0, omega0=10.0, torque=0.0, t_max=10.0, dt=0.01, damping=0.0, precess=False, torque_input=None):
    """
    Simulate rotational dynamics with damping, time-varying torque, and gyroscope precession.
    Parameters:
        I: moment of inertia
        omega0: initial angular velocity
        torque: applied torque (can be float or array)
        t_max: simulation time
        dt: time step
        damping: friction coefficient (optional)
        precess: if True, simulate gyroscope precession
    Returns: DataFrame with time, angle, omega, (precession, nutation if enabled)
    """
    # Input validation
    if t_max <= 0 or dt <= 0:
        return pd.DataFrame({'time': [], 'angle': [], 'omega': []})

    # Parameters are now passed directly

    steps = int(t_max / dt)
    times = np.linspace(0, t_max, steps)
    omega = np.zeros(steps)
    angle = np.zeros(steps)
    omega[0] = omega0
    angle[0] = 0.0

    # For gyroscope precession
    if precess:
        phi = np.zeros(steps)  # precession angle
        nutation = np.zeros(steps)  # nutation angle
        phi[0] = 0.0
        nutation[0] = 0.0

    for k in range(1, steps):
        # Time-varying torque
        tau = torque
        if torque_input is not None:
            tau = torque_input[k] if k < len(torque_input) else torque
        # Damping/friction
        tau_net = tau - damping * omega[k-1]
        omega[k] = omega[k-1] + (tau_net / I) * dt
        angle[k] = angle[k-1] + omega[k-1] * dt
        if precess:
            # Simple gyroscope precession: phi_dot = tau / (I * omega)
            phi[k] = phi[k-1] + (tau / (I * omega[k-1] if omega[k-1] != 0 else 1e-8)) * dt
            # Nutation: small oscillation, for demo
            nutation[k] = 0.1 * np.sin(2 * np.pi * times[k] / t_max)

    data = {'time': times, 'angle': angle, 'omega': omega}
    if precess:
        data['precession'] = phi
        data['nutation'] = nutation
    return pd.DataFrame(data)
