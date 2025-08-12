"""
Rigidbody Collisions: Simulate 2D/3D elastic/inelastic collisions.
"""
import numpy as np
import pandas as pd

def simulate_rigidbody_collisions(bodies=None, restitution=1.0, t_max=10.0, dt=0.01):
    """
    Simulate rigidbody collisions in 3D.
    Parameters: bodies (list of dicts: {pos, vel, mass, radius}), restitution coefficient, simulation time, time step.
    Returns: DataFrame with time, positions, velocities
    """
    # Input validation
    if t_max <= 0 or dt <= 0:
        return pd.DataFrame({'time': [], 'states': []})
    if bodies is None or len(bodies) < 2:
        # Default: two bodies in 3D
        bodies = [
            {'pos': np.array([0.0, 0.0, 0.0]), 'vel': np.array([1.0, 0.0, 0.0]), 'mass': 1.0, 'radius': 0.5},
            {'pos': np.array([2.0, 0.0, 0.0]), 'vel': np.array([-1.0, 0.0, 0.0]), 'mass': 1.0, 'radius': 0.5}
        ]
    n = len(bodies)
    dim = len(bodies[0]['pos'])
    steps = int(t_max / dt)
    times = np.linspace(0, t_max, steps)
    pos_hist = np.zeros((steps, n, dim))
    vel_hist = np.zeros((steps, n, dim))
    pos = np.array([b['pos'] for b in bodies])
    vel = np.array([b['vel'] for b in bodies])
    mass = np.array([b['mass'] for b in bodies])
    radius = np.array([b['radius'] for b in bodies])

    def resolve_collision(i, j):
        r_rel = pos[j] - pos[i]
        v_rel = vel[j] - vel[i]
        dist = np.linalg.norm(r_rel)
        if dist == 0:
            return
        n_vec = r_rel / dist
        v_rel_n = np.dot(v_rel, n_vec)
        if v_rel_n > 0:
            return  # moving apart
        impulse = -(1 + restitution) * v_rel_n / (1/mass[i] + 1/mass[j])
        vel[i] -= impulse * n_vec / mass[i]
        vel[j] += impulse * n_vec / mass[j]

    for k, t in enumerate(times):
        pos_hist[k] = pos
        vel_hist[k] = vel
        pos = pos + vel * dt
        for i in range(n):
            for j in range(i+1, n):
                if np.linalg.norm(pos[i] - pos[j]) < (radius[i] + radius[j]):
                    resolve_collision(i, j)

    # Flatten positions and velocities for DataFrame
    data = {'time': times}
    for i in range(n):
        for d, label in enumerate(['x', 'y', 'z'][:dim]):
            data[f'{label}{i+1}'] = pos_hist[:, i, d]
            data[f'v{label}{i+1}'] = vel_hist[:, i, d]
    return pd.DataFrame(data)
