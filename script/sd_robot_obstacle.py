import numpy as np

def sd_robot_obstacle(robot_point, obs_origin, obs_radius):
    obs_origin = np.asarray(obs_origin, dtype=float)
    p = np.asarray(robot_point, dtype=float)
    d = p - obs_origin
    return np.linalg.norm(d, axis=-1) - obs_radius

def sd_robot_obstacle_gradient(robot_point, obs_origin, obs_radius, eps=1e-12):
    p = np.asarray(robot_point, dtype=float).reshape(2,)
    c = np.asarray(obs_origin, dtype=float).reshape(2,)
    d = p - c
    n = float(np.linalg.norm(d)) + eps
    return d[0] / n, d[1] / n
