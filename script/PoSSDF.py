from sd_annular_sector import sd_annular_sector_fov, sd_annular_sector_fov_gradient
from sd_LOS import sd_LOS, sd_LOS_gradient
from sd_robot_obstacle import sd_robot_obstacle, sd_robot_obstacle_gradient
import numpy as np
from scipy.special import erf

def _as_obstacle_arrays(obs_origins, obs_radii):
    """Normalize obstacle inputs.
    obs_origins: (No,2) array-like
    obs_radii: float or (No,) array-like
    """
    C = np.asarray(obs_origins, dtype=float)
    if C.ndim != 2:
        C = C.reshape(-1, 2)
    if C.shape[1] != 2:
        raise ValueError("obs_origins must have shape (No,2).")
    No = C.shape[0]
    R = np.asarray(obs_radii, dtype=float)
    if R.ndim == 0:
        R = np.full((No,), float(R), dtype=float)
    elif R.shape != (No,):
        raise ValueError("obs_radii must be a scalar or have shape (No,).")
    return C, R


def _phi_from_sd_vec(sd_value, var_proj, sense="leq"):
    """
    Vectorized probability for linearized signed-distance constraint.

    Assume sd_lin ~ N(mean=sd_value, var=var_proj).

    sense="leq": Pr(sd_lin <= 0)  (used for 'inside/collision' events where sd<=0 defines event)
    sense="geq": Pr(sd_lin >= 0)  (used for 'clear LOS' events where sd>=0 defines event)
    """
    sd_value = np.asarray(sd_value, dtype=float)
    var_proj = np.asarray(var_proj, dtype=float)
    # handle near-deterministic
    det = var_proj <= 1e-12
    out = np.empty_like(sd_value, dtype=float)

    # if sense == "leq":
    #     out[det] = (sd_value[det] < 0.0).astype(float)
    #     out[det] = (np.abs(sd_value[det]) < 1e-12).astype(float) * 0.5
    # elif sense == "geq":
    #     out[det] = (sd_value[det] > 0.0).astype(float)
    #     out[det] = (np.abs(sd_value[det]) < 1e-12).astype(float) * 0.5
    # else:
    #     raise ValueError("sense must be 'leq' or 'geq'.")
    # # out[det] = 0.5

    # nd = ~det
    # if np.any(nd):
    #     z = sd_value[nd] / (np.sqrt(var_proj[nd]) * np.sqrt(2.0))
    #     p_leq = 0.5 * (1.0 - erf(z))  # Pr(sd <= 0)
    #     out[nd] = p_leq if sense == "leq" else (1.0 - p_leq)

    z = sd_value / (np.sqrt(var_proj) * np.sqrt(2.0) + 1e-12)
    p_leq = 0.5 * (1.0 - erf(z))  # Pr(sd <= 0)
    out = p_leq if sense == "leq" else (1.0 - p_leq)
    return out


# -----------------------------
# FOV probability (scalar)
# -----------------------------
def gamma_tf_FOV(mu_t, Sigma_t, mu_r, Sigma_r, r_min, r_max, fov_angle) -> float:
    """
    Probability that the target is inside the robot annular-sector FOV.
    Event: sd_FOV <= 0.
    """
    sd_value = sd_annular_sector_fov(
        mu_t,
        sector_origin=mu_r[0:2],
        sector_dir=mu_r[2],
        r_min=r_min,
        r_max=r_max,
        fov_angle=fov_angle,
    )

    dsd_dx_t, dsd_dy_t, dsd_dx_r, dsd_dy_r, dsd_dheading = sd_annular_sector_fov_gradient(
        mu_t, mu_r[0:2], mu_r[2], fov_angle, r_min=r_min, r_max=r_max
    )

    Sigma_t_xy = Sigma_t[:2, :2]

    # Full projection variance for robot (x,y,theta,v) with gradient [gx,gy,gtheta,0]
    g_r = np.array([dsd_dx_r, dsd_dy_r, dsd_dheading, 0.0], dtype=float)
    var_r = float(g_r @ (Sigma_r @ g_r))

    g_t_xy = np.array([dsd_dx_t, dsd_dy_t], dtype=float)
    var_t = float(g_t_xy @ (Sigma_t_xy @ g_t_xy))

    # inside event: sd <= 0
    return float(_phi_from_sd_vec(np.array([sd_value]), np.array([var_r + var_t]), sense="leq")[0])


# -----------------------------
# Vectorized LOS probability for many obstacles
# -----------------------------
def gamma_lo_LOS(mu_t, Sigma_t, mu_r, Sigma_r, obs_origins, obs_radii, eps=1e-12):
    """
    Vectorized gamma_lo for multiple circular obstacles.

    LOS SDF: sd_LOS = dist(segment(robot->target), c_i) - r_i
    Event for clear LOS: sd_LOS >= 0  (no intersection).

    Returns:
        gamma_lo: (No,) array
        sd_values: (No,) array (optional use/debug)
        var_proj: (No,) array (optional use/debug)
    """
    C, R = _as_obstacle_arrays(obs_origins, obs_radii)
    No = C.shape[0]

    a = np.asarray(mu_r[0:2], dtype=float)  # (2,)
    b = np.asarray(mu_t[:2], dtype=float)   # (2,)

    v = b - a
    vv = float(v @ v)

    Sigma_r_xy = np.asarray(Sigma_r[:2, :2], dtype=float)
    Sigma_t_xy = np.asarray(Sigma_t[:2, :2], dtype=float)

    # Degenerate segment (robot==target)
    if vv <= eps:
        d = C - a[None, :]
        dist = np.linalg.norm(d, axis=1) + eps
        sd = dist - R
        u = d / dist[:, None]  # direction from a to obstacle center

        # q=a always -> grad wrt a is -u, wrt b is 0
        grad_a = -u
        grad_b = np.zeros_like(grad_a)

        var_r = np.einsum("ni,ij,nj->n", grad_a, Sigma_r_xy, grad_a)
        var_t = np.einsum("ni,ij,nj->n", grad_b, Sigma_t_xy, grad_b)
        var = var_r + var_t

        gamma = _phi_from_sd_vec(sd, var, sense="geq")
        return gamma, sd, var

    # t_raw for each obstacle center
    w = C - a[None, :]  # (No,2)
    t_raw = (w @ v) / vv  # (No,)
    t = np.clip(t_raw, 0.0, 1.0)
    q = a[None, :] + t[:, None] * v[None, :]  # (No,2)

    d = C - q
    dist = np.linalg.norm(d, axis=1) + eps
    sd = dist - R
    u = d / dist[:, None]  # (No,2)

    # gradients wrt a(=robot pos) and b(=target pos)
    grad_a = np.zeros((No, 2), dtype=float)
    grad_b = np.zeros((No, 2), dtype=float)

    mask0 = t_raw <= 0.0
    mask1 = t_raw >= 1.0
    maskI = ~(mask0 | mask1)

    # projection before a: q=a
    grad_a[mask0] = -u[mask0]
    # projection after b: q=b
    grad_b[mask1] = -u[mask1]
    # interior projection
    if np.any(maskI):
        ti = t[maskI]
        ui = u[maskI]
        grad_a[maskI] = -(1.0 - ti)[:, None] * ui
        grad_b[maskI] = -ti[:, None] * ui

    var_r = np.einsum("ni,ij,nj->n", grad_a, Sigma_r_xy, grad_a)
    var_t = np.einsum("ni,ij,nj->n", grad_b, Sigma_t_xy, grad_b)
    var = var_r + var_t

    # clear LOS: sd >= 0
    gamma = _phi_from_sd_vec(sd, var, sense="geq")
    return gamma, sd, var


# -----------------------------
# Vectorized collision probability for many obstacles
# -----------------------------
def gamma_ro_collision(mu_r, Sigma_r, obs_origins, obs_radii, eps=1e-12):
    """
    Vectorized gamma_ro for multiple circular obstacles.
    Collision SDF: sd = ||p_r - c_i|| - r_i
    Event for collision: sd <= 0.
    """
    C, R = _as_obstacle_arrays(obs_origins, obs_radii)
    p = np.asarray(mu_r[0:2], dtype=float)

    d = p[None, :] - C
    dist = np.linalg.norm(d, axis=1) + eps
    sd = dist - R

    g_xy = d / dist[:, None]  # (No,2)  gradient wrt robot position
    Sigma_r_xy = np.asarray(Sigma_r[:2, :2], dtype=float)
    var = np.einsum("ni,ij,nj->n", g_xy, Sigma_r_xy, g_xy)

    # collision: sd <= 0
    gamma = _phi_from_sd_vec(sd, var, sense="leq")
    return gamma, sd, var


# -----------------------------
# Convenience: BPOD gamma_k from factors
# -----------------------------
def gamma_k_bpod(mu_t, Sigma_t, mu_r, Sigma_r, r_min, r_max, fov_angle, obs_origins, obs_radii):
    """
    Returns:
        gamma_k: scalar BPOD = gamma_tf * prod_i gamma_lo_i
        gamma_tf: scalar
        gamma_lo: (No,) vector
    """
    gamma_tf = gamma_tf_FOV(mu_t, Sigma_t, mu_r, Sigma_r, r_min, r_max, fov_angle)
    gamma_lo, _, _ = gamma_lo_LOS(mu_t, Sigma_t, mu_r, Sigma_r, obs_origins, obs_radii)

    # product can underflow if many obstacles; guard with log if needed
    # Here keep simple product (usually No is small).
    gamma_k = float(gamma_tf * np.prod(gamma_lo))
    return gamma_k, gamma_tf, gamma_lo


if __name__ == "__main__":
    # Example usage
    mu_t_ = np.array([4.0, 0.0])
    Sigma_t = np.array([[1.0, 0.0], 
                        [0.0, 1.0]])
    mu_r = np.array([0.0, 0.0, np.deg2rad(30), 0.0])  # x, y, theta, v
    Sigma_r = np.array([[0.5, 0.0, 0.0, 0.0],
                        [0.0, 0.5, 0.0, 0.0],
                        [0.0, 0.0, 0.001, 0.0],
                        [0.0, 0.0, 0.0, 0.001]])
    heading_r = np.pi / 4  # 45 degrees
    r_min = 1.0
    r_max = 10.0
    fov_angle = np.pi / 3  # 60 degrees

    gamma_tf = gamma_tf_FOV(mu_t_, Sigma_t, mu_r, Sigma_r, r_min, r_max, fov_angle)
    print(f"Probability that target is within FOV: {gamma_tf}")


    import matplotlib.pyplot as plt

    xx, yy = np.meshgrid(np.linspace(-5, 10, 151), np.linspace(-5, 10, 151))
    zz = np.stack([xx, yy], axis=-1)

    prob_map = np.zeros(xx.shape)
    sd_value_map = np.zeros(xx.shape)
    for i in range(xx.shape[0]):
        for j in range(xx.shape[1]):
            mu_t = np.array([xx[i, j], yy[i, j]])
            prob_map[i, j] = gamma_tf_FOV(
                mu_t,
                Sigma_t,
                mu_r,
                Sigma_r,
                r_min,
                r_max,
                fov_angle,
            )
            sd_value_map[i, j] = sd_annular_sector_fov(
                mu_t,
                sector_origin=mu_r[0:2],
                sector_dir=mu_r[2],
                r_min=r_min,
                r_max=r_max,
                fov_angle=fov_angle,
            )
    plt.figure()

    # plt.contourf(xx, yy, sd_value_map, levels=200, cmap="RdBu_r")
    # plt.colorbar(label="Signed Distance")
    plt.contour(xx, yy, sd_value_map, levels=[0.0], colors="k", linewidths=2)
    plt.contour(xx, yy, prob_map, levels=[0.5], colors="g", linewidths=2)
    plt.contour(xx, yy, prob_map, levels=[0.9], colors="r", linewidths=2)
    plt.contourf(xx, yy, prob_map, levels=50, cmap='viridis')
    plt.plot(mu_r[0], mu_r[1], "ro", label="Robot", markersize=8)
    plt.plot(mu_t_[0], mu_t_[1], "bx", label="Target Mean", markersize=8)
    plt.colorbar(label='Probability of being in FOV')
    plt.xlabel('X position')
    plt.ylabel('Y position')
    plt.title('Probability Map of Target in Robot FOV')
    plt.show()

    # #################################################################################

    xx, yy = np.meshgrid(np.linspace(-5, 10, 151), np.linspace(-5, 10, 151))
    zz = np.stack([xx, yy], axis=-1)
    # prob_map = np.zeros(xx.shape)
    sd_value_map = np.zeros(xx.shape)
    mu_t = np.array([5.0, 0.0])

    for i in range(xx.shape[0]):
        for j in range(xx.shape[1]):
            obs_origin = np.array([xx[i, j], yy[i, j]])
            # prob_map[i, j] = gamma_lo_LOS(
            #     mu_t,
            #     Sigma_t,
            #     mu_r,
            #     Sigma_r,
            #     obs_origin=obs_origin,
            #     obs_radius=1.0,
            # )
            sd_value_map[i, j] = sd_LOS(
                obs_origin=obs_origin,
                obs_radius=1.0,
                los_point1=mu_r[0:2],
                los_point2=mu_t,
            )

    prob_map = gamma_lo_LOS(
        mu_t,
        Sigma_t,
        mu_r,
        Sigma_r,
        obs_origins=zz.reshape(-1,2),
        obs_radii=1.0,
    )[0].reshape(xx.shape)
    # sd_value_map = sd_LOS(
    #     obs_origin=zz.reshape(-1,2),
    #     obs_radius=1.0,
    #     los_point1=mu_r[0:2],
    #     los_point2=mu_t,
    # ).reshape(xx.shape)
    

    plt.figure()
    # plt.contourf(xx, yy, sd_value_map, levels=200, cmap="RdBu_r")
    # plt.colorbar(label="Signed Distance")
    plt.contour(xx, yy, sd_value_map, levels=[0.0], colors="k", linewidths=2)
    plt.contourf(xx, yy, prob_map, levels=50, cmap='viridis')
    plt.colorbar(label='Probability of being in LOS')
    plt.xlabel('X position')
    plt.ylabel('Y position')
    plt.title('Probability Map of Target in Robot LOS')
    plt.show()

    # #################################################################################

    xx, yy = np.meshgrid(np.linspace(-5, 10, 200), np.linspace(-5, 10, 200))
    zz = np.stack([xx, yy], axis=-1)
    prob_map = np.zeros(xx.shape)
    sd_value_map = np.zeros(xx.shape)
    # for i in range(xx.shape[0]):
    #     for j in range(xx.shape[1]):
    #         obs_origin = np.array([xx[i, j], yy[i, j]])
    #         prob_map[i, j] = gamma_ro_collision(
    #             mu_r,
    #             Sigma_r,
    #             obs_origin=obs_origin,
    #             obs_radius=1.0,
    #         )
    #         sd_value_map[i, j] = sd_robot_obstacle(
    #             robot_point=mu_r[0:2],
    #             obs_origin=obs_origin,
    #             obs_radius=1.0,
    #         )
    sd_value_map = sd_robot_obstacle(
        robot_point=mu_r[0:2],
        obs_origin=zz.reshape(-1,2),
        obs_radius=1.0,
    ).reshape(xx.shape)
    prob_map = gamma_ro_collision(
        mu_r,
        Sigma_r,
        obs_origins=zz.reshape(-1,2),
        obs_radii=1.0,
    )[0].reshape(xx.shape)

    plt.figure()
    # plt.contourf(xx, yy, sd_value_map, levels=200, cmap="RdBu_r")
    # plt.colorbar(label="Signed Distance")
    plt.contour(xx, yy, sd_value_map, levels=[0.0], colors="k", linewidths=2)
    plt.contourf(xx, yy, prob_map, levels=50, cmap='viridis')
    plt.colorbar(label='Probability of Robot-Obstacle Collision')
    plt.xlabel('X position')
    plt.ylabel('Y position')
    plt.title('Probability Map of Robot-Obstacle Collision')
    plt.show()