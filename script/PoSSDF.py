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

    z = sd_value / (np.sqrt(var_proj) * np.sqrt(2.0) + 1e-12)
    p_leq = 0.5 * (1.0 - erf(z))  # Pr(sd <= 0)
    out = p_leq if sense == "leq" else (1.0 - p_leq)
    return out

def sd_annular_sector(
    z,
    sector_origin,
    sector_dir,   # theta [rad]
    r_min,
    r_max,
    half_angle,   # alpha [rad]
):
    """
    Signed distance to an annular sector.
    Negative inside the sector, positive outside.

    z can be shape (2,) or (...,2). Returns scalar or array (...,).
    """
    z = np.asarray(z, dtype=float)
    o = np.asarray(sector_origin, dtype=float)

    if r_min < 0 or r_max <= 0 or r_min > r_max:
        raise ValueError("Require 0 <= r_min <= r_max and r_max > 0.")
    if not (0.0 < half_angle <= np.pi):
        raise ValueError("Require 0 < half_angle <= pi (in radians).")

    # 1) translate
    p = z - o

    # 2) rotate by -sector_dir so that sector_dir aligns with +x axis
    c, s = np.cos(sector_dir), np.sin(sector_dir)
    x = c * p[..., 0] + s * p[..., 1]
    y = -s * p[..., 0] + c * p[..., 1]

    r = np.sqrt(x * x + y * y)
    if r < 1e-12:
        # at origin, return distance to inner arc
        return r_min
    phi = np.arctan2(y, x)

    # ---- helper: point to segment distance (vectorized) ----
    def dist_point_segment(px, py, ax, ay, bx, by):
        vx, vy = bx - ax, by - ay
        wx, wy = px - ax, py - ay
        vv = vx * vx + vy * vy
        t = np.where(vv > 1e-12, (wx * vx + wy * vy) / vv, 0.0)
        t = np.clip(t, 0.0, 1.0)
        cx = ax + t * vx
        cy = ay + t * vy
        return np.sqrt((px - cx) ** 2 + (py - cy) ** 2)

    # 3) distance to outer/inner arc segments (clamp angle to [-alpha, alpha])
    ang = np.clip(phi, -half_angle, half_angle)

    qox = r_max * np.cos(ang)
    qoy = r_max * np.sin(ang)
    d_outer_arc = np.sqrt((x - qox) ** 2 + (y - qoy) ** 2)

    qix = r_min * np.cos(ang)
    qiy = r_min * np.sin(ang)
    d_inner_arc = np.sqrt((x - qix) ** 2 + (y - qiy) ** 2)

    # 4) distance to the two radial edges (line segments between r_min and r_max)
    ca, sa = np.cos(half_angle), np.sin(half_angle)

    # +alpha edge segment endpoints
    axp, ayp = r_min * ca, r_min * sa
    bxp, byp = r_max * ca, r_max * sa
    d_edge_p = dist_point_segment(x, y, axp, ayp, bxp, byp)

    # -alpha edge segment endpoints
    axm, aym = r_min * ca, -r_min * sa
    bxm, bym = r_max * ca, -r_max * sa
    d_edge_m = dist_point_segment(x, y, axm, aym, bxm, bym)

    # unsigned distance to boundary = min over boundary pieces
    d = np.minimum.reduce([d_outer_arc, d_inner_arc, d_edge_p, d_edge_m])

    # inside test
    inside = (r >= r_min) & (r <= r_max) & (np.abs(phi) <= half_angle)

    # signed distance
    return np.where(inside, -d, d)


# convenience wrapper if you prefer full angle
def sd_annular_sector_fov(z, sector_origin, sector_dir, r_min, r_max, fov_angle):
    return sd_annular_sector(z, sector_origin, sector_dir, r_min, r_max, 0.5 * fov_angle)

def sd_annular_sector_fov_gradient(x_t, x_r, heading_r, fov_angle, eps=1e-12, r_min=0.5, r_max=5.0):
    """
    Analytic (piecewise) gradient of the annular-sector SDF used for FOV.

    Returns:
        dsd/dx_t, dsd/dy_t, dsd/dx_r, dsd/dy_r, dsd/dheading_r

    Sign convention matches sd_annular_sector_fov:
        inside -> sd < 0, outside -> sd > 0

    Notes:
      - This is piecewise because sd is defined via min() over boundary pieces.
      - At points where the closest boundary piece is not unique (corners / tie),
        the gradient is not well-defined; we return the gradient of one active piece.
      - `eps` is only used as a numerical safeguard for division by zero (NOT finite-difference).
    """
    x_t = np.asarray(x_t, dtype=float).reshape(2,)
    x_r = np.asarray(x_r, dtype=float).reshape(2,)

    half_angle = 0.5 * float(fov_angle)

    # translate
    dx, dy = (x_t - x_r)

    # rotate by -heading_r into the sector frame (sector_dir aligns with +x axis)
    c, s = np.cos(heading_r), np.sin(heading_r)
    x = c * dx + s * dy
    y = -s * dx + c * dy

    # polar
    r = np.sqrt(x * x + y * y)
    if r < eps:
        # - heading 방향으로 벗어나는 방향으로 그라디언트 설정
        dsd_dx_t = -c
        dsd_dy_t = -s
        dsd_dx_r = -dsd_dx_t
        dsd_dy_r = -dsd_dy_t
        return dsd_dx_t, dsd_dy_t, dsd_dx_r, dsd_dy_r, 0.0
    phi = np.arctan2(y, x)

    # inside test (same as sd_annular_sector)
    inside = (r >= r_min) and (r <= r_max) and (abs(phi) <= half_angle)

    # helper: closest point on segment AB to point P
    def closest_point_on_segment(px, py, ax, ay, bx, by):
        vx, vy = bx - ax, by - ay
        wx, wy = px - ax, py - ay
        vv = vx * vx + vy * vy
        if vv < eps:
            return ax, ay
        t = (wx * vx + wy * vy) / vv
        t = np.clip(t, 0.0, 1.0)
        return ax + t * vx, ay + t * vy

    # Candidate 1/2: outer/inner arc (with angular clamp)
    ang = np.clip(phi, -half_angle, half_angle)

    qox, qoy = r_max * np.cos(ang), r_max * np.sin(ang)
    qix, qiy = r_min * np.cos(ang), r_min * np.sin(ang)

    # Candidate 3/4: radial edges as segments between r_min and r_max at +/-half_angle
    ca, sa = np.cos(half_angle), np.sin(half_angle)

    # +edge endpoints
    axp, ayp = r_min * ca, r_min * sa
    bxp, byp = r_max * ca, r_max * sa
    qpx, qpy = closest_point_on_segment(x, y, axp, ayp, bxp, byp)

    # -edge endpoints
    axm, aym = r_min * ca, -r_min * sa
    bxm, bym = r_max * ca, -r_max * sa
    qmx, qmy = closest_point_on_segment(x, y, axm, aym, bxm, bym)

    # distances to each candidate closest point + unit gradient
    def dist_and_grad(px, py, qx, qy):
        vx, vy = px - qx, py - qy
        d = np.sqrt(vx * vx + vy * vy)
        if d < eps:
            return 0.0, np.array([0.0, 0.0], dtype=float)
        return d, np.array([vx / d, vy / d], dtype=float)  # unit vector from q to p

    d_outer, g_outer = dist_and_grad(x, y, qox, qoy)
    d_inner, g_inner = dist_and_grad(x, y, qix, qiy)
    d_ep,    g_ep    = dist_and_grad(x, y, qpx, qpy)
    d_em,    g_em    = dist_and_grad(x, y, qmx, qmy)

    ds = np.array([d_outer, d_inner, d_ep, d_em], dtype=float)
    gs = np.stack([g_outer, g_inner, g_ep, g_em], axis=0)  # (4,2)

    idx = int(np.argmin(ds))
    grad_d_local = gs[idx]  # unit grad of unsigned distance in local frame

    # signed distance gradient in local frame
    sgn = -1.0 if inside else 1.0
    grad_sd_local = sgn * grad_d_local

    # world gradient w.r.t target position:
    # local = R(-theta)*(x_t - x_r)  =>  ∂sd/∂x_t = R(theta)*grad_local
    gx_t = c * grad_sd_local[0] - s * grad_sd_local[1]
    gy_t = s * grad_sd_local[0] + c * grad_sd_local[1]

    # w.r.t robot position (origin): opposite sign
    gx_r = -gx_t
    gy_r = -gy_t

    # w.r.t heading: chain rule through local coordinates
    # x =  c*dx + s*dy,   y = -s*dx + c*dy
    # dx/dtheta = y,  dy/dtheta = -x  (in local frame)
    dsd_dtheta = grad_sd_local[0] * y - grad_sd_local[1] * x

    return gx_t, gy_t, gx_r, gy_r, float(dsd_dtheta)



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
    gamma_value = 1.0
    gamma_tf = gamma_tf_FOV(mu_t, Sigma_t, mu_r, Sigma_r, r_min, r_max, fov_angle)
    gamma_value *= gamma_tf
    gamma_lo, _, _ = gamma_lo_LOS(mu_t, Sigma_t, mu_r, Sigma_r, obs_origins, obs_radii)
    gamma_k = gamma_tf * np.exp(np.sum(np.log(np.clip(gamma_lo, 1e-12, 1.0))))
    return gamma_k, gamma_tf, gamma_lo


if __name__ == "__main__":
    # Example usage
    mu_t_ = np.array([0.0, 0.0])
    Sigma_t = np.array([[1.0, 0.0], 
                        [0.0, 1.0]])
    mu_r = np.array([0.0, 0.0, 0, 0.0])  # x, y, theta, v
    Sigma_r = np.array([[0.001, 0.0, 0.0, 0.0],
                        [0.0, 0.001, 0.0, 0.0],
                        [0.0, 0.0, 0.001, 0.0],
                        [0.0, 0.0, 0.0, 0.001]])
    heading_r = np.pi / 4  # 45 degrees
    r_min = 0.8
    r_max = 2.0
    fov_angle = np.pi / 3  # 60 degrees

    eps = 1e-6
    mu_t_ = mu_t_ + np.array([eps, 0])
    obs_temp = np.zeros((0, 2))
    obs_radius_temp = 0
    gamma_forward = gamma_k_bpod(mu_t_, Sigma_t, mu_r, Sigma_r, r_min, r_max, fov_angle, obs_temp, obs_radius_temp)[0]
    mu_t_ = mu_t_ - np.array([2*eps, 0])
    gamma_back = gamma_k_bpod(mu_t_, Sigma_t, mu_r, Sigma_r, r_min, r_max, fov_angle, obs_temp, obs_radius_temp)[0]
    
    
    grad_gamma = (gamma_forward - gamma_back) / (2*eps)
    print("FOV SDF Gradient:", grad_gamma)



    # import matplotlib.pyplot as plt

    # xx, yy = np.meshgrid(np.linspace(-5, 10, 151), np.linspace(-5, 10, 151))
    # zz = np.stack([xx, yy], axis=-1)

    # prob_map = np.zeros(xx.shape)
    # sd_value_map = np.zeros(xx.shape)
    # for i in range(xx.shape[0]):
    #     for j in range(xx.shape[1]):
    #         mu_t = np.array([xx[i, j], yy[i, j]])
    #         prob_map[i, j] = gamma_tf_FOV(
    #             mu_t,
    #             Sigma_t,
    #             mu_r,
    #             Sigma_r,
    #             r_min,
    #             r_max,
    #             fov_angle,
    #         )
    #         sd_value_map[i, j] = sd_annular_sector_fov(
    #             mu_t,
    #             sector_origin=mu_r[0:2],
    #             sector_dir=mu_r[2],
    #             r_min=r_min,
    #             r_max=r_max,
    #             fov_angle=fov_angle,
    #         )
    # plt.figure()

    # # plt.contourf(xx, yy, sd_value_map, levels=200, cmap="RdBu_r")
    # # plt.colorbar(label="Signed Distance")
    # # plt.contour(xx, yy, sd_value_map, levels=[0.0], colors="k", linewidths=2)
    # plt.contour(xx, yy, prob_map, levels=[0.5], colors="g", linewidths=2)
    # plt.contour(xx, yy, prob_map, levels=[0.8], colors="r", linewidths=2)
    # plt.contour(xx, yy, prob_map, levels=[0.9], colors="r", linewidths=2)
    # plt.contour(xx, yy, prob_map, levels=[0.7], colors="r", linewidths=2)
    # plt.contourf(xx, yy, prob_map, levels=50, cmap='viridis')
    # plt.plot(mu_r[0], mu_r[1], "ro", label="Robot", markersize=8)
    # plt.plot(mu_t_[0], mu_t_[1], "bx", label="Target Mean", markersize=8)
    # plt.colorbar(label='Probability of being in FOV')
    # plt.xlabel('X position')
    # plt.ylabel('Y position')
    # plt.title('Probability Map of Target in Robot FOV')
    # plt.show()

    # #################################################################################

    # xx, yy = np.meshgrid(np.linspace(-5, 10, 151), np.linspace(-5, 10, 151))
    # zz = np.stack([xx, yy], axis=-1)
    # # prob_map = np.zeros(xx.shape)
    # sd_value_map = np.zeros(xx.shape)
    # mu_t = np.array([5.0, 0.0])

    # for i in range(xx.shape[0]):
    #     for j in range(xx.shape[1]):
    #         obs_origin = np.array([xx[i, j], yy[i, j]])
    #         # prob_map[i, j] = gamma_lo_LOS(
    #         #     mu_t,
    #         #     Sigma_t,
    #         #     mu_r,
    #         #     Sigma_r,
    #         #     obs_origin=obs_origin,
    #         #     obs_radius=1.0,
    #         # )
    #         sd_value_map[i, j] = sd_LOS(
    #             obs_origin=obs_origin,
    #             obs_radius=1.0,
    #             los_point1=mu_r[0:2],
    #             los_point2=mu_t,
    #         )

    # prob_map = gamma_lo_LOS(
    #     mu_t,
    #     Sigma_t,
    #     mu_r,
    #     Sigma_r,
    #     obs_origins=zz.reshape(-1,2),
    #     obs_radii=1.0,
    # )[0].reshape(xx.shape)
    # # sd_value_map = sd_LOS(
    # #     obs_origin=zz.reshape(-1,2),
    # #     obs_radius=1.0,
    # #     los_point1=mu_r[0:2],
    # #     los_point2=mu_t,
    # # ).reshape(xx.shape)
    

    # plt.figure()
    # # plt.contourf(xx, yy, sd_value_map, levels=200, cmap="RdBu_r")
    # # plt.colorbar(label="Signed Distance")
    # plt.contour(xx, yy, sd_value_map, levels=[0.0], colors="k", linewidths=2)
    # plt.contourf(xx, yy, prob_map, levels=50, cmap='viridis')
    # plt.colorbar(label='Probability of being in LOS')
    # plt.xlabel('X position')
    # plt.ylabel('Y position')
    # plt.title('Probability Map of Target in Robot LOS')
    # plt.show()

    # # #################################################################################

    # xx, yy = np.meshgrid(np.linspace(-5, 10, 200), np.linspace(-5, 10, 200))
    # zz = np.stack([xx, yy], axis=-1)
    # prob_map = np.zeros(xx.shape)
    # sd_value_map = np.zeros(xx.shape)
    # # for i in range(xx.shape[0]):
    # #     for j in range(xx.shape[1]):
    # #         obs_origin = np.array([xx[i, j], yy[i, j]])
    # #         prob_map[i, j] = gamma_ro_collision(
    # #             mu_r,
    # #             Sigma_r,
    # #             obs_origin=obs_origin,
    # #             obs_radius=1.0,
    # #         )
    # #         sd_value_map[i, j] = sd_robot_obstacle(
    # #             robot_point=mu_r[0:2],
    # #             obs_origin=obs_origin,
    # #             obs_radius=1.0,
    # #         )
    # # sd_value_map = sd_robot_obstacle(
    # #     robot_point=mu_r[0:2],
    # #     obs_origin=zz.reshape(-1,2),
    # #     obs_radius=1.0,
    # # ).reshape(xx.shape)
    # prob_map = gamma_ro_collision(
    #     mu_r,
    #     Sigma_r,
    #     obs_origins=zz.reshape(-1,2),
    #     obs_radii=1.0,
    # )[0].reshape(xx.shape)

    # plt.figure()
    # # plt.contourf(xx, yy, sd_value_map, levels=200, cmap="RdBu_r")
    # # plt.colorbar(label="Signed Distance")
    # # plt.contour(xx, yy, sd_value_map, levels=[0.0], colors="k", linewidths=2)
    # plt.contourf(xx, yy, prob_map, levels=50, cmap='viridis')
    # plt.colorbar(label='Probability of Robot-Obstacle Collision')
    # plt.xlabel('X position')
    # plt.ylabel('Y position')
    # plt.title('Probability Map of Robot-Obstacle Collision')
    # plt.show()