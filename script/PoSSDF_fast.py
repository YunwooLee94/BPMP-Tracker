from sd_annular_sector import sd_annular_sector_fov, sd_annular_sector_fov_gradient
from sd_LOS import sd_LOS, sd_LOS_gradient
from sd_robot_obstacle import sd_robot_obstacle, sd_robot_obstacle_gradient
import numpy as np
from scipy.special import erf

def _phi_from_sd(sd_value: float, var_proj: float, sense: str = "leq") -> float:
    """Fast probability for linearized signed-distance constraint.

    We assume sd_lin ~ N(mean=sd_value, var=var_proj).

    - sense="leq": Pr(sd_lin <= 0)  (used for 'inside' events when sd<=0 defines satisfaction)
    - sense="geq": Pr(sd_lin >= 0)  (used for 'clear' events when sd>=0 defines satisfaction)

    Uses erf for numerical stability.
    """
    var = float(var_proj)
    if var <= 1e-12:
        if sense == "leq":
            return 1.0 if sd_value <= 0.0 else 0.0
        else:
            return 1.0 if sd_value >= 0.0 else 0.0

    z = sd_value / (np.sqrt(var) * np.sqrt(2.0))
    # Pr(sd <= 0) = 0.5 * (1 - erf( sd/(sigma*sqrt(2)) ))
    p_leq = 0.5 * (1.0 - erf(z))
    if sense == "leq":
        return float(p_leq)
    # Pr(sd >= 0) = 1 - Pr(sd <= 0)
    return float(1.0 - p_leq)


def linear_inequality_gaussian(a : np.ndarray, b : float, mu: np.ndarray, Sigma: np.ndarray):
    """Compute the signed distance from a Gaussian distribution to a linear inequality constraint.

    The linear inequality constraint is defined as a^T x <= b.
    The Gaussian distribution is defined by mean mu and covariance Sigma.

    Args:
        a (np.ndarray): Coefficient vector of the linear inequality constraint.
        b (float): Right-hand side of the linear inequality constraint.
        mu (np.ndarray): Mean of the Gaussian distribution.
        Sigma (np.ndarray): Covariance matrix of the Gaussian distribution.
    Returns:
        float: Pr(a^T x <= b)
    """
    a = a.reshape(-1, 1)  # ensure a is a column vector
    mean_proj = a.T @ mu  # projected mean
    var_proj = a.T @ Sigma @ a  # projected variance
    std_proj = np.sqrt(var_proj).item()  # projected standard deviation

    return 1/2 * (1 - erf((mean_proj - b) / (std_proj * np.sqrt(2.0) + 1e-12)))


def gamma_tf_FOV(
    mu_t: np.ndarray,
    Sigma_t: np.ndarray,
    mu_r:  np.ndarray,
    Sigma_r: np.ndarray,
    r_min: float,
    r_max: float,
    fov_angle: float,
) -> float:
    """Probability that the target is within the robot FOV (annular sector).

    Event: sd_FOV(mu_t; mu_r) <= 0  (inside => negative).
    We linearize sd around the mean and compute Pr(sd_lin <= 0).
    """
    # signed distance at mean
    sd_value = sd_annular_sector_fov(
        mu_t,
        sector_origin=mu_r[0:2],
        sector_dir=mu_r[2],
        r_min=r_min,
        r_max=r_max,
        fov_angle=fov_angle,
    )

    # analytic (piecewise) gradient of sd wrt (x_t, y_t, x_r, y_r, heading)
    dsd_dx_t, dsd_dy_t, dsd_dx_r, dsd_dy_r, dsd_dheading = sd_annular_sector_fov_gradient(
        mu_t, mu_r[0:2], mu_r[2], fov_angle, r_min=r_min, r_max=r_max
    )

    # projected variance: block-diagonal Sigma = diag(Sigma_r, Sigma_t)
    # robot part is [x_r, y_r, heading, v] (v has zero grad here)
    g_r = np.array([dsd_dx_r, dsd_dy_r, dsd_dheading, 0.0], dtype=float)
    var_r = float(g_r @ (Sigma_r @ g_r))

    # target part is [x_t, y_t]
    g_t = np.array([dsd_dx_t, dsd_dy_t], dtype=float)
    var_t = float(g_t @ (Sigma_t @ g_t))

    return _phi_from_sd(sd_value, var_r + var_t, sense="leq")

def gamma_lo_LOS(
    mu_t: np.ndarray,
    Sigma_t: np.ndarray,
    mu_r:  np.ndarray,
    Sigma_r: np.ndarray,
    obs_origin: np.ndarray,
    obs_radius: float,
) -> float:
    """Probability that the target is in LOS (not occluded) wrt a circular obstacle.

    We use sd_LOS = dist(segment(robot->target), obstacle_center) - obs_radius.
    Event for *clear LOS*: sd_LOS >= 0  (no intersection).
    """
    sd_value = sd_LOS(
        obs_origin=obs_origin,
        obs_radius=obs_radius,
        los_point1=mu_r[0:2],
        los_point2=mu_t,
    )

    # gradient wrt target and robot xy (heading/velocity do not affect LOS geometry)
    dsd_dx_t, dsd_dy_t, dsd_dx_r, dsd_dy_r = sd_LOS_gradient(mu_t, mu_r[0:2], obs_origin, obs_radius)

    g_r = np.array([dsd_dx_r, dsd_dy_r, 0.0, 0.0], dtype=float)
    var_r = float(g_r @ (Sigma_r @ g_r))

    g_t = np.array([dsd_dx_t, dsd_dy_t], dtype=float)
    var_t = float(g_t @ (Sigma_t @ g_t))

    # NOTE: LOS is sd >= 0 (paper Eq.18b), so use sense="geq"
    return _phi_from_sd(sd_value, var_r + var_t, sense="geq")

def gamma_ro_collision(
    mu_r:  np.ndarray,
    Sigma_r: np.ndarray,
    obs_origin: np.ndarray,
    obs_radius: float,
) -> float:
    """Collision probability between robot position and a circular obstacle.

    We use sd_robot_obstacle = ||p_r - c|| - r.
    Event for collision: sd <= 0.
    """
    sd_value = sd_robot_obstacle(
        robot_point=mu_r[0:2],
        obs_origin=obs_origin,
        obs_radius=obs_radius,
    )

    dsd_dx_r, dsd_dy_r = sd_robot_obstacle_gradient(mu_r[0:2], obs_origin, obs_radius)

    g_r = np.array([dsd_dx_r, dsd_dy_r, 0.0, 0.0], dtype=float)
    var_r = float(g_r @ (Sigma_r @ g_r))

    return _phi_from_sd(sd_value, var_r, sense="leq")


if __name__ == "__main__":
    # Example usage
    mu_t = np.array([5.0, 5.0])
    Sigma_t = np.array([[1.0, 0.0], 
                        [0.0, 1.0]])
    mu_r = np.array([0.0, 0.0, 0.0, 0.0])  # x, y, theta, v
    Sigma_r = np.array([[0.5, 0.0, 0.0, 0.0],
                        [0.0, 0.5, 0.0, 0.0],
                        [0.0, 0.0, 0.1, 0.0],
                        [0.0, 0.0, 0.0, 0.1]])
    heading_r = np.pi / 4  # 45 degrees
    r_min = 1.0
    r_max = 10.0
    fov_angle = np.pi / 3  # 60 degrees

    gamma_tf = gamma_tf_FOV(mu_t, Sigma_t, mu_r, Sigma_r, r_min, r_max, fov_angle)
    print(f"Probability that target is within FOV: {gamma_tf}")


    import matplotlib.pyplot as plt

    # xx, yy = np.meshgrid(np.linspace(-5, 10, 200), np.linspace(-5, 10, 200))
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
    # plt.contour(xx, yy, sd_value_map, levels=[0.0], colors="k", linewidths=2)
    # plt.contourf(xx, yy, prob_map, levels=50, cmap='viridis')
    # plt.colorbar(label='Probability of being in FOV')
    # plt.xlabel('X position')
    # plt.ylabel('Y position')
    # plt.title('Probability Map of Target in Robot FOV')
    # plt.show()

    # #################################################################################

    xx, yy = np.meshgrid(np.linspace(-5, 10, 200), np.linspace(-5, 10, 200))
    zz = np.stack([xx, yy], axis=-1)
    prob_map = np.zeros(xx.shape)
    sd_value_map = np.zeros(xx.shape)
    mu_t = np.array([5.0, 5.0])

    for i in range(xx.shape[0]):
        for j in range(xx.shape[1]):
            obs_origin = np.array([xx[i, j], yy[i, j]])
            prob_map[i, j] = gamma_lo_LOS(
                mu_t,
                Sigma_t,
                mu_r,
                Sigma_r,
                obs_origin=obs_origin,
                obs_radius=1.0,
            )
            sd_value_map[i, j] = sd_LOS(
                obs_origin=obs_origin,
                obs_radius=1.0,
                los_point1=mu_r[0:2],
                los_point2=mu_t,
            )
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

    #################################################################################

    # xx, yy = np.meshgrid(np.linspace(-5, 10, 200), np.linspace(-5, 10, 200))
    # zz = np.stack([xx, yy], axis=-1)
    # prob_map = np.zeros(xx.shape)
    # sd_value_map = np.zeros(xx.shape)
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
    # plt.figure()
    # # plt.contourf(xx, yy, sd_value_map, levels=200, cmap="RdBu_r")
    # # plt.colorbar(label="Signed Distance")
    # plt.contour(xx, yy, sd_value_map, levels=[0.0], colors="k", linewidths=2)
    # plt.contourf(xx, yy, prob_map, levels=50, cmap='viridis')
    # plt.colorbar(label='Probability of Robot-Obstacle Collision')
    # plt.xlabel('X position')
    # plt.ylabel('Y position')
    # plt.title('Probability Map of Robot-Obstacle Collision')
    # plt.show()