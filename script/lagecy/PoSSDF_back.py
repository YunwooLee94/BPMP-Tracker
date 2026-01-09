from sd_annular_sector import sd_annular_sector_fov, sd_annular_sector_fov_gradient
from sd_LOS import sd_LOS, sd_LOS_gradient
from sd_robot_obstacle import sd_robot_obstacle, sd_robot_obstacle_gradient
import numpy as np
from scipy.special import erf

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
    """Compute the probability that a target at position x_t is within the FOV of a robot at position x_r.

    The FOV is defined as an annular sector with inner radius r_min, outer radius r_max, and angle fov_angle.

    Args:
        mu_t (np.ndarray): Mean of the target state (x,y : 2D).
        Sigma_t (np.ndarray): Covariance of the target state(2D).
        mu_r (np.ndarray): Mean of the robot state (x,y,theta,v : 4D).
        Sigma_r (np.ndarray): Covariance of the robot state (4D).
        r_min (float): Minimum range of the FOV.
        r_max (float): Maximum range of the FOV.
        fov_angle (float): Field of view angle (radians).
    Returns:
        float: Probability that the target is within the FOV.
    """
    # Compute signed distance from mean position to FOV boundary
    sd_value = sd_annular_sector_fov(
        mu_t,
        sector_origin=mu_r[0:2],
        sector_dir=mu_r[2],
        r_min=r_min,
        r_max=r_max,
        fov_angle=fov_angle,
    )

    # Compute gradient of signed distance w.r.t target position
    sd_value_dx_t, sd_value_dy_t, sd_value_dx_r, sd_value_dy_r, sd_value_dheading_r = \
        sd_annular_sector_fov_gradient(mu_t, mu_r[0:2], mu_r[2], fov_angle, r_min=r_min, r_max=r_max)
    
    sd_grad = np.array([
        sd_value_dx_r,
        sd_value_dy_r,
        sd_value_dheading_r,
        0.0,  # robot velocity has no effect on FOV
        sd_value_dx_t,
        sd_value_dy_t,
    ]).reshape(-1, 1)  # shape (6,1)

    mu = np.concatenate([mu_r, mu_t], axis=0)  # shape (6,)
    Sigma = np.zeros((6,6))
    Sigma[0:4, 0:4] = Sigma_r
    Sigma[4:6, 4:6] = Sigma_t

    # Compute probability using linear inequality Gaussian
    prob_in_fov = linear_inequality_gaussian(
        a=sd_grad,
        b=sd_grad.T @ mu - sd_value,
        mu=mu,
        Sigma=Sigma,
    )

    return prob_in_fov

def gamma_lo_LOS(
    mu_t: np.ndarray,
    Sigma_t: np.ndarray,
    mu_r:  np.ndarray,
    Sigma_r: np.ndarray,
    obs_origin: np.ndarray,
    obs_radius: float,
) -> float:
    """Compute the probability that a target at position x_t is in line-of-sight (LOS) of a robot at position x_r.

    The LOS is defined as the line segment between the robot and the target, with an obstacle defined by its origin and radius.

    Args:
        mu_t (np.ndarray): Mean of the target state (x,y : 2D).
        Sigma_t (np.ndarray): Covariance of the target state(2D).
        mu_r (np.ndarray): Mean of the robot state (x,y,theta,v : 4D).
        Sigma_r (np.ndarray): Covariance of the robot state (4D).
        obs_origin (np.ndarray): Origin of the obstacle.
        obs_radius (float): Radius of the obstacle.
    Returns:
        float: Probability that the obstacle is in LOS.
    """
    # Compute signed distance from mean position to LOS boundary
    sd_value = sd_LOS(
        obs_origin=obs_origin,
        obs_radius=obs_radius,
        los_point1=mu_r[0:2],
        los_point2=mu_t,
    )

    # Compute gradient of signed distance w.r.t target and robot position
    sd_value_dx_t, sd_value_dy_t, sd_value_dx_r, sd_value_dy_r = \
        sd_LOS_gradient(mu_t, mu_r[0:2], obs_origin, obs_radius)
    
    sd_grad = np.array([
        sd_value_dx_r,
        sd_value_dy_r,
        0.0,  # robot heading has no effect on LOS
        0.0,  # robot velocity has no effect on LOS
        sd_value_dx_t,
        sd_value_dy_t,
    ]).reshape(-1, 1)  # shape (6,1)

    mu = np.concatenate([mu_r, mu_t], axis=0)  # shape (6,)
    Sigma = np.zeros((6,6))
    Sigma[0:4, 0:4] = Sigma_r
    Sigma[4:6, 4:6] = Sigma_t
    # Compute probability using linear inequality Gaussian
    prob_in_los = linear_inequality_gaussian(
        a=sd_grad,
        b=sd_grad.T @ mu - sd_value,
        mu=mu,
        Sigma=Sigma,
    )
    prob_outof_los = 1 - prob_in_los  # gamma_lo = 1 - Pr(sd_LOS > 0)
    return prob_outof_los

def gamma_ro_collision(
    mu_r:  np.ndarray,
    Sigma_r: np.ndarray,
    obs_origin: np.ndarray,
    obs_radius: float,
) -> float:
    """Compute the probability that a robot at position x_r collides with an obstacle.

    The obstacle is defined by its origin and radius.

    Args:
        mu_r (np.ndarray): Mean of the robot state (x,y,theta,v : 4D).
        Sigma_r (np.ndarray): Covariance of the robot state (4D).
        obs_origin (np.ndarray): Origin of the obstacle.
        obs_radius (float): Radius of the obstacle.
    Returns:
        float: Probability that the robot collides with the obstacle.
    """
    # Compute signed distance from mean position to obstacle boundary
    sd_value = sd_robot_obstacle(
        robot_point=mu_r[0:2],
        obs_origin=obs_origin,
        obs_radius=obs_radius,
    )

    # Compute gradient of signed distance w.r.t robot position
    sd_value_dx_r, sd_value_dy_r = \
        sd_robot_obstacle_gradient(mu_r[0:2], obs_origin, obs_radius)
    
    sd_grad = np.array([
        sd_value_dx_r,
        sd_value_dy_r,
        0.0,  # robot heading has no effect on collision
        0.0,  # robot velocity has no effect on collision
    ]).reshape(-1, 1)  # shape (4,1)

    mu = mu_r  # shape (4,)
    Sigma = Sigma_r  # shape (4,4)
    # Compute probability using linear inequality Gaussian
    prob_collision = linear_inequality_gaussian(
        a=sd_grad,
        b=sd_grad.T @ mu - sd_value,
        mu=mu,
        Sigma=Sigma,
    )

    return prob_collision


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

    xx, yy = np.meshgrid(np.linspace(-5, 10, 200), np.linspace(-5, 10, 200))
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
    plt.contourf(xx, yy, prob_map, levels=50, cmap='viridis')
    plt.colorbar(label='Probability of being in FOV')
    plt.xlabel('X position')
    plt.ylabel('Y position')
    plt.title('Probability Map of Target in Robot FOV')
    plt.show()

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

    xx, yy = np.meshgrid(np.linspace(-5, 10, 200), np.linspace(-5, 10, 200))
    zz = np.stack([xx, yy], axis=-1)
    prob_map = np.zeros(xx.shape)
    sd_value_map = np.zeros(xx.shape)
    for i in range(xx.shape[0]):
        for j in range(xx.shape[1]):
            obs_origin = np.array([xx[i, j], yy[i, j]])
            prob_map[i, j] = gamma_ro_collision(
                mu_r,
                Sigma_r,
                obs_origin=obs_origin,
                obs_radius=1.0,
            )
            sd_value_map[i, j] = sd_robot_obstacle(
                robot_point=mu_r[0:2],
                obs_origin=obs_origin,
                obs_radius=1.0,
            )
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