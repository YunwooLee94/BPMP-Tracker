import numpy as np
from bpmp_tracker.msg import ObjectStateList, ObjectState
from PoSSDF import gamma_lo_LOS, gamma_ro_collision, gamma_tf_FOV
from dataclasses import dataclass
import time
import cvxpy as cp

DT = 0.1  # time step duration
OBS_RADIUS = 0.5  # obstacle radius for collision checking

@dataclass
class BeliefState():
    mean: np.ndarray
    covariance: np.ndarray
    dim: int

@dataclass
class FOVParams():
    fov: float
    r_min: float
    r_max: float

def robot_step(b_r: BeliefState, control: np.ndarray, process_noise: np.ndarray):
    """
    control: shape (2,) -> [omega, acc]
    """
    x, y, theta, v = b_r.mean
    w, a = control

    # Simple bicycle model for robot motion
    x_new = x + v * np.cos(theta) * DT
    y_new = y + v * np.sin(theta) * DT
    theta_new = theta + w * DT
    v_new = v + a * DT

    mean_new = np.array([x_new, y_new, theta_new, v_new])
    # Covariance update can be added here if needed
    A_mat = np.array([
        [1, 0, -v * np.sin(theta) * DT, np.cos(theta) * DT],
        [0, 1,  v * np.cos(theta) * DT, np.sin(theta) * DT],
        [0, 0, 1, 0],
        [0, 0, 0, 1]
    ])
    covariance_new = A_mat @ b_r.covariance @ A_mat.T + process_noise  # Simple linearized covariance update
    return BeliefState(mean_new, covariance_new, dim=4)

def target_step(b_t: BeliefState, control_target: np.ndarray, process_noise: np.ndarray, sensor_noise: np.ndarray,
                robot_belief: BeliefState, obstacle_states: np.ndarray, fov_params: FOVParams):
    """
    control_target: shape (2,) -> [vx, vy]
    """
    x, y = b_t.mean
    vx, vy = control_target

    # Simple random walk model for target motion
    x_new = x + vx * DT
    y_new = y + vy * DT

    mean_new = np.array([x_new, y_new])
    # Covariance update can be added here if needed
    A_mat = np.array([
        [1, 0],
        [0, 1]
    ])
    covariance_prediction = A_mat @ b_t.covariance @ A_mat.T + process_noise  # Simple linearized covariance update
    gamma_k = get_gamma_k(mean_new, covariance_prediction, robot_belief, obstacle_states, fov_params)
    X_r_t_nrom = np.linalg.norm(robot_belief.mean[0:2] - mean_new)
    c_tild = np.array([[(mean_new[0] - robot_belief.mean[0])/X_r_t_nrom, (mean_new[1] - robot_belief.mean[1])/X_r_t_nrom], 
                       [-(mean_new[1] - robot_belief.mean[1])/X_r_t_nrom**2, (mean_new[0] - robot_belief.mean[0])/X_r_t_nrom**2]])
    k_tild = covariance_prediction @ c_tild.T @ np.linalg.inv(c_tild @ covariance_prediction @ c_tild.T + sensor_noise)
    covariance_new = covariance_prediction - gamma_k * (k_tild @ c_tild @ covariance_prediction)
    return BeliefState(mean_new, covariance_new, dim=2), gamma_k

def get_gamma_k(target_mean: np.ndarray, target_cov: np.ndarray, robot_belief: BeliefState, obstacle_states: np.ndarray, fov_params: FOVParams):
    """
    Compute visibility indicator gamma_k based on whether the target is within the robot's field of view
    and not occluded by obstacles.
    """
    gamma_value = 1.0
    gamma_tf = gamma_tf_FOV(target_mean, target_cov, robot_belief.mean, robot_belief.covariance, fov_params.r_min, fov_params.r_max, fov_params.fov)
    gamma_value *= gamma_tf
    # for obs in obstacle_states: # number of obstacles * 2 [px, py]
    #     obs_center = np.array([obs[0], obs[1]])
    #     gamma_lo = gamma_lo_LOS(target_mean, target_cov, robot_belief.mean, robot_belief.covariance, obs_center, OBS_RADIUS)
    #     gamma_value *= gamma_lo
    gamma_lo, _, _ = gamma_lo_LOS(target_mean, target_cov, robot_belief.mean, robot_belief.covariance, obstacle_states, OBS_RADIUS)
    gamma_value *= np.prod(gamma_lo)
    return gamma_value

def entropy(covariance: np.ndarray):
    """
    Compute the differential entropy of a Gaussian distribution with given covariance.
    """
    dim = covariance.shape[0]
    sign, logdet = np.linalg.slogdet(covariance)
    if sign <= 0:
        raise ValueError("Covariance matrix must be positive definite.")
    return 0.5 * logdet + 0.5 * dim * (1 + np.log(2 * np.pi))

def get_J_m(b_r0, b_t0, u_bar, robot_process_noise, 
            target_process_noise, target_sensor_noise, target_control_predicted, 
            obstacle_states, fov, eta, function_J, gamma_collision_threshold):
    # (1) nominal rollout for robot and target (현재 u_bar 기준)
    b_r_list  = [b_r0]
    for u in u_bar:
        b_r_next = robot_step(b_r_list[-1], u, robot_process_noise)
        b_r_list.append(b_r_next)
    
    b_t_list  = [b_t0]
    gamma_list = []
    gamma_ro_list = []
    for k in range(len(u_bar)):
        if target_control_predicted is not None:
            u_t = target_control_predicted[k]
        else:
            u_t = np.array([0.0, 0.0])  # default zero control for target

        b_t_next, gamma_k = target_step(b_t_list[-1], u_t, target_process_noise, target_sensor_noise,
                                        b_r_list[k+1], obstacle_states[k], fov)
        b_t_list.append(b_t_next)
        gamma_list.append(gamma_k)

        if len(obstacle_states[k]) == 0:
            gamma_ro = 0.0
        for obs in obstacle_states[k]:
            obs_origin = np.array([obs[0], obs[1]])
            gamma_ro, _, _ = gamma_ro_collision(b_r_list[k+1].mean, b_r_list[k+1].covariance, obs_origin, OBS_RADIUS)
        gamma_ro_list.append(gamma_ro)
    
    # get J
    if function_J == "1":
        J = sum([entropy(b_t.covariance) for b_t in b_t_list[1:]])
    elif function_J == "2":
        J = -sum(gamma_list)
    
    # get J_m
    J_m = J + eta * sum([max(0.0, gamma_ro_list[k] - gamma_collision_threshold) for k in range(len(gamma_ro_list))]) 
    return J_m

def get_J_m_grad(b_r0, b_t0, u_bar, robot_process_noise, 
            target_process_noise, target_sensor_noise, target_control_predicted, 
            obstacle_states, fov, eta, function_J, gamma_collision_threshold,
            grad_delta=1e-6):
    N, nu = u_bar.shape
    J_m_grad = np.zeros((N, nu))
    J_m_base = get_J_m(b_r0, b_t0, u_bar, robot_process_noise, 
            target_process_noise, target_sensor_noise, target_control_predicted, 
            obstacle_states, fov, eta, function_J, gamma_collision_threshold)
    for i in range(N):
        for j in range(nu):
            u_bar_perturbed = u_bar.copy()
            u_bar_perturbed[i, j] += grad_delta
            J_m_perturbed = get_J_m(b_r0, b_t0, u_bar_perturbed, robot_process_noise, 
                target_process_noise, target_sensor_noise, target_control_predicted, 
                obstacle_states, fov, eta, function_J, gamma_collision_threshold)
            J_m_grad[i, j] = (J_m_perturbed - J_m_base) / grad_delta
    return J_m_base, J_m_grad

def solve_convex_subproblem(J_grad, J_m_base, u_ref, trust_radius, trust_norm):
    """
    Solve the convex subproblem with trust region constraint.
    J_grad: shape (N, nu)
    u_ref: shape (N, nu)
    trust_radius: scalar
    trust_norm: "inf" or "l2"
    eta: penalty parameter for merit function
    ------------------------------
    Returns:
    u_star: shape (N, nu)
    predicted_decrease: scalar
    """
    N, nu = u_ref.shape
    # minimize: J_grad.flatten() @ (u - u_ref).flatten()
    # subject to: ||u - u_ref||_trust_norm <= trust_radius
    u = cp.Variable((N, nu))
    objective = cp.Minimize(cp.sum(cp.multiply(J_grad, (u - u_ref))))
    if trust_norm == "inf":
        constraints = [cp.norm(u - u_ref, "inf") <= trust_radius]
    elif trust_norm == "l2":
        constraints = [cp.norm(u - u_ref, 2) <= trust_radius]
    else:
        raise ValueError("trust_norm must be 'inf' or 'l2'")
    prob = cp.Problem(objective, constraints)
    try:
        prob.solve(warm_start=True, verbose=False)
    except cp.SolverError:
        pass
    
    if prob.status != cp.OPTIMAL:
        print("Convex subproblem not solved optimally.")
        return None, None
    u_star = u.value
    predicted_decrease = prob.value + J_m_base

    return u_star, predicted_decrease

# Algorithm 2 : with trust region
def SCP(
    # problem data
    b_r0, b_t0,
    robot_process_noise,
    target_process_noise,
    target_sensor_noise,
    target_control_predicted,
    obstacles : ObjectStateList, 
    fov,
    # initial guess
    u_init,                   # shape (N, nu)
    # hyperparams
    eta0=1.0, beta=10.0, eta_max=1e6,   # penalty continuation (outer loop)
    d0=0.5, d_min=1e-4, d_max=5.0,      # trust-region radius
    shrink=0.5, expand=1.5,             # trust-region update factors
    rho_reject=0.65, rho_expand=0.75,    # acceptance thresholds
    tau_conv=1e-3, max_inner=50, max_outer=10,
    trust_norm="inf",                   # "inf" or "l2"
    function_J="1",                     # "1" or "2"
    gamma_collision_threshold=0.1       # collision threshold for inequality constraints
):
    """
    논문 Algorithm 2 스타일:
      - outer loop: penalty eta 증가
      - inner loop: SCP iteration
      - line 12: trust region constraint 추가한 convex subproblem(QP) solve
    """

    u_bar = u_init.copy()   # current nominal control (u^{(n)})
    N, nu = u_bar.shape
    eta = eta0
    d = d0

    obstacle_states = np.zeros((N, len(obstacles.object_state_list), 2)) # shape (N, num_obstacles, 2(px, py))
    for k in range(N):
        for idx, obs in enumerate(obstacles.object_state_list):
            obstacle_states[k, idx, 0] = obs.px + obs.vx * DT * k
            obstacle_states[k, idx, 1] = obs.py + obs.vy * DT * k
    outer_iter = 0
    trust_region_shrunk = False
    while eta < eta_max and outer_iter < max_outer:
        outer_iter += 1

        # inner SCP loop
        inner_iter = 0
        while inner_iter < max_inner:
            try:
                inner_iter += 1

                if trust_region_shrunk:
                    # use cached J_m and grad from previous iteration
                    trust_region_shrunk = False
                else:
                    tic = time.time()
                    # (line 10) compute J_m and its gradient at u_bar
                    J_m_base, J_m_grad = get_J_m_grad(b_r0, b_t0, u_bar, robot_process_noise, 
                        target_process_noise, target_sensor_noise, target_control_predicted, 
                        obstacle_states, fov, eta, function_J, gamma_collision_threshold)
                    toc = time.time()
                # print(f"get J_m and grad time: {toc - tic:.4f} seconds")
                
                # (line 12) solve convex subproblem with TRUST REGION
                #     trust region은 "추정해가 너무 멀리 튀지 않게" 하는 핵심 장치
                u_star, J_tilt_star = solve_convex_subproblem(
                    J_m_grad,
                    J_m_base,
                    u_ref=u_bar,
                    trust_radius=d,
                    trust_norm=trust_norm,
                )

                if u_star is None:
                # QP infeasible or solver fail -> shrink trust region and retry
                    d = max(d * shrink, d_min)
                    if d <= d_min:
                        print("No feasible solution found. Stopping inner loop, trust region radius too small.")
                        break
                    print("No feasible solution found. Shrinking trust region and retrying.")
                    trust_region_shrunk = True
                    continue

                # (5) 실제(nonconvex) merit Jm 평가 (accept/reject 판단)
                #     후보 u_star로 다시 rollout 해서 "진짜 Jm" 계산
                Jm_star = get_J_m(b_r0, b_t0, u_star, robot_process_noise, 
                    target_process_noise, target_sensor_noise, target_control_predicted, 
                    obstacle_states, fov, eta, function_J, gamma_collision_threshold)

                act_dec = J_m_base - Jm_star
                # pred_dec가 0에 가깝거나 음수면 rho가 터지니 방어
                denom = max(J_m_base - J_tilt_star, 1e-12)
                rho = act_dec / denom

                # (6) trust region update + step accept/reject
                if rho < rho_reject or act_dec <= 0.0:
                    # reject: 선형화 모델이 믿을 만하지 않음 -> 반경 줄이고 다시
                    d = max(d * shrink, d_min)
                    if d <= d_min:
                        # 더 줄여도 의미 없으면 inner 종료
                        print("No feasible solution found. Stopping inner loop, trust region radius too small.")
                        break
                    print("Step rejected. Shrinking trust region and retrying.")
                    trust_region_shrunk = True
                    continue
                else:
                    # accept
                    u_prev = u_bar
                    u_bar = u_star

                    # 모델이 잘 맞으면 확장
                    if rho > rho_expand:
                        d = min(d * expand, d_max)

                    # 수렴 체크(컨트롤 변화량)
                    if np.linalg.norm(u_bar - u_prev) < tau_conv:
                        break
            finally:
                # print(f"  Inner iter {inner_iter}: J_m = {Jm_star:.4f}, act_dec = {act_dec:.6f}, pred_dec = {denom:.6f}, rho = {rho:.4f}, trust_radius = {d:.4f}")
                pass

        # (7) penalty 업데이트 (outer continuation)
        eta *= beta
        print(f"Outer iter {outer_iter} completed. Updated penalty eta = {eta:.4f}")

    return u_bar


if __name__ == "__main__":
    test_obstacles = ObjectStateList()
    obs1 = ObjectState()
    obs1.px = 1.0
    obs1.py = 1.0
    obs1.pz = 0.0
    obs1.vx = 0.0
    obs1.vy = -0.0
    obs1.vz = 0.0
    # test_obstacles.object_state_list.append(obs1)

    test_fov_params = FOVParams(fov=np.deg2rad(90), r_min=0.5, r_max=5.0)
    b_r0 = BeliefState(mean=np.array([0.0, 0.0, 0.0, 1.0]), covariance=np.eye(4)*0.001, dim=4)
    b_t0 = BeliefState(mean=np.array([2.0, 0.0]), covariance=np.eye(2)*0.005, dim=2)

    u_init = np.zeros((20, 2))  # 20 time steps, control dim 2
    optimized_u = SCP(
        b_r0, b_t0,
        robot_process_noise=np.eye(4)*0.0001,
        target_process_noise=np.eye(2)*0.001,
        target_sensor_noise=np.eye(2)*0.005,
        target_control_predicted=None,
        obstacles=test_obstacles,
        fov=test_fov_params,
        u_init=u_init
    )
