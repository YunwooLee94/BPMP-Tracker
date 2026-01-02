from __future__ import annotations
from dataclasses import dataclass
from typing import List, Tuple, Optional, Dict
import math
import numpy as np

try:
    import cvxpy as cp
except ImportError:
    cp = None


# =========================
# Basic utils
# =========================
SQRT2 = math.sqrt(2.0)

def norm_cdf(x: float) -> float:
    """Standard normal CDF using erf."""
    return 0.5 * (1.0 + math.erf(x / SQRT2))

def safe_unit(v: np.ndarray, eps: float = 1e-9) -> np.ndarray:
    n = np.linalg.norm(v)
    if n < eps:
        # fallback direction (arbitrary but stable)
        return np.array([1.0, 0.0], dtype=float)
    return v / n

def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))

def prob_halfspace_gaussian(a: np.ndarray, b: float, mu: np.ndarray, Sigma: np.ndarray) -> float:
    """
    P(a^T x <= b) for x ~ N(mu, Sigma).
    Implements paper Eq.(19).  :contentReference[oaicite:2]{index=2}
    """
    a = a.reshape(-1)
    mu = mu.reshape(-1)
    m = float(a @ mu)
    v = float(a @ Sigma @ a)
    if v <= 1e-12:
        # deterministic
        return 1.0 if m <= b else 0.0
    z = (b - m) / math.sqrt(v)
    return norm_cdf(z)


# =========================
# Data structures
# =========================
@dataclass
class CircleObstacle:
    center: np.ndarray  # shape (2,)
    radius: float

@dataclass
class Belief:
    mean: np.ndarray    # shape (d,)
    cov: np.ndarray     # shape (d,d)

@dataclass
class FOVParams:
    r_min: float
    r_max: float
    fov_angle: float  # total angle, e.g., 2*pi/3

@dataclass
class PlannerParams:
    dt: float = 0.5
    N: int = 4

    # robot control bounds
    omega_min: float = -math.pi/3
    omega_max: float =  math.pi/3
    a_min: float = -4.0
    a_max: float =  2.0
    v_min: float = 0.0
    v_max: float = 4.0

    # noise (for covariance prediction; if you already have Q, you can keep tiny)
    Rr: np.ndarray = np.diag([4e-3, 4e-3, 4e-4, 4e-4])  # example
    Rt: np.ndarray = np.diag([1e-2, 1e-2])              # example target pos noise

    # chance constraint threshold for collision
    delta_s: float = 0.05

    # SCP/QP weights
    w_u: float = 1e-2
    w_du: float = 1e-2
    w_vis: float = 1.0

    # SCP iterations
    max_scp_iter: int = 8
    trust_region_init: float = 0.5
    tau_conv: float = 1e-3


# =========================
# Robot / Target dynamics (unicycle + acc) as in paper Eq.(22)  :contentReference[oaicite:3]{index=3}
# z_r = [x, y, theta, v]
# u_r = [omega, a]
# =========================
def robot_step(z: np.ndarray, u: np.ndarray, dt: float) -> np.ndarray:
    x, y, th, v = z
    om, a = u
    x2 = x + v * math.cos(th) * dt
    y2 = y + v * math.sin(th) * dt
    th2 = th + om * dt
    v2 = v + a * dt
    return np.array([x2, y2, th2, v2], dtype=float)

def robot_jacobian_A(z: np.ndarray, u: np.ndarray, dt: float) -> np.ndarray:
    """Jacobian of f wrt z at (z,u)."""
    _, _, th, v = z
    A = np.eye(4)
    A[0, 2] = -v * math.sin(th) * dt
    A[0, 3] =  math.cos(th) * dt
    A[1, 2] =  v * math.cos(th) * dt
    A[1, 3] =  math.sin(th) * dt
    # theta and v are linear in themselves
    return A

def predict_robot_belief(b: Belief, u: np.ndarray, dt: float, Rr: np.ndarray) -> Belief:
    z2 = robot_step(b.mean, u, dt)
    A = robot_jacobian_A(b.mean, u, dt)
    Q2 = A @ b.cov @ A.T + Rr
    return Belief(z2, Q2)

def predict_target_belief_pos(bt: Belief, ut: np.ndarray, dt: float, Rt: np.ndarray) -> Belief:
    """
    Simple single-integrator target: x_{k+1} = x_k + ut*dt
    (paper uses variants; this is enough to wire Algorithm 1/2)
    """
    # TODO gamma를 이용해서 BPOD를 계산하는 부분에서 속도 불확실성을 고려하려면 여기를 바꿔야 함.
    x2 = bt.mean + ut * dt
    P2 = bt.cov + Rt
    return Belief(x2, P2)


# =========================
# Circle-specialized PoSSDF (Algorithm 1 reduced form)
# =========================
def possdf_point_circle_collision_prob(
    mu_p: np.ndarray, Sigma_p: np.ndarray, obs: CircleObstacle
) -> Tuple[float, np.ndarray]:
    """
    Return gamma_ro = P(sd <= 0) for point vs circle obstacle.
    Also return n_hat used for linearization (stored in SDF param set C).
    """
    c, r = obs.center, obs.radius
    n_hat = safe_unit(mu_p - c)
    # sd_L = n_hat^T (p - c) - r <= 0  <=>  n_hat^T p <= n_hat^T c + r
    a = n_hat
    b = float(n_hat @ c + r)
    p = prob_halfspace_gaussian(a, b, mu_p, Sigma_p)
    return p, n_hat

def los_lambda_hat(mu_r: np.ndarray, mu_t: np.ndarray, c: np.ndarray) -> float:
    """
    Compute separative ratio (paper's lambda-hat) using projection of obstacle center
    onto segment from target->robot, clamped to [0,1]. This matches the paper's
    idea around Eq.(20). :contentReference[oaicite:4]{index=4}
    """
    d = mu_r - mu_t
    denom = float(d @ d)
    if denom <= 1e-12:
        return 0.5
    s = float((c - mu_t) @ d) / denom
    return clamp(s, 0.0, 1.0)

def possdf_segment_circle_no_occlusion_prob(
    mu_r: np.ndarray, Sigma_r: np.ndarray,
    mu_t: np.ndarray, Sigma_t: np.ndarray,
    obs: CircleObstacle,
    lambda_hat: float,
    n_hat: Optional[np.ndarray] = None
) -> Tuple[float, np.ndarray]:
    """
    Return gamma_lo = P(sd >= 0) for LOS segment vs circle (no intersection).
    We use paper's adjustment: p1(x)=lambda_hat*x_r+(1-lambda_hat)*x_t (Eq.(20)) :contentReference[oaicite:5]{index=5}
    and fix n_hat at mean.

    sd_L = n_hat^T( p1 - c ) - r
    Want P(sd_L >= 0)  <=>  P( -sd_L <= 0 )
    """
    c, r = obs.center, obs.radius

    p1_hat = lambda_hat * mu_r + (1.0 - lambda_hat) * mu_t
    if n_hat is None:
        n_hat = safe_unit(p1_hat - c)

    # Stack x = [x_r(2), x_t(2)]  ~ N(mu, Sigma) assuming independent blocks
    mu = np.concatenate([mu_r, mu_t], axis=0)
    Sigma = np.zeros((4, 4), dtype=float)
    Sigma[:2, :2] = Sigma_r
    Sigma[2:, 2:] = Sigma_t

    # sd_L = n^T( lambda x_r + (1-lambda) x_t - c ) - r
    # no-occlusion: sd_L >= 0
    # -sd_L <= 0  =>  -n^T(lambda x_r + (1-lambda)x_t) <= -(n^T c + r)
    a = np.zeros(4, dtype=float)
    a[:2] = -lambda_hat * n_hat
    a[2:] = -(1.0 - lambda_hat) * n_hat
    b = -float(n_hat @ c + r)

    p = prob_halfspace_gaussian(a, b, mu, Sigma)
    return p, n_hat


# =========================
# FOV probability gamma_tf (approx via product of halfspaces + radial SDF linearization)
# =========================
def fov_halfspace_normals(theta: float, alpha: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    In robot frame: x_f >= 0,  y_f <= tan(alpha)*x_f,  -y_f <= tan(alpha)*x_f
    Expressed with rel = x_t - x_r in world:
      x_f = h^T rel, y_f = h_perp^T rel
    """
    h = np.array([math.cos(theta), math.sin(theta)], dtype=float)
    h_perp = np.array([-math.sin(theta), math.cos(theta)], dtype=float)
    t = math.tan(alpha)

    # inequalities w^T rel <= 0
    w_front = -h                 # -(h^T rel) <= 0  <=> x_f >= 0
    w_left  = (h_perp - t * h)   # y_f - t x_f <= 0
    w_right = (-h_perp - t * h)  # -y_f - t x_f <= 0
    return w_front, w_left, w_right

def possdf_rel_halfspace_prob(mu_r, Sigma_r, mu_t, Sigma_t, w: np.ndarray) -> float:
    """
    rel = x_t - x_r, want w^T rel <= 0
    => (-w)^T x_r + w^T x_t <= 0
    """
    mu = np.concatenate([mu_r, mu_t], axis=0)
    Sigma = np.zeros((4, 4), dtype=float)
    Sigma[:2, :2] = Sigma_r
    Sigma[2:, 2:] = Sigma_t

    a = np.zeros(4, dtype=float)
    a[:2] = -w
    a[2:] = w
    b = 0.0
    return prob_halfspace_gaussian(a, b, mu, Sigma)

def possdf_radial_prob(mu_r, Sigma_r, mu_t, Sigma_t, r_bound: float, outer: bool) -> float:
    """
    Outer: ||rel|| <= r_max  ->  sd = ||rel|| - r_max <= 0
    Inner: ||rel|| >= r_min  ->  r_min - ||rel|| <= 0
    We linearize at mean rel_hat with n_hat.
    """
    rel_hat = mu_t - mu_r
    n_hat = safe_unit(rel_hat)

    mu = np.concatenate([mu_r, mu_t], axis=0)
    Sigma = np.zeros((4, 4), dtype=float)
    Sigma[:2, :2] = Sigma_r
    Sigma[2:, 2:] = Sigma_t

    # n^T (x_t - x_r) <= r_max   (outer)
    # -n^T (x_t - x_r) <= -r_min (inner)
    a = np.zeros(4, dtype=float)
    if outer:
        a[:2] =  n_hat   # because n^T(x_t-x_r) = -n^T x_r + n^T x_t
        a[2:] = -n_hat
        # Wait: to match a^T [x_r,x_t] <= b, we want -n^T x_r + n^T x_t <= r_max
        a[:2] = -n_hat
        a[2:] =  n_hat
        b = float(r_bound)
    else:
        # -n^T(x_t-x_r) <= -r_min  =>  n^T x_r - n^T x_t <= -r_min
        a[:2] =  n_hat
        a[2:] = -n_hat
        b = float(-r_bound)

    return prob_halfspace_gaussian(a, b, mu, Sigma)

def compute_gamma_tf(
    mu_r: np.ndarray, Sigma_r: np.ndarray, theta_r: float,
    mu_t: np.ndarray, Sigma_t: np.ndarray,
    fov: FOVParams
) -> float:
    """
    Approximate gamma_tf = P(target in FOV).
    We approximate by product of:
      - wedge halfspaces (front/left/right)
      - outer radius, inner radius (linearized)
    (Independence approximation, same spirit as factorization in paper.) :contentReference[oaicite:6]{index=6}
    """
    alpha = 0.5 * fov.fov_angle
    w_front, w_left, w_right = fov_halfspace_normals(theta_r, alpha)

    p_front = possdf_rel_halfspace_prob(mu_r, Sigma_r, mu_t, Sigma_t, w_front)
    p_left  = possdf_rel_halfspace_prob(mu_r, Sigma_r, mu_t, Sigma_t, w_left)
    p_right = possdf_rel_halfspace_prob(mu_r, Sigma_r, mu_t, Sigma_t, w_right)

    p_outer = possdf_radial_prob(mu_r, Sigma_r, mu_t, Sigma_t, fov.r_max, outer=True)
    p_inner = possdf_radial_prob(mu_r, Sigma_r, mu_t, Sigma_t, fov.r_min, outer=False)

    p = p_front * p_left * p_right * p_outer * p_inner
    return float(clamp(p, 0.0, 1.0))


# =========================
# Compute BPOD gamma_k = gamma_tf * Π gamma_lo (paper Eq.(15)) :contentReference[oaicite:7]{index=7}
# and collision risks gamma_ro
# =========================
def compute_bpod_and_collision(
    br: Belief, bt: Belief, obstacles: List[CircleObstacle], fov: FOVParams
) -> Tuple[float, Dict[int, float], Dict[int, float]]:
    """
    Returns:
      gamma (BPOD),
      gamma_lo[i] (no-occlusion prob per obstacle),
      gamma_ro[i] (collision prob per obstacle)
    """
    mu_r = br.mean[:2]
    Sigma_r = br.cov[:2, :2]
    theta_r = float(br.mean[2])

    mu_t = bt.mean[:2]
    Sigma_t = bt.cov[:2, :2]

    gamma_tf = compute_gamma_tf(mu_r, Sigma_r, theta_r, mu_t, Sigma_t, fov)

    gamma_lo = {}
    gamma_ro = {}
    prod_lo = 1.0

    for i, obs in enumerate(obstacles):
        # LOS no-occlusion
        lam = los_lambda_hat(mu_r, mu_t, obs.center)
        p_lo, _ = possdf_segment_circle_no_occlusion_prob(
            mu_r, Sigma_r, mu_t, Sigma_t, obs, lam
        )
        gamma_lo[i] = float(clamp(p_lo, 0.0, 1.0))
        prod_lo *= gamma_lo[i]

        # Robot collision risk
        p_ro, _ = possdf_point_circle_collision_prob(mu_r, Sigma_r, obs)
        gamma_ro[i] = float(clamp(p_ro, 0.0, 1.0))

    gamma = float(clamp(gamma_tf * prod_lo, 0.0, 1.0))
    return gamma, gamma_lo, gamma_ro


# =========================
# SCP / MPC planner (Algorithm 2 skeleton)
# Two-stage idea: fix SDF params from nominal (here: fix normals/lambdas implicitly by freezing them)
# =========================
def zscore_to_margin_weight(sigma: float, eps: float = 1e-6) -> float:
    return 1.0 / max(sigma, eps)

def phi_inv(p: float) -> float:
    """Inverse CDF approximation (for small use). If you have scipy, replace with scipy.stats.norm.ppf."""
    # Rational approximation (Peter J. Acklam). Good enough for planning thresholds.
    # Source: widely used public-domain approximation.
    if p <= 0.0:
        return -1e9
    if p >= 1.0:
        return  1e9
    a = [-3.969683028665376e+01,  2.209460984245205e+02,
         -2.759285104469687e+02,  1.383577518672690e+02,
         -3.066479806614716e+01,  2.506628277459239e+00]
    b = [-5.447609879822406e+01,  1.615858368580409e+02,
         -1.556989798598866e+02,  6.680131188771972e+01,
         -1.328068155288572e+01]
    c = [-7.784894002430293e-03, -3.223964580411365e-01,
         -2.400758277161838e+00, -2.549732539343734e+00,
          4.374664141464968e+00,  2.938163982698783e+00]
    d = [ 7.784695709041462e-03,  3.224671290700398e-01,
          2.445134137142996e+00,  3.754408661907416e+00]
    plow = 0.02425
    phigh = 1 - plow
    if p < plow:
        q = math.sqrt(-2*math.log(p))
        return (((((c[0]*q + c[1])*q + c[2])*q + c[3])*q + c[4])*q + c[5]) / \
               ((((d[0]*q + d[1])*q + d[2])*q + d[3])*q + 1)
    if phigh < p:
        q = math.sqrt(-2*math.log(1-p))
        return -(((((c[0]*q + c[1])*q + c[2])*q + c[3])*q + c[4])*q + c[5]) / \
                 ((((d[0]*q + d[1])*q + d[2])*q + d[3])*q + 1)
    q = p - 0.5
    r = q*q
    return (((((a[0]*r + a[1])*r + a[2])*r + a[3])*r + a[4])*r + a[5])*q / \
           (((((b[0]*r + b[1])*r + b[2])*r + b[3])*r + b[4])*r + 1)

def rollout_nominal(
    br0: Belief, bt0: Belief, u_seq: np.ndarray, ut: np.ndarray, params: PlannerParams
) -> Tuple[List[Belief], List[Belief]]:
    br_list = [br0]
    bt_list = [bt0]
    br = br0
    bt = bt0
    for k in range(params.N):
        br = predict_robot_belief(br, u_seq[k], params.dt, params.Rr)
        bt = predict_target_belief_pos(bt, ut, params.dt, params.Rt)
        br_list.append(br)
        bt_list.append(bt)
    return br_list, bt_list



def flatten_u(u):  # (N,2) -> (2N,)
    return u.reshape(-1)

def unflatten_u(x, N):
    return x.reshape(N, 2)

def eval_J2_and_g(br0, bt0, u_seq, ut, obstacles, fov, params):
    """
    Returns:
      J2 = -sum_{i=1}^N gamma_{k+i}
      g  = [gamma_ro(k+i,obs) - delta_s] stacked over i,obs
    """
    br_list, bt_list = rollout_nominal(br0, bt0, u_seq, ut, params)

    gammas = []
    g_list = []
    for i in range(1, params.N + 1):
        gamma, _, gamma_ro = compute_bpod_and_collision(br_list[i], bt_list[i], obstacles, fov)
        gammas.append(gamma)
        for _, val in gamma_ro.items():
            g_list.append(val - params.delta_s)

    J2 = -float(np.sum(gammas))
    g = np.array(g_list, dtype=float)  # want g <= 0
    return J2, g

def fd_grad(fun, x0, eps=1e-4):
    """finite-diff gradient of scalar fun"""
    g = np.zeros_like(x0)
    f0 = fun(x0)
    for k in range(len(x0)):
        x1 = x0.copy()
        x1[k] += eps
        g[k] = (fun(x1) - f0) / eps
    return g

def fd_jac(fun_vec, x0, eps=1e-4):
    """finite-diff Jacobian of vector fun_vec: R^m"""
    f0 = fun_vec(x0)
    m = len(f0)
    n = len(x0)
    J = np.zeros((m, n), dtype=float)
    for k in range(n):
        x1 = x0.copy()
        x1[k] += eps
        fk = fun_vec(x1)
        J[:, k] = (fk - f0) / eps
    return J

def scp_qp_step_Jm(br0, bt0, u0, ut, obstacles, fov, params, eta, trust_d):
    """
    One inner-loop QP solve:
      minimize linearized Jm = linearized J2 + eta * sum slack for linearized g
    """
    N = params.N
    x0 = flatten_u(u0)

    # build closures
    def J_only(x):
        u = unflatten_u(x, N)
        J2, _ = eval_J2_and_g(br0, bt0, u, ut, obstacles, fov, params)
        return J2

    def g_only(x):
        u = unflatten_u(x, N)
        _, g = eval_J2_and_g(br0, bt0, u, ut, obstacles, fov, params)
        return g

    # values at nominal
    J0 = J_only(x0)
    g0 = g_only(x0)

    # gradients (논문은 여기서 "fixed SDF parameter set"로 빠르게 ∇ 계산) :contentReference[oaicite:4]{index=4}
    grad_J = fd_grad(J_only, x0)
    Jac_g  = fd_jac(g_only, x0)  # shape (m, 2N)

    # QP variables
    x = cp.Variable(x0.shape[0])         # flattened u
    du = x - x0

    m = g0.shape[0]
    s = cp.Variable(m)                   # slack for |g|_+
    cons = []
    cons += [cp.norm_inf(du) <= trust_d]
    cons += [s >= g0 + Jac_g @ du]
    cons += [s >= 0]

    # bounds on u
    u = cp.reshape(x, (N, 2))
    cons += [u[:,0] >= params.omega_min, u[:,0] <= params.omega_max]
    cons += [u[:,1] >= params.a_min,     u[:,1] <= params.a_max]

    # objective: linearized J2 + penalty + small regularization
    obj = (J0 + grad_J @ du) + eta * cp.sum(s) + params.w_u * cp.sum_squares(u)

    prob = cp.Problem(cp.Minimize(obj), cons)
    prob.solve(solver=cp.OSQP, warm_start=True, verbose=False)

    if x.value is None:
        return None  # infeasible
    return unflatten_u(np.array(x.value).astype(float), N)


def plan_mpc_scp(
    br0: Belief,
    bt0: Belief,
    ut: np.ndarray,  # target control estimate (2,)
    obstacles: List[CircleObstacle],
    fov: FOVParams,
    params: PlannerParams,
    u_init: Optional[np.ndarray] = None
) -> np.ndarray:
    """
    Returns optimal control sequence u*(0:N-1,2).
    This is a practical SCP-style implementation:
      - freeze SDF linearization params from nominal rollout
      - solve convex QP in a trust region
      - update and iterate
    """
    if cp is None:
        raise ImportError("cvxpy is required for this SCP/QP implementation. pip install cvxpy")

    N = params.N
    dt = params.dt

    # init controls
    if u_init is None:
        u0 = np.zeros((N, 2), dtype=float)
    else:
        u0 = u_init.copy()

    d = params.trust_region_init

    for it in range(params.max_scp_iter):

        eta = params.w_du / (d + 1e-6)  # trust-region adaptive penalty
        u = scp_qp_step_Jm(br0, bt0, u0, ut, obstacles, fov, params, eta, d)

        if u is None:
            # infeasible: shrink trust region and retry
            d *= 0.5
            continue

        u_new = np.array(u, dtype=float)

        # convergence check
        if np.linalg.norm(u_new - u0) < params.tau_conv:
            u0 = u_new
            break

        u0 = u_new  # accept step (simple version)
        # mild trust region adaptation
        d = min(d * 1.2, 2.0)

    return u0


# =========================
# One-step callable wrapper
# =========================
def planner_step(
    robot_state: np.ndarray,        # (4,) [x,y,theta,v]
    target_state: np.ndarray,       # (2,) [x,y]
    obstacles: List[CircleObstacle],
    ut: np.ndarray,                # (2,) estimated target velocity/control
    fov: FOVParams,
    params: PlannerParams,
    robot_cov: Optional[np.ndarray] = None,
    target_cov: Optional[np.ndarray] = None,
    u_init: Optional[np.ndarray] = None
) -> Tuple[np.ndarray, Dict[str, float]]:
    """
    Returns:
      u0 (2,) = first control [omega,a],
      info dict with predicted BPOD at first horizon step (nominal evaluation).
    """
    if robot_cov is None:
        robot_cov = 1e-6 * np.eye(4)
    if target_cov is None:
        target_cov = 1e-6 * np.eye(2)

    br0 = Belief(robot_state.astype(float), robot_cov.astype(float))
    bt0 = Belief(target_state.astype(float), target_cov.astype(float))

    u_seq = plan_mpc_scp(br0, bt0, ut.astype(float), obstacles, fov, params, u_init=u_init)
    u0 = u_seq[0].copy()

    # evaluate BPOD/collision for next step using Algorithm 1 formulas
    br1 = predict_robot_belief(br0, u0, params.dt, params.Rr)
    bt1 = predict_target_belief_pos(bt0, ut, params.dt, params.Rt)
    gamma, _, gamma_ro = compute_bpod_and_collision(br1, bt1, obstacles, fov)
    info = {
        "bpod_next": float(gamma),
        "max_collision_risk_next": float(max(gamma_ro.values()) if len(gamma_ro) else 0.0),
    }
    return u0, info


if __name__ == "__main__":
    robot_state = np.array([32.0, 7.0, 0.75*math.pi, 0.0])  # [x,y,theta,v]
    target_state = np.array([28.0, 9.0])                    # [x,y]
    ut = np.array([0.5, 0.0])                               # target velocity estimate

    obstacles = [
        CircleObstacle(center=np.array([30.0, 10.0]), radius=1.5),
        CircleObstacle(center=np.array([40.0,  8.0]), radius=2.0),
    ]

    fov = FOVParams(r_min=2.0, r_max=10.0, fov_angle=2*math.pi/3)
    params = PlannerParams(N=4, dt=0.5)

    u0, info = planner_step(robot_state, target_state, obstacles, ut, fov, params)
    print("u0=", u0, "info=", info)
