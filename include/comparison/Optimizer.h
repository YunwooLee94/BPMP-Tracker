// Optimizer.h
#ifndef BPMP_TRACKER_OPTIMIZER_HPP
#define BPMP_TRACKER_OPTIMIZER_HPP

#include <cmath>
#include <array>
#include <string>
#include <algorithm>
#include <limits>
#include <comparison/Problem.h>
#include <Eigen/Dense>

namespace bpmp {

template<typename T, const int size>
using Collection = std::array<T, size>;

struct OptimizationParam
{
    // discretization
    double time_step = 0.5;

    // robot/target noise (python과 동일한 형태)
    Eigen::Matrix<double, Nx_r, Nx_r> robot_process_noise = Eigen::Matrix<double, Nx_r, Nx_r>::Identity() * 0.001;
    Eigen::Matrix<double, Nx_t, Nx_t> target_process_noise = Eigen::Matrix<double, Nx_t, Nx_t>::Identity() * 0.001;
    Eigen::Matrix<double, 2, 2>       target_sensor_noise  = Eigen::Matrix<double, 2, 2>::Identity() * 0.001;

    // initial covariances (실제로는 EKF/필터에서 받아오는 게 더 맞지만, 최소 구현은 param으로 둠)
    Eigen::Matrix<double, Nx_r, Nx_r> robot_cov0 = Eigen::Matrix<double, Nx_r, Nx_r>::Identity() * 0.001;
    Eigen::Matrix<double, Nx_t, Nx_t> target_cov0 = Eigen::Matrix<double, Nx_t, Nx_t>::Identity() * 0.005;

    // robot kinematic limits
    double v0    = 0.0;   // 초기 v (ROS에서 안 주면 param으로 둠)
    double v_max = 1.0;
    double w_max = 1.0;   // |omega| <= w_max
    // double a_min = -1.0;
    // double a_max =  1.0;

    // FOV params (annular sector)
    double fov_angle = M_PI/3.0; // rad
    double r_min = 1.0;
    double r_max = 10.0;

    // obstacle
    double obs_radius = 0.5;

    // merit parameters
    std::string function_J = "2";          // "1": entropy sum, "2": -sum(gamma_k)
    double gamma_collision_threshold = 0.1;

    // SCP outer loop (eta)
    double eta0 = 1.0;
    double eta_max = 1e3;
    double beta = 10.0;
    double tau_p = 1e-2;

    // trust region
    double d0 = 0.25;
    double d_min = 1e-3;
    double d_max = 2.0;
    double shrink = 0.5;     // <1
    double expand = 1.5;     // >1
    double rho_reject = 0.25;
    double rho_expand = 0.75;

    // termination
    int max_inner = 40;
    int max_outer = 5;
    double tau_conv = 1e-3;
    double tau_f    = 1e-4;

    // gradient FD
    double grad_delta = 1e-6;
    bool verbose = false;
};

// ---------- small helpers ----------
inline double clip(double x, double lo, double hi) {
    return std::max(lo, std::min(hi, x));
}

inline double phi_from_sd(double sd_value, double var_proj, bool sense_leq) {
    // sd_lin ~ N(sd_value, var_proj)
    if (var_proj <= 1e-12) {
        if (sense_leq) return (sd_value <= 0.0) ? 1.0 : 0.0;
        else           return (sd_value >= 0.0) ? 1.0 : 0.0;
    }
    const double z = sd_value / (std::sqrt(var_proj) * std::sqrt(2.0));
    const double p_leq = 0.5 * (1.0 - std::erf(z)); // Pr(sd<=0)
    return sense_leq ? p_leq : (1.0 - p_leq);
}

// ---------- SDFs (python fast 버전 그대로 이식) ----------
inline double sd_robot_obstacle(const Eigen::Vector2d& p_r,
                                const Eigen::Vector2d& c,
                                double r) {
    return (p_r - c).norm() - r;
}

inline Eigen::Vector2d sd_robot_obstacle_grad(const Eigen::Vector2d& p_r,
                                              const Eigen::Vector2d& c) {
    Eigen::Vector2d d = p_r - c;
    const double n = std::max(1e-12, d.norm());
    return d / n;
}

inline double sd_LOS(const Eigen::Vector2d& a, // robot
                     const Eigen::Vector2d& b, // target
                     const Eigen::Vector2d& c, // obstacle center
                     double r) {
    Eigen::Vector2d v = b - a;
    const double vv = v.squaredNorm();
    if (vv < 1e-12) {
        return (c - a).norm() - r;
    }
    double t = (c - a).dot(v) / vv;
    t = clip(t, 0.0, 1.0);
    Eigen::Vector2d q = a + t * v;
    return (c - q).norm() - r;
}

inline void sd_LOS_grad(const Eigen::Vector2d& a,
                        const Eigen::Vector2d& b,
                        const Eigen::Vector2d& c,
                        Eigen::Vector2d& grad_b, // dsd/db
                        Eigen::Vector2d& grad_a) // dsd/da
{
    Eigen::Vector2d v = b - a;
    const double vv = v.squaredNorm();
    if (vv < 1e-12) {
        Eigen::Vector2d d = c - a;
        const double n = std::max(1e-12, d.norm());
        Eigen::Vector2d u = d / n;
        grad_a = -u;
        grad_b.setZero();
        return;
    }

    const double t_raw = (c - a).dot(v) / vv;
    const double t = clip(t_raw, 0.0, 1.0);
    Eigen::Vector2d q = a + t * v;
    Eigen::Vector2d d = c - q;
    const double n = std::max(1e-12, d.norm());
    Eigen::Vector2d u = d / n;

    if (t_raw <= 0.0) {
        grad_a = -u;
        grad_b.setZero();
    } else if (t_raw >= 1.0) {
        grad_a.setZero();
        grad_b = -u;
    } else {
        grad_a = -(1.0 - t) * u;
        grad_b = -t * u;
    }
}

// annular sector FOV SDF (local frame min distance) + gradient
inline double sd_annular_sector_fov(const Eigen::Vector2d& z_world,
                                    const Eigen::Vector2d& origin_world,
                                    double heading,
                                    double r_min,
                                    double r_max,
                                    double fov_angle)
{
    const double half = 0.5 * fov_angle;

    // translate
    const Eigen::Vector2d p = z_world - origin_world;

    // rotate by -heading
    const double c = std::cos(heading), s = std::sin(heading);
    const double x =  c * p.x() + s * p.y();
    const double y = -s * p.x() + c * p.y();

    const double r = std::sqrt(x*x + y*y);
    if (r < 1e-12) {
        // at origin, return distance to inner arc
        return r_min;
    }
    const double phi = std::atan2(y, x);

    auto dist_point_segment = [](double px, double py,
                                 double ax, double ay,
                                 double bx, double by) {
        const double vx = bx - ax, vy = by - ay;
        const double wx = px - ax, wy = py - ay;
        const double vv = vx*vx + vy*vy;
        double t = (vv > 1e-12) ? (wx*vx + wy*vy)/vv : 0.0;
        t = clip(t, 0.0, 1.0);
        const double cx = ax + t*vx;
        const double cy = ay + t*vy;
        const double dx = px - cx;
        const double dy = py - cy;
        return std::sqrt(dx*dx + dy*dy);
    };

    const double ang = clip(phi, -half, half);

    const double qox = r_max * std::cos(ang);
    const double qoy = r_max * std::sin(ang);
    const double d_outer = std::sqrt((x-qox)*(x-qox) + (y-qoy)*(y-qoy));

    const double qix = r_min * std::cos(ang);
    const double qiy = r_min * std::sin(ang);
    const double d_inner = std::sqrt((x-qix)*(x-qix) + (y-qiy)*(y-qiy));

    const double ca = std::cos(half), sa = std::sin(half);

    // +edge segment
    const double axp = r_min * ca, ayp = r_min * sa;
    const double bxp = r_max * ca, byp = r_max * sa;
    const double d_ep = dist_point_segment(x,y, axp,ayp, bxp,byp);

    // -edge segment
    const double axm = r_min * ca, aym = -r_min * sa;
    const double bxm = r_max * ca, bym = -r_max * sa;
    const double d_em = dist_point_segment(x,y, axm,aym, bxm,bym);

    const double d = std::min(std::min(d_outer, d_inner), std::min(d_ep, d_em));

    const bool inside = (r >= r_min) && (r <= r_max) && (std::abs(phi) <= half);
    return inside ? -d : d;
}

inline void sd_annular_sector_fov_grad(const Eigen::Vector2d& x_t,
                                       const Eigen::Vector2d& x_r,
                                       double heading,
                                       double r_min,
                                       double r_max,
                                       double fov_angle,
                                       double& dsd_dx_t,
                                       double& dsd_dy_t,
                                       double& dsd_dx_r,
                                       double& dsd_dy_r,
                                       double& dsd_dheading)
{
    const double half = 0.5 * fov_angle;

    const double dx = x_t.x() - x_r.x();
    const double dy = x_t.y() - x_r.y();

    const double c = std::cos(heading), s = std::sin(heading);
    const double x =  c * dx + s * dy;
    const double y = -s * dx + c * dy;

    const double r = std::sqrt(x*x + y*y);
    if (r < 1e-12) {
        // at origin, gradient is undefined
        dsd_dx_t = -c;
        dsd_dy_t = -s;
        dsd_dx_r = -dsd_dx_t;
        dsd_dy_r = -dsd_dy_t;
        dsd_dheading = 0.0;
        return;
    }
    const double phi = std::atan2(y, x);

    const bool inside = (r >= r_min) && (r <= r_max) && (std::abs(phi) <= half);

    auto closest_point_on_segment = [](double px, double py,
                                       double ax, double ay,
                                       double bx, double by) {
        const double vx = bx - ax, vy = by - ay;
        const double wx = px - ax, wy = py - ay;
        const double vv = vx*vx + vy*vy;
        double t = (vv > 1e-12) ? (wx*vx + wy*vy)/vv : 0.0;
        t = clip(t, 0.0, 1.0);
        return std::pair<double,double>(ax + t*vx, ay + t*vy);
    };

    const double ang = clip(phi, -half, half);

    const double qox = r_max * std::cos(ang);
    const double qoy = r_max * std::sin(ang);

    const double qix = r_min * std::cos(ang);
    const double qiy = r_min * std::sin(ang);

    const double ca = std::cos(half), sa = std::sin(half);

    // +edge endpoints
    const double axp = r_min * ca, ayp = r_min * sa;
    const double bxp = r_max * ca, byp = r_max * sa;
    auto qp = closest_point_on_segment(x,y, axp,ayp, bxp,byp);

    // -edge endpoints
    const double axm = r_min * ca, aym = -r_min * sa;
    const double bxm = r_max * ca, bym = -r_max * sa;
    auto qm = closest_point_on_segment(x,y, axm,aym, bxm,bym);

    auto dist_and_unitgrad = [](double px, double py, double qx, double qy) {
        const double vx = px - qx;
        const double vy = py - qy;
        const double d = std::sqrt(vx*vx + vy*vy);
        if (d < 1e-12) return std::pair<double, Eigen::Vector2d>(0.0, Eigen::Vector2d::Zero());
        return std::pair<double, Eigen::Vector2d>(d, Eigen::Vector2d(vx/d, vy/d));
    };

    auto [d_outer, g_outer] = dist_and_unitgrad(x,y, qox,qoy);
    auto [d_inner, g_inner] = dist_and_unitgrad(x,y, qix,qiy);
    auto [d_ep,    g_ep]    = dist_and_unitgrad(x,y, qp.first, qp.second);
    auto [d_em,    g_em]    = dist_and_unitgrad(x,y, qm.first, qm.second);

    double ds[4] = {d_outer, d_inner, d_ep, d_em};
    Eigen::Vector2d gs[4] = {g_outer, g_inner, g_ep, g_em};

    int idx = 0;
    for(int i=1;i<4;i++) if(ds[i] < ds[idx]) idx = i;

    Eigen::Vector2d grad_d_local = gs[idx];
    const double sgn = inside ? -1.0 : 1.0;
    Eigen::Vector2d grad_sd_local = sgn * grad_d_local;

    // world grad wrt target pos
    const double gx_t = c * grad_sd_local.x() - s * grad_sd_local.y();
    const double gy_t = s * grad_sd_local.x() + c * grad_sd_local.y();

    dsd_dx_t = gx_t;
    dsd_dy_t = gy_t;
    dsd_dx_r = -gx_t;
    dsd_dy_r = -gy_t;

    // d(local)/dtheta: dx/dtheta = y, dy/dtheta = -x  (local)
    dsd_dheading = grad_sd_local.x() * y - grad_sd_local.y() * x;
}

// ---------- probabilities ----------
inline double gamma_tf_FOV(const Eigen::Vector2d& mu_t,
                           const Eigen::Matrix2d& Sigma_t,
                           const Eigen::Matrix<double, Nx_r, 1>& mu_r,
                           const Eigen::Matrix<double, Nx_r, Nx_r>& Sigma_r,
                           const OptimizationParam& p)
{
    const Eigen::Vector2d origin = mu_r.segment<2>(0);
    const double heading = mu_r(2);

    const double sd = sd_annular_sector_fov(mu_t, origin, heading, p.r_min, p.r_max, p.fov_angle);

    double dsd_dx_t, dsd_dy_t, dsd_dx_r, dsd_dy_r, dsd_dheading;
    sd_annular_sector_fov_grad(mu_t, origin, heading, p.r_min, p.r_max, p.fov_angle,
                               dsd_dx_t, dsd_dy_t, dsd_dx_r, dsd_dy_r, dsd_dheading);

    // robot proj var: gradient in robot state [x,y,theta,v] = [dsd/dx_r, dsd/dy_r, dsd/dtheta, 0]
    Eigen::Matrix<double, Nx_r, 1> g_r; g_r.setZero();
    g_r(0) = dsd_dx_r; g_r(1) = dsd_dy_r; g_r(2) = dsd_dheading;

    const double var_r = g_r.transpose() * Sigma_r * g_r;

    // target proj var
    Eigen::Vector2d g_t(dsd_dx_t, dsd_dy_t);
    const double var_t = g_t.transpose() * Sigma_t * g_t;

    return phi_from_sd(sd, var_r + var_t, /*sense_leq=*/true); // inside event: sd<=0
}

inline double gamma_k_bpod(const Eigen::Vector2d& mu_t,
                           const Eigen::Matrix2d& Sigma_t,
                           const Eigen::Matrix<double, Nx_r, 1>& mu_r,
                           const Eigen::Matrix<double, Nx_r, Nx_r>& Sigma_r,
                           const std::vector<Eigen::Vector2d>& obs_centers,
                           const OptimizationParam& p)
{
    const double gamma_tf = gamma_tf_FOV(mu_t, Sigma_t, mu_r, Sigma_r, p);

    // LOS factors
    const Eigen::Vector2d a = mu_r.segment<2>(0);
    const Eigen::Vector2d b = mu_t;

    const Eigen::Matrix2d Sigma_r_xy = Sigma_r.block<2,2>(0,0);
    const Eigen::Matrix2d Sigma_t_xy = Sigma_t;

    double log_prod = 0.0;
    for(const auto& c : obs_centers){
        const double sd = sd_LOS(a,b,c,p.obs_radius);

        Eigen::Vector2d grad_b, grad_a;
        sd_LOS_grad(a,b,c, grad_b, grad_a);

        const double var_r = grad_a.transpose() * Sigma_r_xy * grad_a;
        const double var_t = grad_b.transpose() * Sigma_t_xy * grad_b;

        const double gamma_lo = phi_from_sd(sd, var_r + var_t, /*sense_leq=*/false); // clear LOS: sd>=0
        log_prod += std::log(std::max(1e-12, std::min(1.0, gamma_lo)));
    }
    return gamma_tf * std::exp(log_prod);
}

inline double gamma_ro_collision_max(const Eigen::Matrix<double, Nx_r, 1>& mu_r,
                                    const Eigen::Matrix<double, Nx_r, Nx_r>& Sigma_r,
                                    const std::vector<Eigen::Vector2d>& obs_centers,
                                    const OptimizationParam& p)
{
    const Eigen::Vector2d pr = mu_r.segment<2>(0);
    const Eigen::Matrix2d Sigma_r_xy = Sigma_r.block<2,2>(0,0);

    double worst = 0.0;
    for(const auto& c : obs_centers){
        const double sd = sd_robot_obstacle(pr, c, p.obs_radius);
        const Eigen::Vector2d g = sd_robot_obstacle_grad(pr, c);
        const double var = g.transpose() * Sigma_r_xy * g;
        const double gamma = phi_from_sd(sd, var, /*sense_leq=*/true); // collision event: sd<=0
        worst = std::max(worst, gamma);
    }
    return worst;
}

// ---------- belief propagation (python 그대로) ----------
struct BeliefR {
    Eigen::Matrix<double, Nx_r, 1> mean;
    Eigen::Matrix<double, Nx_r, Nx_r> cov;
};
struct BeliefT {
    Eigen::Vector2d mean;
    Eigen::Matrix2d cov;
};

inline BeliefR robot_step(const BeliefR& b_r,
                          const Eigen::Vector2d& u, // [w,v]
                          const OptimizationParam& p)
{
    const double x = b_r.mean(0);
    const double y = b_r.mean(1);
    const double th = b_r.mean(2);
    // const double v = b_r.mean(3);

    const double w = u(0);
    const double v = u(1);

    BeliefR out;
    out.mean(0) = x + v * std::cos(th) * p.time_step;
    out.mean(1) = y + v * std::sin(th) * p.time_step;
    out.mean(2) = th + w * p.time_step;
    // out.mean(3) = clip(v + a * p.time_step, -p.v_max, p.v_max);

    Eigen::Matrix<double, Nx_r, Nx_r> A = Eigen::Matrix<double, Nx_r, Nx_r>::Identity();
    A(0,2) = -v * std::sin(th) * p.time_step;
    // A(0,3) =  std::cos(th) * p.time_step;
    A(1,2) =  v * std::cos(th) * p.time_step;
    // A(1,3) =  std::sin(th) * p.time_step;

    out.cov = A * b_r.cov * A.transpose() + p.robot_process_noise;
    return out;
}

inline double entropy_gaussian(const Eigen::Matrix2d& Sigma)
{
    // 0.5*logdet + 0.5*dim*(1+log(2*pi))
    Eigen::LLT<Eigen::Matrix2d> llt(Sigma);
    const double logdet = 2.0 * std::log(std::max(1e-12, llt.matrixL().determinant()));
    const int dim = 2;
    return 0.5 * logdet + 0.5 * dim * (1.0 + std::log(2.0*M_PI));
}

inline BeliefT target_step(const BeliefT& b_t,
                           const Eigen::Vector2d& u_t, // [vx,vy]
                           const BeliefR& b_r_next,
                           const std::vector<Eigen::Vector2d>& obs_centers,
                           double& gamma_k_out,
                           const OptimizationParam& p)
{
    BeliefT out;
    out.mean = b_t.mean + u_t * p.time_step;

    // prediction cov (A=I)
    Eigen::Matrix2d cov_pred = b_t.cov + p.target_process_noise;

    // gamma_k (BPOD)
    Eigen::Matrix2d Sigma_t = cov_pred;
    gamma_k_out = gamma_k_bpod(out.mean, Sigma_t, b_r_next.mean, b_r_next.cov, obs_centers, p);

    // c_tild, k_tild, covariance update (python 그대로)
    const Eigen::Vector2d pr = b_r_next.mean.segment<2>(0);
    const Eigen::Vector2d diff = out.mean - pr;
    const double nr = std::max(1e-12, diff.norm());
    if (diff.norm() < 1e-12) {
        out.cov = cov_pred;
        return out;
    }

    Eigen::Matrix2d c_tild;
    c_tild(0,0) = diff.x()/nr;
    c_tild(0,1) = diff.y()/nr;
    c_tild(1,0) = -diff.y()/(nr*nr);
    c_tild(1,1) =  diff.x()/(nr*nr);

    Eigen::Matrix2d S = c_tild * cov_pred * c_tild.transpose() + p.target_sensor_noise;
    Eigen::Matrix2d K = cov_pred * c_tild.transpose() * S.inverse();

    out.cov = cov_pred - gamma_k_out * (K * c_tild * cov_pred);
    return out;
}

// ---------- J_m and gradient ----------
// template<const int N_, const int Nu_>
inline void get_J_m_and_constraints(const BeliefR& b_r0,
                                    const BeliefT& b_t0,
                                    const Eigen::Matrix<double, N, Nu>& u_bar,
                                    const Collection<std::vector<Eigen::Vector2d>, N>& obs_centers,
                                    double eta,
                                    double& J_m,
                                    double& constraint_sum,
                                    const OptimizationParam& p,
                                    const Eigen::Vector2d& u_t)
{
    // rollout robot
    std::array<BeliefR, N+1> br;
    br[0] = b_r0;
    for(int k=0;k<N;k++){
        br[k+1] = robot_step(br[k], u_bar.row(k).transpose(), p);
    }

    // rollout target
    std::array<BeliefT, N+1> bt;
    bt[0] = b_t0;

    std::array<double, N> gamma_list{};
    std::array<double, N> gamma_ro_list{};

    for(int k=0;k<N;k++){
        // 최소 구현: target control prediction을 0으로 둠
        // Eigen::Vector2d u_t = u_t[k];

        double gamma_k = 1.0;
        bt[k+1] = target_step(bt[k], u_t, br[k+1], obs_centers[k], gamma_k, p);
        gamma_list[k] = gamma_k;

        if(obs_centers[k].empty()){
            gamma_ro_list[k] = 0.0;
        }else{
            gamma_ro_list[k] = gamma_ro_collision_max(br[k+1].mean, br[k+1].cov, obs_centers[k], p);
        }
    }

    // J
    double J = 0.0;
    if(p.function_J == "1"){
        for(int k=1;k<=N;k++){
            J += entropy_gaussian(bt[k].cov);
        }
    }else{ // "2"
        for(int k=0;k<N;k++){
            J += -gamma_list[k];
        }
    }

    // constraint: sum(max(0, gamma_ro - thr))
    constraint_sum = 0.0;
    for(int k=0;k<N;k++){
        constraint_sum += std::max(0.0, gamma_ro_list[k] - p.gamma_collision_threshold);
    }

    J_m = J + eta * constraint_sum;
}
// template<const int N_, const int Nu_>
inline void get_J_m_grad_FD(const BeliefR& b_r0,
                            const BeliefT& b_t0,
                            const Eigen::Matrix<double, N, Nu>& u_bar,
                            const Collection<std::vector<Eigen::Vector2d>, N>& obs_centers,
                            double eta,
                            Eigen::Matrix<double, N, Nu>& grad,
                            double& J_m_base,
                            double& constraint_base,
                            const OptimizationParam& p,
                            const Eigen::Vector2d& u_t)
{
    get_J_m_and_constraints(b_r0, b_t0, u_bar, obs_centers, eta, J_m_base, constraint_base, p, u_t);

    grad.setZero();
    for(int i=0;i<N;i++){
        for(int j=0;j<Nu;j++){
            Eigen::Matrix<double, N, Nu> u_pert = u_bar;
            u_pert(i,j) += p.grad_delta;

            double Jm_pert_forward, Jm_pert_backward, c_pert;
            get_J_m_and_constraints(b_r0, b_t0, u_pert, obs_centers, eta, Jm_pert_forward, c_pert, p, u_t);
            u_pert(i,j) -= 2 * p.grad_delta;
            get_J_m_and_constraints(b_r0, b_t0, u_pert, obs_centers, eta, Jm_pert_backward, c_pert, p, u_t);
            grad(i,j) = (Jm_pert_forward - Jm_pert_backward) / (2 * p.grad_delta);
        }
    }
}

// ---------- convex subproblem (L∞ trust region, closed-form) ----------
inline void solve_trust_region_inf(const Eigen::Matrix<double, N, Nu>& u_ref,
                                  const Eigen::Matrix<double, N, Nu>& J_grad,
                                  double d,
                                  Eigen::Matrix<double, N, Nu>& u_star,
                                  double& J_tilt_star,
                                  double J_m_base,
                                  const OptimizationParam& p)
{
    // normalize to [0,1] for both channels (python 의도대로)
    auto normalize = [&](double w, double v)->Eigen::Vector2d{
        const double w_n = (w + p.w_max) / (2.0 * p.w_max);          // [-wmax,wmax] -> [0,1]
        const double v_n = (v + p.v_max) / (2.0 * p.v_max);      // [-vmax,vmax] -> [0,1]
        return Eigen::Vector2d(w_n, v_n);
    };
    auto denormalize = [&](double w_n, double v_n)->Eigen::Vector2d{
        const double w = (2.0*p.w_max)*w_n - p.w_max;
        const double v = (2.0*p.v_max)*v_n - p.v_max;
        return Eigen::Vector2d(w, v);
    };

    // scale gradient for normalized variables
    Eigen::Matrix<double, N, Nu> grad_n = J_grad;
    grad_n.col(0) *= (2.0 * p.w_max);
    grad_n.col(1) *= (2.0 * p.v_max);

    // build u_ref_norm
    Eigen::Matrix<double, N, Nu> uref_n;
    for(int k=0;k<N;k++){
        Eigen::Vector2d un = normalize(u_ref(k,0), u_ref(k,1));
        uref_n(k,0) = un(0);
        uref_n(k,1) = un(1);
    }

    // trust region in normalized space
    d = clip(d, p.d_min, p.d_max);

    Eigen::Matrix<double, N, Nu> ustar_n = uref_n;

    for(int k=0;k<N;k++){
        for(int j=0;j<Nu;j++){
            const double sgn = (grad_n(k,j) > 0.0) ? 1.0 : (grad_n(k,j) < 0.0 ? -1.0 : 0.0);
            double cand = uref_n(k,j) - d * sgn;

            // enforce trust region box
            cand = clip(cand, uref_n(k,j) - d, uref_n(k,j) + d);

            // enforce [0,1]
            cand = clip(cand, 0.0, 1.0);

            ustar_n(k,j) = cand;
        }
    }

    // denormalize back
    for(int k=0;k<N;k++){
        Eigen::Vector2d uu = denormalize(ustar_n(k,0), ustar_n(k,1));
        // enforce true bounds (safety)
        uu(0) = clip(uu(0), -p.w_max, p.w_max);
        uu(1) = clip(uu(1), -p.v_max, p.v_max);

        u_star.row(k) = uu.transpose();
    }

    // predicted merit at u_star under linear model
    const double lin_obj = (J_grad.array() * (u_star - u_ref).array()).sum();
    J_tilt_star = J_m_base + lin_obj;
}

// ---------- Optimizer class ----------
// template<const int Nx_, const int Nu_, const int N_>
template<const int N_, const int Nu_>
class Optimizer {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    // using VectorX = Eigen::Matrix<double, Nx_, 1>;
    using VectorU = Eigen::Matrix<double, Nu_, 1>;

    Optimizer(const ProblemDescription<N_,Nu_>& problem_base,
            //   const VectorX& x_init,
              const Collection<VectorU, N_>& u_init,
              const double& /*time_step_unused*/,
              const OptimizationParam& param)
        : /*x_init_(x_init),*/ u_init_(u_init), param_(param)
    {
        problem_ = dynamic_cast<const bpmp::Problem*>(&problem_base);
        if(!problem_) {
            throw std::runtime_error("Optimizer: problem_base is not bpmp::Problem.");
        }
    }

    void Solve() {
        // ---- build obstacle centers (static prediction 최소 구현) ----
        float tic = static_cast<float>(std::clock());
        Collection<std::vector<Eigen::Vector2d>, N_> obs_centers;
        
        for (int k = 0; k < N_; k++) {
            obs_centers[k].clear();
            obs_centers[k].reserve(problem_->obstacle_state_list().size());
            for(const auto& o : problem_->obstacle_state_list()){
                obs_centers[k].emplace_back(o.px + o.vx * (k+1) * param_.time_step, 
                                            o.py + o.vy * (k+1) * param_.time_step);
        }
        }
        

        // ---- initial beliefs from ROS states ----
        BeliefR b_r0;
        b_r0.mean.setZero();
        b_r0.mean(0) = problem_->tracker_state().px;
        b_r0.mean(1) = problem_->tracker_state().py;
        b_r0.mean(2) = problem_->tracker_state().theta;
        // b_r0.mean(3) = param_.v0;
        b_r0.cov = param_.robot_cov0;

        BeliefT b_t0;
        b_t0.mean = Eigen::Vector2d(problem_->target_state().px, problem_->target_state().py);
        b_t0.cov  = param_.target_cov0;

        Eigen::Vector2d u_t;
        u_t(0) = problem_->target_state().vx;
        u_t(1) = problem_->target_state().vy;


        // ---- convert u_init_ to matrix ----
        Eigen::Matrix<double, N_, Nu_> u_bar;
        for(int k=0;k<N_;k++){
            u_bar.row(k) = u_init_[k].transpose();
        }

        double eta = param_.eta0;
        double d   = param_.d0;
        Eigen::Matrix<double, N_, Nu_> u_prev;

        // For Loggning
        int iter_outer = 0;
        int iter_inner = 0;


        // outer loop (eta)
        // for(int outer=0; outer<param_.max_outer; outer++){
        while(true){
            if (eta > param_.eta_max - 1e-6) {
                if (param_.verbose) {
                    std::cout << "[bpmp::Optimizer] Penalty eta reached maximum value. Stopping outer loop." << std::endl;
                }
                break;
            }
            else if (iter_outer >= param_.max_outer) {
                if (param_.verbose) {
                    std::cout << "[bpmp::Optimizer] Maximum outer iterations reached. Stopping outer loop." << std::endl;
                }
                break;
            }
            iter_outer = iter_outer + 1;
            iter_inner = 0;
            d = param_.d0;
            // inner SCP loop
            for(int inner=0; inner<param_.max_inner; inner++){
                iter_inner=inner; 
                // Jm + grad
                Eigen::Matrix<double, N_, Nu_> J_grad;
                double Jm_base = 0.0, c_base = 0.0;
                get_J_m_grad_FD(b_r0, b_t0, u_bar, obs_centers, eta, J_grad, Jm_base, c_base, param_, u_t);

                // solve convex subproblem
                Eigen::Matrix<double, N_, Nu_> u_star;
                double J_tilt_star = 0.0;
                solve_trust_region_inf(u_bar, J_grad, d, u_star, J_tilt_star, Jm_base, param_);

                // evaluate true merit at u_star
                double Jm_star=0.0, c_star=0.0;
                get_J_m_and_constraints(b_r0, b_t0, u_star, obs_centers, eta, Jm_star, c_star, param_, u_t);

                // improvement ratio
                double predicted_dec = Jm_base - J_tilt_star;
                double actual_dec    = Jm_base - Jm_star;

                if (actual_dec < 0.0) {
                    d = d * param_.shrink;
                    if (param_.verbose) {
                        std::cout << "[bpmp::Optimizer] Negative actual decrease. iter : " << iter_inner
                                    << " d: " << d << " actual_dec: " << actual_dec << std::endl;
                    }
                    if (d <= param_.d_min) {
                        if (param_.verbose) {
                            std::cout << "[bpmp::Optimizer] Trust region too small after negative actual decrease. "
                                        << "iter : " << iter_inner << std::endl;
                        }
                        break;
                    }
                    continue; // reject immediately
                }

                
                // double rho = -std::numeric_limits<double>::infinity();
                // if (predicted_dec > 1e-12) rho = actual_dec / predicted_dec;
                if (predicted_dec <= 0.0) {
                    predicted_dec = 1e-12; // avoid div0 or negative predicted decrease
                    if (param_.verbose) {
                        std::cout << "[bpmp::Optimizer] WARNING: Predicted decrease negative. This should not happen. "
                        << "Setting denom to small positive value. iter_inner: " << iter_inner << std::endl;
                    }
                }
                
                double rho = std::max(actual_dec / predicted_dec, 0.0);

                if ((u_star - u_bar).norm() < param_.tau_conv) {
                    // u_prev = u_bar;
                    u_bar = u_star;
                    if (param_.verbose) {
                        std::cout << "[bpmp::Optimizer] Converged by step norm. by (u_star-u_bar) norm. iter : " << iter_inner << std::endl;
                    }
                    break; // converged
                }
                else if (std::abs((J_grad.array() * (u_star - u_bar).array()).sum()) < param_.tau_f) {
                    // u_prev = u_bar;
                    u_bar = u_star;
                    if (param_.verbose) {
                        std::cout << "[bpmp::Optimizer] Converged by stationarity. by (J_grad * (u_star-u_bar)) sum. iter : " << iter_inner << std::endl;
                    }
                    break; // converged
                }

                if (rho < param_.rho_reject) {
                    // reject: shrink trust region
                    d = std::max(param_.d_min, d * param_.shrink);
                    if (param_.verbose) {
                        std::cout << "[bpmp::Optimizer] rho is too small. By rho_reject, Shrinking trust region and retrying."
                        <<" iter: : " << iter_inner<< " d: " << d << " rho: " << rho << std::endl;
                    }
                    if (d <= param_.d_min + 1e-6) {
                        if (param_.verbose) {
                            std::cout << "[bpmp::Optimizer] Trust region radius too small. Stopping inner loop. "
                                        << "iter : " << iter_inner << std::endl;
                        }
                        break;
                    }
                    continue;
                }
                else if (rho >= param_.rho_expand) {
                    // accept
                    // u_prev = u_bar;
                    u_bar = u_star;
                    d = std::min(param_.d_max, d * param_.expand);
                    if (param_.verbose) {
                        std::cout << "[bpmp::Optimizer] Step accepted and trust region expanded. "
                        <<" iter: : " << iter_inner<< " d: " << d << " rho: " << rho << std::endl;
                    }
                }
                else{
                    // accept
                    u_prev = u_bar;
                    u_bar = u_star;
                    if (param_.verbose) {
                        std::cout << "[bpmp::Optimizer] Step accepted without expansion. "
                        <<" iter: : " << iter_inner<< " d: " << d << " rho: " << rho << std::endl;
                    }
                    // no trust region change
                }
            }

            // check constraints at final u_bar
            double Jm_final=0.0, c_final=0.0;
            get_J_m_and_constraints(b_r0, b_t0, u_bar, obs_centers, eta, Jm_final, c_final, param_, u_t);

            if (c_final <= param_.tau_p) {
                if (param_.verbose) {
                    std::cout << "[bpmp::Optimizer] All constraints satisfied. Stopping outer loop. iter_outer: " << iter_outer << std::endl;
                }
                break;
            } else {
                if (param_.verbose) {
                    std::cout << "[bpmp::Optimizer] Outer iter " << iter_outer << " completed. Constraint violation: "
                                << c_final << ". Increasing penalty eta." << std::endl;
                }
                eta = std::min(param_.eta_max, eta * param_.beta);
            }
        }

        // write back to solution array
        for(int k=0;k<N_;k++){
            u_sol_[k] = u_bar.row(k).transpose();
        }
        float toc = static_cast<float>(std::clock());
        float elapsed = (toc - tic) / CLOCKS_PER_SEC;
        if (param_.verbose){
            std::cout << "[bpmp::Optimizer] Iterations: outer = " << iter_outer << ", inner = " << iter_inner << " Solve() time: " << elapsed << " sec" << std::endl;
        }   
    }

    const Collection<VectorU, N_>& solution() const { return u_sol_; }

private:
    const bpmp::Problem* problem_{nullptr};
    // VectorX x_init_;
    Collection<VectorU, N_> u_init_;
    OptimizationParam param_;
    Collection<VectorU, N_> u_sol_{};
};

} // namespace bpmp

#endif // BPMP_TRACKER_OPTIMIZER_HPP
