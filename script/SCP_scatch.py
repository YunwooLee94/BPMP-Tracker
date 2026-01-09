import numpy as np


def rollout_nominal(b_r0, b_t0, u_seq):
    # Placeholder for the nominal rollout function
    pass





def algorithm2_scp_with_trust_region(
    # problem data
    b_r0, b_t0, obstacles, fov_params,
    # initial guess
    u_init,                   # shape (N, nu)
    # hyperparams
    eta0=1.0, beta=10.0, eta_max=1e6,   # penalty continuation (outer loop)
    d0=0.5, d_min=1e-4, d_max=5.0,      # trust-region radius
    shrink=0.5, expand=1.5,             # trust-region update factors
    rho_reject=0.1, rho_expand=0.75,    # acceptance thresholds
    tau_conv=1e-3, max_inner=50, max_outer=10,
    trust_norm="inf",                   # "inf" or "l2"
):
    """
    논문 Algorithm 2 스타일:
      - outer loop: penalty eta 증가
      - inner loop: SCP iteration
      - line 12: trust region constraint 추가한 convex subproblem(QP) solve
    """

    u_bar = u_init.copy()   # current nominal control (u^{(n)})
    eta = eta0
    d = d0

    # (가정) 아래 함수들은 이미 구현되어 있다고 가정:
    # 1) rollout_nominal(b_r0, b_t0, u_seq) -> (traj, beliefs, covs 등)
    # 2) update_sdf_params_C(traj, obstacles, fov) -> C   (Algorithm 1 역할)
    # 3) linearize_convex_model(traj, u_bar, C, eta) -> model
    #       model은 QP가 쓰는 선형화/근사 정보 포함.
    # 4) solve_convex_subproblem(model, u_ref=u_bar, trust_radius=d, trust_norm=...) -> (u_star, predicted_decrease)
    #       여기서 line 12 trust region 제약을 실제로 넣어야 함:
    #         ||u - u_ref||_inf <= d   또는   ||u - u_ref||_2 <= d
    # 5) eval_true_merit_Jm(traj, u_seq, C, eta) -> float
    #
    # predicted_decrease는 보통:
    #   Jm_lin(u_ref) - Jm_lin(u_star)
    # (선형화된/볼록근사된 목적에서의 감소량)으로 리턴시키면 rho 계산이 깔끔해짐.

    outer_iter = 0
    while eta < eta_max and outer_iter < max_outer:
        outer_iter += 1

        # inner SCP loop
        inner_iter = 0
        while inner_iter < max_inner:
            inner_iter += 1

            # (1) nominal rollout (현재 u_bar 기준)
            traj_bar = rollout_nominal(b_r0, b_t0, u_bar)  # states/beliefs along horizon

            # (2) Algorithm 1: SDF 파라미터 세트 C 업데이트 (고정/동결용)
            C_bar = update_sdf_params_C(traj_bar, obstacles, fov_params)

            # (3) convexification (선형화/근사 모델 구성): Jm의 convex surrogate
            model = linearize_convex_model(traj_bar, u_bar, C_bar, eta)

            # (4) [Algorithm 2 line 12] solve convex subproblem with TRUST REGION
            #     trust region은 "추정해가 너무 멀리 튀지 않게" 하는 핵심 장치
            u_star, pred_dec = solve_convex_subproblem(
                model,
                u_ref=u_bar,
                trust_radius=d,
                trust_norm=trust_norm
            )

            if u_star is None:
                # QP infeasible or solver fail -> shrink trust region and retry
                d = max(d * shrink, d_min)
                if d <= d_min:
                    break
                continue

            # (5) 실제(nonconvex) merit Jm 평가 (accept/reject 판단)
            #     후보 u_star로 다시 rollout 해서 "진짜 Jm" 계산
            traj_star = rollout_nominal(b_r0, b_t0, u_star)

            Jm_bar  = eval_true_merit_Jm(traj_bar,  u_bar,  C_bar, eta)
            Jm_star = eval_true_merit_Jm(traj_star, u_star, C_bar, eta)

            act_dec = Jm_bar - Jm_star
            # pred_dec가 0에 가깝거나 음수면 rho가 터지니 방어
            denom = max(pred_dec, 1e-12)
            rho = act_dec / denom

            # (6) trust region update + step accept/reject
            if rho < rho_reject or act_dec <= 0.0:
                # reject: 선형화 모델이 믿을 만하지 않음 -> 반경 줄이고 다시
                d = max(d * shrink, d_min)
                if d <= d_min:
                    # 더 줄여도 의미 없으면 inner 종료
                    break
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

        # (7) penalty 업데이트 (outer continuation)
        eta *= beta

    return u_bar