import numpy as np

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

def sd_annular_sector_fov_gradient(x_t, x_r, heading_r, fov_angle, eps=1e-6, r_min=0.5, r_max=5.0):
    sd_value = sd_annular_sector_fov(
        x_t,
        sector_origin=x_r,
        sector_dir=heading_r,
        r_min=r_min,
        r_max=r_max,
        fov_angle=fov_angle,
    )

    sd_value_dx_t = (sd_annular_sector_fov(
        x_t + np.array([eps, 0.0]),
        sector_origin=x_r,
        sector_dir=heading_r,
        r_min=r_min,
        r_max=r_max,
        fov_angle=fov_angle,
    ) - sd_value) / eps

    sd_value_dy_t = (sd_annular_sector_fov(
        x_t + np.array([0.0, eps]),
        sector_origin=x_r,
        sector_dir=heading_r,
        r_min=r_min,
        r_max=r_max,
        fov_angle=fov_angle,
    ) - sd_value) / eps

    sd_value_dx_r = (sd_annular_sector_fov(
        x_t,
        sector_origin=x_r + np.array([eps, 0.0]),
        sector_dir=heading_r,
        r_min=r_min,
        r_max=r_max,
        fov_angle=fov_angle,
    ) - sd_value) / eps

    sd_value_dy_r = (sd_annular_sector_fov(
        x_t,
        sector_origin=x_r + np.array([0.0, eps]),
        sector_dir=heading_r,
        r_min=r_min,
        r_max=r_max,
        fov_angle=fov_angle,
    ) - sd_value) / eps

    sd_value_dheading_r = (sd_annular_sector_fov(
        x_t,
        sector_origin=x_r,
        sector_dir=heading_r + eps,
        r_min=r_min,
        r_max=r_max,
        fov_angle=fov_angle,
    ) - sd_value) / eps

    # normalize gradient w.r.t x y of robot position, target position to unit norm
    # since the gradient should be 1.0 in magnitude 
    norm_t = np.sqrt(sd_value_dx_t**2 + sd_value_dy_t**2) + 1e-12
    sd_value_dx_t /= norm_t
    sd_value_dy_t /= norm_t
    norm_r = np.sqrt(sd_value_dx_r**2 + sd_value_dy_r**2) + 1e-12
    sd_value_dx_r /= norm_r
    sd_value_dy_r /= norm_r

    return sd_value_dx_t, sd_value_dy_t, sd_value_dx_r, sd_value_dy_r, sd_value_dheading_r



if __name__ == "__main__":
    # simple test
    import matplotlib.pyplot as plt

    xx, yy = np.meshgrid(np.linspace(-5, 5, 200), np.linspace(-5, 5, 200))
    zz = np.stack([xx, yy], axis=-1)

    # x_t = np.array([2.0, 1.0])
    # x_r = np.array([0.0, -1.0])
    # heading_r = np.pi / 4
    x_t = np.array([2.0, 2.0])
    x_r = np.array([0.0, 0.0])
    heading_r = np.pi / 4
    fov_angle = np.pi / 3
    r_min = 1.0
    r_max = 4.0
    eps = 1e-6
    x_t_list=[]
    sd_value_grad_list=[]

    for i in np.arange(-1, 1, 0.1):
        for j in np.arange(-1, 1, 0.1):
            x_t_temp = x_t + np.array([i, j])
            x_t_list.append(x_t_temp.copy())
            sd_value_dx_t, sd_value_dy_t, sd_value_dx_r, sd_value_dy_r, sd_value_dheading_r = sd_annular_sector_fov_gradient(
                x_t_temp, x_r, heading_r, fov_angle, eps, r_min, r_max
            )
            sd_value_grad_list.append([sd_value_dx_t, sd_value_dy_t, sd_value_dx_r, sd_value_dy_r, sd_value_dheading_r])
            print("abs of grad w.r.t Target Pos:", np.sqrt(sd_value_dx_t**2 + sd_value_dy_t**2))


    sd = sd_annular_sector_fov(
        zz,
        sector_origin=x_r,
        sector_dir=heading_r,
        r_min=r_min,
        r_max=r_max,
        fov_angle=fov_angle,
    )

    plt.contourf(xx, yy, sd, levels=200, cmap="RdBu_r")
    plt.colorbar(label="Signed Distance")
    plt.contour(xx, yy, sd, levels=[0.0], colors="k", linewidths=2)

    plt.plot(x_r[0], x_r[1], "ro", label="Robot")
    # plt.arrow(
    #     x_r[0], x_r[1],
    #     0.5 * np.cos(heading_r), 0.5 * np.sin(heading_r),
    #     head_width=0.1, head_length=0.1, fc="r", ec="r", label="Heading"
    # )
    # plot gradient
    # plt.quiver(
    #     x_r[0], x_r[1],
    #     sd_value_dx_r, sd_value_dy_r,
    #     color="r", scale=10.0, width=0.005, label="Grad w.r.t Robot Pos"
    # )
    print("abs of grad w.r.t Robot Pos:", np.sqrt(sd_value_dx_r**2 + sd_value_dy_r**2))
    # plt.quiver(
    #     x_t[0], x_t[1],
    #     sd_value_dx_t, sd_value_dy_t,
    #     color="g", scale=10.0, width=0.005, label="Grad w.r.t Target Pos"
    # )
    for i in range(len(x_t_list)):
        plt.quiver(
            x_t_list[i][0], x_t_list[i][1],
            sd_value_grad_list[i][0], sd_value_grad_list[i][1],
            color="g", scale=60.0, width=0.002
        )
        plt.quiver(
            x_r[0], x_r[1],
            sd_value_grad_list[i][2], sd_value_grad_list[i][3],
            color="r", scale=20.0, width=0.002
        )

    print("abs of grad w.r.t Target Pos:", np.sqrt(sd_value_dx_t**2 + sd_value_dy_t**2))
    plt.plot(x_t[0], x_t[1], "go", label="Target")
    plt.legend()
    plt.axis("equal")
    plt.title("Signed Distance to Annular Sector FOV")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.show()