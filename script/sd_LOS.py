import numpy as np

def sd_LOS(los_point1, los_point2, obs_origin, obs_radius, eps=1e-12):
    a = np.asarray(los_point1, dtype=float).reshape(2,)  # robot
    b = np.asarray(los_point2, dtype=float).reshape(2,)  # target
    c = np.asarray(obs_origin, dtype=float).reshape(2,)  # obstacle center

    v = b - a
    vv = float(np.dot(v, v))
    if vv < eps:
        d = c - a
        return float(np.linalg.norm(d) - obs_radius)

    t = float(np.dot(c - a, v) / vv)
    t = float(np.clip(t, 0.0, 1.0))
    q = a + t * v
    d = c - q
    return float(np.linalg.norm(d) - obs_radius)

def sd_LOS_gradient(x_t, x_r, obs_origin, obs_radius, eps=1e-12):
    """Gradient of sd_LOS wrt target (x_t) and robot (x_r) positions.

    Returns: dsd/dx_t, dsd/dy_t, dsd/dx_r, dsd/dy_r
    """
    a = np.asarray(x_r, dtype=float).reshape(2,)
    b = np.asarray(x_t, dtype=float).reshape(2,)
    c = np.asarray(obs_origin, dtype=float).reshape(2,)

    v = b - a
    vv = float(np.dot(v, v))
    if vv < eps:
        d = c - a
        n = float(np.linalg.norm(d)) + eps
        u = d / n
        grad_a = -u
        grad_b = np.zeros(2)
        return float(grad_b[0]), float(grad_b[1]), float(grad_a[0]), float(grad_a[1])

    t_raw = float(np.dot(c - a, v) / vv)
    t = float(np.clip(t_raw, 0.0, 1.0))
    q = a + t * v
    d = c - q
    n = float(np.linalg.norm(d)) + eps
    u = d / n

    if t_raw <= 0.0:
        grad_a = -u
        grad_b = np.zeros(2)
    elif t_raw >= 1.0:
        grad_a = np.zeros(2)
        grad_b = -u
    else:
        grad_a = -(1.0 - t) * u
        grad_b = -t * u

    return float(grad_b[0]), float(grad_b[1]), float(grad_a[0]), float(grad_a[1])




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
    # heading_r = np.pi / 4
    # fov_angle = np.pi / 3
    # r_min = 1.0
    # r_max = 4.0
    eps = 1e-6

    obstacle_origin = np.array([1.0, 0.0])
    obstacle_radius = 0.5

    obstacle_origin_list=[]
    sd_value_grad_list=[]

    for i in np.arange(-1, 3, 0.1):
        for j in np.arange(-1, 3, 0.1):
            obstacle_origin_tmp = np.array([i, j])
            obstacle_origin_list.append(obstacle_origin_tmp.copy())
            sd_value_dx_t, sd_value_dy_t, sd_value_dx_r, sd_value_dy_r = sd_LOS_gradient(x_t, x_r, obstacle_origin_tmp, obstacle_radius, eps=eps)
            sd_value_grad_list.append([sd_value_dx_t, sd_value_dy_t, sd_value_dx_r, sd_value_dy_r])
            print("abs of grad w.r.t Target Pos:", np.sqrt(sd_value_dx_t**2 + sd_value_dy_t**2))


    sd = np.zeros(zz.shape[0:2], dtype=float)
    for i in range(zz.shape[0]):
        for j in range(zz.shape[1]):
            sd[i, j] = sd_LOS(los_point1=x_t, los_point2=x_r, obs_origin=zz[i, j], obs_radius=obstacle_radius)


    plt.contourf(xx, yy, sd, levels=200, cmap="RdBu_r")
    plt.colorbar(label="Signed Distance")
    plt.contour(xx, yy, sd, levels=[0.0], colors="k", linewidths=2)

    plt.plot(x_r[0], x_r[1], "ro", label="Robot", markersize=8)
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
    for i in range(len(obstacle_origin_list)):
        plt.scatter(obstacle_origin_list[i][0], obstacle_origin_list[i][1], color="k", s=5)
        plt.quiver(
            obstacle_origin_list[i][0], obstacle_origin_list[i][1],
            sd_value_grad_list[i][0], sd_value_grad_list[i][1],
            color="g", scale=60.0, width=0.002
        )
        plt.quiver(
            obstacle_origin_list[i][0], obstacle_origin_list[i][1],
            sd_value_grad_list[i][2], sd_value_grad_list[i][3],
            color="r", scale=60.0, width=0.002
        )

    print("abs of grad w.r.t Target Pos:", np.sqrt(sd_value_dx_t**2 + sd_value_dy_t**2))
    plt.plot(x_t[0], x_t[1], "go", label="Target", markersize=8)
    plt.legend()
    plt.axis("equal")
    plt.title("Signed Distance to Annular Sector FOV")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.show()