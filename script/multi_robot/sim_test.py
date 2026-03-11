import numpy as np
from collections import namedtuple

# params = {
#     'attach_pos': [np.array([0.5, 0.0]), np.array([-0.5, 0.0])],
#     'Kv': [10.0, 10.0],
#     'Ft_max': [10.0, 10.0],
#     'c_roll': [1.0, 1.0],
#     'f_roll': [0.5, 0.5],
#     'eps_v': 0.1,
#     'c_lat': [100.0, 100.0],
#     'mu_lat': [0.5, 0.5],
#     'Kw': [5.0, 5.0],
#     'tau_max': [5.0, 5.0],
#     'c_yaw': [1.0, 1.0],
#     'J_robot': [1.0, 1.0],
#     'm_eff': 10.0,
#     'J_eff': 5.0
# }
SIM_TIME = 0

params = namedtuple('Params', [
    'attach_pos', 'Kv', 'Ft_max', 'c_roll', 'f_roll', 'eps_v', 'c_lat', 'mu_lat', 'Kw', 'tau_max', 'c_yaw', 'J_robot', 'm_eff', 'J_eff', 'N'
])(
    attach_pos=[np.array([0.5, 0.0]), np.array([-0.5, 0.0])],
    Kv=[20.0, 20.0],
    Ft_max=[10.0, 10.0],
    c_roll=[0.00, 0.00],
    f_roll=[0.00, 0.00],
    eps_v=0.1,
    c_lat=[100.0, 100.0],
    mu_lat=[0.5, 0.5],
    Kw=[10.0, 10.0],
    tau_max=[5.0, 5.0],
    c_yaw=[0.0, 0.0],
    J_robot=[0.5, 0.5],
    m_eff=10.0,
    J_eff=5.0,
    N=[200,200]
)
N = 2
def rot2(theta):
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([[c, -s], [s, c]])

def pack(p, theta, v, omega, psi, psi_dot):
    return np.concatenate([p, [theta], v, [omega], psi, psi_dot])

def unpack(state):
    p = state[0:2]
    theta = state[2]
    v = state[3:5]
    omega = state[5]
    psi = state[6:6+N]
    psi_dot = state[6+N:6+2*N]
    return p, theta, v, omega, psi, psi_dot

def perp(v):
    return np.array([-v[1], v[0]])

def cross2(a, b):
    return a[0]*b[1] - a[1]*b[0]

def step(state, cmds, params, dt):
    # state:
    # p: (2,), theta: scalar
    # v: (2,), omega: scalar
    # psi: (N,), psi_dot: (N,)
    
    global SIM_TIME
    SIM_TIME += dt
    p, theta, v, omega, psi, psi_dot = unpack(state)

    R = rot2(theta)

    F_total = np.zeros(2)
    tau_total = 0.0
    psi_ddot = np.zeros_like(psi)

    for i in range(N):
        r_obj = params.attach_pos[i]           # in object frame
        r_w   = R @ r_obj

        # attach point velocity due to object motion
        v_i = v + omega * perp(r_w)

        # robot body axes
        t_i = np.array([np.cos(psi[i]), np.sin(psi[i])])
        n_i = perp(t_i)

        # velocity decomposition
        v_t = np.dot(t_i, v_i)
        v_n = np.dot(n_i, v_i)

        # command
        v_cmd = cmds[i].linvel
        w_cmd = cmds[i].angvel

        # longitudinal drive force (PID-like)
        F_drive = np.clip(
            params.Kv[i] * (v_cmd - v_t),
            -params.Ft_max[i], 
            params.Ft_max[i]
        )

        # passive rolling resistance
        F_roll = -params.c_roll[i] * v_t -params.f_roll[i] * np.tanh(v_t / params.eps_v)

        # lateral friction (much stronger)
        F_lat = -np.clip(
            params.c_lat[i] * v_n,
            -params.mu_lat[i] * params.N[i],
             params.mu_lat[i] * params.N[i]
        )

        # net ground reaction through robot
        F_i = (F_drive + F_roll) * t_i + F_lat * n_i

        print(F_drive, F_roll, (F_drive + F_roll))

        # robot yaw torque
        tau_i = np.clip(
            params.Kw[i] * (w_cmd - psi_dot[i]),
            -params.tau_max[i], params.tau_max[i]
        )
        
        # accumulate object wrench
        F_total += F_i
        tau_total += cross2(r_w, F_i)

        # robot yaw dynamics
        psi_ddot[i] = (tau_i - params.c_yaw[i] * psi_dot[i]) / params.J_robot[i]
        # print(tau_i, psi_ddot[i])

    # object dynamics
    a = F_total / params.m_eff
    alpha = tau_total / params.J_eff

    # semi-implicit Euler
    v += a * dt
    print(a)
    omega += alpha * dt
    psi_dot += psi_ddot * dt
    # print(psi_dot)

    p += v * dt
    theta += omega * dt
    psi += psi_dot * dt

    return pack(p, theta, v, omega, psi, psi_dot)

from matplotlib import pyplot as plt
from matplotlib import patches

def render(state, params):
    global SIM_TIME
    p, theta, _, _, psi, _ = unpack(state)

    R = rot2(theta)

    attach_points = []
    for i in range(N):
        r_obj = params.attach_pos[i]
        r_w = R @ r_obj
        attach_points.append(p + r_w)
    
    plt.plot(p[0], p[1], 'ko')
    plt.arrow(p[0], p[1], 0.5*np.cos(theta), 0.5*np.sin(theta), head_width=0.1, head_length=0.2, fc='k', ec='k')
    # plt.gca().add_patch(patches.Rectangle((p[0]-1.0, p[1]-0.5), 2.0, 1.0, angle=np.degrees(theta), fill=False))
    plt.gca().add_patch(patches.Ellipse(p, 2.0, 1.0, angle=np.degrees(theta), fill=False))
    for i in range(N):
        plt.plot(attach_points[i][0], attach_points[i][1], 'ro')
        plt.arrow(attach_points[i][0], attach_points[i][1], 0.2*np.cos(psi[i]), 0.2*np.sin(psi[i]), head_width=0.05, head_length=0.1, fc='r', ec='r')
    plt.axis('equal')
    plt.xlim(p[0]-2, p[0]+2)
    plt.ylim(p[1]-2, p[1]+2)
    plt.grid()
    plt.title(f"Time: {SIM_TIME:.2f} sec")


def main():

    dt = 0.01
    state = pack(
        p=np.array([0.0, 0.0]),
        theta=0.0,
        v=np.array([0.0, 0.0]),
        omega=0.0,
        psi=np.array([np.pi/2, 0.0]),
        psi_dot=np.array([0.0, 0.0])
    )

    class Cmd:
        def __init__(self, linvel, angvel):
            self.linvel = linvel
            self.angvel = angvel

    cmds = [Cmd(1, 0), Cmd(0, 0)]
    plt.figure()

    while True:

        state = step(state, cmds, params, dt)
        plt.clf()
        render(state, params)
        plt.waitforbuttonpress(0.01)
        
        

if __name__ == '__main__':
    main()