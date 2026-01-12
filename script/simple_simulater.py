import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from SCP_solver_by_paper import SCP, BeliefState, FOVParams, robot_step, target_step
from bpmp_tracker.msg import ObjectStateList, ObjectState
from copy import deepcopy


class simulator:
    def __init__(self, init_robot_state, init_target_state, DT):
        self.robot_state = [init_robot_state[0], init_robot_state[1], init_robot_state[2]]  # x, y, theta
        self.target_state = [init_target_state[0], init_target_state[1]] # x, y only
        # self.obstacle_states = deepcopy(init_obstacle_states)  # list of [x, y] for each obstacle
        self.DT = DT
    def step(self, control_input, target_control_input=None):
        # Update robot state
        v = control_input[0]
        w = control_input[1]
        dt = self.DT
        theta = self.robot_state[2]
        if abs(w) < 1e-6:
            self.robot_state[0] += v * np.cos(theta) * dt
            self.robot_state[1] += v * np.sin(theta) * dt
        else:
            # self.robot_state[0] += (v / w) * (np.sin(theta + w * dt) - np.sin(theta))
            # self.robot_state[1] += (v / w) * (-np.cos(theta + w * dt) + np.cos(theta))
            self.robot_state[0] += v * np.cos(theta) * dt
            self.robot_state[1] += v * np.sin(theta) * dt
        self.robot_state[2] += w * dt
        # self.robot_state[3] = v

        # Update target state
        self.target_state[0] += target_control_input[0] * dt
        self.target_state[1] += target_control_input[1] * dt




# def visualization_SCP(u_cur, obstacle_states, b_r0, b_t0,
#     target_control_predicted,
#     fov,
#     click_to_close=True
# ):
#     # 아무상관없는값들임
#     robot_process_noise = np.eye(4)*0.05
#     target_process_noise= np.eye(2)*0.1
#     target_sensor_noise = np.eye(2)*0.05

#     N, nu = u_cur.shape
#     # Rollout with optimized control
#     b_r_list  = [b_r0]
#     for u in u_cur:
#         b_r_next = robot_step(b_r_list[-1], u, robot_process_noise)
#         b_r_list.append(b_r_next)
    
#     b_t_list  = [b_t0]
#     for k in range(N):
#         if target_control_predicted is not None:
#             u_t = target_control_predicted[k]
#         else:
#             u_t = np.array([0.0, 0.0])  # default zero control for target

#         b_t_next, _ = target_step(b_t_list[-1], u_t, target_process_noise, target_sensor_noise,
#                                         b_r_list[k+1], obstacle_states[k], fov)
#         b_t_list.append(b_t_next)

#     # Plotting
#     plt.clf()

#     robot_x = [b_r.mean[0] for b_r in b_r_list]
#     robot_y = [b_r.mean[1] for b_r in b_r_list]
#     target_x = [b_t.mean[0] for b_t in b_t_list]
#     target_y = [b_t.mean[1] for b_t in b_t_list]

#     plt.plot(robot_x, robot_y, 'b-o', label='Robot Path')
#     plt.plot(target_x, target_y, 'r-s', label='Target Path')

#     if len(obstacle_states[0]) > 0:
#         for obs_t0 in obstacle_states[0]:
#             plt.plot(obs_t0[0], obs_t0[1], 'kx', markersize=10, label='Obstacle')
#         for obs_tN in obstacle_states[-1]:
#             plt.plot(obs_tN[0], obs_tN[1], 'k+', markersize=10)

#     # plot fov
#     for k, b_r in enumerate(b_r_list):
#         fov_angle = fov.fov / 2.0
#         r_min = fov.r_min
#         r_max = fov.r_max
#         theta = b_r.mean[2]
#         # fov_x = [b_r.mean[0], 
#         #          b_r.mean[0] + r_max * np.cos(theta + fov_angle), 
#         #          b_r.mean[0] + r_max * np.cos(theta - fov_angle), 
#         #          b_r.mean[0]]
#         # fov_y = [b_r.mean[1], b_r.mean[1] + r_max * np.sin(theta + fov_angle), b_r.mean[1] + r_max * np.sin(theta - fov_angle), b_r.mean[1]]
#         inner_arc_theta = np.linspace(theta - fov_angle, theta + fov_angle, 20)
#         outer_arc_theta = np.linspace(theta + fov_angle, theta - fov_angle, 20)
#         fov_x = np.concatenate(([b_r.mean[0] + r_min * np.cos(inner_arc_theta)],
#                                 [b_r.mean[0] + r_max * np.cos(outer_arc_theta)],)).flatten()
#         fov_y = np.concatenate(([b_r.mean[1] + r_min * np.sin(inner_arc_theta)],
#                                 [b_r.mean[1] + r_max * np.sin(outer_arc_theta)])).flatten()
#         fov_x = np.append(fov_x, fov_x[0])
#         fov_y = np.append(fov_y, fov_y[0])
#         plt.plot(fov_x, fov_y, 'g--', alpha=0.8 ** (len(b_r_list)-k))
    
#     for i, obs_states in enumerate(obstacle_states):
#         for obs in obs_states:
#             circle = Circle((obs[0], obs[1]), OBS_RADIUS, color='k', alpha=0.8 ** (len(obstacle_states)-i), fill=False)
#             plt.gca().add_patch(circle)
#     plt.xlabel('X position')
#     plt.ylabel('Y position')
#     plt.title('SCP Optimized Paths')
#     plt.legend()
#     plt.axis('equal')
#     plt.grid()
#     if click_to_close:
#         plt.waitforbuttonpress(0.01)
#     else:
#         plt.show()


if __name__ == "__main__":
    test_fov_params = FOVParams(fov=np.deg2rad(100), r_min=0.2, r_max=1.8)
    
    DT = 0.5
    SIM_DT = 0.1
    OBS_RADIUS = 0.25
    N = 4
    u_init = np.zeros((N, 2))
    # u_init[:, 0] = -np.deg2rad(90)  # initial angular velocity
    
    w_max = np.deg2rad(90)  # max angular velocity
    a_max = 1.0              # max acceleration
    a_min = -1.0             # min acceleration
    # plt.figure()
    # obstacles = []
    test_obstacles = ObjectStateList()
    obstacle_states = np.zeros((N,0,2))
    
    simul = simulator(
        init_robot_state=[0.0, 0.0, 0, 0.0],
        init_target_state=[1.0, 0.0],
        # init_obstacle_states=obstacles,
        DT=SIM_DT
    )
    cur_robot_v = 0

    simul_index = 0
    plt.figure()
    
    while True:
        simul_index = simul_index + 1
        target_vel = [0.5*(1-np.cos(simul_index * DT)), 0.0]
        # target_vel = [0.0, 0.0]
        b_r0 = BeliefState(mean=np.array([simul.robot_state[0], simul.robot_state[1], simul.robot_state[2], cur_robot_v]), covariance=np.eye(4)*0.001, dim=4)
        b_t0 = BeliefState(mean=np.array([simul.target_state[0], simul.target_state[1]]), covariance=np.eye(2)*0.005, dim=2)

        optimized_u = SCP(
            b_r0, b_t0,
            target_control_predicted=np.array([target_vel for _ in range(u_init.shape[0])]),
            obstacles=test_obstacles,
            fov=test_fov_params,
            u_init=u_init,
            w_max = w_max,
            a_max = a_max,
            a_min = a_min,
            # d0=10.0,
        )
        cur_robot_v += optimized_u[0][1] * SIM_DT
        simul.step((cur_robot_v, optimized_u[0][0]), target_control_input=target_vel)

        
        # visualization_SCP(
        #     optimized_u, 
        #     obstacle_states=obstacle_states,
        #     b_r0=b_r0,
        #     b_t0=b_t0,
        #     target_control_predicted=np.array([target_vel for _ in range(u_init.shape[0])]),
        #     fov=test_fov_params,
        # )