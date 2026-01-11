import rospy
import numpy as np
from bpmp_tracker.msg import ObjectStateList, ObjectState, UnicycleInput
from nav_msgs.msg import Odometry
from SCP_solver_by_paper import SCP
from dataclasses import dataclass

DT = 0.5  # time step duration
OBS_RADIUS = 0.5  # obstacle radius for collision checking
N_horizon = 4  # prediction horizon


class SCPPossdfRosNode:
    def __init__(self):
        rospy.init_node('scp_possdf_ros_node')

        self.robot_state_sub = rospy.Subscriber('/base_odom', Odometry, self.robot_state_callback)
        self.target_states_sub = rospy.Subscriber('/bpmp_simulator/target_state', ObjectState, self.target_states_callback)
        self.obstacle_state_list_sub = rospy.Subscriber('/bpmp_simulator/obstacle_state_list', ObjectStateList, self.obstacle_states_callback)

        self.robot_control_pub = rospy.Publisher('/bpmp_tracker/unicycle_control_input', UnicycleInput, queue_size=10)

        self.robot_state_prev = [None,None,None] # time, x, y

        self.robot_state = None # array [x, y, theta, vel]
        self.target_states = None # array [x, y, vx, vy]
        self.obstacle_states = None # array [N_horizon, I_th obstacle, x, y]

    def robot_state_callback(self, msg : Odometry):
        if self.robot_state_prev[0] is None\
            or np.sqrt((msg.pose.pose.position.x - self.robot_state_prev[1])**2 +
                       (msg.pose.pose.position.y - self.robot_state_prev[2])**2) < 1.0:
            self.robot_state_prev[0] = rospy.get_time()
            self.robot_state_prev[1] = msg.pose.pose.position.x
            self.robot_state_prev[2] = msg.pose.pose.position.y
            self.robot_state = [msg.pose.pose.position.x,
                                msg.pose.pose.position.y,
                                0.0,
                                0.0]
        else:
            q = msg.pose.pose.orientation
            yaw = np.arctan2(2.0 * (q.w * q.z + q.x * q.y),
                             1.0 - 2.0 * (q.y * q.y + q.z * q.z))

            dt = rospy.get_time() - self.robot_state_prev[0]
            vx = (msg.pose.pose.position.x - self.robot_state_prev[1]) / dt
            vy = (msg.pose.pose.position.y - self.robot_state_prev[2]) / dt
            v = np.sqrt(vx**2 + vy**2)
            sign_of_v = 1.0 if (vx * np.cos(yaw) + vy * np.sin(yaw)) >= 0.0 else -1.0
            

            self.robot_state = [msg.pose.pose.position.x,
                                msg.pose.pose.position.y,
                                yaw,
                                sign_of_v * v]

    def target_states_callback(self, msg: ObjectState):
        self.target_states = [msg.px, msg.py, msg.vx, msg.vy]

    def obstacle_states_callback(self, msg: ObjectStateList):
        self.obstacle_states = []
        for obj in msg.object_state_list:
            vx = obj.vx
            vy = obj.vy
            for i in range(N_horizon):
                self.obstacle_states.append([obj.px + vx * i * DT, obj.py + vy * i * DT])


    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if self.robot_state and self.target_states:
                # Process the robot and target states
                robot_pos = np.array([self.robot_state.pose.pose.position.x,
                                      self.robot_state.pose.pose.position.y])
                target_positions = [np.array([obj.state.position.x, obj.state.position.y]) for obj in self.target_states.objects]

                # Here you would call your SCP solver with the current states
                # For example:
                # optimized_control = SCP(...)

                # For demonstration, we just print the states
                rospy.loginfo(f"Robot Position: {robot_pos}")
                for i, target_pos in enumerate(target_positions):
                    rospy.loginfo(f"Target {i} Position: {target_pos}")

            rate.sleep()