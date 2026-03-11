#!/usr/bin/env python3
import math
import os
import select
import sys
import time
import threading

import rospy
import tf
from geometry_msgs.msg import Twist
from bpmp_tracker.msg import UnicycleInput


if os.name == "nt":
    import msvcrt
else:
    import termios
    import tty

LIN_VEL_STEP_SIZE = 0.02
ANG_VEL_STEP_SIZE = 0.02


def constrain(val, lo, hi):
    if val < lo:
        return lo
    if val > hi:
        return hi
    return val


class Robot:
    def __init__(self, name, index, tf_listener, theta_offset=0.0):
        self.name = name
        self.index = index
        self.tf_frame = f"robot_{index + 1}_base_link"
        self.theta_offset = theta_offset
        self.listener = tf_listener
        self.cmd_pub = rospy.Publisher(
            f"/bpmp_tracker/unicycle_control_input_{index}", UnicycleInput, queue_size=1
        )

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.x_obj = 0.0
        self.y_obj = 0.0
        self.l = 0.0
        self.pose_theta = 0.0

        self.target_linear_velocity = 0.0
        self.target_angular_velocity = 0.0
        self.target_theta = 0.0
        self.desired_theta = 0.0
        self.K_p = 2.0

    def update_pose_from_tf(self):
        try:
            (trans, rot) = self.listener.lookupTransform("map", self.tf_frame, rospy.Time(0))
            self.x, self.y = trans[0], trans[1]
            yaw = tf.transformations.euler_from_quaternion(rot)[2]
            self.theta = yaw + math.pi + self.theta_offset
            return True
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return False

    def initialize_object_frame(self, xmid, ymid, base_angle):
        x_rel = self.x - xmid
        y_rel = self.y - ymid
        self.x_obj = -x_rel * math.cos(-base_angle) + y_rel * math.sin(-base_angle)
        self.y_obj = -x_rel * math.sin(-base_angle) - y_rel * math.cos(-base_angle)

    def set_target(self, linear_vel, target_theta, base_angle, control_angular_velocity):
        self.target_theta = target_theta + base_angle
        self.desired_theta = target_theta + base_angle
        angle_diff = (self.target_theta - self.theta) % (2 * math.pi)
        if angle_diff < 0:
            angle_diff += 2 * math.pi
        if angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        self.target_linear_velocity = linear_vel * math.cos(angle_diff)
        self.target_angular_velocity = self.K_p * angle_diff + control_angular_velocity

    def publish_velocity(self):
        msg = UnicycleInput()
        msg.vel_linear = self.target_linear_velocity
        msg.vel_angular = self.target_angular_velocity
        self.cmd_pub.publish(msg)


class RobotController:
    def __init__(self):
        rospy.init_node("bpmp_manual_control", anonymous=False)
        self.listener = tf.TransformListener()

        # default symmetric two-robot configuration
        config = {
            "robots": {
                "robot1": {"theta_offset": 0.0},
                "robot2": {"theta_offset": 0.0},
            }
        }

        self.robots = []
        for idx, name in enumerate(config["robots"].keys()):
            self.robots.append(
                Robot(
                    name=name,
                    index=idx,
                    tf_listener=self.listener,
                    theta_offset=config["robots"][name]["theta_offset"],
                )
            )

        self.control_first = True
        self.control_second = False
        self.control_all = False

        self.control_linear_velocity = 0.0
        self.control_angular_velocity = 0.0
        self.theta_steer = 0.0
        self.theta_steer_velocity = 0.0
        self.fixed_l = None
        self.last_theta_update = time.time()

        self.desired_pub = rospy.Publisher("/bpmp_manual/desired", Twist, queue_size=1)

    def get_key(self, settings):
        if os.name == "nt":
            return msvcrt.getch().decode("utf-8")
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def update_theta_steer_with_velocity(self):
        now = time.time()
        dt = now - self.last_theta_update
        self.last_theta_update = now
        self.theta_steer += self.theta_steer_velocity * dt
        self.theta_steer = (self.theta_steer + math.pi) % (2 * math.pi) - math.pi

    def run(self):
        settings = None
        if os.name != "nt":
            settings = termios.tcgetattr(sys.stdin)

        rate = rospy.Rate(50.0)
        print(self.help_message())
        while not rospy.is_shutdown():
            key = self.get_key(settings)
            self.process_key(key)
            self.update_theta_steer_with_velocity()
            self.update_robots()
            rate.sleep()

    def process_key(self, key):
        if key == "w":
            self.control_linear_velocity = constrain(
                self.control_linear_velocity + LIN_VEL_STEP_SIZE, -2.0, 2.0
            )
        elif key == "x":
            self.control_linear_velocity = constrain(
                self.control_linear_velocity - LIN_VEL_STEP_SIZE, -2.0, 2.0
            )
        elif key == "a":
            self.control_angular_velocity = constrain(
                self.control_angular_velocity + ANG_VEL_STEP_SIZE, -3.0, 3.0
            )
        elif key == "d":
            self.control_angular_velocity = constrain(
                self.control_angular_velocity - ANG_VEL_STEP_SIZE, -3.0, 3.0
            )
        elif key == "1":
            self.control_first, self.control_second, self.control_all = True, False, False
        elif key == "2":
            self.control_first, self.control_second, self.control_all = False, True, False
        elif key == "3":
            self.control_first = True
            self.control_second = True
            self.control_all = True
            self.fixed_l = None
        elif key == "q":
            self.theta_steer += 0.01 * math.pi
        elif key == "e":
            self.theta_steer -= 0.01 * math.pi
        elif key == "z":
            self.theta_steer_velocity += ANG_VEL_STEP_SIZE
        elif key == "c":
            self.theta_steer_velocity -= ANG_VEL_STEP_SIZE
        elif key == " " or key == "s":
            self.stop_all()
        elif key == "\x03":
            rospy.signal_shutdown("User exit")

    def stop_all(self):
        self.control_linear_velocity = 0.0
        self.control_angular_velocity = 0.0
        self.theta_steer_velocity = 0.0

    def update_robots(self):
        for robot in self.robots:
            robot.update_pose_from_tf()

        if self.control_all and len(self.robots) >= 2:
            self.update_midpoint()
            self.update_all_control()
        else:
            self.update_single_control()

    def update_midpoint(self):
        xmid = (self.robots[1].x + self.robots[0].x) / 2.0
        ymid = (self.robots[1].y + self.robots[0].y) / 2.0
        if self.fixed_l is None:
            self.fixed_l = math.hypot(self.robots[1].x - self.robots[0].x, self.robots[1].y - self.robots[0].y)
        for robot in self.robots:
            robot.l = math.hypot(robot.x - xmid, robot.y - ymid)
            robot.pose_theta = math.atan2(robot.y - ymid, robot.x - xmid) - math.atan2(
                self.robots[1].y - self.robots[0].y, self.robots[1].x - self.robots[0].x
            )
        return xmid, ymid

    def update_all_control(self):
        base_angle = math.atan2(self.robots[1].y - self.robots[0].y, self.robots[1].x - self.robots[0].x)
        self.theta_steer = self.theta_steer % (2 * math.pi)
        if self.theta_steer < 0:
            self.theta_steer += 2 * math.pi

        self.calculate_target_velocities(self.control_linear_velocity, self.control_angular_velocity, base_angle)
        for robot in self.robots:
            robot.publish_velocity()

        desired = Twist()
        desired.linear.x = self.control_linear_velocity
        desired.angular.x = base_angle
        desired.angular.y = self.theta_steer
        desired.angular.z = self.control_angular_velocity
        self.desired_pub.publish(desired)

    def calculate_target_velocities(self, control_linear_velocity, control_angular_velocity, base_angle):
        if abs(control_angular_velocity) < 1e-2:
            for robot in self.robots:
                robot.set_target(control_linear_velocity, self.theta_steer, base_angle, control_angular_velocity)
            return

        if abs(control_linear_velocity) < 1e-2:
            # pure rotation about midpoint
            self.robots[0].set_target(
                (self.fixed_l / 2.0) * control_angular_velocity,
                math.pi / 2.0,
                base_angle,
                control_angular_velocity,
            )
            self.robots[1].set_target(
                (self.fixed_l / 2.0) * control_angular_velocity,
                3.0 * math.pi / 2.0,
                base_angle,
                control_angular_velocity,
            )
            return

        # general curved motion
        rho = control_linear_velocity / control_angular_velocity
        psi = self.theta_steer + (math.pi / 2.0 if control_angular_velocity > 0 else -math.pi / 2.0)
        x = rho * math.cos(psi)
        y = rho * math.sin(psi)

        target_theta1 = math.atan2(y, x - self.robots[0].l) - math.pi / 2.0
        target_theta2 = math.atan2(y, x + self.robots[1].l) - math.pi / 2.0

        if abs(math.sin(target_theta2 - target_theta1)) < 1e-6:
            return

        target_vel1 = -control_angular_velocity * (self.fixed_l) / math.sin(target_theta2 - target_theta1) * math.cos(target_theta2)
        target_vel2 = -control_angular_velocity * (self.fixed_l) / math.sin(target_theta2 - target_theta1) * math.cos(target_theta1)

        self.robots[0].set_target(target_vel1, target_theta1, base_angle, control_angular_velocity)
        self.robots[1].set_target(target_vel2, target_theta2, base_angle, control_angular_velocity)

    def update_single_control(self):
        if self.control_first:
            self.robots[0].target_linear_velocity = self.control_linear_velocity
            self.robots[0].target_angular_velocity = self.control_angular_velocity
            self.robots[0].publish_velocity()
        if self.control_second and len(self.robots) > 1:
            self.robots[1].target_linear_velocity = self.control_linear_velocity
            self.robots[1].target_angular_velocity = self.control_angular_velocity
            self.robots[1].publish_velocity()

    @staticmethod
    def help_message():
        return """
BPMP manual control (ROS1)
--------------------------
w/x : linear velocity up/down
a/d : angular velocity up/down
1/2/3 : control robot1, robot2, both (formation)
q/e : steer angle +/- (formation)
z/c : steer rate +/- (formation)
space or s : stop
CTRL-C to quit
"""


def main():
    controller = RobotController()
    controller.run()


if __name__ == "__main__":
    main()
