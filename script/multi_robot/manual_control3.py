#!/usr/bin/env python3
import math
import os
import select
import sys
import time

import rospy
import tf
from bpmp_tracker.msg import UnicycleInput
from geometry_msgs.msg import Twist


if os.name == "nt":
    import msvcrt
else:
    import termios
    import tty



LIN_VEL_STEP_SIZE = 0.02
ANG_VEL_STEP_SIZE = 0.02
MAX_LIN_VEL = 2.0
MAX_ANG_VEL = 2.0


def constrain(val, lo, hi):
    return max(lo, min(val, hi))


def wrap_to_pi(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


class Robot:
    def __init__(self, index, tf_listener):
        self.index = index
        self.tf_frame = f"robot_{index + 1}_base_link"
        self.listener = tf_listener

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.target_linear_velocity = 0.0
        self.target_angular_velocity = 0.0
        self.desired_theta = 0.0
        self.K_p = 10.0

        self.cmd_pub = rospy.Publisher(
            f"/bpmp_tracker/unicycle_control_input_{index}", UnicycleInput, queue_size=1
        )

    def update_pose_from_tf(self, yaw_only=False):
        try:
            (trans, rot) = self.listener.lookupTransform("object_base_link", self.tf_frame, rospy.Time(0))
            if not yaw_only:
                self.x, self.y = trans[0], trans[1]
            yaw = tf.transformations.euler_from_quaternion(rot)[2]
            self.theta = wrap_to_pi(yaw)
            return True
        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            return False

    def set_target(self, speed, heading, omega):
        self.target_linear_velocity = speed
        self.desired_theta = heading
        angle_error = wrap_to_pi(heading - self.theta)
        self.target_angular_velocity = self.K_p * angle_error + omega

    def publish_velocity(self):
        msg = UnicycleInput()
        msg.vel_linear = self.target_linear_velocity
        msg.vel_angular = self.target_angular_velocity
        self.cmd_pub.publish(msg)


class RobotController:
    def __init__(self):
        rospy.init_node("bpmp_manual_control3", anonymous=False)
        self.listener = tf.TransformListener()

        robot_count = rospy.get_param("~robot_count", 2)
        self.robots = [Robot(i, self.listener) for i in range(robot_count)]
        for robot in self.robots:
            robot.update_pose_from_tf()

        self.desired_pub = rospy.Publisher("/bpmp_manual/desired3", Twist, queue_size=1)

        self.control_first = True
        self.control_second = False
        self.control_all = False

        self.control_linear_velocity = 0.0
        self.control_angular_velocity = 0.0
        self.theta_steer = 0.0

    # main loop
    def run(self):
        settings = None
        if os.name != "nt":
            settings = termios.tcgetattr(sys.stdin)

        rate = rospy.Rate(50.0)
        print(self.help_message())
        while not rospy.is_shutdown():
            try:
                self.listener.waitForTransform("map", self.robots[0].tf_frame, rospy.Time(0), rospy.Duration(0.01))
            except tf.Exception:
                pass
            key = self.get_key(settings)
            self.process_key(key)
            self.update_robots()
            rate.sleep()

    # keyboard
    def get_key(self, settings):
        if os.name == "nt":
            return msvcrt.getch().decode("utf-8")
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        key = sys.stdin.read(1) if rlist else ""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def process_key(self, key):
        if key == "w":
            self.control_linear_velocity = constrain(
                self.control_linear_velocity + LIN_VEL_STEP_SIZE, -MAX_LIN_VEL, MAX_LIN_VEL
            )
        elif key == "x":
            self.control_linear_velocity = constrain(
                self.control_linear_velocity - LIN_VEL_STEP_SIZE, -MAX_LIN_VEL, MAX_LIN_VEL
            )
        elif key == "a":
            self.control_angular_velocity = constrain(
                self.control_angular_velocity + ANG_VEL_STEP_SIZE, -MAX_ANG_VEL, MAX_ANG_VEL
            )
        elif key == "d":
            self.control_angular_velocity = constrain(
                self.control_angular_velocity - ANG_VEL_STEP_SIZE, -MAX_ANG_VEL, MAX_ANG_VEL
            )
        elif key == "q":
            self.theta_steer += 0.01 * math.pi
        elif key == "e":
            self.theta_steer -= 0.01 * math.pi
        elif key == "1":
            self.control_first, self.control_second, self.control_all = True, False, False
        elif key == "2":
            self.control_first, self.control_second, self.control_all = False, True, False
        elif key == "3":
            self.control_first = True
            self.control_second = True
            self.control_all = True
        elif key in (" ", "s"):
            self.stop_all()
        elif key == "\x03":
            rospy.signal_shutdown("User exit")

    # control
    def update_robots(self):
        for robot in self.robots:
            robot.update_pose_from_tf(yaw_only=True)

        if self.control_all and len(self.robots) >= 2:
            self.update_team_control()
        else:
            self.update_single_control()

    def update_team_control(self):
        # print("team control")
        v = self.control_linear_velocity
        omega = self.control_angular_velocity

        self.calculate_target_velocities(v, omega)

        for robot in self.robots:
            robot.publish_velocity()

        self.publish_desired(v, omega)

    def calculate_target_velocities(self, v, omega):
        if abs(omega) < 1e-6:
            for robot in self.robots:
                robot.set_target(v, self.theta_steer, 0.0)
            return

        # ICR in object frame (works for all sign combinations)
        x_icr = -v * math.sin(self.theta_steer) / omega
        y_icr =  v * math.cos(self.theta_steer) / omega

        for robot in self.robots:
            rx, ry = robot.x, robot.y

            # Rigid-body tangential velocity: v = omega x (pos - ICR)
            vx = -omega * (ry - y_icr)
            vy =  omega * (rx - x_icr)

            speed = math.hypot(vx, vy)
            heading = math.atan2(vy, vx)

            robot.set_target(speed, heading, omega)

        # Half-plane flip: reverse robots on the far side of the ICR
        for robot in self.robots:
            dot = x_icr * (robot.x - x_icr) + y_icr * (robot.y - y_icr)
            if dot > 0:
                robot.set_target(
                    -robot.target_linear_velocity,
                    wrap_to_pi(robot.desired_theta + math.pi),
                    omega,
                )

    def update_single_control(self):
        if self.control_first and len(self.robots) > 0:
            r = self.robots[0]
            r.target_linear_velocity = self.control_linear_velocity
            r.target_angular_velocity = self.control_angular_velocity
            r.publish_velocity()
        if self.control_second and len(self.robots) > 1:
            r = self.robots[1]
            r.target_linear_velocity = self.control_linear_velocity
            r.target_angular_velocity = self.control_angular_velocity
            r.publish_velocity()

    # publish helpers
    def publish_desired(self, v, omega):
        desired = Twist()
        desired.linear.x = v
        desired.angular.y = self.theta_steer
        desired.angular.z = omega
        self.desired_pub.publish(desired)

    def stop_all(self):
        self.control_linear_velocity = 0.0
        self.control_angular_velocity = 0.0
        for robot in self.robots:
            robot.set_target(0.0, robot.desired_theta, 0.0)
            robot.publish_velocity()

    @staticmethod
    def help_message():
        return """
BPMP manual control 3 (ROS1, TF-driven)
--------------------------------------
w/x : linear velocity up/down
a/d : angular velocity up/down
q/e : steering angle +/- (formation)
1/2/3 : control robot1, robot2, both
space or s : stop
CTRL-C to quit
"""


def main():
    controller = RobotController()
    controller.run()


if __name__ == "__main__":
    main()
