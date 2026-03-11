import os
import select
import sys
import rclpy
import numpy as np

from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from rclpy.qos import QoSProfile

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

LIN_VEL_STEP_SIZE = 0.02
ANG_VEL_STEP_SIZE = 0.02
MAX_LIN_VEL = 2.0
MAX_ANG_VEL = 2.0

ROBOT_CONFIG = {
    'robot1': {'x':  1.0, 'y': 0.0},
    'robot2': {'x': -1.0, 'y': 0.0},
}


def constrain(value, low, high):
    return max(low, min(value, high))


def wrap_to_pi(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi


class Robot:
    def __init__(self, node, name, x, y):
        self.name = name
        self.node = node
        self.qos = QoSProfile(depth=10)

        self.x = x
        self.y = y
        self.theta = 0.0

        self.target_linear_velocity = 0.0
        self.target_angular_velocity = 0.0
        self.desired_theta = 0.0
        self.K_p = 2.0

        self.cmd_vel_pub = node.create_publisher(
            Twist, f'/multirobot/{name}/cmd_vel', self.qos
        )

        robot_idx = name[-1]
        node.create_subscription(
            Float64,
            f'/robot_{robot_idx}/towing_angle',
            self._angle_callback,
            self.qos,
        )
        node.get_logger().info(
            f'Robot {name}: pos=({x:.3f}, {y:.3f}), '
            f'sub=/robot_{robot_idx}/towing_angle, '
            f'pub=/multirobot/{name}/cmd_vel'
        )

    def _angle_callback(self, msg):
        self.theta = msg.data

    def set_target(self, speed, heading, omega):
        self.target_linear_velocity = speed
        self.desired_theta = heading
        angle_error = wrap_to_pi(heading - self.theta)
        self.target_angular_velocity = self.K_p * angle_error + omega

    def publish_velocity(self):
        twist = Twist()
        twist.linear.x = self.target_linear_velocity
        twist.angular.y = self.desired_theta
        twist.angular.z = self.target_angular_velocity
        self.cmd_vel_pub.publish(twist)


class RobotController:
    def __init__(self):
        self.node = rclpy.create_node('keyboard_control_hw_seminar')
        self.qos = QoSProfile(depth=10)

        self.robots = []
        for name, cfg in ROBOT_CONFIG.items():
            self.robots.append(Robot(self.node, name, cfg['x'], cfg['y']))

        self.desired_pub = self.node.create_publisher(
            Twist, '/desired/cmd_vel', self.qos
        )

        self.control_first = True
        self.control_second = False
        self.control_all = False

        self.control_linear_velocity = 0.0
        self.control_angular_velocity = 0.0
        self.theta_steer = 0.0

    # ── main loop ────────────────────────────────────────────────

    def run(self):
        settings = None
        if os.name != 'nt':
            settings = termios.tcgetattr(sys.stdin)

        try:
            print(self._help_message())
            while rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0.01)
                key = self._get_key(settings)
                self._process_key(key)
                self._update_robots()
        except KeyboardInterrupt:
            pass
        except Exception as e:
            print(e)
        finally:
            self._stop_all()

    # ── keyboard ─────────────────────────────────────────────────

    def _get_key(self, settings):
        if os.name == 'nt':
            return msvcrt.getch().decode('utf-8')
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key

    def _process_key(self, key):
        if key == 'w':
            self.control_linear_velocity = constrain(
                self.control_linear_velocity + LIN_VEL_STEP_SIZE,
                -MAX_LIN_VEL, MAX_LIN_VEL,
            )
            self._print_vels()
        elif key == 'x':
            self.control_linear_velocity = constrain(
                self.control_linear_velocity - LIN_VEL_STEP_SIZE,
                -MAX_LIN_VEL, MAX_LIN_VEL,
            )
            self._print_vels()
        elif key == 'a':
            self.control_angular_velocity = constrain(
                self.control_angular_velocity + ANG_VEL_STEP_SIZE,
                -MAX_ANG_VEL, MAX_ANG_VEL,
            )
            self._print_vels()
        elif key == 'd':
            self.control_angular_velocity = constrain(
                self.control_angular_velocity - ANG_VEL_STEP_SIZE,
                -MAX_ANG_VEL, MAX_ANG_VEL,
            )
            self._print_vels()
        elif key == 'q':
            self.theta_steer += 0.01 * np.pi
            self._print_vels()
        elif key == 'e':
            self.theta_steer -= 0.01 * np.pi
            self._print_vels()
        elif key == '1':
            self.control_first = True
            self.control_second = False
            self.control_all = False
            print('controlling robot 1')
        elif key == '2':
            self.control_first = False
            self.control_second = True
            self.control_all = False
            print('controlling robot 2')
        elif key == '3':
            self.control_first = True
            self.control_second = True
            self.control_all = True
            print('controlling all robots (team)')
        elif key in (' ', 's'):
            self._stop_all()
        elif key == '\x03':
            raise KeyboardInterrupt

    # ── control ──────────────────────────────────────────────────

    def _update_robots(self):
        if self.control_all:
            self._update_team_control()
        else:
            self._update_single_control()

    def _update_team_control(self):
        v = self.control_linear_velocity
        omega = self.control_angular_velocity

        self._calculate_target_velocities(v, omega)

        for robot in self.robots:
            robot.publish_velocity()

        self._publish_desired(v, omega)

    def _calculate_target_velocities(self, v, omega):
        if np.abs(omega) < 1e-6:
            for robot in self.robots:
                robot.set_target(v, self.theta_steer, 0.0)
            return

        # ICR in object frame (works for all sign combinations)
        x_icr = -v * np.sin(self.theta_steer) / omega
        y_icr =  v * np.cos(self.theta_steer) / omega

        for robot in self.robots:
            rx, ry = robot.x, robot.y

            # Rigid-body tangential velocity: v = omega x (pos - ICR)
            vx = -omega * (ry - y_icr)
            vy =  omega * (rx - x_icr)

            speed = np.sqrt(vx ** 2 + vy ** 2)
            heading = np.arctan2(vy, vx)

            robot.set_target(speed, heading, omega)

        # Half-plane flip: reverse robots on the far side of the ICR
        for robot in self.robots:
            dot = x_icr * (robot.x - x_icr) + y_icr * (robot.y - y_icr)
            if dot > 0:
                robot.set_target(
                    -robot.target_linear_velocity,
                    wrap_to_pi(robot.desired_theta + np.pi),
                    omega,
                )

    def _update_single_control(self):
        if self.control_first:
            r = self.robots[0]
            r.target_linear_velocity = self.control_linear_velocity
            r.target_angular_velocity = self.control_angular_velocity
            r.publish_velocity()
        if self.control_second:
            r = self.robots[1]
            r.target_linear_velocity = self.control_linear_velocity
            r.target_angular_velocity = self.control_angular_velocity
            r.publish_velocity()

    # ── publishing ───────────────────────────────────────────────

    def _publish_desired(self, v, omega):
        desired = Twist()
        desired.linear.x = v
        desired.angular.y = self.theta_steer
        desired.angular.z = omega
        self.desired_pub.publish(desired)

    # ── helpers ──────────────────────────────────────────────────

    def _stop_all(self):
        self.control_linear_velocity = 0.0
        self.control_angular_velocity = 0.0
        for robot in self.robots:
            robot.set_target(0.0, robot.desired_theta, 0.0)
            robot.publish_velocity()
        self._print_vels()

    def _print_vels(self):
        print(
            f'\rlin_vel: {self.control_linear_velocity:+.3f}  '
            f'ang_vel: {self.control_angular_velocity:+.3f}  '
            f'steer: {self.theta_steer / np.pi:+.2f}\u03c0',
            end='',
        )
        sys.stdout.flush()

    @staticmethod
    def _help_message():
        return """
Control Your Robots! (Object Frame)
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
q/e : increase/decrease steering angle
1/2 : control robot 1 / robot 2 individually
3   : control all robots as a team

space key, s : force stop

CTRL-C to quit
"""


def main():
    rclpy.init()
    controller = RobotController()
    controller.run()


if __name__ == '__main__':
    main()