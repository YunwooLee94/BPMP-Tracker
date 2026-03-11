import os
import select
import sys
import rclpy
import time
import threading
import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from std_msgs.msg import Int16, Bool
from rclpy.qos import QoSProfile
import tf2_ros
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from transforms3d.euler import quat2euler
from rclpy.duration import Duration
from rclpy.time import Time

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

LIN_VEL_STEP_SIZE = 0.02
ANG_VEL_STEP_SIZE = 0.02

class Robot:
    def __init__(self, node, name, is_real=False, theta_offset=0.0):
        self.name = name
        self.is_real = is_real
        self.node = node
        self.qos = QoSProfile(depth=10)

        # Initialize state variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.theta_offset = theta_offset # offset for the robot's theta

        # Object frame coordinates (initialized when all robots are created)
        self.x_obj = 0.0
        self.y_obj = 0.0

        self.target_linear_velocity = 0.0
        self.target_angular_velocity = 0.0
        self.target_theta = 0.0
        self.desired_theta = 0.0
        self.K_p = 2 # proportional gain for angle

        self.l = 0.0    # distance to the midpoint
        self.pose_theta = 0.0    # angle to the midpoint

        # Create publishers
        self.cmd_vel_pub = node.create_publisher(Twist, f'/multirobot/{name}/cmd_vel', self.qos)
        if is_real:
            self.cw_pub = node.create_publisher(Bool, f'/robot{name[-1]}/gpio_output_27', self.qos)
            self.pwm_pub = node.create_publisher(Int16, f'/robot{name[-1]}/gpio_pwm_17', self.qos)
        
        # TF frame name for this robot
        self.tf_frame = f"robot{name[-1]}_base_2"
        
    def update_pose_from_tf(self, tf_buffer):
        """Update pose from TF buffer (called by controller)"""
        try:
            # Get transform from odom to robotX_base_2
            transform = tf_buffer.lookup_transform(
                'odom', 
                self.tf_frame, 
                rclpy.time.Time()  # This gets transform at current time
            )
            
            # Extract position
            self.x = transform.transform.translation.x
            self.y = transform.transform.translation.y
            
            # Extract orientation and convert to euler angles
            quat = transform.transform.rotation
            # Convert quaternion to euler angles (roll, pitch, yaw)
            euler = quat2euler([quat.w, quat.x, quat.y, quat.z], 'sxyz')
            self.theta = euler[2] + np.pi + self.theta_offset  # yaw + pi + offset to match original orientation
            
            return True
        except tf2_ros.LookupException as e:
            # TF frame doesn't exist yet
            return False
        except tf2_ros.ConnectivityException as e:
            # TF tree is not connected
            return False
        except tf2_ros.ExtrapolationException as e:
            # Time-related issues - this is expected in simulation
            return False
        except Exception as e:
            # Other unexpected errors
            self.node.get_logger().warn(f'Unexpected error getting transform for {self.tf_frame}: {str(e)}')
            return False

    def initialize_object_frame(self, xmid, ymid, base_angle):
        """Initialize object frame coordinates based on current position and base_angle."""
        # Convert to object frame (relative to midpoint of first two robots)
        x_rel = self.x - xmid
        y_rel = self.y - ymid
        # Rotate by base_angle to get object frame coordinates
        self.x_obj = -x_rel * np.cos(-base_angle) + y_rel * np.sin(-base_angle)
        self.y_obj = -x_rel * np.sin(-base_angle) - y_rel * np.cos(-base_angle)

    def set_target(self, linear_vel, target_theta, base_angle, control_angular_velocity):
        self.target_linear_velocity = linear_vel
        self.target_theta = target_theta + base_angle + 0.5 * control_angular_velocity
        self.desired_theta = target_theta + base_angle
        angle_diff = (self.target_theta - self.theta) % (2 * np.pi)
        if angle_diff < 0:
            angle_diff += 2 * np.pi
        if angle_diff > np.pi:
            angle_diff -= 2 * np.pi
        self.target_angular_velocity = self.K_p * angle_diff

    def publish_velocity(self):
        twist = Twist()
        twist.linear.x = self.target_linear_velocity
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = self.desired_theta
        twist.angular.z = self.target_angular_velocity
        self.cmd_vel_pub.publish(twist)

    def publish_motor_control(self, cw_flag, pwm):
        if self.is_real:
            cw_msg = Bool()
            cw_msg.data = cw_flag
            self.cw_pub.publish(cw_msg)

            pwm_msg = Int16()
            pwm_msg.data = pwm
            self.pwm_pub.publish(pwm_msg)

class RobotController:
    def __init__(self):
        self.node = rclpy.create_node('isaacsim_multirobot')
        self.qos = QoSProfile(depth=10)
        
        # Set up simulation time handling
        # Check if we should use simulation time
        self.use_sim_time = self.node.get_parameter('use_sim_time').get_parameter_value().bool_value
        if self.use_sim_time:
            self.node.get_logger().info('Using simulation time')
        else:
            self.node.get_logger().info('Using real time')
        
        # Create single TF buffer and listener for the entire controller
        # Increase buffer size to handle heavy simulation load
        self.tf_buffer = Buffer(cache_time=Duration(seconds=60.0))  # 60 second buffer
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        
        # Load robot configurations from config.yaml using ROS2 package system
        try:
            isaacsim_multirobot_share_dir = get_package_share_directory('isaacsim_multirobot')
            config_path = os.path.join(isaacsim_multirobot_share_dir, 'config', 'config.yaml')
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            self.node.get_logger().info(f'Loaded config from: {config_path}')
        except Exception as e:
            self.node.get_logger().error(f'Failed to load config: {str(e)}')
            # Fallback to default configuration
            config = {
                'robots': {
                    'robot1': {'x': 1.068, 'y': 0.0, 'theta': 0.0, 'theta_offset': 0.0},
                    'robot2': {'x': -1.068, 'y': 0.0, 'theta': 0.0, 'theta_offset': np.pi},
                    'robot3': {'x': 0.2, 'y': -0.743, 'theta': 0.0, 'theta_offset': -np.pi/2},
                    'robot4': {'x': 0.2, 'y': 0.743, 'theta': 0.0, 'theta_offset': np.pi/2}
                }
            }#theta_offset is the offset for the robot's theta -> the model was created with every robot's theta = 0.0
            self.node.get_logger().info('Using default configuration')
        
        # Create robots list
        self.robots = []
        # Add robots based on config
        for robot_name in config['robots'].keys():
            # is_real = robot_name in ['robot1', 'robot2', 'robot3', 'robot4']  # First two robots are real robots
            self.robots.append(Robot(self.node, robot_name, False, config['robots'][robot_name]['theta_offset']))
            self.node.get_logger().info(f'Created robot: {robot_name} (real: False)')
        
        # Create midpoint publisher
        from turtlesim.msg import Pose
        self.midpoint_pub = self.node.create_publisher(Pose, '/mid_point/pose', self.qos)
        self.desired_pub = self.node.create_publisher(Twist, '/desired/cmd_vel', self.qos)
        
        # Control state
        self.control_first = True
        self.control_second = False
        self.control_third = False
        self.control_fourth = False
        self.control_all = False
        self.control_linear_velocity = 0.0
        self.control_angular_velocity = 0.0
        self.theta_steer = 0.0
        
        # Initialize state tracking variables
        self.fixed_l = None  # Fixed l value when '3' is pressed
        self.last_base_angle = 0.0
        
        # Motor control
        self.height_count = 0
        self.height_change_running = False
        self.motor_status = "stop"
        self.pwm = 0
        self.cw_flag = True

    def spin_tf(self):
        """Separate thread for TF processing"""
        while rclpy.ok():
            try:
                rclpy.spin_once(self.node, timeout_sec=0.01)
            except Exception as e:
                self.node.get_logger().error(f'TF thread error: {str(e)}')

    def start_tf_thread(self):
        """Start the TF processing thread"""
        self.tf_thread = threading.Thread(target=self.spin_tf, daemon=True)
        self.tf_thread.start()
        self.node.get_logger().info('TF processing thread started')

    def get_key(self, settings):
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

    def change_height(self):
        while self.height_change_running:
            time.sleep(0.05)
            if self.motor_status == "cw" and self.height_count < 255:
                self.height_count += 1
            elif self.motor_status == "ccw" and self.height_count > 0:
                self.height_count -= 1
            elif self.height_count >= 255 or self.height_count <= 0:
                self.height_change_running = False
                self.pwm = 0
            
            bar_length = 50
            filled_length = int(bar_length * self.height_count / 255)
            bar = '█' * filled_length + '-' * (bar_length - filled_length)
            print(f'\rHeight Count: [{bar}] {self.height_count}/255', end='')
            sys.stdout.flush()

    def print_vels(self):
        print('\ncurrently:\tlinear velocity {0}\t angular velocity {1}\t motor command {2}'.format(
            self.control_linear_velocity,
            self.control_angular_velocity,
            self.motor_status))
        sys.stdout.flush()

    def run(self):
        settings = None
        if os.name != 'nt':
            settings = termios.tcgetattr(sys.stdin)

        try:
            print(self.get_help_message())
            print("TF processing thread started - TF will be processed in background")
            
            while rclpy.ok():
                key = self.get_key(settings)
                self.process_key(key)
                self.update_robots()
                # Don't call rclpy.spin_once here - TF thread handles it

        except Exception as e:
            print(e)
        finally:
            self.stop_all()

    def get_help_message(self):
        return """
Control Your Robots!
---------------------------
Moving around:                     Motors:
        w                               i
   a    s    d                          k
        x                               m

w/x : increase/decrease linear velocity
a/d : increase/decrease angular velocity
i/k/m : move lift upwards/stop/downwards
1/2/3 : controlling 1st robot, 2nd robot, all

space key, s : force stop

CTRL-C to quit
"""

    def process_key(self, key):
        if key == 'w':
            self.control_linear_velocity = self.check_linear_limit_velocity(
                self.control_linear_velocity + LIN_VEL_STEP_SIZE)
            self.print_vels()
        elif key == 'x':
            self.control_linear_velocity = self.check_linear_limit_velocity(
                self.control_linear_velocity - LIN_VEL_STEP_SIZE)
            self.print_vels()
        elif key == 'a':
            self.control_angular_velocity = self.check_angular_limit_velocity(
                self.control_angular_velocity + ANG_VEL_STEP_SIZE)
            self.print_vels()
        elif key == 'd':
            self.control_angular_velocity = self.check_angular_limit_velocity(
                self.control_angular_velocity - ANG_VEL_STEP_SIZE)
            self.print_vels()
        elif key == '1':
            self.control_first = True
            self.control_second = False
            self.control_all = False
            print('controlling first robot')
        elif key == '2':
            self.control_first = False
            self.control_second = True
            self.control_all = False
            print('controlling second robot')
        elif key == '3':
            self.control_first = True
            self.control_second = True
            self.control_all = True
            # Reset fixed_l to force recalculation
            self.fixed_l = None
            xmid, ymid = self.update_midpoint()
            base_angle = np.arctan2(self.robots[1].y - self.robots[0].y, 
                                  self.robots[1].x - self.robots[0].x)
            # Initialize object frame coordinates for all robots
            for robot in self.robots:
                robot.initialize_object_frame(xmid, ymid, base_angle)
            for i in range(len(self.robots)):
                self.robots[i].x_rel = self.robots[i].x - xmid
                self.robots[i].y_rel = self.robots[i].y - ymid
                self.robots[i].pose_theta = np.arctan2(self.robots[i].y - ymid, 
                                                    self.robots[i].x - xmid) - \
                                        np.arctan2(self.robots[1].y - self.robots[0].y, 
                                                    self.robots[1].x - self.robots[0].x)
        elif key == 'q':
            self.theta_steer += 0.01 * np.pi
        elif key == 'e':
            self.theta_steer -= 0.01 * np.pi
        elif key == ' ' or key == 's':
            self.stop_all()
        elif key == 'i':
            self.handle_motor_up()
        elif key == 'k':
            self.handle_motor_stop()
        elif key == 'm':
            self.handle_motor_down()
        elif key == '\x03':
            raise KeyboardInterrupt

    def update_midpoint(self):
        # Calculate midpoint between first two robots
        xmid = (self.robots[1].x + self.robots[0].x) / 2
        ymid = (self.robots[1].y + self.robots[0].y) / 2
        
        # Only calculate l if it hasn't been fixed yet
        if self.fixed_l is None:
            self.fixed_l = np.linalg.norm(np.array([self.robots[1].x - self.robots[0].x, 
                                                  self.robots[1].y - self.robots[0].y]))
            # Calculate relative positions for all robots
        for i in range(len(self.robots)):
            self.robots[i].l = np.linalg.norm(np.array([self.robots[i].x - xmid, 
                                                        self.robots[i].y - ymid]))
            self.robots[i].pose_theta = np.arctan2(self.robots[i].y - ymid, 
                                                    self.robots[i].x - xmid) - \
                                        np.arctan2(self.robots[1].y - self.robots[0].y, 
                                                    self.robots[1].x - self.robots[0].x)
            
            if i >= 2:  # For additional robots
                print(f'Robot {i+1} - l{i+1}={self.robots[i].l:.2f}, pose_theta{i+1}={self.robots[i].pose_theta/np.pi:.2f}pi')
        
            #print('controlling all robots, l =', self.fixed_l)
        return xmid, ymid

    def update_robots(self):
        # Update poses for all robots using the shared TF buffer
        for robot in self.robots:
            robot.update_pose_from_tf(self.tf_buffer)
        
        if self.control_all:
            _,_=self.update_midpoint()
            self.update_all_control()
        else:
            self.update_single_control()

    def update_all_control(self):
        # Calculate base angle and velocities
        base_angle = np.arctan2(self.robots[1].y - self.robots[0].y, 
                              self.robots[1].x - self.robots[0].x)
        
        # Normalize theta_steer
        self.theta_steer = self.theta_steer % (2 * np.pi)
        while self.theta_steer < 0:
            self.theta_steer += 2 * np.pi
            
        print(f'theta_steer = {self.theta_steer/np.pi:.2f}pi')
        
        # Calculate and publish velocities
        self.calculate_target_velocities(
            self.control_linear_velocity,
            self.control_angular_velocity,
            base_angle
        )
        
        # Publish velocities
        for robot in self.robots:
            robot.publish_velocity()
        
        # Publish midpoint and desired velocities
        self.publish_midpoint_and_desired(
            self.control_linear_velocity,
            self.control_angular_velocity,
            base_angle
        )

    def calculate_target_velocities(self, control_linear_velocity, control_angular_velocity, base_angle):
        if np.abs(control_angular_velocity) < 1e-2:
            # Straight line motion
            for i in range(len(self.robots)):
                self.robots[i].set_target(control_linear_velocity, self.theta_steer, base_angle, control_angular_velocity)
                
        elif np.abs(control_linear_velocity) < 1e-2 and np.abs(control_angular_velocity) > 1e-2:
            # Pure rotation
            if control_angular_velocity > 0:
                self.robots[0].set_target(self.fixed_l / 2 * control_angular_velocity, np.pi/2, base_angle, control_angular_velocity)
                self.robots[1].set_target(self.fixed_l / 2 * control_angular_velocity, 3*np.pi/2, base_angle, control_angular_velocity)
                for i in range(2, len(self.robots)):
                    self.robots[i].set_target(
                        self.robots[i].l * control_angular_velocity,
                        self.robots[i].pose_theta - np.pi/2,
                        base_angle,
                        control_angular_velocity
                    )
            else:
                self.robots[0].set_target(-self.fixed_l / 2 * control_angular_velocity, 3*np.pi/2, base_angle, control_angular_velocity)
                self.robots[1].set_target(-self.fixed_l / 2 * control_angular_velocity, np.pi/2, base_angle, control_angular_velocity)
                for i in range(2, len(self.robots)):
                    self.robots[i].set_target(
                        -self.robots[i].l * control_angular_velocity,
                        self.robots[i].pose_theta + np.pi/2,
                        base_angle,
                        control_angular_velocity
                    )
        else:
            print('control_angular_velocity =', control_angular_velocity)
            
            # Calculate rotation center
            if ((control_angular_velocity < 0 and ((self.theta_steer >= 0 and self.theta_steer < np.pi/2) or (self.theta_steer <= 2*np.pi and self.theta_steer > 3/2*np.pi)) and control_linear_velocity > 0) or\
                (control_angular_velocity > 0 and ((self.theta_steer >= 0 and self.theta_steer < np.pi/2) or (self.theta_steer <= 2*np.pi and self.theta_steer > 3/2*np.pi)) and control_linear_velocity < 0)): #R_D
                print('R_D')#OK
                rho = -control_linear_velocity / control_angular_velocity
                psi = self.theta_steer - np.pi/2
                x = rho * np.cos(psi)
                y = rho * np.sin(psi)
                
                target_theta1 = np.arctan2(y, x - self.robots[0].l) + np.pi/2
                target_theta2 = np.arctan2(y, x + self.robots[1].l) + 3*np.pi/2
                target_vel1 = -control_angular_velocity * (self.fixed_l) / np.sin(target_theta2 - target_theta1) * np.cos(target_theta2)
                target_vel2 = control_angular_velocity * (self.fixed_l) / np.sin(target_theta2 - target_theta1) * np.cos(target_theta1)
                target_theta2 += np.pi
                
                self.robots[0].set_target(target_vel1, target_theta1, base_angle, control_angular_velocity)
                self.robots[1].set_target(target_vel2, target_theta2, base_angle, control_angular_velocity)
                
                for i in range(2, len(self.robots)):
                    ri = np.linalg.norm(np.array([
                        self.robots[i].l * np.cos(self.robots[i].pose_theta) + x,
                        self.robots[i].l * np.sin(self.robots[i].pose_theta) + y
                    ]))
                    target_thetai = np.arctan2(
                        self.robots[i].l * np.sin(self.robots[i].pose_theta) + y,
                        self.robots[i].l * np.cos(self.robots[i].pose_theta) + x
                    ) + np.pi/2
                    target_veli = -ri * control_angular_velocity
                    self.robots[i].set_target(target_veli, target_thetai, base_angle, control_angular_velocity)

            elif ((control_angular_velocity > 0 and (self.theta_steer > np.pi/2 and self.theta_steer < 3/2 * np.pi) and control_linear_velocity > 0) or\
                (control_angular_velocity < 0 and (self.theta_steer > np.pi/2 and self.theta_steer < 3/2 * np.pi) and control_linear_velocity < 0)): #L_D
                print('L_D')
                rho = control_linear_velocity / control_angular_velocity
                psi = self.theta_steer + np.pi/2
                x = rho * np.cos(psi)
                y = rho * np.sin(psi)
                
                target_theta1 = np.arctan2(y, x - self.robots[0].l) - np.pi/2
                target_theta2 = np.arctan2(y, x + self.robots[1].l) - np.pi/2
                target_vel1 = -control_angular_velocity * (self.fixed_l) / np.sin(target_theta2 - target_theta1) * np.cos(target_theta2)
                target_vel2 = -control_angular_velocity * (self.fixed_l) / np.sin(target_theta2 - target_theta1) * np.cos(target_theta1)
                
                self.robots[0].set_target(target_vel1, target_theta1, base_angle, control_angular_velocity)
                self.robots[1].set_target(target_vel2, target_theta2, base_angle, control_angular_velocity)
                
                for i in range(2, len(self.robots)):
                    ri = np.linalg.norm(np.array([
                        self.robots[i].l * np.cos(self.robots[i].pose_theta) + x,
                        self.robots[i].l * np.sin(self.robots[i].pose_theta) + y
                    ]))
                    target_thetai = np.arctan2(
                        self.robots[i].l * np.sin(self.robots[i].pose_theta) + y,
                        self.robots[i].l * np.cos(self.robots[i].pose_theta) + x
                    ) - np.pi/2
                    target_veli = ri * control_angular_velocity
                    self.robots[i].set_target(target_veli, target_thetai, base_angle, control_angular_velocity)

            elif ((control_angular_velocity > 0 and ((self.theta_steer >= 0 and self.theta_steer < np.pi/2) or (self.theta_steer <= 2*np.pi and self.theta_steer > 3/2*np.pi)) and control_linear_velocity > 0) or\
                (control_angular_velocity < 0 and ((self.theta_steer >= 0 and self.theta_steer < np.pi/2) or (self.theta_steer <= 2*np.pi and self.theta_steer > 3/2*np.pi)) and control_linear_velocity < 0)): #L_U
                print('L_U')#OK
                rho = control_linear_velocity / control_angular_velocity
                psi = self.theta_steer + np.pi/2
                x = rho * np.cos(psi)
                y = rho * np.sin(psi)
                
                target_theta1 = np.arctan2(y, x - self.robots[0].l) - np.pi/2
                target_theta2 = np.arctan2(y, x + self.robots[1].l) - np.pi/2
                target_vel1 = -control_angular_velocity * (self.fixed_l) / np.sin(target_theta2 - target_theta1) * np.cos(target_theta2)
                target_vel2 = -control_angular_velocity * (self.fixed_l) / np.sin(target_theta2 - target_theta1) * np.cos(target_theta1)
                
                self.robots[0].set_target(target_vel1, target_theta1, base_angle, control_angular_velocity)
                self.robots[1].set_target(target_vel2, target_theta2, base_angle, control_angular_velocity)
                
                for i in range(2, len(self.robots)):
                    ri = np.linalg.norm(np.array([
                        self.robots[i].l * np.cos(self.robots[i].pose_theta) + x,
                        self.robots[i].l * np.sin(self.robots[i].pose_theta) + y
                    ]))
                    target_thetai = np.arctan2(
                        self.robots[i].l * np.sin(self.robots[i].pose_theta) + y,
                        self.robots[i].l * np.cos(self.robots[i].pose_theta) + x
                    ) - np.pi/2
                    target_veli = ri * control_angular_velocity
                    self.robots[i].set_target(target_veli, target_thetai, base_angle, control_angular_velocity)

            elif ((control_angular_velocity > 0 and (self.theta_steer > np.pi/2 and self.theta_steer < 3/2 * np.pi) and control_linear_velocity < 0) or\
                (control_angular_velocity < 0 and (self.theta_steer > np.pi/2 and self.theta_steer < 3/2 * np.pi) and control_linear_velocity > 0)): #R_U
                print('R_U')
                rho = -control_linear_velocity / control_angular_velocity
                psi = self.theta_steer - np.pi/2
                x = rho * np.cos(psi)
                y = rho * np.sin(psi)
                
                target_theta1 = np.arctan2(y, x - self.robots[0].l) + np.pi/2
                target_theta2 = np.arctan2(y, x + self.robots[1].l) + np.pi/2
                target_vel1 = -control_angular_velocity * (self.fixed_l) / np.sin(target_theta2 - target_theta1) * np.cos(target_theta2)
                target_vel2 = -control_angular_velocity * (self.fixed_l) / np.sin(target_theta2 - target_theta1) * np.cos(target_theta1)
                
                self.robots[0].set_target(target_vel1, target_theta1, base_angle, control_angular_velocity)
                self.robots[1].set_target(target_vel2, target_theta2, base_angle, control_angular_velocity)
                
                for i in range(2, len(self.robots)):
                    ri = np.linalg.norm(np.array([
                        self.robots[i].l * np.cos(self.robots[i].pose_theta) + x,
                        self.robots[i].l * np.sin(self.robots[i].pose_theta) + y
                    ]))
                    target_thetai = np.arctan2(
                        self.robots[i].l * np.sin(self.robots[i].pose_theta) + y,
                        self.robots[i].l * np.cos(self.robots[i].pose_theta) + x
                    ) + np.pi/2
                    target_veli = -ri * control_angular_velocity
                    self.robots[i].set_target(target_veli, target_thetai, base_angle, control_angular_velocity)
            
            # Check if rotation center is inside polytope and adjust velocities if needed
            if np.abs(control_angular_velocity) > 1e-2:  # Only check for rotation cases
                # is_inside = self.is_rotation_center_inside_polytope(x, y, base_angle)
                # if is_inside:
                for i in range(len(self.robots)):
                    # Calculate dot product between normal vector (x,y) and vector from (x,y) to robot
                    # If dot product is positive, robot is in the half-plane we want
                    dot_product = x * (self.robots[i].x_obj - x) + y * (self.robots[i].y_obj - y)
                    # print(f"Dot product for robot {i+1}: {dot_product:.2f}")
                    if dot_product > 0:  # Robot is in the desired half-plane
                        # print(f"Robot {i+1} is in the desired half-plane")
                        # Get current target values
                        current_target = self.robots[i].target_theta    
                        current_vel = self.robots[i].target_linear_velocity
                        
                        # Update target values
                        self.robots[i].set_target(-current_vel, current_target + np.pi, 0, control_angular_velocity)

    def publish_midpoint_and_desired(self, lin_vel, ang_vel, base_angle):
        # Publish midpoint pose
        from turtlesim.msg import Pose
        midpoint = Pose()
        midpoint.x = (self.robots[1].x + self.robots[0].x) / 2
        midpoint.y = (self.robots[1].y + self.robots[0].y) / 2
        midpoint.theta = self.theta_steer
        midpoint.linear_velocity = lin_vel
        midpoint.angular_velocity = ang_vel
        self.midpoint_pub.publish(midpoint)
        
        # Publish desired velocities
        desired = Twist()
        desired.linear.x = lin_vel
        desired.linear.y = 0.0
        desired.linear.z = 0.0
        desired.angular.x = base_angle
        desired.angular.y = self.theta_steer
        desired.angular.z = ang_vel
        self.desired_pub.publish(desired)

    def update_single_control(self):
        if self.control_first:
            self.robots[0].target_linear_velocity = self.control_linear_velocity
            self.robots[0].target_angular_velocity = self.control_angular_velocity
            self.robots[0].publish_velocity()
            self.robots[0].publish_motor_control(self.cw_flag, self.pwm)
        if self.control_second:
            self.robots[1].target_linear_velocity = self.control_linear_velocity
            self.robots[1].target_angular_velocity = self.control_angular_velocity
            self.robots[1].publish_velocity()
            self.robots[1].publish_motor_control(self.cw_flag, self.pwm)

    def stop_all(self):
        self.control_linear_velocity = 0.0
        self.control_angular_velocity = 0.0
        self.pwm = 0
        self.motor_status = "stop"
        self.height_change_running = False
        self.print_vels()

    def handle_motor_up(self):
        if self.height_count < 255:
            self.cw_flag = True
            self.pwm = 128
            self.motor_status = "cw"
            self.height_change_running = True
            threading.Thread(target=self.change_height).start()
            self.print_vels()
        else:
            self.pwm = 0
            self.motor_status = "stop"
            self.print_vels()
            print("!!!height upper limit reached!!!")

    def handle_motor_stop(self):
        self.pwm = 0
        self.motor_status = "stop"
        self.height_change_running = False
        self.print_vels()

    def handle_motor_down(self):
        if self.height_count > 0:
            self.cw_flag = False
            self.pwm = 128
            self.motor_status = "ccw"
            self.height_change_running = True
            threading.Thread(target=self.change_height).start()
            self.print_vels()
        else:
            self.pwm = 0
            self.motor_status = "stop"
            self.print_vels()
            print("!!!height lower limit reached!!!")

    @staticmethod
    def check_linear_limit_velocity(velocity):
        # Using reasonable limits for robots
        MAX_LIN_VEL = 2.0
        return constrain(velocity, -MAX_LIN_VEL, MAX_LIN_VEL)

    @staticmethod
    def check_angular_limit_velocity(velocity):
        # Using reasonable limits for robots
        MAX_ANG_VEL = 2.0
        return constrain(velocity, -MAX_ANG_VEL, MAX_ANG_VEL)


def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    return input_vel

def main():
    rclpy.init()
    controller = RobotController()
    controller.start_tf_thread()  # Start the TF processing thread
    controller.run()  # Use the existing run method

if __name__ == '__main__':
    main()
        