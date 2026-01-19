#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PolyTraj.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <cmath>

#include <traj_opt/poly_traj_utils.hpp>
#include <bpmp_tracker/UnicycleInput.h>

ros::Publisher pos_cmd_pub_;
ros::Publisher cmd_vel_pub_;
// ros::Time heartbeat_time_;
bool receive_traj_ = false;
bool flight_start_ = false;
quadrotor_msgs::PolyTraj trajMsg_, trajMsg_last_;
// Eigen::Vector3d last_p_;
double last_yaw_ = 0;
std::vector<double> cur_position(3, 0.0);
constexpr double kYawMaxStep = 0.02;   // rad per command
constexpr double kCmdDt = 0.01;        // matches cmd_timer
constexpr double kYawRateScale = 100;  // 1 / kCmdDt
double wmax_ = 3.0;  // default max angular velocity

void publish_cmd(int traj_id,
                 const Eigen::Vector3d &p,
                 const Eigen::Vector3d &v,
                 const Eigen::Vector3d &a,
                 double y, double yd) {
  quadrotor_msgs::PositionCommand cmd;
  cmd.header.stamp = ros::Time::now();
  cmd.header.frame_id = "world";
  cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
  cmd.trajectory_id = traj_id;

  cmd.position.x = p(0);
  cmd.position.y = p(1);
  cmd.position.z = p(2);
  cmd.velocity.x = v(0);
  cmd.velocity.y = v(1);
  cmd.velocity.z = v(2);
  cmd.acceleration.x = a(0);
  cmd.acceleration.y = a(1);
  cmd.acceleration.z = a(2);
  cmd.yaw = y;
  cmd.yaw_dot = yd;
  pos_cmd_pub_.publish(cmd);
  // last_p_ = p;

  // Also publish cmd_vel for two wheeled robot simulation
  bpmp_tracker::UnicycleInput cmd_vel;
  // project velocity onto robot heading
  // Eigen::Vector3d heading_vec(cos(y), sin(y), 0);
  Eigen::Vector3d heading_vec(cos(cur_position[2]), sin(cur_position[2]), 0);

  Eigen::Vector3d v_proj = heading_vec * v.dot(heading_vec);
  Eigen::Vector3d a_proj = heading_vec * a.dot(heading_vec);
  Eigen::Vector3d v_cmd = v_proj + a_proj * kCmdDt;
  cmd_vel.vel_linear = std::sqrt(v_cmd(0) * v_cmd(0) + v_cmd(1) * v_cmd(1));
  // angular velocity from lateral acceleration
  if (v_proj.squaredNorm() < 1e-6) {
    // std::cout << "[traj_server] v_proj too small, using desired yaw rate." << std::endl;
    cmd_vel.vel_angular = yd;
  } else{
    // std::cout << "[traj_server] v_proj: " << v_proj.transpose() << ", a_proj: " << a_proj.transpose() << std::endl;
    cmd_vel.vel_angular = (a(1) * heading_vec(0) - a(0) * heading_vec(1)) / (v_proj.squaredNorm() + 1e-6);
  }
  cmd_vel.vel_angular = std::max(std::min(cmd_vel.vel_angular, wmax_), -wmax_);
  cmd_vel_pub_.publish(cmd_vel);
}

double update_yaw(double desired_yaw, double &yaw_dot) {
  double d_yaw = desired_yaw - last_yaw_;
  d_yaw = d_yaw >= M_PI ? d_yaw - 2 * M_PI : d_yaw;
  d_yaw = d_yaw <= -M_PI ? d_yaw + 2 * M_PI : d_yaw;
  if (d_yaw > kYawMaxStep) {
    d_yaw = kYawMaxStep;
  } else if (d_yaw < -kYawMaxStep) {
    d_yaw = -kYawMaxStep;
  }
  const double yaw = last_yaw_ + d_yaw;
  yaw_dot = d_yaw * kYawRateScale;
  last_yaw_ = yaw;
  return yaw;
}

bool exe_traj(const quadrotor_msgs::PolyTraj &trajMsg) {
  double t = (ros::Time::now() - trajMsg.start_time).toSec();
  if (t > 0) {
    if (trajMsg.hover) {
      if (trajMsg.hover_p.size() != 3) {
        ROS_ERROR("[traj_server] hover_p is not 3d!");
      }
      Eigen::Vector3d p, v0;
      p.x() = trajMsg.hover_p[0];
      p.y() = trajMsg.hover_p[1];
      p.z() = trajMsg.hover_p[2];
      v0.setZero();
      double yaw_rate = 0.0;
      double yaw = update_yaw(last_yaw_, yaw_rate);
      publish_cmd(trajMsg.traj_id, p, v0, v0, yaw, yaw_rate);
      return true;
    }
    if (trajMsg.order != 5) {
      ROS_ERROR("[traj_server] Only support trajectory order equals 5 now!");
      return false;
    }
    if (trajMsg.duration.size() * (trajMsg.order + 1) != trajMsg.coef_x.size()) {
      ROS_ERROR("[traj_server] WRONG trajectory parameters!");
      return false;
    }
    int piece_nums = trajMsg.duration.size();
    std::vector<double> dura(piece_nums);
    std::vector<CoefficientMat> cMats(piece_nums);
    for (int i = 0; i < piece_nums; ++i) {
      int i6 = i * 6;
      cMats[i].row(0) << trajMsg.coef_x[i6 + 0], trajMsg.coef_x[i6 + 1], trajMsg.coef_x[i6 + 2],
          trajMsg.coef_x[i6 + 3], trajMsg.coef_x[i6 + 4], trajMsg.coef_x[i6 + 5];
      cMats[i].row(1) << trajMsg.coef_y[i6 + 0], trajMsg.coef_y[i6 + 1], trajMsg.coef_y[i6 + 2],
          trajMsg.coef_y[i6 + 3], trajMsg.coef_y[i6 + 4], trajMsg.coef_y[i6 + 5];
      cMats[i].row(2) << trajMsg.coef_z[i6 + 0], trajMsg.coef_z[i6 + 1], trajMsg.coef_z[i6 + 2],
          trajMsg.coef_z[i6 + 3], trajMsg.coef_z[i6 + 4], trajMsg.coef_z[i6 + 5];

      dura[i] = trajMsg.duration[i];
    }
    Trajectory traj(dura, cMats);
    if (t > traj.getTotalDuration()) {
      ROS_ERROR("[traj_server] trajectory too short left!");
      return false;
    }
    Eigen::Vector3d p, v, a;
    p = traj.getPos(t + kCmdDt);
    v = traj.getVel(t + kCmdDt);
    a = traj.getAcc(t + kCmdDt);
    double yaw_rate = 0.0;
    double yaw = update_yaw(trajMsg.yaw, yaw_rate);
    publish_cmd(trajMsg.traj_id, p, v, a, yaw, yaw_rate);
    return true;
  }
  else{
    std::cout << "[traj_server] waiting for trajectory start... t : " << t << std::endl;
  }
  return false;
}

// void heartbeatCallback(const std_msgs::EmptyConstPtr &msg) {
//   heartbeat_time_ = ros::Time::now();
// }

void polyTrajCallback(const quadrotor_msgs::PolyTrajConstPtr &msgPtr) {
  trajMsg_ = *msgPtr;
  if (!receive_traj_) {
    trajMsg_last_ = trajMsg_;
    receive_traj_ = true;
  }
}

void curPoseCallback(const nav_msgs::OdometryConstPtr &msgPtr) {
  cur_position[0] = msgPtr->pose.pose.position.x;
  cur_position[1] = msgPtr->pose.pose.position.y;

  // yaw from quaternion
  double siny_cosp = 2.0 * (msgPtr->pose.pose.orientation.w * msgPtr->pose.pose.orientation.z +
                            msgPtr->pose.pose.orientation.x * msgPtr->pose.pose.orientation.y);
  double cosy_cosp = 1.0 - 2.0 * (msgPtr->pose.pose.orientation.y * msgPtr->pose.pose.orientation.y +
                                  msgPtr->pose.pose.orientation.z * msgPtr->pose.pose.orientation.z);
  cur_position[2] = std::atan2(siny_cosp, cosy_cosp);
}

void cmdCallback(const ros::TimerEvent &e) {
  if (!receive_traj_) {
    return;
  }
  // ros::Time time_now = ros::Time::now();
  // if ((time_now - heartbeat_time_).toSec() > 0.5) {
  //   ROS_ERROR_ONCE("[traj_server] Lost heartbeat from the planner, is he dead?");
  //   publish_cmd(trajMsg_.traj_id, last_p_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), last_yaw_, 0);
  //   return;
  // }
  if (exe_traj(trajMsg_)) {
    trajMsg_last_ = trajMsg_;
    return;
  } else if (exe_traj(trajMsg_last_)) {
    return;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "traj_server");
  ros::NodeHandle nh("~");

  ros::Subscriber poly_traj_sub = nh.subscribe("trajectory", 10, polyTrajCallback);
  ros::Subscriber cur_robot_pose_sub = nh.subscribe("odometry", 10, curPoseCallback);
  nh.param("wmax", wmax_, 3.0);
  
  // ros::Subscriber heartbeat_sub = nh.subscribe("heartbeat", 10, heartbeatCallback);

  pos_cmd_pub_ = nh.advertise<quadrotor_msgs::PositionCommand>("position_cmd", 50);
  cmd_vel_pub_ = nh.advertise<bpmp_tracker::UnicycleInput>("unicycle_control_input", 1);

  ros::Timer cmd_timer = nh.createTimer(ros::Duration(0.01), cmdCallback);

  // ros::Duration(0.2).sleep();

  ROS_WARN("[Traj server]: ready.");

  ros::spin();

  return 0;
}
