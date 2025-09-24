//
// Created by larr-laptop on 25. 4. 15.
//
#include <cmath>
#include <bpmp_simulator/RosTypeConverter.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf/transform_listener.h> 
#include <pcl_ros/transforms.h> 
#include <pcl_ros/point_cloud.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


bpmp::RosTypeConverter::RosTypeConverter():nh_("~") {
    t0_ = ros::Time::now().toSec();
    vel_history_.resize(5);

    std::vector<std::string> default_topics = {
        "/airsim/object/dynamic_1/pose",
        "/airsim/object/dynamic_2/pose",
        "/airsim/object/dynamic_3/pose",
        "/airsim/object/dynamic_4/pose",
        "/airsim/object/dynamic_5/pose",
        "/airsim/object/dynamic_6/pose",
        "/airsim/object/dynamic_7/pose",
        "/airsim/object/dynamic_8/pose",
        "/airsim/object/dynamic_9/pose",
        "/airsim/object/dynamic_10/pose"
    };
    nh_.param<std::vector<std::string>>("object_poses_individual_topics", dynamic_topics_, default_topics);
    nh_.param("dynamic_sync_slop",  sync_slop_sec_, 0.1);
    nh_.param("dynamic_queue_size", queue_size_, 10);
    nh_.param("dynamic_z_offset",   dynamic_z_offset_, 0.0);
    // 경로 시각화 페이드 윈도우(초)
    nh_.param("path_fade_window_sec", dyn_path_fade_window_sec_, dyn_path_fade_window_sec_);

    if (dynamic_topics_.size() != 10) {
        ROS_FATAL_STREAM("Expected 10 dynamic topics but got " << dynamic_topics_.size() << ". Please provide 10.");
        throw std::runtime_error("Need 10 dynamic topics");
    }
    
    TargetPositionSubscriber_ = nh_.subscribe("/target_pose",1,&RosTypeConverter::TargtPositionCallback,this);
    TargetStatePublisher_ = nh_.advertise<bpmp_tracker::ObjectState>("/bpmp_simulator/target_state",1);
    PclSubscriber_ = nh_.subscribe("/points_masked",1,&RosTypeConverter::PclCallback,this);
    PclPublisher_ = nh_.advertise<sensor_msgs::PointCloud2>("/bpmp_simulator/point_cloud_obstacle",1);
    UnicycleInputSubscriber_ = nh_.subscribe("/bpmp_tracker/unicycle_control_input",1,&RosTypeConverter::UnicycleInputCallback,this);
    UnicycleInputPublisher_ = nh_.advertise<geometry_msgs::Twist>("/move_base/cmd_vel",1);
    RobotOdometrySuscriber_ = nh_.subscribe("/airsim/odom", 1,
                                            &RosTypeConverter::RobotOdometryCallback, this);
    RobotOdometryPublisher_ = nh_.advertise<nav_msgs::Odometry>("/base_odom",1);

    DynamicObstaclesPublisher_ = nh_.advertise<bpmp_tracker::ObjectStateList>("/bpmp_simulator/obstacle_state_list", 1);

    // 누적 경로 퍼블리셔 (Path + Marker)
    TargetPathPublisher_ = nh_.advertise<nav_msgs::Path>("/bpmp_simulator/target_path", 1, true);
    RobotPathPublisher_  = nh_.advertise<nav_msgs::Path>("/bpmp_simulator/robot_path", 1, true);
    TargetPathMarkerPublisher_ = nh_.advertise<visualization_msgs::Marker>("/bpmp_simulator/target_path_marker", 1, true);
    RobotPathMarkerPublisher_  = nh_.advertise<visualization_msgs::Marker>("/bpmp_simulator/robot_path_marker", 1, true);
	RobotMarkerPublisher_ = nh_.advertise<visualization_msgs::Marker>("/bpmp_simulator/robot_vis",1);
	TargetMarkerPublisher_ = nh_.advertise<visualization_msgs::Marker>("/bpmp_simulator/target_vis",1);
    // 동적장애물 궤적은 MarkerArray 대신 id별 Marker를 고정 id로 퍼블리시
    DynamicPathsPublisher_ = nh_.advertise<visualization_msgs::MarkerArray>("/bpmp_simulator/dynamic_paths", 1, true);
    DynamicObstaclesMarkerPublisher_ = nh_.advertise<visualization_msgs::MarkerArray>("/bpmp_simulator/dynamic_obstacles", 1);

    // 초기 header와 스타일 설정
    target_path_.header.frame_id = "map";
    robot_path_.header.frame_id  = "map";

    target_path_marker_.header.frame_id = "map";
    target_path_marker_.ns = "target_path";
    target_path_marker_.id = 0;
    target_path_marker_.type = visualization_msgs::Marker::LINE_STRIP;
    target_path_marker_.action = visualization_msgs::Marker::ADD;
    target_path_marker_.scale.x = 0.3; // 두께(증가)
    target_path_marker_.color.a = 1.0;
    target_path_marker_.color.r = 1.0; // 빨강
    target_path_marker_.color.g = 0.0;
    target_path_marker_.color.b = 0.0;

    robot_path_marker_.header.frame_id = "map";
    robot_path_marker_.ns = "robot_path";
    robot_path_marker_.id = 0;
    robot_path_marker_.type = visualization_msgs::Marker::LINE_STRIP;
    robot_path_marker_.action = visualization_msgs::Marker::ADD;
    robot_path_marker_.scale.x = 0.3;
    robot_path_marker_.color.a = 1.0;
    robot_path_marker_.color.r = 0.0;
    robot_path_marker_.color.g = 0.0;
    robot_path_marker_.color.b = 1.0; // 파랑

    target_vis_marker_.header.frame_id = "map";
    target_vis_marker_.ns = "target_vis";
    target_vis_marker_.id = 0;
    target_vis_marker_.type = visualization_msgs::Marker::CYLINDER;
    target_vis_marker_.action = visualization_msgs::Marker::ADD;
    target_vis_marker_.scale.x = 0.6;
    target_vis_marker_.scale.y = 0.6;
    target_vis_marker_.scale.z = 1.0;
    target_vis_marker_.color.a = 1.0;
    target_vis_marker_.color.r = 0.0;
    target_vis_marker_.color.g = 0.0;
    target_vis_marker_.color.b = 1.0; // 파랑

    robot_vis_marker_.header.frame_id = "map";
    robot_vis_marker_.ns = "target_vis";
    robot_vis_marker_.id = 0;
    robot_vis_marker_.type = visualization_msgs::Marker::CYLINDER;
    robot_vis_marker_.action = visualization_msgs::Marker::ADD;
    robot_vis_marker_.scale.x = 0.6;
    robot_vis_marker_.scale.y = 0.6;
    robot_vis_marker_.scale.z = 1.0;
    robot_vis_marker_.color.a = 1.0;
    robot_vis_marker_.color.r = 1.0; // Red
    robot_vis_marker_.color.g = 0.0;
    robot_vis_marker_.color.b = 0.0;


    for (int i=0;i<10;++i){
        auto &m = dyn_path_markers_[i];
        m.header.frame_id = "map";
        m.ns = "dyn_path";
        m.id = i;
        m.type = visualization_msgs::Marker::LINE_STRIP;
        m.action = visualization_msgs::Marker::ADD;
        m.scale.x = 0.3;
        m.color.a = 1.0;
        m.color.r = 0.0;
        m.color.g = 1.0; // 초록
        m.color.b = 0.0;
    }

    for (int i=0; i<10; ++i) {
        subDyn_[i].reset(new message_filters::Subscriber<geometry_msgs::PoseStamped>(nh_, dynamic_topics_[i], queue_size_));
      }
      syncA_.reset(new message_filters::Synchronizer<Policy5>(Policy5(queue_size_),
                 *subDyn_[0], *subDyn_[1], *subDyn_[2], *subDyn_[3], *subDyn_[4]));
      syncB_.reset(new message_filters::Synchronizer<Policy5>(Policy5(queue_size_),
                 *subDyn_[5], *subDyn_[6], *subDyn_[7], *subDyn_[8], *subDyn_[9]));
    
      syncA_->registerCallback(boost::bind(&RosTypeConverter::DynGroupACb, this, _1,_2,_3,_4,_5));
      syncB_->registerCallback(boost::bind(&RosTypeConverter::DynGroupBCb, this, _1,_2,_3,_4,_5));
    
      last_groupA_.reserve(5);
      last_groupB_.reserve(5);
    
}

void bpmp::RosTypeConverter::Run() {
    ros::Rate loop_rate(200.0);
    while(ros::ok()){
        Publish();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void bpmp::RosTypeConverter::TargtPositionCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
    double curr_time = curTime();
    float z_offset = 0.5;
    // 누적 경로 추가 (Path + Marker)
    geometry_msgs::PoseStamped ps;
    ps.header.stamp = ros::Time::now();
    ps.header.frame_id = "map";
    ps.pose = msg->pose;
    ps.pose.position.z += z_offset;
    target_path_.header.stamp = ps.header.stamp;
    target_path_.poses.push_back(ps);
    target_path_times_.push_back(ps.header.stamp);

    target_vis_marker_.pose.position.x = msg->pose.position.x;
    target_vis_marker_.pose.position.y = msg->pose.position.y;
    target_vis_marker_.pose.position.z = msg->pose.position.z;
    target_vis_marker_.pose.orientation.w = 1.0;
    target_vis_marker_.pose.orientation.x = 0.0;
    target_vis_marker_.pose.orientation.y = 0.0;
    target_vis_marker_.pose.orientation.z = 0.0;
    TargetMarkerPublisher_.publish(target_vis_marker_);

    geometry_msgs::Point p;
    p.x = ps.pose.position.x;
    p.y = ps.pose.position.y;
    p.z = ps.pose.position.z;
    target_path_marker_.header.stamp = ps.header.stamp;
    target_path_marker_.points.push_back(p);
    // 페이드 적용: 오래된 점일수록 투명
    // 오래된 포인트 제거 (윈도우: dyn_path_fade_window_sec_)
    {
        const ros::Time now_ts = ps.header.stamp;
        const double window = dyn_path_fade_window_sec_;
        size_t start_idx = 0;
        while (start_idx < target_path_times_.size() && (now_ts - target_path_times_[start_idx]).toSec() > window) {
            start_idx++;
        }
        if (start_idx > 0) {
            target_path_times_.erase(target_path_times_.begin(), target_path_times_.begin()+start_idx);
            target_path_marker_.points.erase(target_path_marker_.points.begin(), target_path_marker_.points.begin()+start_idx);
            if (!target_path_marker_.colors.empty()) target_path_marker_.colors.erase(target_path_marker_.colors.begin(), target_path_marker_.colors.begin()+start_idx);
        }
    }
    // 색 재설정 (빨강, 불투명)
    target_path_marker_.colors.resize(target_path_marker_.points.size());
    for (size_t i=0;i<target_path_marker_.points.size();++i){
        std_msgs::ColorRGBA c; c.r = 1.0f; c.g = 0.0f; c.b = 0.0f; c.a = 1.0f;
        target_path_marker_.colors[i] = c;
    }

    if (TargetPathPublisher_.getNumSubscribers() > 0)
        TargetPathPublisher_.publish(target_path_);
    if (TargetPathMarkerPublisher_.getNumSubscribers() > 0)
        TargetPathMarkerPublisher_.publish(target_path_marker_);

    // 기존 속도 추정 로직
    if(not is_target_position_received_){
        current_target_state_.px = msg->pose.position.x;
        current_target_state_.py = msg->pose.position.y;
        current_target_state_.pz = msg->pose.position.z + z_offset;
        current_target_state_.vx = 0.0;
        current_target_state_.vy = 0.0;
        current_target_state_.vz = 0.0;
        t0_history_ = curr_time;
        odom_count_++;
        is_target_position_received_ = true;
    }
    else{
        if(odom_count_<5){  //TODO: Parameterize (Yunwoo)
            previous_target_state_.px = current_target_state_.px;
            previous_target_state_.py = current_target_state_.py;
            previous_target_state_.pz = current_target_state_.pz;
            current_target_state_.px = msg->pose.position.x;
            current_target_state_.py = msg->pose.position.y;
            current_target_state_.pz = msg->pose.position.z + z_offset;
            current_target_state_.vx = (current_target_state_.px-previous_target_state_.px)/(curr_time-t0_history_);
            current_target_state_.vy = (current_target_state_.py-previous_target_state_.py)/(curr_time-t0_history_);
            current_target_state_.vz = (current_target_state_.pz-previous_target_state_.pz)/(curr_time-t0_history_);
            odom_count_++;
        }
        else{
            previous_target_state_.px = current_target_state_.px;
            previous_target_state_.py = current_target_state_.py;
            previous_target_state_.pz = current_target_state_.pz;
            current_target_state_.px = msg->pose.position.x;
            current_target_state_.py = msg->pose.position.y;
            current_target_state_.pz = msg->pose.position.z + z_offset;
            vel_history_[odom_count_iter_].vx = (current_target_state_.px-previous_target_state_.px)/(curr_time-t0_history_);
            vel_history_[odom_count_iter_].vy = (current_target_state_.py-previous_target_state_.py)/(curr_time-t0_history_);
            double temp_vel_x = 0.0; double temp_vel_y = 0.0;
            for(int i =0;i<5;i++){//TODO: Parmeterize (Yunwoo)
                temp_vel_x += vel_history_[i].vx;
                temp_vel_y += vel_history_[i].vy;
            }
            current_target_state_.vx = temp_vel_x/5.0;
            current_target_state_.vy = temp_vel_y/5.0;
            current_target_state_.vz = 0.0;
            odom_count_iter_++;
            if(odom_count_iter_ == 5)//TODO: Parameterize (Yunwoo)
                odom_count_iter_ = 0;
        }
        t0_history_ = curr_time;
    }
}

void bpmp::RosTypeConverter::RobotOdometryCallback(const nav_msgs::OdometryConstPtr &msg) {
    tf::Transform transform;
    nav_msgs::Odometry temp_msg;
    temp_msg.pose.pose.position.x = msg->pose.pose.position.x;
    temp_msg.pose.pose.position.y = msg->pose.pose.position.y;
    temp_msg.pose.pose.position.z = msg->pose.pose.position.z;;
    temp_msg.pose.pose.orientation.x = msg->pose.pose.orientation.x;
    temp_msg.pose.pose.orientation.y = msg->pose.pose.orientation.y;
    temp_msg.pose.pose.orientation.z = msg->pose.pose.orientation.z;
    temp_msg.pose.pose.orientation.w = msg->pose.pose.orientation.w;
    temp_msg.header.frame_id = "map";
    temp_msg.header.stamp = ros::Time::now();
    RobotOdometryPublisher_.publish(temp_msg);

    robot_vis_marker_.pose.position.x = msg->pose.pose.position.x;
    robot_vis_marker_.pose.position.y = msg->pose.pose.position.y;
    robot_vis_marker_.pose.position.z = msg->pose.pose.position.z;
    robot_vis_marker_.pose.orientation.w = 1.0;
    robot_vis_marker_.pose.orientation.x = 0.0;
    robot_vis_marker_.pose.orientation.y = 0.0;
    robot_vis_marker_.pose.orientation.z = 0.0;
    RobotMarkerPublisher_.publish(robot_vis_marker_);

    // 누적 경로 추가 (Path + Marker)
    geometry_msgs::PoseStamped ps;
    ps.header = temp_msg.header;
    ps.pose = temp_msg.pose.pose;
    robot_path_.header.stamp = ps.header.stamp;
    robot_path_.poses.push_back(ps);
    robot_path_times_.push_back(ps.header.stamp);

    geometry_msgs::Point p;
    p.x = ps.pose.position.x;
    p.y = ps.pose.position.y;
    p.z = ps.pose.position.z;
    robot_path_marker_.header.stamp = ps.header.stamp;
    robot_path_marker_.points.push_back(p);
    // 오래된 포인트 제거 (윈도우: dyn_path_fade_window_sec_)
    {
        const ros::Time now_ts = ps.header.stamp;
        const double window = dyn_path_fade_window_sec_;
        size_t start_idx = 0;
        while (start_idx < robot_path_times_.size() && (now_ts - robot_path_times_[start_idx]).toSec() > window) {
            start_idx++;
        }
        if (start_idx > 0) {
            robot_path_times_.erase(robot_path_times_.begin(), robot_path_times_.begin()+start_idx);
            robot_path_marker_.points.erase(robot_path_marker_.points.begin(), robot_path_marker_.points.begin()+start_idx);
            if (!robot_path_marker_.colors.empty()) robot_path_marker_.colors.erase(robot_path_marker_.colors.begin(), robot_path_marker_.colors.begin()+start_idx);
        }
    }
    // 색 재설정 (파랑, 불투명)
    robot_path_marker_.colors.resize(robot_path_marker_.points.size());
    for (size_t i=0;i<robot_path_marker_.points.size();++i){
        std_msgs::ColorRGBA c; c.r = 0.0f; c.g = 0.0f; c.b = 1.0f; c.a = 1.0f;
        robot_path_marker_.colors[i] = c;
    }

    if (RobotPathPublisher_.getNumSubscribers() > 0)
        RobotPathPublisher_.publish(robot_path_);
    if (RobotPathMarkerPublisher_.getNumSubscribers() > 0)
        RobotPathMarkerPublisher_.publish(robot_path_marker_);

    transform.setOrigin(tf::Vector3(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z));
    tf::Quaternion q(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
    transform.setRotation(q);
    br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "current"));
}

void bpmp::RosTypeConverter::Publish() {
    TargetStatePublisher_.publish(current_target_state_);
    PclPublisher_.publish(pcl_output_);
}

void bpmp::RosTypeConverter::PclCallback(const pcl::PointCloud<pcl::PointXYZ> &msg) {
    pcl::toROSMsg(msg,pcl_output_);
    pcl_output_.header.frame_id="map";
    pcl_output_.header.stamp = ros::Time::now();
}

void bpmp::RosTypeConverter::UnicycleInputCallback(const bpmp_tracker::UnicycleInput &msg) {
    geometry_msgs::Twist control_input;
    control_input.linear.x = msg.vel_linear;
    control_input.linear.y = 0.0;
    control_input.linear.z = 0.0;
    control_input.angular.x = 0.0;
    control_input.angular.y = 0.0;
    control_input.angular.z = msg.vel_angular;
    UnicycleInputPublisher_.publish(control_input);
}

void bpmp::RosTypeConverter::UpdateObstacleFromPose(
    int idx, const geometry_msgs::PoseStampedConstPtr& msg)
{
  double curr_time = curTime();
  auto& o = dyn_[idx];
  const double dt = std::max(1e-6, curr_time - o.t_last);

  if (!o.received) {
    o.curr.px = msg->pose.position.x;
    o.curr.py = msg->pose.position.y;
    o.curr.pz = msg->pose.position.z + dynamic_z_offset_;
    o.curr.vx = 0.0;
    o.curr.vy = 0.0;
    o.curr.vz = 0.0;
    o.t_last  = curr_time;
    o.count   = 1;
    o.iter    = 0;
    o.received = true;
    return;
  }
  o.prev = o.curr;
  o.curr.px = msg->pose.position.x;
  o.curr.py = msg->pose.position.y;
  o.curr.pz = msg->pose.position.z + dynamic_z_offset_;

  if (o.count < 5) {
    o.curr.vx = (o.curr.px - o.prev.px) / dt;
    o.curr.vy = (o.curr.py - o.prev.py) / dt;
    o.curr.vz = (o.curr.pz - o.prev.pz) / dt;
    o.count++;
  } else {
    o.vel_hist[o.iter].vx = (o.curr.px - o.prev.px) / dt;
    o.vel_hist[o.iter].vy = (o.curr.py - o.prev.py) / dt;
    double sx=0.0, sy=0.0;
    for (int k=0; k<5; ++k) { sx += o.vel_hist[k].vx; sy += o.vel_hist[k].vy; }
    o.curr.vx = sx / 5.0;
    o.curr.vy = sy / 5.0;
    o.curr.vz = 0.0; 
    o.iter = (o.iter + 1) % 5;
  }
  o.t_last = curr_time;
}

// ---- 그룹 A(0~4) ----
void bpmp::RosTypeConverter::DynGroupACb(
  const geometry_msgs::PoseStampedConstPtr& m0,
  const geometry_msgs::PoseStampedConstPtr& m1,
  const geometry_msgs::PoseStampedConstPtr& m2,
  const geometry_msgs::PoseStampedConstPtr& m3,
  const geometry_msgs::PoseStampedConstPtr& m4)
{
  UpdateObstacleFromPose(0, m0);
  UpdateObstacleFromPose(1, m1);
  UpdateObstacleFromPose(2, m2);
  UpdateObstacleFromPose(3, m3);
  UpdateObstacleFromPose(4, m4);

  last_groupA_.clear(); last_groupA_.reserve(5);
  for (int i=0;i<5;++i) last_groupA_.push_back(dyn_[i].curr);

  last_stamp_A_ = ros::Time::now();
  TryMergeAndPublish();
}

// ---- 그룹 B(5~9) ----
void bpmp::RosTypeConverter::DynGroupBCb(
  const geometry_msgs::PoseStampedConstPtr& m5,
  const geometry_msgs::PoseStampedConstPtr& m6,
  const geometry_msgs::PoseStampedConstPtr& m7,
  const geometry_msgs::PoseStampedConstPtr& m8,
  const geometry_msgs::PoseStampedConstPtr& m9)
{
  UpdateObstacleFromPose(5, m5);
  UpdateObstacleFromPose(6, m6);
  UpdateObstacleFromPose(7, m7);
  UpdateObstacleFromPose(8, m8);
  UpdateObstacleFromPose(9, m9);

  last_groupB_.clear(); last_groupB_.reserve(5);
  for (int i=5;i<10;++i) last_groupB_.push_back(dyn_[i].curr);

  last_stamp_B_ = ros::Time::now();
  TryMergeAndPublish();
}

// ---- 그룹 A/B 둘 다 준비되면 병합 퍼블리시 ----
void bpmp::RosTypeConverter::TryMergeAndPublish() {
  if (last_groupA_.size() != 5 || last_groupB_.size() != 5) return;

  const double dt = std::fabs((last_stamp_A_ - last_stamp_B_).toSec());
  if (dt > sync_slop_sec_) return; 

  bpmp_tracker::ObjectStateList list_msg;
  list_msg.object_state_list.reserve(10);
  for (const auto& s : last_groupA_) list_msg.object_state_list.push_back(s);
  for (const auto& s : last_groupB_) list_msg.object_state_list.push_back(s);

  // 동적장애물 궤적 누적 (MarkerArray)
  visualization_msgs::MarkerArray arr;
  arr.markers.reserve(10);
  const ros::Time now = ros::Time::now();
  for (int i=0;i<10;++i){
      auto &m = dyn_path_markers_[i];
      m.header.stamp = now;
      geometry_msgs::Point p; p.x = list_msg.object_state_list[i].px; p.y = list_msg.object_state_list[i].py; p.z = list_msg.object_state_list[i].pz;
      m.points.push_back(p);
      dyn_path_times_[i].push_back(now);

      // 오래된 포인트 제거 (윈도우: dyn_path_fade_window_sec_)
      const double window = dyn_path_fade_window_sec_;
      size_t start_idx = 0;
      while (start_idx < dyn_path_times_[i].size() && (now - dyn_path_times_[i][start_idx]).toSec() > window) {
          start_idx++;
      }
      if (start_idx > 0) {
          dyn_path_times_[i].erase(dyn_path_times_[i].begin(), dyn_path_times_[i].begin()+start_idx);
          m.points.erase(m.points.begin(), m.points.begin()+start_idx);
          if (!m.colors.empty()) m.colors.erase(m.colors.begin(), m.colors.begin()+start_idx);
      }

      // colors를 포인트 수에 맞춰 시간 기반 알파(초록)로 세팅: 최신=불투명, 오래됨=투명
      m.colors.resize(m.points.size());
      for (size_t k=0; k<m.points.size(); ++k){
          const double age = (now - dyn_path_times_[i][k]).toSec();
          double a = 1.0 - std::min(1.0, std::max(0.0, age / window));
          std_msgs::ColorRGBA c; c.r = 0.0f; c.g = 1.0f; c.b = 0.0f; c.a = static_cast<float>(a);
          m.colors[k] = c;
      }
      arr.markers.push_back(m);
  }
  if (DynamicPathsPublisher_.getNumSubscribers() > 0)
      DynamicPathsPublisher_.publish(arr);

  // 동적장애물 현재 위치 마커 (초록색 구)
  visualization_msgs::MarkerArray obs;
  obs.markers.reserve(10);
  for (int i=0;i<10;++i){
      visualization_msgs::Marker sphere;
      sphere.header.frame_id = "map";
      sphere.header.stamp = now;
      sphere.ns = "dyn_obs";
      sphere.id = i;
      sphere.type = visualization_msgs::Marker::CYLINDER;
      sphere.action = visualization_msgs::Marker::ADD;
      sphere.scale.x = 1.0; sphere.scale.y = 1.0; sphere.scale.z = 2.0;
      sphere.color.a = 1.0; sphere.color.r = 0.0; sphere.color.g = 1.0; sphere.color.b = 0.0;
      sphere.pose.orientation.w = 1.0;
      sphere.pose.position.x = list_msg.object_state_list[i].px;
      sphere.pose.position.y = list_msg.object_state_list[i].py;
      sphere.pose.position.z = list_msg.object_state_list[i].pz;
      obs.markers.push_back(sphere);
  }
  if (DynamicObstaclesMarkerPublisher_.getNumSubscribers() > 0)
      DynamicObstaclesMarkerPublisher_.publish(obs);

  DynamicObstaclesPublisher_.publish(list_msg);
}

