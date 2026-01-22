//
// Created by larr-laptop on 1/15/26.
//
#include <comparison/ElasticTracker.h>

bpmp::ElasticTracker::ElasticTracker():nh_("~") {
    nh_.param<double>("x_length",param_.param_g.x_length,1.0);
    nh_.param<double>("y_length",param_.param_g.y_length,1.0);
    nh_.param<double>("z_length",param_.param_g.z_length,1.0);
    nh_.param<double>("resolution",param_.param_g.resolution,1.0);
    nh_.param<double>("agent_size", param_.param_g.agent_size,0.25);

    nh_.param<double>("tracking_dist",tracking_dist_,3.0);
    nh_.param<double>("tolerance_d",tolerance_d_,0.3);
    nh_.param<double>("tracking_dur",tracking_dur_,3.0);
    nh_.param<double>("tracking_dt",tracking_dt_,0.2);
    nh_.param<int>("plan_hz",plan_hz_,10);

    gridmapPtr_ = std::make_shared<mapping::OccGridMap>();
    mapping::OccGridMap temp_map;
    temp_map.setup(param_.param_g.resolution,Eigen::Vector3d(param_.param_g.x_length,
                                                             param_.param_g.y_length,param_.param_g.z_length),10.0,true);
    gridmapPtr_.reset(new mapping::OccGridMap(temp_map));
    visPtr_ = std::make_shared<visualization::Visualization>(nh_);
    trajOptPtr_ = std::make_shared<traj_opt::TrajOpt>(nh_);
    prePtr_ = std::make_shared<prediction::Predict>(nh_);
    envPtr_ = std::make_shared<env::Env>(nh_, gridmapPtr_);

    tracker_sub_ = nh_.subscribe("/base_odom", 1,
                                              &ElasticTracker::TrackerStateCallback, this);
    target_sub_ = nh_.subscribe("/bpmp_simulator/target_state", 1,
                                             &ElasticTracker::TargetStateCallback, this);
    dynamic_obstacle_sub_ = nh_.subscribe("/obstacle_pointcloud", 1,
                                                    &ElasticTracker::ObstacleStateListCallback, this);
    cloud_.header.frame_id="map";

    occ_grid_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/elastic_tracker/occ_grid",1);
    traj_pub_ = nh_.advertise<quadrotor_msgs::PolyTraj>("/elastic_tracker/trajectory",1);

}

void bpmp::ElasticTracker::Run() {
    ros::Rate loop_rate(plan_hz_);
    while(ros::ok()){
        if(IsInfoReady()){
            // ROS_WARN("[ELASTIC TRACKER]: All info received. Start Elastic Tracking Planning...");
            Planning();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void bpmp::ElasticTracker::TargetStateCallback(const bpmp_tracker::ObjectState &msg) {
    is_target_info_received_ = true;
    current_target_state_.px = msg.px;
    current_target_state_.py = msg.py;
    current_target_state_.pz = msg.pz;
    current_target_state_.vx = msg.vx;
    current_target_state_.vy = msg.vy;
    current_target_state_.vz = msg.vz;
//    std::cout<<"[ELASTIC TRACKER]: Got Target State"<<std::endl;
}

void bpmp::ElasticTracker::TrackerStateCallback(const nav_msgs::Odometry &msg) {
  current_tracker_state_.px = msg.pose.pose.position.x;
  current_tracker_state_.py = msg.pose.pose.position.y;
  current_tracker_state_.pz = msg.pose.pose.position.z;
  double q[4] {msg.pose.pose.orientation.w,
              msg.pose.pose.orientation.x,
              msg.pose.pose.orientation.y,
              msg.pose.pose.orientation.z};
  current_tracker_state_.theta = atan2(
          2.0 * (q[0] * q[3] + q[1] * q[2]),
          1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3])
        );
  double cur_time = msg.header.stamp.toSec();

  if (!is_tracker_info_received_) {
      tracker_velocity_ = 0.0;
      is_tracker_info_received_ = true;
      std::cout<<"[ELASTIC TRACKER]: Received first tracker info."<<std::endl;
  }
  else if (sqrt(pow(prev_pose_[0] - msg.pose.pose.position.x, 2)) +
          pow(prev_pose_[1] - msg.pose.pose.position.y, 2) > 0.1){
      std::cout<<"[ELASTIC TRACKER]: Tracker position jump detected! Resetting tracker info..."<<std::endl;
      tracker_velocity_ = 0.0;
      is_tracker_info_received_ = false;
  }
  else{
      double dt = cur_time - prev_time_;
      double dx = current_tracker_state_.px - prev_pose_[0];
      double dy = current_tracker_state_.py - prev_pose_[1];
      double velocity_dir = (dx * cos(current_tracker_state_.theta) + dy * sin(current_tracker_state_.theta)) >= 0 ? 1.0 : -1.0;
      if (dt > 1e-6) {
          tracker_velocity_ = velocity_dir * sqrt(dx * dx + dy * dy) / dt;
      } else {
          tracker_velocity_ = 0.0;
      }
  }
  prev_time_ = cur_time;
  prev_pose_[0] = current_tracker_state_.px;
  prev_pose_[1] = current_tracker_state_.py;
//    std::cout<<"[ELASTIC TRACKER]: Got Tracker State"<<std::endl;
}

void bpmp::ElasticTracker::ObstacleStateListCallback(const sensor_msgs::PointCloud2 &msg) {
    is_pcl_info_received_ = true;
    cloud_.points.clear();
    pcl::fromROSMsg(msg, cloud_);
    mapping::OccGridMap grid_map;
    grid_map.setup(param_.param_g.resolution,Eigen::Vector3d(param_.param_g.x_length,
                                                             param_.param_g.y_length,param_.param_g.z_length),10.0,true);
    for (const auto& pt : cloud_) {
        Eigen::Vector3d p(pt.x, pt.y, pt.z);
        grid_map.setOcc(p);
    }
    grid_map.inflate(int(param_.param_g.agent_size/param_.param_g.resolution));
    gridmapPtr_.reset(new mapping::OccGridMap(grid_map));
    sensor_msgs::PointCloud2 occ_msg;
    gridmapPtr_->occ2pc(occ_msg);
    occ_msg.header.stamp = ros::Time::now();
    occ_msg.header.frame_id = "map";
    occ_grid_pub_.publish(occ_msg);
//    std::cout<<"[ELASTIC TRACKER]: Got Obstacle PointCloud"<<std::endl;
}

bool bpmp::ElasticTracker::IsInfoReady() {
    return is_target_info_received_ && is_tracker_info_received_;
}

void bpmp::ElasticTracker::pub_hover_p(const Eigen::Vector3d& hover_p, const ros::Time& stamp) {
    quadrotor_msgs::PolyTraj traj_msg;
    traj_msg.hover = true;
    traj_msg.hover_p.resize(3);
    for (int i = 0; i < 3; ++i) {
      traj_msg.hover_p[i] = hover_p[i];
    }
    traj_msg.start_time = stamp;
    traj_msg.traj_id = traj_id_++;
    traj_pub_.publish(traj_msg);
}

bool bpmp::ElasticTracker::validcheck(const Trajectory& traj, const ros::Time& t_start, const double& check_dur = 1.0) {
    double t0 = (ros::Time::now() - t_start).toSec();
    t0 = t0 > 0.0 ? t0 : 0.0;
    double delta_t = check_dur < traj.getTotalDuration() ? check_dur : traj.getTotalDuration();
    for (double t = t0; t < t0 + delta_t; t += 0.01) {
      Eigen::Vector3d p = traj.getPos(t);
      if (gridmapPtr_->isOccupied(p)) {
        return false;
      }
    }
    return true;
}

void bpmp::ElasticTracker::pub_traj(const Trajectory& traj, const double& yaw, const ros::Time& stamp) {
    quadrotor_msgs::PolyTraj traj_msg;
    traj_msg.hover = false;
    traj_msg.order = 5;
    Eigen::VectorXd durs = traj.getDurations();
    int piece_num = traj.getPieceNum();
    traj_msg.duration.resize(piece_num);
    traj_msg.coef_x.resize(6 * piece_num);
    traj_msg.coef_y.resize(6 * piece_num);
    traj_msg.coef_z.resize(6 * piece_num);
    for (int i = 0; i < piece_num; ++i) {
      traj_msg.duration[i] = durs(i);
      CoefficientMat cMat = traj[i].getCoeffMat();
      int i6 = i * 6;
      for (int j = 0; j < 6; j++) {
        traj_msg.coef_x[i6 + j] = cMat(0, j);
        traj_msg.coef_y[i6 + j] = cMat(1, j);
        traj_msg.coef_z[i6 + j] = cMat(2, j);
      }
    }
    traj_msg.start_time = stamp;
    traj_msg.traj_id = traj_id_++;
    // NOTE yaw
    traj_msg.yaw = yaw;
    traj_pub_.publish(traj_msg);
    // std::cout<<"[planner] Published new trajectory."<<std::endl;
}

void bpmp::ElasticTracker::Planning() {
    // obtain state of odom
    Eigen::Vector3d odom_p(current_tracker_state_.px,
                           current_tracker_state_.py,
                           0);
    Eigen::Vector3d odom_v(tracker_velocity_ * cos(current_tracker_state_.theta),
                           tracker_velocity_ * sin(current_tracker_state_.theta),
                           0);
    // yaw to quaternion
    double cy = cos(current_tracker_state_.theta * 0.5);
    double sy = sin(current_tracker_state_.theta * 0.5);


    Eigen::Quaterniond odom_q(cy, 0, 0, sy);

    // NOTE obtain state of target

    Eigen::Vector3d target_p(current_target_state_.px,
                             current_target_state_.py,
                             0);
    Eigen::Vector3d target_v(current_target_state_.vx,
                             current_target_state_.vy,
                             0);
    // Eigen::Quaterniond target_q; // isn't it useless? //승우
    // target_q.w() = replanStateMsg_.target.pose.pose.orientation.w;
    // target_q.x() = replanStateMsg_.target.pose.pose.orientation.x;
    // target_q.y() = replanStateMsg_.target.pose.pose.orientation.y;
    // target_q.z() = replanStateMsg_.target.pose.pose.orientation.z;

    // NOTE force-hover: waiting for the speed of drone small enough
    if (force_hover_ && odom_v.norm() > 0.1) {
      return;
    }

    // target_p.z() += 1.0; // 필요없을듯?
    // NOTE determin whether to replan
    Eigen::Vector3d dp = target_p - odom_p;
    // std::cout << "dist : " << dp.norm() << std::endl;
    double desired_yaw = std::atan2(dp.y(), dp.x());
    Eigen::Vector3d project_yaw = odom_q.toRotationMatrix().col(0);  // NOTE ZYX
    double now_yaw = std::atan2(project_yaw.y(), project_yaw.x());
    if (std::fabs((target_p - odom_p).norm() - tracking_dist_) < tolerance_d_ &&
        odom_v.norm() < 0.1 && target_v.norm() < 0.2 &&
        std::fabs(desired_yaw - now_yaw) < 0.5) {
      if (!wait_hover_) {
        pub_hover_p(odom_p, ros::Time::now());
        wait_hover_ = true;
      }
      ROS_WARN("[planner] Stopping...");
      // replanStateMsg_.state = -1; // -> useless topic
      // replanState_pub_.publish(replanStateMsg_);
      return;
    } else {
      wait_hover_ = false;
    }
    // }

    // NOTE obtain map
    // while (gridmap_lock_.test_and_set())
      // ;
    // gridmapPtr_->from_msg(map_msg_);
    // replanStateMsg_.occmap = map_msg_;
    // gridmap_lock_.clear();
    prePtr_->setMap(*gridmapPtr_);

    // visualize the ray from drone to target
    if (envPtr_->checkRayValid(odom_p, target_p)) {
      visPtr_->visualize_arrow(odom_p, target_p, "ray", visualization::yellow);
      // std::cout << "[planner] Ray valid." << std::endl;
    } else {
      visPtr_->visualize_arrow(odom_p, target_p, "ray", visualization::red);
      // std::cout << "[planner] Ray invalid." << std::endl;
    }

    // NOTE prediction
    std::vector<Eigen::Vector3d> target_predcit;
    // ros::Time t_start = ros::Time::now();
    bool generate_new_traj_success = prePtr_->predict(target_p, target_v, target_predcit);
    // ros::Time t_stop = ros::Time::now();
    // std::cout << "predict costs: " << (t_stop - t_start).toSec() * 1e3 << "ms" << std::endl;
    if (generate_new_traj_success) {
      Eigen::Vector3d observable_p = target_predcit.back();
      visPtr_->visualize_path(target_predcit, "target_predict");
      std::vector<Eigen::Vector3d> observable_margin;
      for (double theta = 0; theta <= 2 * M_PI; theta += 0.01) {
        observable_margin.emplace_back(observable_p + tracking_dist_ * Eigen::Vector3d(cos(theta), sin(theta), 0));
      }
      visPtr_->visualize_path(observable_margin, "observable_margin");
    }
    else{
      ROS_WARN("[planner] Prediction failed!");
      return;
    }

    /*Planning start here*/
    
    // NOTE replan state
    Eigen::MatrixXd iniState;
    iniState.setZero(3, 3);
    // ros::Time replan_stamp = ros::Time::now() + ros::Duration(0.03); // TODO why????
    ros::Time replan_stamp = ros::Time::now();
    
    double replan_t = (replan_stamp - replan_stamp_).toSec();
    // if (force_hover_ || replan_t > traj_poly_.getTotalDuration()) {
        // should replan from the hover state
        iniState.col(0) = odom_p;
        iniState.col(1) = odom_v;
    // } else {
      // should replan from the last trajectory
      // iniState.col(0) = traj_poly_.getPos(replan_t);
      // iniState.col(1) = traj_poly_.getVel(replan_t);
      // iniState.col(2) = traj_poly_.getAcc(replan_t);
    // }
    // replanStateMsg_.header.stamp = ros::Time::now();
    // replanStateMsg_.iniState.resize(9);
    // Eigen::Map<Eigen::MatrixXd>(replanStateMsg_.iniState.data(), 3, 3) = iniState;

    // NOTE path searching
    Eigen::Vector3d p_start = iniState.col(0);
    std::vector<Eigen::Vector3d> path, way_pts;

    // NOTE calculate time of path searching, corridor generation and optimization

    // static double t_path_ = 0;
    // static double t_corridor_ = 0;
    // static double t_optimization_ = 0;
    // static int times_path_ = 0;
    // static int times_corridor_ = 0;
    // static int times_optimization_ = 0;
    // double t_path = 0;

    if (generate_new_traj_success) {
      ros::Time t_front0 = ros::Time::now();
      // if (land_triger_received_) {
        // generate_new_traj_success = envPtr_->short_astar(p_start, target_p, path);
      // } else {
      envPtr_->setMap(gridmapPtr_);
      generate_new_traj_success = envPtr_->findVisiblePath(p_start, target_predcit, way_pts, path);
      // }
      // ros::Time t_end0 = ros::Time::now();
      // t_path += (t_end0 - t_front0).toSec() * 1e3;
    }

    std::vector<Eigen::Vector3d> visible_ps;
    std::vector<double> thetas;
    Trajectory traj;
    if (generate_new_traj_success) {
      // std::cout << "[planner] Path found." << std::endl;
      visPtr_->visualize_path(path, "astar");
      // if (land_triger_received_) {
      //   for (const auto& p : target_predcit) {
      //     path.push_back(p);
      //   }
      // } else {
        // NOTE generate visible regions
        target_predcit.pop_back();
        way_pts.pop_back();
        // ros::Time t_front1 = ros::Time::now();
        envPtr_->generate_visible_regions(target_predcit, way_pts,
                                          visible_ps, thetas);
        // ros::Time t_end1 = ros::Time::now();
        // t_path += (t_end1 - t_front1).toSec() * 1e3;
        visPtr_->visualize_pointcloud(visible_ps, "visible_ps");
        visPtr_->visualize_fan_shape_meshes(target_predcit, visible_ps, thetas, "visible_region");

        // TODO change the final state
        std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> rays;
        for (int i = 0; i < (int)way_pts.size(); ++i) {
          rays.emplace_back(target_predcit[i], way_pts[i]);
        }
        visPtr_->visualize_pointcloud(way_pts, "way_pts");
        way_pts.insert(way_pts.begin(), p_start);
        // ros::Time t_front2 = ros::Time::now();
        envPtr_->pts2path(way_pts, path);
        // ros::Time t_end2 = ros::Time::now();
        // t_path += (t_end2 - t_front2).toSec() * 1e3;
      // }
      // NOTE corridor generating
      std::vector<Eigen::MatrixXd> hPolys;
      std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> keyPts;

      // ros::Time t_front3 = ros::Time::now();
      envPtr_->generateSFC(path, 2.0, hPolys, keyPts);
      // ros::Time t_end3 = ros::Time::now();
      // double t_corridor = (t_end3 - t_front3).toSec() * 1e3;

      envPtr_->visCorridor(hPolys);
      visPtr_->visualize_pairline(keyPts, "keyPts");

      // NOTE trajectory optimization
      Eigen::MatrixXd finState;
      finState.setZero(3, 3);
      finState.col(0) = path.back();
      finState.col(1) = target_v;
      // ros::Time t_front4 = ros::Time::now();
      // if (land_triger_received_) {
      //   finState.col(0) = target_predcit.back();
      //   generate_new_traj_success = trajOptPtr_->generate_traj(iniState, finState, target_predcit, hPolys, traj);
      // } else {

      generate_new_traj_success = trajOptPtr_->generate_traj(iniState, finState,
                                                              target_predcit, visible_ps, thetas,
                                                              hPolys, traj);
      // }
      // ros::Time t_end4 = ros::Time::now();
      // double t_optimization = (t_end4 - t_front4).toSec() * 1e3;

      // NOTE average calculating time of path searching, corridor generation and optimization

      // t_path_ = (t_path_ * times_path_ + t_path) / (++times_path_);
      // t_corridor_ = (t_corridor_ * times_corridor_ + t_corridor) / (++times_corridor_);
      // t_optimization_ = (t_optimization_ * times_optimization_ + t_optimization) / (++times_optimization_);

      // std::cout << "t_path_: " << t_path_ << " ms" << std::endl;
      // std::cout << "t_corridor_: " << t_corridor_ << " ms" << std::endl;
      // std::cout << "t_optimization_: " << t_optimization_ << " ms" << std::endl;

      visPtr_->visualize_traj(traj, "traj");

    }

    // NOTE collision check
    bool valid = false;
    if (generate_new_traj_success) {
      valid = validcheck(traj, replan_stamp);
    } else {
      // replanStateMsg_.state = -2;
      // replanState_pub_.publish(replanStateMsg_);
      ROS_WARN("[planner] Trajectory generation failed! result path is collided.");
    }
    if (valid) {
      force_hover_ = false;
      ROS_WARN("[planner] REPLAN SUCCESS");
      // replanStateMsg_.state = 0;
      // replanState_pub_.publish(replanStateMsg_);
      Eigen::Vector3d dp = target_p + target_v * 0.03 - iniState.col(0);
      // NOTE : if the drone is going to unknown areas, watch that direction
      // Eigen::Vector3d un_known_p = traj.getPos(1.0);
      // if (gridmapPtr_->isUnKnown(un_known_p)) {
      //   dp = un_known_p - odom_p;
      // }
      double yaw = std::atan2(dp.y(), dp.x());
      // if (land_triger_received_) {
      //   yaw = 2 * std::atan2(target_q.z(), target_q.w());
      // }
      pub_traj(traj, yaw, replan_stamp);
      traj_poly_ = traj;
      replan_stamp_ = replan_stamp;
    } else if (force_hover_) {
      ROS_ERROR("[planner] REPLAN FAILED, HOVERING...");
      // replanStateMsg_.state = 1;
      // replanState_pub_.publish(replanStateMsg_);
      return;
    } else if (!validcheck(traj_poly_, replan_stamp_)) {
      force_hover_ = true;
      ROS_FATAL("[planner] EMERGENCY STOP!!!");
      // replanStateMsg_.state = 2;
      // replanState_pub_.publish(replanStateMsg_);
      pub_hover_p(iniState.col(0), replan_stamp);
      return;
    } else {
      ROS_ERROR("[planner] REPLAN FAILED, EXECUTE LAST TRAJ...");
      // replanStateMsg_.state = 3;
      // replanState_pub_.publish(replanStateMsg_);
      return;  // current generated traj invalid but last is valid
    }
    visPtr_->visualize_traj(traj, "traj");
    
    /*Planning end here*/
}

