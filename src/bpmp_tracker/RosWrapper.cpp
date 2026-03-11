//
// Created by larr-laptop on 25. 6. 9.
//
//
// Created by larr-planning on 24. 3. 4.
//
#include "bpmp_tracker/RosWrapper.h"

void bpmp::RosWrapper::RunRos() {
    ros::Rate loop_rate(this->GetControlFrequency());
    ros::AsyncSpinner spinner(4);
    spinner.start();
    while (ros::ok()) {
        PrepareRosMsgs();
        PublishRosMsgs();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void bpmp::RosWrapper::PrepareRosMsgs() {

}
void bpmp::RosWrapper::PublishRosMsgs() {
    // ROS_INFO_STREAM("TEST");

    { // RAW
        p_base_->mutex_set_[1].lock();
        tracker_raw_primitives_publisher_.publish(
                visualizer_->VisualizeRawPrimitives(p_base_->tracker_raw_primitives_));
        p_base_->mutex_set_[1].unlock();
    }
    {   // FEASIBLE
        p_base_->mutex_set_[1].lock();
        tracker_feasible_primitives_publisher_.publish(
                visualizer_->VisualizeFeasiblePrimitives(p_base_->tracker_raw_primitives_,
                                                         p_base_->tracker_feasible_index_));
        p_base_->mutex_set_[1].unlock();
    }
    {   // BEST
        p_base_->mutex_set_[1].lock();
        if(p_base_->success_flag_){
            ROS_INFO_STREAM("Planning Success. Publish Best Primitive.");
            tracker_best_trajectory_publisher_.publish(
                visualizer_->VisualizeBestPrimitive(p_base_->tracker_raw_primitives_, p_base_->tracker_best_index_));

            // Polynomial Description
            const auto &best = p_base_->tracker_raw_primitives_[p_base_->tracker_best_index_];
            bpmp_tracker::PolyState result;
            result.t0 = best.t0;
            result.tf = best.tf;

            // transform Bernstein control points from tracker frame to map frame
            bpmp::UnicycleState tracker_state;
            p_base_->mutex_set_[0].lock();
            tracker_state = p_base_->current_tracker_list_read_;
            p_base_->mutex_set_[0].unlock();

            const double c = cos(tracker_state.theta);
            const double s = sin(tracker_state.theta);
            for (int i = 0; i < 4; i++) {
                const double lx = best.ctrl_x[i];
                const double ly = best.ctrl_y[i];
                const double lz = best.ctrl_z[i];
                const double wx = tracker_state.px + c * lx - s * ly;
                const double wy = tracker_state.py + s * lx + c * ly;
                const double wz = tracker_state.pz + lz;
                result.x_coeff.push_back(wx);
                result.y_coeff.push_back(wy);
                result.z_coeff.push_back(wz);
            }
            tracker_best_polystate_publisher_.publish(result);
        }
        p_base_->mutex_set_[1].unlock();

    }
    {   // CONTROL INPUT
        p_base_->mutex_set_[1].lock();
        if(p_base_->success_flag_){
            // ROS_INFO_STREAM("Planning Success. Publish Control Input.");
            tracker_control_input_publisher_.publish(GenerateControlInput(p_base_->tracker_raw_primitives_,p_base_->tracker_best_index_,this->GetCurrentTime()));
        }
        else{
            // ROS_INFO_STREAM("Planning Failed. Publish Zero Control Input.");
            bpmp_tracker::UnicycleInput zero_input;
            zero_input.vel_linear = 0.0;
            p_base_->mutex_set_[0].lock();
            double yaw, px,py,qx,qy;
            yaw = p_base_->current_tracker_list_read_.theta;
            px = p_base_->current_tracker_list_read_.px;
            py = p_base_->current_tracker_list_read_.py;
            qx = p_base_->target_prediction_read_.ctrl_x[0];
            qy = p_base_->target_prediction_read_.ctrl_y[0];
            p_base_->mutex_set_[0].unlock();
            double desired_yaw = atan2(qy-py,qx-px);
            double yaw_gap = desired_yaw - yaw;
            yaw_gap = fmod(yaw_gap+M_PI,2.0*M_PI);
            if (yaw_gap<0)
                yaw_gap+=2.0*M_PI;
            yaw_gap -= M_PI;
            zero_input.vel_angular= (2.0)*yaw_gap;
            tracker_control_input_publisher_.publish(zero_input);
        }
        p_base_->mutex_set_[1].unlock();
    }
    {   // Corridor
        p_base_->mutex_set_[1].lock();
        if(not p_base_->polys_.empty()){
            decomp_ros_msgs::PolyhedronArray polyhedron_msg = DecompROS::polyhedron_array_to_ros(p_base_->polys_);
            polyhedron_msg.header.frame_id = ros_param_.tracker_frame_id;
            corridor_publisher_.publish(polyhedron_msg);
        }
        p_base_->mutex_set_[1].unlock();
    }
}

bpmp::TrackingParam bpmp::RosWrapper::GetTrackingParam() {
    return planning_param_;
}
bpmp::VisualizationParam bpmp::RosWrapper::GetVisualizationParam() {
    return vis_param_;
}
void bpmp::RosWrapper::InitSubscriberAndPublisher() {
    // Subscribe Problem Materials
    target_prediction_subscriber_ = nh_.subscribe("/bpmp_predictor/target_prediction", 1,
                                                  &RosWrapper::TargetPredictionCallback, this);
    obstacle_state_list_subscriber_ = nh_.subscribe("/bpmp_simulator/obstacle_state_list", 1,
                                                    &RosWrapper::ObstacleStateListCallback, this);
    tracker_list_state_subscriber_ = nh_.subscribe("/base_odom", 1,
                                                   &RosWrapper::TrackerStateCallback, this);
    pcl_subscriber_ = nh_.subscribe("/bpmp_simulator/point_cloud_obstacle", 1, &RosWrapper::PclCallback, this);
    corridor_publisher_ = nh_.advertise<decomp_ros_msgs::PolyhedronArray>("corridor", 1);

    // Publish Primitives
    tracker_raw_primitives_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("raw_primitive", 1);
    tracker_feasible_primitives_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("feasible_primitives", 1);
    tracker_best_trajectory_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("best_primitives", 1);
    tracker_best_polystate_publisher_ = nh_.advertise<bpmp_tracker::PolyState>("best_polystate", 1);
    tracker_control_input_publisher_ = nh_.advertise<bpmp_tracker::UnicycleInput>("unicycle_control_input", 1);
    buffered_voronoi_cell_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("buffered_voronoi_cell",1);
    visibility_cell_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("visibility_cell",1);
}
void bpmp::RosWrapper::ObstacleStateListCallback(const bpmp_tracker::ObjectStateList &msg) {
    bpmp::State obstacle_state;
    p_base_->mutex_set_[0].lock();
    p_base_->current_obstacle_list_read_.clear();
    for (int i = 0; i < msg.object_state_list.size(); i++) {
        obstacle_state.px = msg.object_state_list[i].px;
        obstacle_state.py = msg.object_state_list[i].py;
        obstacle_state.pz = msg.object_state_list[i].pz;
        obstacle_state.vx = msg.object_state_list[i].vx;
        obstacle_state.vy = msg.object_state_list[i].vy;
        obstacle_state.vz = msg.object_state_list[i].vz;
        p_base_->current_obstacle_list_read_.push_back(obstacle_state);
    }
    p_base_->is_obstacle_info_ = true;
    p_base_->is_dynobs_received_ = true;
    p_base_->mutex_set_[0].unlock();
}
void bpmp::RosWrapper::TrackerStateCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    bpmp::UnicycleState tracker_state;
    p_base_->mutex_set_[0].lock();
    tracker_state.px = msg->pose.pose.position.x;
    tracker_state.py = msg->pose.pose.position.y;
    tracker_state.pz = msg->pose.pose.position.z;
    double q[4] {msg->pose.pose.orientation.w,msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z};
    tracker_state.theta = atan2(
            2.0 * (q[0] * q[3] + q[1] * q[2]),
            1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3])
    );
    p_base_->current_tracker_list_read_ = tracker_state;
    p_base_->is_tracker_info_ = true;
    p_base_->mutex_set_[0].unlock();
}
void bpmp::RosWrapper::TargetPredictionCallback(const bpmp_tracker::PolyState &msg) {
    bpmp::PrimitivePlanning target_trajectory;
    p_base_->mutex_set_[0].lock();
    p_base_->target_prediction_read_.t0 = msg.t0;
    p_base_->target_prediction_read_.tf = msg.tf;
    for (int i = 0; i < 4; i++) {
        p_base_->target_prediction_read_.ctrl_x[i] = msg.x_coeff[i];
        p_base_->target_prediction_read_.ctrl_y[i] = msg.y_coeff[i];
        p_base_->target_prediction_read_.ctrl_z[i] = msg.z_coeff[i];
    }
    p_base_->is_target_info_ = true;
    p_base_->mutex_set_[0].unlock();
}
bpmp::RosWrapper::RosWrapper(std::shared_ptr<bpmp::PlannerBase> p_base) : p_base_(p_base), nh_("~") {
    t0_ = ros::Time::now().toSec();
    // ROS PARAM
    nh_.param<double>("control_frequency", ros_param_.control_frequency, 100.0);
    nh_.param<double>("planning_frequency", ros_param_.planning_frequency, 30.0);
    nh_.param<string>("map_frame_id", ros_param_.map_frame_id, "map");
    nh_.param<string>("tracker_frame_id", ros_param_.tracker_frame_id, "base_link");

    // VISUALIZATION PARAM
    nh_.param<std::string>("map_frame_id", vis_param_.frame_id, "map");
    nh_.param<std::string>("tracker_frame_id", vis_param_.tracker_frame_id, "base_link");
    nh_.param<bool>("raw_primitives/publish", vis_param_.raw_primitives.publish, false);
    nh_.param<int>("raw_primitives/num_time_sample", vis_param_.raw_primitives.num_time_sample, 10);
    nh_.param<double>("raw_primitives/proportion", vis_param_.raw_primitives.proportion, 0.0);
    nh_.param<double>("raw_primitives/line_scale", vis_param_.raw_primitives.line_scale, 0.01);
    nh_.param<double>("raw_primitives/color_a", vis_param_.raw_primitives.color_a, 0.0);
    nh_.param<double>("raw_primitives/color_r", vis_param_.raw_primitives.color_r, 0.0);
    nh_.param<double>("raw_primitives/color_g", vis_param_.raw_primitives.color_g, 0.0);
    nh_.param<double>("raw_primitives/color_b", vis_param_.raw_primitives.color_b, 0.0);
    // Feasible primitives
    nh_.param<bool>("feasible_primitives/publish", vis_param_.feasible_primitives.publish, false);
    nh_.param<int>("feasible_primitives/num_time_sample", vis_param_.feasible_primitives.num_time_sample, 10);
    nh_.param<double>("feasible_primitives/proportion", vis_param_.feasible_primitives.proportion, 0.0);
    nh_.param<double>("feasible_primitives/line_scale", vis_param_.feasible_primitives.line_scale, 0.01);
    nh_.param<double>("feasible_primitives/color_a", vis_param_.feasible_primitives.color_a, 0.0);
    nh_.param<double>("feasible_primitives/color_r", vis_param_.feasible_primitives.color_r, 0.0);
    nh_.param<double>("feasible_primitives/color_g", vis_param_.feasible_primitives.color_g, 0.0);
    nh_.param<double>("feasible_primitives/color_b", vis_param_.feasible_primitives.color_b, 0.0);
    // Best primitive
    nh_.param<int>("best_primitive/num_time_sample", vis_param_.best_primitive.num_time_sample, 10);
    nh_.param<double>("best_primitive/line_scale", vis_param_.best_primitive.line_scale, 0.01);
    nh_.param<double>("best_primitive/color_a", vis_param_.best_primitive.color_a, 0.);
    nh_.param<double>("best_primitive/color_r", vis_param_.best_primitive.color_r, 0.);
    nh_.param<double>("best_primitive/color_g", vis_param_.best_primitive.color_g, 0.);
    nh_.param<double>("best_primitive/color_b", vis_param_.best_primitive.color_b, 0.);
    // Cell
    nh_.param<double>("cell/min_x",vis_param_.cell.axis_min.x,0.0);
    nh_.param<double>("cell/min_y",vis_param_.cell.axis_min.y,0.0);
    nh_.param<double>("cell/min_z",vis_param_.cell.axis_min.z,0.0);
    nh_.param<double>("cell/max_x",vis_param_.cell.axis_max.x,0.0);
    nh_.param<double>("cell/max_y",vis_param_.cell.axis_max.y,0.0);
    nh_.param<double>("cell/max_z",vis_param_.cell.axis_max.z,0.0);
    // Voronoi
    nh_.param<double>("cell/voronoi/color_a",vis_param_.cell.voronoi.color_a,0.0);
    nh_.param<double>("cell/voronoi/color_r",vis_param_.cell.voronoi.color_r,0.0);
    nh_.param<double>("cell/voronoi/color_g",vis_param_.cell.voronoi.color_g,0.0);
    nh_.param<double>("cell/voronoi/color_b",vis_param_.cell.voronoi.color_b,0.0);
    // Visibility
    nh_.param<double>("cell/visibility/color_a",vis_param_.cell.visibility.color_a,0.0);
    nh_.param<double>("cell/visibility/color_r",vis_param_.cell.visibility.color_r,0.0);
    nh_.param<double>("cell/visibility/color_g",vis_param_.cell.visibility.color_g,0.0);
    nh_.param<double>("cell/visibility/color_b",vis_param_.cell.visibility.color_b,0.0);

    visualizer_ = new PlanningVisualizer(vis_param_);
    // Planning
    nh_.param<double>("horizon", planning_param_.horizon, 1.0);
    nh_.param<int>("num_thread", planning_param_.num_thread, 1);
    nh_.param<double>("r_min", planning_param_.r_min, 1.0);
    nh_.param<double>("r_max", planning_param_.r_max, 2.0);
    nh_.param<double>("safe_distance", planning_param_.safe_distance, 0.0);

    nh_.param<int>("num_sample_planning", planning_param_.num_sample_planning, 10);
    nh_.param<double>("vel_max", planning_param_.vel_max, 0.5);
    nh_.param<double>("acc_max", planning_param_.acc_max, 10.0);
    nh_.param<double>("ang_max",planning_param_.ang_max,1.0);
    nh_.param<double>("fov",planning_param_.fov,0.0);
    nh_.param<double>("object_radius", planning_param_.object_radius, 0.15);
    nh_.param<double>("terminal_weight", planning_param_.terminal_weight, 0.0);
    nh_.param<double>("distance_max", planning_param_.distance_max, 1.0);
    nh_.param<bool>("is_experiment", planning_param_.is_experiment, false);
    nh_.param<bool>("is_unstructured",planning_param_.is_unstructured, false);
    nh_.param<int>("check_mode",planning_param_.check_mode,0);
    nh_.param<int>("sample_mode",planning_param_.sample_mode,0);

    nh_.param<double>("axis_limit/min_x", planning_param_.axis_limit.min_x, 0.0);
    nh_.param<double>("axis_limit/min_y", planning_param_.axis_limit.min_y, 0.0);
    nh_.param<double>("axis_limit/max_x", planning_param_.axis_limit.max_x, 0.0);
    nh_.param<double>("axis_limit/max_y", planning_param_.axis_limit.max_y, 0.0);

}
bpmp_tracker::UnicycleInput
bpmp::RosWrapper::GenerateControlInput(const std::vector<bpmp::PrimitivePlanning> &primitive,
                                       const uint &best_index, const double &time_t) {
    bpmp_tracker::UnicycleInput tracker_input;
    double T =
            primitive[best_index].tf - primitive[best_index].t0;
    double T2 = T * T;
    double T_inv = 1.0 / T;
    double T2_inv = 1.0 / T2;
    double pos_ctrl_x[4]{primitive[best_index].ctrl_x[0],
                         primitive[best_index].ctrl_x[1],
                         primitive[best_index].ctrl_x[2],
                         primitive[best_index].ctrl_x[3]};
    double pos_ctrl_y[4]{primitive[best_index].ctrl_y[0],
                         primitive[best_index].ctrl_y[1],
                         primitive[best_index].ctrl_y[2],
                         primitive[best_index].ctrl_y[3]};
    double vel_ctrl_x[3]{3.0 * T_inv * (primitive[best_index].ctrl_x[1] -
                                        primitive[best_index].ctrl_x[0]),
                         3.0 * T_inv * (primitive[best_index].ctrl_x[2] -
                                        primitive[best_index].ctrl_x[1]),
                         3.0 * T_inv * (primitive[best_index].ctrl_x[3] -
                                        primitive[best_index].ctrl_x[2])};
    double vel_ctrl_y[3]{3.0 * T_inv * (primitive[best_index].ctrl_y[1] -
                                        primitive[best_index].ctrl_y[0]),
                         3.0 * T_inv * (primitive[best_index].ctrl_y[2] -
                                        primitive[best_index].ctrl_y[1]),
                         3.0 * T_inv * (primitive[best_index].ctrl_y[3] -
                                        primitive[best_index].ctrl_y[2])};
    double acc_ctrl_x[2]{6.0 * T2_inv * (primitive[best_index].ctrl_x[2] -
                                         2.0 * primitive[best_index].ctrl_x[1] +
                                         primitive[best_index].ctrl_x[0]),
                         6.0 * T2_inv * (primitive[best_index].ctrl_x[3] -
                                         2.0 * primitive[best_index].ctrl_x[2] +
                                         primitive[best_index].ctrl_x[1])};
    double acc_ctrl_y[2]{6.0 * T2_inv * (primitive[best_index].ctrl_y[2] -
                                         2.0 * primitive[best_index].ctrl_y[1] +
                                         primitive[best_index].ctrl_y[0]),
                         6.0 * T2_inv * (primitive[best_index].ctrl_y[3] -
                                         2.0 * primitive[best_index].ctrl_y[2] +
                                         primitive[best_index].ctrl_y[1])};

    tracker_input.vel_linear = sqrt(std::pow(bpmp::getBernsteinValue(vel_ctrl_x, time_t, primitive[best_index].t0, primitive[best_index].tf, 2),2)+
                                    pow(bpmp::getBernsteinValue(vel_ctrl_y, time_t,primitive[best_index].t0, primitive[best_index].tf, 2),2));
    if(pos_ctrl_x[3]<0){
        tracker_input.vel_linear = -tracker_input.vel_linear;
//        tracker_input.vel_angular = 0.0;
//        tracker_input.vel_angular = -tracker_input.vel_angular;
    }

    if (abs(tracker_input.vel_linear)<1e-4){
        tracker_input.vel_linear = 0.0;
        tracker_input.vel_angular = 0.0;
    }
    else{
        double tracker_input_angular_num = bpmp::getBernsteinValue(acc_ctrl_y,time_t,primitive[best_index].t0,primitive[best_index].tf,1)*bpmp::getBernsteinValue(vel_ctrl_x,time_t,primitive[best_index].t0,primitive[best_index].tf,2)-
                                           bpmp::getBernsteinValue(acc_ctrl_x,time_t,primitive[best_index].t0,primitive[best_index].tf,1)*bpmp::getBernsteinValue(vel_ctrl_y,time_t,primitive[best_index].t0,primitive[best_index].tf,2);
        double tracker_input_angular_den = std::pow(bpmp::getBernsteinValue(vel_ctrl_x, time_t, primitive[best_index].t0, primitive[best_index].tf, 2),2)+
                                           pow(bpmp::getBernsteinValue(vel_ctrl_y, time_t,primitive[best_index].t0, primitive[best_index].tf, 2),2);
        tracker_input.vel_angular = tracker_input_angular_num/tracker_input_angular_den;
    }
    static bpmp_tracker::UnicycleInput previous_output = tracker_input;
    tracker_input.vel_linear = 0.1*previous_output.vel_linear+0.9*tracker_input.vel_linear;
    tracker_input.vel_angular = 0.1*previous_output.vel_angular+0.9*tracker_input.vel_angular;
    tracker_input.vel_linear = min(2.5,max(-0.5,tracker_input.vel_linear));
    tracker_input.vel_angular = min(3.141592,max(-3.141592,tracker_input.vel_angular));
    return tracker_input;
}

void bpmp::RosWrapper::PclCallback(const sensor_msgs::PointCloud2_<std::allocator<void>>::ConstPtr &pcl_msgs) {
    p_base_->mutex_set_[0].lock();
    p_base_->point_cloud_3d_.clear();
    p_base_->mutex_set_[0].unlock();

    sensor_msgs::PointCloud pcl;
    if (not pcl_msgs->fields.empty()) {
        sensor_msgs::convertPointCloud2ToPointCloud(*pcl_msgs, pcl);
        pcl.header.frame_id = pcl_msgs->header.frame_id;
        pcl.header.stamp = pcl_msgs->header.stamp;
        vec_Vec3f  obst = DecompROS::cloud_to_vec(pcl);
        p_base_->mutex_set_[0].lock();
        p_base_->point_cloud_3d_ = obst;
        p_base_->is_obstacle_info_ = true;
        p_base_->is_pcl_received_ = true;
        p_base_->mutex_set_[0].unlock();
        //cout<<"GOT PCL FROM SIMULATOR"<<endl;
    }
    else{
        p_base_->mutex_set_[0].lock();
        p_base_->is_obstacle_info_ = false;
        p_base_->is_pcl_received_ = false;
        p_base_->mutex_set_[0].unlock();
    }
}
