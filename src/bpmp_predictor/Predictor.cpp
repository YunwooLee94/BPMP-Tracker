#include "bpmp_predictor/Predictor.h"

bpmp::Predictor::Predictor():nh_("~") {
    // Prediction
    nh_.param<int>("num_thread", param_.num_thread, 1);
    nh_.param<int>("num_sample", param_.num_sample, 1);
    nh_.param<double>("horizon", param_.horizon, 1.0);
    nh_.param<double>("acc_max_target", param_.acc_max_target, 0.1);
    nh_.param<double>("target_size",param_.target_size,0.3);
    // Visualization
    nh_.param<string>("frame_id", vis_param_.frame_id, "map");
    // Raw primitives
    nh_.param<bool>("raw_primitives/publish", vis_param_.raw_primitives.publish, "false");
    nh_.param<int>("raw_primitives/num_time_sample", vis_param_.raw_primitives.num_time_sample, 10);
    nh_.param<double>("raw_primitives/proportion", vis_param_.raw_primitives.proportion, 0.0);
    nh_.param<double>("raw_primitives/line_scale", vis_param_.raw_primitives.line_scale, 0.01);
    nh_.param<double>("raw_primitives/color_a", vis_param_.raw_primitives.color_a, 0.);
    nh_.param<double>("raw_primitives/color_r", vis_param_.raw_primitives.color_r, 0.);
    nh_.param<double>("raw_primitives/color_g", vis_param_.raw_primitives.color_g, 0.);
    nh_.param<double>("raw_primitives/color_b", vis_param_.raw_primitives.color_b, 0.);
    // Feasible primitives
    nh_.param<bool>("feasible_primitives/publish", vis_param_.feasible_primitives.publish, "false");
    nh_.param<int>("feasible_primitives/num_time_sample", vis_param_.feasible_primitives.num_time_sample, 10);
    nh_.param<double>("feasible_primitives/proportion", vis_param_.feasible_primitives.proportion, 0.0);
    nh_.param<double>("feasible_primitives/line_scale", vis_param_.feasible_primitives.line_scale, 0.01);
    nh_.param<double>("feasible_primitives/color_a", vis_param_.feasible_primitives.color_a, 0.);
    nh_.param<double>("feasible_primitives/color_r", vis_param_.feasible_primitives.color_r, 0.);
    nh_.param<double>("feasible_primitives/color_g", vis_param_.feasible_primitives.color_g, 0.);
    nh_.param<double>("feasible_primitives/color_b", vis_param_.feasible_primitives.color_b, 0.);
    // Best primitive
    nh_.param<int>("best_primitive/num_time_sample", vis_param_.best_primitive.num_time_sample, 10);
    nh_.param<double>("best_primitive/line_scale", vis_param_.best_primitive.line_scale, 0.01);
    nh_.param<double>("best_primitive/color_a", vis_param_.best_primitive.color_a, 0.);
    nh_.param<double>("best_primitive/color_r", vis_param_.best_primitive.color_r, 0.);
    nh_.param<double>("best_primitive/color_g", vis_param_.best_primitive.color_g, 0.);
    nh_.param<double>("best_primitive/color_b", vis_param_.best_primitive.color_b, 0.);

    visualizer_.UpdateParameter(vis_param_);
    target_state_subscriber_ = nh_.subscribe("/bpmp_simulator/target_state", 1, &Predictor::TargetStateCallback, this);
    pcl_subscriber_ = nh_.subscribe("/bpmp_simulator/point_cloud_obstacle", 1, &Predictor::PclCallback, this);
    obstacle_state_list_subscriber_ = nh_.subscribe("/bpmp_simulator/obstacle_state_list", 1,
                                                    &Predictor::ObstacleStateListCallback, this);

    raw_primitive_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("TargetRawPrimitive", 1);
    feasible_primitive_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("TargetFeasiblePrimitive", 1);
    prediction_result_publisher_ = nh_.advertise<bpmp_tracker::PolyState>("target_prediction", 1);

    best_primitive_publisher_ = nh_.advertise<visualization_msgs::Marker>("TargetBestPrimitive", 1);
    corridor_publisher_ = nh_.advertise<decomp_ros_msgs::PolyhedronArray>("corridor", 1);
}

void bpmp::Predictor::Run() {
    ros::Rate loop_rate(100.0);
    while (ros::ok()) {
        if (Predict())
            UpdateResult();
        else
            EraseResult();
        ros::spinOnce();
        loop_rate.sleep();
    }
}
void bpmp::Predictor::EraseResult() {

}
void bpmp::Predictor::TargetStateCallback(const bpmp_tracker::ObjectState &msg) {
    current_target_state_.px = msg.px;
    current_target_state_.py = msg.py;
    current_target_state_.pz = msg.pz;
    current_target_state_.vx = msg.vx;
    current_target_state_.vy = msg.vy;
    current_target_state_.vz = msg.vz;
}

bool bpmp::Predictor::Predict() {
    SampleEndPoints();
    GeneratePrimitives();
    vector<bpmp::uint> id_array;
    for(int i=0;i<param_.num_sample;i++)
        id_array.push_back(i);
    switch (EnvironmentMode()){
        case 0:{
            safe_index_.clear();
            safe_index_ = GetSafeIndexUnstructured(id_array); // Target Distance + Collision and Occlusion Avoidance against Obstacles
            break;
        }
        case 1:{
            safe_index_.clear();
            safe_index_ = GetSafeIndexDynamic(id_array);
            break;
        }
        case 2:{
            safe_index_.clear();
            std::vector<uint> temp_safe_index = GetSafeIndexUnstructured(id_array);
            safe_index_ = GetSafeIndexDynamic(temp_safe_index);
            break;
        }
        default:{
            cout<<"CheckModeERROR"<<endl;
            return false;
        }
    }
    if (safe_index_.empty())
        return false;
    GetBestIndex();
    return true;
}
void bpmp::Predictor::UpdateResult() {
    if (not primitive_.empty())
        raw_primitive_publisher_.publish(visualizer_.VisualizeRawPrimitives(primitive_));
    if (not primitive_.empty() and not safe_index_.empty())
        feasible_primitive_publisher_.publish(visualizer_.VisualizeFeasiblePrimitives(primitive_, safe_index_));
    if (not primitive_.empty() and not safe_index_.empty()) {
        best_primitive_publisher_.publish(visualizer_.VisualizeBestPrimitive(primitive_, best_prediction_index_));
        // Polynomial Description
        bpmp_tracker::PolyState result;
        result.t0 = GetCurrentTime();
        result.tf = GetCurrentTime()+param_.horizon;
        for (int i = 0; i < 4; i++) {
            result.x_coeff.push_back(primitive_[best_prediction_index_].ctrl_x[i]);
            result.y_coeff.push_back(primitive_[best_prediction_index_].ctrl_y[i]);
            result.z_coeff.push_back(primitive_[best_prediction_index_].ctrl_z[i]);
        }
        prediction_result_publisher_.publish(result);
    }
    if (not polys_.empty()) {
        decomp_ros_msgs::PolyhedronArray polyhedron_msg = DecompROS::polyhedron_array_to_ros(polys_);
        polyhedron_msg.header.frame_id = vis_param_.frame_id;
        corridor_publisher_.publish(polyhedron_msg);
    }
}

void bpmp::Predictor::SampleEndPoints() {
    end_points_.clear();
    int num_chunk = param_.num_sample / param_.num_thread;
    std::vector<thread> worker_thread;
    vector<vector<Point>> end_points_temp(param_.num_thread);
    for (int i = 0; i < param_.num_thread; i++)
        worker_thread.emplace_back(
                thread(&Predictor::SampleEndPointsSubProcess, this, num_chunk * i, num_chunk * (i + 1),
                       std::ref(end_points_temp[i])));
    for (int i = 0; i < param_.num_thread; i++)
        worker_thread[i].join();
    for (int i = 0; i < param_.num_thread; i++) {
        for (int j = 0; j < end_points_temp[i].size(); j++)
            end_points_.push_back(end_points_temp[i][j]);
    }
    if (not end_points_.empty()) {
        end_points_[0].x = current_target_state_.px + current_target_state_.vx * param_.horizon;
        end_points_[0].y = current_target_state_.py + current_target_state_.vy * param_.horizon;
        end_points_[0].z = current_target_state_.pz + current_target_state_.vz * param_.horizon;
    }
}

void bpmp::Predictor::SampleEndPointsSubProcess(const int &start_idx, const int &end_idx,
                                                std::vector<bpmp::Point> &endpoint_list_sub) {
    bpmp::Point end_point_center{current_target_state_.px + current_target_state_.vx * param_.horizon,
                                 current_target_state_.py + current_target_state_.vy * param_.horizon,
                                 current_target_state_.pz + current_target_state_.vz * param_.horizon};
    const bpmp::uint n_cols = 3;
    const bpmp::uint n_rows = end_idx - start_idx;

    using namespace Eigen;
    Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> gaussian_data_eigen;
    gaussian_data_eigen.setZero(n_rows, n_cols);
    Eigen::Vector3f mean;
    Eigen::Matrix3f covar;
    mean << (float) end_point_center.x, (float) end_point_center.y, (float) end_point_center.z;
    covar << (float) (0.5 * 0.333333 * param_.acc_max_target * param_.horizon * param_.horizon), 0, 0, 0,
            (float) (0.5 * 0.333333 * param_.acc_max_target * param_.horizon * param_.horizon),0, 0, 0, (float) (0.5 * 0.333333 * param_.acc_max_target * param_.horizon * param_.horizon);
    Eigen::EigenMultivariateNormal<float> normX_solver1(mean, covar);
    gaussian_data_eigen << normX_solver1.samples(n_rows).transpose();
    Point tempPoint{end_point_center.x, end_point_center.y, end_point_center.z};
    for (int i = 0; i < n_rows; i++) {
        tempPoint.x = gaussian_data_eigen.coeffRef(i, 0);
        tempPoint.y = gaussian_data_eigen.coeffRef(i, 1);
        tempPoint.z = end_point_center.z;
        endpoint_list_sub.push_back(tempPoint);
    }
}

void bpmp::Predictor::GeneratePrimitives() {
    primitive_.clear();
    int num_chunk = end_points_.size() / param_.num_thread;
    std::vector<thread> worker_thread;
    vector<vector<PrimitiveTarget>> primitive_temp(param_.num_thread);

    for (int i = 0; i < param_.num_thread; i++)
        worker_thread.emplace_back(
                thread(&Predictor::GeneratePrimitivesSubProcess, this, num_chunk * i, num_chunk * (i + 1),
                       std::ref(primitive_temp[i])));
    for (int i = 0; i < param_.num_thread; i++)
        worker_thread[i].join();
    for (int i = 0; i < param_.num_thread; i++) {
        for (int j = 0; j < primitive_temp[i].size(); j++)
            primitive_.push_back(primitive_temp[i][j]);
    }
}

void bpmp::Predictor::GeneratePrimitivesSubProcess(const int &start_idx, const int &end_idx,
                                                   std::vector<bpmp::PrimitiveTarget> &primitive_sub) {
    PrimitiveTarget temp_primitive;
    temp_primitive.t0 = 0.0;
    temp_primitive.tf = param_.horizon;
    double T = temp_primitive.tf - temp_primitive.t0;
    for (int i = start_idx; i < end_idx; i++) {
        temp_primitive.ctrl_x[0] = current_target_state_.px;
        temp_primitive.ctrl_x[1] = current_target_state_.px + 0.33333333 * current_target_state_.vx * T;
        temp_primitive.ctrl_x[2] =
                0.5 * current_target_state_.px + 0.5 * end_points_[i].x + 0.16666667 * current_target_state_.vx * T;
        temp_primitive.ctrl_x[3] = end_points_[i].x;

        temp_primitive.ctrl_y[0] = current_target_state_.py;
        temp_primitive.ctrl_y[1] = current_target_state_.py + 0.33333333 * current_target_state_.vy * T;
        temp_primitive.ctrl_y[2] =
                0.5 * current_target_state_.py + 0.5 * end_points_[i].y + 0.16666667 * current_target_state_.vy * T;
        temp_primitive.ctrl_y[3] = end_points_[i].y;

        temp_primitive.ctrl_z[0] = current_target_state_.pz;
        temp_primitive.ctrl_z[1] = current_target_state_.pz + 0.33333333 * current_target_state_.vz * T;
        temp_primitive.ctrl_z[2] =
                0.5 * current_target_state_.pz + 0.5 * end_points_[i].z + 0.16666667 * current_target_state_.vz * T;
        temp_primitive.ctrl_z[3] = end_points_[i].z;
        primitive_sub.push_back(temp_primitive);
    }
}

std::vector<bpmp::uint> bpmp::Predictor::GetSafeIndexUnstructured(const vector<uint> &prior_idx) {
    vector<bpmp::uint> index;
    int num_chunk = prior_idx.size() / param_.num_thread;
    std::vector<thread> worker_thread;
    vector<vector<bpmp::uint>> safe_index_temp(param_.num_thread);

    LinearConstraint3D corridor_constraints = GenerateCorridor();
    for (int i = 0; i < param_.num_thread; i++)
        worker_thread.emplace_back(
                thread(&Predictor::GetSafeIndexUnstructuredSubProcess, this, corridor_constraints,prior_idx, num_chunk * i, num_chunk * (i + 1),
                       std::ref(safe_index_temp[i])));

    for (int i = 0; i < param_.num_thread; i++)
        worker_thread[i].join();
    for (int i = 0; i < param_.num_thread; i++) {
        for (int j = 0; j < safe_index_temp[i].size(); j++)
            index.push_back(safe_index_temp[i][j]);
    }
    return index;
}

void bpmp::Predictor::GetSafeIndexUnstructuredSubProcess(const LinearConstraint3D &constraint, const vector<uint> &prior_idx, const int &start_idx,
                                                         const int &end_idx, std::vector<uint> &safe_index_sub) {
    Eigen::Vector3d A_comp_temp{0.0,0.0,0.0};
    double b_comp_temp(0.0);
    vector<Eigen::Vector3d> LinearConstraintA;
    vector<double> LinearConstraintb;
    int num_constraint = (int)constraint.A().rows();
    int num_var = (int)constraint.A().cols();

    for(int i = 0;i<num_constraint;i++){
        A_comp_temp[0] = constraint.A().coeffRef(i,0);
        A_comp_temp[1] = constraint.A().coeffRef(i,1);
        A_comp_temp[2] = constraint.A().coeffRef(i,2);
        b_comp_temp = constraint.b().coeffRef(i,0);
        LinearConstraintA.push_back(A_comp_temp);
        LinearConstraintb.push_back(b_comp_temp);
    }
    bool flag_store_in = true;
    bool flag_store_out = true;
    double value;

    for (int idx = start_idx; idx < end_idx; idx++) {
        flag_store_out = true;
        for(int j =0;j<(int)LinearConstraintA.size();j++){
            flag_store_in = true;
            for(int k=0;k<4;k++){
                value = LinearConstraintA[j][0]*primitive_[prior_idx[idx]].ctrl_x[k]+LinearConstraintA[j][1]*primitive_[prior_idx[idx]].ctrl_y[k]+LinearConstraintA[j][2]*primitive_[prior_idx[idx]].ctrl_z[k]
                        -LinearConstraintb[j]+param_.target_size;
                if(value>0.0){
                    flag_store_in = false;
                    break;
                }
            }
            if(not flag_store_in){
                flag_store_out = false;
                break;
            }
        }
        if (flag_store_in and flag_store_out)
            safe_index_sub.push_back(prior_idx[idx]);
    }
}

void bpmp::Predictor::GetBestIndex() {
    int num_chunk = safe_index_.size() / param_.num_thread;
    vector<thread> worker_thread;
    vector<std::pair<uint, double>> min_dist_pair_temp(param_.num_thread);
    for (int i = 0; i < param_.num_thread; i++) {
        worker_thread.emplace_back(thread(&Predictor::GetBestIndexSubProcess, this, num_chunk * i, num_chunk * (i + 1),
                                          std::ref(min_dist_pair_temp[i])));
    }
    for (int i = 0; i < param_.num_thread; i++)
        worker_thread[i].join();
    double min_dist_temp = 99999999999999.0;
    for (int i = 0; i < param_.num_thread; i++) {
        if (min_dist_temp > min_dist_pair_temp[i].second) {
            min_dist_temp = min_dist_pair_temp[i].second;
            best_prediction_index_ = min_dist_pair_temp[i].first;
        }
    }
}

void bpmp::Predictor::GetBestIndexSubProcess(const int &start_idx, const int &end_idx,
                                             std::pair<uint, double> &best_index_sub) {
    int chunk_size = end_idx - start_idx;
    double distance_sum_list[chunk_size];
    for (int idx = 0; idx < chunk_size; idx++) {
        distance_sum_list[idx] = 0.0;
        for (int i = 0; i < safe_index_.size(); i++) {
            distance_sum_list[idx] +=
                    pow(primitive_[safe_index_[idx + start_idx]].ctrl_x[3] - primitive_[safe_index_[i]].ctrl_x[3], 2) +
                    pow(primitive_[safe_index_[idx + start_idx]].ctrl_y[3] - primitive_[safe_index_[i]].ctrl_y[3], 2);
        }
    }
    best_index_sub.second = 9999999999.0;
    for (int idx = 0; idx < chunk_size; idx++) {
        if (best_index_sub.second > distance_sum_list[idx]) {
            best_index_sub.second = distance_sum_list[idx];
            best_index_sub.first = safe_index_[idx + start_idx];
        }
    }
}

void bpmp::Predictor::PclCallback(const sensor_msgs::PointCloud2_<std::allocator<void>>::ConstPtr &pcl_msgs) {
    point_cloud_3d_.clear();
    sensor_msgs::PointCloud pcl;
    if (not pcl_msgs->fields.empty())
        is_pcl_received_ = true;
    else
        is_pcl_received_ = false;

    if (not pcl_msgs->fields.empty()) {
        sensor_msgs::convertPointCloud2ToPointCloud(*pcl_msgs, pcl);
        pcl.header.frame_id = pcl_msgs->header.frame_id;
        pcl.header.stamp = pcl_msgs->header.stamp;
    }
    vec_Vec3f obs = DecompROS::cloud_to_vec(pcl);
    for (const auto &it: obs)
        point_cloud_3d_.push_back(it.topRows<3>());
}

LinearConstraint3D bpmp::Predictor::GenerateCorridor() {
    Eigen::Matrix<double, 3, 1> temp_pose_eigen;
    temp_pose_eigen << current_target_state_.px, current_target_state_.py, current_target_state_.pz;
    Vec3f seed = temp_pose_eigen;
    SeedDecomp3D decomp_util(seed);
    decomp_util.set_obs(point_cloud_3d_);
    decomp_util.set_local_bbox(Vec3f(3.0, 3.0, 2.0));
    decomp_util.dilate(param_.target_size);

    polys_.clear();
    polys_.push_back(decomp_util.get_polyhedron());
    auto poly_hedrons = decomp_util.get_polyhedron();
    LinearConstraint3D corridor_constraint(temp_pose_eigen, poly_hedrons.hyperplanes());
    return corridor_constraint;
}

int bpmp::Predictor::EnvironmentMode() {
        int mode = 0; // 0: unstructured static, 1: dynamic, 2: static+dynamic
        if(is_dyn_obs_received_)
            mode = 1;
        if(is_pcl_received_ and is_dyn_obs_received_)
            mode = 2;
        //cout<<"MODE: "<<mode<<endl;
        return mode;
}

void bpmp::Predictor::ObstacleStateListCallback(const bpmp_tracker::ObjectStateList &msg) {
    bpmp::State obstacle_state;
    current_obstacle_primitive_list_.clear();
    if(msg.object_state_list.empty())
        return;
    PrimitiveTarget temp_primitive;
    for (int i = 0; i < msg.object_state_list.size(); i++) {
        obstacle_state.px = msg.object_state_list[i].px;
        obstacle_state.py = msg.object_state_list[i].py;
        obstacle_state.pz = msg.object_state_list[i].pz;
        obstacle_state.vx = msg.object_state_list[i].vx;
        obstacle_state.vy = msg.object_state_list[i].vy;
        obstacle_state.vz = msg.object_state_list[i].vz;
        temp_primitive.ctrl_x[0] = obstacle_state.px;
        temp_primitive.ctrl_x[1] = obstacle_state.px+0.33333333*obstacle_state.vx*param_.horizon;
        temp_primitive.ctrl_x[2] = obstacle_state.px+0.66666667*obstacle_state.vx*param_.horizon;
        temp_primitive.ctrl_x[3] = obstacle_state.px+obstacle_state.vx*param_.horizon;
        temp_primitive.ctrl_y[0] = obstacle_state.py;
        temp_primitive.ctrl_y[1] = obstacle_state.py+0.33333333*obstacle_state.vy*param_.horizon;
        temp_primitive.ctrl_y[2] = obstacle_state.py+0.66666667*obstacle_state.vy*param_.horizon;
        temp_primitive.ctrl_y[3] = obstacle_state.py+obstacle_state.vy*param_.horizon;
        temp_primitive.ctrl_z[0] = obstacle_state.pz;
        temp_primitive.ctrl_z[1] = obstacle_state.pz+0.33333333*obstacle_state.vz*param_.horizon;
        temp_primitive.ctrl_z[2] = obstacle_state.pz+0.66666667*obstacle_state.vz*param_.horizon;
        temp_primitive.ctrl_z[3] = obstacle_state.pz+obstacle_state.vz*param_.horizon;
        current_obstacle_primitive_list_.push_back(temp_primitive);
    }
}

std::vector<bpmp::uint>  bpmp::Predictor::GetSafeIndexDynamic(const std::vector<uint> &prior_idx) {
    vector<bpmp::uint> index;
    int num_chunk = prior_idx.size() / param_.num_thread;
    std::vector<thread> worker_thread;
    vector<vector<bpmp::uint>> safe_index_temp(param_.num_thread);
    for (int i = 0; i < param_.num_thread; i++)
        worker_thread.emplace_back(
                thread(&Predictor::GetSafeIndexDynamicSubProcess, this, prior_idx, num_chunk * i, num_chunk * (i + 1),
                       std::ref(safe_index_temp[i])));

    for (int i = 0; i < param_.num_thread; i++)
        worker_thread[i].join();
    for (int i = 0; i < param_.num_thread; i++) {
        for (int j = 0; j < safe_index_temp[i].size(); j++)
            index.push_back(safe_index_temp[i][j]);
    }
    return index;
}

void bpmp::Predictor::GetSafeIndexDynamicSubProcess(const std::vector<uint> &prior_idx, const int &start_idx,
                                                    const int &end_idx, std::vector<uint> &safe_index_sub) {
    double safe_distance_squared = pow(2 * param_.target_size, 2);
    double relative_target_pos_x[4], relative_target_pos_y[4]; //obstacle-target
    bool flag_store_in;
    bool flag_store_out;

    double value;
    for (int idx = start_idx; idx < end_idx; idx++) {
        flag_store_in = true;
        flag_store_out = true;
        for(int obs_idx=0;obs_idx<current_obstacle_primitive_list_.size();obs_idx++){
            for (int j = 0; j < 4; j++) {
                relative_target_pos_x[j] = primitive_[prior_idx[idx]].ctrl_x[j] - current_obstacle_primitive_list_[obs_idx].ctrl_x[j];
                relative_target_pos_y[j] = primitive_[prior_idx[idx]].ctrl_y[j] - current_obstacle_primitive_list_[obs_idx].ctrl_y[j];
            }
            for (int j = 0; j <= 6; j++) {  // Collision between obstacle and tracker
                value = 0.0f;
                for (int k = std::max(0, j - 3); k <= std::min(3, j); k++) {
                    value += (double) nchooser(3, k) * (double) nchooser(3, j - k) /
                             (double) nchooser(6, j) *
                             (relative_target_pos_x[k] * relative_target_pos_x[j - k] +
                                     relative_target_pos_y[k] * relative_target_pos_y[j - k]
                             );
                }
                if (value < safe_distance_squared) {
                    flag_store_in = false;
                    break;
                }
            }
            if (not flag_store_in) {
                flag_store_out = false;
                break;
            }
        }
        if(flag_store_out)
            safe_index_sub.push_back(prior_idx[idx]);
    }

}
