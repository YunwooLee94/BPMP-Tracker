//
// Created by larr-laptop on 25. 6. 9.
//
#include <bpmp_tracker/Tracker.h>

bool bpmp::Tracker::Plan(const double &t_trigger) {
    if (CheckInfoAvailable())
        UpdateValue(t_trigger);
    else
        return false;
    if(not SampleEndPoint()){
        return false;
    }
    GeneratePrimitive(t_trigger);
    GetFOVIndex(); // Field-of-View Constraints
    bool pass_test0 = true;
    if(visible_index_.empty()){
        pass_test0 = false;
    }
    if(pass_test0){
        switch (EnvironmentMode()){
            case 0:{
                safe_index_.clear();
                safe_index_ = GetSafeIndexUnstructured(visible_index_); // Target Distance + Collision and Occlusion Avoidance against Obstacles
                break;
            }
            case 1:{
                safe_index_.clear();
                safe_index_ = GetSafeIndexDynamic(visible_index_);
                break;
            }
            case 2:{
                safe_index_.clear();
                std::vector<uint> temp_safe_index = GetSafeIndexUnstructured(visible_index_);
                safe_index_ = GetSafeIndexDynamic(temp_safe_index);
                break;
            }
            default:{
                cout<<"CheckModeERROR"<<endl;
                return false;

            }
        }
    }
    else
        return false;
    bool pass_test1 = true;
    if(safe_index_.empty()){
        pass_test1 = false;
    }
    if (pass_test1) {
        GetDynamicallyFeasibleIndex();
    } else{
        return false;
    }

    bool pass_test2 = true;
    if (dynamically_feasible_index_.empty()) {
        pass_test2 = false;
    }
    if (pass_test2) {
        GetBestIndex();
    } else{
        return false;
    }
    return pass_test2;
}

bpmp::Tracker::Tracker(const bpmp::TrackingParam &param, std::shared_ptr<bpmp::PlannerBase> p_base) : param_(param),
                                                                                                      p_base_(p_base) {
}

void bpmp::Tracker::UpdateValue(const double &t) {
    {   // Tracker State
        p_base_->mutex_set_[0].lock();
        current_tracker_state_ = p_base_->current_tracker_list_read_;
        p_base_->mutex_set_[0].unlock();
        // T: Ground-> Robot
        current_pose_mat_.setIdentity();
        current_pose_mat_.translate(Eigen::Vector3d(current_tracker_state_.px,current_tracker_state_.py,current_tracker_state_.pz));
        current_pose_mat_.rotate(Eigen::Quaterniond(cos(0.5*current_tracker_state_.theta),0.0,0.0,sin(0.5*current_tracker_state_.theta)));
    }
    {   // Obstacle State
        obstacle_primitive_list_.clear();
        bpmp::PrimitivePlanning temp_primitive;
        temp_primitive.t0 = t;
        temp_primitive.tf = t + param_.horizon;
        p_base_->mutex_set_[0].lock();
        num_obstacle_ = (int) p_base_->current_obstacle_list_read_.size();
        for (int i = 0; i < num_obstacle_; i++) {
            temp_primitive.ctrl_x[0] = p_base_->current_obstacle_list_read_[i].px;
            temp_primitive.ctrl_x[1] = p_base_->current_obstacle_list_read_[i].px +
                                       0.33333333 * p_base_->current_obstacle_list_read_[i].vx * param_.horizon;
            temp_primitive.ctrl_x[2] = p_base_->current_obstacle_list_read_[i].px +
                                       0.66666667 * p_base_->current_obstacle_list_read_[i].vx * param_.horizon;
            temp_primitive.ctrl_x[3] = p_base_->current_obstacle_list_read_[i].px +
                                       p_base_->current_obstacle_list_read_[i].vx * param_.horizon;
            temp_primitive.ctrl_y[0] = p_base_->current_obstacle_list_read_[i].py;
            temp_primitive.ctrl_y[1] = p_base_->current_obstacle_list_read_[i].py +
                                       0.33333333 * p_base_->current_obstacle_list_read_[i].vy * param_.horizon;
            temp_primitive.ctrl_y[2] = p_base_->current_obstacle_list_read_[i].py +
                                       0.66666667 * p_base_->current_obstacle_list_read_[i].vy * param_.horizon;
            temp_primitive.ctrl_y[3] = p_base_->current_obstacle_list_read_[i].py +
                                       p_base_->current_obstacle_list_read_[i].vy * param_.horizon;
            temp_primitive.ctrl_z[0] = p_base_->current_obstacle_list_read_[i].pz;
            temp_primitive.ctrl_z[1] = p_base_->current_obstacle_list_read_[i].pz +
                                       0.33333333 * p_base_->current_obstacle_list_read_[i].vz * param_.horizon;
            temp_primitive.ctrl_z[2] = p_base_->current_obstacle_list_read_[i].pz +
                                       0.66666667 * p_base_->current_obstacle_list_read_[i].vz * param_.horizon;
            temp_primitive.ctrl_z[3] = p_base_->current_obstacle_list_read_[i].pz +
                                       p_base_->current_obstacle_list_read_[i].vz * param_.horizon;
            obstacle_primitive_list_.push_back(temp_primitive);
        }
        p_base_->mutex_set_[0].unlock();
//	cout<<"OBSTACLE LIST SIZE: "<<obstacle_primitive_list_.size()<<endl;
    }
    {   // Target State
        vector<Eigen::Vector3d> target_ctrl_points;
        p_base_->mutex_set_[0].lock();
        target_trajectory_.t0 = t;
        target_trajectory_.tf = t + param_.horizon;
        for (int i = 0; i < 4; i++) {
            Eigen::Vector3d ctrl_pts_global (p_base_->target_prediction_read_.ctrl_x[i],p_base_->target_prediction_read_.ctrl_y[i],p_base_->target_prediction_read_.ctrl_z[i]);
            Eigen::Vector3d ctrl_pts_local = current_pose_mat_.inverse()*ctrl_pts_global;
            target_trajectory_.ctrl_x[i] = ctrl_pts_local[0];
            target_trajectory_.ctrl_y[i] = ctrl_pts_local[1];
            target_trajectory_.ctrl_z[i] = ctrl_pts_local[2];
        }
        p_base_->mutex_set_[0].unlock();
    }
    {
        bool pcl_received=false;
        p_base_->mutex_set_[0].lock();
        if(p_base_->is_pcl_received_){
            pcl_received = true;
        }
        p_base_->mutex_set_[0].unlock();
        if(pcl_received){
            vec_Vec3f point_cloud_3d_temp;
            p_base_->mutex_set_[0].lock();
            point_cloud_3d_temp = p_base_->point_cloud_3d_;
            p_base_->mutex_set_[0].unlock();
            Eigen::Vector3d pcl_pts_global;
            Eigen::Vector3d pcl_pts_local;
            Eigen::Matrix<decimal_t, 3, 1> pcl_pts_local_3d;
            Eigen::Transform<double,3,Eigen::Affine> current_pose_mat_inverse = current_pose_mat_.inverse();
            point_cloud_3d_.clear();
            for(int i =0;i<point_cloud_3d_temp.size();i++){
                pcl_pts_global[0] = point_cloud_3d_temp[i][0];
                pcl_pts_global[1] = point_cloud_3d_temp[i][1];
                pcl_pts_global[2] = point_cloud_3d_temp[i][2];
                pcl_pts_local = current_pose_mat_inverse*pcl_pts_global;
                pcl_pts_local_3d[0] = (float)pcl_pts_local[0];
                pcl_pts_local_3d[1] = (float)pcl_pts_local[1];
                pcl_pts_local_3d[2] = (float)pcl_pts_local[2];
                point_cloud_3d_.push_back(pcl_pts_local_3d);
            }
        }
    }
}

bool bpmp::Tracker::CheckInfoAvailable() {
    p_base_->mutex_set_[0].lock();
    bool do_plan = p_base_->is_tracker_info_ and p_base_->is_target_info_ and p_base_->is_obstacle_info_;
    p_base_->mutex_set_[0].unlock();
    return do_plan;
}

bool bpmp::Tracker::SampleEndPoint() {
    end_points_.clear();
    int num_chunk = param_.num_sample_planning / param_.num_thread;
    vector<thread> worker_thread;
    vector<vector<bpmp::Point>> end_point_temp(param_.num_thread);
    for (int j = 0; j < param_.num_thread; j++)
        worker_thread.emplace_back(
                thread(&Tracker::SampleEndPointThread, this, num_chunk * j, num_chunk * (j + 1),
                       std::ref(end_point_temp[j])));
    for (int j = 0; j < param_.num_thread; j++)
        worker_thread[j].join();
    for (int j = 0; j < param_.num_thread; j++) {
        for(int k =0;k<end_point_temp[j].size();k++)
            end_points_.push_back(end_point_temp[j][k]);
    }
    if(end_points_.size()<param_.num_sample_planning)
        return false;
    else
        return true;
}

void bpmp::Tracker::SampleEndPointThread(const int &start_idx, const int &end_idx,
                                         std::vector<bpmp::Point> &endpoint_list_sub) {
    bpmp::Point end_point_center{target_trajectory_.ctrl_x[3], target_trajectory_.ctrl_y[3],
                                 target_trajectory_.ctrl_z[3]};
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> r_dis(param_.r_min, param_.r_max);
    double center_angle;
    double qx = target_trajectory_.ctrl_x[3]-target_trajectory_.ctrl_x[2];
    double qy = target_trajectory_.ctrl_y[3]-target_trajectory_.ctrl_y[2];
    if(abs(qx)<1e-4 and abs(qy)<1e-4 ){
        center_angle = std::atan2(target_trajectory_.ctrl_y[0],target_trajectory_.ctrl_x[0])+M_PI;
    } else{
        center_angle = std::atan2(qy,qx)+M_PI;
        if(qx*target_trajectory_.ctrl_x[0]+qy*target_trajectory_.ctrl_y[0]<0) // Closing Direction
            center_angle = std::atan2(qy,qx);
    }
    double half_range = 0.5* param_.fov;
    std::uniform_real_distribution<> theta_dis(center_angle - half_range, center_angle + half_range);
    double r, theta;
    Point tempPoint{end_point_center.x, end_point_center.y, end_point_center.z};
    bool flag = false;
    for (int i = start_idx; i < end_idx; i++) {
        flag = false;
        for(int j =0;j<100;j++){
            r = r_dis(gen);
            theta = theta_dis(gen);
            tempPoint.x = float(end_point_center.x + r * cos(theta));
            tempPoint.y = float(end_point_center.y + r * sin(theta));
            tempPoint.z = float(end_point_center.z);
            if(tempPoint.x*qx+2*tempPoint.y*qy>=0){
                flag = true;
                break;
            }
        }
        if(flag)
            endpoint_list_sub.push_back(tempPoint);
    }
}

void bpmp::Tracker::GeneratePrimitive(const double &t) {
    primitive_.clear();
    int num_chunk = param_.num_sample_planning / param_.num_thread;
    vector<thread> worker_thread;
    vector<vector<bpmp::PrimitivePlanning>> primitive_temp(param_.num_thread);
    for (int j = 0; j < param_.num_thread; j++)
        worker_thread.emplace_back(
                thread(&Tracker::GeneratePrimitiveThread, this, t, num_chunk * j, num_chunk * (j + 1),
                       std::ref(primitive_temp[j])));
    for (int j = 0; j < param_.num_thread; j++)
        worker_thread[j].join();
    for (int j = 0; j < param_.num_thread; j++) {
        for(int k = 0;k<primitive_temp[j].size();k++)
            primitive_.push_back(primitive_temp[j][k]);
    }
}

void bpmp::Tracker::GeneratePrimitiveThread(const double &t, const int &start_idx,
                                            const int &end_idx,
                                            std::vector<bpmp::PrimitivePlanning> &primitive_list_sub) {
    bpmp::PrimitivePlanning temp_primitive;
    temp_primitive.t0 = t;
    temp_primitive.tf = t + param_.horizon;
    double qx = target_trajectory_.ctrl_x[3]-target_trajectory_.ctrl_x[2];
    double qy = target_trajectory_.ctrl_y[3]-target_trajectory_.ctrl_y[2];
    bool is_target_static_mode = false;
    if(abs(qx)<1e-4 and abs(qy)<1e-4){
        is_target_static_mode = true;
        qx = target_trajectory_.ctrl_x[0];
        qy = target_trajectory_.ctrl_y[0];
    }

    double denom_inv_a = 1.0/(4*qx*qx+3*qy*qy);
    double denom_inv = 1.0/(3*qx*qx+4*qy*qy);
    double coeff1 = qx*qx+2*qy*qy;
    double coeff2 = qx*qy;
    double coeff3 = 3*qx*qx+2*qy*qy;
    for (int i = start_idx; i < end_idx; i++) {
        if (param_.sample_mode==1){
            // x-coeff
            temp_primitive.ctrl_x[0] = 0.0;
            temp_primitive.ctrl_x[1] = 0.33333333*end_points_[i].x;
            temp_primitive.ctrl_x[2] = 0.66666667*end_points_[i].x;
            temp_primitive.ctrl_x[3] = end_points_[i].x;
            // y-coeff
            temp_primitive.ctrl_y[0] = 0.0;
            temp_primitive.ctrl_y[1] = 0.0;
            temp_primitive.ctrl_y[2] = 0.5 * end_points_[i].y;
            temp_primitive.ctrl_y[3] = end_points_[i].y;
            // z-coeff
            temp_primitive.ctrl_z[0] = 0.0;
            temp_primitive.ctrl_z[1] = 0.0;
            temp_primitive.ctrl_z[2] = 0.0;
            temp_primitive.ctrl_z[3] = 0.0;
        }
        else{
            temp_primitive.ctrl_x[0] = 0.0;
            temp_primitive.ctrl_x[1] = (end_points_[i].x*coeff1-coeff2*end_points_[i].y)*denom_inv;
            temp_primitive.ctrl_x[2] = 2*temp_primitive.ctrl_x[1];
            temp_primitive.ctrl_x[3] = end_points_[i].x;

            temp_primitive.ctrl_y[0] = 0.0;
            temp_primitive.ctrl_y[1] = 0.0;
            temp_primitive.ctrl_y[2] = (end_points_[i].y*coeff3-end_points_[i].x*coeff2)*denom_inv;
            temp_primitive.ctrl_y[3] = end_points_[i].y;
//            temp_primitive.ctrl_x[0] = 0.0;
//            temp_primitive.ctrl_x[1] = (end_points_[i].x*(2*qx*qx+qy*qy)+qx*qy*end_points_[i].y)*denom_inv_a;
//            temp_primitive.ctrl_x[2] = (2*end_points_[i].x*(2*qx*qx+qy*qy)+2*end_points_[i].y*qx*qy)*denom_inv_a;
//            temp_primitive.ctrl_x[3] = end_points_[i].x;
//
//            temp_primitive.ctrl_y[0] = 0.0;
//            temp_primitive.ctrl_y[1] = 0.0;
//            temp_primitive.ctrl_y[2] = (end_points_[i].y*(2*qx*qx+3*qy*qy)+end_points_[i].x*qy*qx)*denom_inv_a;
//            temp_primitive.ctrl_y[3] = end_points_[i].y;
            temp_primitive.ctrl_z[0] = 0.0;
            temp_primitive.ctrl_z[1] = 0.0;
            temp_primitive.ctrl_z[2] = 0.0;
            temp_primitive.ctrl_z[3] = 0.0;
        }

        primitive_list_sub.push_back(temp_primitive);
    }
}
void bpmp::Tracker::GetFOVIndex() {
    visible_index_.clear();
    int num_chunk = primitive_.size()/param_.num_thread;
    vector<thread> worker_thread;
    vector<vector<bpmp::uint>> visible_index_temp(param_.num_thread);
    for(int j =0;j<param_.num_thread;j++)
        worker_thread.emplace_back(thread(&Tracker::GetFOVIndexThread,this, num_chunk*j,num_chunk*(j+1),std::ref(visible_index_temp[j])));
    for (int j = 0; j < param_.num_thread; j++)
        worker_thread[j].join();
    for (int j = 0; j < param_.num_thread; j++) {
        for (int k = 0; k < visible_index_temp[j].size(); k++)
            visible_index_.push_back(visible_index_temp[j][k]);
    }
//    printf("%d primitives satisfy FOV Constraints\n",visible_index_.size());
}
void bpmp::Tracker::GetFOVIndexThread(const int &start_idx, const int &end_idx, std::vector<uint> &visible_idx_sub) {
    bool flag_store_fov = true;
    double T_inv = 1.0/param_.horizon;
    double vel_x[3], vel_y[3], rel_x[4], rel_y[4];
    double vel_x_squared[5], vel_y_squared[5], rel_x_squared[7], rel_y_squared[7], vel_2xy[5], rel_xy[7];
    double vel_squared_sum[5], rel_squared_sum[7];
    double value_num;
    double value_den;
    double value;
    for (int idx = start_idx; idx < end_idx; idx++) {
        flag_store_fov = true;
        for (int i = 0;i<4;i++){
            rel_x[i] = target_trajectory_.ctrl_x[i]-primitive_[idx].ctrl_x[i];
            rel_y[i] = target_trajectory_.ctrl_y[i]-primitive_[idx].ctrl_y[i];
        }
        for (int i = 0; i < 3; i++) {
            vel_x[i] = 3.0 * T_inv *
                       (primitive_[idx].ctrl_x[i + 1] -
                        primitive_[idx].ctrl_x[i]);
            vel_y[i] = 3.0 * T_inv *
                       (primitive_[idx].ctrl_y[i + 1] -
                        primitive_[idx].ctrl_y[i]);
        }
        // rel_x_squared, rel_y_squared
        rel_x_squared[0] = rel_x[0]*rel_x[0];
        rel_x_squared[1] = rel_x[0]*rel_x[1];
        rel_x_squared[2] = 0.4*rel_x[0]*rel_x[2]+0.6*rel_x[1]*rel_x[1];
        rel_x_squared[3] = 0.1*rel_x[0]*rel_x[3]+0.9*rel_x[1]*rel_x[2];
        rel_x_squared[4] = 0.4*rel_x[1]*rel_x[3]+0.6*rel_x[2]*rel_x[2];
        rel_x_squared[5] = rel_x[2]*rel_x[3];
        rel_x_squared[6] = rel_x[3]*rel_x[3];
        rel_y_squared[0] = rel_y[0]*rel_y[0];
        rel_y_squared[1] = rel_y[0]*rel_y[1];
        rel_y_squared[2] = 0.4*rel_y[0]*rel_y[2]+0.6*rel_y[1]*rel_y[1];
        rel_y_squared[3] = 0.1*rel_y[0]*rel_y[3]+0.9*rel_y[1]*rel_y[2];
        rel_y_squared[4] = 0.4*rel_y[1]*rel_y[3]+0.6*rel_y[2]*rel_y[2];
        rel_y_squared[5] = rel_y[2]*rel_y[3];
        rel_y_squared[6] = rel_y[3]*rel_y[3];
        // vel_x_squared, vel_y_squared
        vel_x_squared[0] = vel_x[0]*vel_x[0];
        vel_x_squared[1] = vel_x[0]*vel_x[1];
        vel_x_squared[2] = 0.33333333*vel_x[0]*vel_x[2]+0.66666667*vel_x[1]*vel_x[1];
        vel_x_squared[3] = vel_x[1]*vel_x[2];
        vel_x_squared[4] = vel_x[2]*vel_x[2];
        vel_y_squared[0] = vel_y[0]*vel_y[0];
        vel_y_squared[1] = vel_y[0]*vel_y[1];
        vel_y_squared[2] = 0.33333333*vel_y[0]*vel_y[2]+0.66666667*vel_y[1]*vel_y[1];
        vel_y_squared[3] = vel_y[1]*vel_y[2];
        vel_y_squared[4] = vel_y[2]*vel_y[2];
        for(int i=0;i<5;i++)
            vel_squared_sum[i] = vel_x_squared[i]+vel_y_squared[i];
        for(int i=0;i<7;i++)
            rel_squared_sum[i] = rel_x_squared[i]+rel_y_squared[i];
        vel_2xy[0] = 2*vel_x[0]*vel_y[0];
        vel_2xy[1] = vel_x[0]*vel_y[1]+vel_y[0]*vel_x[1];
        vel_2xy[2] = 0.33333333*(vel_x[0]*vel_y[2]+vel_y[0]*vel_x[2])+1.33333334*vel_x[1]*vel_y[1];
        vel_2xy[3] = vel_x[1]*vel_y[2]+vel_y[1]*vel_x[2];
        vel_2xy[4] = 2*vel_x[2]*vel_y[2];
        rel_xy[0] = rel_x[0]*rel_y[0];
        rel_xy[1] = 0.5*(rel_x[0]*rel_y[1]+rel_y[0]*rel_x[1]);
        rel_xy[2] = 0.2*(rel_x[0]*rel_y[2]+rel_y[0]*rel_x[2])+0.6*rel_x[1]*rel_y[1];
        rel_xy[3] = 0.05*(rel_x[0]*rel_y[3]+rel_x[3]*rel_y[0])+0.45*(rel_x[1]*rel_y[2]+rel_y[1]*rel_x[2]);
        rel_xy[4] = 0.2*(rel_x[1]*rel_y[3]+rel_x[3]*rel_y[1])+0.6*rel_x[2]*rel_y[2];
        rel_xy[5] = 0.5*(rel_x[3]*rel_y[2]+rel_y[3]*rel_x[2]);
        rel_xy[6] = rel_x[3]*rel_y[3];
        for (int j = 0; j <= 10; j++) {
            value_den = 0.0, value_num = 0.0;
            for (int k = std::max(0, j - 6); k <= std::min(4, j); k++) {
                value_num += double(nchooser(4, k)) * double(nchooser(6, j-k)) / double(nchooser(10, j))*
                             (vel_x_squared[k]*rel_x_squared[j-k]+vel_y_squared[k]*rel_y_squared[j-k]+vel_2xy[k]*rel_xy[j-k]);
                value_den += double(nchooser(4, k)) * double(nchooser(6, j-k)) / double(nchooser(10, j))*
                             (vel_squared_sum[k]*rel_squared_sum[j-k]);
            }
            if(value_num/value_den<pow(cos(0.5*param_.fov),2)){
                flag_store_fov = false;
                break;
            }
        }
//        if(not flag_store_fov)
//            continue;
//        for (int j=0;j<=5;j++){
//            value = 0.0;
//            for(int k = std::max(0,j-3);k<=std::min(2,j);k++){
//                value += double(nchooser(2,k))*double(nchooser(3,j-k))/double(nchooser(5,j))*
//                        (vel_x[k]*rel_x[j-k]+vel_y[k]*rel_y[j-k]);
//            }
//            if(value<0.0){
//                flag_store_fov = false;
//                break;
//            }
//        }
        if(flag_store_fov)
            visible_idx_sub.push_back(idx);
    }

}
void bpmp::Tracker::GetDynamicallyFeasibleIndexThread(const int &start_idx, const int &end_idx,
                                                      std::vector<uint> &dyn_feas_idx_sub) {
    bool flag_store_velocity = true; // linear velocity
    bool flag_store_acceleration = true; // acceleration
    bool flag_store_rotation = true; // angular velocity
    double value;
    double value_num;
    double value_den;
    double vel_x[3], vel_y[3]; // velocity
    double acc_x[2], acc_y[2]; // acceleration
    double acc_x_elev[3], acc_y_elev[3], ang_num_total[5], ang_num_total_elevation[5], ang_den[5]; // angular
    double T_inv = 1.0 / param_.horizon;
    double T2_inv = T_inv * T_inv;
    double vel_max_squared = param_.vel_max * param_.vel_max;
    double acc_max_squared = param_.acc_max * param_.acc_max;

    for (int idx = start_idx; idx < end_idx; idx++) {
        flag_store_velocity = true;
        for (int i = 0; i < 3; i++) {
            vel_x[i] = 3.0 * T_inv *
                       (primitive_[safe_index_[idx]].ctrl_x[i + 1] -
                        primitive_[safe_index_[idx]].ctrl_x[i]);
            vel_y[i] = 3.0 * T_inv *
                       (primitive_[safe_index_[idx]].ctrl_y[i + 1] -
                        primitive_[safe_index_[idx]].ctrl_y[i]);
        }
        for (int j = 0; j <= 4; j++) {
            value = 0.0;
            for (int k = std::max(0, j - 2); k <= std::min(2, j); k++) {
                value +=
                        double(bpmp::nchooser(2, k)) * double(bpmp::nchooser(2, j - k)) / double(bpmp::nchooser(4, j)) *
                        (vel_x[k] * vel_x[j - k] + vel_y[k] * vel_y[j - k]);
            }
            if (value > vel_max_squared) {
                flag_store_velocity = false;
                break;
            }
        }
        if (not flag_store_velocity)
            continue;
        flag_store_acceleration = true;
        for (int i = 0; i < 2; i++) {
            acc_x[i] = 6.0 * T2_inv *
                       (primitive_[safe_index_[idx]].ctrl_x[i + 2] -
                        2 * primitive_[safe_index_[idx]].ctrl_x[i + 1] +
                        primitive_[safe_index_[idx]].ctrl_x[i]);
            acc_y[i] = 6.0 * T2_inv *
                       (primitive_[safe_index_[idx]].ctrl_y[i + 2] -
                        2 * primitive_[safe_index_[idx]].ctrl_y[i + 1] +
                        primitive_[safe_index_[idx]].ctrl_y[i]);
        }
        for (int j = 0; j <= 2; j++) {
            value = 0.0;
            for (int k = std::max(0, j - 1); k <= std::min(1, j); k++) {
                value +=
                        double(bpmp::nchooser(1, k)) * double(bpmp::nchooser(1, j - k)) / double(bpmp::nchooser(2, j)) *
                        (acc_x[k] * acc_x[j - k] + acc_y[k] * acc_y[j - k]);
            }
            if (value > acc_max_squared) {
                flag_store_acceleration = false;
                break;
            }
        }
//        if (flag_store_acceleration)
//            dyn_feas_idx_sub.push_back(safe_index_[idx]);
        if (not flag_store_acceleration)
            continue;
        flag_store_rotation = true;
        acc_x_elev [0] = acc_x[0], acc_x_elev[1] = 0.5*acc_x[0]+0.5*acc_x[1], acc_x_elev[2] =acc_x[2];
        acc_y_elev [0] = acc_y[0], acc_y_elev[1] = 0.5*acc_y[0]+0.5*acc_y[1], acc_y_elev[2] =acc_y[2];
        for (int j = 0; j <= 4; j++) {
            value_num = 0.0;
            value_den = 0.0;
            for (int k = std::max(0, j - 2); k <= std::min(2, j); k++) {
                value_num +=
                        double(bpmp::nchooser(2, k)) * double(bpmp::nchooser(2, j - k)) / double(bpmp::nchooser(4, j)) *
                        (acc_y_elev[k] * vel_x[j - k] - acc_x_elev[k] * vel_y[j - k]);
                value_den +=
                        double(bpmp::nchooser(2, k)) * double(bpmp::nchooser(2, j - k)) / double(bpmp::nchooser(4, j)) *
                        (vel_x[k] * vel_x[j - k] + vel_y[k] * vel_y[j - k]);
            }
            if(abs(value_den)<1e-4){
                flag_store_rotation = false;
                break;
            }
            if (abs(value_num/value_den) >param_.ang_max) {
                flag_store_rotation = false;
                break;
            }
        }
        if (flag_store_rotation)
            dyn_feas_idx_sub.push_back(safe_index_[idx]);
    }
}

std::vector<uint> bpmp::Tracker::GetSafeIndexDynamic(const std::vector<uint> &index) {
    vector<bpmp::uint> feasible_index;
    int num_chunk = index.size()/param_.num_thread;
    vector<thread> worker_thread;
    vector<vector<bpmp::uint>> safe_index_temp(param_.num_thread);
    for (int j = 0; j < param_.num_thread; j++)
        worker_thread.emplace_back(
                thread(&Tracker::GetSafeIndexDynamicThread, this, index, num_chunk * j, num_chunk * (j + 1),
                       std::ref(safe_index_temp[j])));
    for (int j = 0; j < param_.num_thread; j++)
        worker_thread[j].join();
    for (int j = 0; j < param_.num_thread; j++) {
        for (int k = 0; k < safe_index_temp[j].size(); k++)
            feasible_index.push_back(safe_index_temp[j][k]);
    }
    return feasible_index;
}
std::vector<bpmp::uint> bpmp::Tracker::GetSafeIndexUnstructured(const vector<bpmp::uint> &index) {
    vector<bpmp::uint> feasible_index;
    // Generate Corridor
    GenerateCorridor();
    if(polys_.empty())
        return feasible_index;
    // SafeIndexUnstructured
    int num_chunk = index.size() / param_.num_thread;
    vector<thread> worker_thread;
    vector<vector<bpmp::uint>> safe_index_temp(param_.num_thread);

    for (int j = 0; j < param_.num_thread; j++)
        worker_thread.emplace_back(
                thread(&Tracker::GetSafeIndexUnstructuredThread, this, index, num_chunk * j, num_chunk * (j + 1),
                       std::ref(safe_index_temp[j])));

    for (int j = 0; j < param_.num_thread; j++)
        worker_thread[j].join();
    for (int j = 0; j < param_.num_thread; j++) {
        for (int k = 0; k < safe_index_temp[j].size(); k++)
            feasible_index.push_back(safe_index_temp[j][k]);
    }
    return feasible_index;
}

void bpmp::Tracker::GetSafeIndexUnstructuredThread(const std::vector<uint> &prior_idx, const int &start_idx, const int &end_idx,
                                                   std::vector<uint> &safe_idx_sub) {
    Eigen::Vector3d A_comp_temp{0.0, 0.0, 0.0};
    double b_comp_temp(0.0);
    vector<Eigen::Vector3d> LinearConstraintA;
    vector<double> LinearConstraintb;
    int num_constraint = corridor_constraints_.A().rows();
    for (int i = 0; i < num_constraint; i++) {
        A_comp_temp[0] = corridor_constraints_.A().coeffRef(i, 0);
        A_comp_temp[1] = corridor_constraints_.A().coeffRef(i, 1);
        A_comp_temp[2] = corridor_constraints_.A().coeffRef(i, 2);
        b_comp_temp = corridor_constraints_.b().coeffRef(i, 0);
        LinearConstraintA.push_back(A_comp_temp);
        LinearConstraintb.push_back(b_comp_temp);
    }
    bool flag_store_in1 = true; // collision between obstacle and tracker
    bool flag_store_out1 = true;
    bool flag_store_in2 = true; // distance between tracker and target
    double safe_distance_squared = pow(param_.safe_distance + 2 * param_.object_radius, 2);
    double distance_max_squared = pow(param_.distance_max, 2);
    double relative_target_pos_x[4], relative_target_pos_y[4]; // tracker-target
    double value_sfc;
    double value_distance;
    for (int idx = start_idx; idx < end_idx; idx++) {
        flag_store_out1 = true;
        flag_store_in1 = true;
        // SFC Constraint
        for (int i = 0; i < LinearConstraintA.size(); i++) {
            for (int j = 0; j < 4; j++) {
                value_sfc = LinearConstraintA[i][0] * (primitive_[prior_idx[idx]].ctrl_x[j]) +
                            LinearConstraintA[i][1] * (primitive_[prior_idx[idx]].ctrl_y[j]) +
                            LinearConstraintA[i][2] * (primitive_[prior_idx[idx]].ctrl_z[j])+
                            - LinearConstraintb[i] + param_.object_radius + param_.safe_distance;
                if (value_sfc > 0.0) {
                    flag_store_in1 = false;
                    break;
                }
            }
            if (not flag_store_in1) {
                flag_store_out1 = false;
                break;
            }
        }
        // Distance Constraint
        if (flag_store_out1) {
            flag_store_in2 = true;
            for (int i = 0; i < 4; i++) {
                relative_target_pos_x[i] = primitive_[prior_idx[idx]].ctrl_x[i] - target_trajectory_.ctrl_x[i];
                relative_target_pos_y[i] = primitive_[prior_idx[idx]].ctrl_y[i] - target_trajectory_.ctrl_y[i];
            }
            for (int j = 0; j <= 6; j++) {
                value_distance = 0.0;
                for (int k = std::max(0, j - 3); k <= std::min(3, j); k++) {
                    value_distance += (double) bpmp::nchooser(3, k) * (double) bpmp::nchooser(3, j - k) /
                                      (double) bpmp::nchooser(6, j) *
                                      (relative_target_pos_x[k] * relative_target_pos_x[j - k] +
                                       relative_target_pos_y[k] * relative_target_pos_y[j - k]);
                }
                if (value_distance > distance_max_squared or value_distance < safe_distance_squared) {
                    flag_store_in2 = false;
                    break;
                }
            }
            if (flag_store_in2) {
                safe_idx_sub.push_back(prior_idx[idx]);
            }
        }
    }
}
void bpmp::Tracker::GetSafeIndexThread(const int &start_idx, const int &end_idx,
                                       std::vector<uint> &safe_idx_sub) {

}



void bpmp::Tracker::GetDynamicallyFeasibleIndex() {
    dynamically_feasible_index_.clear();
    int num_chunk = safe_index_.size() / param_.num_thread;
    vector<thread> worker_thread;
    vector<vector<bpmp::uint>> dynamically_feasible_index_temp(param_.num_thread);
    for (int j = 0; j < param_.num_thread; j++)
        worker_thread.emplace_back(
                thread(&Tracker::GetDynamicallyFeasibleIndexThread, this,num_chunk * j, num_chunk * (j + 1),
                       std::ref(dynamically_feasible_index_temp[j])));
    for (int j = 0; j < param_.num_thread; j++)
        worker_thread[j].join();
    for (int j = 0; j < param_.num_thread; j++) {
        for (int k = 0; k < dynamically_feasible_index_temp[j].size(); k++)
            dynamically_feasible_index_.push_back(dynamically_feasible_index_temp[j][k]);
    }
}






void bpmp::Tracker::UpdateResultToBase(const bool &is_success) {
    {   // Success flag
        p_base_->mutex_set_[1].lock();
        p_base_->success_flag_ = is_success;
        p_base_->mutex_set_[1].unlock();
    }
    if (is_success) {
        {   // Primitive
            p_base_->mutex_set_[1].lock();
            p_base_->SetTrackerPrimitives(primitive_);
            p_base_->mutex_set_[1].unlock();
        }
        {   // Feasible Index
            p_base_->mutex_set_[1].lock();
            p_base_->SetFeasibleIndex(dynamically_feasible_index_);
            p_base_->mutex_set_[1].unlock();
        }
        {   // Best Index
            p_base_->mutex_set_[1].lock();
            p_base_->SetBestIndex(best_index_);
            p_base_->mutex_set_[1].unlock();
        }
        {
            p_base_->mutex_set_[1].lock();
            p_base_->SetCorridorVis(polys_);
            p_base_->mutex_set_[1].unlock();
        }
    } else {
        {   // Primitive
            p_base_->mutex_set_[1].lock();
            p_base_->EraseTrackerPrimitives();
            p_base_->mutex_set_[1].unlock();
        }
        {   // Safe Index
            p_base_->mutex_set_[1].lock();
            p_base_->EraseFeasibleIndex();
            p_base_->mutex_set_[1].unlock();
        }
    }
}

void bpmp::Tracker::GetBestIndex() {
    if(dynamically_feasible_index_.size()<param_.num_thread){
        best_index_ = dynamically_feasible_index_[0];
        return;
    }
    int num_chunk = dynamically_feasible_index_.size() / param_.num_thread;
    vector<thread> worker_thread;
    vector<std::pair<bpmp::uint, double>> best_index_temp(param_.num_thread);
    for (int j = 0; j < param_.num_thread; j++)
        worker_thread.emplace_back(
                thread(&Tracker::GetBestIndexThread, this, num_chunk * j, num_chunk * (j + 1),
                       std::ref(best_index_temp[j])));
    for (int j = 0; j < param_.num_thread; j++)
        worker_thread[j].join();
    double min_acceleration = 9999999.0;
    for (int j = 0; j < param_.num_thread; j++) {
        if (min_acceleration > best_index_temp[j].second) {
            min_acceleration = best_index_temp[j].second;
            best_index_ = best_index_temp[j].first;
        };
    }
}

void bpmp::Tracker::GetBestIndexThread(const int &start_idx, const int &end_idx,
                                       std::pair<uint, double> &score_pair) {
    double acc_coeff_x[2];
    double acc_coeff_y[2];
    double min_acc = 99999999.0;

    double T = param_.horizon;
    double acc_coeff = 6.0 / pow(T, 2);
    double acc_squared_sum;
    uint min_acc_idx = -1;
    double heading_angle[2];
    double los_angle[2];
    double heading_mag, los_mag;
    for(int idx = start_idx;idx<end_idx;idx++){
        heading_angle[0] = primitive_[dynamically_feasible_index_[idx]].ctrl_x[3]-primitive_[dynamically_feasible_index_[idx]].ctrl_x[2];
        heading_angle[1] = primitive_[dynamically_feasible_index_[idx]].ctrl_y[3]-primitive_[dynamically_feasible_index_[idx]].ctrl_y[2];
        los_angle[0] = target_trajectory_.ctrl_x[3]-primitive_[dynamically_feasible_index_[idx]].ctrl_x[3];
        los_angle[1] = target_trajectory_.ctrl_y[3]-primitive_[dynamically_feasible_index_[idx]].ctrl_y[3];
        heading_mag = sqrt(pow(heading_angle[0],2)+pow(heading_angle[1],2));
        los_mag = sqrt(pow(los_angle[0],2)+pow(los_angle[1],2));
        if(heading_mag<1e-4)
            continue;
        heading_angle[0] = heading_angle[0] /heading_mag;
        heading_angle[1] = heading_angle[1] /heading_mag;
        los_angle[0] = los_angle[0]/los_mag;
        los_angle[1] = los_angle[1]/los_mag;
        acc_squared_sum = -(heading_angle[0]*los_angle[0]+heading_angle[1]*los_angle[1]);
        if(acc_squared_sum<min_acc){
            min_acc_idx = dynamically_feasible_index_[idx];
            min_acc = acc_squared_sum;
        }
    }
//    for (int idx = start_idx; idx < end_idx; idx++) {
//        acc_coeff_x[0] = acc_coeff * (primitive_[dynamically_feasible_index_[idx]].ctrl_x[2] -
//                                      2.0 *
//                                      primitive_[dynamically_feasible_index_[idx]].ctrl_x[1]
//                                      +
//                                      primitive_[dynamically_feasible_index_[idx]].ctrl_x[0]);
//        acc_coeff_x[1] = acc_coeff * (primitive_[dynamically_feasible_index_[idx]].ctrl_x[3] -
//                                      2.0 *
//                                      primitive_[dynamically_feasible_index_[idx]].ctrl_x[2]
//                                      +
//                                      primitive_[dynamically_feasible_index_[idx]].ctrl_x[1]);
//        acc_coeff_y[0] = acc_coeff * (primitive_[dynamically_feasible_index_[idx]].ctrl_y[2] -
//                                      2.0 *
//                                      primitive_[dynamically_feasible_index_[idx]].ctrl_y[1]
//                                      +
//                                      primitive_[dynamically_feasible_index_[idx]].ctrl_y[0]);
//        acc_coeff_y[1] = acc_coeff * (primitive_[dynamically_feasible_index_[idx]].ctrl_y[3] -
//                                      2.0 *
//                                      primitive_[dynamically_feasible_index_[idx]].ctrl_y[2]
//                                      +
//                                      primitive_[dynamically_feasible_index_[idx]].ctrl_y[1]);
//        acc_squared_sum = 0.0;
//        for (int j = 0; j <= 2; j++) {
//            for (int k = std::max(0, j - 1); k <= std::min(1, j); k++) {
//                acc_squared_sum +=
//                        double(bpmp::nchooser(1, k)) * double(bpmp::nchooser(1, j - k)) / double(bpmp::nchooser(2, j)) *
//                        (acc_coeff_x[k] * acc_coeff_x[j - k] + acc_coeff_y[k] * acc_coeff_y[j - k]);
//            }
//        }
//        if (acc_squared_sum < min_acc) {
//            min_acc_idx = dynamically_feasible_index_[idx];
//            min_acc = acc_squared_sum;
//        }
//    }
    score_pair.second = min_acc;
    score_pair.first = min_acc_idx;
}



void bpmp::Tracker::GenerateCorridor() {
    polys_.clear();
    Eigen::Matrix<double, 3, 1> predicted_point;
    predicted_point[0] = target_trajectory_.ctrl_x[0];
    predicted_point[1] = target_trajectory_.ctrl_y[0];
    predicted_point[2] = 0.0;
//    predicted_point[2] = target_trajectory_.ctrl_z[0];

    Eigen::Matrix<double, 3, 1> start_point;
    start_point[0] = 0.0;
    start_point[1] = 0.0;
    start_point[2] = 0.0;
//        start_point[0] = 1.0;
//        start_point[1] = 2.0;
    vec_Vec3f segment;
    segment.push_back(start_point), segment.push_back(predicted_point);
    EllipsoidDecomp3D decomp_util;
    decomp_util.set_obs(point_cloud_3d_);
    decomp_util.set_local_bbox(Vec3f(5.0, 5.0,1.0));
    decomp_util.dilate(segment);
    vec_E<Polyhedron3D> polys;
    auto poly_hedrons = decomp_util.get_polyhedrons();
    if(not poly_hedrons.empty()){
        polys_.push_back(poly_hedrons[0]);
        LinearConstraint3D corridor_constraint(0.5 * (start_point + predicted_point), poly_hedrons[0].hyperplanes());
        corridor_constraints_=corridor_constraint;
    }
}

int bpmp::Tracker::EnvironmentMode() {
    int mode = 0; // 0: unstructured static, 1: dynamic, 2: static+dynamic
    p_base_->mutex_set_[0].lock();
    if(p_base_->is_dynobs_received_)
        mode = 1;
    if(p_base_->is_pcl_received_ and p_base_->is_dynobs_received_)
        mode = 2;
    p_base_->mutex_set_[0].unlock();
    //cout<<"MODE: "<<mode<<endl;
    return mode;
}

void
bpmp::Tracker::GetSafeIndexDynamicThread(const std::vector<uint> &prior_idx, const int &start_idx, const int &end_idx,
                                         std::vector<uint> &safe_idx_sub) {
    bool flag_store_in1 = true; // collision between obstacle and tracker
    bool flag_store_in2 = true; // occlusion of targets
    bool flag_store_in3 = true; // distance between tracker and target
    bool flag_store_out = true;
    double value;
    double relative_obstacle_pos_x[4], relative_obstacle_pos_y[4]; //drone-obstacle
    double relative_target_obstacle_pos_x[4], relative_target_obstacle_pos_y[4]; //target-obstacle
    double relative_target_pos_x[4], relative_target_pos_y[4]; //drone-target
    double object_radius_squared = param_.object_radius * param_.object_radius;
    double safe_distance_squared = pow(param_.safe_distance + 2 * param_.object_radius, 2);
    double distance_max_squared = pow(param_.distance_max, 2);
    for (int idx = start_idx; idx < end_idx; idx++) {
        flag_store_in1 = true;
        flag_store_in2 = true;
        flag_store_out = true;
        flag_store_in3 = true;
        for (int j = 0; j < 4; j++) {
            relative_target_pos_x[j] = primitive_[prior_idx[idx]].ctrl_x[j] - target_trajectory_.ctrl_x[j];
            relative_target_pos_y[j] = primitive_[prior_idx[idx]].ctrl_y[j] - target_trajectory_.ctrl_y[j];
        }
        double min_value_drone_obstacle =1e8;
        double min_value_target_obstacle =1e8;

        for (int i = 0; i < obstacle_primitive_list_.size(); i++) {
            flag_store_out = true;
            for (int j = 0; j < 4; j++) {
                relative_obstacle_pos_x[j] =
                        primitive_[prior_idx[idx]].ctrl_x[j] - obstacle_primitive_list_[i].ctrl_x[j];
                relative_obstacle_pos_y[j] =
                        primitive_[prior_idx[idx]].ctrl_y[j] - obstacle_primitive_list_[i].ctrl_y[j];
                relative_target_obstacle_pos_x[j] =
                        target_trajectory_.ctrl_x[j] - obstacle_primitive_list_[i].ctrl_x[j];
                relative_target_obstacle_pos_y[j] =
                        target_trajectory_.ctrl_y[j] - obstacle_primitive_list_[i].ctrl_y[j];
            }
            min_value_drone_obstacle =1e8;
            for (int j = 0; j <= 6; j++) {  // Collision between obstacle and tracker
                flag_store_in1 = true;
                value = 0.0f;
                for (int k = std::max(0, j - 3); k <= std::min(3, j); k++) {
                    value += (double) nchooser(3, k) * (double) nchooser(3, j - k) /
                             (double) nchooser(6, j) *
                             (relative_obstacle_pos_x[k] * relative_obstacle_pos_x[j - k] +
                              relative_obstacle_pos_y[k] * relative_obstacle_pos_y[j - k]
                             );
                }
                if (value < safe_distance_squared) {
                    flag_store_in1 = false;
                    break;
                }
                if (value<min_value_drone_obstacle)
                    min_value_drone_obstacle = value;
            }
            if (not flag_store_in1) {
                flag_store_out = false;
                break;
            }
            min_value_drone_obstacle = std::max(0.0, min_value_drone_obstacle);
            //
            min_value_target_obstacle =1e8;
            for (int j = 0;j<=6;j++){
                value = 0.0;
                for (int k=std::max(0,j-3);k<=std::min(3,j);k++){
                    value += (double) nchooser(3,k) * (double) nchooser(3,j-k) / (double) nchooser(6,j)*
                             (relative_target_obstacle_pos_x[k]*relative_target_obstacle_pos_x[j-k]+
                              relative_target_obstacle_pos_y[k]*relative_target_obstacle_pos_y[j-k]);
                }
                if (value<min_value_target_obstacle)
                    min_value_target_obstacle = value;
            }
            min_value_target_obstacle = std::max(0.0,min_value_target_obstacle);
            for (int j = 0; j <= 6; j++) {
                flag_store_in2 = true;
                value = 0.0f;
                for (int k = std::max(0, j - 3); k <= std::min(3, j); k++) {
                    value += (double) nchooser(3, k) * (double) nchooser(3, j - k) /
                             (double) nchooser(6, j) *
                             (relative_obstacle_pos_x[k] * relative_target_obstacle_pos_x[j - k] +
                              relative_obstacle_pos_y[k] * relative_target_obstacle_pos_y[j - k]
                             );
                }
                if (param_.check_mode==0)
                    if(value+2*object_radius_squared+std::min(min_value_drone_obstacle,min_value_target_obstacle)<0){
                        flag_store_in2 = false;
                        break;
                    }
                if (param_.check_mode==1)
                    if(value+2*object_radius_squared<0){
                        flag_store_in2 = false;
                        break;
                    }
                if (param_.check_mode==2)
                    if(value<object_radius_squared){
                        flag_store_in2 = false;
                        break;
                    }
            }
            if (not flag_store_in2) {
                flag_store_out = false;
                break;
            }
        }
        if (flag_store_in1 and flag_store_in2 and flag_store_out) {
            flag_store_in3 = true;
            for (int j = 0; j <= 6; j++) {
                value = 0.0f;
                for (int k = std::max(0, j - 3); k <= std::min(3, j); k++) {
                    value += (double) nchooser(3, k) * (double) nchooser(3, j - k) /
                             (double) nchooser(6, j) *
                             (relative_target_pos_x[k] * relative_target_pos_x[j - k] +
                              relative_target_pos_y[k] * relative_target_pos_y[j - k]
                             );
                }
                if (value < safe_distance_squared or value > distance_max_squared) {
                    flag_store_in3 = false;
                    break;
                }
            }
        }
        if (flag_store_out and flag_store_in3)
            safe_idx_sub.push_back(prior_idx[idx]);
    }
}

bool bpmp::Tracker::IsTargetStatic() {
    bool static_x = abs(target_trajectory_.ctrl_x[0]-target_trajectory_.ctrl_x[1])<1e-2 and
                    abs(target_trajectory_.ctrl_x[1]-target_trajectory_.ctrl_x[2])<1e-2 and
                    abs(target_trajectory_.ctrl_x[2]-target_trajectory_.ctrl_x[3])<1e-2;
    bool static_y = abs(target_trajectory_.ctrl_y[0]-target_trajectory_.ctrl_y[1])<1e-2 and
                    abs(target_trajectory_.ctrl_y[1]-target_trajectory_.ctrl_y[2])<1e-2 and
                    abs(target_trajectory_.ctrl_y[2]-target_trajectory_.ctrl_y[3])<1e-2;
    if (static_x and static_y)
        return true;
    else
        return false;
}







