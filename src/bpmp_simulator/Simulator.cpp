#include <bpmp_simulator/Simulator.h>
#include <cmath>
void bpmp::Simulator::Run() {
    if (not analysis_mode_){
        double simulation_frequency = 1.0/simulation_dt_;
        ros::Rate loop_rate(simulation_frequency);
        double t0 = ros::Time::now().toSec();
        double t_sim;
        while(ros::ok()){
            t_sim = ros::Time::now().toSec()-t0;
            UpdateDynamics(t_sim);
            PrepareRosMsgs(t_sim);
            PublishRosMsgs();
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    else{
        printf("ANALYSIS MODE IS NOT IMPLEMENTED\n");
    }
}
bpmp::Simulator::Simulator() : nh_("~") {
    nh_.param<bool>("analysis_mode",analysis_mode_,false);
    nh_.param<double>("spatial_scale",spatial_scale_,1.0);
    nh_.param<double>("time_scale",time_scale_,1.0);
    nh_.param<string>("map_frame_id", map_frame_id_, "map");
    nh_.param<double>("simulation_dt", simulation_dt_, 0.01);

    nh_.param<bool>("is_unstructured", is_unstructured_, false);
    nh_.param<double>("inflation_size", inflation_size_, 0.5);
    nh_.param<double>("point_resolution", point_resolution_, 0.1);
    nh_.param<double>("agent_size", agent_size_, 0.1);

    // Total number of agents in files
    nh_.param<int>("total_object_number_in_file", total_object_number_in_file_, 0);

    // Obstacle Indices
    if (nh_.hasParam("obstacle_idx_list")) {
        nh_.getParam("obstacle_idx_list", object_idx_list_);
    } else
        ROS_ERROR("Failed to find 'obstacle list'.");

    // Target Index
    nh_.param<int>("target_idx", target_idx_, 0);
    object_idx_list_.push_back(target_idx_);
    object_number_ = (int) object_idx_list_.size(); // obstacles + target
    nh_.param<string>("initial_state_file_name", initial_state_file_name_, "");
    nh_.param<string>("object_history_file_name", object_history_file_name_, "");
    // unstructured file name
    nh_.param<string>("obstacle_configuration_file_name", obstacle_configuration_file_name_, "");
    nh_.param<int>("moving_obstacle_number", moving_obstacle_number_, 0);
    nh_.param<int>("total_test_number",total_test_number_,0);

    nh_.param<double>("target_rate", target_rate_, 0.5);

    obstacle_vis_.header.frame_id = map_frame_id_;
    target_vis_.header.frame_id = map_frame_id_;
    object_vis_.header.frame_id = map_frame_id_;
    // tracker_vis_.header.frame_id = map_frame_id_;


    obstacle_vis_.type = visualization_msgs::Marker::CYLINDER;
    obstacle_vis_.ns = "Obstacle";
    obstacle_vis_.color.a = 1.0;
    obstacle_vis_.color.r = 0.0;
    obstacle_vis_.color.g = 1.0;
    obstacle_vis_.color.b = 0.0;
    obstacle_vis_.scale.x = 2 * agent_size_;
    obstacle_vis_.scale.y = 2 * agent_size_;
    obstacle_vis_.scale.z = 2.0;
    obstacle_vis_.pose.orientation.w = 1.0;
    obstacle_vis_.pose.orientation.x = 0.0;
    obstacle_vis_.pose.orientation.y = 0.0;
    obstacle_vis_.pose.orientation.z = 0.0;

    target_vis_.type = visualization_msgs::Marker::SPHERE;
    target_vis_.ns = "Target";
    target_vis_.id = 0;
    target_vis_.color.a = 1.0;
    target_vis_.color.r = 1.0;
    target_vis_.color.g = 0.0;
    target_vis_.color.b = 0.0;
    target_vis_.scale.x = 2 * agent_size_;
    target_vis_.scale.y = 2 * agent_size_;
    target_vis_.scale.z = 2.0;
    target_vis_.pose.orientation.w = 1.0;
    target_vis_.pose.orientation.x = 0.0;
    target_vis_.pose.orientation.y = 0.0;
    target_vis_.pose.orientation.z = 0.0;

    object_vis_.type = visualization_msgs::Marker::CUBE;
    object_vis_.ns = "Object";
    object_vis_.id = 0;
    object_vis_.color.a = 0.5;
    object_vis_.color.r = 0.0;
    object_vis_.color.g = 0.0;
    object_vis_.color.b = 1.0;


    point_cloud_.header.frame_id = map_frame_id_;
    target_vis_publisher_ = nh_.advertise<visualization_msgs::Marker>("target_vis", 1);
    obstacle_list_vis_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("obstacle_list_vis", 1);
    obstacle_state_list_publisher_ = nh_.advertise<bpmp_tracker::ObjectStateList>("obstacle_state_list", 1);
    target_state_publisher_ = nh_.advertise<bpmp_tracker::ObjectState>("target_state", 1);
    
    pcl_publisher_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("point_cloud_obstacle", 1);
    pcl_boxes_vis_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("pcl_boxes", 1);

    object_vis_publisher_ = nh_.advertise<visualization_msgs::Marker>("object_vis", 1);
    object_state_publisher_ = nh_.advertise<bpmp_tracker::ObjectState>("object_state", 1);


    if(analysis_mode_){
        printf("ANALYSIS IS NOT IMPLEMENTED\n");

    }
    else{
        ReadInitialTrackerStateList();
        ReadObjectTrajectory();
        if (is_unstructured_)
            ReadObstacleConfiguration();
    }
}

void bpmp::Simulator::ReadObjectTrajectory() {
    ifstream object_trajectory_file;
    object_trajectory_file.open(object_history_file_name_.c_str());
    object_history_list_.clear();
    int num_read_unit = 12;
    string line, word;
    vector<string> row;
    vector<vector<string>> content;
    if (object_trajectory_file.is_open()) {
        while (getline(object_trajectory_file, line)) {
            row.clear();
            stringstream str(line);
            while (getline(str, word, ','))
                row.push_back(word);
            content.push_back(row);
        }
    } else
        printf("UNABLE TO READ OBJECT TRAJECTORY HISTORY\n");
    StateHistory temp_obstacle_history;
    for (int i = 0; i < object_number_; i++)
        object_history_list_.push_back(temp_obstacle_history);
    for (int i = 0; i < object_number_; i++) {
        for (int j = 0; j < content.size(); j++) {
            object_history_list_[i].t.push_back(stod(content[j][object_idx_list_[i] * num_read_unit + 1])/time_scale_);   // t
            object_history_list_[i].px.push_back(stod(content[j][object_idx_list_[i] * num_read_unit + 2])*spatial_scale_);   // px
            object_history_list_[i].py.push_back(stod(content[j][object_idx_list_[i] * num_read_unit + 3])*spatial_scale_);   // py
            object_history_list_[i].pz.push_back(stod(content[j][object_idx_list_[i] * num_read_unit + 4])*spatial_scale_);   // pz
            object_history_list_[i].vx.push_back(stod(content[j][object_idx_list_[i] * num_read_unit + 5])*spatial_scale_*time_scale_);   // vx
            object_history_list_[i].vy.push_back(stod(content[j][object_idx_list_[i] * num_read_unit + 6])*spatial_scale_*time_scale_);   // vy
            object_history_list_[i].vz.push_back(stod(content[j][object_idx_list_[i] * num_read_unit + 7])*spatial_scale_*time_scale_);   // vz
        }
    }
}
Eigen::Matrix2d rot2(double theta){
    Eigen::Matrix2d R;
    R << cos(theta), -sin(theta),
         sin(theta), cos(theta);
    return R;
}
void bpmp::Simulator::UpdateDynamics(const double &t) {
    current_obstacle_state_list_.clear();
    State temp_state;
    /////////////
    // static bool only_once = true;
    // if (only_once) {
    //////////////
    {   // Target State
        current_target_state_.px = bpmp::interpolate(object_history_list_[target_idx_].t,
                                                     object_history_list_[target_idx_].px, t * target_rate_);
        current_target_state_.py = bpmp::interpolate(object_history_list_[target_idx_].t,
                                                     object_history_list_[target_idx_].py, t * target_rate_);
        current_target_state_.pz = bpmp::interpolate(object_history_list_[target_idx_].t,
                                                     object_history_list_[target_idx_].pz, t * target_rate_);
        current_target_state_.vx = bpmp::interpolate(object_history_list_[target_idx_].t,
                                                     object_history_list_[target_idx_].vx, t * target_rate_) * target_rate_;
        current_target_state_.vy = bpmp::interpolate(object_history_list_[target_idx_].t,
                                                     object_history_list_[target_idx_].vy, t * target_rate_) * target_rate_;
        current_target_state_.vz = bpmp::interpolate(object_history_list_[target_idx_].t,
                                                     object_history_list_[target_idx_].vz, t * target_rate_) * target_rate_;
    }
    /////////////////
    // only_once = false;
    // }
    /////////////////
    {   // Obstacle State
        for (int i = 0; i < obstacle_idx_list_.size(); i++) {
            temp_state.px = bpmp::interpolate(object_history_list_[obstacle_idx_list_[i]].t,
                                              object_history_list_[obstacle_idx_list_[i]].px, t);
            temp_state.py = bpmp::interpolate(object_history_list_[obstacle_idx_list_[i]].t,
                                              object_history_list_[obstacle_idx_list_[i]].py, t);
            temp_state.pz = bpmp::interpolate(object_history_list_[obstacle_idx_list_[i]].t,
                                              object_history_list_[obstacle_idx_list_[i]].pz, t);
            temp_state.vx = bpmp::interpolate(object_history_list_[obstacle_idx_list_[i]].t,
                                              object_history_list_[obstacle_idx_list_[i]].vx, t);
            temp_state.vy = bpmp::interpolate(object_history_list_[obstacle_idx_list_[i]].t,
                                              object_history_list_[obstacle_idx_list_[i]].vy, t);
            temp_state.vz = bpmp::interpolate(object_history_list_[obstacle_idx_list_[i]].t,
                                              object_history_list_[obstacle_idx_list_[i]].vz, t);
            current_obstacle_state_list_.push_back(temp_state);
        }
    }

    static double t_prev = t;
    
    double dt = t - t_prev;
    if (dt <= 0.0) dt = 1e-6;

    const size_t robot_count = current_unicycle_state_list_.size();
    static std::vector<double> psi_dot;
    if (psi_dot.size() != robot_count) psi_dot.assign(robot_count, 0.0);

    const auto clamp = [](double v, double lo, double hi) {
        return v < lo ? lo : (v > hi ? hi : v);
    };
    const auto perp = [](double x, double y) {
        return std::pair<double, double>{-y, x};
    };

    // parameters (from script/test/tmp.py)
    constexpr double Kv = 20.0;
    constexpr double Ft_max = 10.0;
    constexpr double c_roll = 0.0;
    constexpr double f_roll = 0.0;
    constexpr double eps_v = 0.1;
    constexpr double c_lat = 100.0;
    constexpr double mu_lat = 0.5;
    constexpr double Kw = 10.0;
    constexpr double tau_max = 5.0;
    constexpr double c_yaw = 0.0;
    constexpr double J_robot = 0.5;
    constexpr double m_eff = 10.0;
    constexpr double J_eff = 5.0;
    constexpr double normal_force = 200.0;

    const double theta = current_object_state_.theta;
    const double c_theta = cos(theta);
    const double s_theta = sin(theta);
    double vx = current_object_state_.vx;
    double vy = current_object_state_.vy;
    double omega = current_object_state_.theta_dot;

    double F_total_x = 0.0, F_total_y = 0.0;
    double tau_total = 0.0;
    std::vector<double> psi_ddot(robot_count, 0.0);

    for (size_t i = 0; i < robot_count; ++i) {
        const double r_obj_x = get<0>(init_unicycle_pose_list_[i]);
        const double r_obj_y = get<1>(init_unicycle_pose_list_[i]);

        const double r_w_x = c_theta * r_obj_x - s_theta * r_obj_y;
        const double r_w_y = s_theta * r_obj_x + c_theta * r_obj_y;

        const auto pperp = perp(r_w_x, r_w_y);
        const double v_i_x = vx + omega * pperp.first;
        const double v_i_y = vy + omega * pperp.second;

        const double psi = current_unicycle_state_list_[i].theta;
        const double c_psi = cos(psi);
        const double s_psi = sin(psi);
        const double t_x = c_psi;
        const double t_y = s_psi;
        const double n_x = -s_psi;
        const double n_y = c_psi;

        const double v_t = t_x * v_i_x + t_y * v_i_y;
        const double v_n = n_x * v_i_x + n_y * v_i_y;

        const double v_cmd = unicycle_control_input_list_[i].linear_speed;
        const double w_cmd = unicycle_control_input_list_[i].angular_speed;

        const double F_drive = clamp(Kv * (v_cmd - v_t), -Ft_max, Ft_max);
        const double F_roll = -c_roll * v_t - f_roll * tanh(v_t / eps_v);
        const double F_lat = -clamp(c_lat * v_n, -mu_lat * normal_force, mu_lat * normal_force);

        const double F_i_x = (F_drive + F_roll) * t_x + F_lat * n_x;
        const double F_i_y = (F_drive + F_roll) * t_y + F_lat * n_y;

        F_total_x += F_i_x;
        F_total_y += F_i_y;
        tau_total += r_w_x * F_i_y - r_w_y * F_i_x;

        const double tau_i = clamp(Kw * (w_cmd - psi_dot[i]), -tau_max, tau_max);
        psi_ddot[i] = (tau_i - c_yaw * psi_dot[i]) / J_robot;
    }

    const double a_x = F_total_x / m_eff;
    const double a_y = F_total_y / m_eff;
    const double alpha = tau_total / J_eff;

    // semi-implicit Euler
    vx += a_x * dt;
    vy += a_y * dt;
    omega += alpha * dt;

    const double theta_new = theta + omega * dt;
    for (size_t i = 0; i < robot_count; ++i) {
        psi_dot[i] += psi_ddot[i] * dt;
        current_unicycle_state_list_[i].theta += psi_dot[i] * dt;
        current_unicycle_state_list_[i].theta_dot = psi_dot[i];
    }

    current_object_state_.px += vx * dt;
    current_object_state_.py += vy * dt;
    current_object_state_.theta = theta_new;
    current_object_state_.vx = vx;
    current_object_state_.vy = vy;
    current_object_state_.theta_dot = omega;
    current_object_state_.ax = a_x;
    current_object_state_.ay = a_y;
    current_object_state_.az = 0.0;

    const double c_th = cos(theta_new);
    const double s_th = sin(theta_new);
    for (size_t i = 0; i < robot_count; ++i) {
        const double rx = get<0>(init_unicycle_pose_list_[i]);
        const double ry = get<1>(init_unicycle_pose_list_[i]);
        const double rwx = c_th * rx - s_th * ry;
        const double rwy = s_th * rx + c_th * ry;
        current_unicycle_state_list_[i].px = current_object_state_.px + rwx;
        current_unicycle_state_list_[i].py = current_object_state_.py + rwy;
    }

//    std::cout<<"DT: "<<dt<<std::endl;
    t_prev = t;
}

void bpmp::Simulator::ReadInitialTrackerStateList() {

    std::ifstream initial_state_file(initial_state_file_name_);
    if (!initial_state_file.is_open()) {
        throw std::runtime_error("failed to open " + initial_state_file_name_);
    }
    else{
        printf("INITIAL STATE FILE OPENED SUCCESSFULLY: %s\n", initial_state_file_name_.c_str());
    }

    std::string line;
    current_object_state_.vx = 0.0;
    current_object_state_.vy = 0.0;
    current_object_state_.vz = 0.0;
    current_object_state_.ax = 0.0;
    current_object_state_.ay = 0.0;
    current_object_state_.az = 0.0;

    // 1) 첫 줄: 현재 객체 상태 + 크기
    if (std::getline(initial_state_file, line)) {
        std::istringstream ss(line);
        ss >> current_object_state_.px
        >> current_object_state_.py
        >> current_object_state_.theta
        >> object_rectangle_size_.first
        >> object_rectangle_size_.second;
        current_object_state_.pz = 0.5;
        printf("INITIAL OBJECT STATE: PX: %.2f, PY: %.2f, THETA: %.2f, SIZE_X: %.2f, SIZE_Y: %.2f\n",
               current_object_state_.px, current_object_state_.py, current_object_state_.theta,
               object_rectangle_size_.first, object_rectangle_size_.second);
    }
    object_vis_.scale.x = object_rectangle_size_.first;
    object_vis_.scale.y = object_rectangle_size_.second;
    object_vis_.scale.z = 2.0;

    current_unicycle_state_list_.clear();

    robot_num_ = 0;
    tracker_vis_list_.clear();
    line.clear();

    while (std::getline(initial_state_file, line)) {
        
        std::istringstream ss(line);

        double x, y, theta;
        if (ss >> x >> y >> theta) {
            robot_num_ ++;
            UnicycleState obj{};
            printf("INITIAL UNICYCLE STATE %d: X: %.2f, Y: %.2f, THETA: %.2f\n", robot_num_-1, x, y, theta);
            obj.pz = 0.5;              // 기본값
            obj.px = current_object_state_.px + x * cos(current_object_state_.theta) - y * sin(current_object_state_.theta);  // 객체 위치에 상대적으로 배치
            obj.py = current_object_state_.py + x * sin(current_object_state_.theta) + y * cos(current_object_state_.theta);
            obj.theta = current_object_state_.theta + theta;
            current_unicycle_state_list_.push_back(obj);   // 컨테이너에 저장
            init_unicycle_pose_list_.emplace_back(x, y);
            unicycle_control_input_list_.emplace_back(); // 초기화된 제어 입력 추가

            // Modified as multiple unicycle trackers are supported
            ros::Publisher tracker_state_publisher = nh_.advertise<bpmp_tracker::UnicycleState>("tracker_state_" + std::to_string(robot_num_-1), 1);
            tracker_state_publisher_list_.push_back(tracker_state_publisher);
            unicycle_control_input_subscriber_list_.push_back(
                    nh_.subscribe<bpmp_tracker::UnicycleInput>(
                            "/bpmp_tracker/unicycle_control_input_" + std::to_string(robot_num_-1), 1,
                            boost::bind(&Simulator::unicycle_input_callback_multi_, this, _1, robot_num_-1)));
            tracker_vis_publisher_list_.push_back(nh_.advertise<nav_msgs::Odometry>("/base_odom_" + std::to_string(robot_num_-1), 1));
            tracker_vis_list_.emplace_back(); // 초기화된 시각화 메시지 추가
            tracker_vis_list_.back().header.frame_id = map_frame_id_;

        }
        line.clear();
    }
    initial_state_file.close();
    for (auto &input : unicycle_control_input_list_) {
        input.linear_speed = 0.0;
        input.angular_speed = 0.0;
    }

    printf("TOTAL UNICYCLE TRACKER NUMBER: %d\n", robot_num_);
    tracker_state_msg_list_.clear();
    for (int i = 0; i < robot_num_; i++) {
        bpmp_tracker::UnicycleState temp_msg;
        tracker_state_msg_list_.push_back(temp_msg);
    }
}

void bpmp::Simulator::PrepareRosMsgs(const double &t) {
    // Target Visualization
    target_vis_.pose.position.x = current_target_state_.px;
    target_vis_.pose.position.y = current_target_state_.py;
    target_vis_.pose.position.z = current_target_state_.pz;

    // Obstacle Visualization
    obstacle_list_vis_.markers.clear();
    for (int i = 0; i < current_obstacle_state_list_.size(); i++) {
        obstacle_vis_.id = i;
        obstacle_vis_.ns = std::to_string(i);
        obstacle_vis_.pose.position.x = current_obstacle_state_list_[i].px;
        obstacle_vis_.pose.position.y = current_obstacle_state_list_[i].py;
        obstacle_vis_.pose.position.z = current_obstacle_state_list_[i].pz;
        obstacle_list_vis_.markers.push_back(obstacle_vis_);
    }
    // Tracker Visualization (Nav_msgs in Unicycle Simulator)
    for (int i = 0; i < current_unicycle_state_list_.size(); i++) {
        tracker_vis_list_[i].header.stamp = ros::Time::now();
        tracker_vis_list_[i].pose.pose.position.x = current_unicycle_state_list_[i].px;
        tracker_vis_list_[i].pose.pose.position.y = current_unicycle_state_list_[i].py;
        tracker_vis_list_[i].pose.pose.position.z = 0.5;
        tracker_vis_list_[i].pose.pose.orientation.x = 0.0;
        tracker_vis_list_[i].pose.pose.orientation.y = 0.0;
        tracker_vis_list_[i].pose.pose.orientation.z = sin(0.5*current_unicycle_state_list_[i].theta);
        tracker_vis_list_[i].pose.pose.orientation.w = cos(0.5*current_unicycle_state_list_[i].theta);
        // Tracker tf Publish
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(tracker_vis_list_[i].pose.pose.position.x,tracker_vis_list_[i].pose.pose.position.y,tracker_vis_list_[i].pose.pose.position.z));
        tf::Quaternion q(tracker_vis_list_[i].pose.pose.orientation.x,tracker_vis_list_[i].pose.pose.orientation.y,tracker_vis_list_[i].pose.pose.orientation.z,tracker_vis_list_[i].pose.pose.orientation.w);
        transform.setRotation(q);
        br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "robot_" + std::to_string(i) + "_base_link"));
        tracker_state_msg_list_[i].px = current_unicycle_state_list_[i].px;
        tracker_state_msg_list_[i].py = current_unicycle_state_list_[i].py;
        tracker_state_msg_list_[i].pz = current_unicycle_state_list_[i].pz;
        tracker_state_msg_list_[i].theta = current_unicycle_state_list_[i].theta;
    }
    
    // target_state
    target_state_msg_.px = current_target_state_.px;
    target_state_msg_.py = current_target_state_.py;
    target_state_msg_.pz = current_target_state_.pz;
    target_state_msg_.vx = current_target_state_.vx;
    target_state_msg_.vy = current_target_state_.vy;
    target_state_msg_.vz = current_target_state_.vz;
    // obstacle_state
    bpmp_tracker::ObjectState object_state_temp;
    obstacle_state_list_msg_.object_state_list.clear();
    for (int i = 0; i < current_obstacle_state_list_.size(); i++) {
        object_state_temp.px = current_obstacle_state_list_[i].px;
        object_state_temp.py = current_obstacle_state_list_[i].py;
        object_state_temp.pz = current_obstacle_state_list_[i].pz;
        object_state_temp.vx = current_obstacle_state_list_[i].vx;
        object_state_temp.vy = current_obstacle_state_list_[i].vy;
        object_state_temp.vz = current_obstacle_state_list_[i].vz;
        obstacle_state_list_msg_.object_state_list.push_back(object_state_temp);
    }

    // object_vis
    object_vis_.pose.position.x = current_object_state_.px;
    object_vis_.pose.position.y = current_object_state_.py;
    object_vis_.pose.position.z = 0.5;
    object_vis_.pose.orientation.x = 0.0;
    object_vis_.pose.orientation.y = 0.0;
    object_vis_.pose.orientation.z = sin(0.5*current_object_state_.theta);
    object_vis_.pose.orientation.w = cos(0.5*current_object_state_.theta);

    object_state_msg_.px = current_object_state_.px;
    object_state_msg_.py = current_object_state_.py;
    object_state_msg_.pz = current_object_state_.pz;
    object_state_msg_.vx = current_object_state_.vx;
    object_state_msg_.vy = current_object_state_.vy;
    object_state_msg_.vz = current_object_state_.vz;
    object_state_msg_.ax = current_object_state_.ax;
    object_state_msg_.ay = current_object_state_.ay;
    object_state_msg_.az = current_object_state_.az;
    object_state_msg_.theta = current_object_state_.theta;
    object_state_msg_.theta_dot = current_object_state_.theta_dot;

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(object_vis_.pose.position.x,object_vis_.pose.position.y,object_vis_.pose.position.z));
    tf::Quaternion q(object_vis_.pose.orientation.x,object_vis_.pose.orientation.y,object_vis_.pose.orientation.z,object_vis_.pose.orientation.w);
    transform.setRotation(q);
    br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "object_base_link"));


}

void bpmp::Simulator::PublishRosMsgs() {
    for (int i = 0; i < current_unicycle_state_list_.size(); i++) {
        tracker_vis_publisher_list_[i].publish(tracker_vis_list_[i]);
        tracker_state_publisher_list_[i].publish(tracker_state_msg_list_[i]);
    }
    object_state_publisher_.publish(object_state_msg_);
    object_vis_publisher_.publish(object_vis_);
    obstacle_list_vis_publisher_.publish(obstacle_list_vis_);
    target_vis_publisher_.publish(target_vis_);
    if(is_unstructured_){
        pcl_publisher_.publish(point_cloud_);
        pcl_boxes_vis_publisher_.publish(pcl_boxes_vis_);
    }
    obstacle_state_list_publisher_.publish(obstacle_state_list_msg_);
    target_state_publisher_.publish(target_state_msg_);
}

void bpmp::Simulator::ReadObstacleConfiguration() {
    std::ifstream obstacle_file;
    if(obstacle_configuration_file_name_.c_str()==""){
        cout<<"NO PCL FILE"<<endl;
        return;
    }
    else
        cout<<"YES PCL FILE"<<endl;
    obstacle_file.open(obstacle_configuration_file_name_.c_str());
    double v1[2]{0.0, 0.0};
    double v2[2]{0.0, 0.0};
    double min_x, min_y, max_x, max_y;
    double p1[2], p2[2], p3[2], p4[2];
    int num_point;
    /*
     *  p1 --------p2
     *  -          -
     *  -          -
     *  p3---------p4
     */
    pcl::PointXYZ temp_point;

    visualization_msgs::Marker single_obstacle;
    single_obstacle.pose.orientation.w = 1.0;
    single_obstacle.pose.orientation.x = 0.0;
    single_obstacle.pose.orientation.y = 0.0;
    single_obstacle.pose.orientation.z = 0.0;
    single_obstacle.color.a = 1.0;
    single_obstacle.color.r = 0.0;
    single_obstacle.color.g = 0.0;
    single_obstacle.color.b = 0.0;
    single_obstacle.ns = "PCL_OBSTACLE";
    single_obstacle.header.frame_id = map_frame_id_;
    single_obstacle.type = visualization_msgs::Marker::CUBE;
    int id = 0;
    if (obstacle_file.is_open()) {
        while (obstacle_file >> v1[0] >> v1[1] >> v2[0] >> v2[1]) {
            if (v1[0] <= v2[0]) {
                min_x = v1[0];
                max_x = v2[0];
            } else {
                min_x = v2[0];
                max_x = v1[0];
            }
            if (v1[1] <= v2[1]) {
                min_y = v1[1];
                max_y = v2[1];
            } else {
                min_y = v2[1];
                max_y = v1[1];
            }
            p1[0] = min_x - inflation_size_;
            p1[1] = max_y + inflation_size_;
            p2[0] = max_x + inflation_size_;
            p2[1] = max_y + inflation_size_;
            p3[0] = min_x - inflation_size_;
            p3[1] = min_y - inflation_size_;
            p4[0] = max_x + inflation_size_;
            p4[1] = min_y - inflation_size_;
            // p1 - p2
            num_point = floor((p2[0] - p1[0]) / point_resolution_ + 0.5);
            for (int i = 0; i <= num_point; i++) {
                temp_point.x = float(p1[0] + (p2[0] - p1[0]) / num_point * i);
                temp_point.y = float(p1[1] + (p2[1] - p1[1]) / num_point * i);
                temp_point.z = 1.0;
                point_cloud_.push_back(temp_point);
            }
            //p1 - p3
            num_point = floor((p1[1] - p3[1]) / point_resolution_ + 0.5);
            for (int i = 0; i <= num_point; i++) {
                temp_point.x = float(p1[0] + (p3[0] - p1[0]) / num_point * i);
                temp_point.y = float(p1[1] + (p3[1] - p1[1]) / num_point * i);
                temp_point.z = 1.0;
                point_cloud_.push_back(temp_point);
            }
            // p3 - p4
            num_point = floor((p4[0] - p3[0]) / point_resolution_ + 0.5);
            for (int i = 0; i <= num_point; i++) {
                temp_point.x = float(p3[0] + (p4[0] - p3[0]) / num_point * i);
                temp_point.y = float(p3[1] + (p4[1] - p3[1]) / num_point * i);
                temp_point.z = 1.0;
                point_cloud_.push_back(temp_point);
            }
            // p4 - p2
            num_point = floor((p2[1] - p4[1]) / point_resolution_ + 0.5);
            for (int i = 0; i <= num_point; i++) {
                temp_point.x = float(p4[0] + (p2[0] - p4[0]) / num_point * i);
                temp_point.y = float(p4[1] + (p2[1] - p4[1]) / num_point * i);
                temp_point.z = 1.0;
                point_cloud_.push_back(temp_point);
            }
            single_obstacle.id = id++;
            single_obstacle.pose.position.x = 0.25 * (p1[0] + p2[0] + p3[0] + p4[0]);
            single_obstacle.pose.position.y = 0.25 * (p1[1] + p2[1] + p3[1] + p4[1]);
            single_obstacle.pose.position.z = 1.0;
            single_obstacle.scale.x = p2[0] - p1[0];
            single_obstacle.scale.y = p1[1] - p3[1];
            single_obstacle.scale.z = 2.0;
            pcl_boxes_vis_.markers.push_back(single_obstacle);
        }
    }
    obstacle_file.close();
}

void bpmp::Simulator::unicycle_input_callback_multi_(const bpmp_tracker::UnicycleInput::ConstPtr &msg, int robot_num) {
    unicycle_control_input_list_[robot_num].linear_speed = msg->vel_linear;
    unicycle_control_input_list_[robot_num].angular_speed = msg->vel_angular;
}
