#include "comparison/Baseline.h"

bpmp::Baseline::Baseline(): nh_("~") {

    // ---- 필수 파라미터들 (없으면 default) ----
    nh_.param<double>("time_step", param_.time_step, 0.5);

    nh_.param<double>("w_max", param_.w_max, 1.0);
    nh_.param<double>("a_min", param_.a_min, -1.0);
    nh_.param<double>("a_max", param_.a_max,  1.0);
    nh_.param<double>("v0",    param_.v0,    0.0);
    nh_.param<double>("v_max", param_.v_max, 1.0);

    double cov_temp;
    nh_.param<double>("robot_process_noise", cov_temp, 0.05);
    param_.robot_process_noise = Eigen::Matrix<double, Nx_r, Nx_r>::Identity() * cov_temp;
    nh_.param<double>("target_process_noise", cov_temp, 0.1);
    param_.target_process_noise = Eigen::Matrix<double, Nx_t, Nx_t>::Identity() * cov_temp;
    nh_.param<double>("target_sensor_noise",  cov_temp,  0.05);
    param_.target_sensor_noise = Eigen::Matrix<double, 2, 2>::Identity() * cov_temp;

    nh_.param<double>("obs_radius", param_.obs_radius, 0.5);

    nh_.param<double>("fov_angle", param_.fov_angle, M_PI/3.0);
    nh_.param<double>("r_min", param_.r_min, 1.0);
    nh_.param<double>("r_max", param_.r_max, 10.0);

    nh_.param<std::string>("function_J", param_.function_J, std::string("2"));
    nh_.param<double>("gamma_collision_threshold", param_.gamma_collision_threshold, 0.2);

    nh_.param<double>("eta0", param_.eta0, 1.0);
    nh_.param<double>("eta_max", param_.eta_max, 1e3);
    nh_.param<double>("beta", param_.beta, 10.0);
    nh_.param<double>("tau_p", param_.tau_p, 1e-2);

    nh_.param<double>("d0", param_.d0, 0.25);
    nh_.param<double>("d_min", param_.d_min, 1e-3);
    nh_.param<double>("d_max", param_.d_max, 2.0);
    nh_.param<double>("shrink", param_.shrink, 0.5);
    nh_.param<double>("expand", param_.expand, 1.5);
    nh_.param<double>("rho_reject", param_.rho_reject, 0.25);
    nh_.param<double>("rho_expand", param_.rho_expand, 0.75);

    nh_.param<int>("max_inner", param_.max_inner, 20);
    nh_.param<int>("max_outer", param_.max_outer, 10);
    nh_.param<double>("tau_conv", param_.tau_conv, 1e-3);
    nh_.param<double>("tau_f", param_.tau_f, 1e-5);

    nh_.param<double>("grad_delta", param_.grad_delta, 1e-6);
    nh_.param<bool>("verbose", param_.verbose, false);

    // ---- SUBSCRIBER ----
    tracker_state_subscriber_ = nh_.subscribe("/base_odom", 1,
                                              &Baseline::TrackerStateCallback, this);
    target_state_subscriber_ = nh_.subscribe("/bpmp_simulator/target_state", 1,
                                             &Baseline::TargetStateCallback, this);
    obstacle_state_list_subscriber_ = nh_.subscribe("/bpmp_simulator/obstacle_state_list", 1,
                                                    &Baseline::ObstacleStateListCallback, this);

    // ---- PUBLISHER ----
    tracker_control_input_publisher_ =
        nh_.advertise<bpmp_tracker::UnicycleInput>("/bpmp_tracker/unicycle_control_input", 1);
    active_obstacle_marker_array_publisher_ =
        nh_.advertise<visualization_msgs::MarkerArray>("/active_obstacle_list", 1);
    fov_sector_marker_publisher_ =
        nh_.advertise<visualization_msgs::Marker>("/fov_sector", 1);
    robot_planning_path_publisher_ = 
        nh_.advertise<visualization_msgs::Marker>("/robot_planning_path", 1);
    target_predicted_path_publisher_ =
        nh_.advertise<visualization_msgs::Marker>("/target_predicted_path", 1);
    cur_robot_vel_path_publisher_ =
        nh_.advertise<visualization_msgs::Marker>("/cur_robot_vel_path", 1);

}

void bpmp::Baseline::Run() {
    cout << "Baseline node started." << endl;
    ros::Rate loop_rate(20.0);
    while(ros::ok()){
        if(is_info_received()){
            MakeControl();
            // cout << "Baseline control published." << endl;
        }
        else{
            cout << "Waiting for tracker/target state..." << endl;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

bool bpmp::Baseline::is_info_received() {
    // obstacle은 상황에 따라 비어있을 수도 있어서 필수로 강제하진 않음
    return is_target_info_received && is_tracker_info_received;
}

void bpmp::Baseline::MakeControl() {
    // ---- warm start (paper: extrapolate previous solution) ----
    static bool has_prev = false;
    static double prev_time = 0.0;
    static Collection<Eigen::Matrix<double,Nu,1>,N> u_prev;
    static double prev_control_v = 0.0;

    Collection<Eigen::Matrix<double,Nu,1>,N> u0;
    if(!has_prev){
        for(int k=0;k<N;k++){
            u0[k] = (Eigen::Matrix<double,Nu,1>() << 0.0, 0.0).finished();
        }
        prev_time = ros::Time::now().toSec();
    }else{
        // ver1 : shift left by one, repeat last
        // for(int k=0;k<N-1;k++){
        //     u0[k] = u_prev[k+1];
        // }
        // u0[N-1] = u_prev[N-1];

        // ver2 : use previous control as initial guess without shifting
        // u0 = u_prev;
    }

    // build problem
    std::shared_ptr<bpmp::Problem> prob =
        std::make_shared<bpmp::Problem>(current_target_state_,
                                        current_obstacle_state_list_,
                                        current_tracker_state_);

    Eigen::Matrix<double,Nx_r,1> x0_new;
    x0_new = (Eigen::Matrix<double,Nx_r,1>()<<
              current_tracker_state_.px,
              current_tracker_state_.py,
              current_tracker_state_.theta,
              current_tracker_state_.velocity).finished();

    Collection<Eigen::Matrix<double,Nu,1>,N> uN_new = u0;

    // solve
    Optimizer<N,Nu> optimizer(*prob, uN_new, param_.time_step, param_);
    optimizer.Solve();
    const auto& u_sol = optimizer.solution();

    // save for next extrapolation
    u_prev = u_sol;
    has_prev = true;

    // ---- publish first control ----
    bpmp_tracker::UnicycleInput cmd;
    // UnicycleInput.msg: vel_linear, vel_angular
    cmd.vel_angular = std::max(std::min(u_sol[0](0), param_.w_max), -param_.w_max);  // w (angular velocity)
    // cmd.vel_linear = u_sol[0](1);   // a (acceleration/linear velocity)
    // -> 수정: UnicycleInput에서 vel_linear는 속도이므로, 현재 속도 + 가속도로 설정
    double dt = ros::Time::now().toSec() - prev_time;
    prev_time = ros::Time::now().toSec();
    double acc = std::max(std::min(u_sol[0](1), param_.a_max), param_.a_min); // a (acceleration)
    // cmd.vel_linear = std::max(std::min(current_tracker_state_.velocity + acc * dt, param_.v_max), -param_.v_max); // v = v0 + a*dt
    std::cout << "[Baseline] Current velocity: " << current_tracker_state_.velocity << ", Acc: " << acc << ", prev_control_v: " << prev_control_v << ", dt: " << dt << std::endl;
    cmd.vel_linear = std::max(std::min(prev_control_v + acc * dt, param_.v_max), -param_.v_max); // v = v0 + a*dt
    prev_control_v = cmd.vel_linear;
    current_tracker_state_.velocity = cmd.vel_linear; // update current velocity
    tracker_control_input_publisher_.publish(cmd);

    // publish robot planning path marker
    visualization_msgs::Marker path_marker;
    path_marker.header.frame_id = "map";
    path_marker.ns = "robot_planning_path";
    path_marker.id = 0;
    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::Marker::ADD;
    path_marker.scale.x = 0.1; // line width
    path_marker.color.r = 0.2;
    path_marker.color.g = 0.8;
    path_marker.color.b = 0.2;
    path_marker.color.a = 0.9;
    path_marker.lifetime = ros::Duration(0.2);
    path_marker.points.clear();
    // initial position
    geometry_msgs::Point p_init;
    p_init.x = current_tracker_state_.px;
    p_init.y = current_tracker_state_.py;
    p_init.z = 0.1;
    path_marker.points.push_back(p_init);
    // predicted positions
    Eigen::Matrix<double,Nx_r,1> xk = x0_new;
    for(int k=0; k<N; k++){
        // simulate one step
        double theta = xk(2);
        double v = xk(3);
        double w = u_sol[k](0);
        double a = u_sol[k](1);
        // Euler integration
        xk(0) += v * cos(theta) * param_.time_step;
        xk(1) += v * sin(theta) * param_.time_step;
        xk(2) += w * param_.time_step;
        // xk(3) = std::min(std::max(xk(3) + a * param_.time_step, -param_.v_max), param_.v_max);
        xk(3) += a * param_.time_step;
        // add to marker
        geometry_msgs::Point p;
        p.x = xk(0);
        p.y = xk(1);
        p.z = 0.1;
        path_marker.points.push_back(p);
    }

    robot_planning_path_publisher_.publish(path_marker);

    // publish target predicted path marker
    visualization_msgs::Marker target_path_marker;
    target_path_marker.header.frame_id = "map";
    target_path_marker.ns = "target_predicted_path";
    target_path_marker.id = 0;
    target_path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    target_path_marker.action = visualization_msgs::Marker::ADD;
    target_path_marker.scale.x = 0.1; // line width
    target_path_marker.color.r = 0.2;
    target_path_marker.color.g = 0.2;
    target_path_marker.color.b = 0.8;
    target_path_marker.color.a = 0.9;
    target_path_marker.lifetime = ros::Duration(0.2);
    target_path_marker.points.clear();
    // initial position
    geometry_msgs::Point tp_init;
    tp_init.x = current_target_state_.px;
    tp_init.y = current_target_state_.py;
    tp_init.z = 0.1;
    target_path_marker.points.push_back(tp_init);
    // predicted positions (assuming constant velocity model)
    Eigen::Matrix<double,4,1> txk;
    txk = (Eigen::Matrix<double,4,1>()<<
              current_target_state_.px,
                current_target_state_.py,
                0.0,
                0.0).finished(); // pz, vz are not used
    for(int k=0; k<N; k++){
        // simulate one step
        double vtx = current_target_state_.vx;
        double vty = current_target_state_.vy;
        // Euler integration
        txk(0) += vtx * param_.time_step;
        txk(1) += vty * param_.time_step;
        // add to marker
        geometry_msgs::Point p;
        p.x = txk(0);
        p.y = txk(1);
        p.z = 0.1;
        target_path_marker.points.push_back(p);
    }
    target_predicted_path_publisher_.publish(target_path_marker);

    // publish current robot velocity vector marker
    visualization_msgs::Marker vel_marker;
    vel_marker.header.frame_id = "map";
    vel_marker.ns = "cur_robot_vel_path";
    vel_marker.id = 0;
    vel_marker.type = visualization_msgs::Marker::LINE_STRIP;
    vel_marker.action = visualization_msgs::Marker::ADD;
    vel_marker.scale.x = 0.2; // shaft diameter
    vel_marker.scale.y = 0.4; // head diameter
    vel_marker.scale.z = 0.4; // head length
    vel_marker.color.r = 0.8;
    vel_marker.color.g = 0.2;
    vel_marker.color.b = 0.2;
    vel_marker.color.a = 0.9;
    vel_marker.lifetime = ros::Duration(0.2);
    // start point
    geometry_msgs::Point sp;
    sp.x = current_tracker_state_.px;
    sp.y = current_tracker_state_.py;
    sp.z = 0.1;
    vel_marker.points.push_back(sp);
    // intermediate point (for better visibility, constant velocity assumption)
    double next_x = current_tracker_state_.px;
    double next_y = current_tracker_state_.py;
    double next_theta = current_tracker_state_.theta;
    for (int i = 1; i <= N; i++) {
        geometry_msgs::Point mp;
        next_x += (current_tracker_state_.velocity * cos(next_theta)) * param_.time_step;
        next_y += (current_tracker_state_.velocity * sin(next_theta)) * param_.time_step;
        next_theta += (cmd.vel_angular) * param_.time_step; // assuming constant angular velocity
        mp.x = next_x;
        mp.y = next_y;
        mp.z = 0.1;
        vel_marker.points.push_back(mp);
    }
    cur_robot_vel_path_publisher_.publish(vel_marker);

}

void bpmp::Baseline::ObstacleStateListCallback(const bpmp_tracker::ObjectStateList &msg) {
    is_obstacle_info_received = true;

    // ★ 누적 버그 방지: 매 callback마다 clear
    current_obstacle_state_list_.clear();
    current_obstacle_state_list_.reserve(msg.object_state_list.size());

    bpmp::State obstacle_state;
    for (size_t i = 0; i < msg.object_state_list.size(); i++) {
        
        obstacle_state.px = msg.object_state_list[i].px;
        obstacle_state.py = msg.object_state_list[i].py;
        obstacle_state.pz = msg.object_state_list[i].pz;
        // if(isInFOV(obstacle_state)) { // 제거
            current_obstacle_state_list_.push_back(obstacle_state);
        // }
    }

    // [ADD] Active obstacle marker publish
    // clear previous markers by publishing with DELETEALL action
    active_obstacle_marker_array_.markers.clear();
    visualization_msgs::Marker delete_marker;
    delete_marker.action = visualization_msgs::Marker::DELETEALL;
    active_obstacle_marker_array_.markers.push_back(delete_marker);
    active_obstacle_marker_array_publisher_.publish(active_obstacle_marker_array_);
    active_obstacle_marker_array_.markers.clear();
    for (size_t i = 0; i < current_obstacle_state_list_.size(); i++) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.ns = "active_obstacle";
        marker.id = i;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = current_obstacle_state_list_[i].px;
        marker.pose.position.y = current_obstacle_state_list_[i].py;
        marker.pose.position.z = 3.0;
        marker.pose.orientation.w = 1.0;

        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;

        // 끊김 방지용: publish가 끊기면 자동 소거
        marker.lifetime = ros::Duration(0.2);

        // 기본색(빨강)
        marker.color.r = 1.0;
        marker.color.g = 0.2;
        marker.color.b = 0.2;
        marker.color.a = 0.9;

        active_obstacle_marker_array_.markers.push_back(marker);
        // cout << "[Baseline] Active obstacle " << i
        //      << " at (" << marker.pose.position.x << ", "
        //      << marker.pose.position.y << ", "
        //      << marker.pose.position.z << ")" << endl;
    }
    active_obstacle_marker_array_publisher_.publish(active_obstacle_marker_array_);
}

bool bpmp::Baseline::isInFOV(bpmp::State obstacle_state) {
    // FOV 내에 있는지 확인
    double dx = obstacle_state.px - current_tracker_state_.px;
    double dy = obstacle_state.py - current_tracker_state_.py;
    double distance = std::hypot(dx, dy);
    if (distance < param_.r_min || distance > param_.r_max) {
        return false;
    }

    double angle_to_obstacle = std::atan2(dy, dx);
    double angle_diff = angle_to_obstacle - current_tracker_state_.theta;

    // 각도 차이를 [-pi, pi] 범위로 조정
    while (angle_diff > M_PI) angle_diff -= 2.0 * M_PI;
    while (angle_diff < -M_PI) angle_diff += 2.0 * M_PI;

    if (std::abs(angle_diff) > param_.fov_angle / 2.0) {
        return false;
    }

    return true;
}

void bpmp::Baseline::TrackerStateCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    double vx = 0.0;
    double vy = 0.0;
    double dt = 0.0;
    double t_cur = ros::Time::now().toSec();
    if (!is_tracker_info_received) {
        // copy initial odom
        prev_tracker_time_ = t_cur;
        prev_tracker_state_.px = msg->pose.pose.position.x;
        prev_tracker_state_.py = msg->pose.pose.position.y;
    }
    else if(std::hypot(prev_tracker_state_.px-msg->pose.pose.position.x,
                    prev_tracker_state_.py-msg->pose.pose.position.y) > 0.1) {
        // pose is jumped!!
        cout << "[Baseline] Warning: Tracker odometry jumped. Resetting velocity to zero." << endl;
        prev_tracker_time_ = t_cur;
        prev_tracker_state_.px = msg->pose.pose.position.x;
        prev_tracker_state_.py = msg->pose.pose.position.y;
        prev_tracker_state_.velocity = 0.0;
    }
    else{;
        dt = (t_cur - prev_tracker_time_);
        if (dt <= 1e-6) { 
            cout << "[Baseline] Warning: Non-positive time difference in tracker odometry. Setting dt to 1e-6." << endl;
            cout << msg->header.stamp.toSec() << endl;
            dt = 1e-6; // to avoid zero division
        }
        vx = (msg->pose.pose.position.x - prev_tracker_state_.px) / (dt);
        vy = (msg->pose.pose.position.y - prev_tracker_state_.py) / (dt);
        prev_tracker_state_.px = msg->pose.pose.position.x;
        prev_tracker_state_.py = msg->pose.pose.position.y;
        prev_tracker_time_ = t_cur;
    }
    is_tracker_info_received = true;
    

    current_tracker_state_.px = msg->pose.pose.position.x;
    current_tracker_state_.py = msg->pose.pose.position.y;

    double q[4] {msg->pose.pose.orientation.w,
                 msg->pose.pose.orientation.x,
                 msg->pose.pose.orientation.y,
                 msg->pose.pose.orientation.z};

    current_tracker_state_.theta = atan2(
            2.0 * (q[0] * q[3] + q[1] * q[2]),
            1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3])
    );
    int velocity_sign = (vx * cos(current_tracker_state_.theta) + vy * sin(current_tracker_state_.theta) >= 0) ? 1 : -1;
    // double cur_vel = velocity_sign * std::min(std::hypot(vx, vy), param_.v_max);
    // current_tracker_state_.velocity = current_tracker_state_.velocity*0.8 + cur_vel*0.2; // low-pass filter
    // current_tracker_state_.velocity = velocity_sign * std::min(std::hypot(vx, vy), param_.v_max);
    // current_tracker_state_.velocity = velocity_sign * std::hypot(vx, vy);

    // [ADD] FOV sector marker publish
    visualization_msgs::Marker fov_marker;
    fov_marker.header.frame_id = "map";
    fov_marker.header.stamp = ros::Time::now();
    fov_marker.ns = "fov_sector";
    fov_marker.id = 0;
    fov_marker.type = visualization_msgs::Marker::LINE_STRIP;
    fov_marker.action = visualization_msgs::Marker::ADD;
    fov_marker.pose.position.x = 0;
    fov_marker.pose.position.y = 0;
    fov_marker.pose.position.z = 1.0;
    fov_marker.pose.orientation.w = 1.0;
    fov_marker.scale.x = 0.1;
    fov_marker.color.a = 1.0;
    fov_marker.color.r = 0.0;
    fov_marker.color.g = 1.0;
    fov_marker.color.b = 0.0;
    double angle_left = current_tracker_state_.theta + param_.fov_angle / 2.0;
    double angle_right = current_tracker_state_.theta - param_.fov_angle / 2.0;
    for (int i = 0; i < 10; i++) {
        double angle = angle_right + i * (angle_left - angle_right) / 9.0;
        geometry_msgs::Point p;
        p.x = current_tracker_state_.px + param_.r_max * cos(angle);
        p.y = current_tracker_state_.py + param_.r_max * sin(angle);
        p.z = 1.0;
        fov_marker.points.push_back(p);
    }
    for (int i = 9; i >= 0; i--) {
        double angle = angle_right + i * (angle_left - angle_right) / 9.0;
        geometry_msgs::Point p;
        p.x = current_tracker_state_.px + param_.r_min * cos(angle);
        p.y = current_tracker_state_.py + param_.r_min * sin(angle);
        p.z = 1.0;
        fov_marker.points.push_back(p);
    }
    geometry_msgs::Point p;
    p.x = current_tracker_state_.px + param_.r_max * cos(angle_right);
    p.y = current_tracker_state_.py + param_.r_max * sin(angle_right);
    p.z = 1.0;
    fov_marker.points.push_back(p);

    // geometry_msgs::Point p_start;
    // p_start.x = current_tracker_state_.px;
    // p_start.y = current_tracker_state_.py;
    // p_start.z = 1.0;
    // geometry_msgs::Point p_left;
    // p_left.x = current_tracker_state_.px + param_.r_max * cos(angle_left);
    // p_left.y = current_tracker_state_.py + param_.r_max * sin(angle_left);
    // p_left.z = 1.0;
    // geometry_msgs::Point p_right;
    // p_right.x = current_tracker_state_.px + param_.r_max * cos(angle_right);
    // p_right.y = current_tracker_state_.py + param_.r_max * sin(angle_right);
    // p_right.z = 1.0;
    // geometry_msgs::Point p_end;
    // p_end.x = current_tracker_state_.px;
    // p_end.y = current_tracker_state_.py;
    // p_end.z = 1.0;
    // fov_marker.points.push_back(p_start);
    // fov_marker.points.push_back(p_left);
    // fov_marker.points.push_back(p_right);
    // fov_marker.points.push_back(p_end);
    fov_sector_marker_publisher_.publish(fov_marker);
}

void bpmp::Baseline::TargetStateCallback(const bpmp_tracker::ObjectState &msg) {
    is_target_info_received = true;
    current_target_state_.px = msg.px;
    current_target_state_.py = msg.py;
    current_target_state_.vx = msg.vx;
    current_target_state_.vy = msg.vy;
}
