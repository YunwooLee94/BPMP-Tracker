//
// Created by larr-laptop on 12/25/25.
//
#include <comparison/TrackingController.h>
#include <cmath>
#include <algorithm>
#include <sstream>

void bpmp::TrackingController::Run() {
    ros::Rate loop_rate(50.0);
    while (ros::ok()) {
        if (is_info_received()) MakeControl();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void bpmp::TrackingController::MakeControl() {
    using std::min;
    using std::max;

    auto clamp = [](double x, double lo, double hi) {
        return std::min(hi, std::max(lo, x));
    };
    auto wrap_pi = [](double a) {
        return std::atan2(std::sin(a), std::cos(a));  // (-pi, pi]
    };

    const double eps = 1e-9;

    // Current state
    const double rx = current_tracker_state_.px;
    const double ry = current_tracker_state_.py;
    const double th = current_tracker_state_.theta;

    const double tx = current_target_state_.px;
    const double ty = current_target_state_.py;

    // =========================
    // (VIS) Tracking circle around target
    // =========================
    tracking_circle_marker_.header.stamp = ros::Time::now();
    tracking_circle_marker_.pose.position.x = tx;
    tracking_circle_marker_.pose.position.y = ty;
    tracking_circle_marker_.pose.position.z = 1.0;
    tracking_circle_marker_.points.clear();
    {
        const int N = 72;
        for (int i = 0; i <= N; ++i) {
            const double a = (2.0 * M_PI) * (double)i / (double)N;
            geometry_msgs::Point p;
            p.x = param_.tracking_radius * std::cos(a);
            p.y = param_.tracking_radius * std::sin(a);
            p.z = 0.0;
            tracking_circle_marker_.points.push_back(p);
        }
    }
    tracking_circle_publisher_.publish(tracking_circle_marker_);

    // =========================================================
    // 1) Nearest obstacle (if available)
    // =========================================================
    bool has_obstacle = false;
    bpmp::State nearest_obs;
    int nearest_obs_idx = -1;
    double d_obs_min = 1e9;
    const double infl = max(param_.sonar_detection_range, eps);
    bool in_range = false;        // d_obs_min < infl
    bool avoid_active = false;    // in_range && p1>0 && (actually used to modify P)

    if (is_obstacle_info_received && !current_obstacle_state_list_.empty()) {
        for (int i = 0; i < (int)current_obstacle_state_list_.size(); ++i) {
            const auto& obs = current_obstacle_state_list_[i];
            const double d = std::hypot(obs.px - rx, obs.py - ry);
            if (d < d_obs_min) {
                d_obs_min = d;
                nearest_obs = obs;
                nearest_obs_idx = i;
            }
        }
        has_obstacle = (nearest_obs_idx >= 0);
    }
    in_range = has_obstacle && (d_obs_min < infl);


    // =========================================================
    // 2) Local goal P on tracking circle (+ optional obstacle augmentation)
    //    P0 = T + rd * unit(T->R)
    // =========================================================
    double tr_x = rx - tx;
    double tr_y = ry - ty;
    double tr_n = std::hypot(tr_x, tr_y);
    if (tr_n < eps) {  // degenerate: use heading
        tr_x = std::cos(th);
        tr_y = std::sin(th);
        tr_n = 1.0;
    }
    const double u_tr_x = tr_x / tr_n;
    const double u_tr_y = tr_y / tr_n;

    // Base local goal on the tracking circle (DO NOT modify tracking_radius usage)
    const double P0x = tx + param_.tracking_radius * u_tr_x;
    const double P0y = ty + param_.tracking_radius * u_tr_y;

    double px = P0x;
    double py = P0y;

    // --- Obstacle augmentation (Fig.3 style) ---
    if (has_obstacle && in_range) {
        const double ox = nearest_obs.px;
        const double oy = nearest_obs.py;
    
        double ot_x = ox - rx;
        double ot_y = oy - ry;
        double ot_n = std::hypot(ot_x, ot_y);
        if (ot_n < eps) ot_n = eps;
        ot_x /= ot_n;  ot_y /= ot_n;
        const double on_x = -ot_y;
        const double on_y =  ot_x;
    
        const double rp_x = P0x - rx;
        const double rp_y = P0y - ry;
        const double p1 = rp_x * ot_x + rp_y * ot_y;
        const double p2 = rp_x * on_x + rp_y * on_y;
    
        avoid_active = (p1 > 0.0);  // in_range는 이미 만족
    
        if (avoid_active) {
            double kj = clamp(param_.gain_o, 0.0, 1.0);
    
            const double rp_new_x = (kj * p1) * ot_x + (p2) * on_x;
            const double rp_new_y = (kj * p1) * ot_y + (p2) * on_y;
    
            px = rx + rp_new_x;
            py = ry + rp_new_y;
    
            // 필요하면 로그는 유지 (MarkerArray는 제거)
            const double dp = std::hypot(px - P0x, py - P0y);
            if (dp > 1e-4) {
                const double obs_speed = std::hypot(nearest_obs.vx, nearest_obs.vy);
                ROS_INFO_THROTTLE(0.5,
                    "[P-augment] P:(%.2f,%.2f)->(%.2f,%.2f) | idx=%d dO=%.2f infl=%.2f kj=%.2f p1=%.2f p2=%.2f vO=%.2f",
                    P0x, P0y, px, py, nearest_obs_idx, d_obs_min, infl, kj, p1, p2, obs_speed);
            }
        }
    }

    // =========================================================
    // (VIS) Active obstacle marker (always updated)
    //   - green: nearest exists but not actively modifying P
    //   - yellow: actively modifies P (in_range && p1>0)
    // =========================================================
    active_obstacle_marker_.header.stamp = ros::Time::now();

    if (!has_obstacle) {
        active_obstacle_marker_.action = visualization_msgs::Marker::DELETE;
        active_obstacle_marker_publisher_.publish(active_obstacle_marker_);
    } else {
        active_obstacle_marker_.action = visualization_msgs::Marker::ADD;
        active_obstacle_marker_.pose.position.x = nearest_obs.px;
        active_obstacle_marker_.pose.position.y = nearest_obs.py;
        active_obstacle_marker_.pose.position.z = 3.0;

        if (!in_range) {
            // 멀면 흐릿한 초록
            active_obstacle_marker_.color.r = 0.2;
            active_obstacle_marker_.color.g = 1.0;
            active_obstacle_marker_.color.b = 0.2;
            active_obstacle_marker_.color.a = 0.35;
        } else if (!avoid_active) {
            // 범위 내지만 P 수정에는 영향 없음: 초록
            active_obstacle_marker_.color.r = 0.2;
            active_obstacle_marker_.color.g = 1.0;
            active_obstacle_marker_.color.b = 0.2;
            active_obstacle_marker_.color.a = 0.9;
        } else {
            // ✅ 진짜로 P 수정에 영향을 준 장애물: 노랑
            active_obstacle_marker_.color.r = 1.0;
            active_obstacle_marker_.color.g = 1.0;
            active_obstacle_marker_.color.b = 0.2;
            active_obstacle_marker_.color.a = 0.95;
        }

        active_obstacle_marker_publisher_.publish(active_obstacle_marker_);
    }

    // P0 (base) / P (final) visualization (always published for debugging)
    visualization_msgs::MarkerArray p_markers;
    p_markers.markers.resize(3);

    // P0 (green sphere)
    p_markers.markers[0].header.frame_id = "map";
    p_markers.markers[0].header.stamp = ros::Time::now();
    p_markers.markers[0].ns = "p_augment";
    p_markers.markers[0].id = 0;
    p_markers.markers[0].type = visualization_msgs::Marker::SPHERE;
    p_markers.markers[0].action = visualization_msgs::Marker::ADD;
    p_markers.markers[0].pose.orientation.w = 1.0;
    p_markers.markers[0].pose.position.x = P0x;
    p_markers.markers[0].pose.position.y = P0y;
    p_markers.markers[0].pose.position.z = 1.0;
    p_markers.markers[0].scale.x = 0.25;
    p_markers.markers[0].scale.y = 0.25;
    p_markers.markers[0].scale.z = 0.25;
    p_markers.markers[0].color.r = 0.2;
    p_markers.markers[0].color.g = 1.0;
    p_markers.markers[0].color.b = 0.2;
    p_markers.markers[0].color.a = 1.0;
    p_markers.markers[0].lifetime = ros::Duration(0.0);

    // P (magenta sphere)
    p_markers.markers[1] = p_markers.markers[0];
    p_markers.markers[1].id = 1;
    p_markers.markers[1].pose.position.x = px;
    p_markers.markers[1].pose.position.y = py;
    p_markers.markers[1].color.r = 1.0;
    p_markers.markers[1].color.g = 0.2;
    p_markers.markers[1].color.b = 1.0;

    // Line connecting P0 -> P
    p_markers.markers[2] = p_markers.markers[0];
    p_markers.markers[2].ns = "p_augment_line";
    p_markers.markers[2].id = 2;
    p_markers.markers[2].type = visualization_msgs::Marker::LINE_STRIP;
    p_markers.markers[2].scale.x = 0.05;
    p_markers.markers[2].color.r = 1.0;
    p_markers.markers[2].color.g = 1.0;
    p_markers.markers[2].color.b = 0.0;
    p_markers.markers[2].color.a = 0.8;
    geometry_msgs::Point a0, a1;
    a0.x = P0x; a0.y = P0y; a0.z = 1.0;
    a1.x = px; a1.y = py; a1.z = 1.0;
    p_markers.markers[2].points = {a0, a1};

    p_augment_debug_marker_publisher_.publish(p_markers);

    // Local goal visualization (pose + point)
    local_goal_.header.stamp = ros::Time::now();
    local_goal_.pose.position.x = px;
    local_goal_.pose.position.y = py;

    const double yaw_goal = std::atan2(ty - ry, tx - rx);
    const double half = 0.5 * yaw_goal;
    local_goal_.pose.orientation.w = std::cos(half);
    local_goal_.pose.orientation.x = 0.0;
    local_goal_.pose.orientation.y = 0.0;
    local_goal_.pose.orientation.z = std::sin(half);
    local_goal_pose_publisher_.publish(local_goal_);

    geometry_msgs::PointStamped local_goal_point;
    local_goal_point.header = local_goal_.header;
    local_goal_point.point = local_goal_.pose.position;
    local_goal_point_publisher_.publish(local_goal_point);

    // =========================================================
    // 3) Control law (Eq.3 form)
    // =========================================================
    bpmp_tracker::UnicycleInput u;

    const double ex = px - rx;
    const double ey = py - ry;
    double r = std::hypot(ex, ey);

    // near-goal hold to avoid atan2 jitter
    const double r_enter = 0.03;
    const double r_exit  = 0.08;
    if (!hold_near_goal_ && r < r_enter) hold_near_goal_ = true;
    if ( hold_near_goal_ && r > r_exit ) hold_near_goal_ = false;

    if (hold_near_goal_) {
        u.vel_linear = 0.0;
        u.vel_angular = 0.0;
    } else {
        if (r < eps) r = eps;
        const double yaw_to_P = std::atan2(ey, ex);
        const double phi_P = wrap_pi(th - yaw_to_P);
        const double yaw_to_T = std::atan2(ty - ry, tx - rx);
        const double phi_T = wrap_pi(th - yaw_to_T);
        

        u.vel_linear  = param_.gain_k1 * r * std::cos(phi_P);
        u.vel_angular = -param_.gain_k1 * std::sin(phi_T) * std::cos(phi_T)
                        -param_.gain_k2 * phi_T;

        const double dp = std::hypot(px - P0x, py - P0y);
        ROS_INFO_THROTTLE(0.5,
            "[CTRL] useP=(%.2f,%.2f) baseP0=(%.2f,%.2f) dp=%.3f | r=%.3f phi_P=%.1fdeg v=%.2f w=%.2f avoid=%d",
            px, py, P0x, P0y, dp,
            r, phi_P * 180.0 / M_PI, u.vel_linear, u.vel_angular, (int)avoid_active);

        // Force log when augmentation is active (no throttle)
        if (avoid_active) {
            ROS_INFO("[CTRL-AUG] P0=(%.2f,%.2f) P=(%.2f,%.2f) dp=%.3f r=%.3f phi=%.1fdeg v=%.2f w=%.2f",
                P0x, P0y, px, py, dp,
                r, phi_P * 180.0 / M_PI, u.vel_linear, u.vel_angular);
        }
    }

    // =========================================================
    // 4) Clamp
    // =========================================================
    u.vel_linear  = clamp(u.vel_linear,  -param_.v_max, param_.v_max);
    u.vel_angular = clamp(u.vel_angular, -param_.w_max, param_.w_max);

    // =========================================================
    // 5) Publish
    // =========================================================
    tracker_control_input_publisher_.publish(u);

    const double dPT = std::hypot(px - tx, py - ty);
    ROS_INFO_THROTTLE(0.5,
        "[A] dPT=%.3f (want %.3f) dObs=%.2f hasObs=%d hold=%d",
        dPT, param_.tracking_radius, d_obs_min, (int)has_obstacle, (int)hold_near_goal_);
}

bool bpmp::TrackingController::is_info_received() {
    return is_target_info_received and is_tracker_info_received;
}

bpmp::TrackingController::TrackingController() : nh_("~") {
    nh_.param<double>("gain_k1", param_.gain_k1, 1.0);
    nh_.param<double>("gain_k2", param_.gain_k2, 1.0);
    nh_.param<double>("gain_o",  param_.gain_o,  1.0);
    nh_.param<double>("tracking_radius", param_.tracking_radius, 1.0);
    nh_.param<double>("obstacle_fov_deg", param_.obstacle_fov_deg, 120.0);
    nh_.param<double>("sonar_detection_range", param_.sonar_detection_range, 3.0);
    nh_.param<double>("v_max", param_.v_max, 1.0);
    nh_.param<double>("w_max", param_.w_max, 1.0);

    // SUBSCRIBER
    tracker_state_subscriber_ = nh_.subscribe("/base_odom", 1,
        &TrackingController::TrackerStateCallback, this);
    target_state_subscriber_ = nh_.subscribe("/bpmp_simulator/target_state", 1,
        &TrackingController::TargetStateCallback, this);
    obstacle_state_list_subscriber_ = nh_.subscribe("/bpmp_simulator/obstacle_state_list", 1,
        &TrackingController::ObstacleStateListCallback, this);

    tracker_control_input_publisher_ = nh_.advertise<bpmp_tracker::UnicycleInput>(
        "/bpmp_tracker/unicycle_control_input", 1);
    local_goal_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/local_goal", 1);
    local_goal_point_publisher_ = nh_.advertise<geometry_msgs::PointStamped>("/local_goal_point", 1);

    tracking_circle_publisher_ = nh_.advertise<visualization_msgs::Marker>("/tracking_circle", 1);

    p_augment_debug_marker_publisher_ =
        nh_.advertise<visualization_msgs::MarkerArray>("/p_augment_debug", 1, true);

    local_goal_.header.frame_id = "map";
    local_goal_.pose.position.z = 1.0;

    tracking_circle_marker_.header.frame_id = "map";
    tracking_circle_marker_.ns = "tracking_circle";
    tracking_circle_marker_.id = 0;
    tracking_circle_marker_.type = visualization_msgs::Marker::LINE_STRIP;
    tracking_circle_marker_.action = visualization_msgs::Marker::ADD;
    tracking_circle_marker_.pose.orientation.w = 1.0;
    tracking_circle_marker_.scale.x = 0.05;
    tracking_circle_marker_.color.a = 1.0;
    tracking_circle_marker_.color.r = 0.1;
    tracking_circle_marker_.color.g = 0.6;
    tracking_circle_marker_.color.b = 1.0;

    // Active obstacle marker publisher
    active_obstacle_marker_publisher_ =
        nh_.advertise<visualization_msgs::Marker>("/active_obstacle", 1);

    // [ADD] Active obstacle marker init
    active_obstacle_marker_.header.frame_id = "map";
    active_obstacle_marker_.ns = "active_obstacle";
    active_obstacle_marker_.id = 0;
    active_obstacle_marker_.type = visualization_msgs::Marker::SPHERE;
    active_obstacle_marker_.action = visualization_msgs::Marker::ADD;
    active_obstacle_marker_.pose.orientation.w = 1.0;

    active_obstacle_marker_.scale.x = 0.5;
    active_obstacle_marker_.scale.y = 0.5;
    active_obstacle_marker_.scale.z = 0.5;

    // 끊김 방지용: publish가 끊기면 자동 소거
    active_obstacle_marker_.lifetime = ros::Duration(0.2);

    // 기본색(초록)
    active_obstacle_marker_.color.r = 0.2;
    active_obstacle_marker_.color.g = 1.0;
    active_obstacle_marker_.color.b = 0.2;
    active_obstacle_marker_.color.a = 0.9;
    }

void bpmp::TrackingController::ObstacleStateListCallback(const bpmp_tracker::ObjectStateList &msg) {
    is_obstacle_info_received = true;
    current_obstacle_state_list_.clear();

    bpmp::State s;
    for (int i = 0; i < (int)msg.object_state_list.size(); ++i) {
        s.px = msg.object_state_list[i].px;
        s.py = msg.object_state_list[i].py;
        s.pz = msg.object_state_list[i].pz;
        s.vx = msg.object_state_list[i].vx;
        s.vy = msg.object_state_list[i].vy;
        s.vz = msg.object_state_list[i].vz;
        current_obstacle_state_list_.push_back(s);
    }
}

void bpmp::TrackingController::TrackerStateCallback(const nav_msgs::Odometry_<std::allocator<void>>::ConstPtr &msg) {
    is_tracker_info_received = true;
    current_tracker_state_.px = msg->pose.pose.position.x;
    current_tracker_state_.py = msg->pose.pose.position.y;
    current_tracker_state_.pz = 1.0;
    double q[4] {msg->pose.pose.orientation.w,msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z};
    current_tracker_state_.theta = atan2(
        2.0 * (q[0] * q[3] + q[1] * q[2]),
        1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3])
    );
}

void bpmp::TrackingController::TargetStateCallback(const bpmp_tracker::ObjectState &msg) {
    is_target_info_received = true;
    current_target_state_.px = msg.px;
    current_target_state_.py = msg.py;
    current_target_state_.pz = msg.pz;
}
