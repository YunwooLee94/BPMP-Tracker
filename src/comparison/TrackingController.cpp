//
// Created by larr-laptop on 12/25/25.
//
#include <comparison/TrackingController.h>

void bpmp::TrackingController::Run() {
    ros::Rate loop_rate(50.0);
    while(ros::ok()){
        if(is_info_received()) // Target, Tracker 정보 있다면 제어 계산 해라.
            MakeControl();
        ros::spinOnce();
        loop_rate.sleep();
    }

}

void bpmp::TrackingController::MakeControl() { // TODO: Implement This Code
    // 1. 장애물 중 가장 가까운 장애물 찾기 (변수:is_obstacle_info_received 가 있으니, 장애물 있을 때, 없을 때는 넘어가도록 코드 짜주세요.)

    // 2. Local Goal (position, heading angle 찾기), Local Goal Visualization 해주세요.

    // LOCAL GOAL VISUALIZATION
    //local_goal_.pose.position.x=*;
    //local_goal_.pose.position.y=*;
    //local_goal_pose.orientation.w = *;
    //local_goal_pose.orientation.x = *;
    //local_goal_pose.orientation.y = *;
    //local_goal_pose.orientation.z = *;
    local_goal_pose_publisher_.publish(local_goal_);

    // 3. 제어식 구현
    bpmp_tracker::UnicycleInput control_input;
    // control_input.linear_speed =*;
    // control_input.angular_speed =*;

    // 4. v, w input range 제한 시키기 [-vmax, vmax], [-wmax, wmax]
    control_input.vel_linear = min(param_.v_max,max(-param_.v_max,control_input.vel_linear));
    control_input.vel_angular = min(param_.w_max,max(-param_.w_max,control_input.vel_angular));

    // 5. 최종 v, w publish 하기
    tracker_control_input_publisher_.publish(control_input);
}

bool bpmp::TrackingController::is_info_received() {
    return is_target_info_received and is_tracker_info_received;
}

bpmp::TrackingController::TrackingController():nh_("~") {
    // PARAMETER SETTINGS
    nh_.param<double>("gain_k1", param_.gain_k1,1.0);
    nh_.param<double>("gain_k2",param_.gain_k2,1.0);
    nh_.param<double>("gain_o",param_.gain_o,1.0);
    nh_.param<double>("tracking_radius",param_.tracking_radius,1.0);
    nh_.param<double>("v_max",param_.v_max,1.0);
    nh_.param<double>("w_max",param_.w_max,1.0);

    // SUBSCRIBER
    tracker_state_subscriber_ = nh_.subscribe("/base_odom", 1,
                                              &TrackingController::TrackerStateCallback, this);
    target_state_subscriber_ = nh_.subscribe("/bpmp_simulator/target_state", 1,
                                             &TrackingController::TargetStateCallback, this);
    obstacle_state_list_subscriber_ = nh_.subscribe("/bpmp_simulator/obstacle_state_list", 1,
                                                    &TrackingController::ObstacleStateListCallback, this);
    // PUBLISHER
    tracker_control_input_publisher_ = nh_.advertise<bpmp_tracker::UnicycleInput>("/bpmp_tracker/unicycle_control_input", 1);
    local_goal_pose_publisher_ = nh_.advertise<geometry_msgs::PoseStamped>("/local_goal",1);

    local_goal_.header.frame_id="map";
    local_goal_.pose.position.z = 1.0;
}

void bpmp::TrackingController::ObstacleStateListCallback(const bpmp_tracker::ObjectStateList &msg) {
    is_obstacle_info_received = true;
    bpmp::State obstacle_state;
    for (int i = 0; i < msg.object_state_list.size(); i++) {
        obstacle_state.px = msg.object_state_list[i].px;
        obstacle_state.py = msg.object_state_list[i].py;
        obstacle_state.pz = msg.object_state_list[i].pz;
        current_obstacle_state_list_.push_back(obstacle_state);
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
