//
// Created by larr-laptop on 12/28/25.
//
#include "comparison/Baseline.h"

bpmp::Baseline::Baseline(): nh_("~") {

    nh_.param<double>("time_step", param_.time_step,0.1);
    nh_.param<double>("param_1", param_.param_1,0.1);
    nh_.param<double>("param_2", param_.param_2,0.1);
    nh_.param<double>("param_3", param_.param_3,0.1);
    nh_.param<double>("param_4", param_.param_3,0.1);

    // SUBSCRIBER
    tracker_state_subscriber_ = nh_.subscribe("/base_odom", 1,
                                              &Baseline::TrackerStateCallback, this);
    target_state_subscriber_ = nh_.subscribe("/bpmp_simulator/target_state", 1,
                                             &Baseline::TargetStateCallback, this);
    obstacle_state_list_subscriber_ = nh_.subscribe("/bpmp_simulator/obstacle_state_list", 1,
                                                    &Baseline::ObstacleStateListCallback, this);
    // PUBLISHER
    tracker_control_input_publisher_ = nh_.advertise<bpmp_tracker::UnicycleInput>("/bpmp_tracker/unicycle_control_input", 1);
}

void bpmp::Baseline::Run() {
    ros::Rate loop_rate(50.0);
    while(ros::ok()){
        if(is_info_received()) // Target, Tracker 정보 있다면 제어 계산 해라.
            MakeControl();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

bool bpmp::Baseline::is_info_received() {
    return is_target_info_received and is_tracker_info_received;
}

void bpmp::Baseline::MakeControl() {
    std::shared_ptr<bpmp::Problem> prob = std::make_shared<Problem>(current_target_state_,
                                                                    current_obstacle_state_list_,current_tracker_state_);
    std::array<Matrix<double,Nu,1>,N> u0;
    for(int i = 0; i < N; i++)
    {
        u0[i] = (Matrix<double,Nu,1>()<< 0,0.0).finished();
    }
    Matrix<double,Nx,1> x0_new;
    Collection<Matrix<double,Nu,1>,N> uN_new;
    Collection<Matrix<double,Nx,1>,N+1> xN_new;
    // if car state contains strange value;
    for (int i = 0; i < N; i++) {
        xN_new[i].setZero();
        uN_new[i].setZero();
    }
    xN_new[N].setZero();
    x0_new = (Matrix<double,Nx,1>()<<current_tracker_state_.px, current_tracker_state_.py,
            current_tracker_state_.theta).finished();
    uN_new = u0;
    Optimizer<Nx,Nu,N> optimizer(*prob,x0_new,uN_new,0.1,param_);
    optimizer.Solve();
}

void bpmp::Baseline::ObstacleStateListCallback(const bpmp_tracker::ObjectStateList &msg) {
    is_obstacle_info_received = true;
    bpmp::State obstacle_state;
    for (int i = 0; i < msg.object_state_list.size(); i++) {
        obstacle_state.px = msg.object_state_list[i].px;
        obstacle_state.py = msg.object_state_list[i].py;
        obstacle_state.pz = msg.object_state_list[i].pz;
        current_obstacle_state_list_.push_back(obstacle_state);
    }
}

void bpmp::Baseline::TrackerStateCallback(const nav_msgs::Odometry_<std::allocator<void>>::ConstPtr &msg) {
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

void bpmp::Baseline::TargetStateCallback(const bpmp_tracker::ObjectState &msg) {
    is_target_info_received = true;
    current_target_state_.px = msg.px;
    current_target_state_.py = msg.py;
    current_target_state_.pz = msg.pz;
}



