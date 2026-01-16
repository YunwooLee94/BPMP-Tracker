//
// Created by larr-laptop on 1/15/26.
//
#include <comparison/ElasticTracker.h>

bpmp::ElasticTracker::ElasticTracker():nh_("~") {
    tracker_sub_ = nh_.subscribe("/base_odom", 1,
                                              &ElasticTracker::TrackerStateCallback, this);
    target_sub_ = nh_.subscribe("/bpmp_simulator/target_state", 1,
                                             &ElasticTracker::TargetStateCallback, this);
    dynamic_obstacle_sub_ = nh_.subscribe("/obstacle_pointcloud", 1,
                                                    &ElasticTracker::ObstacleStateListCallback, this);
    cloud_.header.frame_id="map";

}

void bpmp::ElasticTracker::Run() {
    ros::Rate loop_rate(20.0);
    while(ros::ok()){
        if(IsInfoReady())
            Planning();
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
    is_tracker_info_received_ = true;
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
//    std::cout<<"[ELASTIC TRACKER]: Got Tracker State"<<std::endl;
}

void bpmp::ElasticTracker::ObstacleStateListCallback(const sensor_msgs::PointCloud2 &msg) {
    is_pcl_info_received_ = true;
    cloud_.points.clear();
    pcl::fromROSMsg(msg, cloud_);
//    std::cout<<"[ELASTIC TRACKER]: Got Point-cloud"<<std::endl;
}

bool bpmp::ElasticTracker::IsInfoReady() {
    return is_target_info_received_ && is_tracker_info_received_;
}

void bpmp::ElasticTracker::Planning() {

}
