//
// Created by larr-laptop on 1/14/26.
//
#include <comparison/ElasticTypeConverter.h>

bpmp::ElasticTypeConverter::ElasticTypeConverter():nh_("~") {
    sub_dynamic_obstacle_ = new message_filters::Subscriber<bpmp_tracker::ObjectStateList>(nh_,"/bpmp_simulator/obstacle_state_list",1);
    sub_tracker_odometry_ = new message_filters::Subscriber<nav_msgs::Odometry>(nh_,"/base_odom",1);
    sub_sync_ = new message_filters::Synchronizer<bpmp::ObstacleTrackerSync>(bpmp::ObstacleTrackerSync(10), *this->sub_dynamic_obstacle_, *this->sub_tracker_odometry_);
    sub_sync_->registerCallback(boost::bind(&ElasticTypeConverter::SyncCallback,this,_1,_2));
    nh_.param<double>("sensing_range",sensing_range_,1.0);
}

void bpmp::ElasticTypeConverter::Run() {
    ros::Rate loop_rate(30.0);
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void bpmp::ElasticTypeConverter::SyncCallback(const bpmp_tracker::ObjectStateListConstPtr & dyn_ptr,
                                              const nav_msgs::OdometryConstPtr &tracker_ptr) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
}

bpmp::ElasticTypeConverter::~ElasticTypeConverter() {
    delete sub_sync_;
    delete sub_dynamic_obstacle_;
    delete sub_tracker_odometry_;
}
