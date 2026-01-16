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
    nh_.param<double>("object_radius",object_radius_,0.25);
    pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/obstacle_pointcloud",1);
    for(int i =0;i<num_theta_;i++){
        theta_array_.push_back(M_PI*2.0/double(num_theta_)*double(i));
    }
    for(int i=0;i<num_z_;i++){
        z_array_.push_back(double(2*i)/double(num_z_));
    }

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
    if(dyn_ptr->object_state_list.empty())
        return;
    int num_obstacle = dyn_ptr->object_state_list.size();
    double distance;
    pcl::PointXYZ point;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    for(int i =0;i<num_obstacle;i++){
        distance = std::hypot(dyn_ptr->object_state_list[i].px-tracker_ptr->pose.pose.position.x, dyn_ptr->object_state_list[i].py-tracker_ptr->pose.pose.position.y);
        if(distance<sensing_range_){
            for(int j =0;j<num_theta_;j++){
                for(int k=0;k<num_z_;k++){
                    point.x = dyn_ptr->object_state_list[i].px + object_radius_*std::cos(theta_array_[j]);
                    point.y = dyn_ptr->object_state_list[i].py + object_radius_*std::sin(theta_array_[j]);
                    point.z = z_array_[k];
                    cloud.points.push_back(point);
                }
            }
        }
    }
    if(cloud.empty())
        return;
//    std::cout<<"There are pcl."<<std::endl;
    sensor_msgs::PointCloud2 msg;
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    cloud.header.frame_id = "map";
    cloud.header.stamp = pcl_conversions::toPCL(ros::Time::now());
    pcl::toROSMsg(cloud, msg);
    pc_pub_.publish(msg);
}

bpmp::ElasticTypeConverter::~ElasticTypeConverter() {
    delete sub_sync_;
    delete sub_dynamic_obstacle_;
    delete sub_tracker_odometry_;
}
