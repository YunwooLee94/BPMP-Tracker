//
// Created by larr-laptop on 1/14/26.
//

#ifndef BPMP_TRACKER_ELASTICTYPECONVERTER_H
#define BPMP_TRACKER_ELASTICTYPECONVERTER_H
#include <ros/ros.h>
#include <bpmp_tracker/ObjectState.h>
#include <bpmp_tracker/ObjectStateList.h>
#include <nav_msgs/Odometry.h>
#include <bpmp_utils/Utils.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <vector>
namespace bpmp{
    typedef message_filters::sync_policies::ApproximateTime<bpmp_tracker::ObjectStateList, nav_msgs::Odometry> ObstacleTrackerSync;
    class ElasticTypeConverter{
    private:
        ros::NodeHandle nh_;
        void SyncCallback(const bpmp_tracker::ObjectStateListConstPtr&, const nav_msgs::OdometryConstPtr &);
        message_filters::Synchronizer<bpmp::ObstacleTrackerSync> *sub_sync_;
        message_filters::Subscriber<bpmp_tracker::ObjectStateList> *sub_dynamic_obstacle_;
        message_filters::Subscriber<nav_msgs::Odometry> *sub_tracker_odometry_;
        ros::Publisher pc_pub_;
        double sensing_range_;
        double object_radius_;

        const int num_theta_ = 10;
        const int num_z_ = 20;
        std::vector<double> theta_array_ ;
        std::vector<double> z_array_;
    public:
        ElasticTypeConverter();
        ~ElasticTypeConverter();
        void Run();
    };
}


#endif //BPMP_TRACKER_ELASTICTYPECONVERTER_H
