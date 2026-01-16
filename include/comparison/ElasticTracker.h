//
// Created by larr-laptop on 1/15/26.
//

#ifndef BPMP_TRACKER_ELASTICTRACKER_H
#define BPMP_TRACKER_ELASTICTRACKER_H
#include <ros/ros.h>
#include <bpmp_utils/Utils.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <bpmp_tracker/UnicycleInput.h>
#include <nav_msgs/Odometry.h>
#include <bpmp_tracker/ObjectState.h>
#include <bpmp_tracker/ObjectStateList.h>

namespace bpmp{
    class ElasticTracker{
    private:
        ros::NodeHandle nh_;
        // Subscriber: Tracker, Obstacle, Target
        ros::Subscriber tracker_sub_;
        ros::Subscriber dynamic_obstacle_sub_;
        ros::Subscriber target_sub_;

        void ObstacleStateListCallback(const sensor_msgs::PointCloud2 &msg);
        void TrackerStateCallback(const nav_msgs::Odometry &msg);
        void TargetStateCallback(const bpmp_tracker::ObjectState &msg);

        bpmp::UnicycleState current_tracker_state_;
        pcl::PointCloud<pcl::PointXYZ> cloud_;
        bpmp::State current_target_state_;

        bool IsInfoReady();
        bool is_target_info_received_{false};
        bool is_tracker_info_received_{false};
        bool is_pcl_info_received_{false};

        void Planning();
    public:
        ElasticTracker();
        void Run();
    };
}


#endif //BPMP_TRACKER_ELASTICTRACKER_H
