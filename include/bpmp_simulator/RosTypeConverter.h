//
// Created by larr-laptop on 25. 4. 15.
//

#ifndef BPMP_TRACKER_ROSTYPECONVERTER_H
#define BPMP_TRACKER_ROSTYPECONVERTER_H
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h> // Target
#include <nav_msgs/Odometry.h> // Robot
#include <bpmp_tracker/RobotState.h>
#include <bpmp_tracker/ObjectState.h>
#include <bpmp_tracker/UnicycleInput.h>
#include <vector>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
namespace bpmp{
    struct TargetVelocity{
        double vx;
        double vy;
    };
    class RosTypeConverter{
    public:
        RosTypeConverter();
        void Run();
    private:
        ros::NodeHandle nh_;
        ros::Subscriber TargetPositionSubscriber_;
        ros::Subscriber RobotOdometrySuscriber_;
        ros::Subscriber PclSubscriber_;
        ros::Publisher TargetStatePublisher_;
        ros::Publisher PclPublisher_;
        ros::Subscriber UnicycleInputSubscriber_;
        ros::Publisher UnicycleInputPublisher_;

        double t0_;
        double t0_history_;
        int odom_count_=0;
        int odom_count_iter_ =0;
        tf::TransformBroadcaster br_;
        double curTime() {return (ros::Time::now().toSec()-t0_);};
        bool is_target_position_received_{false};
        bool is_unicycle_state_received_{false};
        void TargtPositionCallback(const geometry_msgs::PoseStampedConstPtr &msg);
        void PclCallback(const pcl::PointCloud<pcl::PointXYZ> &msg);
        void UnicycleInputCallback(const bpmp_tracker::UnicycleInput &msg);
        sensor_msgs::PointCloud2 pcl_output_;
        void RobotOdometryCallback(const nav_msgs::OdometryConstPtr &msg);
        bpmp_tracker::ObjectState current_target_state_;
        bpmp_tracker::ObjectState previous_target_state_;
        std::vector<TargetVelocity> vel_history_;
        void Publish();
    };
};

#endif //BPMP_TRACKER_ROSTYPECONVERTER_H
