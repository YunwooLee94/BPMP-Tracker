//
// Created by larr-laptop on 12/28/25.
//

#ifndef BPMP_TRACKER_BASELINE_H
#define BPMP_TRACKER_BASELINE_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <comparison/Optimizer.h>

namespace bpmp{
    class Baseline{
    private:
        ros::NodeHandle nh_;
        // CONTROLLER VARIABLE
        bpmp::State current_target_state_;
        vector<bpmp::State> current_obstacle_state_list_;
        bpmp::UnicycleState current_tracker_state_;

        OptimizationParam param_;

        bool is_tracker_info_received{false};
        bool is_target_info_received{false};
        bool is_obstacle_info_received{false};

        //ROS SUBSCRIBER
        ros::Subscriber tracker_state_subscriber_;
        ros::Subscriber target_state_subscriber_;
        ros::Subscriber obstacle_state_list_subscriber_;
        void ObstacleStateListCallback(const bpmp_tracker::ObjectStateList &msg);
        void TrackerStateCallback(const nav_msgs::Odometry::ConstPtr &msg);
        void TargetStateCallback(const bpmp_tracker::ObjectState &msg);

        //ROS PUBLISHER
        ros::Publisher tracker_control_input_publisher_;

        bool is_info_received();
        void MakeControl();
    public:
        Baseline();
        void Run();
    };
}

#endif //BPMP_TRACKER_BASELINE_H
