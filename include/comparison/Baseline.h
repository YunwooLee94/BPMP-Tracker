//
// Created by larr-laptop on 12/28/25.
//

#ifndef BPMP_TRACKER_BASELINE_H
#define BPMP_TRACKER_BASELINE_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <comparison/Optimizer.h>
#include <bpmp_tracker/UnicycleInput.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

namespace bpmp{
    class Baseline{
    private:
        ros::NodeHandle nh_;
        // CONTROLLER VARIABLE
        bpmp::TargetStateBaseline current_target_state_;
        vector<bpmp::State> current_obstacle_state_list_;
        bpmp::UnicycleStateBaseline current_tracker_state_;
        bpmp::UnicycleStateBaseline prev_tracker_state_;
        double prev_tracker_time_{0.0};

        OptimizationParam param_;

        bool is_tracker_info_received{false};
        bool is_target_info_received{false};
        bool is_obstacle_info_received{false};

        //ROS SUBSCRIBER
        nav_msgs::Odometry prev_robot_odom_;
        ros::Subscriber tracker_state_subscriber_;
        ros::Subscriber target_state_subscriber_;
        ros::Subscriber obstacle_state_list_subscriber_;
        void ObstacleStateListCallback(const bpmp_tracker::ObjectStateList &msg);
        void TrackerStateCallback(const nav_msgs::Odometry::ConstPtr &msg);
        void TargetStateCallback(const bpmp_tracker::ObjectState &msg);
        bool isInFOV(bpmp::State obstacle_state);

        //ROS PUBLISHER
        ros::Publisher tracker_control_input_publisher_;
        ros::Publisher active_obstacle_marker_array_publisher_;
        ros::Publisher fov_sector_marker_publisher_;
        ros::Publisher robot_planning_path_publisher_;
        ros::Publisher target_predicted_path_publisher_;
        ros::Publisher cur_robot_vel_path_publisher_;
        visualization_msgs::MarkerArray active_obstacle_marker_array_;



        bool is_info_received();
        void MakeControl();
    public:
        Baseline();
        void Run();
    };
}

#endif //BPMP_TRACKER_BASELINE_H
