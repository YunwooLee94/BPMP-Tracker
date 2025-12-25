#ifndef BPMP_TRACKER_COMPARISON_CONTROLLER_H
#define BPMP_TRACKER_COMPARISON_CONTROLLER_H
#include <ros/ros.h>
#include <bpmp_tracker/UnicycleInput.h>
#include <nav_msgs/Odometry.h>
#include <bpmp_utils/Utils.h>
#include <bpmp_tracker/UnicycleInput.h>
#include <bpmp_tracker/ObjectState.h>
#include <bpmp_tracker/ObjectStateList.h>
#include <geometry_msgs/PoseStamped.h>

namespace bpmp{
    struct ComparisonControllerParam{
        double gain_k1;
        double gain_k2;
        double gain_o;
        double tracking_radius;
        double v_max;
        double w_max;
    };

    class TrackingController{
    private:
        ros::NodeHandle nh_;
        // parameter
        ComparisonControllerParam param_;

        // CONTROLLER VARIABLE
        bpmp::State current_target_state_;
        vector<bpmp::State> current_obstacle_state_list_;
        bpmp::UnicycleState current_tracker_state_;
        geometry_msgs::PoseStamped local_goal_;

        bool is_tracker_info_received{false};
        bool is_target_info_received{false};
        bool is_obstacle_info_received{false};
        bool is_info_received();

        //ROS SUBSCRIBER
        ros::Subscriber tracker_state_subscriber_;
        ros::Subscriber target_state_subscriber_;
        ros::Subscriber obstacle_state_list_subscriber_;
        void ObstacleStateListCallback(const bpmp_tracker::ObjectStateList &msg);
        void TrackerStateCallback(const nav_msgs::Odometry::ConstPtr &msg);
        void TargetStateCallback(const bpmp_tracker::ObjectState &msg);

        //ROS PUBLISHER
        ros::Publisher tracker_control_input_publisher_;
        ros::Publisher local_goal_pose_publisher_;


        void MakeControl();

    public:
        TrackingController();
        void Run();
    };
}

#endif //BPMP_TRACKER_COMPARISON_CONTROLLER_H
