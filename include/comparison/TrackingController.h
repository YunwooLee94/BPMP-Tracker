#ifndef BPMP_TRACKER_COMPARISON_CONTROLLER_H
#define BPMP_TRACKER_COMPARISON_CONTROLLER_H
#include <ros/ros.h>
#include <bpmp_tracker/UnicycleInput.h>
#include <nav_msgs/Odometry.h>
#include <bpmp_utils/Utils.h>
#include <bpmp_tracker/ObjectState.h>
#include <bpmp_tracker/ObjectStateList.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <vector>

namespace bpmp{
    struct ComparisonControllerParam{
        double gain_k1;
        double gain_k2;
        double gain_o;
        double tracking_radius;
        double obstacle_fov_deg;  // 센서 FOV
        double sonar_detection_range; // 감지 임계값
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
        std::vector<bpmp::State> current_obstacle_state_list_;
        bpmp::UnicycleState current_tracker_state_;
        geometry_msgs::PoseStamped local_goal_;

        bool is_tracker_info_received{false};
        bool is_target_info_received{false};
        bool is_obstacle_info_received{false};
        bool is_info_received();

        // ===== r->0 근처 각도 jump 방지용 =====
        bool hold_near_goal_{false};

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
        ros::Publisher local_goal_point_publisher_;
        ros::Publisher tracking_circle_publisher_;
        visualization_msgs::Marker tracking_circle_marker_;

        ros::Publisher active_obstacle_marker_publisher_;
        visualization_msgs::Marker active_obstacle_marker_;

        // Debug visualization: P0 -> P modification (points/line)
        ros::Publisher p_augment_debug_marker_publisher_;


        void MakeControl();

    public:
        TrackingController();
        void Run();
    };
}

#endif //BPMP_TRACKER_COMPARISON_CONTROLLER_H
