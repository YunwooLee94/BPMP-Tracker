//
// Created by larr-laptop on 25. 4. 8.
//

#ifndef BPMP_TRACKER_SIMULATOR_H
#define BPMP_TRACKER_SIMULATOR_H
#include <ros/ros.h>
#include <string>
#include <fstream>
#include <istream>
#include <visualization_msgs/MarkerArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_broadcaster.h>
#include <bpmp_tracker/ControlInput.h>
#include <bpmp_tracker/ControlInputList.h>
#include <bpmp_tracker/ObjectState.h>
#include <bpmp_tracker/ObjectStateList.h>
#include <bpmp_tracker/UnicycleInput.h>
#include <bpmp_tracker/UnicycleState.h>
#include <bpmp_utils/Utils.h>
#include <algorithm>
#include <vector>
#include <ctime>
#include <bpmp_utils/eigenmvn.h>
#include "../bpmp_utils/convhull_3d.h"
#include <bpmp_utils/geo_utils.hpp>
#include <bpmp_utils/quickhull.hpp>
#include <bpmp_utils/sdlp.hpp>
typedef Eigen::Vector3d Vec3;
typedef std::vector<Eigen::Vector3d> Vec3List;
typedef std_msgs::ColorRGBA ColorMsg;
typedef std::vector<ColorMsg> ColorMsgs;
typedef geometry_msgs::Point PointMsg;



namespace bpmp{
    using namespace std;
    class Simulator{
    public:
        Simulator();
        void Run();
    private:
        ros::NodeHandle nh_;
        bool is_unstructured_{false};
        double inflation_size_{0.5};
        double point_resolution_{0.01};
        double simulation_dt_;
        string map_frame_id_;
        bool analysis_mode_{false};
        double spatial_scale_{1.0};
        double time_scale_{1.0};
        double agent_size_;
        vector<int>object_idx_list_;
        int total_object_number_in_file_;
        int object_number_;
        int target_idx_;
        int moving_obstacle_number_;
        int total_test_number_;
        vector<int> obstacle_idx_list_;
        nav_msgs::Path target_path_;
        nav_msgs::Path robot_path_;
        visualization_msgs::MarkerArray  obstacle_path_;
        visualization_msgs::Marker single_obstacle_path_;
        visualization_msgs::Marker fov_vis_;

        inline PointMsg GetDefaultPointMsg(){
            PointMsg p;
            p.x = 0.0, p.y = 0.0, p.z = 0.0;
            return p;
        }
        inline geometry_msgs::Quaternion GetDefaultQuaternionMsg(){
            geometry_msgs::Quaternion q;
            q.w = 1.0, q.x = 0.0, q.y = 0.0, q.z =0.0;
            return q;
        }
        inline geometry_msgs::Vector3  GetDefaultScaleMsg(){
            geometry_msgs::Vector3 v;
            v.x = 1.0, v.y = 1.0, v.z = 1.0;
            return v;
        }


        pcl::PointCloud<pcl::PointXYZ> point_cloud_;
        vector<StateHistory> object_history_list_;
        State current_tracker_state_;
        UnicycleState current_unicycle_state_;
        ControlInput tracker_control_input;
        UnicycleControlInput unicycle_control_input_;
        State current_target_state_;
        vector<State> current_obstacle_state_list_;
        string initial_state_file_name_;
        string object_history_file_name_;
        string object_history_file_path_name_;
        string obstacle_configuration_file_name_;

        void ReadInitialTrackerStateList();
        void ReadObjectTrajectory();
        void ReadObstacleConfiguration();
        void UpdateDynamics(const double &t);
        void PrepareRosMsgs(const double &t);
        void PublishRosMsgs();
        void ShuffleScenario();
        std::vector<int> GenerateUniqueRandomArray(int size, int lower_bound, int upper_bound);
        std::vector<bpmp::AffineCoeff3D> GetHalfSpaceFromBoundary();
        visualization_msgs::Marker VisualizeConvexHull(const Vec3List &convex_hull);
        std::vector<bpmp::AffineCoeff2D> GetFOVConstraints();

        visualization_msgs::MarkerArray obstacle_list_vis_;
        visualization_msgs::Marker obstacle_vis_;
        visualization_msgs::Marker target_vis_;
        nav_msgs::Odometry tracker_vis_;
        visualization_msgs::Marker gar_robot_;
        visualization_msgs::MarkerArray pcl_boxes_vis_;

        ros::Publisher target_vis_publisher_;
        ros::Publisher obstacle_list_vis_publisher_;
        ros::Publisher tracker_vis_publisher_;
        ros::Publisher pcl_publisher_;
        ros::Publisher pcl_boxes_vis_publisher_;
        ros::Publisher target_path_publisher_;
        ros::Publisher robot_path_publisher_;
        ros::Publisher gar_robot_publisher_;
        ros::Publisher fov_publisher_;
        ros::Publisher obstacle_path_list_publisher_;
        Vec3List GetFeasibleVertices(const vector<AffineCoeff2D> &constraint);

        ros::Subscriber control_input_subscriber_;
        ros::Subscriber unicycle_control_input_subscriber_;
        void control_input_callback(const bpmp_tracker::ControlInput &msg);
        void unicycle_input_callback(const bpmp_tracker::UnicycleInput &msg);
        ros::Publisher target_state_publisher_;
        ros::Publisher obstacle_state_list_publisher_;
        ros::Publisher tracker_state_publisher_;
        bpmp_tracker::ObjectState target_state_msg_;
        bpmp_tracker::ObjectStateList obstacle_state_list_msg_;
        bpmp_tracker::UnicycleState tracker_state_msg_;

        tf::TransformBroadcaster br_;

    };
}

#endif //BPMP_TRACKER_SIMULATOR_H
