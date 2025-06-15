//
// Created by larr-laptop on 25. 6. 9.
//

#ifndef BPMP_TRACKER_ROSWRAPPER_H
#define BPMP_TRACKER_ROSWRAPPER_H

#include <ros/ros.h>
#include <thread>
#include <bpmp_tracker/Tracker.h>
#include <bpmp_tracker/PlanningVisualizer.h>
#include <bpmp_tracker/ObjectState.h>
#include <bpmp_tracker/ObjectStateList.h>
#include <bpmp_tracker/PolyState.h>
#include <bpmp_tracker/ControlInput.h>
#include <bpmp_tracker/ControlInputList.h>
#include <bpmp_tracker/UnicycleInput.h>
#include <nav_msgs/Odometry.h>

namespace bpmp{
    struct RosParam{
        double control_frequency;
        double planning_frequency;
    };
    class RosWrapper{
    public:
        RosWrapper(shared_ptr<PlannerBase> p_base);
        void RunRos();
        double GetCurrentTime(){return ros::Time::now().toSec()-t0_;};
        bpmp::TrackingParam GetTrackingParam();
        bpmp::VisualizationParam GetVisualizationParam();
        double GetPlanningFrequency(){return ros_param_.planning_frequency;};
        double GetControlFrequency(){return ros_param_.control_frequency;};
        void InitSubscriberAndPublisher();
    private:
        ros::NodeHandle nh_;
        shared_ptr<PlannerBase> p_base_;
        bpmp::VisualizationParam vis_param_;
        bpmp::RosParam ros_param_;
        bpmp::TrackingParam planning_param_;
        bpmp::PlanningVisualizer * visualizer_;
        double t0_;
        void PrepareRosMsgs();
        void PublishRosMsgs();

        ros::Subscriber tracker_list_state_subscriber_;
        ros::Subscriber target_prediction_subscriber_;
        ros::Subscriber obstacle_state_list_subscriber_;

        ros::Publisher corridor_publisher_;
        ros::Subscriber pcl_subscriber_;
        void PclCallback(const sensor_msgs::PointCloud2::ConstPtr &pcl_msgs);

        ros::Publisher tracker_raw_primitives_publisher_;
        ros::Publisher tracker_feasible_primitives_publisher_;
        ros::Publisher tracker_best_trajectory_publisher_;

        ros::Publisher buffered_voronoi_cell_publisher_;
        ros::Publisher visibility_cell_publisher_;

        ros::Publisher tracker_control_input_publisher_;


        void ObstacleStateListCallback(const bpmp_tracker::ObjectStateList &msg);
        void TrackerStateCallback(const nav_msgs::Odometry::ConstPtr &msg);
        void TargetPredictionCallback(const bpmp_tracker::PolyState &msg);

        bpmp_tracker::UnicycleInput GenerateControlInput(const vector<bpmp::PrimitivePlanning> & primitive, const bpmp::uint &best_index, const double &time_t);
    };
}
#endif //BPMP_TRACKER_ROSWRAPPER_H
