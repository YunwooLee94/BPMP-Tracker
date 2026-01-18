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
#include <comparison/Mapping.h>
#include <traj_opt/traj_opt.h>
#include <comparison/Prediction.hpp>
#include <quadrotor_msgs/PolyTraj.h>
#include <comparison/Visualization.hpp>


namespace bpmp{
    struct ElasticGridParm{
        double resolution;
        double x_length;
        double y_length;
        double z_length;
        double agent_size;
    };
    struct ElasticPlanningParam{

    };
    struct ElasticParam{
        ElasticGridParm param_g;
        ElasticPlanningParam param_p;
    };
    class ElasticTracker{
    private:
        ros::NodeHandle nh_;
        // Subscriber: Tracker, Obstacle, Target
        ros::Subscriber tracker_sub_;
        ros::Subscriber dynamic_obstacle_sub_;
        ros::Subscriber target_sub_;
        ros::Publisher occ_grid_pub_;
        ElasticParam param_;

        void ObstacleStateListCallback(const sensor_msgs::PointCloud2 &msg);
        void TrackerStateCallback(const nav_msgs::Odometry &msg);
        void TargetStateCallback(const bpmp_tracker::ObjectState &msg);

        bpmp::UnicycleState current_tracker_state_;
        pcl::PointCloud<pcl::PointXYZ> cloud_;
        bpmp::State current_target_state_;
        std::vector<double> prev_pose_{0.0,0.0};
        double prev_time_{0.0};
        double tracker_velocity_{0.0};

        bool IsInfoReady();
        bool is_target_info_received_{false};
        bool is_tracker_info_received_{false};
        bool is_pcl_info_received_{false};
        bool wait_hover_{true};


        std::shared_ptr<mapping::OccGridMap> gridmapPtr_;

        double tracking_dist_, tolerance_d_, tracking_dur_, tracking_dt_;
        int traj_id_{0};

        Trajectory traj_poly_;
        std::shared_ptr<traj_opt::TrajOpt> trajOptPtr_;
        std::shared_ptr<prediction::Predict> prePtr_;
        std::shared_ptr<visualization::Visualization> visPtr_;


        void Planning();
        void pub_hover_p(const Eigen::Vector3d& hover_p, const ros::Time& stamp);
        
        ros::Publisher traj_pub_;

    public:
        ElasticTracker();
        void Run();
    };
}


#endif //BPMP_TRACKER_ELASTICTRACKER_H
