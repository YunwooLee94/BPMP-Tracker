#ifndef BPMP_TRACKER_PREDICTOR_H
#define BPMP_TRACKER_PREDICTOR_H

#include <ros/ros.h>
#include <bpmp_predictor/PredictionVisualizer.h>
#include <bpmp_utils/eigenmvn.h>
#include <bpmp_tracker/ObjectState.h>
#include <bpmp_tracker/ObjectStateList.h>
#include <bpmp_tracker/PolyState.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>
#include <decomp_util/seed_decomp.h>
#include <decomp_ros_utils/data_ros_utils.h>

namespace bpmp{
    struct PredictonParam{
        int num_thread;
        int num_sample;
        double horizon;
        double acc_max_target;
        double target_size;
    };
    class Predictor{
    public:
        Predictor();
        void Run();
    private:
        ros::NodeHandle nh_;
        PredictonParam param_;

        bpmp::VisualizationParam vis_param_;
        bpmp::PredictionVisualizer visualizer_;

        ros::Subscriber target_state_subscriber_;
        ros::Subscriber obstacle_state_list_subscriber_;

        ros::Subscriber pcl_subscriber_;
        ros::Publisher corridor_publisher_;

        ros::Publisher raw_primitive_publisher_;
        ros::Publisher feasible_primitive_publisher_;
        ros::Publisher best_primitive_publisher_;
        ros::Publisher prediction_result_publisher_;

        double t0_{ros::Time::now().toSec()};
        double GetCurrentTime(){return ros::Time::now().toSec()-t0_;};
        void TargetStateCallback(const bpmp_tracker::ObjectState &msg);
        void PclCallback(const sensor_msgs::PointCloud2::ConstPtr &pcl_msgs);
        void ObstacleStateListCallback(const bpmp_tracker::ObjectStateList &msg);

        bool is_pcl_received_{false};
        bool is_dyn_obs_received_{false};

        bpmp::State current_target_state_;
        vector<bpmp::PrimitiveTarget> current_obstacle_primitive_list_;
        vec_Vec3f point_cloud_3d_;
        vec_E<Polyhedron3D> polys_;

        vector<bpmp::Point> end_points_;
        vector<bpmp::PrimitiveTarget> primitive_;
        vector<bpmp::uint> safe_index_;

        bool Predict();
        void SampleEndPoints();
        void SampleEndPointsSubProcess(const int &start_idx, const int &end_idx, vector<Point> &endpoint_list_sub);
        void GeneratePrimitives();
        void GeneratePrimitivesSubProcess(const int &start_idx, const int &end_idx, vector<PrimitiveTarget> &primitive_sub);
        vector<bpmp::uint> GetSafeIndexUnstructured(const vector<bpmp::uint> &prior_idx);
        void GetSafeIndexUnstructuredSubProcess(const LinearConstraint3D &constraint, const vector<uint> &prior_idx, const int &start_idx, const int &end_idx, std::vector<uint> &safe_index_sub);
        vector<bpmp::uint> GetSafeIndexDynamic(const vector<bpmp::uint> &prior_idx);
        void GetSafeIndexDynamicSubProcess(const vector<uint> &prior_idx, const int &start_idx, const int &end_idx, std::vector<uint> &safe_index_sub);

        LinearConstraint3D GenerateCorridor();
        void GetBestIndex();
        void GetBestIndexSubProcess(const int &start_idx, const int &end_idx, std::pair<uint, double> &best_index_sub);
        uint best_prediction_index_;
        void UpdateResult();
        void EraseResult();


        int EnvironmentMode();

    };
}
#endif //BPMP_TRACKER_PREDICTOR_H
