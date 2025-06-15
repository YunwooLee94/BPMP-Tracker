//
// Created by larr-laptop on 25. 6. 9.
//

#ifndef BPMP_TRACKER_PLANNERBASE_H
#define BPMP_TRACKER_PLANNERBASE_H
#include <mutex>
//#include <bpmp_tracker/PlanningVisualizer.h>
#include <string>
#include <bpmp_utils/Utils.h>
#include <bpmp_utils/BernsteinUtils.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>
#include <decomp_util/ellipsoid_decomp.h>
#include <decomp_ros_utils/data_ros_utils.h>

namespace bpmp{
    struct TrackingParam{
        double horizon{0.0};
        bool is_unstructured{false};
        int num_thread{1};
        int num_sample_planning{1};
        double r_min{0.0};
        double r_max{0.0};
        double safe_distance{0.0};
        double vel_max{0.0};
        double acc_max{0.0};
        double ang_max{0.0};
        double fov{0.0};
        double object_radius{0.0};
        double terminal_weight{0.0};
        double distance_max{10.0};
        bool is_experiment{false};
        struct{
            double min_x{0.0};
            double min_y{0.0};
            double max_x{0.0};
            double max_y{0.0};
        }axis_limit;
    };

    class PlannerBase{
    private:
    public:
        std::mutex mutex_set_[2];
        // BUFFER DATA FROM ROS WRAPPER
        bpmp::UnicycleState current_tracker_list_read_;
        vector<bpmp::State> current_obstacle_list_read_;
        bpmp::PrimitivePlanning target_prediction_read_;

        vector<bpmp::PrimitivePlanning> tracker_raw_primitives_;
        vector<bpmp::uint> tracker_feasible_index_;
        bpmp::uint tracker_best_index_;

        vec_Vec3f point_cloud_3d_;
        vec_E<Polyhedron3D> polys_;

        bool is_tracker_info_{false};
        bool is_target_info_{false};
        bool is_obstacle_info_{false};

        bool success_flag_{false};

        void SetTrackerPrimitives(const vector<PrimitivePlanning>&tracker_primitive);
        void EraseTrackerPrimitives();

        void SetFeasibleIndex(const vector<bpmp::uint> &feasible_index);
        void EraseFeasibleIndex();
        void SetBestIndex(const bpmp::uint &best_index);
        void SetCorridorVis(const vec_E<Polyhedron3D> &poly);

    };
}

#endif //BPMP_TRACKER_PLANNERBASE_H
