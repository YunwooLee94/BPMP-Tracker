//
// Created by larr-laptop on 25. 6. 9.
//
#ifndef BPMP_TRACKER_TRACKER_H
#define BPMP_TRACKER_TRACKER_H
#include <bpmp_tracker/PlannerBase.h>
#include <bpmp_utils/eigenmvn.h>
namespace bpmp{
    class Tracker{
    public:
        Tracker(const TrackingParam & param, shared_ptr<PlannerBase> p_base);

        shared_ptr <PlannerBase> p_base_;
        bool Plan(const double &t_trigger);
        int EnvironmentMode();
        void UpdateResultToBase(const bool& is_success);
    private:
        TrackingParam param_;
        vector<bpmp::PrimitivePlanning> primitive_;
        vector<bpmp::PrimitivePlanning> obstacle_primitive_list_;
        bpmp::PrimitivePlanning target_trajectory_;
        bpmp::UnicycleState current_tracker_state_;
        Eigen::Transform<double,3,Eigen::Affine> current_pose_mat_;
        int num_tracker_;
        int num_obstacle_;

        vector<bpmp::Point> end_points_;
        vector<bpmp::uint> visible_index_;
        vector<bpmp::uint> safe_index_;
        vector<bpmp::uint> inter_safe_index_;
        vector<bpmp::uint> inter_visible_index_;
        vector<bpmp::uint> dynamically_feasible_index_;

        bpmp::uint best_index_;

        vec_Vec3f point_cloud_3d_;
        vec_E<Polyhedron3D> polys_;
        LinearConstraint3D corridor_constraints_;

        void UpdateValue(const double &t);
        bool CheckInfoAvailable();

        void SampleEndPoint();
        void SampleEndPointThread(const int & start_idx, const int & end_idx, vector<bpmp::Point>& endpoint_list_sub);
        void GeneratePrimitive(const double &t);
        void GeneratePrimitiveThread(const double &t, const int &start_idx, const int &end_idx, vector<bpmp::PrimitivePlanning> & primitive_list_sub);
        void GetFOVIndex();
        void GetFOVIndexThread(const int &start_idx, const int &end_idx, vector<bpmp::uint> &visible_idx_sub);
        vector<bpmp::uint> GetSafeIndexUnstructured(const vector<bpmp::uint> &index);
        void GetSafeIndexUnstructuredThread(const std::vector<uint> &prior_idx, const int &start_idx, const int &end_idx, vector<bpmp::uint> &safe_idx_sub);
        vector<bpmp::uint> GetSafeIndexDynamic(const vector<bpmp::uint> &index);
        void GetSafeIndexDynamicThread(const std::vector<uint> &prior_idx,const int &start_idx, const int &end_idx, vector<bpmp::uint> &safe_idx_sub);

        void GetSafeIndexThread(const int &start_idx, const int &end_idx, vector<bpmp::uint> &safe_idx_sub);
        void GenerateCorridor();

        void GetDynamicallyFeasibleIndex();
        void GetDynamicallyFeasibleIndexThread(const int &start_idx, const int &end_idx, vector<bpmp::uint> &dyn_feas_idx_sub);

        void GetBestIndex();
        void GetBestIndexThread(const int &start_idx, const int &end_idx, std::pair<uint,double> &score_pair);
    };
}
#endif //BPMP_TRACKER_TRACKER_H
