//
// Created by larr-laptop on 25. 6. 9.
//
//
// Created by larr-planning on 24. 3. 5.
//
#include <bpmp_tracker/PlannerBase.h>

void
bpmp::PlannerBase::SetTrackerPrimitives(const std::vector<bpmp::PrimitivePlanning> &tracker_primitive) {
    tracker_raw_primitives_.clear();
    tracker_raw_primitives_ = tracker_primitive;
}

void bpmp::PlannerBase::EraseTrackerPrimitives() {
    tracker_raw_primitives_.clear();
}

void bpmp::PlannerBase::EraseFeasibleIndex() {
    tracker_feasible_index_.clear();
}

void bpmp::PlannerBase::SetFeasibleIndex(const std::vector<uint> &feasible_index) {
    tracker_feasible_index_.clear();
    tracker_feasible_index_ = feasible_index;
}

void bpmp::PlannerBase::SetBestIndex(const uint &best_index) {
    tracker_best_index_ = best_index;
}

void bpmp::PlannerBase::SetCorridorVis(const vec_E<Polyhedron3D> &poly) {
    polys_ = poly;
}
