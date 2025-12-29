//
// Created by larr-laptop on 12/28/25.
//

#ifndef BPMP_TRACKER_PROBLEM_H
#define BPMP_TRACKER_PROBLEM_H
#include <comparison/Description.h>
#include <comparison/Dimension.h>
#include <bpmp_tracker/UnicycleInput.h>
#include <bpmp_utils/Utils.h>
#include <bpmp_tracker/UnicycleInput.h>
#include <bpmp_tracker/ObjectState.h>
#include <bpmp_tracker/ObjectStateList.h>
template<typename T, const int size>
using Collection = std::array<T, size>;

namespace bpmp{
    class Problem : public ProblemDescription<Nx,Nu>{
    private:
        bpmp::State target_state_;
        vector<bpmp::State> obstacle_state_list_;
        bpmp::UnicycleState tracker_state_;
    public:
        Problem (const bpmp::State &target_state, const vector<bpmp::State> &obstacle_state_list,
                 const bpmp::UnicycleState &tracker_state): target_state_(target_state),obstacle_state_list_(obstacle_state_list),
                                                            tracker_state_(tracker_state){ };

    };

}


#endif //BPMP_TRACKER_PROBLEM_H
