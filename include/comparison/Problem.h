// Problem.h
#ifndef BPMP_TRACKER_PROBLEM_HPP
#define BPMP_TRACKER_PROBLEM_HPP

#include <comparison/Description.h>
#include <bpmp_utils/Utils.h>
#include <bpmp_tracker/ObjectState.h>
#include <bpmp_tracker/ObjectStateList.h>
#include <vector>

namespace bpmp{
    class Problem final : public ProblemDescription<N,Nu>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        Problem(const bpmp::TargetStateBaseline& target_state,
                const std::vector<bpmp::State>& obstacle_state_list,
                const bpmp::UnicycleState& tracker_state)
            : target_state_(target_state),
              obstacle_state_list_(obstacle_state_list),
              tracker_state_(tracker_state) {}

        const bpmp::TargetStateBaseline& target_state() const { return target_state_; }
        const std::vector<bpmp::State>& obstacle_state_list() const { return obstacle_state_list_; }
        const bpmp::UnicycleState& tracker_state() const { return tracker_state_; }

    private:
        bpmp::TargetStateBaseline target_state_;
        std::vector<bpmp::State> obstacle_state_list_;
        bpmp::UnicycleState tracker_state_;
    };
}

#endif //BPMP_TRACKER_PROBLEM_HPP