// Description.h
#ifndef BPMP_TRACKER_DESCRIPTION_HPP
#define BPMP_TRACKER_DESCRIPTION_HPP

#include <Eigen/Geometry>
#include <comparison/Dimension.h>

template<const int N, const int Nu>
class ProblemDescription{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual ~ProblemDescription() = default;  // ★ 이 줄이 핵심 (polymorphic)

    ProblemDescription() = default;
};

#endif //BPMP_TRACKER_DESCRIPTION_HPP
