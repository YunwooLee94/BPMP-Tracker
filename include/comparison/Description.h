//
// Created by larr-laptop on 12/28/25.
//

#ifndef BPMP_TRACKER_DESCRIPTION_H
#define BPMP_TRACKER_DESCRIPTION_H

#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

// base class for type definition
template<const int Nx, const int Nu>
class DescriptionBase
{
protected:
    typedef Matrix<double,Nx,1> VectorX;
    typedef Matrix<double,Nu,1> VectorU;
};

template<const int Nx, const int Nu>
class ProblemDescription : public DescriptionBase<Nx,Nu>
{
public:
    using typename DescriptionBase<Nx,Nu>::VectorX;
    using typename DescriptionBase<Nx,Nu>::VectorU;
    explicit ProblemDescription(){};
    ~ProblemDescription(){};
};





#endif //BPMP_TRACKER_DESCRIPTION_H
