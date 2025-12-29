//
// Created by larr-laptop on 12/28/25.
//

#ifndef BPMP_TRACKER_OPTIMIZER_HPP
#define BPMP_TRACKER_OPTIMIZER_HPP
#include <cmath>
#include <array>
#include <comparison/Problem.h>

namespace bpmp{
    template<typename T, const int size>
    using Collection = std::array<T, size>;

    struct OptimizationParam
    {
        double time_step;
        double param_1;
        double param_2;
        double param_3;
        double param_4;
    };


    template<const int Nx, const int Nu, const int N>
    class Optimizer {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    protected:
        typedef Matrix<double,Nx,1>     VectorX;
        typedef Matrix<double,Nu,1>     VectorU;
    public:
        Optimizer(const ProblemDescription<Nx,Nu>&,const VectorX &, const Collection<VectorU,N>&, const double&, const OptimizationParam&);
        void Solve();
    private:
        void SetParameters(const OptimizationParam &param);
        OptimizationParam param_;
    };

    template<const int Nx, const int Nu, const int N>
    Optimizer<Nx, Nu, N>::Optimizer(const ProblemDescription <Nx, Nu> & problem, const Optimizer::VectorX & x_init,
                                    const Collection<Optimizer::VectorU, N> & u_init, const double & time_step,
                                    const OptimizationParam & param) {
        SetParameters(param);

    }

    template<const int Nx, const int Nu, const int N>
    void Optimizer<Nx, Nu, N>::SetParameters(const OptimizationParam &param) {
        param_ = param;
    }

    template<const int Nx, const int Nu, const int N>
    void Optimizer<Nx, Nu, N>::Solve() {
//        std::cout<<"SOLVE THE PROBLEM"<<std::endl;
    }

}

#endif //BPMP_TRACKER_OPTIMIZER_HPP
