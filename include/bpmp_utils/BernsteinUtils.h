//
// Created by larr-laptop on 25. 4. 8.
//

#ifndef BPMP_TRACKER_BERNSTEIN_UTILS_H
#define BPMP_TRACKER_BERNSTEIN_UTILS_H
#include <cmath>
#include <bpmp_utils/Utils.h>

namespace bpmp{
    int factorial(int num);

    int nchoosek(int n, int r);

    int nchooser(int n, int r);

    double getBernsteinValue(double bern_ctrl_pts[], double t, double t0, double tf, int poly_order);

}


#endif //BPMP_TRACKER_BERNSTEIN_UTILS_H
