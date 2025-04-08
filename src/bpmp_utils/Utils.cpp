//
// Created by larr-laptop on 25. 4. 8.
//
#include <bpmp_utils/Utils.h>

double bpmp::interpolate(const std::vector<double> &xData, const std::vector<double> &yData, const double &x) {
    int size = (int)xData.size();
    if(xData.size()<2)
        return yData[0];

    if(x<xData[0])
        return yData[0];

    if(x>xData[size-1])
        return yData.back();

    double xL= xData[0], yL = yData[0], xR = xData.back(), yR = yData.back();

    for(int i =0;i<size-2;i++){
        if(xData[i]<=x and x<xData[i+1]){
            xL = xData[i]; yL = yData[i];
            xR = xData[i+1], yR = yData[i+1];
            break;
        }
    }
    double dydx = (yR-yL)/(xR-xL);
    return yL + dydx * (x-xL);
}