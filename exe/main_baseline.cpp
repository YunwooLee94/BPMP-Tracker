//
// Created by larr-laptop on 12/28/25.
//
#include <comparison/Baseline.h>
int main(int argc, char**argv){
    ros::init(argc,argv,"baseline");
    bpmp::Baseline baseline;
    baseline.Run();
    return 0;
}
