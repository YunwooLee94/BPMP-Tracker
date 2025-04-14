//
// Created by larr-laptop on 25. 4. 8.
//
#include <bpmp_simulator/Simulator.h>

int main(int argc, char**argv){
    ros::init(argc,argv,"bpmp_simulator");
    bpmp::Simulator simulator;
    simulator.Run();
    return 0;
}