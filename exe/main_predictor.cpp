//
// Created by larr-laptop on 25. 4. 15.
//
#include <bpmp_predictor/Predictor.h>

int main(int argc, char**argv){
    ros::init(argc,argv,"bpmp_predictor");
    bpmp::Predictor predictor;
    predictor.Run();
    return 0;
}