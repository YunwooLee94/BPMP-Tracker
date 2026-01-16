//
// Created by larr-laptop on 1/15/26.
//
#include <comparison/ElasticTracker.h>
int main(int argc, char**argv){
    ros::init(argc,argv,"elastic_tracker");
    bpmp::ElasticTracker tracker;
    tracker.Run();
    return 0;
}