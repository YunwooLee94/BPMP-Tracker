//
// Created by larr-laptop on 25. 6. 9.
//
#include <bpmp_tracker/Wrapper.h>
int main(int argc, char**argv){
    ros::init(argc,argv,"bpmp_tracker");
    bpmp::Wrapper wrapper;
    wrapper.Run();
    return 0;
}
