//
// Created by larr-laptop on 25. 4. 15.
//
#include <bpmp_simulator/RosTypeConverter.h>

int main(int argc, char**argv){
    ros::init(argc,argv,"bpmp_ros_type_converter");
    bpmp::RosTypeConverter converter;
    converter.Run();
    return 0;
}