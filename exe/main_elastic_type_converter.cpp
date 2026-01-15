//
// Created by larr-laptop on 1/14/26.
//
#include <comparison/ElasticTypeConverter.h>
int main(int argc, char**argv){
    ros::init(argc,argv,"elastic_type_converter");
    bpmp::ElasticTypeConverter converter;
    converter.Run();
    return 0;
}
