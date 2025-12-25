#include <comparison/TrackingController.h>
int main(int argc, char**argv){
    ros::init(argc,argv,"tracking_controller");
    bpmp::TrackingController controller;
    controller.Run();
    return 0;
}

