//
// Created by larr-laptop on 25. 6. 9.
//

#ifndef BPMP_TRACKER_WRAPPER_H
#define BPMP_TRACKER_WRAPPER_H

#include <ros/ros.h>
#include <bpmp_tracker/RosWrapper.h>

namespace bpmp{
    using namespace std;
    class Wrapper{
    public:
        Wrapper();
        void Run();
    private:
        thread thread_planner_;
        thread thread_ros_wrapper_;
        shared_ptr<PlannerBase> p_base_shared_;

        bpmp::RosWrapper* ros_wrapper_ptr_;
        bpmp::Tracker * tracker_;

        ros::NodeHandle nh_;

        void InitRosSubscriberAndPublisher();
        void UpdateTrackerParameters();
        void RunPlanning();
    };

}


#endif //BPMP_TRACKER_WRAPPER_H
