//
// Created by larr-laptop on 25. 6. 9.
//
#include <bpmp_tracker/Wrapper.h>

void bpmp::Wrapper::Run() {
    thread_planner_ = thread(&Wrapper::RunPlanning, this);
    ROS_INFO_ONCE("THREAD FOR PLANNING CREATED.");
    thread_ros_wrapper_ = thread(&RosWrapper::RunRos, ros_wrapper_ptr_);
    ROS_INFO_ONCE("THREAD FOR RUNROS CREATED.");
    thread_planner_.join();
    thread_ros_wrapper_.join();
}

bpmp::Wrapper::Wrapper():p_base_shared_(std::make_shared<bpmp::PlannerBase>()) {
    ros_wrapper_ptr_ = new RosWrapper(p_base_shared_);
    ros_wrapper_ptr_->InitSubscriberAndPublisher();
    tracker_ = new Tracker(ros_wrapper_ptr_->GetTrackingParam(),p_base_shared_);
}

void bpmp::Wrapper::RunPlanning() {
    ros::Rate loop_rate(ros_wrapper_ptr_->GetPlanningFrequency());
    while(ros::ok()){
        bool planning_success = tracker_->Plan(ros_wrapper_ptr_->GetCurrentTime());
        tracker_->UpdateResultToBase(planning_success);
        ros::spinOnce();
        loop_rate.sleep();
    }
}