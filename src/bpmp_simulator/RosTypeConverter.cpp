//
// Created by larr-laptop on 25. 4. 15.
//
#include <bpmp_simulator/RosTypeConverter.h>

bpmp::RosTypeConverter::RosTypeConverter():nh_("~") {
    t0_ = ros::Time::now().toSec();
    TargetPositionSubscriber_ = nh_.subscribe("/target_pose",1,&RosTypeConverter::TargtPositionCallback,this);
    TargetStatePublisher_ = nh_.advertise<bpmp_tracker::ObjectState>("/bpmp_simulator/target_state",1);
}

void bpmp::RosTypeConverter::Run() {
    ros::Rate loop_rate(200.0);
    while(ros::ok()){
        Publish();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void bpmp::RosTypeConverter::TargtPositionCallback(const geometry_msgs::PoseStampedConstPtr &msg) {
    double curr_time = curTime();
    if(not is_target_position_received_){
        current_target_state_.px = msg->pose.position.x;
        current_target_state_.py = msg->pose.position.y;
        current_target_state_.pz = msg->pose.position.z;
        current_target_state_.vx = 0.0;
        current_target_state_.vy = 0.0;
        current_target_state_.vz = 0.0;
        t0_history_ = curr_time;
        odom_count_++;
        is_target_position_received_ = true;
    }
    else{
        if(odom_count_<5){  //TODO: Parameterize (Yunwoo)
            previous_target_state_.px = current_target_state_.px;
            previous_target_state_.py = current_target_state_.py;
            previous_target_state_.pz = current_target_state_.pz;
            current_target_state_.px = msg->pose.position.x;
            current_target_state_.py = msg->pose.position.y;
            current_target_state_.pz = msg->pose.position.z;
            current_target_state_.vx = (current_target_state_.px-previous_target_state_.px)/(curr_time-t0_history_);
            current_target_state_.vy = (current_target_state_.py-previous_target_state_.py)/(curr_time-t0_history_);
            current_target_state_.vz = (current_target_state_.pz-previous_target_state_.pz)/(curr_time-t0_history_);
            odom_count_++;
        }
        else{
            previous_target_state_.px = current_target_state_.px;
            previous_target_state_.py = current_target_state_.py;
            previous_target_state_.pz = current_target_state_.pz;
            current_target_state_.px = msg->pose.position.x;
            current_target_state_.py = msg->pose.position.y;
            current_target_state_.pz = msg->pose.position.z;
            vel_history_[odom_count_iter_].vx = (current_target_state_.px-previous_target_state_.px)/(curr_time-t0_history_);
            vel_history_[odom_count_iter_].vy = (current_target_state_.py-previous_target_state_.py)/(curr_time-t0_history_);
            double temp_vel_x = 0.0; double temp_vel_y = 0.0;
            for(int i =0;i<5;i++){//TODO: Parmeterize (Yunwoo)
                temp_vel_x += vel_history_[i].vx;
                temp_vel_y += vel_history_[i].vy;
            }
            current_target_state_.vx = temp_vel_x/5.0;
            current_target_state_.vy = temp_vel_y/5.0;
            current_target_state_.vz = 0.0;
            odom_count_iter_++;
            if(odom_count_iter_ == 5)//TODO: Parameterize (Yunwoo)
                odom_count_iter_ = 0;
        }
        t0_history_ = curr_time;
    }
}

void bpmp::RosTypeConverter::RobotOdometryCallback(const nav_msgs::OdometryConstPtr &msg) {

}

void bpmp::RosTypeConverter::Publish() {
    TargetStatePublisher_.publish(current_target_state_);
}
