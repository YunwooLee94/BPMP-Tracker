//
// Created by larr-laptop on 25. 4. 15.
//
#include <bpmp_simulator/RosTypeConverter.h>

bpmp::RosTypeConverter::RosTypeConverter():nh_("~") {
    t0_ = ros::Time::now().toSec();
    TargetPositionSubscriber_ = nh_.subscribe("/target_pose",1,&RosTypeConverter::TargtPositionCallback,this);
    TargetStatePublisher_ = nh_.advertise<bpmp_tracker::ObjectState>("/bpmp_simulator/target_state",1);
    PclSubscriber_ = nh_.subscribe("/points_masked",1,&RosTypeConverter::PclCallback,this);
    PclPublisher_ = nh_.advertise<sensor_msgs::PointCloud2>("/bpmp_simulator/point_cloud_obstacle",1);
    UnicycleInputSubscriber_ = nh_.subscribe("/bpmp_tracker/unicycle_control_input",1,&RosTypeConverter::UnicycleInputCallback,this);
    UnicycleInputPublisher_ = nh_.advertise<geometry_msgs::Twist>("/move_base/cmd_vel",1);
    RobotOdometrySuscriber_ = nh_.subscribe("/base_odom", 1,
                                            &RosTypeConverter::RobotOdometryCallback, this);
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
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z));
    tf::Quaternion q(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
    transform.setRotation(q);
    br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "current"));
}

void bpmp::RosTypeConverter::Publish() {
    TargetStatePublisher_.publish(current_target_state_);
    PclPublisher_.publish(pcl_output_);
}

void bpmp::RosTypeConverter::PclCallback(const pcl::PointCloud<pcl::PointXYZ> &msg) {
    pcl::toROSMsg(msg,pcl_output_);
    pcl_output_.header.frame_id="map";
    pcl_output_.header.stamp = ros::Time::now();
}

void bpmp::RosTypeConverter::UnicycleInputCallback(const bpmp_tracker::UnicycleInput &msg) {
    geometry_msgs::Twist control_input;
    control_input.linear.x = msg.vel_linear;
    control_input.linear.y = 0.0;
    control_input.linear.z = 0.0;
    control_input.angular.x = 0.0;
    control_input.angular.y = 0.0;
    control_input.angular.z = msg.vel_angular;
    UnicycleInputPublisher_.publish(control_input);
}
