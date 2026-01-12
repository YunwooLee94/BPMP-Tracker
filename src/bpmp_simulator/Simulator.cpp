#define CONVHULL_3D_ENABLE
#include <bpmp_simulator/Simulator.h>

void bpmp::Simulator::Run() {
    if (not analysis_mode_){
        double simulation_frequency = 1.0/simulation_dt_;
        ros::Rate loop_rate(simulation_frequency);
        double t0 = ros::Time::now().toSec();
        double t_sim;
        while(ros::ok()){
            t_sim = ros::Time::now().toSec()-t0;
            UpdateDynamics(t_sim);
            PrepareRosMsgs(t_sim);
            PublishRosMsgs();
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    else{
        ros::Rate loop_rate(1.0/simulation_dt_);
        double t0 = ros::Time::now().toSec();
        double t_sim;
        int test_cnt = 1; // For same target
        int scenario_count = 0;
        int accumulated_success_count = 0;
        int success_count = 0;
        int episode_count = 0;
        int object_file_count = 0;
        int object_file_number = 3;
        bool fail_flag_target = false;
        bool fail_flag_obstacle = false;
        bool fail_flag_fov = false;
        bool error_flag = false;
        while(ros::ok()){
            if(object_file_count==object_file_number){
            	accumulated_success_count += success_count;
            	cout <<"TOTAL SCENARIO: "<<object_file_number*total_test_number_<< " SCENARIO CNT: "
                     << object_file_count*total_test_number_+scenario_count << ", SUCCESS CNT: " << accumulated_success_count << endl;
                break;
            }
            if(test_cnt>total_test_number_){
                accumulated_success_count += success_count;
                cout <<"UNTIL "<<object_file_count+1<<"-th Scenario: " <<"TOTAL SCENARIO: "<<object_file_number*total_test_number_<< " SUB SCENARIO CNT: "
                     << object_file_count*total_test_number_+scenario_count << " SUB SUCCESS CNT: " << accumulated_success_count << endl;
                test_cnt = 1;
                scenario_count = 0;
                success_count = 0;
                object_history_file_name_ = object_history_file_path_name_+"/gar_"+std::to_string(++object_file_count)+".csv";
                ReadObjectTrajectory();
            }
            t_sim = ros::Time::now().toSec() - t0;
            if (t_sim > object_history_list_[0].t.back()) {
                episode_count++;
                if(not error_flag)
                    scenario_count++;
                bool is_tracker_far_from_target = false;
                double distance = sqrt(pow(current_target_state_.px - current_unicycle_state_.px, 2) +
                                       pow(current_target_state_.py - current_unicycle_state_.py, 2));
                if (distance > 5.0)
                    is_tracker_far_from_target = true;

                if (not(fail_flag_target or fail_flag_obstacle or fail_flag_fov or is_tracker_far_from_target))
                    success_count++;
                else {
                    if(fail_flag_target)
                        ROS_WARN("ROBOT COLLLIDES OR IS TOO FAR WITH TARGET");
                    if(fail_flag_obstacle)
                        ROS_WARN("ROBOT COLLIDES WITH OBSTACLES");
                    if(fail_flag_fov)
                        ROS_WARN("ROBOT FAILS TO KEEP TARGET WITHIN FOV");
                }
                cout <<"PROCESS TOTAL SCENARIO: "<<total_test_number_<< " PROCCESS SCENARIO CNT: "
                     << scenario_count << " PROCESS SUCCESS CNT: " << success_count << endl;

                fail_flag_obstacle = false;
                fail_flag_target = false;
                fail_flag_fov = false;
                t0 = ros::Time::now().toSec();
                test_cnt++;
                ShuffleScenario();
                double theta;
                error_flag = true;
                for(int try_idx=0;try_idx<100;try_idx++) {
                    std::random_device rd;
                    std::mt19937 gen(rd());
                    double target_vel_start_x=0.0, target_vel_start_y=0.0;
                    for(int i=0;i<100;i++){
                        target_vel_start_x+=object_history_list_[target_idx_].vx[i];
                        target_vel_start_y+=object_history_list_[target_idx_].vy[i];
                        if(abs(target_vel_start_x)>1e-2 or abs(target_vel_start_y)>1e-2)
                            break;
                    }
                    double start_azimuth = atan2(target_vel_start_y,target_vel_start_x)+M_PI;

                    std::uniform_real_distribution<> theta_dis(start_azimuth-M_PI*0.1666667, start_azimuth+M_PI*0.1666667);
                    unicycle_control_input_.linear_speed =0.0, unicycle_control_input_.angular_speed = 0.0;
                    theta = theta_dis(gen);
                    current_unicycle_state_.px = object_history_list_[target_idx_].px.front() + 1.0 * cos(theta);
                    current_unicycle_state_.py = object_history_list_[target_idx_].py.front() + 1.0 * sin(theta);
                    current_unicycle_state_.pz = 0.5;
                    current_unicycle_state_.theta = atan2(object_history_list_[target_idx_].py.front()-current_unicycle_state_.py,
                                                          object_history_list_[target_idx_].px.front()-current_unicycle_state_.px);

                    bool is_okay_at_start = true;
                    double temp_distance_squared = 0.0;

                    for (int j = 0; j < obstacle_idx_list_.size(); j++) {
                        temp_distance_squared =
                                pow(current_unicycle_state_.px - object_history_list_[obstacle_idx_list_[j]].px.front(), 2) +
                                pow(current_unicycle_state_.py - object_history_list_[obstacle_idx_list_[j]].py.front(), 2);
                        if (temp_distance_squared < 9*agent_size_ * agent_size_)
                            is_okay_at_start = false;
                    }
                    if (is_okay_at_start){
                        error_flag = false;
                        break;
                    }

                }
                t_sim = ros::Time::now().toSec() - t0;
            }
            UpdateDynamics(t_sim);
            if (sqrt(pow(current_target_state_.px - current_unicycle_state_.px, 2) +
                     pow(current_target_state_.py - current_unicycle_state_.py, 2)) < 2 * agent_size_) {
                fail_flag_target = true;
            } // Target-Robot Collision
            if (sqrt(pow(current_target_state_.px - current_unicycle_state_.px, 2) +
                     pow(current_target_state_.py - current_unicycle_state_.py, 2)) > 5.0) {
                fail_flag_target = true;
            } // Target-Robot Collision
            for (int idx = 0; idx < moving_obstacle_number_; idx++) {
                if (sqrt(pow(current_obstacle_state_list_[idx].px - current_unicycle_state_.px, 2) +
                         pow(current_obstacle_state_list_[idx].py - current_unicycle_state_.py, 2)) < 2 * agent_size_) {
                    fail_flag_obstacle = true;
                }
            } // Obstacle-Robot Too Close
            double direction = atan2(current_target_state_.py-current_unicycle_state_.py,current_target_state_.px-current_unicycle_state_.px);
            double yaw_gap = direction-current_unicycle_state_.theta;
            yaw_gap = fmod(yaw_gap+M_PI,2.0*M_PI);
            if(yaw_gap<0)
                yaw_gap += 2.0*M_PI;
            yaw_gap -= M_PI;
            if(abs(yaw_gap)>1.0472) // 1.0472: 60 degree, 1.309: 75 degree
                fail_flag_fov = true;
            PrepareRosMsgs(t_sim);
            PublishRosMsgs();
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
}
bpmp::Simulator::Simulator() : nh_("~") {
    nh_.param<bool>("analysis_mode",analysis_mode_,false);
    nh_.param<double>("spatial_scale",spatial_scale_,1.0);
    nh_.param<double>("time_scale",time_scale_,1.0);
    nh_.param<string>("map_frame_id", map_frame_id_, "map");
    nh_.param<double>("simulation_dt", simulation_dt_, 0.01);

    nh_.param<bool>("is_unstructured", is_unstructured_, false);
    nh_.param<double>("inflation_size", inflation_size_, 0.5);
    nh_.param<double>("point_resolution", point_resolution_, 0.1);
    nh_.param<double>("agent_size", agent_size_, 0.1);

    // Total number of agents in files
    nh_.param<int>("total_object_number_in_file", total_object_number_in_file_, 0);

    // Obstacle Indices
    if (nh_.hasParam("obstacle_idx_list")) {
        nh_.getParam("obstacle_idx_list", obstacle_idx_list_);
    } else
        ROS_ERROR("Failed to find 'obstacle list'.");
    object_idx_list_ = obstacle_idx_list_;
    // Target Index
    nh_.param<int>("target_idx", target_idx_, 0);
    object_idx_list_.push_back(target_idx_);
    object_number_ = (int) object_idx_list_.size(); // obstacles + target
    nh_.param<string>("initial_state_file_name", initial_state_file_name_, "");
    nh_.param<string>("object_history_file_name", object_history_file_name_, "");
    nh_.param<string>("object_history_file_path_name", object_history_file_path_name_, "");
    // unstructured file name
    nh_.param<string>("obstacle_configuration_file_name", obstacle_configuration_file_name_, "");
    nh_.param<int>("moving_obstacle_number", moving_obstacle_number_, 0);
    nh_.param<int>("total_test_number",total_test_number_,0);


    obstacle_vis_.header.frame_id = map_frame_id_;
    target_vis_.header.frame_id = map_frame_id_;
    tracker_vis_.header.frame_id = map_frame_id_;


    obstacle_vis_.type = visualization_msgs::Marker::CYLINDER;
    obstacle_vis_.ns = "Obstacle";
    obstacle_vis_.color.a = 1.0;
    obstacle_vis_.color.r = 0.0;
    obstacle_vis_.color.g = 1.0;
    obstacle_vis_.color.b = 0.0;
    obstacle_vis_.scale.x = 2 * agent_size_;
    obstacle_vis_.scale.y = 2 * agent_size_;
    obstacle_vis_.scale.z = 2.0;
    obstacle_vis_.pose.orientation.w = 1.0;
    obstacle_vis_.pose.orientation.x = 0.0;
    obstacle_vis_.pose.orientation.y = 0.0;
    obstacle_vis_.pose.orientation.z = 0.0;

    target_vis_.type = visualization_msgs::Marker::CYLINDER;
    target_vis_.ns = "Target";
    target_vis_.id = 0;
    target_vis_.color.a = 1.0;
    target_vis_.color.r = 1.0;
    target_vis_.color.g = 0.0;
    target_vis_.color.b = 0.0;
    target_vis_.scale.x = 2 * agent_size_;
    target_vis_.scale.y = 2 * agent_size_;
    target_vis_.scale.z = 2.0;
    target_vis_.pose.orientation.w = 1.0;
    target_vis_.pose.orientation.x = 0.0;
    target_vis_.pose.orientation.y = 0.0;
    target_vis_.pose.orientation.z = 0.0;

    target_path_.header.frame_id =map_frame_id_;
    robot_path_.header.frame_id = map_frame_id_;
//    obstacle_path_.markers.resize(moving_obstacle_number_);

    fov_vis_.header.frame_id = map_frame_id_;
    fov_vis_.color.a = 0.7;
    fov_vis_.color.r = 1.0;
    fov_vis_.color.g = 1.0;
    fov_vis_.color.b = 0.0;
    fov_vis_.pose.position = GetDefaultPointMsg();
    fov_vis_.pose.orientation = GetDefaultQuaternionMsg();
    fov_vis_.scale = GetDefaultScaleMsg();
    fov_vis_.type = visualization_msgs::Marker::TRIANGLE_LIST;

    single_obstacle_path_.header.frame_id = map_frame_id_;
    single_obstacle_path_.color.a = 1.0;
    single_obstacle_path_.color.r = 0.18;
    single_obstacle_path_.color.g = 0.44;
    single_obstacle_path_.color.b = 0.25;
    single_obstacle_path_.type=visualization_msgs::Marker::LINE_STRIP;
    single_obstacle_path_.ns="OBSTACLE_PATH";
    single_obstacle_path_.action = visualization_msgs::Marker::ADD;
    single_obstacle_path_.scale.x = 0.1;
    single_obstacle_path_.pose.position.x = 0.0;
    single_obstacle_path_.pose.position.y = 0.0;
    single_obstacle_path_.pose.position.z = 0.0;
    single_obstacle_path_.pose.orientation.w = 1.0;
    single_obstacle_path_.pose.orientation.x = 0.0;
    single_obstacle_path_.pose.orientation.y = 0.0;
    single_obstacle_path_.pose.orientation.z = 0.0;
    for(int i=0;i<moving_obstacle_number_;i++){
        single_obstacle_path_.id = i;
        obstacle_path_.markers.push_back(single_obstacle_path_);
    }


    point_cloud_.header.frame_id = map_frame_id_;

    target_vis_publisher_ = nh_.advertise<visualization_msgs::Marker>("target_vis", 1);
    obstacle_list_vis_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("obstacle_list_vis", 1);
    tracker_vis_publisher_ = nh_.advertise<nav_msgs::Odometry>("/base_odom", 1);

    obstacle_state_list_publisher_ = nh_.advertise<bpmp_tracker::ObjectStateList>("obstacle_state_list", 1);
    target_state_publisher_ = nh_.advertise<bpmp_tracker::ObjectState>("target_state", 1);
    tracker_state_publisher_ = nh_.advertise<bpmp_tracker::UnicycleState>("tracker_state", 1);

    pcl_publisher_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("point_cloud_obstacle", 1);
    pcl_boxes_vis_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("pcl_boxes", 1);


    control_input_subscriber_ = nh_.subscribe("/bpmp_tracker/tracker_control_input", 1,
                                              &Simulator::control_input_callback, this);
    unicycle_control_input_subscriber_ = nh_.subscribe("/bpmp_tracker/unicycle_control_input", 1,
                                              &Simulator::unicycle_input_callback, this);
    target_path_publisher_ = nh_.advertise<nav_msgs::Path>("target_path_vis",1);
    robot_path_publisher_ = nh_.advertise<nav_msgs::Path>("robot_path_vis",1);
    gar_robot_publisher_ = nh_.advertise<visualization_msgs::Marker>("robot_vis",1);
    fov_publisher_ = nh_.advertise<visualization_msgs::Marker>("fov_vis",1);
    obstacle_path_list_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("obstacle_path_vis",1);
    if(analysis_mode_){
        ReadObjectTrajectory();
        ShuffleScenario();
        double theta;
        while (true) {
            std::random_device rd;
            std::mt19937 gen(rd());
            double target_vel_start_x=0.0, target_vel_start_y=0.0;
            for(int i=0;i<100;i++){
                target_vel_start_x+=object_history_list_[target_idx_].vx[i];
                target_vel_start_y+=object_history_list_[target_idx_].vy[i];
                if(abs(target_vel_start_x)>1e-2 or abs(target_vel_start_y)>1e-2)
                    break;
            }
            double start_azimuth = atan2(target_vel_start_y,target_vel_start_x)+M_PI;
            cout<<"START AZIMUTH Y: "<<target_vel_start_y
                <<"X: "<<target_vel_start_x<<endl;

            std::uniform_real_distribution<> theta_dis(start_azimuth-M_PI*0.1666667, start_azimuth+M_PI*0.1666667);
            unicycle_control_input_.linear_speed =0.0, unicycle_control_input_.angular_speed = 0.0;
            theta = theta_dis(gen);
            current_unicycle_state_.px = object_history_list_[target_idx_].px.front() + 1.0 * cos(theta);
            current_unicycle_state_.py = object_history_list_[target_idx_].py.front() + 1.0 * sin(theta);
            current_unicycle_state_.pz = 0.5;
            current_unicycle_state_.theta = atan2(object_history_list_[target_idx_].py.front()-current_unicycle_state_.py,
                                                  object_history_list_[target_idx_].px.front()-current_unicycle_state_.px);

            bool is_okay_at_start = true;
            double temp_distance_squared = 0.0;

            for (int j = 0; j < obstacle_idx_list_.size(); j++) {
                temp_distance_squared =
                        pow(current_unicycle_state_.px - object_history_list_[obstacle_idx_list_[j]].px.front(), 2) +
                        pow(current_unicycle_state_.py - object_history_list_[obstacle_idx_list_[j]].py.front(), 2);
                if (temp_distance_squared < 9*agent_size_ * agent_size_)
                    is_okay_at_start = false;
            }
            if (is_okay_at_start)
                break;
        }
    }
    else{
//        ReadInitialTrackerStateList();
        ReadObjectTrajectory();
        unicycle_control_input_.linear_speed =0.0, unicycle_control_input_.angular_speed = 0.0;
        double theta = 3.0*3.141592/4.0;
        current_unicycle_state_.px = object_history_list_[target_idx_].px.front() + 1.0 * cos(theta);
        current_unicycle_state_.py = object_history_list_[target_idx_].py.front() + 1.0 * sin(theta);
        current_unicycle_state_.pz = 0.5;
        current_unicycle_state_.theta = atan2(object_history_list_[target_idx_].py.front()-current_unicycle_state_.py,
                                              object_history_list_[target_idx_].px.front()-current_unicycle_state_.px);

        if (is_unstructured_)
            ReadObstacleConfiguration();
    }
}

void bpmp::Simulator::ReadObjectTrajectory() {
    ifstream object_trajectory_file;
    object_history_file_name_ = object_history_file_path_name_+"/gar_1"+".csv";
    object_trajectory_file.open(object_history_file_name_.c_str());
    object_history_list_.clear();	
    int num_read_unit = 12;
    string line, word;
    vector<string> row;
    vector<vector<string>> content;
    if (object_trajectory_file.is_open()) {
        while (getline(object_trajectory_file, line)) {
            row.clear();
            stringstream str(line);
            while (getline(str, word, ','))
                row.push_back(word);
            content.push_back(row);
        }
    } else
        printf("UNABLE TO READ OBJECT TRAJECTORY HISTORY\n");
    StateHistory temp_obstacle_history;
    for (int i = 0; i < object_number_; i++)
        object_history_list_.push_back(temp_obstacle_history);
    for (int i = 0; i < object_number_; i++) {
        for (int j = 0; j < content.size(); j++) {
            object_history_list_[i].t.push_back(stod(content[j][object_idx_list_[i] * num_read_unit + 1])/time_scale_);   // t
            object_history_list_[i].px.push_back(stod(content[j][object_idx_list_[i] * num_read_unit + 2])*spatial_scale_);   // px
            object_history_list_[i].py.push_back(stod(content[j][object_idx_list_[i] * num_read_unit + 3])*spatial_scale_);   // py
            object_history_list_[i].pz.push_back(stod(content[j][object_idx_list_[i] * num_read_unit + 4])*spatial_scale_);   // pz
            object_history_list_[i].vx.push_back(stod(content[j][object_idx_list_[i] * num_read_unit + 5])*spatial_scale_*time_scale_);   // vx
            object_history_list_[i].vy.push_back(stod(content[j][object_idx_list_[i] * num_read_unit + 6])*spatial_scale_*time_scale_);   // vy
            object_history_list_[i].vz.push_back(stod(content[j][object_idx_list_[i] * num_read_unit + 7])*spatial_scale_*time_scale_);   // vz
        }
    }
//    double min_distance_between_object = 999999.0;
//    double temp_distance;
//    for(int i=0;i<object_number_-1;i++){
//        for(int j=i+1;j<object_number_;j++){
//            for(int k=0;k<object_history_list_[i].t.size();k++){
//                temp_distance = sqrt(pow(object_history_list_[i].px[k]-object_history_list_[j].px[k],2)+pow(object_history_list_[i].py[k]-object_history_list_[j].py[k],2));
//                if(temp_distance<min_distance_between_object)
//                    min_distance_between_object = temp_distance;
//            }
//        }
//    }
//    cout<<"[ATTENTION]: MIN DISTANCE BTW OBJECTS: "<<min_distance_between_object<<endl;
//    double max_velocity_target=-1;
//    for(int i=0;i<object_number_;i++){
//        for(int j=0;j<object_history_list_[i].t.size();j++){
//            if(max_velocity_target<sqrt(pow(object_history_list_[i].vx[j],2)+pow(object_history_list_[i].vy[j],2)))
//                max_velocity_target = sqrt(pow(object_history_list_[i].vx[j],2)+pow(object_history_list_[i].vy[j],2));
//        }
//    }
//    cout<<"[ATTENTION]: MAX VELOCITY OF TARGET: "<<max_velocity_target<<endl;

}

void bpmp::Simulator::UpdateDynamics(const double &t) {
    current_obstacle_state_list_.clear();
    State temp_state;
    {   // Target State
        current_target_state_.px = bpmp::interpolate(object_history_list_[target_idx_].t,
                                                     object_history_list_[target_idx_].px, t);
        current_target_state_.py = bpmp::interpolate(object_history_list_[target_idx_].t,
                                                     object_history_list_[target_idx_].py, t);
        current_target_state_.pz = bpmp::interpolate(object_history_list_[target_idx_].t,
                                                     object_history_list_[target_idx_].pz, t);
        current_target_state_.vx = bpmp::interpolate(object_history_list_[target_idx_].t,
                                                     object_history_list_[target_idx_].vx, t);
        current_target_state_.vy = bpmp::interpolate(object_history_list_[target_idx_].t,
                                                     object_history_list_[target_idx_].vy, t);
        current_target_state_.vz = bpmp::interpolate(object_history_list_[target_idx_].t,
                                                     object_history_list_[target_idx_].vz, t);
    }
    {   // Obstacle State
        for (int i = 0; i < obstacle_idx_list_.size(); i++) {
            temp_state.px = bpmp::interpolate(object_history_list_[obstacle_idx_list_[i]].t,
                                              object_history_list_[obstacle_idx_list_[i]].px, t);
            temp_state.py = bpmp::interpolate(object_history_list_[obstacle_idx_list_[i]].t,
                                              object_history_list_[obstacle_idx_list_[i]].py, t);
            temp_state.pz = bpmp::interpolate(object_history_list_[obstacle_idx_list_[i]].t,
                                              object_history_list_[obstacle_idx_list_[i]].pz, t);
            temp_state.vx = bpmp::interpolate(object_history_list_[obstacle_idx_list_[i]].t,
                                              object_history_list_[obstacle_idx_list_[i]].vx, t);
            temp_state.vy = bpmp::interpolate(object_history_list_[obstacle_idx_list_[i]].t,
                                              object_history_list_[obstacle_idx_list_[i]].vy, t);
            temp_state.vz = bpmp::interpolate(object_history_list_[obstacle_idx_list_[i]].t,
                                              object_history_list_[obstacle_idx_list_[i]].vz, t);
            current_obstacle_state_list_.push_back(temp_state);
        }
    }

    static double t_prev = t;
    double dt = t - t_prev;
    current_unicycle_state_.px = current_unicycle_state_.px + unicycle_control_input_.linear_speed*cos(current_unicycle_state_.theta)*dt;
    current_unicycle_state_.py = current_unicycle_state_.py + unicycle_control_input_.linear_speed*sin(current_unicycle_state_.theta)*dt;
    current_unicycle_state_.theta = current_unicycle_state_.theta +unicycle_control_input_.angular_speed*dt;
//    std::cout<<"DT: "<<dt<<std::endl;
    t_prev = t;
}

void bpmp::Simulator::ReadInitialTrackerStateList() {
    std::ifstream initial_state_file;
    initial_state_file.open(initial_state_file_name_.c_str());
    if (initial_state_file.is_open())
        initial_state_file>>current_unicycle_state_.px >>current_unicycle_state_.py >> current_unicycle_state_.pz >>current_unicycle_state_.theta;
    else{
        current_unicycle_state_.px = 0.0, current_unicycle_state_.py = 0.0, current_unicycle_state_.pz = 1.0, current_unicycle_state_.theta = 0.0;
    }
    initial_state_file.close();
    unicycle_control_input_.linear_speed =0.0, unicycle_control_input_.angular_speed = 0.0;
}

void bpmp::Simulator::PrepareRosMsgs(const double &t) {
    // Target Visualization
    target_vis_.pose.position.x = current_target_state_.px;
    target_vis_.pose.position.y = current_target_state_.py;
    target_vis_.pose.position.z = current_target_state_.pz;

    // Obstacle Visualization
    obstacle_list_vis_.markers.clear();
    for (int i = 0; i < current_obstacle_state_list_.size(); i++) {
        obstacle_vis_.id = i;
        obstacle_vis_.ns = std::to_string(i);
        obstacle_vis_.pose.position.x = current_obstacle_state_list_[i].px;
        obstacle_vis_.pose.position.y = current_obstacle_state_list_[i].py;
        obstacle_vis_.pose.position.z = current_obstacle_state_list_[i].pz;
        obstacle_list_vis_.markers.push_back(obstacle_vis_);
    }
    // Tracker Visualization (Nav_msgs in Unicycle Simulator)
    tracker_vis_.pose.pose.position.x = current_unicycle_state_.px;
    tracker_vis_.pose.pose.position.y = current_unicycle_state_.py;
    tracker_vis_.pose.pose.position.z = 0.5;
    tracker_vis_.pose.pose.orientation.x = 0.0;
    tracker_vis_.pose.pose.orientation.y = 0.0;
    tracker_vis_.pose.pose.orientation.z = sin(0.5*current_unicycle_state_.theta);
    tracker_vis_.pose.pose.orientation.w = cos(0.5*current_unicycle_state_.theta);
    // Tracker tf Publish
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(tracker_vis_.pose.pose.position.x,tracker_vis_.pose.pose.position.y,tracker_vis_.pose.pose.position.z));
    tf::Quaternion q(tracker_vis_.pose.pose.orientation.x,tracker_vis_.pose.pose.orientation.y,tracker_vis_.pose.pose.orientation.z,tracker_vis_.pose.pose.orientation.w);
    transform.setRotation(q);
    br_.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", "current"));
    // target_state
    target_state_msg_.px = current_target_state_.px;
    target_state_msg_.py = current_target_state_.py;
    target_state_msg_.pz = current_target_state_.pz;
    target_state_msg_.vx = current_target_state_.vx;
    target_state_msg_.vy = current_target_state_.vy;
    target_state_msg_.vz = current_target_state_.vz;
    // obstacle_state
    bpmp_tracker::ObjectState object_state_temp;
    obstacle_state_list_msg_.object_state_list.clear();
    geometry_msgs::Point obstacle_pts;
    // Fill obstacle_state_list_msg_ (required for controller)
    for (int i = 0; i < current_obstacle_state_list_.size(); i++) {
        object_state_temp.px = current_obstacle_state_list_[i].px;
        object_state_temp.py = current_obstacle_state_list_[i].py;
        object_state_temp.pz = current_obstacle_state_list_[i].pz;
        object_state_temp.vx = current_obstacle_state_list_[i].vx;
        object_state_temp.vy = current_obstacle_state_list_[i].vy;
        object_state_temp.vz = current_obstacle_state_list_[i].vz;
        obstacle_state_list_msg_.object_state_list.push_back(object_state_temp);
        /* OBSTACLE PATH VISUALIZATION (optional)
        obstacle_pts.x = object_state_temp.px;
        obstacle_pts.y = object_state_temp.py;
        obstacle_pts.z = object_state_temp.pz;
        obstacle_path_.markers[i].points.push_back(obstacle_pts);
        */
    }
    tracker_state_msg_.px = current_unicycle_state_.px;
    tracker_state_msg_.py = current_unicycle_state_.py;
    tracker_state_msg_.pz = current_unicycle_state_.pz;
    tracker_state_msg_.theta = current_unicycle_state_.theta;
    /* TARGET, ROBOT PATH VISUALIZATION
    geometry_msgs::PoseStamped temp_target_pose;
    temp_target_pose.header.frame_id = map_frame_id_;
    temp_target_pose.pose.position.x = current_target_state_.px;
    temp_target_pose.pose.position.y = current_target_state_.py;
    temp_target_pose.pose.position.z = current_target_state_.pz;
    temp_target_pose.pose.orientation.w = 1.0;
    temp_target_pose.pose.orientation.x = 0.0;
    temp_target_pose.pose.orientation.y = 0.0;
    temp_target_pose.pose.orientation.z = 0.0;
    target_path_.poses.push_back(temp_target_pose);

    geometry_msgs::PoseStamped  temp_robot_pose;
    temp_robot_pose.header.frame_id = map_frame_id_;
    temp_robot_pose.pose.position.x = current_unicycle_state_.px;
    temp_robot_pose.pose.position.y = current_unicycle_state_.py;
    temp_robot_pose.pose.position.z = current_unicycle_state_.pz;
    temp_robot_pose.pose.orientation.w = sin(current_unicycle_state_.theta);
    temp_robot_pose.pose.orientation.x = cos(current_unicycle_state_.theta);
    temp_robot_pose.pose.orientation.y = 0.0;
    temp_robot_pose.pose.orientation.z = 0.0;
    robot_path_.poses.push_back(temp_robot_pose);
    */
    gar_robot_.color.a = 1.0;
    gar_robot_.color.r = 0.0;
    gar_robot_.color.g = 0.0;
    gar_robot_.color.b = 1.0;
    gar_robot_.header.frame_id = map_frame_id_;
    gar_robot_.type = visualization_msgs::Marker::CYLINDER;
    gar_robot_.scale.x = 2.0*agent_size_;
    gar_robot_.scale.y = 2.0*agent_size_;
    gar_robot_.scale.z = 1.0;
    gar_robot_.pose.position.x = current_unicycle_state_.px;
    gar_robot_.pose.position.y = current_unicycle_state_.py;
    gar_robot_.pose.position.z = 0.5;
    gar_robot_.pose.orientation.w = 1.0;
    gar_robot_.pose.orientation.x = 0.0;
    gar_robot_.pose.orientation.y = 0.0;
    gar_robot_.pose.orientation.z = 0.0;

}

void bpmp::Simulator::PublishRosMsgs() {
    tracker_vis_publisher_.publish(tracker_vis_);
    obstacle_list_vis_publisher_.publish(obstacle_list_vis_);
    target_vis_publisher_.publish(target_vis_);
    if(is_unstructured_){
        pcl_publisher_.publish(point_cloud_);
        pcl_boxes_vis_publisher_.publish(pcl_boxes_vis_);
    }
    tracker_state_publisher_.publish(tracker_state_msg_);
    obstacle_state_list_publisher_.publish(obstacle_state_list_msg_);
    target_state_publisher_.publish(target_state_msg_);
    target_path_publisher_.publish(target_path_);
    robot_path_publisher_.publish(robot_path_);
    gar_robot_publisher_.publish(gar_robot_);
    obstacle_path_list_publisher_.publish(obstacle_path_);

    std::vector<bpmp::AffineCoeff2D> fov_constraints = GetFOVConstraints();
    Vec3List vertices = GetFeasibleVertices(fov_constraints);
    fov_publisher_.publish(VisualizeConvexHull(vertices));


}

void bpmp::Simulator::control_input_callback(const bpmp_tracker::ControlInput &msg) {
    tracker_control_input.px = msg.px;
    tracker_control_input.py = msg.py;
    tracker_control_input.pz = msg.pz;
    tracker_control_input.vx = msg.vx;
    tracker_control_input.vy = msg.vy;
    tracker_control_input.vz = msg.vz;
    tracker_control_input.ax = msg.ax;
    tracker_control_input.ay = msg.ay;
    tracker_control_input.az = msg.az;
}

void bpmp::Simulator::ReadObstacleConfiguration() {
    std::ifstream obstacle_file;
    if(obstacle_configuration_file_name_.c_str()==""){
        cout<<"NO PCL FILE"<<endl;
        return;
    }
    else
        cout<<"YES PCL FILE"<<endl;
    obstacle_file.open(obstacle_configuration_file_name_.c_str());
    double v1[2]{0.0, 0.0};
    double v2[2]{0.0, 0.0};
    double min_x, min_y, max_x, max_y;
    double p1[2], p2[2], p3[2], p4[2];
    int num_point;
    /*
     *  p1 --------p2
     *  -          -
     *  -          -
     *  p3---------p4
     */
    pcl::PointXYZ temp_point;

    visualization_msgs::Marker single_obstacle;
    single_obstacle.pose.orientation.w = 1.0;
    single_obstacle.pose.orientation.x = 0.0;
    single_obstacle.pose.orientation.y = 0.0;
    single_obstacle.pose.orientation.z = 0.0;
    single_obstacle.color.a = 1.0;
    single_obstacle.color.r = 0.0;
    single_obstacle.color.g = 0.0;
    single_obstacle.color.b = 0.0;
    single_obstacle.ns = "PCL_OBSTACLE";
    single_obstacle.header.frame_id = map_frame_id_;
    single_obstacle.type = visualization_msgs::Marker::CUBE;
    int id = 0;
    if (obstacle_file.is_open()) {
        while (obstacle_file >> v1[0] >> v1[1] >> v2[0] >> v2[1]) {
            if (v1[0] <= v2[0]) {
                min_x = v1[0];
                max_x = v2[0];
            } else {
                min_x = v2[0];
                max_x = v1[0];
            }
            if (v1[1] <= v2[1]) {
                min_y = v1[1];
                max_y = v2[1];
            } else {
                min_y = v2[1];
                max_y = v1[1];
            }
            p1[0] = min_x - inflation_size_;
            p1[1] = max_y + inflation_size_;
            p2[0] = max_x + inflation_size_;
            p2[1] = max_y + inflation_size_;
            p3[0] = min_x - inflation_size_;
            p3[1] = min_y - inflation_size_;
            p4[0] = max_x + inflation_size_;
            p4[1] = min_y - inflation_size_;
            // p1 - p2
            num_point = floor((p2[0] - p1[0]) / point_resolution_ + 0.5);
            for (int i = 0; i <= num_point; i++) {
                temp_point.x = float(p1[0] + (p2[0] - p1[0]) / num_point * i);
                temp_point.y = float(p1[1] + (p2[1] - p1[1]) / num_point * i);
                temp_point.z = 1.0;
                point_cloud_.push_back(temp_point);
            }
            //p1 - p3
            num_point = floor((p1[1] - p3[1]) / point_resolution_ + 0.5);
            for (int i = 0; i <= num_point; i++) {
                temp_point.x = float(p1[0] + (p3[0] - p1[0]) / num_point * i);
                temp_point.y = float(p1[1] + (p3[1] - p1[1]) / num_point * i);
                temp_point.z = 1.0;
                point_cloud_.push_back(temp_point);
            }
            // p3 - p4
            num_point = floor((p4[0] - p3[0]) / point_resolution_ + 0.5);
            for (int i = 0; i <= num_point; i++) {
                temp_point.x = float(p3[0] + (p4[0] - p3[0]) / num_point * i);
                temp_point.y = float(p3[1] + (p4[1] - p3[1]) / num_point * i);
                temp_point.z = 1.0;
                point_cloud_.push_back(temp_point);
            }
            // p4 - p2
            num_point = floor((p2[1] - p4[1]) / point_resolution_ + 0.5);
            for (int i = 0; i <= num_point; i++) {
                temp_point.x = float(p4[0] + (p2[0] - p4[0]) / num_point * i);
                temp_point.y = float(p4[1] + (p2[1] - p4[1]) / num_point * i);
                temp_point.z = 1.0;
                point_cloud_.push_back(temp_point);
            }
            single_obstacle.id = id++;
            single_obstacle.pose.position.x = 0.25 * (p1[0] + p2[0] + p3[0] + p4[0]);
            single_obstacle.pose.position.y = 0.25 * (p1[1] + p2[1] + p3[1] + p4[1]);
            single_obstacle.pose.position.z = 1.0;
            single_obstacle.scale.x = p2[0] - p1[0];
            single_obstacle.scale.y = p1[1] - p3[1];
            single_obstacle.scale.z = 2.0;
            pcl_boxes_vis_.markers.push_back(single_obstacle);
        }
    }
    obstacle_file.close();
}

void bpmp::Simulator::unicycle_input_callback(const bpmp_tracker::UnicycleInput &msg) {
    unicycle_control_input_.linear_speed = msg.vel_linear;
    unicycle_control_input_.angular_speed = msg.vel_angular;
}

void bpmp::Simulator::ShuffleScenario() {
    vector<int> index_array = GenerateUniqueRandomArray(moving_obstacle_number_ + 1, 0, object_number_-1);
    target_idx_ = index_array.back();
    index_array.pop_back();
    obstacle_idx_list_.clear();
    obstacle_idx_list_ = index_array;
}

std::vector<int> bpmp::Simulator::GenerateUniqueRandomArray(int size, int lower_bound, int upper_bound) {
    // Ensure that the range is large enough to have unique numbers
    if (upper_bound - lower_bound + 1 < size) {
        throw std::invalid_argument("Range too small to generate unique numbers of this size.");
    }
    // Create a vector containing all possible numbers within the range
    std::vector<int> allNumbers;
    for (int i = lower_bound; i <= upper_bound; ++i) {
        allNumbers.push_back(i);
    }
    // Initialize a random number generator
    std::mt19937 rng(static_cast<unsigned int>(std::time(nullptr)));
    // Shuffle the numbers
    std::shuffle(allNumbers.begin(), allNumbers.end(), rng);
    // Take the first 'size' elements from the shuffled vector
    std::vector<int> randomArray(allNumbers.begin(), allNumbers.begin() + size);
    return randomArray;
}

Vec3List bpmp::Simulator::GetFeasibleVertices(const std::vector<bpmp::AffineCoeff2D> &constraint) {
    Vec3List vertices;

    if(constraint.empty())
        return vertices;
    bpmp::AffineCoeff3D single_constraint;
    vector<bpmp::AffineCoeff3D> constraint_list;

    for(int idx=0;idx<constraint.size();idx++){
        single_constraint[0] = constraint[idx][0];
        single_constraint[1] = constraint[idx][1];
        single_constraint[2] = 0.0;
        single_constraint[3] = constraint[idx][2];
        constraint_list.push_back(single_constraint);
    }
    vector<bpmp::AffineCoeff3D> world_boundary_constraint = GetHalfSpaceFromBoundary();
    for(int i =0;i<world_boundary_constraint.size();i++)
        constraint_list.push_back(world_boundary_constraint[i]);

    Eigen::Matrix3Xd vertices_mtx;
    Eigen::MatrixX4d hs_mtx(constraint_list.size(), 4);
    int row = 0;
    for(const auto& h: constraint_list){
        // h = {x | n \dot (x - p) - d >= 0}
        // h1*x + h2*y +h3*z+ h4 <=0
        hs_mtx(row, 0) = h[0];
        hs_mtx(row, 1) = h[1];
        hs_mtx(row, 2) = h[2];
        hs_mtx(row, 3) = h[3];
        row++;
    }
    if(not geo_utils::enumerateVs(hs_mtx,vertices_mtx)){
        bool success = geo_utils::enumerateVs(hs_mtx,vertices_mtx);
        if(not success){
            std::cout<<"[Constraints]: Fail to find feasible vertices!"<<std::endl;
            return vertices;
        }
    }
    for(int i=0;i<vertices_mtx.cols();i++)
        vertices.emplace_back(vertices_mtx.col(i)(0),vertices_mtx.col(i)(1),vertices_mtx.col(i)(2));

    return vertices;
}

std::vector<bpmp::AffineCoeff3D> bpmp::Simulator::GetHalfSpaceFromBoundary() {
    vector<bpmp::AffineCoeff3D> boundary_halfspaces;
    bpmp::AffineCoeff3D half_space;
    {   // x-limit
        half_space[0] = -1.0;
        half_space[1] = 0.0;
        half_space[2] = 0.0;
        half_space[3] = -20.0;
        boundary_halfspaces.push_back(half_space);
        half_space[0] = 1.0;
        half_space[1] = 0.0;
        half_space[2] = 0.0;
        half_space[3] =-20.0;
        boundary_halfspaces.push_back(half_space);
    }
    {   // y-limit
        half_space[0] = 0.0;
        half_space[1] = -1.0;
        half_space[2] = 0.0;
        half_space[3] = -20.0;
        boundary_halfspaces.push_back(half_space);
        half_space[0] = 0.0;
        half_space[1] = 1.0;
        half_space[2] = 0.0;
        half_space[3] = -20.0;
        boundary_halfspaces.push_back(half_space);
    }
    {   // z-limit
        half_space[0] = 0.0;
        half_space[1] = 0.0;
        half_space[2] = -1.0;
        half_space[3] = 0;
        boundary_halfspaces.push_back(half_space);
        half_space[0] = 0.0;
        half_space[1] = 0.0;
        half_space[2] = 1.0;
        half_space[3] = -2.0;
        boundary_halfspaces.push_back(half_space);
    }
    return boundary_halfspaces;
}

visualization_msgs::Marker bpmp::Simulator::VisualizeConvexHull(const Vec3List &convex_hull) {
    if(convex_hull.empty())
        return visualization_msgs::Marker{};

    size_t num_vertices = convex_hull.size();
    ch_vertex *vertices;
    vertices = (ch_vertex *) malloc(num_vertices * sizeof(ch_vertex));
    for (size_t i = 0; i < num_vertices; i++) {
        vertices[i].x = convex_hull[i].x();
        vertices[i].y = convex_hull[i].y();
        vertices[i].z = convex_hull[i].z();
    }
    int *face_indices = nullptr;
    int num_faces;
    convhull_3d_build(vertices, num_vertices, &face_indices, &num_faces);

    PointMsg vertex;
    fov_vis_.colors.clear();
    fov_vis_.points.clear();
    fov_vis_.id =0;
    fov_vis_.ns="FOV";

    ColorMsg cell_color;
    cell_color.a = 0.2;
    cell_color.r = 1.0;
    cell_color.g = 1.0;
    cell_color.b = 0.0;
    for (int i = 0; i < num_faces; i++) {
        auto vertex1 = vertices[face_indices[i * 3]];
        auto vertex2 = vertices[face_indices[i * 3 + 1]];
        auto vertex3 = vertices[face_indices[i * 3 + 2]];
        vertex.x = vertex1.x, vertex.y = vertex1.y, vertex.z = vertex1.z;
        fov_vis_.points.emplace_back(vertex);
        vertex.x = vertex2.x, vertex.y = vertex2.y, vertex.z = vertex2.z;
        fov_vis_.points.emplace_back(vertex);
        vertex.x = vertex3.x, vertex.y = vertex3.y, vertex.z = vertex3.z;
        fov_vis_.points.emplace_back(vertex);
        fov_vis_.colors.push_back(cell_color);
    }

    free(vertices);
    free(face_indices);
    return fov_vis_;
}

std::vector<bpmp::AffineCoeff2D> bpmp::Simulator::GetFOVConstraints() {
    std::vector<bpmp::AffineCoeff2D> constraint;
    double cos_plus, sin_plus;
    double cos_minus, sin_minus;
    cos_plus = cos(current_unicycle_state_.theta+M_PI*0.33333333);
    cos_minus = cos(current_unicycle_state_.theta-M_PI*0.33333333);
    sin_plus = sin(current_unicycle_state_.theta+M_PI*0.33333333);
    sin_minus = sin(current_unicycle_state_.theta-M_PI*0.33333333);
    // FIRST
    AffineCoeff2D first;
    first[0] = -sin_plus, first[1] = cos_plus, first[2] = current_unicycle_state_.px*sin_plus - current_unicycle_state_.py*cos_plus;
    constraint.push_back(first);
    // SECOND
    AffineCoeff2D  second;
    second[0] = sin_minus, second[1] = -cos_minus, second[2] = -current_unicycle_state_.px*sin_minus+current_unicycle_state_.py*cos_minus;
    constraint.push_back(second);
    return constraint;
}
