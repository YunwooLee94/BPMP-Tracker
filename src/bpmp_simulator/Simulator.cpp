#include <bpmp_simulator/Simulator.h>
void bpmp::Simulator::Run() {
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
bpmp::Simulator::Simulator() : nh_("~") {
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
        nh_.getParam("obstacle_idx_list", object_idx_list_);
    } else
        ROS_ERROR("Failed to find 'obstacle list'.");

    // Target Index
    nh_.param<int>("target_idx", target_idx_, 0);
    target_idx_ = 0; // Analysis
    object_idx_list_.push_back(target_idx_);
    object_number_ = (int) object_idx_list_.size(); // obstacles + target
    nh_.param<string>("initial_state_file_name", initial_state_file_name_, "");
    nh_.param<string>("object_history_file_name", object_history_file_name_, "");
    // unstructured file name
    nh_.param<string>("obstacle_configuration_file_name", obstacle_configuration_file_name_, "");
    nh_.param<int>("moving_obstacle_number", moving_obstacle_number_, 0);

    obstacle_vis_.header.frame_id = map_frame_id_;
    target_vis_.header.frame_id = map_frame_id_;

    tracker_vis_.header.frame_id = map_frame_id_;
    tracker_vis_.type = visualization_msgs::Marker::CYLINDER;
    tracker_vis_.ns = "Tracker";
    tracker_vis_.color.a = 1.0;
    tracker_vis_.color.r = 0.0;
    tracker_vis_.color.g = 0.0;
    tracker_vis_.color.b = 1.0;
    tracker_vis_.scale.x = 2 * agent_size_;
    tracker_vis_.scale.y = 2 * agent_size_;
    tracker_vis_.scale.z = 2.0;
    tracker_vis_.pose.orientation.w = 1.0;
    tracker_vis_.pose.orientation.x = 0.0;
    tracker_vis_.pose.orientation.y = 0.0;
    tracker_vis_.pose.orientation.z = 0.0;

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

    point_cloud_.header.frame_id = map_frame_id_;

    target_vis_publisher_ = nh_.advertise<visualization_msgs::Marker>("target_vis", 1);
    obstacle_list_vis_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("obstacle_list_vis", 1);
    tracker_vis_publisher_ = nh_.advertise<visualization_msgs::Marker>("tracker_vis", 1);

    obstacle_state_list_publisher_ = nh_.advertise<bpmp_tracker::ObjectStateList>("obstacle_state_list", 1);
    target_state_publisher_ = nh_.advertise<bpmp_tracker::ObjectState>("target_state", 1);
    tracker_state_publisher_ = nh_.advertise<bpmp_tracker::ObjectState>("tracker_state_list", 1);

    pcl_publisher_ = nh_.advertise<pcl::PointCloud<pcl::PointXYZ>>("point_cloud_obstacle", 1);
    pcl_boxes_vis_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("pcl_boxes", 1);


    control_input_subscriber_ = nh_.subscribe("/bpmp_tracker/tracker_control_input", 1,
                                              &Simulator::control_input_callback, this);
    ReadInitialTrackerStateList();
    ReadObjectTrajectory();
    if (is_unstructured_)
        ReadObstacleConfiguration();
}

void bpmp::Simulator::ReadObjectTrajectory() {
    ifstream object_trajectory_file;
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
            object_history_list_[i].t.push_back(stod(content[j][object_idx_list_[i] * num_read_unit + 1]));   // t
            object_history_list_[i].px.push_back(stod(content[j][object_idx_list_[i] * num_read_unit + 2]));   // px
            object_history_list_[i].py.push_back(stod(content[j][object_idx_list_[i] * num_read_unit + 3]));   // py
            object_history_list_[i].pz.push_back(stod(content[j][object_idx_list_[i] * num_read_unit + 4]));   // pz
            object_history_list_[i].vx.push_back(stod(content[j][object_idx_list_[i] * num_read_unit + 5]));   // vx
            object_history_list_[i].vy.push_back(stod(content[j][object_idx_list_[i] * num_read_unit + 6]));   // vy
            object_history_list_[i].vz.push_back(stod(content[j][object_idx_list_[i] * num_read_unit + 7]));   // vz
        }
    }
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

    current_tracker_state_.px = current_tracker_state_.px + current_tracker_state_.vx * dt;
    current_tracker_state_.py = current_tracker_state_.py + current_tracker_state_.vy * dt;
    current_tracker_state_.pz = current_tracker_state_.pz + current_tracker_state_.vz * dt;
    current_tracker_state_.vx = current_tracker_state_.vx + tracker_control_input.ax * dt;
    current_tracker_state_.vy = current_tracker_state_.vy + tracker_control_input.ay * dt;
    current_tracker_state_.vz = current_tracker_state_.vz + tracker_control_input.az * dt;

    t_prev = t;
}

void bpmp::Simulator::ReadInitialTrackerStateList() {
    std::ifstream initial_state_file;
    initial_state_file.open(initial_state_file_name_.c_str());
    State tracker_state;
    if (initial_state_file.is_open())
        initial_state_file>>current_tracker_state_.px >>current_tracker_state_.py >>current_tracker_state_.pz;
    initial_state_file.close();
    tracker_control_input.px =0.0, tracker_control_input.py =0.0,tracker_control_input.pz =0.0;
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
    // Tracker Visualization
    tracker_vis_.id = 0;
    tracker_vis_.pose.position.x = current_tracker_state_.px;
    tracker_vis_.pose.position.y = current_tracker_state_.py;
    tracker_vis_.pose.position.z = current_tracker_state_.pz;
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
    for (int i = 0; i < current_obstacle_state_list_.size(); i++) {
        object_state_temp.px = current_obstacle_state_list_[i].px;
        object_state_temp.py = current_obstacle_state_list_[i].py;
        object_state_temp.pz = current_obstacle_state_list_[i].pz;
        object_state_temp.vx = current_obstacle_state_list_[i].vx;
        object_state_temp.vy = current_obstacle_state_list_[i].vy;
        object_state_temp.vz = current_obstacle_state_list_[i].vz;
        obstacle_state_list_msg_.object_state_list.push_back(object_state_temp);
    }
    tracker_state_msg_.px = current_target_state_.px;
    tracker_state_msg_.py = current_target_state_.py;
    tracker_state_msg_.pz = current_target_state_.pz;
    tracker_state_msg_.vx = current_target_state_.px;
    tracker_state_msg_.vy = current_target_state_.py;
    tracker_state_msg_.vz = current_target_state_.pz;
}

void bpmp::Simulator::PublishRosMsgs() {
    tracker_vis_publisher_.publish(tracker_vis_);
    obstacle_list_vis_publisher_.publish(obstacle_list_vis_);
    target_vis_publisher_.publish(target_vis_);
    pcl_publisher_.publish(point_cloud_);
    pcl_boxes_vis_publisher_.publish(pcl_boxes_vis_);

    tracker_state_publisher_.publish(tracker_state_msg_);
    obstacle_state_list_publisher_.publish(obstacle_state_list_msg_);
    target_state_publisher_.publish(target_state_msg_);
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