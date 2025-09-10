//
// Created by larr-laptop on 25. 6. 9.
//
#define CONVHULL_3D_ENABLE
#include <bpmp_tracker/PlanningVisualizer.h>
visualization_msgs::MarkerArray
bpmp::PlanningVisualizer::VisualizeRawPrimitives(const vector<bpmp::PrimitivePlanning> & primitive) {
    visualization_msgs::MarkerArray visual_output;
    if (not param_.raw_primitives.publish)
        return visual_output;
    if (primitive.empty())
        return visual_output;
    raw_primitive_.ns= "TRACKER_RAW";
    std::vector<double> time_seq;
    double seg_t0 = primitive[0].t0;
    double seg_tf = primitive[0].tf;
    for (int i = 0; i < param_.raw_primitives.num_time_sample; i++)
        time_seq.push_back(seg_t0 + (double) i * (seg_tf - seg_t0) /
                                    (double) (param_.raw_primitives.num_time_sample - 1));
    geometry_msgs::Point temp_point;
    int num_primitive_vis = (int) ((double) primitive.size() * param_.raw_primitives.proportion);
    int sample_step = primitive.size() / num_primitive_vis;
    double ctrl_pts_x[4];
    double ctrl_pts_y[4];
    double ctrl_pts_z[4];
    for (int idx = 0; idx < num_primitive_vis - 1; idx++) {
        raw_primitive_.points.clear();
        raw_primitive_.id = idx;
        for (int i = 0; i < 4; i++) {
            ctrl_pts_x[i] = primitive[sample_step * idx].ctrl_x[i];
            ctrl_pts_y[i] = primitive[sample_step * idx].ctrl_y[i];
            ctrl_pts_z[i] = primitive[sample_step * idx].ctrl_z[i];
        }
        for (int i = 0; i < time_seq.size(); i++) {
            temp_point.x =
                    getBernsteinValue(ctrl_pts_x, time_seq[i], primitive[sample_step * idx].t0,
                                      primitive[sample_step * idx].tf, 3);
            temp_point.y =
                    getBernsteinValue(ctrl_pts_y, time_seq[i], primitive[sample_step * idx].t0,
                                      primitive[sample_step * idx].tf, 3);
            temp_point.z =
                    getBernsteinValue(ctrl_pts_z, time_seq[i], primitive[sample_step * idx].t0,
                                      primitive[sample_step * idx].tf, 3);
            raw_primitive_.points.push_back(temp_point);
        }
        visual_output.markers.push_back(raw_primitive_);
    }


    return visual_output;
}

visualization_msgs::MarkerArray
bpmp::PlanningVisualizer::VisualizeFeasiblePrimitives(const vector<bpmp::PrimitivePlanning> &primitive, const vector<bpmp::uint> &feasible_index) {
    visualization_msgs::MarkerArray visual_output;
    if (not param_.feasible_primitives.publish)
        return visual_output;
    if (primitive.empty())
        return visual_output;
    if (feasible_index.empty())
        return visual_output;
    feasible_primitive_.ns = "TRACKER_FEASIBLE";
    std::vector<double> time_seq;
    double seg_t0 = primitive[0].t0;
    double seg_tf = primitive[0].tf;
    for (int i = 0; i < param_.feasible_primitives.num_time_sample; i++)
        time_seq.push_back(seg_t0 + (double)i * (seg_tf - seg_t0) /
                                    (double)(param_.feasible_primitives.num_time_sample - 1));
    geometry_msgs::Point temp_point;
    int num_primitive_vis =
            (int)((double)feasible_index.size() * param_.feasible_primitives.proportion);
    int sample_step;
    if(num_primitive_vis!=0)
        sample_step =  feasible_index.size() / num_primitive_vis;
    else
        num_primitive_vis = 1;

    double ctrl_pts_x[4];
    double ctrl_pts_y[4];
    double ctrl_pts_z[4];
    for (int idx = 0; idx < num_primitive_vis - 1; idx++) {
        feasible_primitive_.points.clear();
        feasible_primitive_.id = idx;
        for (int i = 0; i < 4; i++) {
            ctrl_pts_x[i] = primitive[feasible_index[sample_step * idx]].ctrl_x[i];
            ctrl_pts_y[i] = primitive[feasible_index[sample_step * idx]].ctrl_y[i];
            ctrl_pts_z[i] = primitive[feasible_index[sample_step * idx]].ctrl_z[i];
        }
        for (int i = 0; i < time_seq.size(); i++) {
            temp_point.x = getBernsteinValue(
                    ctrl_pts_x, time_seq[i], primitive[feasible_index[sample_step * idx]].t0,
                    primitive[sample_step * idx].tf, 3);
            temp_point.y = getBernsteinValue(
                    ctrl_pts_y, time_seq[i], primitive[feasible_index[sample_step * idx]].t0,
                    primitive[sample_step * idx].tf, 3);
            temp_point.z = getBernsteinValue(
                    ctrl_pts_z, time_seq[i], primitive[feasible_index[sample_step * idx]].t0,
                    primitive[sample_step * idx].tf, 3);
            feasible_primitive_.points.push_back(temp_point);
        }
        visual_output.markers.push_back(feasible_primitive_);
    }
    { // ERASE
        static int last_num_id =2000;
        static bool once_flag = true;

        visualization_msgs::Marker erase_marker;
        erase_marker.action = visualization_msgs::Marker::DELETE;
        erase_marker.ns = "TRACKER_FEASIBLE";
        erase_marker.header.frame_id = param_.frame_id;
        for (int i = num_primitive_vis - 1; i <= last_num_id; i++) {
            erase_marker.id = i;
            visual_output.markers.push_back(erase_marker);
        }
        last_num_id = num_primitive_vis - 1;
    }
    return visual_output;
}

visualization_msgs::MarkerArray
bpmp::PlanningVisualizer::VisualizeBestPrimitive(const vector<bpmp::PrimitivePlanning> &primitive, const uint& best_index) {
    if(primitive.empty())
        return visualization_msgs::MarkerArray{};

    visualization_msgs::MarkerArray visual_output;
    std::vector<double> time_seq;
    double seg_t0 = primitive[0].t0;
    double seg_tf = primitive[0].tf;
    for (int i = 0; i < param_.best_primitive.num_time_sample; i++)
        time_seq.push_back(seg_t0 + (double)i * (seg_tf - seg_t0) /
                                    (double)(param_.best_primitive.num_time_sample - 1));
    geometry_msgs::Point temp_point;
    double ctrl_pts_x[4];
    double ctrl_pts_y[4];
    double ctrl_pts_z[4];
    best_primitive_.points.clear();
    best_primitive_.ns="TRACKER_BEST";
    best_primitive_.id = 0;
    for (int i = 0; i < 4; i++) {
        ctrl_pts_x[i] = primitive[best_index].ctrl_x[i];
        ctrl_pts_y[i] = primitive[best_index].ctrl_y[i];
        ctrl_pts_z[i] = primitive[best_index].ctrl_z[i];
    }
    for (int i = 0; i < time_seq.size(); i++) {
        temp_point.x = getBernsteinValue(
                ctrl_pts_x, time_seq[i], primitive[best_index].t0,
                primitive[best_index].tf, 3);
        temp_point.y = getBernsteinValue(
                ctrl_pts_y, time_seq[i], primitive[best_index].t0,
                primitive[best_index].tf, 3);
        temp_point.z = getBernsteinValue(
                ctrl_pts_z, time_seq[i], primitive[best_index].t0,
                primitive[best_index].tf, 3);
        best_primitive_.points.push_back(temp_point);
    }
    visual_output.markers.push_back(best_primitive_);
    return visual_output;
}


void bpmp::PlanningVisualizer::UpdateParameter(const bpmp::VisualizationParam &param) {
    param_ = param;
    // RAW PRIMITIVE
    raw_primitive_.header.frame_id = "current"; // TODO: have to be parameterized
    raw_primitive_.type = visualization_msgs::Marker::LINE_STRIP;
    raw_primitive_.scale.x = param_.raw_primitives.line_scale;
    raw_primitive_.ns = "TRACKER_RAW";
    raw_primitive_.color.a = (float) param_.raw_primitives.color_a;
    raw_primitive_.color.r = (float) param_.raw_primitives.color_r;
    raw_primitive_.color.g = (float) param_.raw_primitives.color_g;
    raw_primitive_.color.b = (float) param_.raw_primitives.color_b;
    raw_primitive_.pose.orientation.w = 1.0;
    raw_primitive_.pose.orientation.x = 0.0;
    raw_primitive_.pose.orientation.y = 0.0;
    raw_primitive_.pose.orientation.z = 0.0;
    // FEASIBLE PRIMITIVE
    feasible_primitive_.header.frame_id = "current";
    feasible_primitive_.type = visualization_msgs::Marker::LINE_STRIP;
    feasible_primitive_.scale.x = param_.feasible_primitives.line_scale;
    feasible_primitive_.ns = "TRACKER_FEASIBLE";
    feasible_primitive_.color.a = (float) param_.feasible_primitives.color_a;
    feasible_primitive_.color.r = (float) param_.feasible_primitives.color_r;
    feasible_primitive_.color.g = (float) param_.feasible_primitives.color_g;
    feasible_primitive_.color.b = (float) param_.feasible_primitives.color_b;
    feasible_primitive_.pose.orientation.w = 1.0;
    feasible_primitive_.pose.orientation.x = 0.0;
    feasible_primitive_.pose.orientation.y = 0.0;
    feasible_primitive_.pose.orientation.z = 0.0;
    // BEST PRIMITIVE
    best_primitive_.header.frame_id = "current";
    best_primitive_.type = visualization_msgs::Marker::LINE_STRIP;
    best_primitive_.scale.x = param_.best_primitive.line_scale;
    best_primitive_.ns = "TRACKER_BEST";
    best_primitive_.color.a = (float) param_.best_primitive.color_a;
    best_primitive_.color.r = (float) param_.best_primitive.color_r;
    best_primitive_.color.g = (float) param_.best_primitive.color_g;
    best_primitive_.color.b = (float) param_.best_primitive.color_b;
    best_primitive_.pose.orientation.w = 1.0;
    best_primitive_.pose.orientation.x = 0.0;
    best_primitive_.pose.orientation.y = 0.0;
    best_primitive_.pose.orientation.z = 0.0;
}

bpmp::PlanningVisualizer::PlanningVisualizer(const bpmp::VisualizationParam &vis_param) {
    param_ = vis_param;
    // RAW PRIMITIVE
    raw_primitive_.header.frame_id = "current";
    raw_primitive_.type = visualization_msgs::Marker::LINE_STRIP;
    raw_primitive_.scale.x = param_.raw_primitives.line_scale;
    raw_primitive_.ns = "TRACKER_RAW";
    raw_primitive_.color.a = (float) param_.raw_primitives.color_a;
    raw_primitive_.color.r = (float) param_.raw_primitives.color_r;
    raw_primitive_.color.g = (float) param_.raw_primitives.color_g;
    raw_primitive_.color.b = (float) param_.raw_primitives.color_b;
    raw_primitive_.pose.orientation.w = 1.0;
    raw_primitive_.pose.orientation.x = 0.0;
    raw_primitive_.pose.orientation.y = 0.0;
    raw_primitive_.pose.orientation.z = 0.0;
    // FEASIBLE PRIMITIVE
    feasible_primitive_.header.frame_id = "current";
    feasible_primitive_.type = visualization_msgs::Marker::LINE_STRIP;
    feasible_primitive_.scale.x = param_.feasible_primitives.line_scale;
    feasible_primitive_.ns = "TRACKER_FEASIBLE";
    feasible_primitive_.color.a = (float) param_.feasible_primitives.color_a;
    feasible_primitive_.color.r = (float) param_.feasible_primitives.color_r;
    feasible_primitive_.color.g = (float) param_.feasible_primitives.color_g;
    feasible_primitive_.color.b = (float) param_.feasible_primitives.color_b;
    feasible_primitive_.pose.orientation.w = 1.0;
    feasible_primitive_.pose.orientation.x = 0.0;
    feasible_primitive_.pose.orientation.y = 0.0;
    feasible_primitive_.pose.orientation.z = 0.0;
    // BEST PRIMITIVE
    best_primitive_.header.frame_id = "current";
    best_primitive_.type = visualization_msgs::Marker::LINE_STRIP;
    best_primitive_.scale.x = param_.best_primitive.line_scale;
    best_primitive_.ns = "TRACKER_BEST";
    best_primitive_.color.a = (float) param_.best_primitive.color_a;
    best_primitive_.color.r = (float) param_.best_primitive.color_r;
    best_primitive_.color.g = (float) param_.best_primitive.color_g;
    best_primitive_.color.b = (float) param_.best_primitive.color_b;
    best_primitive_.pose.orientation.w = 1.0;
    best_primitive_.pose.orientation.x = 0.0;
    best_primitive_.pose.orientation.y = 0.0;
    best_primitive_.pose.orientation.z = 0.0;


}

std::vector<bpmp::AffineCoeff3D> bpmp::PlanningVisualizer::GetHalfSpaceFromBoundary() {
    vector<bpmp::AffineCoeff3D> boundary_halfspaces;
    bpmp::AffineCoeff3D half_space;
    {   // x-limit
        half_space[0] = -1.0;
        half_space[1] = 0.0;
        half_space[2] = 0.0;
        half_space[3] = param_.cell.axis_min.x;
        boundary_halfspaces.push_back(half_space);
        half_space[0] = 1.0;
        half_space[1] = 0.0;
        half_space[2] = 0.0;
        half_space[3] = -param_.cell.axis_max.x;
        boundary_halfspaces.push_back(half_space);
    }
    {   // y-limit
        half_space[0] = 0.0;
        half_space[1] = -1.0;
        half_space[2] = 0.0;
        half_space[3] = param_.cell.axis_min.y;
        boundary_halfspaces.push_back(half_space);
        half_space[0] = 0.0;
        half_space[1] = 1.0;
        half_space[2] = 0.0;
        half_space[3] = -param_.cell.axis_max.y;
        boundary_halfspaces.push_back(half_space);
    }
    {   // z-limit
        half_space[0] = 0.0;
        half_space[1] = 0.0;
        half_space[2] = -1.0;
        half_space[3] = param_.cell.axis_min.z;
        boundary_halfspaces.push_back(half_space);
        half_space[0] = 0.0;
        half_space[1] = 0.0;
        half_space[2] = 1.0;
        half_space[3] = -param_.cell.axis_max.z;
        boundary_halfspaces.push_back(half_space);
    }
    return boundary_halfspaces;
}
