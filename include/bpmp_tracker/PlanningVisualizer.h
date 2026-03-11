//
// Created by larr-laptop on 25. 6. 9.
//
#ifndef BPMP_TRACKER_PLANNINGVISUALIZER_H
#define BPMP_TRACKER_PLANNINGVISUALIZER_H
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <string>
#include <bpmp_utils/Utils.h>
#include <bpmp_utils/BernsteinUtils.h>

typedef geometry_msgs::Point PointMsg;
typedef Eigen::Vector3d Vec3;
typedef std::vector<Eigen::Vector3d> Vec3List;



namespace bpmp{
    struct VisualizationParam{
        std::string frame_id;
        std::string tracker_frame_id;
        struct{
            bool publish{false};
            int num_time_sample{10};
            double proportion{0.0};
            double line_scale{0.01};
            double color_a{0.0};
            double color_r{0.0};
            double color_g{0.0};
            double color_b{0.0};
        }raw_primitives;
        struct{
            bool publish{false};
            int num_time_sample{10};
            double proportion{0.0};
            double line_scale{0.01};
            double color_a{0.0};
            double color_r{0.0};
            double color_g{0.0};
            double color_b{0.0};
        }feasible_primitives;
        struct{
            int num_time_sample{10};
            double line_scale{0.01};
            double color_a{0.0};
            double color_r{0.0};
            double color_g{0.0};
            double color_b{0.0};
        }best_primitive;
        struct{
            struct{
                double x;
                double y;
                double z;
            }axis_min;
            struct{
                double x;
                double y;
                double z;
            }axis_max;
            struct{
                double color_a{0.0};
                double color_r{0.0};
                double color_g{0.0};
                double color_b{0.0};
            }visibility;
            struct{
                double color_a{0.0};
                double color_r{0.0};
                double color_g{0.0};
                double color_b{0.0};
            }voronoi;
        }cell;
    };
    class PlanningVisualizer{
    public:
        PlanningVisualizer(const bpmp::VisualizationParam & vis_param);
        void UpdateParameter(const bpmp::VisualizationParam & param);

        visualization_msgs::MarkerArray VisualizeRawPrimitives(const vector<bpmp::PrimitivePlanning> & primitive);
        visualization_msgs::MarkerArray VisualizeFeasiblePrimitives(const vector<bpmp::PrimitivePlanning> &primitive, const vector<bpmp::uint> &feasible_index);
        visualization_msgs::MarkerArray VisualizeBestPrimitive(const vector<bpmp::PrimitivePlanning>&primitive, const uint & best_index);

    private:
        visualization_msgs::Marker raw_primitive_;
        visualization_msgs::Marker feasible_primitive_;
        visualization_msgs::Marker best_primitive_;
        VisualizationParam param_;
        vector<AffineCoeff3D> GetHalfSpaceFromBoundary();   // [xmin,ymin,zmin] ~[xmax,ymax,zmax]

    };
}
#endif //BPMP_TRACKER_PLANNINGVISUALIZER_H