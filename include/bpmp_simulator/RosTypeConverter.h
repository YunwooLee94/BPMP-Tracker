//
// Created by larr-laptop on 25. 4. 15.
//
#pragma once
#ifndef BPMP_TRACKER_ROSTYPECONVERTER_H
#define BPMP_TRACKER_ROSTYPECONVERTER_H
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h> // Target
#include <nav_msgs/Odometry.h> // Robot
#include <bpmp_tracker/RobotState.h>
#include <bpmp_tracker/ObjectState.h>
#include <bpmp_tracker/ObjectStateList.h>
#include <geometry_msgs/Twist.h>
#include <bpmp_tracker/UnicycleInput.h>
#include <vector>
#include <array>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/message_traits.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


namespace bpmp{
    struct TargetVelocity{
        double vx;
        double vy;
    };
    class RosTypeConverter{
    public:
        RosTypeConverter();
        void Run();
    private:
        ros::NodeHandle nh_;
        ros::NodeHandle pnh_;
        ros::Subscriber TargetPositionSubscriber_;
        ros::Subscriber RobotOdometrySuscriber_;
        ros::Subscriber PclSubscriber_;
        ros::Publisher TargetStatePublisher_;
        ros::Publisher PclPublisher_;
        ros::Subscriber UnicycleInputSubscriber_;
        ros::Publisher UnicycleInputPublisher_;
        ros::Publisher RobotOdometryPublisher_;
        std::vector<std::string> dynamic_topics_;
        ros::Publisher DynamicObstaclesPublisher_;

        struct ObstacleTracker {
            bpmp_tracker::ObjectState curr;   // 현재 추정 상태 
            bpmp_tracker::ObjectState prev;   // 이전 상태(속도 계산용)
            std::array<TargetVelocity,5> vel_hist{}; // 단순 이동평균(5샘플)
            int  count{0};      // warm-up 5회까지
            int  iter{0};       // vel_hist 인덱스
            double t_last{0.0}; // 마지막 갱신 curTime()
            bool received{false};
        };
        std::array<ObstacleTracker,10> dyn_;     // 10개 장애물 버퍼
        
        // 누적 경로/마커 퍼블리셔
        ros::Publisher TargetPathPublisher_;
        ros::Publisher RobotPathPublisher_;
        ros::Publisher DynamicPathsPublisher_;
        ros::Publisher TargetPathMarkerPublisher_;
        ros::Publisher RobotPathMarkerPublisher_;
        ros::Publisher DynamicObstaclesMarkerPublisher_;

        // 누적 경로/마커 데이터
        nav_msgs::Path target_path_;
        nav_msgs::Path robot_path_;
        std::array<visualization_msgs::Marker,10> dyn_path_markers_;
        visualization_msgs::Marker target_path_marker_;
        visualization_msgs::Marker robot_path_marker_;
        // 동적장애물 궤적 페이드 표현을 위한 포인트별 타임스탬프
        std::array<std::vector<ros::Time>,10> dyn_path_times_;
        // 오래된 포인트 알파 페이드 시간창 (초)
        double dyn_path_fade_window_sec_{5.0};
        // 타겟/로봇 경로 포인트 타임스탬프 (페이드용)
        std::vector<ros::Time> target_path_times_;
        std::vector<ros::Time> robot_path_times_;


        using Policy5 = message_filters::sync_policies::ApproximateTime<
        geometry_msgs::PoseStamped, geometry_msgs::PoseStamped, geometry_msgs::PoseStamped,
        geometry_msgs::PoseStamped, geometry_msgs::PoseStamped>;

        std::unique_ptr< message_filters::Subscriber<geometry_msgs::PoseStamped> > subDyn_[10];
        std::unique_ptr< message_filters::Synchronizer<Policy5> > syncA_;
        std::unique_ptr< message_filters::Synchronizer<Policy5> > syncB_;

        // 그룹별 최신 묶음 보관 후 병합
        std::vector<bpmp_tracker::ObjectState> last_groupA_;
        std::vector<bpmp_tracker::ObjectState> last_groupB_;
        ros::Time last_stamp_A_;
        ros::Time last_stamp_B_;

        // 동기화 파라미터
        double sync_slop_sec_{0.1};   // 그룹 A/B 간 병합 허용 시간차(초)
        int queue_size_{10};
        double dynamic_z_offset_{0.0};

        void UpdateObstacleFromPose(int idx, const geometry_msgs::PoseStampedConstPtr& msg);

        // 그룹 콜백 & 병합 함수
        void DynGroupACb(const geometry_msgs::PoseStampedConstPtr& m0,
            const geometry_msgs::PoseStampedConstPtr& m1,
            const geometry_msgs::PoseStampedConstPtr& m2,
            const geometry_msgs::PoseStampedConstPtr& m3,
            const geometry_msgs::PoseStampedConstPtr& m4);
        void DynGroupBCb(const geometry_msgs::PoseStampedConstPtr& m5,
            const geometry_msgs::PoseStampedConstPtr& m6,
            const geometry_msgs::PoseStampedConstPtr& m7,
            const geometry_msgs::PoseStampedConstPtr& m8,
            const geometry_msgs::PoseStampedConstPtr& m9);

        void TryMergeAndPublish();
        double t0_;
        double t0_history_;
        int odom_count_=0;
        int odom_count_iter_ =0;
        tf::TransformBroadcaster br_;
        double curTime() {return (ros::Time::now().toSec()-t0_);};
        bool is_target_position_received_{false};
        bool is_unicycle_state_received_{false};
        void TargtPositionCallback(const geometry_msgs::PoseStampedConstPtr &msg);
        void PclCallback(const pcl::PointCloud<pcl::PointXYZ> &msg);
        void UnicycleInputCallback(const bpmp_tracker::UnicycleInput &msg);
        sensor_msgs::PointCloud2 pcl_output_;
        void RobotOdometryCallback(const nav_msgs::OdometryConstPtr &msg);
        bpmp_tracker::ObjectState current_target_state_;
        bpmp_tracker::ObjectState previous_target_state_;
        std::vector<TargetVelocity> vel_history_;
        void Publish();
        tf::TransformListener tf_listener_;
        double speed_log_period_{1.0};
    };
};

#endif //BPMP_TRACKER_ROSTYPECONVERTER_H
