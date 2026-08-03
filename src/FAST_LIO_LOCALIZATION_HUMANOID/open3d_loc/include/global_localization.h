#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/static_transform_broadcaster.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/float32.hpp>
#include <pct_scan_navigation/msg/localization_status.hpp>
#include <pct_scan_navigation/srv/load_localization_map.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <open3d/Open3D.h>
#include <atomic>
#include <deque>
#include <cmath>
#include <string>

class GloabalLocalization : public rclcpp::Node
{
private:
    /* data */
public:
    GloabalLocalization();
    ~GloabalLocalization();

    /// @brief 初始化定位
    bool LocalizationInitialize();

    /// @brief 订阅FAST-LIO发布的imu_link在odom下的里程计
    void CallbackImulink2Odom(const nav_msgs::msg::Odometry::SharedPtr imulink2odom);
    /// @brief 订阅FAST-LIO发布的imu_link点云，先转换成base_link，再转换成odom用于定位匹配
    void CallbackScanBody(const sensor_msgs::msg::PointCloud2::SharedPtr scan_in_imu_link);

    /// @brief 订阅在初始位姿
    void CallbackInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr initialpose);

    void StartLoc();

    void Localization();

    /// @brief 欧拉角转mat3x3
    /// @param euler
    /// @return
    Eigen::Matrix3d Euler2Matrix3d(const Eigen::Vector3d euler);

    /// @brief compute 3d distance between two points
    /// @param a
    /// @param b
    /// @return 距离值
    double ComputeMotionDis(const Eigen::Vector3d &a, const Eigen::Vector3d &b);

private:
    using LoadLocalizationMap = pct_scan_navigation::srv::LoadLocalizationMap;
    using LocalizationStatus = pct_scan_navigation::msg::LocalizationStatus;

    bool LoadMapFromPath(const std::string &path_map, const std::string &map_name, std::string *message);
    void PublishLocalizationStatus();
    void SetLocalizationStatus(uint8_t state, const std::string &reason);
    void HandleLoadMap(const std::shared_ptr<LoadLocalizationMap::Request> request,
                       std::shared_ptr<LoadLocalizationMap::Response> response);

    rclcpp::CallbackGroup::SharedPtr state_callback_group_;
    rclcpp::CallbackGroup::SharedPtr scan_callback_group_;

    /// 命名约定: Alink2Blink 表示 A_link 在 B_link 下的位姿, 即 T_B_A.
    /// @brief 订阅imulink2odom,即fast_lio的里程计信息
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_imulink2odom_;

    /// @brief 订阅当前帧imu_link点云
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_scan_cur_;

    /// @brief 订阅初始位姿
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr sub_initialpose_;

    /// @brief base_link在odom下的位姿, T_odom_base
    Eigen::Matrix4d mat_baselink2odom_;
    /// @brief odom在map下的位姿, T_map_odom
    Eigen::Matrix4d mat_odom2map_;
    /// @brief base_link在map下的位姿, T_map_base = mat_odom2map * mat_baselink2odom
    Eigen::Matrix4d mat_baselink2map_;
    /// @brief initialpose初始位姿
    Eigen::Matrix4d mat_initialpose_;

    std::mutex lock_mat_odom2map_;

    /// @brief motion_link在base_link下的位姿, T_base_motion
    Eigen::Matrix4d mat_motionlink2baselink_;

    /// @brief imu_link在base_link下的位姿, T_base_imu
    Eigen::Matrix4d mat_imulink2baselink_;

    /// @brief 初始位姿, x, y, z, roll, pitch, yaw (单位:度degrees)
    std::vector<double> initialpose_;

    /// @brief 原始地图点云
    std::shared_ptr<open3d::geometry::PointCloud> pcd_map_ori_;
    std::shared_ptr<open3d::geometry::PointCloud> pcd_map_fine_;
    std::shared_ptr<open3d::geometry::PointCloud> pcd_scan_cur_;
    std::mutex lock_map_;

    std::deque<std::shared_ptr<open3d::geometry::PointCloud>> que_pcd_scan_;
    int queue_maxsize_;
    double voxelsize_coarse_;
    double voxel_downsample_size_ = 0.1;
    double icp_distance_threshold_ = 0.15;
    double fitness_eval_threshold_ = 0.15;
    double normal_search_radius_ = 0.4;
    double max_icp_translation_ = 0.3;
    double max_icp_yaw_deg_ = 1.0;
    double max_init_icp_translation_ = 2.0;
    double max_init_icp_yaw_deg_ = 15.0;
    double min_init_fitness_improvement_ = 0.02;
    double scan_map_filter_radius_ = 0.0;
    int localization_lost_fail_count_ = 3;
    int tracking_fail_count_ = 0;
    int min_source_points_ = 2500;
    int min_target_points_ = 50000;

    /// @brief 定位配准fitness(overlap)阈值
    double threshold_fitness_;
    /// @brief 配准fitness(overlap)阈值
    double threshold_fitness_init_;

    std::thread thread_loc_;
    std::mutex lock_scan_;
    std::atomic_bool flag_exit_{false};

    rclcpp::Time timestamp_odom_;
    std::mutex lock_timestamp_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_map_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_scan_base_link_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_scan_map_;
    rclcpp::TimerBase::SharedPtr map_publish_timer_;
    sensor_msgs::msg::PointCloud2 map_msg_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_localization_3d_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_localization_3d_confidence_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_localization_3d_delay_ms_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_open3d_odometry_;
    rclcpp::Publisher<LocalizationStatus>::SharedPtr pub_localization_status_;
    rclcpp::TimerBase::SharedPtr localization_status_timer_;
    rclcpp::Service<LoadLocalizationMap>::SharedPtr load_map_srv_;

    geometry_msgs::msg::PoseStamped localization_3d_;
    std_msgs::msg::Float32 localization_3d_confidence_;
    std_msgs::msg::Float32 localization_3d_delay_ms_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> br_odom2map_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;

    std::string current_map_name_;
    std::string current_map_path_;
    std::atomic<uint8_t> localization_state_{LocalizationStatus::UNINITIALIZED};
    std::string localization_reason_{"startup"};
    std::mutex lock_localization_status_;

    /// @brief 定位频率(定位间隔时间，多少秒1次)
    double loc_frequence_;

    /// @brief 初始化成功标志
    std::atomic_bool loc_initialized_{false};

    /// @brief 手动给定 initialpose 后，请求定位线程重新执行初始化 ICP
    std::atomic_bool relocalization_requested_{false};

    /// @brief 当前定位overlap，confidence
    std::atomic<double> loc_fitness_{0.0};

    /// 1202
    /// @brief 上次更新定位时的定位值
    Eigen::Vector3d last_loc_;
    // Eigen::Vector3d cur_loc_;
    /// @brief 更新地图子图的距离,超过则更新地图子图
    double dis_updatemap_;
    bool last_open3d_odom_valid_ = false;
    rclcpp::Time last_open3d_odom_stamp_;
    double last_open3d_odom_x_ = 0.0;
    double last_open3d_odom_y_ = 0.0;
    double last_open3d_odom_yaw_ = 0.0;

};
