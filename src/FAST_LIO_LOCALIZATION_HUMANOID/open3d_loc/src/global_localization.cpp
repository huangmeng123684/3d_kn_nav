#include "open3d_registration/open3d_registration.h"
#include "open3d_conversions/open3d_conversions.h"
#include "global_localization.h"

#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <fstream>
#include <functional>
#include <sstream>
#include <stdexcept>

namespace
{
bool TransformFloat3Fields(sensor_msgs::msg::PointCloud2 &cloud,
                           const Eigen::Matrix3d &rotation,
                           const Eigen::Vector3d &translation,
                           const std::string &x_name,
                           const std::string &y_name,
                           const std::string &z_name,
                           bool translate)
{
    bool has_x = false;
    bool has_y = false;
    bool has_z = false;
    for (const auto &field : cloud.fields)
    {
        if (field.datatype != sensor_msgs::msg::PointField::FLOAT32)
        {
            continue;
        }
        has_x = has_x || field.name == x_name;
        has_y = has_y || field.name == y_name;
        has_z = has_z || field.name == z_name;
    }
    if (!has_x || !has_y || !has_z)
    {
        return false;
    }

    sensor_msgs::PointCloud2Iterator<float> x(cloud, x_name);
    sensor_msgs::PointCloud2Iterator<float> y(cloud, y_name);
    sensor_msgs::PointCloud2Iterator<float> z(cloud, z_name);
    for (; x != x.end(); ++x, ++y, ++z)
    {
        Eigen::Vector3d point(*x, *y, *z);
        point = rotation * point;
        if (translate)
        {
            point += translation;
        }
        *x = static_cast<float>(point.x());
        *y = static_cast<float>(point.y());
        *z = static_cast<float>(point.z());
    }
    return true;
}

void FilterNearOrigin(sensor_msgs::msg::PointCloud2 &cloud, double filter_radius)
{
    if (filter_radius <= 0.0)
    {
        return;
    }

    const double radius2 = filter_radius * filter_radius;
    std::vector<uint8_t> filtered;
    filtered.reserve(cloud.data.size());

    sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
    for (size_t point_offset = 0; x != x.end(); ++x, ++y, ++z, point_offset += cloud.point_step)
    {
        const double distance2 =
            static_cast<double>(*x) * static_cast<double>(*x) +
            static_cast<double>(*y) * static_cast<double>(*y) +
            static_cast<double>(*z) * static_cast<double>(*z);
        if (std::isfinite(distance2) && distance2 < radius2)
        {
            continue;
        }
        filtered.insert(filtered.end(),
                        cloud.data.begin() + point_offset,
                        cloud.data.begin() + point_offset + cloud.point_step);
    }

    cloud.height = 1;
    cloud.width = static_cast<uint32_t>(filtered.size() / cloud.point_step);
    cloud.row_step = cloud.width * cloud.point_step;
    cloud.data = std::move(filtered);
    cloud.is_dense = false;
}
} // namespace

bool GloabalLocalization::LoadMapFromPath(const std::string &path_map,
                                          const std::string &map_name,
                                          std::string *message)
{
    if (path_map.empty())
    {
        if (message)
            *message = "path_map is empty";
        return false;
    }

    auto map_ori = std::make_shared<open3d::geometry::PointCloud>();
    if (!open3d::io::ReadPointCloud(path_map, *map_ori) || map_ori->IsEmpty())
    {
        if (message)
            *message = "read map from path failed: " + path_map;
        return false;
    }

    auto map_coarse = map_ori->VoxelDownSample(voxelsize_coarse_);
    map_coarse->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(voxelsize_coarse_ * 2, 30));
    if (!map_coarse->HasColors())
    {
        map_coarse->PaintUniformColor({1, 0, 0});
    }

    sensor_msgs::msg::PointCloud2 next_map_msg;
    open3d_conversions::open3dToRos(*map_coarse, next_map_msg);
    next_map_msg.header.frame_id = "map";
    next_map_msg.header.stamp = this->now();

    auto map_fine = map_ori->VoxelDownSample(voxel_downsample_size_);
    map_fine->colors_.clear();
    map_fine->EstimateNormals(open3d::geometry::KDTreeSearchParamHybrid(normal_search_radius_, 30));

    std::string resolved_map_name = map_name;
    if (resolved_map_name.empty())
    {
        const auto slash = path_map.find_last_of("/\\");
        const std::string filename =
            slash == std::string::npos ? path_map : path_map.substr(slash + 1);
        const auto dot = filename.find_last_of('.');
        resolved_map_name = dot == std::string::npos ? filename : filename.substr(0, dot);
    }

    {
        std::lock_guard<std::mutex> map_lock(lock_map_);
        pcd_map_fine_ = map_fine;
        map_msg_ = next_map_msg;
        current_map_path_ = path_map;
    }
    {
        std::lock_guard<std::mutex> status_lock(lock_localization_status_);
        current_map_name_ = resolved_map_name;
    }

    {
        std::lock_guard<std::mutex> scan_lock(lock_scan_);
        que_pcd_scan_.clear();
        pcd_scan_cur_.reset(new open3d::geometry::PointCloud);
    }
    last_loc_ = Eigen::Vector3d(0, 0, -5000);
    tracking_fail_count_ = 0;
    loc_fitness_.store(0.0);
    pub_map_->publish(next_map_msg);

    if (message)
        *message = "loaded map: " + path_map;
    return true;
}

void GloabalLocalization::PublishLocalizationStatus()
{
    LocalizationStatus msg;
    msg.header.stamp = this->now();
    msg.header.frame_id = "map";
    msg.state = localization_state_.load();
    msg.fitness = static_cast<float>(loc_fitness_.load());
    {
        std::lock_guard<std::mutex> status_lock(lock_localization_status_);
        msg.map_name = current_map_name_;
        msg.reason = localization_reason_;
    }
    pub_localization_status_->publish(msg);
}

void GloabalLocalization::SetLocalizationStatus(uint8_t state, const std::string &reason)
{
    bool changed = localization_state_.load() != state;
    localization_state_.store(state);
    {
        std::lock_guard<std::mutex> status_lock(lock_localization_status_);
        changed = changed || localization_reason_ != reason;
        localization_reason_ = reason;
    }
    if (changed)
        PublishLocalizationStatus();
}

void GloabalLocalization::HandleLoadMap(
    const std::shared_ptr<LoadLocalizationMap::Request> request,
    std::shared_ptr<LoadLocalizationMap::Response> response)
{
    SetLocalizationStatus(LocalizationStatus::MAP_SWITCHING, "map_switching");

    const double old_fitness_eval_threshold = fitness_eval_threshold_;
    const double old_threshold_fitness = threshold_fitness_;
    const double old_threshold_fitness_init = threshold_fitness_init_;
    if (request->use_localization_thresholds)
    {
        if (request->fitness_eval_threshold > 0.0)
            fitness_eval_threshold_ = request->fitness_eval_threshold;
        if (request->threshold_fitness > 0.0)
            threshold_fitness_ = request->threshold_fitness;
        if (request->threshold_fitness_init > 0.0)
            threshold_fitness_init_ = request->threshold_fitness_init;
    }

    std::string message;
    if (!LoadMapFromPath(request->pcd_path, request->map_name, &message))
    {
        fitness_eval_threshold_ = old_fitness_eval_threshold;
        threshold_fitness_ = old_threshold_fitness;
        threshold_fitness_init_ = old_threshold_fitness_init;
        response->success = false;
        response->message = message;
        SetLocalizationStatus(LocalizationStatus::UNINITIALIZED, "map_load_failed");
        return;
    }

    loc_initialized_.store(false);
    relocalization_requested_.store(false);
    {
        std::lock_guard<std::mutex> state_lock(lock_mat_odom2map_);
        mat_odom2map_ = Eigen::Matrix4d::Identity();
    }
    response->success = true;
    response->message = message;
    SetLocalizationStatus(LocalizationStatus::UNINITIALIZED, "map_loaded");
}

GloabalLocalization::GloabalLocalization() : Node("global_loc_node")
{

    flag_exit_.store(false);
    loc_initialized_.store(false);
    mat_baselink2odom_ = Eigen::Matrix4d::Identity();
    mat_odom2map_ = Eigen::Matrix4d::Identity();
    mat_baselink2map_ = Eigen::Matrix4d::Identity();
    mat_initialpose_ = Eigen::Matrix4d::Identity();
    mat_motionlink2baselink_ = Eigen::Matrix4d::Identity();
    mat_imulink2baselink_ = Eigen::Matrix4d::Identity();
    last_loc_ = Eigen::Vector3d(0, 0, -5000);

    pcd_map_ori_.reset(new open3d::geometry::PointCloud);
    pcd_scan_cur_.reset(new open3d::geometry::PointCloud);
    pcd_map_fine_.reset(new open3d::geometry::PointCloud);
    queue_maxsize_ = 5;

    rclcpp::QoS map_qos(rclcpp::KeepLast(1));
    map_qos.reliable();
    map_qos.transient_local();
    pub_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/map_3d", map_qos);
    rclcpp::QoS scan_base_link_qos(rclcpp::KeepLast(1));
    scan_base_link_qos.reliable();
    scan_base_link_qos.durability_volatile();
    pub_scan_base_link_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/scan_base_link", scan_base_link_qos);
    pub_scan_map_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/scan_map", scan_base_link_qos);
    pub_localization_3d_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/localization_3d", 1);
    pub_localization_3d_confidence_ = this->create_publisher<std_msgs::msg::Float32>("/localization_3d_confidence", 1);
    pub_localization_3d_delay_ms_ = this->create_publisher<std_msgs::msg::Float32>("/localization_3d_delay_ms", 1);
    pub_open3d_odometry_ = this->create_publisher<nav_msgs::msg::Odometry>("/Odometry_open3d", 20);
    pub_localization_status_ = this->create_publisher<LocalizationStatus>("/localization_status", 10);
    localization_status_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(250),
        std::bind(&GloabalLocalization::PublishLocalizationStatus, this));
    load_map_srv_ = this->create_service<LoadLocalizationMap>(
        "~/load_map",
        std::bind(&GloabalLocalization::HandleLoadMap, this,
                  std::placeholders::_1, std::placeholders::_2));

    loc_frequence_ = 2.0; //
    loc_fitness_.store(0.0);

    state_callback_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    scan_callback_group_ =
        this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    rclcpp::SubscriptionOptions state_subscription_options;
    state_subscription_options.callback_group = state_callback_group_;
    rclcpp::SubscriptionOptions scan_subscription_options;
    scan_subscription_options.callback_group = scan_callback_group_;

    // 注册回调函数
    sub_imulink2odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/Odometry_loc", 50,
        std::bind(&GloabalLocalization::CallbackImulink2Odom, this, std::placeholders::_1),
        state_subscription_options);
    sub_scan_cur_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud_registered_body_1", 50,
        std::bind(&GloabalLocalization::CallbackScanBody, this, std::placeholders::_1),
        scan_subscription_options);
    sub_initialpose_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
        "/initialpose", 50,
        std::bind(&GloabalLocalization::CallbackInitialPose, this, std::placeholders::_1),
        state_subscription_options);

    // 队列最大数量
    this->declare_parameter<int>("pcd_queue_maxsize", 5);

    // 定位间隔时间
    this->declare_parameter<double>("loc_frequence", 2.0);

    // voxelsize
    this->declare_parameter<double>("voxelsize_coarse", 0.2);
    this->declare_parameter<double>("voxel_downsample_size", 0.1);
    this->declare_parameter<double>("icp_distance_threshold", 0.15);
    this->declare_parameter<double>("fitness_eval_threshold", 0.15);
    this->declare_parameter<double>("normal_search_radius", 0.4);
    this->declare_parameter<double>("max_icp_translation", 0.3);
    this->declare_parameter<double>("max_icp_yaw_deg", 1.0);
    this->declare_parameter<double>("max_init_icp_translation", 2.0);
    this->declare_parameter<double>("max_init_icp_yaw_deg", 15.0);
    this->declare_parameter<double>("min_init_fitness_improvement", 0.02);
    this->declare_parameter<double>("scan_map_filter_radius", 0.0);
    this->declare_parameter<int>("localization_lost_fail_count", 3);
    this->declare_parameter<int>("min_source_points", 2500);
    this->declare_parameter<int>("min_target_points", 50000);
    this->declare_parameter<double>("threshold_fitness_init", 0.9);
    this->declare_parameter<double>("threshold_fitness", 0.9);
    this->declare_parameter<std::vector<double>>("initialpose", std::vector<double>());
    this->declare_parameter<double>("dis_updatemap", 1);
    this->declare_parameter<double>("map_publish_interval", 2.0);
    this->declare_parameter<std::string>("path_imu_to_base", "");
    this->declare_parameter<std::string>("map_name", "");

    this->get_parameter("pcd_queue_maxsize", queue_maxsize_);
    if (queue_maxsize_ < 1)
    {
        RCLCPP_WARN(this->get_logger(), "pcd_queue_maxsize=%d is invalid, use 1", queue_maxsize_);
        queue_maxsize_ = 1;
    }
    this->get_parameter("loc_frequence", loc_frequence_);
    this->get_parameter("voxelsize_coarse", voxelsize_coarse_);
    this->get_parameter("voxel_downsample_size", voxel_downsample_size_);
    this->get_parameter("icp_distance_threshold", icp_distance_threshold_);
    this->get_parameter("fitness_eval_threshold", fitness_eval_threshold_);
    this->get_parameter("normal_search_radius", normal_search_radius_);
    this->get_parameter("max_icp_translation", max_icp_translation_);
    this->get_parameter("max_icp_yaw_deg", max_icp_yaw_deg_);
    this->get_parameter("max_init_icp_translation", max_init_icp_translation_);
    this->get_parameter("max_init_icp_yaw_deg", max_init_icp_yaw_deg_);
    this->get_parameter("min_init_fitness_improvement", min_init_fitness_improvement_);
    this->get_parameter("scan_map_filter_radius", scan_map_filter_radius_);
    this->get_parameter("localization_lost_fail_count", localization_lost_fail_count_);
    this->get_parameter("min_source_points", min_source_points_);
    this->get_parameter("min_target_points", min_target_points_);
    this->get_parameter("threshold_fitness_init", threshold_fitness_init_);
    this->get_parameter("threshold_fitness", threshold_fitness_);
    this->get_parameter("initialpose", initialpose_);
    this->get_parameter("dis_updatemap", dis_updatemap_);
    std::string path_imu_to_base = "";
    this->get_parameter("path_imu_to_base", path_imu_to_base);
    this->get_parameter("map_name", current_map_name_);
    double map_publish_interval = 2.0;
    this->get_parameter("map_publish_interval", map_publish_interval);

    RCLCPP_INFO(this->get_logger(),
                "registration params: voxelsize_coarse=%.3f, voxel_downsample_size=%.3f, "
                "icp_distance_threshold=%.3f, fitness_eval_threshold=%.3f, "
                "normal_search_radius=%.3f, threshold_fitness=%.3f, threshold_fitness_init=%.3f, "
                "max_icp_translation=%.3f, max_icp_yaw_deg=%.3f, max_init_icp_translation=%.3f, "
                "max_init_icp_yaw_deg=%.3f, min_init_fitness_improvement=%.3f, scan_map_filter_radius=%.3f, "
                "min_source_points=%d, min_target_points=%d",
                voxelsize_coarse_, voxel_downsample_size_, icp_distance_threshold_,
                fitness_eval_threshold_, normal_search_radius_, threshold_fitness_, threshold_fitness_init_,
                max_icp_translation_, max_icp_yaw_deg_, max_init_icp_translation_, max_init_icp_yaw_deg_,
                min_init_fitness_improvement_, scan_map_filter_radius_, min_source_points_, min_target_points_);

    if (initialpose_.size() != 6)
    {
        RCLCPP_WARN(this->get_logger(),
                    "invalid initialpose parameter size=%zu, expected 6 values [x,y,z,roll,pitch,yaw], use identity",
                    initialpose_.size());
        initialpose_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    }
    RCLCPP_INFO(this->get_logger(),
                "initialpose param: xyz=(%.3f, %.3f, %.3f), rpy_deg=(%.3f, %.3f, %.3f)",
                initialpose_[0], initialpose_[1], initialpose_[2],
                initialpose_[3], initialpose_[4], initialpose_[5]);
    mat_initialpose_.block<3, 3>(0, 0) = Euler2Matrix3d(Eigen::Vector3d(initialpose_[3], initialpose_[4], initialpose_[5]));
    mat_initialpose_.block<3, 1>(0, 3) = Eigen::Vector3d(initialpose_[0], initialpose_[1], initialpose_[2]);

    // 读取地图
    RCLCPP_INFO(this->get_logger(), "开始读取点云地图");
    std::string path_map = "";
    this->declare_parameter<std::string>("path_map", "");
    this->get_parameter("path_map", path_map);
    std::string map_load_message;
    if (!LoadMapFromPath(path_map, current_map_name_, &map_load_message))
    {
        RCLCPP_ERROR(this->get_logger(), "%s", map_load_message.c_str());
        rclcpp::shutdown();
    }
    if (map_publish_interval > 0.0)
    {
        auto map_publish_period = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(map_publish_interval));
        map_publish_timer_ = this->create_wall_timer(
            map_publish_period,
            [this]()
            {
                std::lock_guard<std::mutex> map_lock(lock_map_);
                map_msg_.header.stamp = this->now();
                pub_map_->publish(map_msg_);
            });
        RCLCPP_INFO(this->get_logger(),
                    "map publisher uses transient_local QoS and republish interval %.3f s",
                    map_publish_interval);
    }

    static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    auto publish_static_tf_from_matrix =
        [this](const std::string &parent_frame,
               const std::string &child_frame,
               const Eigen::Matrix4d &matrix,
               const std::string &source_name)
    {
        Eigen::Quaterniond quat(matrix.block<3, 3>(0, 0));
        if (!std::isfinite(quat.norm()) || quat.norm() < 1e-6)
        {
            RCLCPP_WARN(this->get_logger(),
                        "invalid quaternion when publishing static tf %s -> %s from %s, use identity rotation",
                        parent_frame.c_str(), child_frame.c_str(), source_name.c_str());
            quat = Eigen::Quaterniond::Identity();
        }
        else
        {
            quat.normalize();
        }

        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = parent_frame;
        transform.child_frame_id = child_frame;
        transform.transform.translation.x = matrix(0, 3);
        transform.transform.translation.y = matrix(1, 3);
        transform.transform.translation.z = matrix(2, 3);
        transform.transform.rotation.x = quat.x();
        transform.transform.rotation.y = quat.y();
        transform.transform.rotation.z = quat.z();
        transform.transform.rotation.w = quat.w();
        static_broadcaster_->sendTransform(transform);

        RCLCPP_INFO(this->get_logger(),
                    "publish static tf %s -> %s from %s: xyz=(%.3f, %.3f, %.3f), quat=(%.6f, %.6f, %.6f, %.6f)",
                    parent_frame.c_str(), child_frame.c_str(), source_name.c_str(),
                    matrix(0, 3), matrix(1, 3), matrix(2, 3),
                    quat.x(), quat.y(), quat.z(), quat.w());
    };

    auto publish_static_tf_from_param =
        [this](const std::string &param_name,
               const std::string &parent_frame,
               const std::string &child_frame,
               Eigen::Matrix4d *matrix_out,
               const std::function<void(const std::string &, const std::string &, const Eigen::Matrix4d &, const std::string &)> &publish_static_tf)
    {
        const std::vector<double> default_tf = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
        this->declare_parameter<std::vector<double>>(param_name, default_tf);
        std::vector<double> tf_param;
        this->get_parameter(param_name, tf_param);

        bool valid = tf_param.size() == 7;
        for (double value : tf_param)
        {
            valid = valid && std::isfinite(value);
        }
        if (!valid)
        {
            RCLCPP_WARN(this->get_logger(),
                        "invalid static tf param %s, expected [x,y,z,qx,qy,qz,qw], use identity",
                        param_name.c_str());
            tf_param = default_tf;
        }

        Eigen::Quaterniond quat(tf_param[6], tf_param[3], tf_param[4], tf_param[5]);
        if (!std::isfinite(quat.norm()) || quat.norm() < 1e-6)
        {
            RCLCPP_WARN(this->get_logger(),
                        "invalid quaternion in static tf param %s, use identity rotation",
                        param_name.c_str());
            quat = Eigen::Quaterniond::Identity();
        }
        else
        {
            quat.normalize();
        }

        Eigen::Matrix4d matrix = Eigen::Matrix4d::Identity();
        matrix.block<3, 3>(0, 0) = quat.toRotationMatrix();
        matrix.block<3, 1>(0, 3) = Eigen::Vector3d(tf_param[0], tf_param[1], tf_param[2]);
        if (matrix_out != nullptr)
        {
            *matrix_out = matrix;
        }

        publish_static_tf(parent_frame, child_frame, matrix, param_name);
    };

    auto load_imu_to_base_from_file =
        [this](const std::string &path, Eigen::Matrix4d &matrix_out)
    {
        if (path.empty())
        {
            return false;
        }

        std::ifstream file(path);
        if (!file.is_open())
        {
            RCLCPP_WARN(this->get_logger(), "failed to open path_imu_to_base: %s", path.c_str());
            return false;
        }

        std::string line;
        while (std::getline(file, line))
        {
            if (line.empty() || line[0] == '#')
            {
                continue;
            }

            int id = 0;
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;
            double qx = 0.0;
            double qy = 0.0;
            double qz = 0.0;
            double qw = 1.0;
            std::istringstream line_stream(line);
            if (!(line_stream >> id >> x >> y >> z >> qx >> qy >> qz >> qw))
            {
                RCLCPP_WARN(this->get_logger(),
                            "invalid path_imu_to_base line, expected: id x y z qx qy qz qw, line: %s",
                            line.c_str());
                continue;
            }

            Eigen::Quaterniond quat(qw, qx, qy, qz);
            if (!std::isfinite(quat.norm()) || quat.norm() < 1e-6)
            {
                RCLCPP_WARN(this->get_logger(),
                            "invalid quaternion in path_imu_to_base: %s", path.c_str());
                return false;
            }
            quat.normalize();

            matrix_out = Eigen::Matrix4d::Identity();
            matrix_out.block<3, 3>(0, 0) = quat.toRotationMatrix();
            matrix_out.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z);

            RCLCPP_INFO(this->get_logger(),
                        "loaded imu_to_base id=%d from %s: xyz=(%.3f, %.3f, %.3f), quat=(%.6f, %.6f, %.6f, %.6f)",
                        id, path.c_str(), x, y, z,
                        qx, qy, qz, qw);
            return true;
        }

        RCLCPP_WARN(this->get_logger(), "path_imu_to_base has no valid transform: %s", path.c_str());
        return false;
    };

    if (load_imu_to_base_from_file(path_imu_to_base, mat_imulink2baselink_))
    {
        publish_static_tf_from_matrix("base_link", "imu_link", mat_imulink2baselink_, "path_imu_to_base");
        RCLCPP_INFO(this->get_logger(),
                    "localization source: /cloud_registered_body_1 (imu_link) -> base_link -> odom before registration");
    }
    else
    {
        publish_static_tf_from_param("static_tf_imu_link_to_base_link", "base_link", "imu_link",
                                     &mat_imulink2baselink_, publish_static_tf_from_matrix);
        RCLCPP_WARN(this->get_logger(),
                    "path_imu_to_base is not loaded; use static_tf_imu_link_to_base_link for imu_link -> base_link scan transform");
    }
    publish_static_tf_from_param("static_tf_motion_link_to_base_link", "base_link", "motion_link",
                                 &mat_motionlink2baselink_, publish_static_tf_from_matrix);

    RCLCPP_WARN(this->get_logger(), "initialize finished");

    br_odom2map_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    StartLoc();
}

GloabalLocalization::~GloabalLocalization()
{
    flag_exit_.store(true);
    if (thread_loc_.joinable())
    {
        thread_loc_.join();
    }
}

Eigen::Matrix3d GloabalLocalization::Euler2Matrix3d(const Eigen::Vector3d euler)
{
    Eigen::Matrix3d mat3d;
    // convert degrees to radians
    auto eulerAngle = euler / 180 * M_PI;
    Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle[0], Eigen::Vector3d::UnitX()));
    Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle[1], Eigen::Vector3d::UnitY()));
    Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle[2], Eigen::Vector3d::UnitZ()));
    mat3d = rollAngle * pitchAngle * yawAngle;
    return mat3d;
}
void GloabalLocalization::CallbackImulink2Odom(const nav_msgs::msg::Odometry::SharedPtr imulink2odom)
{
    const rclcpp::Time output_stamp(imulink2odom->header.stamp);
    {
        std::lock_guard<std::mutex> timestamp_lock(lock_timestamp_);
        timestamp_odom_ = output_stamp;
    }
    Eigen::Isometry3d mat_current = Eigen::Isometry3d::Identity();
    tf2::fromMsg(imulink2odom->pose.pose, mat_current);
    auto mat_imulink2odom = mat_current.matrix();

    Eigen::Matrix4d mat_odom2map_snapshot = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d mat_baselink2odom_snapshot = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d mat_baselink2map_snapshot = Eigen::Matrix4d::Identity();
    {
        std::lock_guard<std::mutex> state_lock(lock_mat_odom2map_);
        mat_baselink2odom_ = mat_imulink2odom * mat_imulink2baselink_.inverse();
        mat_baselink2map_ = mat_odom2map_ * mat_baselink2odom_;
        mat_odom2map_snapshot = mat_odom2map_;
        mat_baselink2odom_snapshot = mat_baselink2odom_;
        mat_baselink2map_snapshot = mat_baselink2map_;
    }

    /// 发布tf关系
    geometry_msgs::msg::TransformStamped transform_odom2map;
    transform_odom2map.header.frame_id = "map";
    transform_odom2map.child_frame_id = "odom";
    transform_odom2map.header.stamp = output_stamp;
    transform_odom2map.transform.translation.x = mat_odom2map_snapshot(0, 3);
    transform_odom2map.transform.translation.y = mat_odom2map_snapshot(1, 3);
    transform_odom2map.transform.translation.z = mat_odom2map_snapshot(2, 3);
    Eigen::Quaterniond quat_odom2map(mat_odom2map_snapshot.block<3, 3>(0, 0));
    quat_odom2map.normalize();
    transform_odom2map.transform.rotation.x = quat_odom2map.x();
    transform_odom2map.transform.rotation.y = quat_odom2map.y();
    transform_odom2map.transform.rotation.z = quat_odom2map.z();
    transform_odom2map.transform.rotation.w = quat_odom2map.w();
    br_odom2map_->sendTransform(transform_odom2map);

    geometry_msgs::msg::TransformStamped transform_baselink2odom;
    transform_baselink2odom.header.frame_id = "odom";
    transform_baselink2odom.child_frame_id = "base_link";
    transform_baselink2odom.header.stamp = output_stamp;
    transform_baselink2odom.transform.translation.x = mat_baselink2odom_snapshot(0, 3);
    transform_baselink2odom.transform.translation.y = mat_baselink2odom_snapshot(1, 3);
    transform_baselink2odom.transform.translation.z = mat_baselink2odom_snapshot(2, 3);
    Eigen::Quaterniond quat_baselink2odom(mat_baselink2odom_snapshot.block<3, 3>(0, 0));
    quat_baselink2odom.normalize();
    transform_baselink2odom.transform.rotation.x = quat_baselink2odom.x();
    transform_baselink2odom.transform.rotation.y = quat_baselink2odom.y();
    transform_baselink2odom.transform.rotation.z = quat_baselink2odom.z();
    transform_baselink2odom.transform.rotation.w = quat_baselink2odom.w();
    br_odom2map_->sendTransform(transform_baselink2odom);

    const bool localization_ready = loc_initialized_.load();
    if (!localization_ready)
    {
        last_open3d_odom_valid_ = false;
    }

    if (localization_ready)
    {
        Eigen::Quaterniond open3d_quat(mat_baselink2map_snapshot.block<3, 3>(0, 0));
        open3d_quat.normalize();
        const double open3d_yaw =
            std::atan2(mat_baselink2map_snapshot(1, 0), mat_baselink2map_snapshot(0, 0));

        nav_msgs::msg::Odometry odom_open3d;
        odom_open3d.header.frame_id = "map";
        odom_open3d.header.stamp = output_stamp;
        odom_open3d.child_frame_id = "base_link";
        odom_open3d.pose.pose.position.x = mat_baselink2map_snapshot(0, 3);
        odom_open3d.pose.pose.position.y = mat_baselink2map_snapshot(1, 3);
        odom_open3d.pose.pose.position.z = mat_baselink2map_snapshot(2, 3);
        odom_open3d.pose.pose.orientation.x = open3d_quat.x();
        odom_open3d.pose.pose.orientation.y = open3d_quat.y();
        odom_open3d.pose.pose.orientation.z = open3d_quat.z();
        odom_open3d.pose.pose.orientation.w = open3d_quat.w();

        if (last_open3d_odom_valid_)
        {
            const double dt = (output_stamp - last_open3d_odom_stamp_).seconds();
            if (dt > 1e-3)
            {
                odom_open3d.twist.twist.linear.x =
                    (odom_open3d.pose.pose.position.x - last_open3d_odom_x_) / dt;
                odom_open3d.twist.twist.linear.y =
                    (odom_open3d.pose.pose.position.y - last_open3d_odom_y_) / dt;
                double dyaw = open3d_yaw - last_open3d_odom_yaw_;
                while (dyaw > M_PI) dyaw -= 2.0 * M_PI;
                while (dyaw < -M_PI) dyaw += 2.0 * M_PI;
                odom_open3d.twist.twist.angular.z = dyaw / dt;
            }
        }
        last_open3d_odom_valid_ = true;
        last_open3d_odom_stamp_ = output_stamp;
        last_open3d_odom_x_ = odom_open3d.pose.pose.position.x;
        last_open3d_odom_y_ = odom_open3d.pose.pose.position.y;
        last_open3d_odom_yaw_ = open3d_yaw;
        pub_open3d_odometry_->publish(odom_open3d);
    }

    /// 定位初始化完成后发布运动中心定位结果
    if (loc_initialized_.load())
    {
        Eigen::Matrix4d mat_motionlink2map = mat_baselink2map_snapshot * mat_motionlink2baselink_;
        Eigen::Isometry3d Isometry3d_motionlink2map;
        Isometry3d_motionlink2map.matrix() = mat_motionlink2map;

        localization_3d_confidence_.data = static_cast<float>(loc_fitness_.load());
        pub_localization_3d_confidence_->publish(localization_3d_confidence_);
        localization_3d_delay_ms_.data = (this->now() - output_stamp).seconds() * 1000.0;
        pub_localization_3d_delay_ms_->publish(localization_3d_delay_ms_);
        localization_3d_.header.frame_id = "map";
        localization_3d_.header.stamp = output_stamp;
        localization_3d_.pose = tf2::toMsg(Isometry3d_motionlink2map);
        pub_localization_3d_->publish(localization_3d_);
    }
}
void GloabalLocalization::CallbackScanBody(
    const sensor_msgs::msg::PointCloud2::SharedPtr scan_in_imu_link)
{
    bool has_odom = false;
    rclcpp::Time latest_odom_stamp;
    {
        std::lock_guard<std::mutex> timestamp_lock(lock_timestamp_);
        latest_odom_stamp = timestamp_odom_;
        has_odom = timestamp_odom_.seconds() != 0.0;
    }

    auto pcd_base_link = std::make_shared<open3d::geometry::PointCloud>();
    sensor_msgs::msg::PointCloud2::ConstSharedPtr const_scan_ptr = scan_in_imu_link;
    open3d_conversions::rosToOpen3d(const_scan_ptr, *pcd_base_link, true);

    // /cloud_registered_body_1 is expressed in imu_link. Convert it to base_link
    // first, so the map and scan share the same physical body frame convention.
    pcd_base_link->Transform(mat_imulink2baselink_);

    if (pub_scan_base_link_->get_subscription_count() > 0)
    {
        sensor_msgs::msg::PointCloud2 scan_base_link_msg;
        open3d_conversions::open3dToRos(*pcd_base_link, scan_base_link_msg, "base_link");
        scan_base_link_msg.header.stamp = has_odom ? latest_odom_stamp : this->now();
        pub_scan_base_link_->publish(scan_base_link_msg);
    }
    if (!has_odom)
    {
        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                              "skip body scan for localization before Odometry_loc is received");
        return;
    }

    Eigen::Matrix4d mat_baselink2odom_snapshot = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d mat_odom2map_snapshot = Eigen::Matrix4d::Identity();
    {
        std::lock_guard<std::mutex> state_lock(lock_mat_odom2map_);
        mat_baselink2odom_snapshot = mat_baselink2odom_;
        mat_odom2map_snapshot = mat_odom2map_;
    }

    if (pub_scan_map_->get_subscription_count() > 0)
    {
        sensor_msgs::msg::PointCloud2 scan_map = *scan_in_imu_link;
        scan_map.header.frame_id = "map";
        scan_map.header.stamp = latest_odom_stamp;

        const Eigen::Matrix4d mat_baselink2map = mat_odom2map_snapshot * mat_baselink2odom_snapshot;
        const Eigen::Matrix3d imu_to_base_rotation = mat_imulink2baselink_.block<3, 3>(0, 0);
        const Eigen::Vector3d imu_to_base_translation = mat_imulink2baselink_.block<3, 1>(0, 3);
        const Eigen::Matrix3d base_to_map_rotation = mat_baselink2map.block<3, 3>(0, 0);
        const Eigen::Vector3d base_to_map_translation = mat_baselink2map.block<3, 1>(0, 3);
        try
        {
            if (!TransformFloat3Fields(scan_map, imu_to_base_rotation, imu_to_base_translation, "x", "y", "z", true))
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "skip /scan_map publish: /cloud_registered_body_1 has no float32 x/y/z fields");
            }
            else
            {
                TransformFloat3Fields(scan_map, imu_to_base_rotation, Eigen::Vector3d::Zero(),
                                      "normal_x", "normal_y", "normal_z", false);
                FilterNearOrigin(scan_map, scan_map_filter_radius_);
                TransformFloat3Fields(scan_map, base_to_map_rotation, base_to_map_translation, "x", "y", "z", true);
                TransformFloat3Fields(scan_map, base_to_map_rotation, Eigen::Vector3d::Zero(),
                                      "normal_x", "normal_y", "normal_z", false);
                pub_scan_map_->publish(scan_map);
            }
        }
        catch (const std::runtime_error &error)
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "skip /scan_map publish: %s", error.what());
        }
    }

    auto pcd_received = std::make_shared<open3d::geometry::PointCloud>(*pcd_base_link);
    pcd_received->Transform(mat_baselink2odom_snapshot);

    std::vector<std::shared_ptr<open3d::geometry::PointCloud>> scan_window;
    {
        std::lock_guard<std::mutex> scan_lock(lock_scan_);
        que_pcd_scan_.push_back(pcd_received);
        while (que_pcd_scan_.size() > static_cast<size_t>(queue_maxsize_))
        {
            que_pcd_scan_.pop_front();
        }

        if (que_pcd_scan_.size() >= static_cast<size_t>(queue_maxsize_))
        {
            scan_window.assign(que_pcd_scan_.begin(), que_pcd_scan_.end());
        }
    }

    if (!scan_window.empty())
    {
        auto combined_scan = std::make_shared<open3d::geometry::PointCloud>();
        for (const auto &scan : scan_window)
        {
            *combined_scan += *scan;
        }
        std::lock_guard<std::mutex> scan_lock(lock_scan_);
        pcd_scan_cur_ = combined_scan;
    }
}

bool GloabalLocalization::LocalizationInitialize()
{
    SetLocalizationStatus(LocalizationStatus::INITIALIZING, "initializing");
    /// 裁剪后的地图
    std::shared_ptr<open3d::geometry::PointCloud> map_fine_crop(new open3d::geometry::PointCloud);

    /// 当前环境感知子图点云
    std::shared_ptr<open3d::geometry::PointCloud> pcd_scan(new open3d::geometry::PointCloud);

    /// 用于配准的source target
    std::shared_ptr<open3d::geometry::PointCloud> source(new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::PointCloud> target(new open3d::geometry::PointCloud);

    /// cropbox,用于裁剪地图和当前环境感知子图
    std::shared_ptr<open3d::geometry::OrientedBoundingBox> OBB_map(new open3d::geometry::OrientedBoundingBox);
    std::shared_ptr<open3d::geometry::OrientedBoundingBox> OBB_scan(new open3d::geometry::OrientedBoundingBox);

    /// 当前baselink到odom和map坐标系的关系
    Eigen::Matrix4d mat_baselink2odom_cur = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d mat_baselink2map_cur = Eigen::Matrix4d::Identity();

    /// 固定感知子图/历史地图子图大小
    OBB_map->extent_ = Eigen::Vector3d(60, 60, 40);
    OBB_map->color_ = Eigen::Vector3d(1, 0.5, 0);
    OBB_scan->extent_ = Eigen::Vector3d(60, 60, 40);
    OBB_scan->color_ = Eigen::Vector3d(0, 1, 0);

    double fitness_initial; /// overlap
    double loc_cost = 0;    /// 定位耗时(ms)
    int count_success = 0;
    bool init_success = false;
    while (rclcpp::ok() && !flag_exit_.load())
    {
        auto loc_s = std::chrono::high_resolution_clock::now(); /// 开始定位计时
        std::shared_ptr<open3d::geometry::PointCloud> scan_snapshot;
        {
            std::lock_guard<std::mutex> scan_lock(lock_scan_);
            scan_snapshot = pcd_scan_cur_;
        }
        if (scan_snapshot == nullptr || scan_snapshot->IsEmpty())
        {
            SetLocalizationStatus(LocalizationStatus::INITIALIZING, "no_scan");
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }
        else
        {
            /// 获取最新关系
            Eigen::Matrix4d reg_matrix = Eigen::Matrix4d::Identity();
            {
                std::lock_guard<std::mutex> state_lock(lock_mat_odom2map_);
                mat_baselink2odom_cur = mat_baselink2odom_;
                reg_matrix = mat_odom2map_;
            }
            *pcd_scan = *scan_snapshot;
            mat_baselink2map_cur = reg_matrix * mat_baselink2odom_cur;

            /// 将cropbox转换到对应位置进行裁剪点云
            OBB_map->center_ = mat_baselink2map_cur.block<3, 1>(0, 3);
            OBB_map->R_ = mat_baselink2map_cur.block<3, 3>(0, 0);
            OBB_scan->center_ = mat_baselink2odom_cur.block<3, 1>(0, 3);
            OBB_scan->R_ = mat_baselink2odom_cur.block<3, 3>(0, 0);
            {
                std::lock_guard<std::mutex> map_lock(lock_map_);
                if (!pcd_map_fine_ || pcd_map_fine_->IsEmpty())
                {
                    SetLocalizationStatus(LocalizationStatus::UNINITIALIZED, "map_empty");
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                    continue;
                }
                *map_fine_crop = *pcd_map_fine_->Crop(*OBB_map);
            }

            /// 配准计时
            target = map_fine_crop;

            source = pcd_scan->Crop(*OBB_scan);
            const size_t source_before_voxel_size = source->points_.size();
            source = source->VoxelDownSample(voxel_downsample_size_);
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                  "init preprocess: target=%zu, source=%zu->%zu, target_has_normal=%s, source_has_normal=%s",
                                  target->points_.size(), source_before_voxel_size, source->points_.size(),
                                  target->HasNormals() ? "true" : "false", source->HasNormals() ? "true" : "false");

            if (source->points_.size() < static_cast<size_t>(min_source_points_) ||
                target->points_.size() < static_cast<size_t>(min_target_points_))
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "skip init icp: source=%zu (min=%d), target=%zu (min=%d), map_center=(%.3f, %.3f, %.3f), scan_center=(%.3f, %.3f, %.3f)",
                                     source->points_.size(), min_source_points_, target->points_.size(), min_target_points_,
                                     OBB_map->center_.x(), OBB_map->center_.y(), OBB_map->center_.z(),
                                     OBB_scan->center_.x(), OBB_scan->center_.y(), OBB_scan->center_.z());
                SetLocalizationStatus(LocalizationStatus::INITIALIZING, "invalid_cloud");
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
                continue;
            }

            source->Transform(reg_matrix);
            auto eva_before_icp = open3d::pipelines::registration::EvaluateRegistration(*source, *target, fitness_eval_threshold_);
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                  "init before icp: eva_fitness=%f, inlier_rmse=%f, eval_threshold=%.3f",
                                  eva_before_icp.fitness_, eva_before_icp.inlier_rmse_, fitness_eval_threshold_);

            auto multiScale_reg_matrix = pcd_tools::RegistrationMultiScaleIcp(source, target, voxel_downsample_size_, 1, {1, 2, 4});
            reg_matrix = multiScale_reg_matrix * reg_matrix;
            source->Transform(multiScale_reg_matrix);
            auto eva_result_coarse = open3d::pipelines::registration::EvaluateRegistration(*source, *target, fitness_eval_threshold_);
            double init_delta_trans = multiScale_reg_matrix.block<3, 1>(0, 3).norm();
            double init_delta_yaw = std::atan2(multiScale_reg_matrix(1, 0), multiScale_reg_matrix(0, 0)) * 180.0 / M_PI;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "init icp result: eva_before=%f, eva_after=%f, inlier_rmse=%f, delta_trans=%.3f, delta_yaw_deg=%.3f, threshold_fitness_init=%.3f, max_init_icp_translation=%.3f, max_init_icp_yaw_deg=%.3f",
                                 eva_before_icp.fitness_, eva_result_coarse.fitness_, eva_result_coarse.inlier_rmse_,
                                 init_delta_trans, init_delta_yaw, threshold_fitness_init_, max_init_icp_translation_, max_init_icp_yaw_deg_);
            fitness_initial = eva_result_coarse.fitness_;

            bool safe_init_step = init_delta_trans <= max_init_icp_translation_ &&
                                  std::abs(init_delta_yaw) <= max_init_icp_yaw_deg_;
            bool init_fitness_improved = fitness_initial > eva_before_icp.fitness_ + min_init_fitness_improvement_;
            bool accept_init = fitness_initial > threshold_fitness_init_ && safe_init_step;
            bool update_init_candidate = safe_init_step && (accept_init || init_fitness_improved);

            if (update_init_candidate)
            {
                {
                    std::lock_guard<std::mutex> state_lock(lock_mat_odom2map_);
                    mat_odom2map_ = reg_matrix;
                }
                RCLCPP_INFO(this->get_logger(),
                            "update init candidate: eva_before=%f, eva_after=%f, improvement=%f, success=%s, odom2map_xyz=(%.3f, %.3f, %.3f)",
                            eva_before_icp.fitness_, fitness_initial, fitness_initial - eva_before_icp.fitness_,
                            accept_init ? "true" : "false", reg_matrix(0, 3), reg_matrix(1, 3), reg_matrix(2, 3));
            }
            else
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "reject init icp: eva_before=%f, eva_after=%f, threshold=%.3f, improvement=%f, min_improvement=%.3f, delta_trans=%.3f, max_delta=%.3f, delta_yaw_deg=%.3f, max_yaw_deg=%.3f, source=%zu, target=%zu",
                                     eva_before_icp.fitness_, fitness_initial, threshold_fitness_init_,
                                     fitness_initial - eva_before_icp.fitness_, min_init_fitness_improvement_,
                                     init_delta_trans, max_init_icp_translation_, init_delta_yaw, max_init_icp_yaw_deg_,
                                     source->points_.size(), target->points_.size());
            }
            auto loc_e = std::chrono::high_resolution_clock::now(); /// 结束定位计时
            loc_cost = std::chrono::duration_cast<std::chrono::microseconds>(loc_e - loc_s).count() / 1000.0;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "init localization cost: %.3f ms", loc_cost);

            if (accept_init)
            {
                count_success += 1;
                /// 连续两次定位成功后定位初始化成功
                if (count_success >= 2)
                {
                    init_success = true;
                    break;
                }
            }
            else
            {
                count_success = 0;
            }
        }
    }

    if (!init_success)
    {
        RCLCPP_WARN(this->get_logger(), "localization initialize stopped before success");
        SetLocalizationStatus(LocalizationStatus::UNINITIALIZED, "init_stopped");
        return false;
    }

    RCLCPP_INFO(this->get_logger(), "\n---------------------------------------------------------\n");
    RCLCPP_INFO(this->get_logger(), "---------------------------------------------------------\n");
    RCLCPP_INFO(this->get_logger(), "---------------------------------------------------------\n");
    RCLCPP_INFO(this->get_logger(), "localization initialize success\n");
    RCLCPP_INFO(this->get_logger(), "---------------------------------------------------------\n");
    RCLCPP_INFO(this->get_logger(), "---------------------------------------------------------\n");
    RCLCPP_INFO(this->get_logger(), "---------------------------------------------------------\n");

    tracking_fail_count_ = 0;
    SetLocalizationStatus(LocalizationStatus::INIT_SUCCESS, "ok");
    return true;
}
void GloabalLocalization::Localization()
{
    RCLCPP_INFO(this->get_logger(), "wait for Odometry_loc");
    // 等待接收到第一条里程计消息（通过检查timestamp是否有效）
    while (rclcpp::ok() && !flag_exit_.load())
    {
        {
            std::lock_guard<std::mutex> timestamp_lock(lock_timestamp_);
            if (timestamp_odom_.seconds() != 0.0)
            {
                break;
            }
        }
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for Odometry_loc...");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    RCLCPP_INFO(this->get_logger(), "Received Odometry_loc");

    RCLCPP_INFO(this->get_logger(), "wait for cloud_registered_body_1");
    // 等待接收到第一条点云消息（通过检查pcd_scan_cur_是否为空）
    while (rclcpp::ok() && !flag_exit_.load())
    {
        bool has_scan = false;
        {
            std::lock_guard<std::mutex> scan_lock(lock_scan_);
            has_scan = pcd_scan_cur_ != nullptr && !pcd_scan_cur_->IsEmpty();
        }
        if (has_scan)
        {
            break;
        }
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for cloud_registered_body_1...");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    RCLCPP_INFO(this->get_logger(), "Received cloud_registered_body_1");
    if (flag_exit_.load())
    {
        return;
    }

    // initialize
    /****初始化定位****/
    Eigen::Matrix4d mat_baselink2odom_init = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d mat_odom2map_init = Eigen::Matrix4d::Identity();
    {
        std::lock_guard<std::mutex> state_lock(lock_mat_odom2map_);
        mat_odom2map_ = mat_initialpose_ * mat_baselink2odom_.inverse(); /// initialpose 表示 base_link 在 map 下的位姿
        mat_baselink2odom_init = mat_baselink2odom_;
        mat_odom2map_init = mat_odom2map_;
    }
    RCLCPP_INFO(this->get_logger(),
                "initial odom2map from initialpose and current odom: initial_xyz=(%.3f, %.3f, %.3f), odom_xyz=(%.3f, %.3f, %.3f), odom2map_xyz=(%.3f, %.3f, %.3f)",
                mat_initialpose_(0, 3), mat_initialpose_(1, 3), mat_initialpose_(2, 3),
                mat_baselink2odom_init(0, 3), mat_baselink2odom_init(1, 3), mat_baselink2odom_init(2, 3),
                mat_odom2map_init(0, 3), mat_odom2map_init(1, 3), mat_odom2map_init(2, 3));
    if (!LocalizationInitialize())
    {
        return;
    }

    loc_initialized_.store(true); /// 初始化成功
    SetLocalizationStatus(LocalizationStatus::TRACKING, "ok");

    RCLCPP_INFO(this->get_logger(), "Localization initialization complete");

    std::shared_ptr<open3d::geometry::PointCloud> pcd_scan(new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::PointCloud> source(new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::PointCloud> target(new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::PointCloud> map_fine_crop(new open3d::geometry::PointCloud);
    std::shared_ptr<open3d::geometry::OrientedBoundingBox> OBB_map(new open3d::geometry::OrientedBoundingBox);
    std::shared_ptr<open3d::geometry::OrientedBoundingBox> OBB_scan(new open3d::geometry::OrientedBoundingBox);
    OBB_map->color_ = Eigen::Vector3d(1, 0.5, 0);
    OBB_map->extent_ = Eigen::Vector3d(60, 60, 40);

    OBB_scan->extent_ = Eigen::Vector3d(60, 60, 40);
    OBB_scan->color_ = Eigen::Vector3d(0, 1, 0);
    rclcpp::Time time_current;
    {
        std::lock_guard<std::mutex> timestamp_lock(lock_timestamp_);
        time_current = timestamp_odom_;
    }
    rclcpp::Time time_last = time_current - rclcpp::Duration(3, 0);

    double time_diff_loc = 5;                                     /// 前后两次定位的时间差(s)
    std::chrono::high_resolution_clock::time_point time_last_loc; /// 上次定位的完成时间点
    std::chrono::high_resolution_clock::time_point time_this_loc; /// 当前定位的开始时间点
    double loc_cost = 0;                                          /// 定位耗时(ms)
    while (rclcpp::ok() && !flag_exit_.load())
    {
        if (relocalization_requested_.exchange(false))
        {
            SetLocalizationStatus(LocalizationStatus::INITIALIZING, "initialpose");
            loc_initialized_.store(false);
            loc_fitness_.store(0.0);
            last_loc_ = Eigen::Vector3d(0, 0, -5000);

            RCLCPP_WARN(this->get_logger(),
                        "manual initialpose requested relocalization; run initialization ICP with init constraints");
            if (!LocalizationInitialize())
            {
                return;
            }

            loc_initialized_.store(true);
            SetLocalizationStatus(LocalizationStatus::TRACKING, "ok");
            last_loc_ = Eigen::Vector3d(0, 0, -5000);
            map_fine_crop->Clear();
            loc_cost = 0.0;
            time_last_loc = std::chrono::high_resolution_clock::now();
            RCLCPP_INFO(this->get_logger(), "manual relocalization complete");
            continue;
        }

        if (!loc_initialized_.load())
        {
            SetLocalizationStatus(LocalizationStatus::UNINITIALIZED, "waiting_initialpose");
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            continue;
        }

        {
            std::lock_guard<std::mutex> timestamp_lock(lock_timestamp_);
            time_current = timestamp_odom_;
        }
        auto time_diff_frame = time_current.seconds() - time_last.seconds();
        time_last = time_current;
        if (std::fabs(time_diff_frame) < 1e-6)
        {
            loc_cost = 0.0;
            continue;
        }

        time_this_loc = std::chrono::high_resolution_clock::now();
        time_diff_loc = std::chrono::duration_cast<std::chrono::microseconds>(time_this_loc - time_last_loc).count() / 1000000.0 + loc_cost / 1000.0;

        if (time_diff_loc < loc_frequence_)
        {
            int wait_time = int((loc_frequence_ - time_diff_loc) * 1000);
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                  "tracking wait: time_diff=%.3f s, sleep %d ms", time_diff_loc, wait_time);
            std::this_thread::sleep_for(std::chrono::milliseconds(wait_time));
        }
        else
        {
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                  "tracking run now: time_diff=%.3f s", time_diff_loc);
        }
        auto loc_s = std::chrono::high_resolution_clock::now(); /// 开始定位计时

        std::shared_ptr<open3d::geometry::PointCloud> scan_snapshot;
        {
            std::lock_guard<std::mutex> scan_lock(lock_scan_);
            scan_snapshot = pcd_scan_cur_;
        }
        if (scan_snapshot == nullptr || scan_snapshot->IsEmpty())
        {
            SetLocalizationStatus(LocalizationStatus::TRACKING_WARN, "no_scan");
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }
        else
        {
            Eigen::Matrix4d mat_baselink2odom_cur = Eigen::Matrix4d::Identity();
            Eigen::Matrix4d mat_baselink2map_cur = Eigen::Matrix4d::Identity();
            Eigen::Matrix4d reg_matrix = Eigen::Matrix4d::Identity();

            {
                std::lock_guard<std::mutex> state_lock(lock_mat_odom2map_);
                mat_baselink2odom_cur = mat_baselink2odom_;
                reg_matrix = mat_odom2map_;
            }
            *pcd_scan = *scan_snapshot;
            mat_baselink2map_cur = reg_matrix * mat_baselink2odom_cur;

            Eigen::Vector3d cur_loc(mat_baselink2map_cur(0, 3), mat_baselink2map_cur(1, 3), mat_baselink2map_cur(2, 3));
            auto dis_motion = ComputeMotionDis(last_loc_, cur_loc);
            if (dis_motion > dis_updatemap_)
            {
                auto submap_s = std::chrono::high_resolution_clock::now();

                RCLCPP_INFO(this->get_logger(),
                            "update submap: last=(%.3f, %.3f, %.3f), current=(%.3f, %.3f, %.3f), distance=%.3f",
                            last_loc_.x(), last_loc_.y(), last_loc_.z(), cur_loc.x(), cur_loc.y(), cur_loc.z(), dis_motion);
                last_loc_ = cur_loc;
                OBB_map->center_ = mat_baselink2map_cur.block<3, 1>(0, 3);
                OBB_map->R_ = mat_baselink2map_cur.block<3, 3>(0, 0);

                /// 粗地图和精地图
                {
                    std::lock_guard<std::mutex> map_lock(lock_map_);
                    if (!pcd_map_fine_ || pcd_map_fine_->IsEmpty())
                    {
                        SetLocalizationStatus(LocalizationStatus::TRACKING_LOST, "map_empty");
                        continue;
                    }
                    *map_fine_crop = *pcd_map_fine_->Crop(*OBB_map);
                }

                auto submap_e = std::chrono::high_resolution_clock::now();
                auto submap_cost = std::chrono::duration_cast<std::chrono::microseconds>(submap_e - submap_s).count() / 1000.0;
                RCLCPP_DEBUG(this->get_logger(), "submap_cost: %.3f ms", submap_cost);
            }

            OBB_scan->center_ = mat_baselink2odom_cur.block<3, 1>(0, 3);
            OBB_scan->R_ = mat_baselink2odom_cur.block<3, 3>(0, 0);

            target = map_fine_crop;

            source = pcd_scan->Crop(*OBB_scan);
            const size_t source_before_voxel_size = source->points_.size();
            source = source->VoxelDownSample(voxel_downsample_size_);
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                  "tracking preprocess: target=%zu, source=%zu->%zu, voxel=%.3f",
                                  target->points_.size(), source_before_voxel_size, source->points_.size(),
                                  voxel_downsample_size_);

            if (source->points_.size() < static_cast<size_t>(min_source_points_) ||
                target->points_.size() < static_cast<size_t>(min_target_points_))
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "skip tracking icp: source=%zu (min=%d), target=%zu (min=%d), map_center=(%.3f, %.3f, %.3f), scan_center=(%.3f, %.3f, %.3f)",
                                     source->points_.size(), min_source_points_, target->points_.size(), min_target_points_,
                                     OBB_map->center_.x(), OBB_map->center_.y(), OBB_map->center_.z(),
                                     OBB_scan->center_.x(), OBB_scan->center_.y(), OBB_scan->center_.z());
                tracking_fail_count_++;
                SetLocalizationStatus(
                    tracking_fail_count_ >= localization_lost_fail_count_
                        ? LocalizationStatus::TRACKING_LOST
                        : LocalizationStatus::TRACKING_WARN,
                    "invalid_cloud");
                continue;
            }

            auto eva_before_icp = open3d::pipelines::registration::EvaluateRegistration(*source, *target, fitness_eval_threshold_, reg_matrix);
            RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                  "tracking before icp: eva_fitness=%f, inlier_rmse=%f, eval_threshold=%.3f, source=%zu, target=%zu",
                                  eva_before_icp.fitness_, eva_before_icp.inlier_rmse_, fitness_eval_threshold_,
                                  source->points_.size(), target->points_.size());

            auto reg_result2 = pcd_tools::RegistrationIcp(source, target, icp_distance_threshold_, reg_matrix, 1);
            reg_matrix = reg_result2.transformation_ * reg_matrix;
            auto eva_result2 = open3d::pipelines::registration::EvaluateRegistration(*source, *target, fitness_eval_threshold_, reg_matrix);
            /// 给发布的置信度赋值
            loc_fitness_.store(eva_result2.fitness_);
            double delta_trans = reg_result2.transformation_.block<3, 1>(0, 3).norm();
            double delta_yaw = std::atan2(reg_result2.transformation_(1, 0), reg_result2.transformation_(0, 0)) * 180.0 / M_PI;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "tracking icp result: reg_fitness=%f, eva_before=%f, eva_after=%f, inlier_rmse=%f, delta_trans=%.3f, delta_yaw_deg=%.3f, icp_threshold=%.3f, eval_threshold=%.3f",
                                 reg_result2.fitness_, eva_before_icp.fitness_, eva_result2.fitness_, reg_result2.inlier_rmse_,
                                 delta_trans, delta_yaw, icp_distance_threshold_, fitness_eval_threshold_);
            /// 超过阈值才更新,防止因配准结果有问题而导致定位出问题
            const double loc_fitness = loc_fitness_.load();
            bool accept_tracking = loc_fitness > threshold_fitness_ &&
                                   delta_trans <= max_icp_translation_ &&
                                   std::abs(delta_yaw) <= max_icp_yaw_deg_;
            if (accept_tracking)
            {
                std::lock_guard<std::mutex> state_lock(lock_mat_odom2map_);
                mat_odom2map_ = reg_matrix;
                tracking_fail_count_ = 0;
                SetLocalizationStatus(LocalizationStatus::TRACKING, "ok");
            }
            else
            {
                std::string reject_reason = "fitness_low";
                if (delta_trans > max_icp_translation_)
                    reject_reason = "delta_too_large";
                else if (std::abs(delta_yaw) > max_icp_yaw_deg_)
                    reject_reason = "yaw_delta_too_large";
                tracking_fail_count_++;
                SetLocalizationStatus(
                    tracking_fail_count_ >= localization_lost_fail_count_
                        ? LocalizationStatus::TRACKING_LOST
                        : LocalizationStatus::TRACKING_WARN,
                    reject_reason);
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "reject tracking icp: eva_fitness=%f, threshold=%.3f, delta_trans=%.3f, max_delta=%.3f, delta_yaw_deg=%.3f, max_yaw_deg=%.3f, source=%zu, target=%zu",
                                     loc_fitness, threshold_fitness_, delta_trans, max_icp_translation_, delta_yaw, max_icp_yaw_deg_,
                                     source->points_.size(), target->points_.size());
            }

            auto loc_e = std::chrono::high_resolution_clock::now(); /// 结束定位计时
            time_last_loc = loc_e;
            loc_cost = std::chrono::duration_cast<std::chrono::microseconds>(loc_e - loc_s).count() / 1000.0;
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                                 "tracking localization cost: %.3f ms", loc_cost);
        }
    }
}

void GloabalLocalization::StartLoc()
{
    thread_loc_ = std::thread(&GloabalLocalization::Localization, this);
}

void GloabalLocalization::CallbackInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr initialpose)
{
    const double current_fitness = loc_fitness_.load();
    const bool was_initialized = loc_initialized_.load();
    RCLCPP_INFO(this->get_logger(), "received initialpose: current confidence=%f, loc_initialized=%s",
                current_fitness, was_initialized ? "true" : "false");

    RCLCPP_INFO(this->get_logger(),
                "initialpose msg: xyz=(%.3f, %.3f, %.3f), quat=(%.6f, %.6f, %.6f, %.6f)",
                initialpose->pose.pose.position.x, initialpose->pose.pose.position.y, initialpose->pose.pose.position.z,
                initialpose->pose.pose.orientation.x, initialpose->pose.pose.orientation.y,
                initialpose->pose.pose.orientation.z, initialpose->pose.pose.orientation.w);

    Eigen::Quaterniond rotation_q(
        initialpose->pose.pose.orientation.w,
        initialpose->pose.pose.orientation.x,
        initialpose->pose.pose.orientation.y,
        initialpose->pose.pose.orientation.z);
    if (!std::isfinite(rotation_q.norm()) || rotation_q.norm() < 1e-6)
    {
        RCLCPP_WARN(this->get_logger(), "invalid initialpose quaternion, use identity rotation");
        rotation_q = Eigen::Quaterniond::Identity();
    }
    else
    {
        rotation_q.normalize();
    }

    Eigen::Matrix4d mat_initialpose_msg = Eigen::Matrix4d::Identity();
    mat_initialpose_msg.block<3, 3>(0, 0) = rotation_q.matrix();
    mat_initialpose_msg.block<3, 1>(0, 3) =
        Eigen::Vector3d(initialpose->pose.pose.position.x,
                        initialpose->pose.pose.position.y,
                        initialpose->pose.pose.position.z);

    Eigen::Matrix4d mat_baselink2odom_snapshot = Eigen::Matrix4d::Identity();
    Eigen::Matrix4d mat_odom2map_snapshot = Eigen::Matrix4d::Identity();
    {
        std::lock_guard<std::mutex> state_lock(lock_mat_odom2map_);
        mat_initialpose_ = mat_initialpose_msg;
        mat_baselink2odom_snapshot = mat_baselink2odom_;
        mat_odom2map_ = mat_initialpose_ * mat_baselink2odom_.inverse();
        mat_odom2map_snapshot = mat_odom2map_;
    }
    RCLCPP_INFO(this->get_logger(),
                "update odom2map from initialpose: initial_xyz=(%.3f, %.3f, %.3f), odom_xyz=(%.3f, %.3f, %.3f), odom2map_xyz=(%.3f, %.3f, %.3f)",
                mat_initialpose_msg(0, 3), mat_initialpose_msg(1, 3), mat_initialpose_msg(2, 3),
                mat_baselink2odom_snapshot(0, 3), mat_baselink2odom_snapshot(1, 3), mat_baselink2odom_snapshot(2, 3),
                mat_odom2map_snapshot(0, 3), mat_odom2map_snapshot(1, 3), mat_odom2map_snapshot(2, 3));

    if (was_initialized)
    {
        {
            std::lock_guard<std::mutex> scan_lock(lock_scan_);
            que_pcd_scan_.clear();
            pcd_scan_cur_.reset(new open3d::geometry::PointCloud);
        }
        loc_initialized_.store(false);
        loc_fitness_.store(0.0);
        relocalization_requested_.store(true);
        RCLCPP_WARN(this->get_logger(),
                    "manual initialpose accepted after initialization; clear scan window and request relocalization");
    }
    else
    {
        relocalization_requested_.store(true);
        SetLocalizationStatus(LocalizationStatus::INITIALIZING, "initialpose");
        RCLCPP_INFO(this->get_logger(),
                    "manual initialpose accepted before initialization; request initialization ICP");
    }
}

double GloabalLocalization::ComputeMotionDis(const Eigen::Vector3d &a, const Eigen::Vector3d &b)
{
    return std::sqrt(std::pow(a.x() - b.x(), 2) + std::pow(a.y() - b.y(), 2) + std::pow(a.z() - b.z(), 2));
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GloabalLocalization>();

    // 使用多线程执行器，可以指定线程数
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
