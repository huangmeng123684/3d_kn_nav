#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include "open3d_loc/srv/get_pose.hpp"
#include "open3d_loc/srv/pose_deviation.hpp"
#include "open3d_loc/srv/publish_goal.hpp"
#include "open3d_loc/srv/relocalize.hpp"

class LocalizationServiceNode : public rclcpp::Node
{
public:
    LocalizationServiceNode() : Node("localization_service_node")
    {
        this->declare_parameter<std::string>("initialpose_topic", "/initialpose");
        this->declare_parameter<std::string>("current_pose_topic", "/Odometry_open3d");
        this->declare_parameter<std::string>("goal_topic", "/goal_pose");
        this->declare_parameter<double>("relocalize_timeout_sec", 10.0);

        this->get_parameter("initialpose_topic", initialpose_topic_);
        this->get_parameter("current_pose_topic", current_pose_topic_);
        this->get_parameter("goal_topic", goal_topic_);
        this->get_parameter("relocalize_timeout_sec", relocalize_timeout_sec_);
        if (!std::isfinite(relocalize_timeout_sec_) || relocalize_timeout_sec_ <= 0.0)
        {
            throw std::invalid_argument("relocalize_timeout_sec must be finite and positive");
        }

        subscription_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        service_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = subscription_group_;

        initialpose_pub_ =
            this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(initialpose_topic_, 10);
        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic_, 10);
        current_pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            current_pose_topic_, 20,
            std::bind(&LocalizationServiceNode::currentPoseCallback, this, std::placeholders::_1),
            sub_options);

        relocalize_srv_ = this->create_service<open3d_loc::srv::Relocalize>(
            "/open3d_loc/relocalize",
            std::bind(&LocalizationServiceNode::handleRelocalize, this,
                      std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default, service_group_);
        pose_deviation_srv_ = this->create_service<open3d_loc::srv::PoseDeviation>(
            "/open3d_loc/pose_deviation",
            std::bind(&LocalizationServiceNode::handlePoseDeviation, this,
                      std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default, service_group_);
        get_pose_srv_ = this->create_service<open3d_loc::srv::GetPose>(
            "/open3d_loc/get_pose",
            std::bind(&LocalizationServiceNode::handleGetPose, this,
                      std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default, service_group_);
        publish_goal_srv_ = this->create_service<open3d_loc::srv::PublishGoal>(
            "/open3d_loc/publish_goal",
            std::bind(&LocalizationServiceNode::handlePublishGoal, this,
                      std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default, service_group_);

        RCLCPP_INFO(this->get_logger(),
                    "navigation services ready: relocalize=/open3d_loc/relocalize, "
                    "get_pose=/open3d_loc/get_pose, publish_goal=/open3d_loc/publish_goal, "
                    "pose_deviation=/open3d_loc/pose_deviation, current_pose_topic=%s, goal_topic=%s",
                    current_pose_topic_.c_str(), goal_topic_.c_str());
    }

private:
    struct ReferencePose
    {
        geometry_msgs::msg::Pose pose;
        double yaw = 0.0;
    };

    struct PoseSnapshot
    {
        bool pose_valid = false;
        geometry_msgs::msg::Pose pose;
        uint64_t pose_count = 0;
    };

    struct Deviation
    {
        double error_x = 0.0;
        double error_y = 0.0;
        double distance_xy = 0.0;
        double yaw_error_rad = 0.0;
        double yaw_error_deg = 0.0;
    };

    struct ResetRelocalizeFlag
    {
        explicit ResetRelocalizeFlag(std::atomic_bool &flag) : flag_(flag) {}
        ~ResetRelocalizeFlag() { flag_.store(false); }
        std::atomic_bool &flag_;
    };

    static double normalizeAngle(double angle)
    {
        while (angle > M_PI)
        {
            angle -= 2.0 * M_PI;
        }
        while (angle < -M_PI)
        {
            angle += 2.0 * M_PI;
        }
        return angle;
    }

    static bool quaternionToYaw(double qx, double qy, double qz, double qw,
                                double &yaw, geometry_msgs::msg::Quaternion *normalized_quat,
                                std::string &message)
    {
        if (!std::isfinite(qx) || !std::isfinite(qy) || !std::isfinite(qz) || !std::isfinite(qw))
        {
            message = "quaternion contains non-finite value";
            return false;
        }

        const double norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
        if (!std::isfinite(norm) || norm < 1e-6)
        {
            message = "quaternion norm is too small";
            return false;
        }

        qx /= norm;
        qy /= norm;
        qz /= norm;
        qw /= norm;

        tf2::Quaternion q(qx, qy, qz, qw);
        double roll = 0.0;
        double pitch = 0.0;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        if (normalized_quat != nullptr)
        {
            normalized_quat->x = qx;
            normalized_quat->y = qy;
            normalized_quat->z = qz;
            normalized_quat->w = qw;
        }
        return true;
    }

    static bool buildReference(double x, double y, double z,
                               double qx, double qy, double qz, double qw,
                               ReferencePose &reference, std::string &message)
    {
        if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z))
        {
            message = "position contains non-finite value";
            return false;
        }

        reference.pose.position.x = x;
        reference.pose.position.y = y;
        reference.pose.position.z = z;
        return quaternionToYaw(qx, qy, qz, qw, reference.yaw, &reference.pose.orientation, message);
    }

    static bool computeDeviation(const ReferencePose &reference,
                                 const geometry_msgs::msg::Pose &current_pose,
                                 Deviation &deviation,
                                 std::string &message)
    {
        double current_yaw = 0.0;
        if (!quaternionToYaw(current_pose.orientation.x, current_pose.orientation.y,
                             current_pose.orientation.z, current_pose.orientation.w,
                             current_yaw, nullptr, message))
        {
            message = "current pose has invalid quaternion: " + message;
            return false;
        }

        const double dx = current_pose.position.x - reference.pose.position.x;
        const double dy = current_pose.position.y - reference.pose.position.y;
        const double c = std::cos(reference.yaw);
        const double s = std::sin(reference.yaw);

        deviation.error_x = c * dx + s * dy;
        deviation.error_y = -s * dx + c * dy;
        deviation.distance_xy = std::sqrt(deviation.error_x * deviation.error_x +
                                          deviation.error_y * deviation.error_y);
        deviation.yaw_error_rad = normalizeAngle(current_yaw - reference.yaw);
        deviation.yaw_error_deg = deviation.yaw_error_rad * 180.0 / M_PI;
        return true;
    }

    PoseSnapshot getSnapshot()
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        PoseSnapshot snapshot;
        snapshot.pose_valid = current_pose_valid_;
        snapshot.pose = current_pose_;
        snapshot.pose_count = pose_update_count_;
        return snapshot;
    }

    void currentPoseCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        ReferencePose normalized;
        std::string message;
        const auto &position = msg->pose.pose.position;
        const auto &orientation = msg->pose.pose.orientation;
        if (!buildReference(position.x, position.y, position.z,
                            orientation.x, orientation.y, orientation.z, orientation.w,
                            normalized, message))
        {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                 "ignore invalid /Odometry_open3d pose: %s", message.c_str());
            return;
        }

        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            current_pose_ = normalized.pose;
            current_pose_valid_ = true;
            ++pose_update_count_;
        }
        state_cv_.notify_all();
    }

    void publishInitialPose(const ReferencePose &reference)
    {
        geometry_msgs::msg::PoseWithCovarianceStamped initialpose;
        initialpose.header.frame_id = "map";
        initialpose.header.stamp = this->now();
        initialpose.pose.pose = reference.pose;
        initialpose_pub_->publish(initialpose);
    }

    void fillDeviationResponse(const PoseSnapshot &snapshot, const ReferencePose &reference,
                               open3d_loc::srv::PoseDeviation::Response &response)
    {
        response.current_pose = snapshot.pose;
        Deviation deviation;
        std::string message;
        if (!computeDeviation(reference, snapshot.pose, deviation, message))
        {
            response.success = false;
            response.message = message;
            return;
        }

        response.success = true;
        response.error_x = deviation.error_x;
        response.error_y = deviation.error_y;
        response.distance_xy = deviation.distance_xy;
        response.yaw_error_rad = deviation.yaw_error_rad;
        response.yaw_error_deg = deviation.yaw_error_deg;
        response.message = "ok";
    }

    void handleRelocalize(const std::shared_ptr<open3d_loc::srv::Relocalize::Request> request,
                          std::shared_ptr<open3d_loc::srv::Relocalize::Response> response)
    {
        bool expected = false;
        if (!relocalize_running_.compare_exchange_strong(expected, true))
        {
            response->success = false;
            response->message = "another relocalize request is running";
            return;
        }
        ResetRelocalizeFlag reset_flag(relocalize_running_);

        ReferencePose reference;
        std::string message;
        if (!buildReference(request->x, request->y, request->z,
                            request->qx, request->qy, request->qz, request->qw,
                            reference, message))
        {
            response->success = false;
            response->message = message;
            return;
        }

        std::uint64_t start_pose_count = 0;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            start_pose_count = pose_update_count_;
            current_pose_valid_ = false;
        }
        publishInitialPose(reference);

        const auto timeout = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(relocalize_timeout_sec_));
        const auto deadline = std::chrono::steady_clock::now() + timeout;

        bool success = false;
        {
            std::unique_lock<std::mutex> lock(state_mutex_);
            success = state_cv_.wait_until(lock, deadline, [this, start_pose_count]() {
                return current_pose_valid_ && pose_update_count_ > start_pose_count;
            });
        }

        if (!success)
        {
            response->success = false;
            response->message = "timeout waiting for relocalization result on /Odometry_open3d";
            return;
        }

        response->success = true;
        response->message = "relocalization succeeded";
    }

    void handlePoseDeviation(const std::shared_ptr<open3d_loc::srv::PoseDeviation::Request> request,
                             std::shared_ptr<open3d_loc::srv::PoseDeviation::Response> response)
    {
        ReferencePose reference;
        std::string message;
        if (!buildReference(request->x, request->y, request->z,
                            request->qx, request->qy, request->qz, request->qw,
                            reference, message))
        {
            response->success = false;
            response->message = message;
            return;
        }

        PoseSnapshot snapshot = getSnapshot();
        if (!snapshot.pose_valid)
        {
            response->success = false;
            response->message = "no current pose from /Odometry_open3d";
            return;
        }

        fillDeviationResponse(snapshot, reference, *response);
    }

    void handleGetPose(const std::shared_ptr<open3d_loc::srv::GetPose::Request>,
                       std::shared_ptr<open3d_loc::srv::GetPose::Response> response)
    {
        if (relocalize_running_.load())
        {
            response->success = false;
            response->message = "relocalization is running";
            return;
        }

        PoseSnapshot snapshot = getSnapshot();
        if (!snapshot.pose_valid)
        {
            response->success = false;
            response->message = "no current base_link pose from /Odometry_open3d";
            return;
        }

        ReferencePose normalized;
        std::string message;
        const auto &position = snapshot.pose.position;
        const auto &orientation = snapshot.pose.orientation;
        if (!buildReference(position.x, position.y, position.z,
                            orientation.x, orientation.y, orientation.z, orientation.w,
                            normalized, message))
        {
            response->success = false;
            response->message = "invalid pose from /Odometry_open3d: " + message;
            return;
        }

        response->success = true;
        response->x = normalized.pose.position.x;
        response->y = normalized.pose.position.y;
        response->z = normalized.pose.position.z;
        response->qx = normalized.pose.orientation.x;
        response->qy = normalized.pose.orientation.y;
        response->qz = normalized.pose.orientation.z;
        response->qw = normalized.pose.orientation.w;
        response->message = "ok";
    }

    void handlePublishGoal(const std::shared_ptr<open3d_loc::srv::PublishGoal::Request> request,
                           std::shared_ptr<open3d_loc::srv::PublishGoal::Response> response)
    {
        ReferencePose goal;
        std::string message;
        if (!buildReference(request->x, request->y, request->z,
                            request->qx, request->qy, request->qz, request->qw,
                            goal, message))
        {
            response->success = false;
            response->message = message;
            return;
        }

        geometry_msgs::msg::PoseStamped goal_message;
        goal_message.header.frame_id = "map";
        goal_message.header.stamp = this->now();
        goal_message.pose = goal.pose;
        goal_pub_->publish(goal_message);

        response->success = true;
        response->message = "goal published on " + goal_topic_;
    }

    std::string initialpose_topic_;
    std::string current_pose_topic_;
    std::string goal_topic_;
    double relocalize_timeout_sec_ = 10.0;

    rclcpp::CallbackGroup::SharedPtr subscription_group_;
    rclcpp::CallbackGroup::SharedPtr service_group_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr current_pose_sub_;
    rclcpp::Service<open3d_loc::srv::Relocalize>::SharedPtr relocalize_srv_;
    rclcpp::Service<open3d_loc::srv::PoseDeviation>::SharedPtr pose_deviation_srv_;
    rclcpp::Service<open3d_loc::srv::GetPose>::SharedPtr get_pose_srv_;
    rclcpp::Service<open3d_loc::srv::PublishGoal>::SharedPtr publish_goal_srv_;

    std::mutex state_mutex_;
    std::condition_variable state_cv_;
    geometry_msgs::msg::Pose current_pose_;
    bool current_pose_valid_ = false;
    uint64_t pose_update_count_ = 0;
    std::atomic_bool relocalize_running_{false};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LocalizationServiceNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
