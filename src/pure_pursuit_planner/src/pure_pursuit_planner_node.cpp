// Modified for PCT integration:
//   - Path topic: /pct_path (was tgt_path)
//   - Odometry topic: /Odometry_open3d (was odom), frame_id=map, child=base_link
//   - ck (curvature) computed from path geometry (PCT stores z-height, not curvature)
//   - cyaw (tangent heading) computed from adjacent waypoints (PCT publishes identity quaternion)
//   - Path updates: old path_subscribe_flag removed → new path always replaces old

#include "pure_pursuit_planner/pure_pursuit_planner_node.hpp"
#include <cmath>
#include <stdexcept>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/twist.hpp"


namespace pure_pursuit_planner {

// ── helper: compute tangent yaw from consecutive points ──────────────────
static std::vector<double> compute_yaw_from_path(const std::vector<double>& cx,
                                                  const std::vector<double>& cy) {
    std::vector<double> cyaw(cx.size(), 0.0);
    if (cx.size() < 2) return cyaw;

    for (size_t i = 0; i < cx.size() - 1; i++) {
        cyaw[i] = std::atan2(cy[i + 1] - cy[i], cx[i + 1] - cx[i]);
    }
    // last point inherits previous yaw
    cyaw.back() = cyaw[cyaw.size() - 2];
    return cyaw;
}

// ── helper: compute discrete curvature from path geometry ────────────────
// κ_i = Δθ / Δs  where Δθ is turning angle at point i, Δs is avg segment length
static std::vector<double> compute_curvature_from_path(const std::vector<double>& cx,
                                                        const std::vector<double>& cy) {
    std::vector<double> ck(cx.size(), 0.0);
    if (cx.size() < 3) return ck;

    for (size_t i = 1; i < cx.size() - 1; i++) {
        // vectors: prev→cur and cur→next
        double dx1 = cx[i] - cx[i - 1];
        double dy1 = cy[i] - cy[i - 1];
        double dx2 = cx[i + 1] - cx[i];
        double dy2 = cy[i + 1] - cy[i];

        double len1 = std::hypot(dx1, dy1);
        double len2 = std::hypot(dx2, dy2);
        if (len1 < 1e-6 || len2 < 1e-6) continue;

        // turning angle Δθ
        double dot = (dx1 * dx2 + dy1 * dy2) / (len1 * len2);
        dot = std::clamp(dot, -1.0, 1.0);
        double dtheta = std::acos(dot);

        // arc length ≈ average of two segments
        double ds = 0.5 * (len1 + len2);

        ck[i] = dtheta / ds;  // curvature [rad/m]
    }
    // endpoints inherit from neighbor
    ck[0] = ck[1];
    ck.back() = ck[ck.size() - 2];
    return ck;
}


PurePursuitNode::PurePursuitNode(const rclcpp::NodeOptions& options)
: Node("pure_pursuit_node", options), planner_(config_) {

    declareAndGetParameters();

    // PCT: subscribe to /pct_path (nav_msgs/Path, frame_id=map)
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        "/pct_path", 10, std::bind(&PurePursuitNode::pathCallback, this, std::placeholders::_1));

    final_approach_sub_ = create_subscription<std_msgs::msg::Bool>(
        final_approach_topic_, rclcpp::QoS(1).reliable().transient_local(),
        std::bind(&PurePursuitNode::finalApproachCallback, this, std::placeholders::_1));

    // open3d_loc: subscribe to odometry (frame_id=map, child_frame_id=base_link)
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10, std::bind(&PurePursuitNode::odomCallback, this, std::placeholders::_1));

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    timer_ = create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&PurePursuitNode::timerCallback, this));
    planner_ = PurePursuitComponent(config_);  // re-init with populated config_

    RCLCPP_INFO(
        this->get_logger(),
        "Pure Pursuit ready: odom_topic=%s, odom_timeout=%.2f s, "
        "standalone_goal_completion=%s",
        odom_topic_.c_str(), config_.odom_timeout,
        config_.standalone_goal_completion ? "true" : "false");
}

void PurePursuitNode::declareAndGetParameters() {
    config_.k = this->declare_parameter("k", 0.5);
    config_.Lfc = this->declare_parameter("Lfc", 0.8);
    config_.Kp = this->declare_parameter("Kp", 1.0);
    config_.dt = this->declare_parameter("dt", 0.1);
    config_.goal_threshold = this->declare_parameter("goal_threshold", 0.4);
    config_.max_acceleration = this->declare_parameter("max_acceleration", 0.08);
    config_.minCurvature = this->declare_parameter("minCurvature", 0.0);
    config_.maxCurvature = this->declare_parameter("maxCurvature", 3.0);
    config_.minVelocity = this->declare_parameter("minVelocity", 0.1);
    config_.maxVelocity = this->declare_parameter("maxVelocity", 0.5);
    config_.maxAngularVelocity = this->declare_parameter("maxAngularVelocity", 0.5);
    config_.rotate_to_path_threshold =
        this->declare_parameter("rotate_to_path_threshold", 1.047);
    config_.rotate_to_path_tolerance =
        this->declare_parameter("rotate_to_path_tolerance", 0.349);
    config_.goal_yaw_tolerance = this->declare_parameter("goal_yaw_tolerance", 0.175);
    config_.rotate_to_heading_gain =
        this->declare_parameter("rotate_to_heading_gain", 1.0);
    config_.standalone_goal_completion =
        this->declare_parameter("standalone_goal_completion", false);
    config_.obstacle_th = this->declare_parameter("obstacle_th", 0.5);
    odom_topic_ = this->declare_parameter<std::string>("odom_topic", "/Odometry_open3d");
    final_approach_topic_ = this->declare_parameter<std::string>(
        "final_approach_topic", "/pct_scan_navigation/final_approach");
    config_.final_heading_entry_distance =
        this->declare_parameter("final_heading_entry_distance", config_.goal_threshold);
    config_.final_heading_command_deadband =
        this->declare_parameter("final_heading_command_deadband", 0.02);
    config_.min_final_angular_velocity =
        this->declare_parameter("min_final_angular_velocity", 0.20);
    config_.odom_timeout = this->declare_parameter("odom_timeout", 1.0);
    if (!std::isfinite(config_.goal_threshold) || config_.goal_threshold <= 0.0) {
        throw std::invalid_argument("goal_threshold must be finite and greater than zero");
    }
    if (!std::isfinite(config_.final_heading_entry_distance) ||
        config_.final_heading_entry_distance <= 0.0) {
        throw std::invalid_argument(
            "final_heading_entry_distance must be finite and greater than zero");
    }
    if (!std::isfinite(config_.final_heading_command_deadband) ||
        config_.final_heading_command_deadband < 0.0 ||
        config_.final_heading_command_deadband > M_PI) {
        throw std::invalid_argument(
            "final_heading_command_deadband must be finite and in [0, pi]");
    }
    if (!std::isfinite(config_.min_final_angular_velocity) ||
        config_.min_final_angular_velocity < 0.0) {
        throw std::invalid_argument(
            "min_final_angular_velocity must be finite and non-negative");
    }
    if (!std::isfinite(config_.Lfc) || config_.Lfc <= 0.0) {
        throw std::invalid_argument("Lfc must be finite and greater than zero");
    }
    if (!std::isfinite(config_.minVelocity) || !std::isfinite(config_.maxVelocity) ||
        config_.minVelocity < 0.0 || config_.minVelocity > config_.maxVelocity) {
        throw std::invalid_argument("velocity limits must be finite, non-negative, and ordered");
    }
    if (!std::isfinite(config_.maxCurvature) || config_.maxCurvature <= 0.0) {
        throw std::invalid_argument("maxCurvature must be finite and greater than zero");
    }
    if (!std::isfinite(config_.maxAngularVelocity) || config_.maxAngularVelocity <= 0.0) {
        throw std::invalid_argument("maxAngularVelocity must be finite and greater than zero");
    }
    if (!std::isfinite(config_.rotate_to_path_threshold) ||
        !std::isfinite(config_.rotate_to_path_tolerance) ||
        config_.rotate_to_path_tolerance <= 0.0 ||
        config_.rotate_to_path_tolerance >= config_.rotate_to_path_threshold ||
        config_.rotate_to_path_threshold > M_PI) {
        throw std::invalid_argument(
            "rotation thresholds must satisfy 0 < tolerance < threshold <= pi");
    }
    if (!std::isfinite(config_.goal_yaw_tolerance) ||
        config_.goal_yaw_tolerance <= 0.0 || config_.goal_yaw_tolerance > M_PI) {
        throw std::invalid_argument("goal_yaw_tolerance must be in (0, pi]");
    }
    if (!std::isfinite(config_.rotate_to_heading_gain) ||
        config_.rotate_to_heading_gain <= 0.0) {
        throw std::invalid_argument("rotate_to_heading_gain must be finite and positive");
    }
    if (!std::isfinite(config_.odom_timeout) || config_.odom_timeout <= 0.0) {
        throw std::invalid_argument("odom_timeout must be finite and greater than zero");
    }
    if (odom_topic_.empty()) {
        throw std::invalid_argument("odom_topic must not be empty");
    }
    if (final_approach_topic_.empty()) {
        throw std::invalid_argument("final_approach_topic must not be empty");
    }
}

void PurePursuitNode::pathCallback(const nav_msgs::msg::Path::SharedPtr msg) {
    if (msg->poses.empty()) {
        cx_.clear(); cy_.clear(); cyaw_.clear(); ck_.clear();
        path_received_ = false;
        final_approach_ = false;
        planner_.odom_sub_flag = false;
        planner_.oldNearestPointIndex = -1;
        publishZeroVelocity();
        RCLCPP_WARN(this->get_logger(), "Received empty /pct_path — path cleared and robot stopped.");
        return;
    }

    // Verify frame — PCT publishes in 'map', Odometry_open3d is also in 'map'
    if (msg->header.frame_id != "map" && msg->header.frame_id != "odom") {
        RCLCPP_WARN(this->get_logger(),
            "Path frame_id '%s' is not 'map' or 'odom'. Behavior may be incorrect.",
            msg->header.frame_id.c_str());
    }

    // Clear old path and rebuild
    cx_.clear(); cy_.clear(); cyaw_.clear(); ck_.clear();

    // Step 1: extract raw x/y from PCT path
    std::vector<double> raw_x, raw_y;
    for (const auto& pose : msg->poses) {
        if (!std::isfinite(pose.pose.position.x) ||
            !std::isfinite(pose.pose.position.y)) {
            cx_.clear(); cy_.clear(); cyaw_.clear(); ck_.clear();
            path_received_ = false;
            planner_.odom_sub_flag = false;
            planner_.oldNearestPointIndex = -1;
            publishZeroVelocity();
            RCLCPP_ERROR(this->get_logger(),
                "Received non-finite /pct_path point — path cleared and robot stopped.");
            return;
        }
        raw_x.push_back(pose.pose.position.x);
        raw_y.push_back(pose.pose.position.y);
    }

    // Step 2: compute cyaw (tangent heading) from adjacent points
    //         and preserve the explicit final goal orientation from the path.
    cyaw_ = compute_yaw_from_path(raw_x, raw_y);
    const auto& final_orientation = msg->poses.back().pose.orientation;
    const double quaternion_norm = std::sqrt(
        final_orientation.x * final_orientation.x +
        final_orientation.y * final_orientation.y +
        final_orientation.z * final_orientation.z +
        final_orientation.w * final_orientation.w);
    if (std::isfinite(quaternion_norm) && quaternion_norm > 1e-6) {
        tf2::Quaternion final_quaternion;
        tf2::fromMsg(final_orientation, final_quaternion);
        final_quaternion.normalize();
        tf2::Matrix3x3 final_rotation(final_quaternion);
        double final_roll, final_pitch, final_yaw;
        final_rotation.getRPY(final_roll, final_pitch, final_yaw);
        if (std::isfinite(final_yaw)) {
            cyaw_.back() = final_yaw;
        }
    } else {
        RCLCPP_WARN(this->get_logger(),
            "Final path orientation is invalid; using the final path tangent.");
    }

    // Step 3: compute ck (curvature) from path geometry
    //         PCT stores z-height in position.z, NOT curvature
    ck_ = compute_curvature_from_path(raw_x, raw_y);

    // Step 4: store final path
    cx_ = raw_x;
    cy_ = raw_y;

    // Reset state for new path
    path_received_ = true;
    planner_.odom_sub_flag = false;   // trigger re-init of nearest-point search
    planner_.oldNearestPointIndex = -1;

    RCLCPP_INFO(this->get_logger(),
        "Received /pct_path: %zu waypoints, frame=%s. "
        "Computed tangent yaw and curvature and preserved final goal yaw.",
        cx_.size(), msg->header.frame_id.c_str());
}

void PurePursuitNode::finalApproachCallback(
    const std_msgs::msg::Bool::SharedPtr msg) {
    final_approach_ = msg->data;
}

void PurePursuitNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // Odometry_open3d: header.frame_id = 'map', child_frame_id = 'base_link'
    // Position is directly in map frame → matches /pct_path coordinates
    const auto& position = msg->pose.pose.position;
    const auto& orientation = msg->pose.pose.orientation;
    if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
        !std::isfinite(orientation.x) || !std::isfinite(orientation.y) ||
        !std::isfinite(orientation.z) || !std::isfinite(orientation.w) ||
        !std::isfinite(msg->twist.twist.linear.x)) {
        RCLCPP_ERROR(this->get_logger(),
            "Ignoring /Odometry_open3d containing non-finite values.");
        return;
    }

    current_pose_.x = position.x;
    current_pose_.y = position.y;
    current_vx_ = msg->twist.twist.linear.x;

    tf2::Quaternion quat;
    tf2::fromMsg(msg->pose.pose.orientation, quat);
    tf2::Matrix3x3 mat(quat);
    double roll_tmp, pitch_tmp, yaw_tmp;
    mat.getRPY(roll_tmp, pitch_tmp, yaw_tmp);

    current_pose_.yaw = yaw_tmp;

    if (!std::isfinite(current_pose_.yaw)) {
        RCLCPP_ERROR(this->get_logger(),
            "Ignoring /Odometry_open3d with invalid orientation.");
        return;
    }

    pose_received_ = true;
    last_odom_time_ = std::chrono::steady_clock::now();
    odom_timeout_stop_published_ = false;
}

void PurePursuitNode::timerCallback() {
    if (!pose_received_) return;

    const double odom_age = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - last_odom_time_).count();
    if (odom_age > config_.odom_timeout) {
        if (!odom_timeout_stop_published_) {
            publishZeroVelocity();
            odom_timeout_stop_published_ = true;
            RCLCPP_ERROR(this->get_logger(),
                "%s timed out after %.3f s — publishing zero velocity.",
                odom_topic_.c_str(), odom_age);
        }
        return;
    }

    // Keep the bridge enabled after an empty path by refreshing it with a
    // zero command. No bridge-specific mode or service is needed.
    if (!path_received_) {
        publishZeroVelocity();
        return;
    }

    auto cmd_velocity = planner_.computeVelocity(
        cx_, cy_, cyaw_, ck_, current_pose_, current_vx_, final_approach_);

    if (cmd_velocity.size() != 2 || !std::isfinite(cmd_velocity[0]) ||
        !std::isfinite(cmd_velocity[1])) {
        publishZeroVelocity();
        RCLCPP_ERROR(this->get_logger(),
            "Pure pursuit produced an invalid command — publishing zero velocity.");
        return;
    }

    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = cmd_velocity[0];
    cmd_vel.angular.z = cmd_velocity[1];

    cmd_vel_pub_->publish(cmd_vel);
}

void PurePursuitNode::publishZeroVelocity() {
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel_pub_->publish(cmd_vel);
}

}  // namespace pure_pursuit_planner
