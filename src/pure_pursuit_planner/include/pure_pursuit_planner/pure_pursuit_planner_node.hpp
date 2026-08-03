// Directory: pure_pursuit_planner/include/pure_pursuit_planner/pure_pursuit_node.hpp
#pragma once

#include <chrono>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>

#include "pure_pursuit_planner/pure_pursuit_planner_component.hpp"

namespace pure_pursuit_planner {

class PurePursuitNode : public rclcpp::Node {
public:
    explicit PurePursuitNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    void pathCallback(const nav_msgs::msg::Path::SharedPtr msg);
    void finalApproachCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void timerCallback();
    void publishZeroVelocity();

    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr final_approach_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    PurePursuitComponent planner_;

    std::vector<double> cx_, cy_, cyaw_, ck_;
    bool path_received_ = false;
    bool pose_received_ = false;
    bool final_approach_ = false;
    bool odom_timeout_stop_published_ = false;

    std::chrono::steady_clock::time_point last_odom_time_{};
    std::string odom_topic_;
    std::string final_approach_topic_;

    double current_vx_ = 0.0;
    Pose2D current_pose_;

    // パラメータ
    PurePursuitConfig config_;

    void declareAndGetParameters();
};

} // namespace pure_pursuit_planner
