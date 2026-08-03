#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>

#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "pct_scan_navigation/waypoint_utils.hpp"

namespace
{

class PctScanCoordinator : public rclcpp::Node
{
public:
  PctScanCoordinator() : Node("pct_scan_coordinator")
  {
    mode_ = declare_parameter<int>("mode", 2);
    global_frame_ = declare_parameter<std::string>("global_frame", "map");
    path_topic_ = declare_parameter<std::string>("path_topic", "/pct_path");
    waypoints_topic_ =
        declare_parameter<std::string>("waypoints_topic", "/scan_planner/waypoints");
    waypoint_spacing_ = declare_parameter<double>("waypoint_spacing", 1.0);
    waypoint_z_offset_ = declare_parameter<double>("waypoint_z_offset", 0.0);

    if (mode_ == 3)
      throw std::runtime_error("navigation mode 3 is not implemented");
    if (mode_ != 1 && mode_ != 2)
      throw std::runtime_error("mode must be 1 or 2");
    if (!std::isfinite(waypoint_spacing_) || waypoint_spacing_ <= 0.0 ||
        !std::isfinite(waypoint_z_offset_))
      throw std::runtime_error(
          "waypoint_spacing must be positive and waypoint_z_offset must be finite");

    reset_route_srv_ = create_service<std_srvs::srv::Trigger>(
        "~/reset_route",
        std::bind(&PctScanCoordinator::handleResetRoute, this,
                  std::placeholders::_1, std::placeholders::_2));

    if (mode_ == 1)
    {
      RCLCPP_INFO(get_logger(),
                  "Mode 1: coordinator idle; SCAN-Planner receives /goal_pose directly");
      return;
    }

    auto latched = rclcpp::QoS(1).reliable().transient_local();
    waypoints_pub_ = create_publisher<nav_msgs::msg::Path>(waypoints_topic_, latched);
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        path_topic_, latched,
        std::bind(&PctScanCoordinator::pathCallback, this, std::placeholders::_1));

    publishEmptyPath();
    RCLCPP_INFO(get_logger(),
                "Mode 2 ready: path=%s waypoints=%s spacing=%.2f z_offset=%.2f",
                path_topic_.c_str(), waypoints_topic_.c_str(),
                waypoint_spacing_, waypoint_z_offset_);
  }

private:
  void pathCallback(const nav_msgs::msg::Path::ConstSharedPtr message)
  {
    if (!message || message->poses.empty())
    {
      clearRoute("received empty PCT path");
      return;
    }

    nav_msgs::msg::Path sampled;
    std::uint64_t signature = 0;
    std::string reason;
    if (!pct_scan_navigation::sampleWaypoints(
            *message, global_frame_, waypoint_spacing_, waypoint_z_offset_,
            sampled, signature, reason))
    {
      RCLCPP_WARN(get_logger(), "Reject PCT path: %s", reason.c_str());
      return;
    }
    if (route_active_ && signature == path_signature_)
    {
      RCLCPP_DEBUG(get_logger(), "Ignore unchanged PCT path");
      return;
    }

    path_signature_ = signature;
    route_active_ = true;

    publishWaypointPath(sampled);
    RCLCPP_INFO(get_logger(), "Accepted PCT path: input=%zu sampled=%zu",
                message->poses.size(), sampled.poses.size());
  }

  void publishWaypointPath(nav_msgs::msg::Path &output)
  {
    output.header.stamp = now();
    output.header.frame_id = global_frame_;
    for (auto &pose : output.poses)
      pose.header = output.header;
    waypoints_pub_->publish(output);
  }

  void publishEmptyPath()
  {
    nav_msgs::msg::Path output;
    output.header.stamp = now();
    output.header.frame_id = global_frame_;
    waypoints_pub_->publish(output);
  }

  void clearRoute(const std::string &reason)
  {
    route_active_ = false;
    path_signature_ = 0;
    publishEmptyPath();
    RCLCPP_WARN(get_logger(), "%s; clear SCAN waypoint route", reason.c_str());
  }

  void handleResetRoute(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    if (mode_ == 2)
      clearRoute("route reset requested");
    response->success = true;
    response->message = mode_ == 2 ? "route cleared" : "coordinator idle";
  }

  int mode_{2};
  std::string global_frame_, path_topic_, waypoints_topic_;
  double waypoint_spacing_{1.0}, waypoint_z_offset_{0.0};
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr waypoints_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_route_srv_;
  std::uint64_t path_signature_{0};
  bool route_active_{false};
};

}  // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  try
  {
    rclcpp::spin(std::make_shared<PctScanCoordinator>());
  }
  catch (const std::exception &error)
  {
    RCLCPP_FATAL(rclcpp::get_logger("pct_scan_coordinator"), "%s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
