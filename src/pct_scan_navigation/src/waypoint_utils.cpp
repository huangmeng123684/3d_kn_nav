#include "pct_scan_navigation/waypoint_utils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

namespace
{

constexpr double kEpsilon = 1e-9;

bool finitePoint(const geometry_msgs::msg::Point &point)
{
  return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
}

double distance3D(const geometry_msgs::msg::Point &a, const geometry_msgs::msg::Point &b)
{
  return std::hypot(std::hypot(a.x - b.x, a.y - b.y), a.z - b.z);
}

void mixHash(std::uint64_t &hash, std::int64_t value)
{
  hash ^= static_cast<std::uint64_t>(value);
  hash *= 1099511628211ULL;
}

}  // namespace

namespace pct_scan_navigation
{

bool sampleWaypoints(
    const nav_msgs::msg::Path &input,
    const std::string &global_frame,
    double spacing,
    double z_offset,
    nav_msgs::msg::Path &output,
    std::uint64_t &signature,
    std::string &reason)
{
  output = nav_msgs::msg::Path();
  signature = 0;
  reason.clear();

  if (input.poses.empty())
  {
    reason = "path is empty";
    return false;
  }
  if (input.header.frame_id != global_frame)
  {
    reason = "path frame must be " + global_frame;
    return false;
  }
  if (!std::isfinite(spacing) || spacing <= 0.0 || !std::isfinite(z_offset))
  {
    reason = "spacing must be positive and z offset must be finite";
    return false;
  }

  signature = 1469598103934665603ULL;
  for (const auto &pose : input.poses)
  {
    if ((!pose.header.frame_id.empty() && pose.header.frame_id != global_frame) ||
        !finitePoint(pose.pose.position))
    {
      reason = "path contains an invalid pose or frame";
      return false;
    }
    mixHash(signature, std::llround(pose.pose.position.x * 1000.0));
    mixHash(signature, std::llround(pose.pose.position.y * 1000.0));
    mixHash(signature, std::llround(pose.pose.position.z * 1000.0));
  }
  const auto &final_orientation = input.poses.back().pose.orientation;
  if (!std::isfinite(final_orientation.x) || !std::isfinite(final_orientation.y) ||
      !std::isfinite(final_orientation.z) || !std::isfinite(final_orientation.w))
  {
    reason = "final orientation is invalid";
    return false;
  }
  mixHash(signature, std::llround(final_orientation.x * 1000.0));
  mixHash(signature, std::llround(final_orientation.y * 1000.0));
  mixHash(signature, std::llround(final_orientation.z * 1000.0));
  mixHash(signature, std::llround(final_orientation.w * 1000.0));
  mixHash(signature, static_cast<std::int64_t>(input.poses.size()));

  std::vector<double> segment_lengths;
  segment_lengths.reserve(input.poses.size() > 1 ? input.poses.size() - 1 : 0);
  double total_length = 0.0;
  for (std::size_t i = 1; i < input.poses.size(); ++i)
  {
    const double length = distance3D(
        input.poses[i - 1].pose.position, input.poses[i].pose.position);
    segment_lengths.push_back(length);
    total_length += length;
  }

  output.header = input.header;
  output.header.frame_id = global_frame;
  std::size_t segment_index = 0;
  double segment_start_distance = 0.0;
  for (double target_distance = spacing;
       target_distance < total_length - kEpsilon;
       target_distance += spacing)
  {
    while (segment_index < segment_lengths.size() &&
           segment_start_distance + segment_lengths[segment_index] < target_distance - kEpsilon)
    {
      segment_start_distance += segment_lengths[segment_index];
      ++segment_index;
    }
    while (segment_index < segment_lengths.size() && segment_lengths[segment_index] <= kEpsilon)
    {
      segment_start_distance += segment_lengths[segment_index];
      ++segment_index;
    }
    if (segment_index >= segment_lengths.size())
      break;

    const auto &start = input.poses[segment_index].pose.position;
    const auto &end = input.poses[segment_index + 1].pose.position;
    const double ratio = std::clamp(
        (target_distance - segment_start_distance) / segment_lengths[segment_index], 0.0, 1.0);

    geometry_msgs::msg::PoseStamped sample;
    sample.header = output.header;
    sample.pose.position.x = start.x + ratio * (end.x - start.x);
    sample.pose.position.y = start.y + ratio * (end.y - start.y);
    sample.pose.position.z = start.z + ratio * (end.z - start.z) + z_offset;
    sample.pose.orientation.w = 1.0;
    output.poses.push_back(sample);
  }

  geometry_msgs::msg::PoseStamped final_pose = input.poses.back();
  final_pose.header = output.header;
  final_pose.pose.position.z += z_offset;
  output.poses.push_back(final_pose);
  return true;
}

}  // namespace pct_scan_navigation
