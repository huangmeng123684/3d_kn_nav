#ifndef PCT_SCAN_NAVIGATION__WAYPOINT_UTILS_HPP_
#define PCT_SCAN_NAVIGATION__WAYPOINT_UTILS_HPP_

#include <cstdint>
#include <string>

#include <nav_msgs/msg/path.hpp>

namespace pct_scan_navigation
{

bool sampleWaypoints(
    const nav_msgs::msg::Path &input,
    const std::string &global_frame,
    double spacing,
    double z_offset,
    nav_msgs::msg::Path &output,
    std::uint64_t &signature,
    std::string &reason);

}  // namespace pct_scan_navigation

#endif  // PCT_SCAN_NAVIGATION__WAYPOINT_UTILS_HPP_
