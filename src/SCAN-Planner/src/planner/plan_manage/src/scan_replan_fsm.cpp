
#include <plan_manage/scan_replan_fsm.h>
#include <cmath>
#include <stdexcept>

namespace
{
  template <typename T>
  T load_parameter(rclcpp::Node *node, const std::string &name, const T &default_value)
  {
    if (!node->has_parameter(name)) node->declare_parameter<T>(name, default_value);
    return node->get_parameter(name).get_value<T>();
  }
} // namespace

namespace scan_planner
{

  void SCANReplanFSM::init(rclcpp::Node *node)
  {
    node_ = node;
    exec_state_ = FSM_EXEC_STATE::INIT;
    trigger_ = false;
    have_target_ = false;
    have_odom_ = false;
    have_new_target_ = false;
    have_end_yaw_ = false;
    rviz_height_ready_ = false;
    go2_execution_frozen_ = false;
    flag_escape_emergency_ = true;
    need_hover_stop_ = false;
    replan_fail_count_ = 0;
    last_freeze_update_time_ = node_->now();

    /*  fsm param  */
    navi_mode_ = load_parameter<int>(node_, "fsm.navi_mode", -1);
    replan_thresh_ = load_parameter<double>(node_, "fsm.thresh_replan", -1.0);
    no_replan_thresh_ = load_parameter<double>(node_, "fsm.thresh_no_replan", -1.0);
    planning_horizon_ = load_parameter<double>(node_, "fsm.planning_horizon", -1.0);
    emergency_time_ = load_parameter<double>(node_, "fsm.emergency_time", 1.0);
    finish_dist_ = load_parameter<double>(node_, "fsm.finish_dist", 0.15);
    finish_yaw_ = load_parameter<double>(node_, "fsm.finish_yaw", 0.10);
    enable_fail_safe_ = load_parameter<bool>(node_, "fsm.fail_safe", true);
    max_replan_fail_count_ = load_parameter<int>(node_, "fsm.max_replan_fail_count", 1000);
    self_inflation_z_up_ = load_parameter<double>(node_, "grid_map.obstacles_inflation_z_up", 0.0);
    self_inflation_z_down_ = load_parameter<double>(node_, "grid_map.obstacles_inflation_z_down", 0.0);
    self_double_cylinder_radius_ = load_parameter<double>(node_, "grid_map.double_cylinder_radius", 0.0);
    self_double_cylinder_offset_ = load_parameter<double>(node_, "grid_map.double_cylinder_offset", 0.0);
    body_height_ = load_parameter<double>(node_, "grid_map.body_height", 0.4);
    self_inflation_frame_id_ = load_parameter<std::string>(node_, "grid_map.frame_id", "world");
    current_map_name_ = load_parameter<std::string>(node_, "map_name", "");

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(node_));
    planner_manager_.reset(new SCANPlannerManager);
    planner_manager_->initPlanModules(node_, visualization_);

    /* callback */
    exec_timer_ = node_->create_wall_timer(std::chrono::milliseconds(10),
                                           std::bind(&SCANReplanFSM::execFSMCallback, this));
    safety_timer_ = node_->create_wall_timer(std::chrono::milliseconds(50),
                                             std::bind(&SCANReplanFSM::checkCollisionCallback, this));
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
        "body_pose", rclcpp::SensorDataQoS(),
        std::bind(&SCANReplanFSM::odometryCallback, this, std::placeholders::_1));
    go2_execution_frozen_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
        "planning/go2_execution_frozen", 10,
        std::bind(&SCANReplanFSM::go2ExecutionFrozenCallback, this, std::placeholders::_1));
    current_map_sub_ = node_->create_subscription<pct_scan_navigation::msg::MapStatus>(
        "/current_map", rclcpp::QoS(1).reliable().transient_local(),
        std::bind(&SCANReplanFSM::currentMapCallback, this, std::placeholders::_1));

    bspline_pub_ = node_->create_publisher<scan_planner_msgs::msg::Bspline>("planning/bspline", 10);
    data_disp_pub_ = node_->create_publisher<scan_planner_msgs::msg::DataDisp>("planning/data_display", 100);
    self_inflation_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
        "self_inflation", rclcpp::QoS(1).reliable().transient_local());
    local_target_pub_ = node_->create_publisher<visualization_msgs::msg::Marker>(
        "/scan_planner/local_target", rclcpp::QoS(1).reliable().transient_local());
    navigation_status_pub_ =
        node_->create_publisher<pct_scan_navigation::msg::NavigationStatus>("/navigation_status", 10);
    navigation_status_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(250),
        std::bind(&SCANReplanFSM::publishNavigationStatus, this));
    reset_navigation_srv_ = node_->create_service<std_srvs::srv::Trigger>(
        "~/reset_navigation",
        std::bind(&SCANReplanFSM::handleResetNavigation, this,
                  std::placeholders::_1, std::placeholders::_2));

    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET)
      goal_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
          "move_base_simple/goal", 1,
          std::bind(&SCANReplanFSM::rvizGoalCallback, this, std::placeholders::_1));
    else if (navi_mode_ == NAVI_MODE::WAYPOINT_PATH)
      waypoint_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
          "waypoints", rclcpp::QoS(1).reliable().transient_local(),
          std::bind(&SCANReplanFSM::dynamicWaypointCallback, this, std::placeholders::_1));
    else if (navi_mode_ == NAVI_MODE::REFERENCE_PATH)
      path_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
          "initial_path", 1, std::bind(&SCANReplanFSM::pathCallback, this, std::placeholders::_1));
    else
      throw std::runtime_error("fsm.navi_mode must be 1, 2, or 3");
  }

  void SCANReplanFSM::rvizGoalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg)
  {
    if (!msg)
      return;

    if (!rviz_height_ready_)
    {
      RCLCPP_WARN(node_->get_logger(), "Ignore RViz goal before receiving initial body pose");
      return;
    }

    auto path = std::make_shared<nav_msgs::msg::Path>();
    path->header = msg->header;
    path->poses.push_back(*msg);
    waypointCallback(path);
  }

  void SCANReplanFSM::waypointCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg)
  {
    if (!msg || msg->poses.empty())
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Empty waypoint message; ignoring");
      return;
    }

    if (msg->poses[0].pose.position.z < -0.1)
      return;

    cout << "Triggered!" << endl;
    navigation_status_reason_ = "ok";
    trigger_ = true;
    init_pt_ = odom_pos_;
    updateGoalYaw(msg->poses[0].pose.orientation, "RViz goal");

    bool success = false;
    end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, rviz_goal_height_;
    success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    if (success)
      success = adjustGlobalTargetIfOccupied();

    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero();
      have_target_ = true;
      have_new_target_ = true;

      /*** FSM ***/
      if (exec_state_ == WAIT_TARGET)
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
      else if (exec_state_ == EXEC_TRAJ || exec_state_ == FINAL_YAW_ALIGN)
        changeFSMExecState(REPLAN_TRAJ, "TRIG");

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory");
    }
  }

  bool SCANReplanFSM::planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints)
  {
    if (waypoints.empty())
    {
      RCLCPP_WARN(node_->get_logger(), "No waypoint supplied for global trajectory");
      return false;
    }

    end_pt_ = waypoints.back();

    for (size_t i = 0; i < waypoints.size(); i++)
    {
      visualization_->displayGoalPoint(waypoints[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
    }

    bool success = planner_manager_->planGlobalTrajWaypoints(
        odom_pos_,
        odom_vel_,
        Eigen::Vector3d::Zero(),
        waypoints,
        Eigen::Vector3d::Zero(),
        Eigen::Vector3d::Zero());

    if (!success)
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory from waypoints");
      return false;
    }

    if (!adjustGlobalTargetIfOccupied())
      return false;

    constexpr double step_size_t = 0.1;
    int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    for (int i = 0; i < i_end; i++)
    {
      gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    }

    end_vel_.setZero();
    have_target_ = true;
    have_new_target_ = true;
    visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, static_cast<int>(waypoints.size()) - 1);

    return true;
  }

  bool SCANReplanFSM::adjustGlobalTargetIfOccupied()
  {
    auto map = planner_manager_->grid_map_;
    auto &global_data = planner_manager_->global_data_;
    const double duration = global_data.global_duration_;
    if (!map || duration < 1e-3)
      return true;

    constexpr double sample_dt = 0.05;
    const int sample_num = std::max(1, static_cast<int>(std::ceil(duration / sample_dt)));
    const Eigen::Vector3d final_pt = global_data.global_traj_.evaluate(duration);
    const Eigen::Vector3d final_prev = global_data.global_traj_.evaluate(duration * (sample_num - 1) / sample_num);
    const int final_occ = map->getInflateOccupancy(final_pt, estimateYawFromSegment(final_prev, final_pt));
    if (final_occ <= 0)
      return true;

    for (int i = sample_num; i >= 0; --i)
    {
      const double t = duration * i / sample_num;
      const double prev_t = duration * std::max(0, i - 1) / sample_num;
      const Eigen::Vector3d pt = global_data.global_traj_.evaluate(t);
      const Eigen::Vector3d prev_pt = global_data.global_traj_.evaluate(prev_t);

      if (map->getInflateOccupancy(pt, estimateYawFromSegment(prev_pt, pt)) == 0)
      {
        const Eigen::Vector3d raw_end = end_pt_;
        end_pt_ = pt;
        global_data.global_duration_ = t;
        global_data.last_progress_time_ = std::min(global_data.last_progress_time_, t);
        RCLCPP_WARN(node_->get_logger(),
                    "Target [%.2f, %.2f, %.2f] is occupied; using [%.2f, %.2f, %.2f]",
                    raw_end(0), raw_end(1), raw_end(2), end_pt_(0), end_pt_(1), end_pt_(2));
        return true;
      }
    }

    RCLCPP_ERROR(node_->get_logger(),
                 "Target is occupied and no collision-free point was found on the global trajectory");
    return false;
  }

  void SCANReplanFSM::pathCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg)
  {
    if (!msg || msg->poses.empty())
    {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 1000,
                           "Received empty initial_path; ignoring");
      return;
    }

    acceptWaypointPath(*msg, body_height_, "Reference path");
  }

  void SCANReplanFSM::dynamicWaypointCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg)
  {
    if (!msg || msg->poses.empty())
    {
      pending_waypoint_path_.reset();
      cancelWaypointNavigation();
      return;
    }
    if (!have_odom_)
    {
      pending_waypoint_path_ = std::make_shared<nav_msgs::msg::Path>(*msg);
      RCLCPP_INFO(node_->get_logger(), "Queue waypoint path until body_pose is available");
      return;
    }
    acceptWaypointPath(*msg, 0.0, "Dynamic waypoint path");
  }

  bool SCANReplanFSM::acceptWaypointPath(const nav_msgs::msg::Path &path, double z_offset,
                                         const std::string &label)
  {
    std::vector<Eigen::Vector3d> waypoints;
    waypoints.reserve(path.poses.size());
    for (const auto &pose_stamped : path.poses)
    {
      const auto &position = pose_stamped.pose.position;
      if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z))
      {
        RCLCPP_WARN(node_->get_logger(), "%s contains a non-finite waypoint", label.c_str());
        return false;
      }
      waypoints.emplace_back(position.x, position.y, position.z + z_offset);
    }

    trigger_ = true;
    navigation_status_reason_ = "ok";
    init_pt_ = odom_pos_;
    updateGoalYaw(path.poses.back().pose.orientation, label);
    if (!planGlobalTrajByWaypoints(waypoints))
    {
      RCLCPP_ERROR(node_->get_logger(), "Unable to generate global trajectory from %s", label.c_str());
      return false;
    }

    active_waypoints_ = std::move(waypoints);
    waypoint_arc_lengths_.clear();
    waypoint_arc_lengths_.reserve(active_waypoints_.size());
    Eigen::Vector3d previous = init_pt_;
    double path_length = 0.0;
    for (const auto &waypoint : active_waypoints_)
    {
      path_length += (waypoint - previous).norm();
      waypoint_arc_lengths_.push_back(path_length);
      previous = waypoint;
    }
    progress_arc_length_ = 0.0;

    if (exec_state_ == WAIT_TARGET || exec_state_ == INIT)
      changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
    else if (exec_state_ == EXEC_TRAJ || exec_state_ == REPLAN_TRAJ ||
             exec_state_ == FINAL_YAW_ALIGN)
      changeFSMExecState(REPLAN_TRAJ, "TRIG");
    RCLCPP_INFO(node_->get_logger(), "%s accepted: %zu guides, final=[%.2f %.2f %.2f]",
                label.c_str(), active_waypoints_.size(),
                end_pt_(0), end_pt_(1), end_pt_(2));
    return true;
  }

  void SCANReplanFSM::cancelWaypointNavigation()
  {
    resetNavigation("canceled");
    RCLCPP_INFO(node_->get_logger(), "Dynamic waypoint path cleared; navigation stopped");
  }

  void SCANReplanFSM::odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg)
  {
    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    if (navi_mode_ == NAVI_MODE::MANUAL_TARGET && !rviz_height_ready_)
    {
      rviz_goal_height_ = odom_pos_(2);
      rviz_height_ready_ = true;
      RCLCPP_INFO(node_->get_logger(), "Set RViz goal height from initial body_pose z: %.3f", rviz_goal_height_);
    }

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    //odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;
    publishSelfInflationMarker();
    if (navi_mode_ == NAVI_MODE::WAYPOINT_PATH && pending_waypoint_path_)
    {
      auto pending = pending_waypoint_path_;
      pending_waypoint_path_.reset();
      dynamicWaypointCallback(pending);
    }
  }

  void SCANReplanFSM::go2ExecutionFrozenCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg)
  {
    go2_execution_frozen_ = msg->data;
  }

  void SCANReplanFSM::currentMapCallback(
      const pct_scan_navigation::msg::MapStatus::ConstSharedPtr &msg)
  {
    if (!msg)
      return;

    current_map_name_ = msg->map_name;
    current_map_state_ = msg->state;
    if (msg->state == pct_scan_navigation::msg::MapStatus::LOADING)
      navigation_status_reason_ = "map_switching";
    else if (msg->state == pct_scan_navigation::msg::MapStatus::FAILED)
      navigation_status_reason_ = "map_failed";
    else if (msg->state == pct_scan_navigation::msg::MapStatus::LOADED)
      navigation_status_reason_ = "map_loaded";
    else
      navigation_status_reason_ = msg->reason.empty() ? "map_unloaded" : msg->reason;

    publishNavigationStatus();
  }

  void SCANReplanFSM::updateLocalTrajTimeFreeze()
  {
    const rclcpp::Time now = node_->now();
    double dt = (now - last_freeze_update_time_).seconds();
    last_freeze_update_time_ = now;

    if (dt <= 0.0 || dt > 0.2)
      return;

    LocalTrajData *info = &planner_manager_->local_data_;
    if (go2_execution_frozen_ && info->start_time_.seconds() > 1e-5)
      info->start_time_ += rclcpp::Duration::from_seconds(dt);
  }

  double SCANReplanFSM::getOdomYaw() const
  {
    Eigen::Vector3d heading = odom_orient_.toRotationMatrix().col(0);
    if (heading.head<2>().squaredNorm() < 1e-8)
      return 0.0;
    return std::atan2(heading(1), heading(0));
  }

  void SCANReplanFSM::updateGoalYaw(const geometry_msgs::msg::Quaternion &orientation,
                                    const std::string &label)
  {
    const double norm = std::hypot(std::hypot(orientation.x, orientation.y),
                                   std::hypot(orientation.z, orientation.w));
    if (!std::isfinite(norm) || norm < 1e-6)
    {
      have_end_yaw_ = false;
      RCLCPP_WARN(node_->get_logger(), "%s has invalid orientation; use position-only finish",
                  label.c_str());
      return;
    }

    const double x = orientation.x / norm;
    const double y = orientation.y / norm;
    const double z = orientation.z / norm;
    const double w = orientation.w / norm;
    end_yaw_ = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
    have_end_yaw_ = std::isfinite(end_yaw_);
  }

  bool SCANReplanFSM::goalReached() const
  {
    if ((odom_pos_ - end_pt_).head<2>().norm() > finish_dist_)
      return false;
    if (!have_end_yaw_)
      return true;

    const double yaw_error = std::atan2(std::sin(end_yaw_ - getOdomYaw()),
                                        std::cos(end_yaw_ - getOdomYaw()));
    return std::abs(yaw_error) <= finish_yaw_;
  }

  double SCANReplanFSM::estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const
  {
    Eigen::Vector2d diff(to(0) - from(0), to(1) - from(1));
    if (diff.squaredNorm() < 1e-8)
      return getOdomYaw();
    return std::atan2(diff(1), diff(0));
  }

  void SCANReplanFSM::publishSelfInflationMarker()
  {
    const double radius = std::max(0.0, self_double_cylinder_radius_);
    const double z_up = std::max(0.0, self_inflation_z_up_);
    const double z_down = std::max(0.0, self_inflation_z_down_);
    const double height = std::max(1e-3, z_up + z_down);

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = self_inflation_frame_id_.empty() ? "world" : self_inflation_frame_id_;
    marker.header.stamp = node_->now();
    marker.ns = "self_inflation";
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 2.0 * radius;
    marker.scale.y = 2.0 * radius;
    marker.scale.z = height;
    marker.color.r = 0.1;
    marker.color.g = 0.6;
    marker.color.b = 1.0;
    marker.color.a = 0.4;
    marker.lifetime = rclcpp::Duration::from_seconds(0.2);

    Eigen::Vector3d center = odom_pos_;
    center(2) += 0.5 * (z_up - z_down);

    Eigen::Vector3d heading(std::cos(getOdomYaw()), std::sin(getOdomYaw()), 0.0);
    Eigen::Vector3d front = center + self_double_cylinder_offset_ * heading;
    Eigen::Vector3d rear = center - self_double_cylinder_offset_ * heading;

    marker.id = 0;
    marker.pose.position.x = front(0);
    marker.pose.position.y = front(1);
    marker.pose.position.z = front(2);
    self_inflation_pub_->publish(marker);

    marker.id = 1;
    marker.pose.position.x = rear(0);
    marker.pose.position.y = rear(1);
    marker.pose.position.z = rear(2);
    self_inflation_pub_->publish(marker);
  }

  void SCANReplanFSM::publishLocalTargetMarker(bool visible)
  {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = self_inflation_frame_id_.empty() ? "map" : self_inflation_frame_id_;
    marker.header.stamp = node_->now();
    marker.ns = "scan_planner_local_target";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visible ? visualization_msgs::msg::Marker::ADD
                            : visualization_msgs::msg::Marker::DELETE;
    marker.pose.orientation.w = 1.0;

    if (visible)
    {
      marker.pose.position.x = local_target_pt_(0);
      marker.pose.position.y = local_target_pt_(1);
      marker.pose.position.z = local_target_pt_(2);
      marker.scale.x = marker.scale.y = marker.scale.z = 0.30;
      marker.color.r = marker.color.g = 1.0;
      marker.color.a = 1.0;
    }

    local_target_pub_->publish(marker);
  }

  void SCANReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continuously_called_times_++;
    else
      continuously_called_times_ = 1;

    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ",
                                  "EXEC_TRAJ", "FINAL_YAW_ALIGN", "EMERGENCY_STOP"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
    publishNavigationStatus();
  }

  std::pair<int, SCANReplanFSM::FSM_EXEC_STATE> SCANReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continuously_called_times_, exec_state_);
  }

  void SCANReplanFSM::printFSMExecState()
  {
    static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ",
                                  "EXEC_TRAJ", "FINAL_YAW_ALIGN", "EMERGENCY_STOP"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }

  void SCANReplanFSM::execFSMCallback()
  {
    updateLocalTrajTimeFreeze();

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!trigger_)
        cout << "wait for goal." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        return;
      }
      if (!trigger_)
      {
        return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_)
        return;
      else
      {
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case GEN_NEW_TRAJ:
    {
      setStartStateFromOdomOrCurrentTraj();

      // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      // start_yaw_(1) = start_yaw_(2) = 0.0;

      bool flag_random_poly_init;
      if (timesOfConsecutiveStateCalls().first == 1)
        flag_random_poly_init = false;
      else
        flag_random_poly_init = true;

      bool success = callReboundReplan(true, flag_random_poly_init);
      if (success)
      {

        replan_fail_count_ = 0;
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
      }
      else
      {
        replan_fail_count_++;
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {
      if ((odom_pos_ - end_pt_).head<2>().norm() <= finish_dist_)
      {
        replan_fail_count_ = 0;
        changeFSMExecState(FINAL_YAW_ALIGN, "FSM");
        break;
      }

      if (planFromCurrentTraj())
      {
        replan_fail_count_ = 0;
        changeFSMExecState(EXEC_TRAJ, "FSM");
      }
      else
      {
        replan_fail_count_++;
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      rclcpp::Time time_now = node_->now();
      double t_cur = (time_now - info->start_time_).seconds();
      t_cur = min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      /* && (end_pt_ - pos).norm() < 0.5 */
      if (t_cur > info->duration_ - 1e-2)
      {
        if ((odom_pos_ - end_pt_).head<2>().norm() > finish_dist_)
        {
          changeFSMExecState(REPLAN_TRAJ, "FSM");
          return;
        }
        changeFSMExecState(FINAL_YAW_ALIGN, "FSM");
        return;
      }
      else if ((end_pt_ - pos).norm() < no_replan_thresh_)
      {
        // cout << "near end" << endl;
        return;
      }
      else if ((info->start_pos_ - pos).norm() < replan_thresh_)
      {
        // cout << "near start" << endl;
        return;
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }
      break;
    }

    case FINAL_YAW_ALIGN:
    {
      const double position_error = (odom_pos_ - end_pt_).head<2>().norm();
      if (position_error > finish_dist_ + no_replan_thresh_)
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
        return;
      }
      if (!goalReached())
        return;

      navigation_status_reason_ = "goal_reached";
      publishNavigationStatus();
      active_waypoints_.clear();
      waypoint_arc_lengths_.clear();
      progress_arc_length_ = 0.0;
      have_target_ = false;
      have_end_yaw_ = false;
      publishLocalTargetMarker(false);
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case EMERGENCY_STOP:
    {

      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (enable_fail_safe_ && !need_hover_stop_ && odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        else if (enable_fail_safe_ && need_hover_stop_ && odom_vel_.norm() < 0.1)
        {
          RCLCPP_INFO(node_->get_logger(),
                      "Exiting EMERGENCY_STOP; switching to WAIT_TARGET for a new target");
          need_hover_stop_ = false;
          have_target_ = false;
          trigger_ = false;
          changeFSMExecState(WAIT_TARGET, "EMERGENCY_EXIT");
        }
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    finishProcess();

    data_disp_.header.stamp = node_->now();
    data_disp_pub_->publish(data_disp_);
  }

  void SCANReplanFSM::finishProcess()
  {
    if (replan_fail_count_ >= max_replan_fail_count_)
    {
      RCLCPP_WARN(node_->get_logger(),
                  "Replan failed %d times; emergency stop and wait for a new target", replan_fail_count_);
      replan_fail_count_ = 0;
      need_hover_stop_ = true;
      flag_escape_emergency_ = true;
      navigation_status_reason_ = "replan_failed";
      changeFSMExecState(EMERGENCY_STOP, "finishProcess");
    }
  }

  uint8_t SCANReplanFSM::navigationStateFromFSM() const
  {
    using Status = pct_scan_navigation::msg::NavigationStatus;
    if (current_map_state_ == pct_scan_navigation::msg::MapStatus::LOADING)
      return Status::MAP_SWITCHING;
    if (current_map_state_ == pct_scan_navigation::msg::MapStatus::FAILED)
      return Status::FAILED;
    if (navigation_status_reason_ == "goal_reached")
      return Status::GOAL_REACHED;
    if (navigation_status_reason_ == "canceled")
      return Status::CANCELED;

    switch (exec_state_)
    {
    case INIT:
      return Status::IDLE;
    case WAIT_TARGET:
      return have_target_ ? Status::PLANNING_LOCAL : Status::WAITING_GOAL;
    case GEN_NEW_TRAJ:
      return Status::PLANNING_LOCAL;
    case REPLAN_TRAJ:
      return Status::AVOIDING;
    case EXEC_TRAJ:
    case FINAL_YAW_ALIGN:
      return Status::NAVIGATING;
    case EMERGENCY_STOP:
      return Status::BLOCKED;
    }
    return Status::FAILED;
  }

  double SCANReplanFSM::distanceToGoal() const
  {
    if (!have_target_ && active_waypoints_.empty())
      return 0.0;
    return (odom_pos_ - end_pt_).head<2>().norm();
  }

  uint32_t SCANReplanFSM::remainingWaypointCount() const
  {
    const auto first_remaining = std::upper_bound(
        waypoint_arc_lengths_.begin(), waypoint_arc_lengths_.end(),
        progress_arc_length_ + 1e-3);
    return static_cast<uint32_t>(
        std::distance(first_remaining, waypoint_arc_lengths_.end()));
  }

  void SCANReplanFSM::publishNavigationStatus()
  {
    if (!navigation_status_pub_)
      return;
    pct_scan_navigation::msg::NavigationStatus msg;
    msg.header.stamp = node_->now();
    msg.header.frame_id = self_inflation_frame_id_.empty() ? "map" : self_inflation_frame_id_;
    msg.state = navigationStateFromFSM();
    msg.map_name = current_map_name_;
    msg.goal_active = have_target_;
    msg.distance_to_goal = static_cast<float>(distanceToGoal());
    msg.remaining_waypoints = remainingWaypointCount();
    if (current_map_state_ == pct_scan_navigation::msg::MapStatus::LOADING)
      msg.reason = "map_switching";
    else if (current_map_state_ == pct_scan_navigation::msg::MapStatus::FAILED)
      msg.reason = navigation_status_reason_.empty() ? "map_failed" : navigation_status_reason_;
    else if (navigation_status_reason_.empty() || navigation_status_reason_ == "goal_reached")
      msg.reason = navigation_status_reason_.empty() ? "ok" : navigation_status_reason_;
    else
      msg.reason = navigation_status_reason_;
    navigation_status_pub_->publish(msg);

    if (navigation_status_reason_ == "goal_reached" && !have_target_)
      navigation_status_reason_ = "ok";
  }

  void SCANReplanFSM::resetNavigation(const std::string &reason)
  {
    const bool was_active = have_target_ || !active_waypoints_.empty();
    active_waypoints_.clear();
    waypoint_arc_lengths_.clear();
    progress_arc_length_ = 0.0;
    pending_waypoint_path_.reset();
    have_target_ = false;
    have_new_target_ = false;
    trigger_ = false;
    replan_fail_count_ = 0;
    need_hover_stop_ = false;
    have_end_yaw_ = false;
    navigation_status_reason_ = reason;
    publishLocalTargetMarker(false);
    if (was_active && have_odom_)
      callEmergencyStop(odom_pos_);
    changeFSMExecState(WAIT_TARGET, "RESET");
    publishNavigationStatus();
  }

  void SCANReplanFSM::handleResetNavigation(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    resetNavigation("soft_reset");
    response->success = true;
    response->message = "navigation reset";
  }

  bool SCANReplanFSM::planFromCurrentTraj()
  {
    setStartStateFromOdomOrCurrentTraj();

    if (navi_mode_ != NAVI_MODE::WAYPOINT_PATH)
    {
      const Eigen::Vector2d to_goal =
          end_pt_.head<2>() - odom_pos_.head<2>();
      if (to_goal.norm() > 1e-3 &&
          start_vel_.head<2>().dot(to_goal) < 0.0)
      {
        start_vel_.setZero();
        start_acc_.setZero();
      }

      const bool global_success = planner_manager_->planGlobalTraj(
          start_pt_, start_vel_, start_acc_, end_pt_,
          Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
      if (!global_success)
      {
        RCLCPP_ERROR(node_->get_logger(),
                     "[navi_mode=%d] Unable to refresh global trajectory from odom to current target",
                     navi_mode_);
        return false;
      }
      if (!adjustGlobalTargetIfOccupied())
        return false;
    }
    else if (planner_manager_->global_data_.global_duration_ <= 1e-3)
    {
      RCLCPP_ERROR(node_->get_logger(), "Mode 2 has no valid global reference trajectory");
      return false;
    }

    bool success = callReboundReplan(true, false);
    if (!success)
    {
      success = callReboundReplan(true, true);
      if (!success)
        return false;
    }

    return true;
  }

  void SCANReplanFSM::setStartStateFromOdomOrCurrentTraj()
  {
    start_pt_ = odom_pos_;
    start_vel_ = odom_vel_;
    start_acc_.setZero();

    LocalTrajData *info = &planner_manager_->local_data_;
    if (info->start_time_.seconds() < 1e-5 || info->duration_ <= 1e-5)
      return;

    const double raw_t_cur = (node_->now() - info->start_time_).seconds();
    if (raw_t_cur < -1e-3 || raw_t_cur > info->duration_ + 0.2)
      return;

    const double t_cur = std::min(std::max(raw_t_cur, 0.0), info->duration_);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);
  }

  void SCANReplanFSM::checkCollisionCallback()
  {
    updateLocalTrajTimeFreeze();

    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;

    if (exec_state_ == WAIT_TARGET || info->start_time_.seconds() < 1e-5)
      return;

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    double t_cur = (node_->now() - info->start_time_).seconds();
    double t_2_3 = info->duration_ * 2 / 3;
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      if (t_cur < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        break;

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t);
      Eigen::Vector3d pos_next = info->position_traj_.evaluateDeBoorT(std::min(t + time_step, info->duration_));
      if (map->getInflateOccupancy(pos, estimateYawFromSegment(pos, pos_next)))
      {
        if (planFromCurrentTraj()) // Make a chance
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          return;
        }
        else
        {
          if (t - t_cur < emergency_time_) // 0.8s of emergency time
          {
            RCLCPP_WARN(node_->get_logger(), "Obstacle discovered; emergency stop in %.3fs", t - t_cur);
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else
          {
            //ROS_WARN("current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
        break;
      }
    }
  }

  bool SCANReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    if (!getLocalTarget())
      return false;

    bool plan_success =
        planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
    have_new_target_ = false;

    cout << "final_plan_success=" << plan_success << endl;

    if (plan_success)
    {

      auto info = &planner_manager_->local_data_;

      /* publish traj */
      scan_planner_msgs::msg::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::msg::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      if (have_end_yaw_)
        bspline.yaw_pts.push_back(end_yaw_);

      bspline_pub_->publish(bspline);

      visualization_->displayOptimalTraj(info->position_traj_, 0);
    }

    return plan_success;
  }

  bool SCANReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    scan_planner_msgs::msg::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::msg::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_->publish(bspline);

    return true;
  }

  bool SCANReplanFSM::getLocalTarget()
  {
    auto &global = planner_manager_->global_data_;
    const double duration = global.global_duration_;
    const double max_vel = planner_manager_->pp_.max_vel_;
    const double lookahead =
        std::min(planning_horizon_, planner_manager_->pp_.planning_horizon_);
    if (!std::isfinite(duration) || duration <= 1e-3 ||
        !std::isfinite(max_vel) || max_vel <= 1e-3 ||
        !std::isfinite(lookahead) || lookahead <= 1e-3)
    {
      RCLCPP_ERROR(node_->get_logger(),
                   "Invalid global trajectory or local-target limits: duration=%.3f "
                   "max_vel=%.3f lookahead=%.3f",
                   duration, max_vel, lookahead);
      return false;
    }

    const double time_step = std::clamp(
        planning_horizon_ / (40.0 * max_vel), 0.01, 0.10);
    const double previous_progress =
        std::clamp(global.last_progress_time_, 0.0, duration);
    double projection_time = previous_progress;
    double projection_arc = 0.0;
    double best_distance_sq =
        (global.global_traj_.evaluate(previous_progress) - odom_pos_).squaredNorm();

    double segment_start_time = previous_progress;
    Eigen::Vector3d segment_start = global.global_traj_.evaluate(segment_start_time);
    double searched_arc = 0.0;
    while (segment_start_time < duration - 1e-6 &&
           searched_arc < planning_horizon_ - 1e-6)
    {
      const double raw_end_time = std::min(duration, segment_start_time + time_step);
      const Eigen::Vector3d raw_end = global.global_traj_.evaluate(raw_end_time);
      const double raw_length = (raw_end - segment_start).norm();
      if (raw_length <= 1e-9)
      {
        segment_start_time = raw_end_time;
        segment_start = raw_end;
        continue;
      }

      const double allowed_length =
          std::min(raw_length, planning_horizon_ - searched_arc);
      const double allowed_ratio = allowed_length / raw_length;
      const double segment_end_time =
          segment_start_time + allowed_ratio * (raw_end_time - segment_start_time);
      const Eigen::Vector3d segment_end =
          segment_start + allowed_ratio * (raw_end - segment_start);
      const Eigen::Vector3d segment = segment_end - segment_start;
      const double segment_length_sq = segment.squaredNorm();
      const double ratio = std::clamp(
          (odom_pos_ - segment_start).dot(segment) / segment_length_sq, 0.0, 1.0);
      const Eigen::Vector3d projected = segment_start + ratio * segment;
      const double distance_sq = (projected - odom_pos_).squaredNorm();
      if (distance_sq < best_distance_sq)
      {
        best_distance_sq = distance_sq;
        projection_time =
            segment_start_time + ratio * (segment_end_time - segment_start_time);
        projection_arc = searched_arc + ratio * allowed_length;
      }

      searched_arc += allowed_length;
      segment_start_time = segment_end_time;
      segment_start = segment_end;
      if (allowed_ratio < 1.0)
        break;
    }

    global.last_progress_time_ = projection_time;
    progress_arc_length_ += projection_arc;

    const Eigen::Vector3d reference_velocity =
        global.global_traj_.evaluateVel(projection_time);
    if (reference_velocity.norm() > 1e-3 &&
        start_vel_.dot(reference_velocity) < 0.0)
    {
      start_vel_.setZero();
      start_acc_.setZero();
    }

    double target_time = projection_time;
    local_target_pt_ = global.global_traj_.evaluate(projection_time);
    double target_arc = 0.0;
    segment_start_time = projection_time;
    segment_start = local_target_pt_;
    while (segment_start_time < duration - 1e-6 &&
           target_arc < lookahead - 1e-6)
    {
      const double segment_end_time =
          std::min(duration, segment_start_time + time_step);
      const Eigen::Vector3d segment_end =
          global.global_traj_.evaluate(segment_end_time);
      const double segment_length = (segment_end - segment_start).norm();
      if (segment_length <= 1e-9)
      {
        segment_start_time = segment_end_time;
        segment_start = segment_end;
        continue;
      }

      if (target_arc + segment_length >= lookahead)
      {
        const double ratio = (lookahead - target_arc) / segment_length;
        target_time =
            segment_start_time + ratio * (segment_end_time - segment_start_time);
        local_target_pt_ =
            segment_start + ratio * (segment_end - segment_start);
        target_arc = lookahead;
        break;
      }

      target_arc += segment_length;
      target_time = segment_end_time;
      local_target_pt_ = segment_end;
      segment_start_time = segment_end_time;
      segment_start = segment_end;
    }
    if (target_time >= duration - 1e-6)
    {
      target_time = duration;
      local_target_pt_ = end_pt_;
    }

    auto map = planner_manager_->grid_map_;
    if (map && map->isInMap(local_target_pt_))
    {
      const Eigen::Vector3d target_previous =
          global.global_traj_.evaluate(std::max(projection_time, target_time - time_step));
      if (map->getInflateOccupancy(
              local_target_pt_,
              estimateYawFromSegment(target_previous, local_target_pt_)) > 0)
      {
        bool found_free_target = false;
        for (double candidate_time = target_time;
             candidate_time >= projection_time - 1e-6;
             candidate_time -= time_step)
        {
          const double clamped_time =
              std::max(projection_time, candidate_time);
          const Eigen::Vector3d candidate =
              global.global_traj_.evaluate(clamped_time);
          if (!map->isInMap(candidate))
            continue;
          const Eigen::Vector3d previous =
              global.global_traj_.evaluate(
                  std::max(projection_time, clamped_time - time_step));
          if (map->getInflateOccupancy(
                  candidate, estimateYawFromSegment(previous, candidate)) == 0)
          {
            local_target_pt_ = candidate;
            target_time = clamped_time;
            found_free_target = true;
            break;
          }
          if (clamped_time <= projection_time + 1e-6)
            break;
        }

        if (found_free_target)
        {
          RCLCPP_WARN_THROTTLE(
              node_->get_logger(), *node_->get_clock(), 1000,
              "Occupied local target moved backward along the global reference");
        }
        else
        {
          RCLCPP_WARN_THROTTLE(
              node_->get_logger(), *node_->get_clock(), 1000,
              "Local target is occupied and no earlier free target was found");
        }
      }
    }

    local_target_vel_ = global.global_traj_.evaluateVel(target_time);
    const double target_speed = local_target_vel_.norm();
    if (target_speed > max_vel)
      local_target_vel_ *= max_vel / target_speed;
    publishLocalTargetMarker(true);
    return true;
  }

} // namespace scan_planner
