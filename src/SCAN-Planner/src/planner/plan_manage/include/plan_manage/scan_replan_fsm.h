#ifndef _SCAN_REPLAN_FSM_H_
#define _SCAN_REPLAN_FSM_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <iostream>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <vector>
#include <visualization_msgs/msg/marker.hpp>
#include <pct_scan_navigation/msg/map_status.hpp>
#include <pct_scan_navigation/msg/navigation_status.hpp>

#include <bspline_opt/bspline_optimizer.h>
#include <plan_env/grid_map.h>
#include <scan_planner_msgs/msg/bspline.hpp>
#include <scan_planner_msgs/msg/data_disp.hpp>
#include <plan_manage/planner_manager.h>
#include <traj_utils/planning_visualization.h>

using std::vector;

namespace scan_planner
{

  class SCANReplanFSM
  {

  private:
    /* ---------- flag ---------- */
    enum FSM_EXEC_STATE
    {
      INIT,
      WAIT_TARGET,
      GEN_NEW_TRAJ,
      REPLAN_TRAJ,
      EXEC_TRAJ,
      FINAL_YAW_ALIGN,
      EMERGENCY_STOP
    };
    enum NAVI_MODE
    {
      MANUAL_TARGET = 1,
      WAYPOINT_PATH = 2,
      REFERENCE_PATH = 3,
    };

    /* planning utils */
    SCANPlannerManager::Ptr planner_manager_;
    PlanningVisualization::Ptr visualization_;
    scan_planner_msgs::msg::DataDisp data_disp_;

    /* parameters */
    int navi_mode_;
    double no_replan_thresh_, replan_thresh_;
    double planning_horizon_;
    double emergency_time_;
    double finish_dist_, finish_yaw_;
    double rviz_goal_height_;
    double self_inflation_z_up_, self_inflation_z_down_;
    double self_double_cylinder_radius_, self_double_cylinder_offset_;
    double body_height_;
    std::string self_inflation_frame_id_;

    /* planning data */
    bool trigger_, have_target_, have_odom_, have_new_target_;
    bool have_end_yaw_;
    bool rviz_height_ready_;
    bool go2_execution_frozen_;
    bool enable_fail_safe_, need_hover_stop_;
    FSM_EXEC_STATE exec_state_;
    int continuously_called_times_{0};
    int replan_fail_count_{0};
    int max_replan_fail_count_{1000};
    rclcpp::Time last_freeze_update_time_;

    Eigen::Vector3d odom_pos_, odom_vel_, odom_acc_; // odometry state
    Eigen::Quaterniond odom_orient_;

    Eigen::Vector3d init_pt_, start_pt_, start_vel_, start_acc_, start_yaw_; // start state
    Eigen::Vector3d end_pt_, end_vel_;                                       // goal state
    double end_yaw_{0.0};
    Eigen::Vector3d local_target_pt_, local_target_vel_;                     // local target state
    std::vector<Eigen::Vector3d> active_waypoints_;
    std::vector<double> waypoint_arc_lengths_;
    double progress_arc_length_{0.0};
    nav_msgs::msg::Path::SharedPtr pending_waypoint_path_;
    std::string current_map_name_;
    uint8_t current_map_state_{pct_scan_navigation::msg::MapStatus::UNLOADED};
    std::string navigation_status_reason_{"startup"};

    bool flag_escape_emergency_;

    /* ROS utils */
    rclcpp::Node *node_{nullptr};
    rclcpp::TimerBase::SharedPtr exec_timer_, safety_timer_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr waypoint_sub_, path_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr go2_execution_frozen_sub_;
    rclcpp::Subscription<pct_scan_navigation::msg::MapStatus>::SharedPtr current_map_sub_;
    rclcpp::Publisher<scan_planner_msgs::msg::Bspline>::SharedPtr bspline_pub_;
    rclcpp::Publisher<scan_planner_msgs::msg::DataDisp>::SharedPtr data_disp_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr self_inflation_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr local_target_pub_;
    rclcpp::Publisher<pct_scan_navigation::msg::NavigationStatus>::SharedPtr navigation_status_pub_;
    rclcpp::TimerBase::SharedPtr navigation_status_timer_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_navigation_srv_;

    /* helper functions */
    bool callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj); // front-end and back-end method
    bool callEmergencyStop(Eigen::Vector3d stop_pos);                          // front-end and back-end method
    bool planFromCurrentTraj();
    void setStartStateFromOdomOrCurrentTraj();

    /* return value: std::pair< Times of the same state be continuously called, current continuously called state > */
    void changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call);
    std::pair<int, SCANReplanFSM::FSM_EXEC_STATE> timesOfConsecutiveStateCalls();
    void printFSMExecState();

    bool planGlobalTrajByWaypoints(const std::vector<Eigen::Vector3d> &waypoints);
    bool acceptWaypointPath(const nav_msgs::msg::Path &path, double z_offset,
                            const std::string &label);
    void cancelWaypointNavigation();
    bool adjustGlobalTargetIfOccupied();
    bool getLocalTarget();
    uint32_t remainingWaypointCount() const;
    void finishProcess();
    void publishSelfInflationMarker();
    void publishLocalTargetMarker(bool visible);
    void updateGoalYaw(const geometry_msgs::msg::Quaternion &orientation,
                       const std::string &label);
    bool goalReached() const;
    double getOdomYaw() const;
    double estimateYawFromSegment(const Eigen::Vector3d &from, const Eigen::Vector3d &to) const;
    void updateLocalTrajTimeFreeze();
    void publishNavigationStatus();
    uint8_t navigationStateFromFSM() const;
    double distanceToGoal() const;
    void resetNavigation(const std::string &reason);
    void handleResetNavigation(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                               std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    /* ROS functions */
    void execFSMCallback();
    void checkCollisionCallback();
    void rvizGoalCallback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &msg);
    void waypointCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg);
    void dynamicWaypointCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg);
    void pathCallback(const nav_msgs::msg::Path::ConstSharedPtr &msg);
    void odometryCallback(const nav_msgs::msg::Odometry::ConstSharedPtr &msg);
    void go2ExecutionFrozenCallback(const std_msgs::msg::Bool::ConstSharedPtr &msg);
    void currentMapCallback(const pct_scan_navigation::msg::MapStatus::ConstSharedPtr &msg);

    bool checkCollision();

  public:
    SCANReplanFSM(/* args */)
    {
    }
    ~SCANReplanFSM()
    {
    }

    void init(rclcpp::Node *node);

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };

} // namespace scan_planner

#endif
