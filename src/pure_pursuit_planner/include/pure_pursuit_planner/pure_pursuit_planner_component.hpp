// Directory: pure_pursuit_planner/include/pure_pursuit_planner/pure_pursuit_component.hpp
#pragma once

#include <vector>
#include <utility>

namespace pure_pursuit_planner {

struct Pose2D {
    double x;
    double y;
    double yaw;
};

struct PurePursuitConfig {
    double k = 0.5;
    double Lfc = 0.8;
    double Kp = 1.0;
    double dt = 0.1;
    double goal_threshold = 0.4;
    double final_heading_entry_distance = 0.20;
    double final_heading_command_deadband = 0.02;
    double min_final_angular_velocity = 0.20;
    double max_acceleration = 0.08;
    double minCurvature = 0.0;
    double maxCurvature = 3.0;
    double minVelocity = 0.4;
    double maxVelocity = 0.7;
    double maxAngularVelocity = 1.3;
    double rotate_to_path_threshold = 1.047;
    double rotate_to_path_tolerance = 0.349;
    double goal_yaw_tolerance = 0.175;
    double rotate_to_heading_gain = 1.0;
    bool standalone_goal_completion = false;
    double obstacle_th = 0.5;
    double odom_timeout = 0.3;
};

class PurePursuitComponent {
public:
    PurePursuitComponent(const PurePursuitConfig& config);

    void setPath(const std::vector<double>& cx,
                 const std::vector<double>& cy,
                 const std::vector<double>& cyaw,
                 const std::vector<double>& ck);

    void setPose(const Pose2D& pose, double velocity);

    std::vector<double> computeVelocity(
        const std::vector<double>& cx,
        const std::vector<double>& cy,
        const std::vector<double>& cyaw,
        const std::vector<double>& ck,
        const Pose2D& pose, 
        double velocity,
        bool final_approach = false
    );

    double calcLf(double k, double current_velocity, double Lfc) const;

    bool odom_sub_flag = false;

    int oldNearestPointIndex = -1;

    int calcOldNearestPointIndex() const;

    int calcFirstNearestPointIndex() const;

    std::pair<double, double> isGoalReached(double v, double w) const;

    std::pair<int, double> searchTargetIndex();

    double curvatureToVelocity(double curvature) const;

    double alphaExceptionHandling(double tempAlpha) const;

    double calculateAngularVelocity(double v, double alpha, double Lf) const;

private:
    double calcDistance(double x1, double y1, double x2, double y2) const;
    double normalizeAngle(double angle) const;
    double calculateFinalRotationAngularVelocity(double yaw_error) const;
    double calculateRotationAngularVelocity(double yaw_error) const;
    std::pair<double, std::pair<double, double>> calcClosestPointOnPath();
    std::pair<double, double> calcAcceleration(double current_vel);

    // Config and state
    PurePursuitConfig cfg_;
    Pose2D current_pose_;
    double current_velocity_ = 0.0;

    std::vector<double> cx_, cy_, cyaw_, ck_;
    int oldNearestPointIndex_ = -1;
    int targetIndex_ = 0;

    double obstacle_x_ = 0.0;
    double obstacle_y_ = 0.0;
    bool obstacle_detected_ = false;
    bool avoidance_flag_ = false;

    double previous_velocity_ = 0.0;
    double previous_time_ = 0.0;
    bool goal_reached_ = false;
    bool rotating_to_path_ = false;

    double init_x_ = 0.0;
    double init_y_ = 0.0;
    double temp_target_x_ = 0.0;
    double temp_target_y_ = 0.0;
    double pre_min_distance_ = 0.0;
    double diff_min_dist_ = 0.0;
};

} // namespace pure_pursuit_planner
