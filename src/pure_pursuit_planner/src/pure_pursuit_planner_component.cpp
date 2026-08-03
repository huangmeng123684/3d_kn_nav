// Directory: pure_pursuit_planner/src/pure_pursuit_component.cpp
#include "pure_pursuit_planner/pure_pursuit_planner_component.hpp"
#include <cmath>
#include <limits>
#include <iostream>
#include <algorithm>

namespace pure_pursuit_planner {

PurePursuitComponent::PurePursuitComponent(const PurePursuitConfig& config)
: cfg_(config) {}

void PurePursuitComponent::setPath(const std::vector<double>& cx,
                                    const std::vector<double>& cy,
                                    const std::vector<double>& cyaw,
                                    const std::vector<double>& ck) {
    cx_ = cx;
    cy_ = cy;
    cyaw_ = cyaw;
    ck_ = ck;
    //std::cout << "odom_sub_flag: " << odom_sub_flag << std::endl;
    if (!odom_sub_flag){
        odom_sub_flag = true;
        oldNearestPointIndex = -1;
    }
}

void PurePursuitComponent::setPose(const Pose2D& pose, double velocity) {
    current_pose_ = pose;
    current_velocity_ = velocity;
}

std::vector<double> PurePursuitComponent::computeVelocity(
    const std::vector<double>& cx,
    const std::vector<double>& cy,
    const std::vector<double>& cyaw,
    const std::vector<double>& ck,
    const Pose2D& pose, 
    double velocity,
    bool final_approach
) 
    {
    if (cx.empty() || cx.size() != cy.size() || cx.size() != cyaw.size() ||
        cx.size() != ck.size() || !std::isfinite(cfg_.maxCurvature) ||
        cfg_.maxCurvature <= 0.0) {
        return {0.0, 0.0};
    }
    setPath(cx, cy, cyaw, ck);
    setPose(pose, velocity);

    const double path_end_distance = calcDistance(
        current_pose_.x, current_pose_.y, cx_.back(), cy_.back());
    if (final_approach && path_end_distance <= cfg_.final_heading_entry_distance) {
        rotating_to_path_ = false;
        const double goal_yaw_error = normalizeAngle(cyaw_.back() - current_pose_.yaw);
        if (std::abs(goal_yaw_error) <= cfg_.final_heading_command_deadband) {
            return {0.0, 0.0};
        }
        return {0.0, calculateFinalRotationAngularVelocity(goal_yaw_error)};
    }
    if (cfg_.standalone_goal_completion &&
        path_end_distance <= cfg_.goal_threshold) {
        rotating_to_path_ = false;
        return {0.0, 0.0};
    }

    //std::cout << "odom_sub_flag: " << odom_sub_flag << std::endl;
    auto [ind, Lf] = searchTargetIndex();
    //std::cout << "ind: " << ind << std::endl;
    //std::cout << "Lf: " << Lf << std::endl;
    //std::cout << "cx: " << cx_.size() << std::endl;

    targetIndex_ = ind;

    if (targetIndex_ < 0 || targetIndex_ >= static_cast<int>(cx_.size()) || Lf <= 0.0) {
        return {0.0, 0.0};
    }

    double tx = cx_[targetIndex_];
    double ty = cy_[targetIndex_];
    //double target_yaw = cyaw_[targetIndex_];
    double target_curvature = ck_[targetIndex_];
    //std::cout << "target_curvature: " << std::abs(target_curvature) << std::endl;
    double dx = tx - current_pose_.x;
    double dy = ty - current_pose_.y;

    double alpha = normalizeAngle(std::atan2(dy, dx) - current_pose_.yaw);

    if (rotating_to_path_) {
        if (std::abs(alpha) <= cfg_.rotate_to_path_tolerance) {
            rotating_to_path_ = false;
        } else {
            return {0.0, calculateRotationAngularVelocity(alpha)};
        }
    } else if (std::abs(alpha) >= cfg_.rotate_to_path_threshold) {
        rotating_to_path_ = true;
        return {0.0, calculateRotationAngularVelocity(alpha)};
    }

    double curvature = std::max(cfg_.minCurvature, std::min(std::abs(target_curvature), cfg_.maxCurvature));
    //std::cout << "curvature: " << std::abs(curvature) << std::endl;
    curvature = curvature / cfg_.maxCurvature;

    //double v = (cfg_.maxVelocity- cfg_.minVelocity) * pow(sin(acos(std::cbrt(curvature))), 3) + cfg_.minVelocity; //[m/s]
    double v = curvatureToVelocity(curvature);

    v = std::clamp(v, cfg_.minVelocity, cfg_.maxVelocity);
    //std::cout << "v: " << v << std::endl;

    //double w = v * std::sin(alpha) / Lf;
    double w = calculateAngularVelocity(v, alpha, Lf);
    w = std::clamp(w, -cfg_.maxAngularVelocity, cfg_.maxAngularVelocity);

    std::vector<double> cmd_velocity{v, w};


    return cmd_velocity;
}

double PurePursuitComponent::alphaExceptionHandling(double tempAlpha) const {
    // 角度を -π〜π の範囲に正規化
    tempAlpha = std::fmod(tempAlpha + M_PI, 2 * M_PI);
    if (tempAlpha < 0)
        tempAlpha += 2 * M_PI;
    tempAlpha -= M_PI;

    // π ± ε のときだけ補正
    constexpr double eps = 0.1;
    if (std::abs(tempAlpha - M_PI) < eps || std::abs(tempAlpha + M_PI) < eps) {
        tempAlpha += 0.15;
    }

    return tempAlpha;
}


double PurePursuitComponent::calculateAngularVelocity(double v, double alpha, double Lf) const {
    return v * std::sin(alpha) / Lf;
}


double PurePursuitComponent::normalizeAngle(double angle) const {
    return std::atan2(std::sin(angle), std::cos(angle));
}


double PurePursuitComponent::calculateRotationAngularVelocity(double yaw_error) const {
    return std::clamp(
        cfg_.rotate_to_heading_gain * yaw_error,
        -cfg_.maxAngularVelocity,
        cfg_.maxAngularVelocity);
}

double PurePursuitComponent::calculateFinalRotationAngularVelocity(double yaw_error) const {
    double command = calculateRotationAngularVelocity(yaw_error);
    if (std::abs(command) <= 0.0) {
        return command;
    }
    const double minimum = std::min(cfg_.min_final_angular_velocity,
                                    cfg_.maxAngularVelocity);
    if (std::abs(command) < minimum) {
        command = std::copysign(minimum, command);
    }
    return std::clamp(command, -cfg_.maxAngularVelocity, cfg_.maxAngularVelocity);
}


double PurePursuitComponent::curvatureToVelocity(double curvature) const {
    return (cfg_.maxVelocity- cfg_.minVelocity) * pow(sin(acos(std::cbrt(curvature))), 3) + cfg_.minVelocity;
}

std::pair<double, double> PurePursuitComponent::isGoalReached(double v, double w) const {
    return {v, w};
}

double PurePursuitComponent::calcLf(double k, double current_velocity, double Lfc) const {
    return k * current_velocity + Lfc;
}

int PurePursuitComponent::calcFirstNearestPointIndex() const {
    if (cx_.empty() || cx_.size() != cy_.size()) {
        return -1;
    }
    double min_distance = std::numeric_limits<double>::max();
    int min_index = -1;
    for (size_t i = 0; i < cx_.size(); i++) {
        double distance = calcDistance(current_pose_.x, current_pose_.y, cx_[i], cy_[i]);
        if (distance < min_distance) {
            min_distance = distance;
            min_index = i;
        }
    }
    return min_index;
}

int PurePursuitComponent::calcOldNearestPointIndex() const {
    if (cx_.empty() || cx_.size() != cy_.size()) {
        return -1;
    }
    bool count_flag = false;
    int count = 0, min_index = -1;
    double min_distance = std::numeric_limits<double>::max();
    
    std::vector<double> min_distance_list; 
    std::vector<int> min_distance_idx_list; 
    min_distance_list.clear();
    min_distance_idx_list.clear();
    int search_start = std::max(0, oldNearestPointIndex - 20);
    for (int i = search_start; i < static_cast<int>(cx_.size()) - 1; i++) {
        double distanceThisIndex = calcDistance(current_pose_.x, current_pose_.y, cx_[i],     cy_[i]);
        double distanceNextIndex = calcDistance(current_pose_.x, current_pose_.y, cx_[i + 1], cy_[i + 1]);
        if (distanceThisIndex < distanceNextIndex) {
            count_flag = true;
        }
        if (count_flag){
            count ++;
        }
        if (distanceThisIndex < min_distance) {
            min_distance = distanceThisIndex;
            //min_index = i;
            //RCLCPP_INFO(this->get_logger(), "Received path point: (%d)", min_index);
            // 配列に保存
            min_distance_list.push_back(min_distance);
            min_distance_idx_list.push_back(i);
        }
        
    }
    // add find the index of path nearest to the current index from list 'min_distance_idx_lst'
    int tmp_idx_dis;
    int min_idx_distance = std::numeric_limits<int>::max();
    for (size_t j=0; j < min_distance_idx_list.size() ;j++){
        tmp_idx_dis = min_distance_idx_list[j]-oldNearestPointIndex;
        if ( min_idx_distance > tmp_idx_dis){
            if (tmp_idx_dis < 100 ){
                min_index = min_distance_idx_list[j];
                //RCLCPP_INFO(this->get_logger(), "CCCC path point: (%ld)", min_distance_idx_list.size());
    
            }
        }
    }
    //RCLCPP_INFO(this->get_logger(), "index of path point: (%ld)", min_distance_idx_list.size());
    return min_index;
}

std::pair<int, double> PurePursuitComponent::searchTargetIndex() {
    if (odom_sub_flag && !cx_.empty() && cx_.size() == cy_.size()){
        double Lf = calcLf(cfg_.k, current_velocity_, cfg_.Lfc);
        if (!std::isfinite(Lf) || Lf <= 0.0) {
            return {-1, 0.0};
        }
        //RCLCPP_INFO(this->get_logger(), "Lf: %lf", Lf);
        if (oldNearestPointIndex == -1) {
            oldNearestPointIndex = calcFirstNearestPointIndex();
        } else {
            oldNearestPointIndex = calcOldNearestPointIndex();
        }

        if (oldNearestPointIndex < 0 ||
            oldNearestPointIndex >= static_cast<int>(cx_.size())) {
            oldNearestPointIndex = calcFirstNearestPointIndex();
        }
        if (oldNearestPointIndex < 0 ||
            oldNearestPointIndex >= static_cast<int>(cx_.size())) {
            return {-1, Lf};
        }

        int ind = oldNearestPointIndex;

        while (Lf > calcDistance(current_pose_.x, current_pose_.y, cx_[ind], cy_[ind])) {
            if (ind + 1 >= static_cast<int>(cx_.size())) {
                break;
            }
            ind++;
        }
        //std::cout << "ind, Lf: " << ind << ", " << Lf << std::endl;
        return { ind, Lf };
    }else{
        std::cout << "[WARN] searchTargetIndex() called before path was set." << std::endl;
        return {-1, 0.0};
    }
}

double PurePursuitComponent::calcDistance(double x1, double y1, double x2, double y2) const {
    return std::hypot(x2 - x1, y2 - y1);
}

} // namespace pure_pursuit_planner
