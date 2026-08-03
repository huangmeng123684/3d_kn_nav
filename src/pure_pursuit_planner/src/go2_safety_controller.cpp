#include "pure_pursuit_planner/go2_safety_controller.hpp"

#include <algorithm>
#include <cmath>

namespace pure_pursuit_planner
{

namespace
{

double approach(double current, double target, double maximum_delta)
{
  return current + std::clamp(target - current, -maximum_delta, maximum_delta);
}

}  // namespace

Go2SafetyController::Go2SafetyController(
  SportCommandInterface & sport_client, const Go2SafetyConfig & config)
: sport_client_(sport_client), config_(config)
{
}

void Go2SafetyController::updateOdometryHeartbeat(TimePoint now)
{
  odometry_received_ = true;
  last_odometry_time_ = now;
}

void Go2SafetyController::updateSportStateHeartbeat(TimePoint now)
{
  sport_state_received_ = true;
  last_sport_state_time_ = now;
}

bool Go2SafetyController::enable(TimePoint now, std::string & reason)
{
  if (armed_) {
    reason = "bridge is already enabled";
    return true;
  }
  if (!heartbeatFresh(
      odometry_received_, last_odometry_time_, now, config_.odometry_timeout))
  {
    reason = "cannot enable: odometry heartbeat is missing or stale";
    return false;
  }
  if (!heartbeatFresh(
      sport_state_received_, last_sport_state_time_, now, config_.sport_state_timeout))
  {
    reason = "cannot enable: sport state heartbeat is missing or stale";
    return false;
  }

  if (!sendStop()) {
    last_fault_ = "cannot enable: StopMove failed";
    reason = last_fault_;
    return false;
  }

  target_command_ = {};
  last_output_ = {};
  command_received_ = false;
  waiting_for_command_ = true;
  last_tick_time_ = now;
  last_fault_.clear();
  armed_ = true;
  reason = "bridge enabled; waiting for a new cmd_vel";
  return true;
}

void Go2SafetyController::disable(const std::string & reason)
{
  sendStop();
  armed_ = false;
  waiting_for_command_ = true;
  command_received_ = false;
  target_command_ = {};
  last_output_ = {};
  last_fault_ = reason;
}

bool Go2SafetyController::acceptCommand(
  const Go2VelocityCommand & command, TimePoint now, std::string & reason)
{
  if (!armed_) {
    reason = "ignored cmd_vel while bridge is disabled";
    return false;
  }
  if (!commandIsFinite(command)) {
    reason = "cmd_vel contains a non-finite value";
    faultAndDisarm(reason);
    return false;
  }

  target_command_ = clampCommand(command);
  last_command_time_ = now;
  command_received_ = true;
  waiting_for_command_ = false;

  if (commandIsZero(target_command_)) {
    if (!sendStop()) {
      reason = "failed to stop for zero cmd_vel";
      faultAndDisarm(reason);
      return false;
    }
    last_tick_time_ = now;
  }

  reason.clear();
  return true;
}

void Go2SafetyController::tick(TimePoint now)
{
  if (!armed_) {
    return;
  }
  if (!heartbeatFresh(
      odometry_received_, last_odometry_time_, now, config_.odometry_timeout))
  {
    faultAndDisarm("odometry heartbeat timed out");
    return;
  }
  if (!heartbeatFresh(
      sport_state_received_, last_sport_state_time_, now, config_.sport_state_timeout))
  {
    faultAndDisarm("sport state heartbeat timed out");
    return;
  }
  if (waiting_for_command_) {
    last_tick_time_ = now;
    return;
  }
  if (!heartbeatFresh(true, last_command_time_, now, config_.command_timeout)) {
    faultAndDisarm("cmd_vel timed out");
    return;
  }
  if (commandIsZero(target_command_)) {
    last_tick_time_ = now;
    return;
  }

  const double elapsed = std::max(
    0.0, std::chrono::duration<double>(now - last_tick_time_).count());
  last_tick_time_ = now;
  const double linear_delta = config_.max_linear_acceleration * elapsed;
  const double yaw_delta = config_.max_yaw_acceleration * elapsed;

  Go2VelocityCommand next;
  next.vx = approach(last_output_.vx, target_command_.vx, linear_delta);
  next.vy = approach(last_output_.vy, target_command_.vy, linear_delta);
  next.vyaw = approach(last_output_.vyaw, target_command_.vyaw, yaw_delta);

  const int result = sport_client_.move(
    static_cast<float>(next.vx), static_cast<float>(next.vy),
    static_cast<float>(next.vyaw));
  if (result != 0) {
    faultAndDisarm("SportClient::Move failed with code " + std::to_string(result));
    return;
  }
  last_output_ = next;
}

void Go2SafetyController::shutdown()
{
  if (shutdown_) {
    return;
  }
  shutdown_ = true;
  disable("bridge shutdown");
}

bool Go2SafetyController::armed() const
{
  return armed_;
}

bool Go2SafetyController::waitingForCommand() const
{
  return waiting_for_command_;
}

const Go2VelocityCommand & Go2SafetyController::lastOutput() const
{
  return last_output_;
}

const std::string & Go2SafetyController::lastFault() const
{
  return last_fault_;
}

bool Go2SafetyController::heartbeatFresh(
  bool received, TimePoint stamp, TimePoint now,
  std::chrono::duration<double> timeout) const
{
  return received && now >= stamp && now - stamp <= timeout;
}

bool Go2SafetyController::commandIsFinite(const Go2VelocityCommand & command) const
{
  return std::isfinite(command.vx) && std::isfinite(command.vy) &&
         std::isfinite(command.vyaw);
}

bool Go2SafetyController::commandIsZero(const Go2VelocityCommand & command) const
{
  constexpr double epsilon = 1e-6;
  return std::abs(command.vx) <= epsilon && std::abs(command.vy) <= epsilon &&
         std::abs(command.vyaw) <= epsilon;
}

Go2VelocityCommand Go2SafetyController::clampCommand(
  const Go2VelocityCommand & command) const
{
  Go2VelocityCommand result;
  result.vx = std::clamp(command.vx, config_.min_vx, config_.max_vx);
  result.vy = std::clamp(command.vy, -config_.max_abs_vy, config_.max_abs_vy);
  result.vyaw = std::clamp(
    command.vyaw, -config_.max_abs_vyaw, config_.max_abs_vyaw);
  return result;
}

void Go2SafetyController::faultAndDisarm(const std::string & reason)
{
  sendStop();
  armed_ = false;
  waiting_for_command_ = true;
  command_received_ = false;
  target_command_ = {};
  last_output_ = {};
  last_fault_ = reason;
}

bool Go2SafetyController::sendStop()
{
  const int move_result = sport_client_.move(0.0F, 0.0F, 0.0F);
  const int stop_result = sport_client_.stopMove();
  last_output_ = {};
  return move_result == 0 && stop_result == 0;
}

}  // namespace pure_pursuit_planner
