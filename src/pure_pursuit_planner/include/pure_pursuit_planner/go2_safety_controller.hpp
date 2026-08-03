#pragma once

#include <chrono>
#include <string>

namespace pure_pursuit_planner
{

struct Go2VelocityCommand
{
  double vx{0.0};
  double vy{0.0};
  double vyaw{0.0};
};

struct Go2SafetyConfig
{
  double min_vx{0.0};
  double max_vx{0.25};
  double max_abs_vy{0.0};
  double max_abs_vyaw{0.5};
  double max_linear_acceleration{0.25};
  double max_yaw_acceleration{0.5};
  std::chrono::duration<double> command_timeout{0.3};
  std::chrono::duration<double> odometry_timeout{0.3};
  std::chrono::duration<double> sport_state_timeout{0.5};
};

class SportCommandInterface
{
public:
  virtual ~SportCommandInterface() = default;
  virtual int move(float vx, float vy, float vyaw) = 0;
  virtual int stopMove() = 0;
};

class Go2SafetyController
{
public:
  using TimePoint = std::chrono::steady_clock::time_point;

  Go2SafetyController(SportCommandInterface & sport_client, const Go2SafetyConfig & config);

  void updateOdometryHeartbeat(TimePoint now);
  void updateSportStateHeartbeat(TimePoint now);

  bool enable(TimePoint now, std::string & reason);
  void disable(const std::string & reason);
  bool acceptCommand(const Go2VelocityCommand & command, TimePoint now, std::string & reason);
  void tick(TimePoint now);
  void shutdown();

  bool armed() const;
  bool waitingForCommand() const;
  const Go2VelocityCommand & lastOutput() const;
  const std::string & lastFault() const;

private:
  bool heartbeatFresh(
    bool received, TimePoint stamp, TimePoint now,
    std::chrono::duration<double> timeout) const;
  bool commandIsFinite(const Go2VelocityCommand & command) const;
  bool commandIsZero(const Go2VelocityCommand & command) const;
  Go2VelocityCommand clampCommand(const Go2VelocityCommand & command) const;
  void faultAndDisarm(const std::string & reason);
  bool sendStop();

  SportCommandInterface & sport_client_;
  Go2SafetyConfig config_;

  bool armed_{false};
  bool waiting_for_command_{true};
  bool odometry_received_{false};
  bool sport_state_received_{false};
  bool command_received_{false};
  bool shutdown_{false};

  TimePoint last_odometry_time_{};
  TimePoint last_sport_state_time_{};
  TimePoint last_command_time_{};
  TimePoint last_tick_time_{};

  Go2VelocityCommand target_command_{};
  Go2VelocityCommand last_output_{};
  std::string last_fault_{};
};

}  // namespace pure_pursuit_planner
