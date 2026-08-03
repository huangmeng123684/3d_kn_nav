#include <gtest/gtest.h>

#include <chrono>
#include <limits>
#include <string>
#include <vector>

#include "pure_pursuit_planner/go2_safety_controller.hpp"

namespace pure_pursuit_planner
{
namespace
{

using namespace std::chrono_literals;

class FakeSportClient : public SportCommandInterface
{
public:
  int move(float vx, float vy, float vyaw) override
  {
    moves.push_back({vx, vy, vyaw});
    if (fail_nonzero_move && (vx != 0.0F || vy != 0.0F || vyaw != 0.0F)) {
      return move_result;
    }
    return 0;
  }

  int stopMove() override
  {
    ++stop_count;
    return stop_result;
  }

  std::vector<Go2VelocityCommand> moves;
  int stop_count{0};
  int move_result{3104};
  int stop_result{0};
  bool fail_nonzero_move{false};
};

Go2SafetyController::TimePoint at(std::chrono::milliseconds offset)
{
  return Go2SafetyController::TimePoint{} + offset;
}

void provideHeartbeats(Go2SafetyController & controller, Go2SafetyController::TimePoint now)
{
  controller.updateOdometryHeartbeat(now);
  controller.updateSportStateHeartbeat(now);
}

TEST(Go2SafetyController, StartsDisabledAndRequiresFreshHeartbeats)
{
  FakeSportClient client;
  Go2SafetyController controller(client, Go2SafetyConfig{});
  std::string reason;

  EXPECT_FALSE(controller.armed());
  EXPECT_FALSE(controller.enable(at(0ms), reason));
  EXPECT_NE(reason.find("odometry"), std::string::npos);

  controller.updateOdometryHeartbeat(at(0ms));
  EXPECT_FALSE(controller.enable(at(0ms), reason));
  EXPECT_NE(reason.find("sport state"), std::string::npos);

  controller.updateSportStateHeartbeat(at(0ms));
  EXPECT_TRUE(controller.enable(at(0ms), reason));
  EXPECT_TRUE(controller.armed());
  EXPECT_TRUE(controller.waitingForCommand());
  EXPECT_EQ(client.stop_count, 1);
  ASSERT_EQ(client.moves.size(), 1U);
  EXPECT_DOUBLE_EQ(client.moves.back().vx, 0.0);
}

TEST(Go2SafetyController, IgnoresPreArmCommandAndRequiresNewCommand)
{
  FakeSportClient client;
  Go2SafetyController controller(client, Go2SafetyConfig{});
  std::string reason;

  EXPECT_FALSE(controller.acceptCommand({0.2, 0.0, 0.0}, at(0ms), reason));
  provideHeartbeats(controller, at(10ms));
  ASSERT_TRUE(controller.enable(at(10ms), reason));
  controller.tick(at(100ms));

  EXPECT_TRUE(controller.armed());
  EXPECT_TRUE(controller.waitingForCommand());
  EXPECT_EQ(client.moves.size(), 1U);
}

TEST(Go2SafetyController, ClampsAndSlewLimitsCommands)
{
  FakeSportClient client;
  Go2SafetyController controller(client, Go2SafetyConfig{});
  std::string reason;
  provideHeartbeats(controller, at(0ms));
  ASSERT_TRUE(controller.enable(at(0ms), reason));

  ASSERT_TRUE(controller.acceptCommand({1.0, 1.0, -2.0}, at(10ms), reason));
  controller.tick(at(100ms));

  ASSERT_TRUE(controller.armed());
  ASSERT_EQ(client.moves.size(), 2U);
  EXPECT_NEAR(client.moves.back().vx, 0.025, 1e-6);
  EXPECT_DOUBLE_EQ(client.moves.back().vy, 0.0);
  EXPECT_NEAR(client.moves.back().vyaw, -0.05, 1e-6);

  controller.updateOdometryHeartbeat(at(150ms));
  controller.updateSportStateHeartbeat(at(150ms));
  ASSERT_TRUE(controller.acceptCommand({1.0, 1.0, -2.0}, at(150ms), reason));
  controller.tick(at(200ms));
  EXPECT_NEAR(client.moves.back().vx, 0.05, 1e-6);
  EXPECT_NEAR(client.moves.back().vyaw, -0.1, 1e-6);
}

TEST(Go2SafetyController, ZeroCommandStopsImmediatelyButStaysArmed)
{
  FakeSportClient client;
  Go2SafetyController controller(client, Go2SafetyConfig{});
  std::string reason;
  provideHeartbeats(controller, at(0ms));
  ASSERT_TRUE(controller.enable(at(0ms), reason));
  ASSERT_TRUE(controller.acceptCommand({0.2, 0.0, 0.2}, at(10ms), reason));
  controller.tick(at(100ms));

  ASSERT_TRUE(controller.acceptCommand({}, at(110ms), reason));
  EXPECT_TRUE(controller.armed());
  EXPECT_EQ(client.stop_count, 2);
  EXPECT_DOUBLE_EQ(controller.lastOutput().vx, 0.0);
}

TEST(Go2SafetyController, InvalidCommandStopsAndLatchesDisabled)
{
  FakeSportClient client;
  Go2SafetyController controller(client, Go2SafetyConfig{});
  std::string reason;
  provideHeartbeats(controller, at(0ms));
  ASSERT_TRUE(controller.enable(at(0ms), reason));

  EXPECT_FALSE(
    controller.acceptCommand(
      {std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0}, at(10ms), reason));
  EXPECT_FALSE(controller.armed());
  EXPECT_NE(controller.lastFault().find("non-finite"), std::string::npos);

  provideHeartbeats(controller, at(20ms));
  controller.tick(at(20ms));
  EXPECT_FALSE(controller.armed());
}

TEST(Go2SafetyController, CommandTimeoutStopsAndRequiresReenable)
{
  FakeSportClient client;
  Go2SafetyController controller(client, Go2SafetyConfig{});
  std::string reason;
  provideHeartbeats(controller, at(0ms));
  ASSERT_TRUE(controller.enable(at(0ms), reason));
  ASSERT_TRUE(controller.acceptCommand({0.2, 0.0, 0.0}, at(10ms), reason));
  provideHeartbeats(controller, at(300ms));

  controller.tick(at(311ms));
  EXPECT_FALSE(controller.armed());
  EXPECT_NE(controller.lastFault().find("cmd_vel"), std::string::npos);

  provideHeartbeats(controller, at(320ms));
  ASSERT_FALSE(controller.acceptCommand({0.2, 0.0, 0.0}, at(320ms), reason));
  EXPECT_FALSE(controller.armed());
}

TEST(Go2SafetyController, HeartbeatTimeoutsStopAndDisarm)
{
  FakeSportClient client;
  Go2SafetyConfig config;
  config.odometry_timeout = 300ms;
  config.sport_state_timeout = 500ms;
  Go2SafetyController controller(client, config);
  std::string reason;
  provideHeartbeats(controller, at(0ms));
  ASSERT_TRUE(controller.enable(at(0ms), reason));
  ASSERT_TRUE(controller.acceptCommand({0.1, 0.0, 0.0}, at(10ms), reason));
  controller.updateSportStateHeartbeat(at(301ms));

  controller.tick(at(301ms));
  EXPECT_FALSE(controller.armed());
  EXPECT_NE(controller.lastFault().find("odometry"), std::string::npos);

  provideHeartbeats(controller, at(400ms));
  ASSERT_TRUE(controller.enable(at(400ms), reason));
  ASSERT_TRUE(controller.acceptCommand({0.1, 0.0, 0.0}, at(410ms), reason));
  controller.updateOdometryHeartbeat(at(901ms));
  controller.tick(at(901ms));
  EXPECT_FALSE(controller.armed());
  EXPECT_NE(controller.lastFault().find("sport state"), std::string::npos);
}

TEST(Go2SafetyController, SdkMoveFailureStopsAndDisarms)
{
  FakeSportClient client;
  Go2SafetyController controller(client, Go2SafetyConfig{});
  std::string reason;
  provideHeartbeats(controller, at(0ms));
  ASSERT_TRUE(controller.enable(at(0ms), reason));
  ASSERT_TRUE(controller.acceptCommand({0.2, 0.0, 0.0}, at(10ms), reason));
  client.fail_nonzero_move = true;

  controller.tick(at(100ms));
  EXPECT_FALSE(controller.armed());
  EXPECT_NE(controller.lastFault().find("3104"), std::string::npos);
  EXPECT_GE(client.stop_count, 2);
}

TEST(Go2SafetyController, ExplicitDisableAndShutdownAlwaysStop)
{
  FakeSportClient client;
  Go2SafetyController controller(client, Go2SafetyConfig{});
  std::string reason;
  provideHeartbeats(controller, at(0ms));
  ASSERT_TRUE(controller.enable(at(0ms), reason));

  controller.disable("operator disabled bridge");
  EXPECT_FALSE(controller.armed());
  EXPECT_EQ(client.stop_count, 2);

  controller.shutdown();
  EXPECT_FALSE(controller.armed());
  EXPECT_EQ(client.stop_count, 3);
  controller.shutdown();
  EXPECT_EQ(client.stop_count, 3);
}

}  // namespace
}  // namespace pure_pursuit_planner
