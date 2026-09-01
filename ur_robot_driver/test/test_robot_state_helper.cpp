// Copyright 2026 Universal Robots A/S
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

// Tests for the concurrency and lifetime bugs reported in
// https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/issues/1943
//
// These tests exercise only the goal-callback decision logic through the
// protected default constructor. rclcpp::init() is required because
// RobotStateHelper now inherits from rclcpp::Node.

// cppcheck-suppress-file syntaxError

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/types.hpp"
#include "ur_client_library/ur/datatypes.h"
#include "ur_dashboard_msgs/action/set_mode.hpp"
#include "ur_robot_driver/robot_state_helper.hpp"

namespace ur_robot_driver
{

// Thin wrapper that uses the protected default constructor and exposes the
// internal state and goal callback needed by the tests.
class RobotStateHelperTestWrapper : public RobotStateHelper
{
public:
  RobotStateHelperTestWrapper() : RobotStateHelper()
  {
  }

  void setRobotMode(urcl::RobotMode mode)
  {
    robot_mode_ = mode;
  }
  void setSafetyMode(urcl::SafetyMode mode)
  {
    safety_mode_ = mode;
  }
  void setInAction(bool val)
  {
    in_action_ = val;
  }
  bool isInAction() const
  {
    return in_action_;
  }

  rclcpp_action::GoalResponse callGoalCallback(const rclcpp_action::GoalUUID& uuid,
                                               std::shared_ptr<const ur_dashboard_msgs::action::SetMode::Goal> goal)
  {
    return setModeGoalCallback(uuid, goal);
  }
};

// ---------------------------------------------------------------------------
// Helper factories
// ---------------------------------------------------------------------------

static rclcpp_action::GoalUUID makeUuid()
{
  rclcpp_action::GoalUUID uuid{};
  uuid.fill(0);
  return uuid;
}

static std::shared_ptr<const ur_dashboard_msgs::action::SetMode::Goal> makeGoal(int mode = 7 /* RUNNING */)
{
  auto g = std::make_shared<ur_dashboard_msgs::action::SetMode::Goal>();
  g->target_robot_mode = mode;
  return g;
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

// Regression for issue #1943: the goal callback must reject a new goal while
// a SetMode action is already in progress.  Before the fix, in_action_ was
// never checked and a second goal was silently accepted, leading to shared-
// state corruption.
TEST(RobotStateHelperGoalCallback, RejectsGoalWhenInAction)
{
  RobotStateHelperTestWrapper helper;
  helper.setRobotMode(urcl::RobotMode::RUNNING);
  helper.setSafetyMode(urcl::SafetyMode::NORMAL);
  helper.setInAction(true);

  const auto response = helper.callGoalCallback(makeUuid(), makeGoal());

  // cppcheck-suppress knownConditionTrueFalse
  const rclcpp_action::GoalResponse expected = rclcpp_action::GoalResponse::REJECT;
  EXPECT_EQ(response, expected);
}

// Baseline: a goal must be accepted when the robot mode and safety mode are
// known and no action is currently running.
TEST(RobotStateHelperGoalCallback, AcceptsGoalWhenModesKnownAndIdle)
{
  RobotStateHelperTestWrapper helper;
  helper.setRobotMode(urcl::RobotMode::POWER_OFF);
  helper.setSafetyMode(urcl::SafetyMode::NORMAL);
  helper.setInAction(false);

  const auto response = helper.callGoalCallback(makeUuid(), makeGoal());

  const rclcpp_action::GoalResponse expected = rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  EXPECT_EQ(response, expected);
}

// Regression: the goal callback must reject when robot mode is still UNKNOWN
// (robot not yet connected), regardless of in_action_.
TEST(RobotStateHelperGoalCallback, RejectsGoalWhenRobotModeUnknown)
{
  RobotStateHelperTestWrapper helper;
  // robot_mode_ stays UNKNOWN
  helper.setSafetyMode(urcl::SafetyMode::NORMAL);
  helper.setInAction(false);

  const auto response = helper.callGoalCallback(makeUuid(), makeGoal());

  const rclcpp_action::GoalResponse expected = rclcpp_action::GoalResponse::REJECT;
  EXPECT_EQ(response, expected);
}

// Regression: the goal callback must reject when safety mode is still
// UNDEFINED, regardless of in_action_.
TEST(RobotStateHelperGoalCallback, RejectsGoalWhenSafetyModeUndefined)
{
  RobotStateHelperTestWrapper helper;
  helper.setRobotMode(urcl::RobotMode::RUNNING);
  // safety_mode_ stays UNDEFINED_SAFETY_MODE
  helper.setInAction(false);

  const auto response = helper.callGoalCallback(makeUuid(), makeGoal());

  const rclcpp_action::GoalResponse expected = rclcpp_action::GoalResponse::REJECT;
  EXPECT_EQ(response, expected);
}

// Verify that in_action_ is initially false so a fresh helper is immediately
// ready to accept a goal.
TEST(RobotStateHelperGoalCallback, InActionFalseOnConstruction)
{
  RobotStateHelperTestWrapper helper;
  EXPECT_FALSE(helper.isInAction());
}

}  // namespace ur_robot_driver

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleMock(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
