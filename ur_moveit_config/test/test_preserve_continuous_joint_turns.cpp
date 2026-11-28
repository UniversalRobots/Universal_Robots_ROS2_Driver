// Copyright 2026, Universal Robots ROS 2 Driver contributors
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
//    * Neither the name of the copyright holder nor the names of its
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

// cppcheck does not expand the Google Test fixture macros in the standalone hook.
// cppcheck-suppress-file syntaxError

#include <gtest/gtest.h>
#include <srdfdom/model.h>
#include <urdf/model.h>

#include <limits>
#include <numbers>

#include "../src/preserve_continuous_joint_turns.hpp"

namespace ur_moveit_config
{
namespace
{
constexpr double RAD = std::numbers::pi / 180.0;

class PreserveContinuousJointTurnsTest : public testing::Test
{
protected:
  void SetUp() override
  {
    auto urdf_model = std::make_shared<urdf::Model>();
    ASSERT_TRUE(urdf_model->initString(R"(
      <robot name="fixture">
        <link name="base"/><link name="arm"/><link name="tool"/>
        <joint name="elbow" type="revolute">
          <parent link="base"/><child link="arm"/>
          <limit effort="10" velocity="1" lower="-1" upper="1"/>
        </joint>
        <joint name="wrist" type="continuous">
          <parent link="arm"/><child link="tool"/>
          <limit effort="10" velocity="1"/>
        </joint>
      </robot>)"));
    auto srdf_model = std::make_shared<srdf::Model>();
    ASSERT_TRUE(srdf_model->initString(*urdf_model, R"(
      <robot name="fixture">
        <group name="arm"><chain base_link="base" tip_link="tool"/></group>
      </robot>)"));
    model_ = std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
  }

  moveit::core::RobotState makeState(double wrist, double elbow = 0.0) const
  {
    moveit::core::RobotState state(model_);
    state.setToDefaultValues();
    state.setVariablePosition("elbow", elbow);
    state.setVariablePosition("wrist", wrist * RAD);
    state.setVariableVelocity("wrist", 0.25);
    state.setVariableAcceleration("wrist", -0.1);
    return state;
  }

  robot_trajectory::RobotTrajectory makePath(double first, double last) const
  {
    robot_trajectory::RobotTrajectory result(model_, "arm");
    result.addSuffixWayPoint(makeState(first), 0);
    result.addSuffixWayPoint(makeState(last), 2);
    return result;
  }

  moveit::core::RobotModelPtr model_;
};

TEST_F(PreserveContinuousJointTurnsTest, PreservesPositiveAndNegativeTurns)
{
  for (const double direction : { 1.0, -1.0 }) {
    auto path = makePath(-179.0 * direction, -183.0 * direction);
    preserveContinuousJointTurns(makeState(181.0 * direction), path);
    EXPECT_NEAR(path.getFirstWayPoint().getVariablePosition("wrist"), 181.0 * direction * RAD, 1e-12);
    EXPECT_NEAR(path.getLastWayPoint().getVariablePosition("wrist"), 177.0 * direction * RAD, 1e-12);
    EXPECT_DOUBLE_EQ(path.getLastWayPoint().getVariablePosition("elbow"), 0.0);
    EXPECT_DOUBLE_EQ(path.getLastWayPoint().getVariableVelocity("wrist"), 0.25);
    EXPECT_DOUBLE_EQ(path.getLastWayPoint().getVariableAcceleration("wrist"), -0.1);
    EXPECT_DOUBLE_EQ(path.getWayPointDurationFromPrevious(1), 2.0);
  }
}

TEST_F(PreserveContinuousJointTurnsTest, PreservesIntentionalFullTurns)
{
  auto path = makePath(-179.0, 181.0);
  preserveContinuousJointTurns(makeState(541.0), path);
  EXPECT_NEAR(path.getFirstWayPoint().getVariablePosition("wrist"), 541.0 * RAD, 1e-12);
  EXPECT_NEAR(path.getLastWayPoint().getVariablePosition("wrist"), 901.0 * RAD, 1e-12);
}

TEST_F(PreserveContinuousJointTurnsTest, DoesNotChangeAlreadyAlignedPath)
{
  auto path = makePath(179.0, 181.0);
  preserveContinuousJointTurns(makeState(179.0), path);
  EXPECT_DOUBLE_EQ(path.getFirstWayPoint().getVariablePosition("wrist"), 179.0 * RAD);
  EXPECT_DOUBLE_EQ(path.getLastWayPoint().getVariablePosition("wrist"), 181.0 * RAD);
}

TEST_F(PreserveContinuousJointTurnsTest, DoesNotChangeDifferentStartPose)
{
  auto path = makePath(-179.0, -183.0);
  preserveContinuousJointTurns(makeState(181.0, 0.1), path);
  preserveContinuousJointTurns(makeState(182.0), path);
  EXPECT_DOUBLE_EQ(path.getFirstWayPoint().getVariablePosition("wrist"), -179.0 * RAD);
  EXPECT_DOUBLE_EQ(path.getLastWayPoint().getVariablePosition("wrist"), -183.0 * RAD);
}

TEST_F(PreserveContinuousJointTurnsTest, LeavesInvalidWaypointUntouched)
{
  auto path = makePath(-179.0, std::numeric_limits<double>::quiet_NaN());
  preserveContinuousJointTurns(makeState(181.0), path);
  EXPECT_DOUBLE_EQ(path.getFirstWayPoint().getVariablePosition("wrist"), -179.0 * RAD);
}

TEST_F(PreserveContinuousJointTurnsTest, AcceptsEmptyPath)
{
  robot_trajectory::RobotTrajectory path(model_, "arm");
  preserveContinuousJointTurns(makeState(181.0), path);
  EXPECT_TRUE(path.empty());
}
}  // namespace
}  // namespace ur_moveit_config
