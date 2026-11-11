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

#include <gtest/gtest.h>

#include "ur_robot_driver/tool_contact_request.hpp"

using ur_robot_driver::ToolContactRequest;
using ur_robot_driver::toolContactRequest;

namespace
{
// The robot side of URPositionHardwareInterface::check_tool_contact_controller(): counts what is sent
// and tracks the state interface the way the hardware interface does.
struct FakeHardwareInterface
{
  double state = 0.0;
  int starts = 0;
  int ends = 0;
  bool start_succeeds = true;
  bool end_succeeds = true;

  void write(const double commanded_state)
  {
    switch (toolContactRequest(commanded_state, state)) {
      case ToolContactRequest::START:
        ++starts;
        state = start_succeeds ? ur_robot_driver::TOOL_CONTACT_EXECUTING : ur_robot_driver::TOOL_CONTACT_FAILURE_BEGIN;
        break;
      case ToolContactRequest::END:
        ++ends;
        state = end_succeeds ? ur_robot_driver::TOOL_CONTACT_SUCCESS_END : ur_robot_driver::TOOL_CONTACT_FAILURE_END;
        break;
      case ToolContactRequest::NONE:
        if (commanded_state != ur_robot_driver::TOOL_CONTACT_WAITING_BEGIN &&
            commanded_state != ur_robot_driver::TOOL_CONTACT_WAITING_END) {
          state = commanded_state;
        }
        break;
    }
  }

  // The controller is deactivated: tool contact is ended on the robot, write() no longer looks at
  // the tool contact command, and both the state and the leftover command are reset to standby.
  // Returns the command a write() cycle sees until the controller writes one again.
  double stopController()
  {
    state = ur_robot_driver::TOOL_CONTACT_STANDBY;
    return ur_robot_driver::TOOL_CONTACT_STANDBY;
  }

  void writeCycles(const double commanded_state, const int cycles)
  {
    for (int i = 0; i < cycles; ++i) {
      write(commanded_state);
    }
  }
};

constexpr double STANDBY = ur_robot_driver::TOOL_CONTACT_STANDBY;
constexpr int CYCLES = 1000;
}  // namespace

// The controller holds the command at WAITING_BEGIN while tool contact is executing. Starting in
// every cycle would reset the result in every cycle and could overwrite the robot's result.
TEST(ToolContactRequestTest, StartIsSentOnceWhileTheCommandIsHeld)
{
  FakeHardwareInterface hw;
  hw.writeCycles(ur_robot_driver::TOOL_CONTACT_WAITING_BEGIN, CYCLES);
  EXPECT_EQ(hw.starts, 1);
  EXPECT_EQ(hw.ends, 0);
  EXPECT_EQ(hw.state, ur_robot_driver::TOOL_CONTACT_EXECUTING);
}

// After a detected contact the controller holds the command at WAITING_END until the next goal.
TEST(ToolContactRequestTest, EndIsSentOnceWhileTheCommandIsHeld)
{
  FakeHardwareInterface hw;
  hw.writeCycles(ur_robot_driver::TOOL_CONTACT_WAITING_BEGIN, CYCLES);
  hw.writeCycles(ur_robot_driver::TOOL_CONTACT_WAITING_END, CYCLES);
  EXPECT_EQ(hw.starts, 1);
  EXPECT_EQ(hw.ends, 1);
  EXPECT_EQ(hw.state, ur_robot_driver::TOOL_CONTACT_SUCCESS_END);
}

TEST(ToolContactRequestTest, NextGoalAfterADetectedContactStartsAgain)
{
  FakeHardwareInterface hw;
  for (int goal = 1; goal <= 3; ++goal) {
    hw.writeCycles(ur_robot_driver::TOOL_CONTACT_WAITING_BEGIN, CYCLES);
    hw.writeCycles(ur_robot_driver::TOOL_CONTACT_WAITING_END, CYCLES);
    EXPECT_EQ(hw.starts, goal);
    EXPECT_EQ(hw.ends, goal);
  }
}

// A cancelled goal: the controller commands the end, then returns to standby.
TEST(ToolContactRequestTest, NextGoalAfterACancelStartsAgain)
{
  FakeHardwareInterface hw;
  hw.writeCycles(ur_robot_driver::TOOL_CONTACT_WAITING_BEGIN, CYCLES);
  hw.write(ur_robot_driver::TOOL_CONTACT_WAITING_END);
  hw.writeCycles(STANDBY, CYCLES);
  EXPECT_EQ(hw.state, STANDBY);
  hw.writeCycles(ur_robot_driver::TOOL_CONTACT_WAITING_BEGIN, CYCLES);
  EXPECT_EQ(hw.starts, 2);
  EXPECT_EQ(hw.ends, 1);
}

// The controller is deactivated while tool contact is executing. A request that follows the next
// activation right away, before a cycle has seen the controller's standby command, must still start.
TEST(ToolContactRequestTest, FirstRequestAfterAStopWhileExecutingStartsAgain)
{
  FakeHardwareInterface hw;
  hw.writeCycles(ur_robot_driver::TOOL_CONTACT_WAITING_BEGIN, CYCLES);
  hw.stopController();
  hw.writeCycles(ur_robot_driver::TOOL_CONTACT_WAITING_BEGIN, CYCLES);
  EXPECT_EQ(hw.starts, 2);
  EXPECT_EQ(hw.state, ur_robot_driver::TOOL_CONTACT_EXECUTING);
}

// The controller is deactivated after a detected contact, with its command still at WAITING_END. The
// cycles between the hardware interface learning that the controller runs again and the
// controller's first command must not send another end request.
TEST(ToolContactRequestTest, LeftoverCommandAfterAStopSendsNothing)
{
  FakeHardwareInterface hw;
  hw.writeCycles(ur_robot_driver::TOOL_CONTACT_WAITING_BEGIN, CYCLES);
  hw.writeCycles(ur_robot_driver::TOOL_CONTACT_WAITING_END, CYCLES);
  const double leftover_command = hw.stopController();
  hw.writeCycles(leftover_command, CYCLES);
  EXPECT_EQ(hw.starts, 1);
  EXPECT_EQ(hw.ends, 1);
  EXPECT_EQ(hw.state, STANDBY);
}

// A request that could not be sent is tried again for as long as the controller asks for it.
TEST(ToolContactRequestTest, FailedRequestsAreRetried)
{
  FakeHardwareInterface hw;
  hw.start_succeeds = false;
  hw.writeCycles(ur_robot_driver::TOOL_CONTACT_WAITING_BEGIN, 3);
  EXPECT_EQ(hw.starts, 3);
  EXPECT_EQ(hw.state, ur_robot_driver::TOOL_CONTACT_FAILURE_BEGIN);

  hw.start_succeeds = true;
  hw.end_succeeds = false;
  hw.write(ur_robot_driver::TOOL_CONTACT_WAITING_BEGIN);
  hw.writeCycles(ur_robot_driver::TOOL_CONTACT_WAITING_END, 3);
  EXPECT_EQ(hw.ends, 3);
  EXPECT_EQ(hw.state, ur_robot_driver::TOOL_CONTACT_FAILURE_END);
}

TEST(ToolContactRequestTest, OtherCommandsSendNothing)
{
  FakeHardwareInterface hw;
  hw.writeCycles(0.0, CYCLES);
  hw.writeCycles(STANDBY, CYCLES);
  EXPECT_EQ(hw.starts, 0);
  EXPECT_EQ(hw.ends, 0);
  EXPECT_EQ(hw.state, STANDBY);
}

int main(int argc, char* argv[])
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
