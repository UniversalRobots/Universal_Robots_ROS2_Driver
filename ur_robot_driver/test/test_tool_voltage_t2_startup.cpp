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

#include <vector>

#include "ur_robot_driver/hardware_interface.hpp"

namespace
{
class ToolVoltageT2StartupHw : public ur_robot_driver::URPositionHardwareInterface
{
public:
  std::vector<int> applied_voltages;

  bool sendToolVoltageT2(const int voltage) override
  {
    applied_voltages.push_back(voltage);
    return true;
  }

  // Program has already stopped, and that stop was consumed, so a later command cannot rely on the
  // stop count to clear the startup-sent flag.
  void stoppedAfterConsumedDisconnect()
  {
    robot_program_stop_count_.store(1);
    startup_tool_voltage_t2_stop_count_seen_ = 1;
    robot_program_running_.store(false);
    tool_t2_supported_ = true;
    startup_tool_voltage_t2_ = 24;
    startup_tool_voltage_t2_sent_ = false;
    startup_tool_voltage_t2_error_reported_ = false;
    tool_voltage_t2_cmd_ = NO_NEW_CMD_;
  }

  void request(const double voltage)
  {
    tool_voltage_t2_cmd_ = voltage;
  }

  void setProgramRunning(const bool running)
  {
    robot_program_running_.store(running);
  }

  // A stop and restart that both happen between two async checks.
  void stopAndRestart()
  {
    handleRobotProgramState(false);
    handleRobotProgramState(true);
  }

  bool startupSent() const
  {
    return startup_tool_voltage_t2_sent_;
  }

  void handle()
  {
    handleConnectorT2Voltage();
  }
};
}  // namespace

TEST(ToolVoltageT2Startup, RequestBeforeProgramStartDoesNotSkipConfiguredVoltage)
{
  ToolVoltageT2StartupHw hw;
  hw.stoppedAfterConsumedDisconnect();
  hw.request(0.0);

  hw.handle();

  EXPECT_FALSE(hw.startupSent());
  ASSERT_EQ(hw.applied_voltages.size(), 1u);
  EXPECT_EQ(hw.applied_voltages[0], 0);

  hw.setProgramRunning(true);
  hw.handle();

  EXPECT_TRUE(hw.startupSent());
  ASSERT_EQ(hw.applied_voltages.size(), 2u);
  EXPECT_EQ(hw.applied_voltages[1], 24);
}

TEST(ToolVoltageT2Startup, RequestWhileProgramIsRunningOverridesStartupVoltage)
{
  ToolVoltageT2StartupHw hw;
  hw.stoppedAfterConsumedDisconnect();
  hw.setProgramRunning(true);
  hw.request(48.0);

  hw.handle();

  EXPECT_TRUE(hw.startupSent());
  ASSERT_EQ(hw.applied_voltages.size(), 1u);
  EXPECT_EQ(hw.applied_voltages[0], 48);

  hw.handle();

  EXPECT_EQ(hw.applied_voltages.size(), 1u);
}

TEST(ToolVoltageT2Startup, StopAndRestartBetweenChecksReappliesConfiguredVoltage)
{
  ToolVoltageT2StartupHw hw;
  hw.stoppedAfterConsumedDisconnect();
  hw.setProgramRunning(true);

  hw.handle();

  ASSERT_EQ(hw.applied_voltages.size(), 1u);
  EXPECT_EQ(hw.applied_voltages[0], 24);

  hw.stopAndRestart();
  hw.handle();

  ASSERT_EQ(hw.applied_voltages.size(), 2u);
  EXPECT_EQ(hw.applied_voltages[1], 24);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
