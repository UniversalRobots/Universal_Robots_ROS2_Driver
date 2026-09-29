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

// Regression tests for the read()/write() failure handling introduced to make hardware faults
// trigger a lifecycle error transition. These exercise URPositionHardwareInterface directly through a
// white-box subclass (no real robot/ursim connection), following the pattern used by
// test_robot_state_helper.cpp.

// cppcheck-suppress-file syntaxError

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <limits>

#include "ur_robot_driver/hardware_interface.hpp"

namespace ur_robot_driver
{

// Thin wrapper exposing the protected state needed to drive read()/write() in isolation, and
// overriding the driver-call seams so tests never need a real urcl::UrDriver instance.
class URPositionHardwareInterfaceTestWrapper : public URPositionHardwareInterface
{
public:
  URPositionHardwareInterfaceTestWrapper()
  {
    // Mirror the relevant parts of on_init()/initAsyncIO() so every NaN-guarded field starts in its
    // "no new command" state instead of whatever garbage an unconstructed member holds.
    non_blocking_read_ = false;
    non_blocking_read_timeout_ = rclcpp::Duration(0, 0);
    time_since_successful_read_ = rclcpp::Duration(0, 0);
    rtde_comm_has_been_started_ = true;  // skip the ur_driver_->startRTDECommunication() call
    packet_read_ = false;
    initialized_ = false;
    stop_requested_ = false;
    robot_program_running_ = false;
    runtime_state_ = static_cast<uint32_t>(urcl::rtde_interface::RUNTIME_STATE::STOPPED);

    position_controller_running_ = false;
    velocity_controller_running_ = false;
    torque_controller_running_ = false;
    freedrive_mode_controller_running_ = false;
    freedrive_activated_ = false;
    passthrough_trajectory_controller_running_ = false;
    motion_primitives_forward_controller_running_ = false;
    twist_controller_running_ = false;
    tool_contact_controller_running_ = false;
    tool_contact_set_state_ = 0.0;

    force_mode_task_frame_.fill(NO_NEW_CMD_);
    force_mode_selection_vector_.fill(NO_NEW_CMD_);
    force_mode_wrench_.fill(NO_NEW_CMD_);
    force_mode_limits_.fill(NO_NEW_CMD_);
    force_mode_type_ = NO_NEW_CMD_;
    force_mode_disable_cmd_ = NO_NEW_CMD_;
    force_mode_damping_ = NO_NEW_CMD_;
    force_mode_gain_scaling_ = NO_NEW_CMD_;

    // Non-null but never dereferenced: only used for `ur_driver_ != nullptr` guard checks, since the
    // overridden seams below never touch the real driver.
    ur_driver_ = std::shared_ptr<urcl::UrDriver>(reinterpret_cast<urcl::UrDriver*>(this), [](urcl::UrDriver*) {});

    get_data_package = [this]() { return get_data_package_result_; };
  }

  void setNonBlockingRead(bool val)
  {
    non_blocking_read_ = val;
  }
  void setNonBlockingReadTimeout(rclcpp::Duration timeout)
  {
    non_blocking_read_timeout_ = timeout;
  }
  void setTimeSinceSuccessfulRead(rclcpp::Duration elapsed)
  {
    time_since_successful_read_ = elapsed;
  }
  void setGetDataPackageResult(bool val)
  {
    get_data_package_result_ = val;
  }
  void setRtdeCommHasBeenStarted(bool val)
  {
    rtde_comm_has_been_started_ = val;
  }
  void setInitialized(bool val)
  {
    initialized_ = val;
  }
  bool isInitialized() const
  {
    return initialized_.load();
  }
  void callResetActivationState()
  {
    resetActivationState();
    // resetActivationState() also flips this to false so on_configure() re-triggers the real RTDE
    // startup; re-arm it here since ur_driver_ is a fake pointer in this test.
    rtde_comm_has_been_started_ = true;
  }
  void setNonBlockingReadTimeoutParameter(const std::string& value)
  {
    info_.hardware_parameters = {
      { "robot_ip", "127.0.0.1" },
      { "script_filename", "unused" },
      { "output_recipe_filename", "unused" },
      { "input_recipe_filename", "unused" },
      { "headless_mode", "false" },
      { "reverse_port", "50001" },
      { "script_sender_port", "50002" },
      { "use_currents_as_efforts", "true" },
      { "reverse_ip", "127.0.0.1" },
      { "trajectory_port", "50003" },
      { "script_command_port", "50004" },
      { "non_blocking_read", "true" },
      { "non_blocking_read_timeout", value },
    };
  }

  void setRuntimeStatePlaying()
  {
    runtime_state_ = static_cast<uint32_t>(urcl::rtde_interface::RUNTIME_STATE::PLAYING);
  }
  void setRobotProgramRunning(bool val)
  {
    robot_program_running_ = val;
  }
  void setPositionControllerRunning(bool val)
  {
    position_controller_running_ = val;
  }
  void setToolContactControllerRunning(bool val, double set_state)
  {
    tool_contact_controller_running_ = val;
    tool_contact_set_state_ = set_state;
  }
  void setPassthroughTrajectoryControllerRunning(bool val, double transfer_state, double abort)
  {
    passthrough_trajectory_controller_running_ = val;
    passthrough_trajectory_transfer_state_ = transfer_state;
    passthrough_trajectory_abort_ = abort;
    info_.rw_rate = 500;
  }
  void setMotionPrimitivesControllerRunning(bool val)
  {
    motion_primitives_forward_controller_running_ = val;
  }
  void setMoprimMotionType(MoprimMotionHelperType motion_type)
  {
    hw_moprim_commands_.fill(NO_NEW_CMD_);
    hw_moprim_commands_[0] = static_cast<double>(motion_type);
  }
  void setMoprimMotionType(int8_t motion_type)
  {
    hw_moprim_commands_.fill(NO_NEW_CMD_);
    hw_moprim_commands_[0] = static_cast<double>(motion_type);
  }
  void fillMoprimCommandQueue()
  {
    std::array<double, 25> command{};
    while (moprim_cmd_queue_.push(command)) {
    }
  }
  bool readyForNewMoprim() const
  {
    return ready_for_new_moprim_;
  }
  size_t queuedMoprimCommands() const
  {
    return moprim_cmd_queue_.size();
  }
  void setMoprimSequenceInProgress()
  {
    build_moprim_sequence_ = true;
    moprim_sequence_.push_back(nullptr);
    current_moprim_execution_status_ = MoprimExecutionState::EXECUTING;
  }
  bool moprimStateIsReset() const
  {
    return !build_moprim_sequence_ && moprim_sequence_.empty() &&
           current_moprim_execution_status_ == MoprimExecutionState::IDLE && !ready_for_new_moprim_ &&
           std::isnan(hw_moprim_commands_[0]) &&
           hw_moprim_states_[0] == static_cast<double>(MoprimExecutionState::IDLE) && hw_moprim_states_[1] == 0.0;
  }
  void setKeepaliveResult(bool val)
  {
    keepalive_result_ = val;
  }

  void setWriteJointCommandResult(bool val)
  {
    write_joint_command_result_ = val;
  }
  void setStartToolContactResult(bool val)
  {
    start_tool_contact_result_ = val;
  }
  void setTrajectoryControlResult(bool val)
  {
    trajectory_control_result_ = val;
  }
  int writeJointCommandCallCount() const
  {
    return write_joint_command_calls_;
  }
  int startToolContactCallCount() const
  {
    return start_tool_contact_calls_;
  }
  int trajectoryControlCallCount() const
  {
    return trajectory_control_calls_;
  }
  int keepaliveCallCount() const
  {
    return keepalive_calls_;
  }

protected:
  bool writeJointCommandToDriver(const urcl::vector6d_t& /*values*/, urcl::comm::ControlMode /*control_mode*/,
                                 const urcl::RobotReceiveTimeout& /*timeout*/) override
  {
    ++write_joint_command_calls_;
    return write_joint_command_result_;
  }
  bool writeTrajectoryControlMessageToDriver(urcl::control::TrajectoryControlMessage /*trajectory_action*/,
                                             int /*point_number*/,
                                             const urcl::RobotReceiveTimeout& /*timeout*/) override
  {
    ++trajectory_control_calls_;
    return trajectory_control_result_;
  }
  bool writeKeepaliveToDriver() override
  {
    ++keepalive_calls_;
    return keepalive_result_;
  }
  bool startToolContactOnDriver() override
  {
    ++start_tool_contact_calls_;
    return start_tool_contact_result_;
  }
  bool endToolContactOnDriver() override
  {
    return true;
  }

private:
  bool get_data_package_result_ = false;
  bool write_joint_command_result_ = true;
  bool start_tool_contact_result_ = true;
  bool trajectory_control_result_ = true;
  bool keepalive_result_ = true;
  int write_joint_command_calls_ = 0;
  int start_tool_contact_calls_ = 0;
  int trajectory_control_calls_ = 0;
  int keepalive_calls_ = 0;
};

namespace
{
using hardware_interface::return_type;

TEST(HardwareInterfaceReadFaults, BlockingReadFailureReturnsError)
{
  URPositionHardwareInterfaceTestWrapper hw;
  hw.setNonBlockingRead(false);
  hw.setGetDataPackageResult(false);

  EXPECT_EQ(hw.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)), return_type::ERROR);
}

TEST(HardwareInterfaceReadFaults, ReadMissDoesNotMarkHardwareInitialized)
{
  URPositionHardwareInterfaceTestWrapper hw;
  hw.setNonBlockingRead(true);
  hw.setNonBlockingReadTimeout(rclcpp::Duration::from_seconds(0.04));
  hw.setGetDataPackageResult(false);

  EXPECT_FALSE(hw.isInitialized());
  EXPECT_EQ(hw.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)), return_type::OK);
  EXPECT_FALSE(hw.isInitialized());
}

TEST(HardwareInterfaceReadFaults, ActivationResetClearsInitializedState)
{
  URPositionHardwareInterfaceTestWrapper hw;
  hw.setInitialized(true);

  hw.callResetActivationState();

  EXPECT_FALSE(hw.isInitialized());
}

TEST(HardwareInterfaceReadFaults, NonBlockingMissesStayOkUnderTimeout)
{
  URPositionHardwareInterfaceTestWrapper hw;
  hw.setNonBlockingRead(true);
  hw.setNonBlockingReadTimeout(rclcpp::Duration::from_seconds(0.04));
  hw.setGetDataPackageResult(false);

  const auto period = rclcpp::Duration::from_seconds(0.01);
  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(hw.read(rclcpp::Time(0), period), return_type::OK);
  }
}

TEST(HardwareInterfaceReadFaults, NonBlockingMissesExceedingTimeoutReturnError)
{
  URPositionHardwareInterfaceTestWrapper hw;
  hw.setNonBlockingRead(true);
  hw.setNonBlockingReadTimeout(rclcpp::Duration::from_seconds(0.04));
  hw.setGetDataPackageResult(false);

  const auto period = rclcpp::Duration::from_seconds(0.01);
  // 0.01, 0.02, 0.03, 0.04 -> still within/at the timeout.
  for (int i = 0; i < 4; ++i) {
    EXPECT_EQ(hw.read(rclcpp::Time(0), period), return_type::OK);
  }
  // 0.05 -> exceeds the timeout.
  EXPECT_EQ(hw.read(rclcpp::Time(0), period), return_type::ERROR);
}

TEST(HardwareInterfaceReadFaults, TimeoutStateResetAllowsRecoveryAfterReconfigure)
{
  URPositionHardwareInterfaceTestWrapper hw;
  hw.setNonBlockingRead(true);
  hw.setNonBlockingReadTimeout(rclcpp::Duration::from_seconds(0.04));
  hw.setGetDataPackageResult(false);

  hw.setTimeSinceSuccessfulRead(rclcpp::Duration::from_seconds(0.05));
  EXPECT_EQ(hw.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)), return_type::ERROR);

  hw.callResetActivationState();

  EXPECT_EQ(hw.read(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)), return_type::OK);
}

class HardwareInterfaceTimeoutParameterTest : public ::testing::TestWithParam<std::string>
{
};

TEST_P(HardwareInterfaceTimeoutParameterTest, InvalidValueReturnsLifecycleError)
{
  URPositionHardwareInterfaceTestWrapper hw;
  hw.setNonBlockingReadTimeoutParameter(GetParam());

  EXPECT_EQ(hw.on_configure(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::ERROR);
}

INSTANTIATE_TEST_SUITE_P(InvalidValues, HardwareInterfaceTimeoutParameterTest,
                         ::testing::Values("", "not-a-number", "0.04s", "nan", "1e999", "1e10"));

TEST(HardwareInterfaceWriteFaults, JointCommandFailureReturnsError)
{
  URPositionHardwareInterfaceTestWrapper hw;
  hw.setRuntimeStatePlaying();
  hw.setRobotProgramRunning(true);
  hw.setPositionControllerRunning(true);
  hw.setWriteJointCommandResult(false);
  hw.setToolContactControllerRunning(true, /*set_state=*/2.0);

  EXPECT_EQ(hw.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)), return_type::ERROR);
  EXPECT_EQ(hw.writeJointCommandCallCount(), 1);
  EXPECT_EQ(hw.startToolContactCallCount(), 0);
}

TEST(HardwareInterfaceWriteFaults, PassthroughNoopFailureSkipsCancel)
{
  URPositionHardwareInterfaceTestWrapper hw;
  hw.setRuntimeStatePlaying();
  hw.setRobotProgramRunning(true);
  hw.setPassthroughTrajectoryControllerRunning(true, /*transfer_state=*/1.0, /*abort=*/1.0);
  hw.setTrajectoryControlResult(false);

  EXPECT_EQ(hw.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)), return_type::ERROR);
  EXPECT_EQ(hw.trajectoryControlCallCount(), 1);
}

TEST(HardwareInterfaceWriteFaults, MoprimCancelFailureSkipsKeepalive)
{
  URPositionHardwareInterfaceTestWrapper hw;
  hw.setRuntimeStatePlaying();
  hw.setRobotProgramRunning(true);
  hw.setMotionPrimitivesControllerRunning(true);
  hw.setMoprimMotionType(MoprimMotionHelperType::STOP_MOTION);
  hw.setTrajectoryControlResult(false);

  EXPECT_EQ(hw.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)), return_type::ERROR);
  EXPECT_EQ(hw.trajectoryControlCallCount(), 1);
  EXPECT_EQ(hw.keepaliveCallCount(), 0);
}

TEST(HardwareInterfaceWriteFaults, FullMoprimQueueReturnsError)
{
  URPositionHardwareInterfaceTestWrapper hw;
  hw.setRuntimeStatePlaying();
  hw.setRobotProgramRunning(true);
  hw.setMotionPrimitivesControllerRunning(true);
  hw.setMoprimMotionType(MoprimMotionType::LINEAR_JOINT);
  hw.fillMoprimCommandQueue();

  EXPECT_EQ(hw.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)), return_type::ERROR);
  EXPECT_FALSE(hw.readyForNewMoprim());
}

TEST(HardwareInterfaceWriteFaults, MoprimQueueAndSequenceResetAfterWriteFault)
{
  URPositionHardwareInterfaceTestWrapper hw;
  hw.setRuntimeStatePlaying();
  hw.setRobotProgramRunning(true);
  hw.setMotionPrimitivesControllerRunning(true);
  hw.setMoprimMotionType(MoprimMotionType::LINEAR_JOINT);
  hw.setKeepaliveResult(false);

  EXPECT_EQ(hw.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)), return_type::ERROR);
  ASSERT_EQ(hw.queuedMoprimCommands(), 1u);
  hw.setMoprimSequenceInProgress();

  hw.callResetActivationState();

  EXPECT_EQ(hw.queuedMoprimCommands(), 0u);
  EXPECT_TRUE(hw.moprimStateIsReset());
}

TEST(HardwareInterfaceWriteFaults, ToolContactHelperFailureReturnsError)
{
  URPositionHardwareInterfaceTestWrapper hw;
  hw.setRuntimeStatePlaying();
  hw.setRobotProgramRunning(true);
  hw.setPositionControllerRunning(true);
  hw.setWriteJointCommandResult(true);
  hw.setToolContactControllerRunning(true, /*set_state=*/2.0);
  hw.setStartToolContactResult(false);

  EXPECT_EQ(hw.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)), return_type::ERROR);
  EXPECT_EQ(hw.startToolContactCallCount(), 1);
}

}  // namespace
}  // namespace ur_robot_driver
