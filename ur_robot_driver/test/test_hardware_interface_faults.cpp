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
    // Mirror the relevant parts of on_init()/resetAsyncIO() so every NaN-guarded field starts in its
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
    force_mode_async_success_ = NO_NEW_CMD_;
    tool_contact_state_ = 0.0;

    installFakeDriver();

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
  void setValidDataPackage(const urcl::vector6d_t& joint_positions)
  {
    const std::vector<std::string> recipe = { "actual_q",
                                              "actual_qd",
                                              "actual_current_as_torque",
                                              "target_speed_fraction",
                                              "speed_scaling",
                                              "runtime_state",
                                              "actual_TCP_force",
                                              "actual_TCP_pose",
                                              "target_TCP_pose",
                                              "standard_analog_input0",
                                              "standard_analog_input1",
                                              "standard_analog_output0",
                                              "standard_analog_output1",
                                              "tool_mode",
                                              "tool_analog_input0",
                                              "tool_analog_input1",
                                              "tool_output_voltage",
                                              "tool_output_current",
                                              "tool_temperature",
                                              "robot_mode",
                                              "safety_mode",
                                              "robot_status_bits",
                                              "safety_status_bits",
                                              "actual_digital_input_bits",
                                              "actual_digital_output_bits",
                                              "analog_io_types",
                                              "tool_analog_input_types",
                                              "tcp_offset",
                                              "payload",
                                              "payload_cog",
                                              "payload_inertia" };
    data_package_buffer_ = std::make_unique<urcl::rtde_interface::DataPackage>(recipe);
    ASSERT_TRUE(data_package_buffer_->setData("actual_q", joint_positions));
    ASSERT_TRUE(data_package_buffer_->setData("runtime_state",
                                              static_cast<uint32_t>(urcl::rtde_interface::RUNTIME_STATE::STOPPED)));
  }
  bool positionCommandsMatch(const urcl::vector6d_t& joint_positions) const
  {
    return urcl_position_commands_ == joint_positions && urcl_position_commands_old_ == joint_positions;
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
    resetHardwareInterfaceState();
    // resetHardwareInterfaceState() also flips this to false so on_configure() re-triggers the real RTDE
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
  void setLifecycleRecoveryParameters()
  {
    setNonBlockingReadTimeoutParameter("0.04");
    info_.hardware_parameters["non_blocking_read"] = "false";
  }
  bool hasDriver() const
  {
    return ur_driver_ != nullptr;
  }
  bool readTimeoutAgeIsZero() const
  {
    return time_since_successful_read_.nanoseconds() == 0;
  }
  int configureResourcesCallCount() const
  {
    return configure_resources_calls_;
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
  void setAllControllerModesRunning()
  {
    position_controller_running_ = true;
    velocity_controller_running_ = true;
    torque_controller_running_ = true;
    force_mode_controller_running_ = true;
    freedrive_mode_controller_running_ = true;
    passthrough_trajectory_controller_running_ = true;
    tool_contact_controller_running_ = true;
    twist_controller_running_ = true;
    motion_primitives_forward_controller_running_ = true;
    urcl_twist_commands_ = { { 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 } };
  }
  bool controllerModesStopped() const
  {
    return !position_controller_running_ && !velocity_controller_running_ && !torque_controller_running_ &&
           !force_mode_controller_running_ && !freedrive_mode_controller_running_ &&
           !passthrough_trajectory_controller_running_ && !tool_contact_controller_running_ &&
           !twist_controller_running_ && !motion_primitives_forward_controller_running_;
  }
  bool twistCommandIsZero() const
  {
    return urcl_twist_commands_ == urcl::vector6d_t{ { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
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
  void setPendingForceModeCommand()
  {
    force_mode_task_frame_.fill(0.0);
    force_mode_selection_vector_.fill(1.0);
    force_mode_wrench_.fill(0.0);
    force_mode_limits_.fill(0.1);
    force_mode_type_ = 2.0;
    force_mode_damping_ = 0.8;
    force_mode_gain_scaling_ = 0.5;
  }
  void setForceModeAsyncSuccess(double val)
  {
    force_mode_async_success_ = val;
  }
  void setForceModeDisableCmd(double val)
  {
    force_mode_disable_cmd_ = val;
  }
  double forceModeAsyncSuccess() const
  {
    return force_mode_async_success_;
  }
  bool forceModeCommandCleared() const
  {
    return std::isnan(force_mode_task_frame_[0]) && std::isnan(force_mode_type_) && std::isnan(force_mode_disable_cmd_);
  }
  double toolContactState() const
  {
    return tool_contact_state_;
  }
  void setPendingPassthroughTransfer()
  {
    passthrough_trajectory_transfer_state_ = 2.0;
    passthrough_trajectory_size_ = 3.0;
    passthrough_trajectory_time_from_start_ = 0.5;
    passthrough_last_point_time_ = 0.5;
    passthrough_point_index_received_ = 2;
    passthrough_point_index_sent_ = 1;
    passthrough_trajectory_started_ = true;
    trajectory_joint_positions_.resize(3);
    trajectory_times_.resize(3);
  }
  bool deferredCommandsCleared() const
  {
    const bool force_mode_cleared =
        std::isnan(force_mode_task_frame_[0]) && std::isnan(force_mode_selection_vector_[0]) &&
        std::isnan(force_mode_wrench_[0]) && std::isnan(force_mode_limits_[0]) && std::isnan(force_mode_type_) &&
        std::isnan(force_mode_damping_) && std::isnan(force_mode_gain_scaling_) && std::isnan(force_mode_disable_cmd_);
    const bool passthrough_cleared = passthrough_trajectory_transfer_state_ == 0.0 &&
                                     passthrough_trajectory_abort_ == 0.0 && passthrough_trajectory_size_ == 0.0 &&
                                     passthrough_trajectory_time_from_start_ == 0.0 &&
                                     trajectory_joint_positions_.empty() && trajectory_times_.empty() &&
                                     passthrough_last_point_time_ == 0.0 && passthrough_point_index_received_ == 0 &&
                                     passthrough_point_index_sent_ == 0 && !passthrough_trajectory_started_;
    const bool misc_cleared = std::isnan(target_speed_fraction_cmd_) && std::isnan(resend_robot_program_cmd_) &&
                              std::isnan(zero_ftsensor_cmd_) && std::isnan(hand_back_control_cmd_) &&
                              std::isnan(freedrive_mode_enable_) && std::isnan(freedrive_mode_abort_) &&
                              !freedrive_activated_ && std::isnan(payload_mass_) && std::isnan(gravity_vector_[0]) &&
                              std::isnan(tool_voltage_cmd_) && std::isnan(standard_dig_out_bits_cmd_[0]);
    return force_mode_cleared && passthrough_cleared && misc_cleared;
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
  void setEndToolContactResult(bool val)
  {
    end_tool_contact_result_ = val;
  }
  void setEndForceModeResult(bool val)
  {
    end_force_mode_result_ = val;
  }
  void setStopForceModeRequested()
  {
    stop_modes_ = { { STOP_FORCE_MODE } };
    start_modes_.resize(1);
    force_mode_controller_running_ = true;
  }
  void setStopToolContactRequested()
  {
    stop_modes_ = { { STOP_TOOL_CONTACT } };
    start_modes_.resize(1);
    tool_contact_controller_running_ = true;
  }
  bool forceModeControllerRunning() const
  {
    return force_mode_controller_running_;
  }
  bool positionControllerRunning() const
  {
    return position_controller_running_;
  }
  bool toolContactControllerRunning() const
  {
    return tool_contact_controller_running_;
  }
  void setStopForceModeAndToolContactRequested()
  {
    stop_modes_ = { { STOP_FORCE_MODE, STOP_TOOL_CONTACT } };
    start_modes_.resize(1);
    force_mode_controller_running_ = true;
    tool_contact_controller_running_ = true;
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
  int endToolContactCallCount() const
  {
    return end_tool_contact_calls_;
  }
  int endForceModeCallCount() const
  {
    return end_force_mode_calls_;
  }

protected:
  void transformForceTorque() override
  {
  }

  hardware_interface::CallbackReturn configureHardwareResources() override
  {
    ++configure_resources_calls_;
    non_blocking_read_ = false;
    rtde_comm_has_been_started_ = true;
    installFakeDriver();
    get_data_package = [this]() { return get_data_package_result_; };
    return hardware_interface::CallbackReturn::SUCCESS;
  }

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
  bool endForceModeOnDriver() override
  {
    ++end_force_mode_calls_;
    return end_force_mode_result_;
  }
  bool startToolContactOnDriver() override
  {
    ++start_tool_contact_calls_;
    return start_tool_contact_result_;
  }
  bool endToolContactOnDriver() override
  {
    ++end_tool_contact_calls_;
    return end_tool_contact_result_;
  }

private:
  void installFakeDriver()
  {
    // Non-null but never dereferenced: only used for `ur_driver_ != nullptr` guard checks.
    ur_driver_ = std::shared_ptr<urcl::UrDriver>(reinterpret_cast<urcl::UrDriver*>(this), [](urcl::UrDriver*) {});
  }

  bool get_data_package_result_ = false;
  bool write_joint_command_result_ = true;
  bool start_tool_contact_result_ = true;
  bool end_tool_contact_result_ = true;
  bool end_force_mode_result_ = true;
  bool trajectory_control_result_ = true;
  bool keepalive_result_ = true;
  int write_joint_command_calls_ = 0;
  int start_tool_contact_calls_ = 0;
  int trajectory_control_calls_ = 0;
  int keepalive_calls_ = 0;
  int end_tool_contact_calls_ = 0;
  int end_force_mode_calls_ = 0;
  int configure_resources_calls_ = 0;
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

TEST(HardwareInterfaceReadFaults, ActivationResetClearsControllerModesAndTwistCommand)
{
  URPositionHardwareInterfaceTestWrapper hw;
  hw.setAllControllerModesRunning();

  hw.callResetActivationState();

  EXPECT_TRUE(hw.controllerModesStopped());
  EXPECT_TRUE(hw.twistCommandIsZero());
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

TEST(HardwareInterfaceReadFaults, SuccessfulReadAfterReconfigureInitializesAndRestartsTimeout)
{
  URPositionHardwareInterfaceTestWrapper hw;
  hw.setInitialized(true);
  hw.callResetActivationState();
  hw.setNonBlockingRead(true);
  hw.setNonBlockingReadTimeout(rclcpp::Duration::from_seconds(0.04));
  hw.setGetDataPackageResult(false);

  const auto period = rclcpp::Duration::from_seconds(0.01);
  for (int read_index = 0; read_index < 3; ++read_index) {
    ASSERT_EQ(hw.read(rclcpp::Time(0), period), return_type::OK);
  }
  EXPECT_FALSE(hw.isInitialized());
  EXPECT_FALSE(hw.readTimeoutAgeIsZero());

  const urcl::vector6d_t joint_positions = { { 0.1, 0.2, 0.3, 0.4, 0.5, 0.6 } };
  hw.setValidDataPackage(joint_positions);
  hw.setGetDataPackageResult(true);

  ASSERT_EQ(hw.read(rclcpp::Time(0), period), return_type::OK);
  EXPECT_TRUE(hw.isInitialized());
  EXPECT_TRUE(hw.positionCommandsMatch(joint_positions));
  EXPECT_TRUE(hw.readTimeoutAgeIsZero());

  hw.setGetDataPackageResult(false);
  for (int read_index = 0; read_index < 4; ++read_index) {
    EXPECT_EQ(hw.read(rclcpp::Time(0), period), return_type::OK);
  }
  EXPECT_TRUE(hw.isInitialized());
  EXPECT_EQ(hw.read(rclcpp::Time(0), period), return_type::ERROR);
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
  // TOOL_CONTACT_FAILURE_BEGIN
  EXPECT_EQ(hw.toolContactState(), 4.0);
}

TEST(HardwareInterfaceWriteFaults, WriteFaultFailsPendingToolContactEnd)
{
  URPositionHardwareInterfaceTestWrapper hw;
  hw.setRuntimeStatePlaying();
  hw.setRobotProgramRunning(true);
  hw.setPositionControllerRunning(true);
  hw.setWriteJointCommandResult(false);
  hw.setToolContactControllerRunning(true, /*set_state=*/5.0);

  EXPECT_EQ(hw.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)), return_type::ERROR);
  EXPECT_EQ(hw.endToolContactCallCount(), 0);
  // TOOL_CONTACT_FAILURE_END
  EXPECT_EQ(hw.toolContactState(), 7.0);
}

TEST(HardwareInterfaceWriteFaults, WriteFaultFailsPendingForceModeStart)
{
  URPositionHardwareInterfaceTestWrapper hw;
  hw.setRuntimeStatePlaying();
  hw.setRobotProgramRunning(true);
  hw.setPositionControllerRunning(true);
  hw.setPendingForceModeCommand();
  hw.setForceModeAsyncSuccess(2.0);
  hw.setWriteJointCommandResult(false);

  EXPECT_EQ(hw.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)), return_type::ERROR);
  EXPECT_EQ(hw.forceModeAsyncSuccess(), 0.0);
  EXPECT_TRUE(hw.forceModeCommandCleared());
}

TEST(HardwareInterfaceWriteFaults, WriteFaultFailsPendingForceModeStop)
{
  URPositionHardwareInterfaceTestWrapper hw;
  hw.setRuntimeStatePlaying();
  hw.setRobotProgramRunning(true);
  hw.setPositionControllerRunning(true);
  hw.setForceModeDisableCmd(1.0);
  hw.setForceModeAsyncSuccess(2.0);
  hw.setWriteJointCommandResult(false);

  EXPECT_EQ(hw.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)), return_type::ERROR);
  EXPECT_EQ(hw.endForceModeCallCount(), 0);
  EXPECT_EQ(hw.forceModeAsyncSuccess(), 0.0);
  EXPECT_TRUE(hw.forceModeCommandCleared());
}

TEST(HardwareInterfaceLifecycleRecovery, WriteFaultRecoversAfterErrorAndReconfigure)
{
  URPositionHardwareInterfaceTestWrapper hw;
  hw.setLifecycleRecoveryParameters();
  hw.setRuntimeStatePlaying();
  hw.setRobotProgramRunning(true);
  hw.setAllControllerModesRunning();
  hw.setInitialized(true);
  hw.setTimeSinceSuccessfulRead(rclcpp::Duration::from_seconds(0.05));
  hw.setPendingForceModeCommand();
  hw.setPendingPassthroughTransfer();
  hw.setWriteJointCommandResult(false);

  EXPECT_EQ(hw.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)), return_type::ERROR);
  EXPECT_EQ(hw.on_error(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_FALSE(hw.hasDriver());

  EXPECT_EQ(hw.on_configure(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(hw.configureResourcesCallCount(), 1);
  EXPECT_TRUE(hw.hasDriver());
  EXPECT_FALSE(hw.isInitialized());
  EXPECT_TRUE(hw.controllerModesStopped());
  EXPECT_TRUE(hw.deferredCommandsCleared());
  EXPECT_TRUE(hw.readTimeoutAgeIsZero());

  EXPECT_EQ(hw.on_activate(rclcpp_lifecycle::State()), hardware_interface::CallbackReturn::SUCCESS);
  hw.setRuntimeStatePlaying();
  hw.setRobotProgramRunning(true);
  hw.setPositionControllerRunning(true);
  hw.setWriteJointCommandResult(true);

  EXPECT_EQ(hw.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)), return_type::OK);
  EXPECT_EQ(hw.writeJointCommandCallCount(), 2);
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

TEST(HardwareInterfaceWriteFaults, DeferredCommandsClearedAfterWriteFault)
{
  URPositionHardwareInterfaceTestWrapper hw;
  hw.setRuntimeStatePlaying();
  hw.setRobotProgramRunning(true);
  hw.setPositionControllerRunning(true);
  hw.setPendingForceModeCommand();
  hw.setPendingPassthroughTransfer();
  hw.setWriteJointCommandResult(false);

  ASSERT_EQ(hw.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)), return_type::ERROR);

  hw.callResetActivationState();
  EXPECT_TRUE(hw.deferredCommandsCleared());

  // The stale force mode request must not be executed by the first write after the reconfigure;
  // start_force_mode() would dereference the fake driver pointer if it were still pending.
  hw.setWriteJointCommandResult(true);
  EXPECT_EQ(hw.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)), return_type::OK);
}

TEST(HardwareInterfaceModeSwitchFaults, ForceModeStopFailureReturnsError)
{
  URPositionHardwareInterfaceTestWrapper hw;
  hw.setStopForceModeRequested();
  hw.setEndForceModeResult(false);

  EXPECT_EQ(hw.perform_command_mode_switch({}, {}), return_type::ERROR);
  EXPECT_EQ(hw.endForceModeCallCount(), 1);
  EXPECT_TRUE(hw.forceModeControllerRunning());
}

TEST(HardwareInterfaceModeSwitchFaults, ToolContactStopFailureReturnsError)
{
  URPositionHardwareInterfaceTestWrapper hw;
  hw.setStopToolContactRequested();
  hw.setEndToolContactResult(false);

  EXPECT_EQ(hw.perform_command_mode_switch({}, {}), return_type::ERROR);
  EXPECT_EQ(hw.endToolContactCallCount(), 1);
  EXPECT_TRUE(hw.toolContactControllerRunning());
}

TEST(HardwareInterfaceModeSwitchFaults, ForceModeStopFailureLeavesOtherControllersUnchanged)
{
  URPositionHardwareInterfaceTestWrapper hw;
  hw.setStopForceModeAndToolContactRequested();
  hw.setPositionControllerRunning(true);
  hw.setRuntimeStatePlaying();
  hw.setRobotProgramRunning(true);
  hw.setEndForceModeResult(false);

  EXPECT_EQ(hw.perform_command_mode_switch({}, {}), return_type::ERROR);
  EXPECT_TRUE(hw.forceModeControllerRunning());
  EXPECT_TRUE(hw.toolContactControllerRunning());
  EXPECT_TRUE(hw.positionControllerRunning());

  EXPECT_EQ(hw.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)), return_type::OK);
  EXPECT_EQ(hw.writeJointCommandCallCount(), 1);
}

TEST(HardwareInterfaceModeSwitchFaults, ToolContactStopFailureLeavesOtherControllersUnchanged)
{
  URPositionHardwareInterfaceTestWrapper hw;
  hw.setStopForceModeAndToolContactRequested();
  hw.setPositionControllerRunning(true);
  hw.setRuntimeStatePlaying();
  hw.setRobotProgramRunning(true);
  hw.setEndToolContactResult(false);

  EXPECT_EQ(hw.perform_command_mode_switch({}, {}), return_type::ERROR);
  EXPECT_TRUE(hw.forceModeControllerRunning());
  EXPECT_TRUE(hw.toolContactControllerRunning());
  EXPECT_TRUE(hw.positionControllerRunning());

  EXPECT_EQ(hw.write(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.01)), return_type::OK);
  EXPECT_EQ(hw.writeJointCommandCallCount(), 1);
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
