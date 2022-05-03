// Copyright 2019, FZI Forschungszentrum Informatik
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

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Lovro Ivanov lovro.ivanov@gmail.com
 * \author  Andy Zelenak zelenak@picknik.ai
 * \author  Marvin Große Besselmann grosse@fzi.de
 * \date    2020-11-9
 *
 */
//----------------------------------------------------------------------
#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "ur_client_library/exceptions.h"
#include "ur_client_library/ur/tool_communication.h"

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "ur_robot_driver/hardware_interface.hpp"
#include "ur_robot_driver/urcl_log_handler.hpp"

namespace rtde = urcl::rtde_interface;

namespace ur_robot_driver
{
hardware_interface::CallbackReturn
URPositionHardwareInterface::on_init(const hardware_interface::HardwareInfo& system_info)
{
  if (hardware_interface::SystemInterface::on_init(system_info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  info_ = system_info;

  // initialize
  urcl_joint_positions_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  urcl_joint_velocities_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  urcl_joint_efforts_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  urcl_ft_sensor_measurements_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  urcl_tcp_pose_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  urcl_position_commands_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  urcl_position_commands_old_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  urcl_velocity_commands_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  stop_modes_ = { StoppingInterface::NONE, StoppingInterface::NONE, StoppingInterface::NONE,
                  StoppingInterface::NONE, StoppingInterface::NONE, StoppingInterface::NONE };
  start_modes_ = {};
  position_controller_running_ = false;
  velocity_controller_running_ = false;
  runtime_state_ = static_cast<uint32_t>(rtde::RUNTIME_STATE::STOPPED);
  pausing_state_ = PausingState::RUNNING;
  pausing_ramp_up_increment_ = 0.01;
  controllers_initialized_ = false;
  first_pass_ = true;
  initialized_ = false;
  async_thread_shutdown_ = false;
  system_interface_initialized_ = 0.0;

  for (const hardware_interface::ComponentInfo& joint : info_.joints) {
    if (joint.command_interfaces.size() != 2) {
      RCLCPP_FATAL(rclcpp::get_logger("URPositionHardwareInterface"),
                   "Joint '%s' has %zu command interfaces found. 2 expected.", joint.name.c_str(),
                   joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(rclcpp::get_logger("URPositionHardwareInterface"),
                   "Joint '%s' have %s command interfaces found as first command interface. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(rclcpp::get_logger("URPositionHardwareInterface"),
                   "Joint '%s' have %s command interfaces found as second command interface. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 3) {
      RCLCPP_FATAL(rclcpp::get_logger("URPositionHardwareInterface"), "Joint '%s' has %zu state interface. 3 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(rclcpp::get_logger("URPositionHardwareInterface"),
                   "Joint '%s' have %s state interface as first state interface. '%s' expected.", joint.name.c_str(),
                   joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(rclcpp::get_logger("URPositionHardwareInterface"),
                   "Joint '%s' have %s state interface as second state interface. '%s' expected.", joint.name.c_str(),
                   joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_FATAL(rclcpp::get_logger("URPositionHardwareInterface"),
                   "Joint '%s' have %s state interface as third state interface. '%s' expected.", joint.name.c_str(),
                   joint.state_interfaces[2].name.c_str(), hardware_interface::HW_IF_EFFORT);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> URPositionHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &urcl_joint_positions_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &urcl_joint_velocities_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &urcl_joint_efforts_[i]));
  }

  state_interfaces.emplace_back(
      hardware_interface::StateInterface("speed_scaling", "speed_scaling_factor", &speed_scaling_combined_));

  for (auto& sensor : info_.sensors) {
    for (uint j = 0; j < sensor.state_interfaces.size(); ++j) {
      state_interfaces.emplace_back(hardware_interface::StateInterface(sensor.name, sensor.state_interfaces[j].name,
                                                                       &urcl_ft_sensor_measurements_[j]));
    }
  }

  for (size_t i = 0; i < 18; ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface("gpio", "digital_output_" + std::to_string(i),
                                                                     &actual_dig_out_bits_copy_[i]));
    state_interfaces.emplace_back(
        hardware_interface::StateInterface("gpio", "digital_input_" + std::to_string(i), &actual_dig_in_bits_copy_[i]));
  }

  for (size_t i = 0; i < 11; ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface("gpio", "safety_status_bit_" + std::to_string(i),
                                                                     &safety_status_bits_copy_[i]));
  }

  for (size_t i = 0; i < 4; ++i) {
    state_interfaces.emplace_back(
        hardware_interface::StateInterface("gpio", "analog_io_type_" + std::to_string(i), &analog_io_types_copy_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface("gpio", "robot_status_bit_" + std::to_string(i),
                                                                     &robot_status_bits_copy_[i]));
  }

  for (size_t i = 0; i < 2; ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "gpio", "tool_analog_input_type_" + std::to_string(i), &tool_analog_input_types_copy_[i]));

    state_interfaces.emplace_back(
        hardware_interface::StateInterface("gpio", "tool_analog_input_" + std::to_string(i), &tool_analog_input_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "gpio", "standard_analog_input_" + std::to_string(i), &standard_analog_input_[i]));

    state_interfaces.emplace_back(hardware_interface::StateInterface(
        "gpio", "standard_analog_output_" + std::to_string(i), &standard_analog_output_[i]));
  }

  state_interfaces.emplace_back(
      hardware_interface::StateInterface("gpio", "tool_output_voltage", &tool_output_voltage_copy_));

  state_interfaces.emplace_back(hardware_interface::StateInterface("gpio", "robot_mode", &robot_mode_copy_));

  state_interfaces.emplace_back(hardware_interface::StateInterface("gpio", "safety_mode", &safety_mode_copy_));

  state_interfaces.emplace_back(hardware_interface::StateInterface("gpio", "tool_mode", &tool_mode_copy_));

  state_interfaces.emplace_back(
      hardware_interface::StateInterface("gpio", "tool_output_current", &tool_output_current_));

  state_interfaces.emplace_back(hardware_interface::StateInterface("gpio", "tool_temperature", &tool_temperature_));

  state_interfaces.emplace_back(
      hardware_interface::StateInterface("system_interface", "initialized", &system_interface_initialized_));

  state_interfaces.emplace_back(
      hardware_interface::StateInterface("gpio", "program_running", &robot_program_running_copy_));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> URPositionHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &urcl_position_commands_[i]));

    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &urcl_velocity_commands_[i]));
  }

  command_interfaces.emplace_back(hardware_interface::CommandInterface("gpio", "io_async_success", &io_async_success_));

  command_interfaces.emplace_back(
      hardware_interface::CommandInterface("speed_scaling", "target_speed_fraction_cmd", &target_speed_fraction_cmd_));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      "speed_scaling", "target_speed_fraction_async_success", &scaling_async_success_));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      "resend_robot_program", "resend_robot_program_cmd", &resend_robot_program_cmd_));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
      "resend_robot_program", "resend_robot_program_async_success", &resend_robot_program_async_success_));

  command_interfaces.emplace_back(hardware_interface::CommandInterface("payload", "mass", &payload_mass_));
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface("payload", "cog.x", &payload_center_of_gravity_[0]));
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface("payload", "cog.y", &payload_center_of_gravity_[1]));
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface("payload", "cog.z", &payload_center_of_gravity_[2]));
  command_interfaces.emplace_back(
      hardware_interface::CommandInterface("payload", "payload_async_success", &payload_async_success_));

  for (size_t i = 0; i < 18; ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        "gpio", "standard_digital_output_cmd_" + std::to_string(i), &standard_dig_out_bits_cmd_[i]));
  }

  for (size_t i = 0; i < 2; ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
        "gpio", "standard_analog_output_cmd_" + std::to_string(i), &standard_analog_output_cmd_[i]));
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn
URPositionHardwareInterface::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Starting ...please wait...");

  // The robot's IP address.
  std::string robot_ip = info_.hardware_parameters["robot_ip"];
  // Path to the urscript code that will be sent to the robot
  std::string script_filename = info_.hardware_parameters["script_filename"];
  // Path to the file containing the recipe used for requesting RTDE outputs.
  std::string output_recipe_filename = info_.hardware_parameters["output_recipe_filename"];
  // Path to the file containing the recipe used for requesting RTDE inputs.
  std::string input_recipe_filename = info_.hardware_parameters["input_recipe_filename"];
  // Start robot in headless mode. This does not require the 'External Control' URCap to be running
  // on the robot, but this will send the URScript to the robot directly. On e-Series robots this
  // requires the robot to run in 'remote-control' mode.
  bool headless_mode =
      (info_.hardware_parameters["headless_mode"] == "true") || (info_.hardware_parameters["headless_mode"] == "True");
  // Port that will be opened to communicate between the driver and the robot controller.
  int reverse_port = stoi(info_.hardware_parameters["reverse_port"]);
  // The driver will offer an interface to receive the program's URScript on this port.
  int script_sender_port = stoi(info_.hardware_parameters["script_sender_port"]);
  //  std::string tf_prefix = info_.hardware_parameters["tf_prefix"];
  //  std::string tf_prefix;

  // Enables non_blocking_read mode. Should only be used with combined_robot_hw. Disables error generated when read
  // returns without any data, sets the read timeout to zero, and synchronises read/write operations. Enabling this when
  // not used with combined_robot_hw can suppress important errors and affect real-time performance.
  non_blocking_read_ = static_cast<bool>(stoi(info_.hardware_parameters["non_blocking_read"]));

  // Specify gain for servoing to position in joint space.
  // A higher gain can sharpen the trajectory.
  int servoj_gain = stoi(info_.hardware_parameters["servoj_gain"]);
  // Specify lookahead time for servoing to position in joint space.
  // A longer lookahead time can smooth the trajectory.
  double servoj_lookahead_time = stod(info_.hardware_parameters["servoj_lookahead_time"]);

  bool use_tool_communication = (info_.hardware_parameters["use_tool_communication"] == "true") ||
                                (info_.hardware_parameters["use_tool_communication"] == "True");

  // Hash of the calibration reported by the robot. This is used for validating the robot
  // description is using the correct calibration. If the robot's calibration doesn't match this
  // hash, an error will be printed. You can use the robot as usual, however Cartesian poses of the
  // endeffector might be inaccurate. See the "ur_calibration" package on help how to generate your
  // own hash matching your actual robot.
  std::string calibration_checksum = info_.hardware_parameters["kinematics/hash"];

  std::unique_ptr<urcl::ToolCommSetup> tool_comm_setup;
  if (use_tool_communication) {
    tool_comm_setup = std::make_unique<urcl::ToolCommSetup>();

    using ToolVoltageT = std::underlying_type<urcl::ToolVoltage>::type;
    ToolVoltageT tool_voltage;
    // Tool voltage that will be set as soon as the UR-Program on the robot is started. Note: This
    // parameter is only evaluated, when the parameter "use_tool_communication" is set to TRUE.
    // Then, this parameter is required.}
    tool_voltage = std::stoi(info_.hardware_parameters["tool_voltage"]);

    tool_comm_setup->setToolVoltage(static_cast<urcl::ToolVoltage>(tool_voltage));

    using ParityT = std::underlying_type<urcl::Parity>::type;
    ParityT parity;
    // Parity used for tool communication. Will be set as soon as the UR-Program on the robot is
    // started. Can be 0 (None), 1 (odd) and 2 (even).
    //
    // Note: This parameter is only evaluated, when the parameter "use_tool_communication"
    // is set to TRUE.  Then, this parameter is required.
    parity = std::stoi(info_.hardware_parameters["tool_parity"]);
    tool_comm_setup->setParity(static_cast<urcl::Parity>(parity));

    int baud_rate;
    // Baud rate used for tool communication. Will be set as soon as the UR-Program on the robot is
    // started. See UR documentation for valid baud rates.
    //
    // Note: This parameter is only evaluated, when the parameter "use_tool_communication"
    // is set to TRUE.  Then, this parameter is required.
    baud_rate = std::stoi(info_.hardware_parameters["tool_baud_rate"]);
    tool_comm_setup->setBaudRate(static_cast<uint32_t>(baud_rate));

    int stop_bits;
    // Number of stop bits used for tool communication. Will be set as soon as the UR-Program on the robot is
    // started. Can be 1 or 2.
    //
    // Note: This parameter is only evaluated, when the parameter "use_tool_communication"
    // is set to TRUE.  Then, this parameter is required.
    stop_bits = std::stoi(info_.hardware_parameters["tool_stop_bits"]);
    tool_comm_setup->setStopBits(static_cast<uint32_t>(stop_bits));

    int rx_idle_chars;
    // Number of idle chars for the RX unit used for tool communication. Will be set as soon as the UR-Program on the
    // robot is started. Valid values: min=1.0, max=40.0
    //
    // Note: This parameter is only evaluated, when the parameter "use_tool_communication"
    // is set to TRUE.  Then, this parameter is required.
    rx_idle_chars = std::stoi(info_.hardware_parameters["tool_rx_idle_chars"]);
    tool_comm_setup->setRxIdleChars(rx_idle_chars);

    int tx_idle_chars;
    // Number of idle chars for the TX unit used for tool communication. Will be set as soon as the UR-Program on the
    // robot is started. Valid values: min=0.0, max=40.0
    //
    // Note: This parameter is only evaluated, when the parameter "use_tool_communication"
    // is set to TRUE.  Then, this parameter is required.
    tx_idle_chars = std::stoi(info_.hardware_parameters["tool_tx_idle_chars"]);
    tool_comm_setup->setTxIdleChars(tx_idle_chars);
  }

  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Initializing driver...");
  registerUrclLogHandler();
  try {
    ur_driver_ = std::make_unique<urcl::UrDriver>(
        robot_ip, script_filename, output_recipe_filename, input_recipe_filename,
        std::bind(&URPositionHardwareInterface::handleRobotProgramState, this, std::placeholders::_1), headless_mode,
        std::move(tool_comm_setup), (uint32_t)reverse_port, (uint32_t)script_sender_port, servoj_gain,
        servoj_lookahead_time, non_blocking_read_);
  } catch (urcl::ToolCommNotAvailable& e) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("URPositionHardwareInterface"), "See parameter use_tool_communication");

    return hardware_interface::CallbackReturn::ERROR;
  } catch (urcl::UrException& e) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("URPositionHardwareInterface"), e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Calibration checksum: '%s'.",
              calibration_checksum.c_str());
  // check calibration
  // https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/c3378599d5fa73a261328b326392e847f312ab6b/ur_robot_driver/src/hardware_interface.cpp#L296-L309
  if (ur_driver_->checkCalibration(calibration_checksum)) {
    RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Calibration checked successfully.");
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("URPositionHardwareInterface"),

                        "The calibration parameters of the connected robot don't match the ones from the given "
                        "kinematics config file. Please be aware that this can lead to critical inaccuracies of tcp "
                        "positions. Use the ur_calibration tool to extract the correct calibration from the robot and "
                        "pass that into the description. See "
                        "[https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver/blob/main/ur_calibration/"
                        "README.md] for details.");
  }

  ur_driver_->startRTDECommunication();

  async_thread_ = std::make_shared<std::thread>(&URPositionHardwareInterface::asyncThread, this);

  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "System successfully started!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
URPositionHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Stopping ...please wait...");

  async_thread_shutdown_ = true;
  async_thread_->join();
  async_thread_.reset();

  ur_driver_.reset();

  unregisterUrclLogHandler();

  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "System successfully stopped!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

template <typename T>
void URPositionHardwareInterface::readData(const std::unique_ptr<rtde::DataPackage>& data_pkg,
                                           const std::string& var_name, T& data)
{
  if (!data_pkg->getData(var_name, data)) {
    // This throwing should never happen unless misconfigured
    std::string error_msg = "Did not find '" + var_name + "' in data sent from robot. This should not happen!";
    throw std::runtime_error(error_msg);
  }
}

template <typename T, size_t N>
void URPositionHardwareInterface::readBitsetData(const std::unique_ptr<rtde::DataPackage>& data_pkg,
                                                 const std::string& var_name, std::bitset<N>& data)
{
  if (!data_pkg->getData<T, N>(var_name, data)) {
    // This throwing should never happen unless misconfigured
    std::string error_msg = "Did not find '" + var_name + "' in data sent from robot. This should not happen!";
    throw std::runtime_error(error_msg);
  }
}

void URPositionHardwareInterface::asyncThread()
{
  while (!async_thread_shutdown_) {
    if (initialized_) {
      //        RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Initialized in async thread");
      checkAsyncIO();
    }
    std::this_thread::sleep_for(std::chrono::nanoseconds(20000000));
  }
}

hardware_interface::return_type URPositionHardwareInterface::read()
{
  std::unique_ptr<rtde::DataPackage> data_pkg = ur_driver_->getDataPackage();

  if (data_pkg) {
    packet_read_ = true;
    readData(data_pkg, "actual_q", urcl_joint_positions_);
    readData(data_pkg, "actual_qd", urcl_joint_velocities_);
    readData(data_pkg, "actual_current", urcl_joint_efforts_);

    readData(data_pkg, "target_speed_fraction", target_speed_fraction_);
    readData(data_pkg, "speed_scaling", speed_scaling_);
    readData(data_pkg, "runtime_state", runtime_state_);
    readData(data_pkg, "actual_TCP_force", urcl_ft_sensor_measurements_);
    readData(data_pkg, "actual_TCP_pose", urcl_tcp_pose_);
    readData(data_pkg, "standard_analog_input0", standard_analog_input_[0]);
    readData(data_pkg, "standard_analog_input1", standard_analog_input_[1]);
    readData(data_pkg, "standard_analog_output0", standard_analog_output_[0]);
    readData(data_pkg, "standard_analog_output1", standard_analog_output_[1]);
    readData(data_pkg, "tool_mode", tool_mode_);
    readData(data_pkg, "tool_analog_input0", tool_analog_input_[0]);
    readData(data_pkg, "tool_analog_input1", tool_analog_input_[1]);
    readData(data_pkg, "tool_output_voltage", tool_output_voltage_);
    readData(data_pkg, "tool_output_current", tool_output_current_);
    readData(data_pkg, "tool_temperature", tool_temperature_);
    readData(data_pkg, "robot_mode", robot_mode_);
    readData(data_pkg, "safety_mode", safety_mode_);
    readBitsetData<uint32_t>(data_pkg, "robot_status_bits", robot_status_bits_);
    readBitsetData<uint32_t>(data_pkg, "safety_status_bits", safety_status_bits_);
    readBitsetData<uint64_t>(data_pkg, "actual_digital_input_bits", actual_dig_in_bits_);
    readBitsetData<uint64_t>(data_pkg, "actual_digital_output_bits", actual_dig_out_bits_);
    readBitsetData<uint32_t>(data_pkg, "analog_io_types", analog_io_types_);
    readBitsetData<uint32_t>(data_pkg, "tool_analog_input_types", tool_analog_input_types_);

    // required transforms
    extractToolPose();
    transformForceTorque();

    // TODO(anyone): logic for sending other stuff to higher level interface

    // pausing state follows runtime state when pausing
    if (runtime_state_ == static_cast<uint32_t>(rtde::RUNTIME_STATE::PAUSED)) {
      pausing_state_ = PausingState::PAUSED;
    } else if (runtime_state_ == static_cast<uint32_t>(rtde::RUNTIME_STATE::PLAYING) &&
               pausing_state_ == PausingState::PAUSED) {
      // When the robot resumed program execution and pausing state was PAUSED, we enter RAMPUP
      speed_scaling_combined_ = 0.0;
      pausing_state_ = PausingState::RAMPUP;
    }

    if (pausing_state_ == PausingState::RAMPUP) {
      double speed_scaling_ramp = speed_scaling_combined_ + pausing_ramp_up_increment_;
      speed_scaling_combined_ = std::min(speed_scaling_ramp, speed_scaling_ * target_speed_fraction_);

      if (speed_scaling_ramp > speed_scaling_ * target_speed_fraction_) {
        pausing_state_ = PausingState::RUNNING;
      }
    } else if (runtime_state_ == static_cast<uint32_t>(rtde::RUNTIME_STATE::RESUMING)) {
      // We have to keep speed scaling on ROS side at 0 during RESUMING to prevent controllers from
      // continuing to interpolate
      speed_scaling_combined_ = 0.0;
    } else {
      // Normal case
      speed_scaling_combined_ = speed_scaling_ * target_speed_fraction_;
    }

    if (first_pass_ && !initialized_) {
      initAsyncIO();
      // initialize commands
      urcl_position_commands_ = urcl_position_commands_old_ = urcl_joint_positions_;
      urcl_velocity_commands_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
      target_speed_fraction_cmd_ = NO_NEW_CMD_;
      resend_robot_program_cmd_ = NO_NEW_CMD_;
      initialized_ = true;
    }

    updateNonDoubleValues();

    return hardware_interface::return_type::OK;
  }

  RCLCPP_ERROR(rclcpp::get_logger("URPositionHardwareInterface"), "Unable to read from hardware...");
  // TODO(anyone): could not read from the driver --> return ERROR --> on error will be called
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type URPositionHardwareInterface::write()
{
  // If there is no interpreting program running on the robot, we do not want to send anything.
  // TODO(anyone): We would still like to disable the controllers requiring a writable interface. In ROS1
  // this was done externally using the controller_stopper.
  if ((runtime_state_ == static_cast<uint32_t>(rtde::RUNTIME_STATE::PLAYING) ||
       runtime_state_ == static_cast<uint32_t>(rtde::RUNTIME_STATE::PAUSING)) &&
      robot_program_running_ && (!non_blocking_read_ || packet_read_)) {
    if (position_controller_running_) {
      ur_driver_->writeJointCommand(urcl_position_commands_, urcl::comm::ControlMode::MODE_SERVOJ);

    } else if (velocity_controller_running_) {
      ur_driver_->writeJointCommand(urcl_velocity_commands_, urcl::comm::ControlMode::MODE_SPEEDJ);

    } else {
      ur_driver_->writeKeepalive();
    }

    packet_read_ = false;
  }

  return hardware_interface::return_type::OK;
}

void URPositionHardwareInterface::handleRobotProgramState(bool program_running)
{
  robot_program_running_ = program_running;
}

void URPositionHardwareInterface::initAsyncIO()
{
  for (size_t i = 0; i < 18; ++i) {
    standard_dig_out_bits_cmd_[i] = NO_NEW_CMD_;
  }

  for (size_t i = 0; i < 2; ++i) {
    standard_analog_output_cmd_[i] = NO_NEW_CMD_;
  }

  payload_mass_ = NO_NEW_CMD_;
  payload_center_of_gravity_ = { NO_NEW_CMD_, NO_NEW_CMD_, NO_NEW_CMD_ };
}

void URPositionHardwareInterface::checkAsyncIO()
{
  for (size_t i = 0; i < 18; ++i) {
    if (!std::isnan(standard_dig_out_bits_cmd_[i]) && ur_driver_ != nullptr) {
      if (i <= 7) {
        io_async_success_ =
            ur_driver_->getRTDEWriter().sendStandardDigitalOutput(i, static_cast<bool>(standard_dig_out_bits_cmd_[i]));
      } else if (i <= 15) {
        io_async_success_ = ur_driver_->getRTDEWriter().sendConfigurableDigitalOutput(
            static_cast<uint8_t>(i - 8), static_cast<bool>(standard_dig_out_bits_cmd_[i]));
      } else {
        io_async_success_ = ur_driver_->getRTDEWriter().sendToolDigitalOutput(
            static_cast<uint8_t>(i - 16), static_cast<bool>(standard_dig_out_bits_cmd_[i]));
      }
      standard_dig_out_bits_cmd_[i] = NO_NEW_CMD_;
    }
  }

  for (size_t i = 0; i < 2; ++i) {
    if (!std::isnan(standard_analog_output_cmd_[i]) && ur_driver_ != nullptr) {
      io_async_success_ = ur_driver_->getRTDEWriter().sendStandardAnalogOutput(i, standard_analog_output_cmd_[i]);
      standard_analog_output_cmd_[i] = NO_NEW_CMD_;
    }
  }

  if (!std::isnan(target_speed_fraction_cmd_) && ur_driver_ != nullptr) {
    scaling_async_success_ = ur_driver_->getRTDEWriter().sendSpeedSlider(target_speed_fraction_cmd_);
    target_speed_fraction_cmd_ = NO_NEW_CMD_;
  }

  if (!std::isnan(resend_robot_program_cmd_) && ur_driver_ != nullptr) {
    try {
      resend_robot_program_async_success_ = ur_driver_->sendRobotProgram();
    } catch (const urcl::UrException& e) {
      RCLCPP_ERROR(rclcpp::get_logger("URPositionHardwareInterface"), "Service Call failed: '%s'", e.what());
    }
    resend_robot_program_cmd_ = NO_NEW_CMD_;
  }

  if (!std::isnan(payload_mass_) && !std::isnan(payload_center_of_gravity_[0]) &&
      !std::isnan(payload_center_of_gravity_[1]) && !std::isnan(payload_center_of_gravity_[2]) &&
      ur_driver_ != nullptr) {
    try {
      // create command as string from interfaces
      // ROS1 driver hardware_interface.cpp#L450-L456
      std::stringstream str_command;
      str_command.imbue(std::locale::classic());
      str_command << "sec setup():" << std::endl
                  << " set_payload(" << payload_mass_ << ", [" << payload_center_of_gravity_[0] << ", "
                  << payload_center_of_gravity_[1] << ", " << payload_center_of_gravity_[2] << "])" << std::endl
                  << "end";
      payload_async_success_ = ur_driver_->sendScript(str_command.str());
    } catch (const urcl::UrException& e) {
      RCLCPP_ERROR(rclcpp::get_logger("URPositionHardwareInterface"), "Service Call failed: '%s'", e.what());
    }
    payload_mass_ = NO_NEW_CMD_;
    payload_center_of_gravity_ = { NO_NEW_CMD_, NO_NEW_CMD_, NO_NEW_CMD_ };
  }
}

void URPositionHardwareInterface::updateNonDoubleValues()
{
  for (size_t i = 0; i < 18; ++i) {
    actual_dig_out_bits_copy_[i] = static_cast<double>(actual_dig_out_bits_[i]);
    actual_dig_in_bits_copy_[i] = static_cast<double>(actual_dig_in_bits_[i]);
  }

  for (size_t i = 0; i < 11; ++i) {
    safety_status_bits_copy_[i] = static_cast<double>(safety_status_bits_[i]);
  }

  for (size_t i = 0; i < 4; ++i) {
    analog_io_types_copy_[i] = static_cast<double>(analog_io_types_[i]);
    robot_status_bits_copy_[i] = static_cast<double>(robot_status_bits_[i]);
  }

  for (size_t i = 0; i < 2; ++i) {
    tool_analog_input_types_copy_[i] = static_cast<double>(tool_analog_input_types_[i]);
  }

  tool_output_voltage_copy_ = static_cast<double>(tool_output_voltage_);
  robot_mode_copy_ = static_cast<double>(robot_mode_);
  safety_mode_copy_ = static_cast<double>(safety_mode_);
  tool_mode_copy_ = static_cast<double>(tool_mode_);
  system_interface_initialized_ = initialized_ ? 1.0 : 0.0;
  robot_program_running_copy_ = robot_program_running_ ? 1.0 : 0.0;
}

void URPositionHardwareInterface::transformForceTorque()
{
  // imported from ROS1 driver - hardware_interface.cpp#L867-L876
  tcp_force_.setValue(urcl_ft_sensor_measurements_[0], urcl_ft_sensor_measurements_[1],
                      urcl_ft_sensor_measurements_[2]);
  tcp_torque_.setValue(urcl_ft_sensor_measurements_[3], urcl_ft_sensor_measurements_[4],
                       urcl_ft_sensor_measurements_[5]);

  tf2::Quaternion rotation_quat;
  tf2::fromMsg(tcp_transform_.transform.rotation, rotation_quat);
  tcp_force_ = tf2::quatRotate(rotation_quat.inverse(), tcp_force_);
  tcp_torque_ = tf2::quatRotate(rotation_quat.inverse(), tcp_torque_);

  urcl_ft_sensor_measurements_ = { tcp_force_.x(),  tcp_force_.y(),  tcp_force_.z(),
                                   tcp_torque_.x(), tcp_torque_.y(), tcp_torque_.z() };
}

void URPositionHardwareInterface::extractToolPose()
{
  // imported from ROS1 driver hardware_interface.cpp#L911-L928
  double tcp_angle =
      std::sqrt(std::pow(urcl_tcp_pose_[3], 2) + std::pow(urcl_tcp_pose_[4], 2) + std::pow(urcl_tcp_pose_[5], 2));

  tf2::Vector3 rotation_vec(urcl_tcp_pose_[3], urcl_tcp_pose_[4], urcl_tcp_pose_[5]);
  tf2::Quaternion rotation;
  if (tcp_angle > 1e-16) {
    rotation.setRotation(rotation_vec.normalized(), tcp_angle);
  } else {
    rotation.setValue(0.0, 0.0, 0.0, 1.0);  // default Quaternion is 0,0,0,0 which is invalid
  }
  tcp_transform_.transform.translation.x = urcl_tcp_pose_[0];
  tcp_transform_.transform.translation.y = urcl_tcp_pose_[1];
  tcp_transform_.transform.translation.z = urcl_tcp_pose_[2];

  tcp_transform_.transform.rotation = tf2::toMsg(rotation);
}

hardware_interface::return_type URPositionHardwareInterface::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces)
{
  hardware_interface::return_type ret_val = hardware_interface::return_type::OK;

  start_modes_.clear();
  stop_modes_.clear();

  // Starting interfaces
  // add start interface per joint in tmp var for later check
  for (const auto& key : start_interfaces) {
    for (auto i = 0u; i < info_.joints.size(); i++) {
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
        start_modes_.push_back(hardware_interface::HW_IF_POSITION);
      }
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) {
        start_modes_.push_back(hardware_interface::HW_IF_VELOCITY);
      }
    }
  }
  // set new mode to all interfaces at the same time
  if (start_modes_.size() != 0 && start_modes_.size() != 6) {
    ret_val = hardware_interface::return_type::ERROR;
  }

  // all start interfaces must be the same - can't mix position and velocity control
  if (start_modes_.size() != 0 && !std::equal(start_modes_.begin() + 1, start_modes_.end(), start_modes_.begin())) {
    ret_val = hardware_interface::return_type::ERROR;
  }

  // Stopping interfaces
  // add stop interface per joint in tmp var for later check
  for (const auto& key : stop_interfaces) {
    for (auto i = 0u; i < info_.joints.size(); i++) {
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION) {
        stop_modes_.push_back(StoppingInterface::STOP_POSITION);
      }
      if (key == info_.joints[i].name + "/" + hardware_interface::HW_IF_VELOCITY) {
        stop_modes_.push_back(StoppingInterface::STOP_VELOCITY);
      }
    }
  }
  // stop all interfaces at the same time
  if (stop_modes_.size() != 0 &&
      (stop_modes_.size() != 6 || !std::equal(stop_modes_.begin() + 1, stop_modes_.end(), stop_modes_.begin()))) {
    ret_val = hardware_interface::return_type::ERROR;
  }

  controllers_initialized_ = true;
  return ret_val;
}

hardware_interface::return_type URPositionHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces, const std::vector<std::string>& stop_interfaces)
{
  hardware_interface::return_type ret_val = hardware_interface::return_type::OK;

  if (stop_modes_.size() != 0 &&
      std::find(stop_modes_.begin(), stop_modes_.end(), StoppingInterface::STOP_POSITION) != stop_modes_.end()) {
    position_controller_running_ = false;
    urcl_position_commands_ = urcl_position_commands_old_ = urcl_joint_positions_;
  } else if (stop_modes_.size() != 0 &&
             std::find(stop_modes_.begin(), stop_modes_.end(), StoppingInterface::STOP_VELOCITY) != stop_modes_.end()) {
    velocity_controller_running_ = false;
    urcl_velocity_commands_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
  }

  if (start_modes_.size() != 0 &&
      std::find(start_modes_.begin(), start_modes_.end(), hardware_interface::HW_IF_POSITION) != start_modes_.end()) {
    velocity_controller_running_ = false;
    urcl_position_commands_ = urcl_position_commands_old_ = urcl_joint_positions_;
    position_controller_running_ = true;

  } else if (start_modes_.size() != 0 && std::find(start_modes_.begin(), start_modes_.end(),
                                                   hardware_interface::HW_IF_VELOCITY) != start_modes_.end()) {
    position_controller_running_ = false;
    urcl_velocity_commands_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 } };
    velocity_controller_running_ = true;
  }

  start_modes_.clear();
  stop_modes_.clear();

  return ret_val;
}
}  // namespace ur_robot_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ur_robot_driver::URPositionHardwareInterface, hardware_interface::SystemInterface)
