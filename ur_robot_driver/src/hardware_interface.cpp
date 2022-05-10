// Copyright 2019, FZI Forschungszentrum Informatik
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

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
hardware_interface::return_type URPositionHardwareInterface::configure(const HardwareInfo& system_info)
{
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
  runtime_state_ = static_cast<uint32_t>(rtde::RUNTIME_STATE::STOPPED);
  pausing_state_ = PausingState::RUNNING;
  pausing_ramp_up_increment_ = 0.01;
  controllers_initialized_ = false;
  first_pass_ = true;
  initialized_ = false;
  async_thread_shutdown_ = false;
  system_interface_initialized_ = 0.0;

  for (const hardware_interface::ComponentInfo& joint : info_.joints) {
    if (joint.name == "gpio" || joint.name == "speed_scaling") {
      continue;
    }
    if (joint.command_interfaces.size() != 2) {
      RCLCPP_FATAL(rclcpp::get_logger("URPositionHardwareInterface"),
                   "Joint '%s' has %d command interfaces found. 2 expected.", joint.name.c_str(),
                   joint.command_interfaces.size());
      return return_type::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(rclcpp::get_logger("URPositionHardwareInterface"),
                   "Joint '%s' have %s command interfaces found as first command interface. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return return_type::ERROR;
    }

    if (joint.command_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(rclcpp::get_logger("URPositionHardwareInterface"),
                   "Joint '%s' have %s command interfaces found as second command interface. '%s' expected.",
                   joint.name.c_str(), joint.command_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return return_type::ERROR;
    }

    if (joint.state_interfaces.size() != 3) {
      RCLCPP_FATAL(rclcpp::get_logger("URPositionHardwareInterface"), "Joint '%s' has %d state interface. 3 expected.",
                   joint.name.c_str(), joint.state_interfaces.size());
      return return_type::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(rclcpp::get_logger("URPositionHardwareInterface"),
                   "Joint '%s' have %s state interface as first state interface. '%s' expected.", joint.name.c_str(),
                   joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return return_type::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(rclcpp::get_logger("URPositionHardwareInterface"),
                   "Joint '%s' have %s state interface as second state interface. '%s' expected.", joint.name.c_str(),
                   joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_POSITION);
      return return_type::ERROR;
    }

    if (joint.state_interfaces[2].name != hardware_interface::HW_IF_EFFORT) {
      RCLCPP_FATAL(rclcpp::get_logger("URPositionHardwareInterface"),
                   "Joint '%s' have %s state interface as third state interface. '%s' expected.", joint.name.c_str(),
                   joint.state_interfaces[2].name.c_str(), hardware_interface::HW_IF_POSITION);
      return return_type::ERROR;
    }
  }

  status_ = status::CONFIGURED;

  return return_type::OK;
}

std::vector<hardware_interface::StateInterface> URPositionHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    if (info_.joints[i].name == "gpio" || info_.joints[i].name == "speed_scaling") {
      continue;
    }
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
    if (info_.joints[i].name == "gpio" || info_.joints[i].name == "speed_scaling") {
      continue;
    }
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

return_type URPositionHardwareInterface::start()
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
  bool headless_mode = static_cast<bool>(stoi(info_.hardware_parameters["headless_mode"]));
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

  bool use_tool_communication = static_cast<bool>(stoi(info_.hardware_parameters["use_tool_communication"]));

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

    return return_type::ERROR;
  } catch (urcl::UrException& e) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("URPositionHardwareInterface"), e.what());
    return return_type::ERROR;
  }
  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Calibration checksum: '%s'.",
              calibration_checksum.c_str());
  // check calibration
  // https://github.com/UniversalRobots/Universal_Robots_ROS_Driver/blob/c3378599d5fa73a261328b326392e847f312ab6b/ur_robot_driver/src/hardware_interface.cpp#L296-L309
  if (ur_driver_->checkCalibration(calibration_checksum)) {
    RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Calibration checked successfully.");
  } else {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("URPositionHardwareInterface"), "The calibration parameters of the "
                                                                           "connected robot don't match the ones from "
                                                                           "the given kinematics "
                                                                           "config file. Please be aware that this can "
                                                                           "lead to critical inaccuracies of tcp "
                                                                           "positions. Use "
                                                                           "the ur_calibration tool to extract the "
                                                                           "correct calibration from the robot and "
                                                                           "pass that into the "
                                                                           "description. See "
                                                                           "[https://github.com/UniversalRobots/"
                                                                           "Universal_Robots_ROS_Driver#extract-"
                                                                           "calibration-information] "
                                                                           "for details.");
  }

  ur_driver_->startRTDECommunication();

  async_thread_ = std::make_shared<std::thread>(&URPositionHardwareInterface::asyncThread, this);

  status_ = hardware_interface::status::STARTED;

  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "System successfully started!");

  return return_type::OK;
}

return_type URPositionHardwareInterface::stop()
{
  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Stopping ...please wait...");

  async_thread_shutdown_ = true;
  async_thread_->join();
  async_thread_.reset();

  ur_driver_.reset();

  unregisterUrclLogHandler();

  status_ = hardware_interface::status::STOPPED;

  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "System successfully stopped!");

  return return_type::OK;
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
    std::this_thread::sleep_for(std::chrono::nanoseconds(20000));
  }
}

return_type URPositionHardwareInterface::read()
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
      target_speed_fraction_cmd_ = NO_NEW_CMD_;
      initialized_ = true;
    }

    updateNonDoubleValues();

    return return_type::OK;
  }

  return return_type::ERROR;
}

return_type URPositionHardwareInterface::write()
{
  if ((runtime_state_ == static_cast<uint32_t>(rtde::RUNTIME_STATE::PLAYING) ||
       runtime_state_ == static_cast<uint32_t>(rtde::RUNTIME_STATE::PAUSING)) &&
      robot_program_running_ && (!non_blocking_read_ || packet_read_)) {
    bool new_data_available = false;
    // create a lambda subtract functor
    std::function<double(double, double)> substractor = [](double a, double b) { return std::abs(a - b); };

    // create a position difference vector
    std::vector<double> pos_diff;
    pos_diff.resize(urcl_position_commands_.size());
    std::transform(urcl_position_commands_.begin(), urcl_position_commands_.end(), urcl_position_commands_old_.begin(),
                   pos_diff.begin(), substractor);

    double pos_diff_sum = 0.0;
    std::for_each(pos_diff.begin(), pos_diff.end(), [&pos_diff_sum](double a) { return pos_diff_sum += a; });

    if (pos_diff_sum != 0.0) {
      new_data_available = true;
    }

    if (new_data_available) {
      ur_driver_->writeJointCommand(urcl_position_commands_, urcl::comm::ControlMode::MODE_SERVOJ);
      // remember old values
      urcl_position_commands_old_ = urcl_position_commands_;
    } else {
      ur_driver_->writeKeepalive();
    }

    packet_read_ = false;

    // remember old values
    urcl_position_commands_old_ = urcl_position_commands_;

    return return_type::OK;
  } else {
    // TODO(anyone): could not read from the driver --> reset controllers
    return return_type::ERROR;
  }
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
}  // namespace ur_robot_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ur_robot_driver::URPositionHardwareInterface, hardware_interface::SystemInterface)
