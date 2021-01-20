// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik
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
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Lovro Ivanov lovro.ivanov@gmail.com
 * \author  Andy Zelenak zelenak@picknik.ai
 * \date    2019-04-11
 *
 */
//----------------------------------------------------------------------
#pragma once

// System
#include <memory>
#include <vector>

// ros2_control hardware_interface
#include "hardware_interface/actuator.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/sensor.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "hardware_interface/visibility_control.h"

// UR stuff
#include <ur_client_library/ur/ur_driver.h>
#include <ur_robot_driver/dashboard_client_ros.h>

// ROS
#include "rclcpp/macros.hpp"

using hardware_interface::Actuator;
using hardware_interface::HardwareInfo;
using hardware_interface::return_type;
using hardware_interface::Sensor;
using hardware_interface::status;

namespace ur_robot_driver
{
/*!
 * \brief The HardwareInterface class handles the interface between the ROS system and the main
 * driver. It contains the read and write methods of the main control loop and registers various ROS
 * topics and services.
 */
class URPositionHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(URPositionHardwareInterface);

  return_type configure(const HardwareInfo& system_info) final;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() final;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() final;

  status get_status() const final
  {
    return status_;
  }

  std::string get_name() const final
  {
    return info_.name;
  }

  return_type start() final;
  return_type stop() final;
  return_type read() final;
  return_type write() final;

  /*!
   * \brief Callback to handle a change in the current state of the URCaps program running on the
   * robot. Executed only on the state change.
   *
   * \param program_running True when the URCap program is running on the robot.
   */
  void handleRobotProgramState(bool program_running);

protected:
  template <typename T>
  void readData(const std::unique_ptr<urcl::rtde_interface::DataPackage>& data_pkg, const std::string& var_name,
                T& data);
  template <typename T, size_t N>
  void readBitsetData(const std::unique_ptr<urcl::rtde_interface::DataPackage>& data_pkg, const std::string& var_name,
                      std::bitset<N>& data);

  HardwareInfo info_;
  status status_;

  urcl::vector6d_t urcl_position_commands_;
  urcl::vector6d_t urcl_position_commands_old_;
  urcl::vector6d_t urcl_velocity_commands_;
  urcl::vector6d_t urcl_joint_positions_;
  urcl::vector6d_t urcl_joint_velocities_;
  urcl::vector6d_t urcl_joint_efforts_;
  urcl::vector6d_t urcl_ft_sensor_measurements_;
  urcl::vector6d_t urcl_tcp_pose_;
  bool packet_read_;

  uint32_t runtime_state_;

  std::bitset<18> actual_dig_out_bits_;
  std::bitset<18> actual_dig_in_bits_;
  std::array<double, 2> standard_analog_input_;
  std::array<double, 2> standard_analog_output_;
  std::bitset<4> analog_io_types_;
  uint32_t tool_mode_;
  std::bitset<2> tool_analog_input_types_;
  std::array<double, 2> tool_analog_input_;
  int32_t tool_output_voltage_;
  double tool_output_current_;
  double tool_temperature_;
  double speed_scaling_;
  double target_speed_fraction_;
  double speed_scaling_combined_;
  int32_t robot_mode_;
  int32_t safety_mode_;
  std::bitset<4> robot_status_bits_;
  std::bitset<11> safety_status_bits_;

  bool robot_program_running_;
  bool non_blocking_read_;

  bool position_interface_in_use_;

  std::unique_ptr<urcl::UrDriver> ur_driver_;
};
}  // namespace ur_robot_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ur_robot_driver::URPositionHardwareInterface, hardware_interface::SystemInterface)
