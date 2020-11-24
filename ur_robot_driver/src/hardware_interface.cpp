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
 * \author  Andy Zelenak zelenak@picknik.ai
 * \date    2020-11-9
 *
 */
//----------------------------------------------------------------------
#include <ur_robot_driver/hardware_interface.h>
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include <ur_client_library/ur/tool_communication.h>
#include <ur_client_library/exceptions.h>

#include "rclcpp/rclcpp.hpp"

namespace rtde = urcl::rtde_interface;

namespace ur_robot_driver
{
hardware_interface::return_type URPositionHardwareInterface::configure(const HardwareInfo& system_info)
{
  info_ = system_info;

  // resize and initialize
  commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // TODO all the checking from HardwareInfo which holds urdf info
//  for (const hardware_interface::ComponentInfo& joint : info_.joints)
//  {
//    if (joint.type.compare("ros2_control_components/PositionJoint") != 0)
//    {
//      status_ = status::UNKNOWN;
//      return return_type::ERROR;
//    }
//  }

  // TODO fetch parameters (robot_ip, write&read params, ...), this can also be done in start

  status_ = status::CONFIGURED;

  return return_type::OK;
}

std::vector<hardware_interface::StateInterface> URPositionHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(
        hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> URPositionHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(
        hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &commands_[i]));
  }

  return command_interfaces;
}

return_type URPositionHardwareInterface::start()
{
  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Starting ...please wait...");

  std::this_thread::sleep_for(std::chrono::seconds(2));

  // TODO initialize driver

  // TODO initialize dashboard client

  // set some default values
  // TODO replace with reading current state of the joints
  for (uint i = 0; i < states_.size(); i++)
  {
    if (std::isnan(states_[i]) || std::isnan(commands_[i]))
    {
      states_[i] = 0;
      commands_[i] = 0;
    }
  }

  status_ = hardware_interface::status::STARTED;

  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "System Sucessfully started!");

  return return_type::OK;
}

return_type URPositionHardwareInterface::stop()
{
  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Stopping ...please wait...");

  std::this_thread::sleep_for(std::chrono::seconds(2));

  // TODO stop/reset driver

  // TODO stop/reset dashboard client

  status_ = hardware_interface::status::STOPPED;

  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "System sucessfully stopped!");

  return return_type::OK;
}

template <typename T>
void URPositionHardwareInterface::readData(const std::unique_ptr<rtde::DataPackage>& data_pkg,
                                           const std::string& var_name, T& data)
{
  if (!data_pkg->getData(var_name, data))
  {
    // This throwing should never happen unless misconfigured
    std::string error_msg = "Did not find '" + var_name + "' in data sent from robot. This should not happen!";
    throw std::runtime_error(error_msg);
  }
}

template <typename T, size_t N>
void URPositionHardwareInterface::readBitsetData(const std::unique_ptr<rtde::DataPackage>& data_pkg,
                                                 const std::string& var_name, std::bitset<N>& data)
{
  if (!data_pkg->getData<T, N>(var_name, data))
  {
    // This throwing should never happen unless misconfigured
    std::string error_msg = "Did not find '" + var_name + "' in data sent from robot. This should not happen!";
    throw std::runtime_error(error_msg);
  }
}

return_type URPositionHardwareInterface::read()
{
  // TODO add receiving commands from driver

  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Reading ...");
  return return_type::OK;


  std::unique_ptr<rtde::DataPackage> data_pkg = ur_driver_->getDataPackage();
  if (data_pkg)
  {
    packet_read_ = true;
    readData(data_pkg, "actual_q", urcl_joint_positions_);
  }

  return return_type::OK;
}

return_type URPositionHardwareInterface::write()
{
  // TODO send commands_ to driver
  RCLCPP_INFO(rclcpp::get_logger("URPositionHardwareInterface"), "Writing ...");
  return return_type::OK;

  for (uint i = 0; i < info_.joints.size(); i++)
    urcl_position_commands_[i] = commands_[i];

  ur_driver_->writeJointCommand(urcl_position_commands_, urcl::comm::ControlMode::MODE_SERVOJ);

  packet_read_ = false;

  return return_type::OK;
}
}  // namespace ur_robot_driver
