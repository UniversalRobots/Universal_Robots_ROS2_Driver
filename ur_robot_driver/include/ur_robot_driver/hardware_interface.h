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
 * \author  Felix Exner exner@fzi.de
 * \date    2019-04-11
 *
 */
//----------------------------------------------------------------------
#pragma once

// System
#include <memory>
#include <vector>

// ros2_control hardware_interface
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/components/actuator.hpp"
#include "hardware_interface/components/sensor.hpp"
#include "hardware_interface/components/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_status_values.hpp"
#include "hardware_interface/visibility_control.h"

// ROS
#include "rclcpp/macros.hpp"

using hardware_interface::HardwareInfo;
using hardware_interface::return_type;
using hardware_interface::status;
using hardware_interface::components::Actuator;
using hardware_interface::components::Sensor;

namespace ur_robot_driver
{
/*!
 * \brief The HardwareInterface class handles the interface between the ROS system and the main
 * driver. It contains the read and write methods of the main control loop and registers various ROS
 * topics and services.
 */
class URHardwareInterface : public hardware_interface::components::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(URHardwareInterface);

  hardware_interface::return_type configure(const HardwareInfo& system_info) final;

  status get_status() const final
  {
    return status_;
  }

  return_type start() final;
  return_type stop() final;
  return_type read() final;
  return_type write() final;

protected:
  HardwareInfo info_;
  status status_;
  std::vector<double> joint_angle_commands_, current_joint_angles_;
};
}  // namespace ur_robot_driver
