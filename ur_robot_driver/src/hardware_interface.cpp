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

namespace ur_robot_driver
{
hardware_interface::return_type URHardwareInterface::configure(const HardwareInfo& system_info)
{
  info_ = system_info;
  status_ = status::CONFIGURED;

  current_joint_angles_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  joint_angle_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo& joint : info_.joints)
  {
    if (joint.type.compare("ros2_control_components/PositionJoint") != 0)
    {
      status_ = status::UNKNOWN;
      return return_type::ERROR;
    }
  }

  return return_type::OK;
}
}  // namespace ur_robot_driver
