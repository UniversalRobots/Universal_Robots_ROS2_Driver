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
 * \author  Marvin Große Besselmann grosse@fzi.de
 * \date    2021-02-18
 *
 */
//----------------------------------------------------------------------
#ifndef UR_CONTROLLERS_SCALED_TRAJECTORY_CONTROLLER_H_INCLUDED
#define UR_CONTROLLERS_SCALED_TRAJECTORY_CONTROLLER_H_INCLUDED

#include "joint_trajectory_controller/joint_trajectory_controller.hpp"

namespace ur_controllers
{
// template <class SegmentImpl, class HardwareInterface>
class ScaledJointTrajectoryController : public joint_trajectory_controller::JointTrajectoryController
{
public:
  ScaledJointTrajectoryController() = default;
  ~ScaledJointTrajectoryController() override = default;

  controller_interface::InterfaceConfiguration state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration conf;
    conf = JointTrajectoryController::state_interface_configuration();
    conf.names.push_back("speed_scaling/speed_scaling_factor");
    return conf;
  }

  controller_interface::return_type update()
  {
    if (state_interfaces_.back().get_name() == "speed_scaling")
    {
      scaling_factor_ = state_interfaces_.back().get_value();
    }
    else
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Speed scaling interface not fount. This should not have happended");
    }

    return JointTrajectoryController::update();
  }

private:
  double scaling_factor_;
};
}  // namespace ur_controllers

#endif  // ifndef UR_CONTROLLERS_SCALED_TRAJECTORY_CONTROLLER_H_INCLUDED
