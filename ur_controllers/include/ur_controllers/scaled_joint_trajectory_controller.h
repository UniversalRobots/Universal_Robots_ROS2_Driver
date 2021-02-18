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
  virtual ~ScaledJointTrajectoryController() = default;

private:
  double scaling_factor_;
};
}  // namespace ur_controllers

#endif  // ifndef UR_CONTROLLERS_SCALED_TRAJECTORY_CONTROLLER_H_INCLUDED
