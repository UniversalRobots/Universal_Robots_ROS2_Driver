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
 * \date    2019-11-04
 *
 */
//----------------------------------------------------------------------
#pragma once

#include <sstream>
#include <ur_client_library/ur/datatypes.h>
#include <ur_dashboard_msgs/msg/robot_mode.hpp>
#include <ur_dashboard_msgs/msg/safety_mode.hpp>
#include <ur_dashboard_msgs/action/set_mode.hpp>

namespace ur_robot_driver
{
/*!
 * \brief A small helper class around the robot state (constisting of 'robot_mode' and
 * 'safety_mode')
 *
 * This class logs any changes of the robot state and provides an action interface bring the robot
 * into a required state. For modifying the robot mode it uses the dashboard server interface
 * provided by the ur_robot_driver.
 */
class RobotStateHelper
{
};
}  // namespace ur_robot_driver
