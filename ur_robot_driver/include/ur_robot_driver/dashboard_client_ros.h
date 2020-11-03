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
 * \date    2019-10-21
 *
 */
//----------------------------------------------------------------------

#ifndef UR_ROBOT_DRIVER_ROS_DASHBOARD_CLIENT_H_INCLUDED
#define UR_ROBOT_DRIVER_ROS_DASHBOARD_CLIENT_H_INCLUDED

#include <regex>

#include <ur_client_library/ur/dashboard_client.h>

#include <ur_dashboard_msgs/msg/program_state.hpp>
#include <ur_dashboard_msgs/srv/add_to_log.hpp>
#include <ur_dashboard_msgs/srv/get_loaded_program.hpp>
#include <ur_dashboard_msgs/srv/get_program_state.hpp>
#include <ur_dashboard_msgs/srv/get_robot_mode.hpp>
#include <ur_dashboard_msgs/srv/get_safety_mode.hpp>
#include <ur_dashboard_msgs/srv/is_program_running.hpp>
#include <ur_dashboard_msgs/srv/is_program_saved.hpp>
#include <ur_dashboard_msgs/srv/load.hpp>
#include <ur_dashboard_msgs/srv/popup.hpp>
#include <ur_dashboard_msgs/srv/raw_request.hpp>

namespace ur_driver
{
/*!
 * \brief ROS wrapper for UR's dashboard server access. Many (not necessarily all) dashboard
 * functionalities are wrapped into ROS services here.
 */
class DashboardClientROS
{
};
}  // namespace ur_driver
#endif  // ifndef UR_ROBOT_DRIVER_ROS_DASHBOARD_CLIENT_H_INCLUDED
