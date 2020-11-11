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

#include <ur_robot_driver/dashboard_client_ros.h>

namespace ur_robot_driver
{
DashboardClientROS::DashboardClientROS(const rclcpp::Node::SharedPtr& node, const std::string& robot_ip)
  : node_(node), client_(robot_ip)
{
  connect();

  // Service to release the brakes. If the robot is currently powered off, it will get powered on on the fly.
  brake_release_service_ = createDashboardTriggerSrv("brake_release", "brake release\n", "Brake releasing");

  // If this service is called the operational mode can again be changed from PolyScope, and the user password is
  // enabled.
  clear_operational_mode_service_ = createDashboardTriggerSrv("clear_operational_mode", "clear operational mode\n",
                                                              "No longer controlling the operational mode\\. "
                                                              "Current "
                                                              "operational mode: "
                                                              "'(MANUAL|AUTOMATIC)'\\.");

  // Close a (non-safety) popup on the teach pendant.
  close_popup_service_ = createDashboardTriggerSrv("close_popup", "close popup\n", "closing popup");

  // Close a safety popup on the teach pendant.
  close_safety_popup_service_ =
      createDashboardTriggerSrv("close_safety_popup", "close safety popup\n", "closing safety popup");

  // Pause a running program.
  pause_service_ = createDashboardTriggerSrv("pause", "pause\n", "Pausing program");

  // Start execution of a previously loaded program
  play_service_ = createDashboardTriggerSrv("play", "play\n", "Starting program");

  // Power off the robot motors
  power_off_service_ = createDashboardTriggerSrv("power_off", "power off\n", "Powering off");

  // Power on the robot motors. To fully start the robot, call 'brake_release' afterwards.
  power_on_service_ = createDashboardTriggerSrv("power_on", "power on\n", "Powering on");

  // Used when robot gets a safety fault or violation to restart the safety. After safety has been rebooted the robot
  // will be in Power Off. NOTE: You should always ensure it is okay to restart the system. It is highly recommended to
  // check the error log before using this command (either via PolyScope or e.g. ssh connection).
  restart_safety_service_ = createDashboardTriggerSrv("restart_safety", "restart safety\n", "Restarting safety");

  // Shutdown the robot controller
  shutdown_service_ = createDashboardTriggerSrv("shutdown", "shutdown\n", "Shutting down");

  // Stop program execution on the robot
  stop_service_ = createDashboardTriggerSrv("stop", "stop\n", "Stopped");

  // Dismiss a protective stop to continue robot movements. NOTE: It is the responsibility of the user to ensure the
  // cause of the protective stop is resolved before calling this service.
  unlock_protective_stop_service_ =
      createDashboardTriggerSrv("unlock_protective_stop", "unlock protective stop\n", "Protective stop releasing");

  // Query whether there is currently a program running
  running_service_ = node_->create_service<ur_dashboard_msgs::srv::IsProgramRunning>(
      "program_running",
      std::bind(&DashboardClientROS::handleRunningQuery, this, std::placeholders::_1, std::placeholders::_2));

  // Load a robot installation from a file
  get_loaded_program_service_ = node_->create_service<ur_dashboard_msgs::srv::GetLoadedProgram>(
      "get_loaded_program", [&](const ur_dashboard_msgs::srv::GetLoadedProgram::Request::SharedPtr req,
                                ur_dashboard_msgs::srv::GetLoadedProgram::Response::SharedPtr resp) {
        resp->answer = this->client_.sendAndReceive("get loaded program\n");
        std::smatch match;
        std::regex expected("Loaded program: (.+)");
        resp->success = std::regex_match(resp->answer, match, expected);
        if (resp->success)
        {
          resp->program_name = match[1];
        }
        return true;
      });

  // Load a robot installation from a file
  load_installation_service_ = node_->create_service<ur_dashboard_msgs::srv::Load>(
      "load_installation", [&](const ur_dashboard_msgs::srv::Load::Request::SharedPtr req,
                               ur_dashboard_msgs::srv::Load::Response::SharedPtr resp) {
        resp->answer = this->client_.sendAndReceive("load installation " + req->filename + "\n");
        resp->success = std::regex_match(resp->answer, std::regex("Loading installation: .+"));
        return true;
      });

  // Load a robot program from a file
  load_program_service_ = node->create_service<ur_dashboard_msgs::srv::Load>(
      "load_program", [&](const ur_dashboard_msgs::srv::Load::Request::SharedPtr req,
                          ur_dashboard_msgs::srv::Load::Response::SharedPtr resp) {
        resp->answer = this->client_.sendAndReceive("load " + req->filename + "\n");
        resp->success = std::regex_match(resp->answer, std::regex("Loading program: .+"));
        return true;
      });
}

bool DashboardClientROS::connect()
{
  timeval tv;
  // Timeout after which a call to the dashboard server will be considered failure if no answer has been received.
  tv.tv_sec = node_->declare_parameter<double>("receive_timeout", 1);
  tv.tv_usec = 0;
  client_.setReceiveTimeout(tv);
  return client_.connect();
}

bool DashboardClientROS::handleRunningQuery(const ur_dashboard_msgs::srv::IsProgramRunning::Request::SharedPtr req,
                                            ur_dashboard_msgs::srv::IsProgramRunning::Response::SharedPtr resp)
{
  resp->answer = this->client_.sendAndReceive("running\n");
  std::regex expected("Program running: (true|false)");
  std::smatch match;
  resp->success = std::regex_match(resp->answer, match, expected);

  if (resp->success)
  {
    resp->program_running = (match[1] == "true");
  }

  return true;
}
}  // namespace ur_robot_driver
