// Copyright 2019, FZI Forschungszentrum Informatik
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the {copyright_holder} nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Felix Exner exner@fzi.de
 * \date    2019-10-21
 * \author  Marvin Große Besselmann grosse@fzi.de
 * \date    2021-03-22
 *
 */
//----------------------------------------------------------------------

#include <ur_client_library/exceptions.h>
#include <ur_client_library/primary/primary_client.h>
#include <ur_client_library/ur/dashboard_client.h>

#include <memory>
#include <string>

#include <ur_robot_driver/dashboard_client_ros.hpp>

namespace ur_robot_driver
{
DashboardClientROS::DashboardClientROS(const rclcpp::Node::SharedPtr& node, const std::string& robot_ip)
  : node_(node), primary_client_(robot_ip, notifier_)
{
  node_->declare_parameter<double>("receive_timeout", 2);

  primary_client_.start(10, std::chrono::seconds(10));
  auto robot_version = primary_client_.getRobotVersion();
  primary_client_.stop();
  auto dashboard_policy = urcl::DashboardClient::ClientPolicy::G5;

  if (robot_version->major > 5) {
    if (robot_version->major == 10 && robot_version->minor < 11) {
      RCLCPP_FATAL(node_->get_logger(),
                   "The dashboard server for PolyScope X is only available from version 10.11.0 and later. The "
                   "connected robot has version %s. Exiting now.",
                   robot_version->toString().c_str());
      exit(1);
    }
    dashboard_policy = urcl::DashboardClient::ClientPolicy::POLYSCOPE_X;
  }

  RCLCPP_INFO(node_->get_logger(), "Connecting to Dashboard Server at %s with policy %s", robot_ip.c_str(),
              dashboard_policy == urcl::DashboardClient::ClientPolicy::G5 ? "G5" : "Polyscope X");
  client_ = std::make_unique<urcl::DashboardClient>(robot_ip, dashboard_policy);

  connect();

  // Service to release the brakes. If the robot is currently powered off, it will get powered on on the fly.
  brake_release_service_ = createDashboardTriggerSrv(
      "~/brake_release", std::bind(&urcl::DashboardClient::commandBrakeReleaseWithResponse, client_.get()));

  // If this service is called the operational mode can again be changed from PolyScope, and the user password is
  // enabled.
  clear_operational_mode_service_ = createDashboardTriggerSrv(
      "~/clear_operational_mode",
      std::bind(&urcl::DashboardClient::commandClearOperationalModeWithResponse, client_.get()));

  // Close a (non-safety) popup on the teach pendant.
  close_popup_service_ = createDashboardTriggerSrv(
      "~/close_popup", std::bind(&urcl::DashboardClient::commandClosePopupWithResponse, client_.get()));

  // Close a safety popup on the teach pendant.
  close_safety_popup_service_ = createDashboardTriggerSrv(
      "~/close_safety_popup", std::bind(&urcl::DashboardClient::commandCloseSafetyPopupWithResponse, client_.get()));

  // Pause a running program.
  pause_service_ =
      createDashboardTriggerSrv("~/pause", std::bind(&urcl::DashboardClient::commandPauseWithResponse, client_.get()));

  // Start execution of a previously loaded program
  play_service_ =
      createDashboardTriggerSrv("~/play", std::bind(&urcl::DashboardClient::commandPlayWithResponse, client_.get()));

  if (dashboard_policy == urcl::DashboardClient::ClientPolicy::POLYSCOPE_X) {
    resume_service_ = createDashboardTriggerSrv(
        "~/resume", std::bind(&urcl::DashboardClient::commandResumeWithResponse, client_.get()));
  }

  // Power off the robot motors
  power_off_service_ = createDashboardTriggerSrv(
      "~/power_off", std::bind(&urcl::DashboardClient::commandPowerOffWithResponse, client_.get()));

  // Power on the robot motors. To fully start the robot, call 'brake_release' afterwards.
  power_on_service_ =
      createDashboardTriggerSrv("~/power_on", std::bind(&urcl::DashboardClient::commandPowerOnWithResponse,
                                                        client_.get(), std::chrono::seconds(300)));

  // Used when robot gets a safety fault or violation to restart the safety. After safety has been rebooted the robot
  // will be in Power Off. NOTE: You should always ensure it is okay to restart the system. It is highly recommended to
  // check the error log before using this command (either via PolyScope or e.g. ssh connection).
  restart_safety_service_ = createDashboardTriggerSrv(
      "~/restart_safety", std::bind(&urcl::DashboardClient::commandRestartSafetyWithResponse, client_.get()));

  // Shutdown the robot controller
  shutdown_service_ = createDashboardTriggerSrv(
      "~/shutdown", std::bind(&urcl::DashboardClient::commandShutdownWithResponse, client_.get()));

  // Stop program execution on the robot
  stop_service_ =
      createDashboardTriggerSrv("~/stop", std::bind(&urcl::DashboardClient::commandStopWithResponse, client_.get()));

  // Dismiss a protective stop to continue robot movements. NOTE: It is the responsibility of the user to ensure the
  // cause of the protective stop is resolved before calling this service.
  unlock_protective_stop_service_ = createDashboardTriggerSrv(
      "~/unlock_protective_stop",
      std::bind(&urcl::DashboardClient::commandUnlockProtectiveStopWithResponse, client_.get()));

  // Query whether there is currently a program running
  running_service_ = node_->create_service<ur_dashboard_msgs::srv::IsProgramRunning>(
      "~/program_running",
      std::bind(&DashboardClientROS::handleRunningQuery, this, std::placeholders::_1, std::placeholders::_2));

  // Load a robot installation from a file
  get_loaded_program_service_ = node_->create_service<ur_dashboard_msgs::srv::GetLoadedProgram>(
      "~/get_loaded_program", [&](const ur_dashboard_msgs::srv::GetLoadedProgram::Request::SharedPtr req,
                                  ur_dashboard_msgs::srv::GetLoadedProgram::Response::SharedPtr resp) {
        auto dashboard_response =
            dashboardCallWithChecks([this, req]() { return client_->commandGetLoadedProgramWithResponse(); }, resp);
        if (resp->success) {
          handleDashboardResponseData(
              [dashboard_response, resp]() {
                resp->program_name = std::get<std::string>(dashboard_response.data.at("program_name"));
              },
              resp, dashboard_response);
        }
        return true;
      });

  // Load a robot installation from a file
  load_installation_service_ = node_->create_service<ur_dashboard_msgs::srv::Load>(
      "~/load_installation", [&](const ur_dashboard_msgs::srv::Load::Request::SharedPtr req,
                                 ur_dashboard_msgs::srv::Load::Response::SharedPtr resp) {
        auto dashboard_response = dashboardCallWithChecks(
            [this, req]() { return client_->commandLoadInstallationWithResponse(req->filename); }, resp);
        return true;
      });

  // Load a robot program from a file
  load_program_service_ = node->create_service<ur_dashboard_msgs::srv::Load>(
      "~/load_program", [&](const ur_dashboard_msgs::srv::Load::Request::SharedPtr req,
                            ur_dashboard_msgs::srv::Load::Response::SharedPtr resp) {
        auto dashboard_response = dashboardCallWithChecks(
            [this, req]() { return client_->commandLoadProgramWithResponse(req->filename); }, resp);
        if (dashboard_response.data.find("status_code") != dashboard_response.data.end()) {
          resp->answer += ", status_code: " + std::to_string(std::get<int>(dashboard_response.data["status_code"]));
        }
        return true;
      });

  // // Query whether the current program is saved
  is_program_saved_service_ = node_->create_service<ur_dashboard_msgs::srv::IsProgramSaved>(
      "~/program_saved",
      std::bind(&DashboardClientROS::handleSavedQuery, this, std::placeholders::_1, std::placeholders::_2));

  // Service to show a popup on the UR Teach pendant.
  popup_service_ = node_->create_service<ur_dashboard_msgs::srv::Popup>(
      "~/popup", [&](ur_dashboard_msgs::srv::Popup::Request::SharedPtr req,
                     ur_dashboard_msgs::srv::Popup::Response::SharedPtr resp) {
        auto dashboard_response =
            dashboardCallWithChecks([this, req]() { return client_->commandPopupWithResponse(req->message); }, resp);

        return true;
      });

  // Service to query the current program state
  program_state_service_ = node_->create_service<ur_dashboard_msgs::srv::GetProgramState>(
      "~/program_state", [&](const ur_dashboard_msgs::srv::GetProgramState::Request::SharedPtr /*unused*/,
                             ur_dashboard_msgs::srv::GetProgramState::Response::SharedPtr resp) {
        auto dashboard_response =
            dashboardCallWithChecks([this]() { return client_->commandProgramStateWithResponse(); }, resp);

        if (resp->success) {
          handleDashboardResponseData(
              [dashboard_response, resp]() {
                resp->state.state = std::get<std::string>(dashboard_response.data.at("program_state"));
                // PolyScope X doesn't report the program name
                if (dashboard_response.data.find("program_name") != dashboard_response.data.end()) {
                  resp->program_name = std::get<std::string>(dashboard_response.data.at("program_name"));
                }
              },
              resp, dashboard_response);
        }
        return true;
      });

  // Service to query the current safety mode
  safety_mode_service_ = node_->create_service<ur_dashboard_msgs::srv::GetSafetyMode>(
      "~/get_safety_mode",
      std::bind(&DashboardClientROS::handleSafetyModeQuery, this, std::placeholders::_1, std::placeholders::_2));

  // Service to query the current robot mode
  robot_mode_service_ = node_->create_service<ur_dashboard_msgs::srv::GetRobotMode>(
      "~/get_robot_mode",
      std::bind(&DashboardClientROS::handleRobotModeQuery, this, std::placeholders::_1, std::placeholders::_2));

  // Service to add a message to the robot's log
  add_to_log_service_ = node->create_service<ur_dashboard_msgs::srv::AddToLog>(
      "~/add_to_log", [&](const ur_dashboard_msgs::srv::AddToLog::Request::SharedPtr req,
                          ur_dashboard_msgs::srv::AddToLog::Response::SharedPtr resp) {
        dashboardCallWithChecks([this, req]() { return client_->commandAddToLogWithResponse(req->message); }, resp);
        return true;
      });

  // General purpose service to send arbitrary messages to the dashboard server
  raw_request_service_ = node_->create_service<ur_dashboard_msgs::srv::RawRequest>(
      "~/raw_request", [&](const ur_dashboard_msgs::srv::RawRequest::Request::SharedPtr req,
                           ur_dashboard_msgs::srv::RawRequest::Response::SharedPtr resp) {
        try {
          resp->answer = this->client_->sendAndReceive(req->query + "\n");
        } catch (const urcl::UrException& e) {
          RCLCPP_ERROR(rclcpp::get_logger("Dashboard_Client"), "Service Call failed: '%s'", e.what());
          resp->answer = e.what();
        }
        return true;
      });

  // Service to reconnect to the dashboard server
  reconnect_service_ = node_->create_service<std_srvs::srv::Trigger>(
      "~/connect",
      [&](const std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr resp) {
        try {
          resp->success = connect();
        } catch (const urcl::UrException& e) {
          RCLCPP_ERROR(rclcpp::get_logger("Dashboard_Client"), "Service Call failed: '%s'", e.what());
          resp->message = e.what();
          resp->success = false;
        }
        return true;
      });

  // Disconnect from the dashboard service.
  quit_service_ =
      createDashboardTriggerSrv("~/quit", std::bind(&urcl::DashboardClient::commandQuitWithResponse, client_.get()));

  // Service to query whether the robot is in remote control.
  is_in_remote_control_service_ = node_->create_service<ur_dashboard_msgs::srv::IsInRemoteControl>(
      "~/is_in_remote_control",
      std::bind(&DashboardClientROS::handleRemoteControlQuery, this, std::placeholders::_1, std::placeholders::_2));
}

bool DashboardClientROS::connect()
{
  timeval tv;
  // Timeout after which a call to the dashboard server will be considered failure if no answer has been received.
  double time_buffer = 0;
  node_->get_parameter("receive_timeout", time_buffer);
  tv.tv_sec = time_buffer;
  tv.tv_usec = 0;
  client_->setReceiveTimeout(tv);
  return client_->connect();
}

bool DashboardClientROS::handleRunningQuery(const ur_dashboard_msgs::srv::IsProgramRunning::Request::SharedPtr req,
                                            ur_dashboard_msgs::srv::IsProgramRunning::Response::SharedPtr resp)
{
  auto dashboard_response = dashboardCallWithChecks([this]() { return client_->commandRunningWithResponse(); }, resp);
  if (resp->success) {
    handleDashboardResponseData(
        [dashboard_response, resp]() { resp->program_running = std::get<bool>(dashboard_response.data.at("running")); },
        resp, dashboard_response);
  }

  return true;
}

bool DashboardClientROS::handleSavedQuery(ur_dashboard_msgs::srv::IsProgramSaved::Request::SharedPtr req,
                                          ur_dashboard_msgs::srv::IsProgramSaved::Response::SharedPtr resp)
{
  auto dashboard_response =
      dashboardCallWithChecks([this]() { return client_->commandIsProgramSavedWithResponse(); }, resp);
  if (resp->success) {
    handleDashboardResponseData(
        [dashboard_response, resp]() {
          resp->program_saved = std::get<bool>(dashboard_response.data.at("saved"));
          if (dashboard_response.data.find("program_name") != dashboard_response.data.end()) {
            resp->program_name = std::get<std::string>(dashboard_response.data.at("program_name"));
          }
        },
        resp, dashboard_response);
  }

  return true;
}

bool DashboardClientROS::handleSafetyModeQuery(const ur_dashboard_msgs::srv::GetSafetyMode::Request::SharedPtr req,
                                               ur_dashboard_msgs::srv::GetSafetyMode::Response::SharedPtr resp)
{
  auto dashboard_response =
      dashboardCallWithChecks([this]() { return client_->commandSafetyModeWithResponse(); }, resp);
  if (resp->success) {
    handleDashboardResponseData(
        [dashboard_response, resp]() {
          const std::string safetymode_str = std::get<std::string>(dashboard_response.data.at("safety_mode"));
          if (safetymode_str == "NORMAL") {
            resp->safety_mode.mode = ur_dashboard_msgs::msg::SafetyMode::NORMAL;
          } else if (safetymode_str == "REDUCED") {
            resp->safety_mode.mode = ur_dashboard_msgs::msg::SafetyMode::REDUCED;
          } else if (safetymode_str == "PROTECTIVE_STOP") {
            resp->safety_mode.mode = ur_dashboard_msgs::msg::SafetyMode::PROTECTIVE_STOP;
          } else if (safetymode_str == "RECOVERY") {
            resp->safety_mode.mode = ur_dashboard_msgs::msg::SafetyMode::RECOVERY;
          } else if (safetymode_str == "SAFEGUARD_STOP") {
            resp->safety_mode.mode = ur_dashboard_msgs::msg::SafetyMode::SAFEGUARD_STOP;
          } else if (safetymode_str == "SYSTEM_EMERGENCY_STOP") {
            resp->safety_mode.mode = ur_dashboard_msgs::msg::SafetyMode::SYSTEM_EMERGENCY_STOP;
          } else if (safetymode_str == "ROBOT_EMERGENCY_STOP") {
            resp->safety_mode.mode = ur_dashboard_msgs::msg::SafetyMode::ROBOT_EMERGENCY_STOP;
          } else if (safetymode_str == "VIOLATION") {
            resp->safety_mode.mode = ur_dashboard_msgs::msg::SafetyMode::VIOLATION;
          } else if (safetymode_str == "FAULT") {
            resp->safety_mode.mode = ur_dashboard_msgs::msg::SafetyMode::FAULT;
          }
        },
        resp, dashboard_response);
  }
  return true;
}

bool DashboardClientROS::handleRobotModeQuery(const ur_dashboard_msgs::srv::GetRobotMode::Request::SharedPtr req,
                                              ur_dashboard_msgs::srv::GetRobotMode::Response::SharedPtr resp)
{
  auto dashboard_response = dashboardCallWithChecks([this]() { return client_->commandRobotModeWithResponse(); }, resp);
  if (resp->success) {
    handleDashboardResponseData(
        [dashboard_response, resp]() {
          const std::string robotmode_str = std::get<std::string>(dashboard_response.data.at("robot_mode"));
          if (robotmode_str == "NO_CONTROLLER") {
            resp->robot_mode.mode = ur_dashboard_msgs::msg::RobotMode::NO_CONTROLLER;
          } else if (robotmode_str == "DISCONNECTED") {
            resp->robot_mode.mode = ur_dashboard_msgs::msg::RobotMode::DISCONNECTED;
          } else if (robotmode_str == "CONFIRM_SAFETY") {
            resp->robot_mode.mode = ur_dashboard_msgs::msg::RobotMode::CONFIRM_SAFETY;
          } else if (robotmode_str == "BOOTING") {
            resp->robot_mode.mode = ur_dashboard_msgs::msg::RobotMode::BOOTING;
          } else if (robotmode_str == "POWER_OFF") {
            resp->robot_mode.mode = ur_dashboard_msgs::msg::RobotMode::POWER_OFF;
          } else if (robotmode_str == "POWER_ON") {
            resp->robot_mode.mode = ur_dashboard_msgs::msg::RobotMode::POWER_ON;
          } else if (robotmode_str == "IDLE") {
            resp->robot_mode.mode = ur_dashboard_msgs::msg::RobotMode::IDLE;
          } else if (robotmode_str == "BACKDRIVE") {
            resp->robot_mode.mode = ur_dashboard_msgs::msg::RobotMode::BACKDRIVE;
          } else if (robotmode_str == "RUNNING") {
            resp->robot_mode.mode = ur_dashboard_msgs::msg::RobotMode::RUNNING;
          } else if (robotmode_str == "UPDATING_FIRMWARE") {
            resp->robot_mode.mode = ur_dashboard_msgs::msg::RobotMode::UPDATING_FIRMWARE;
          }
        },
        resp, dashboard_response);
  }
  return true;
}

bool DashboardClientROS::handleRemoteControlQuery(
    const ur_dashboard_msgs::srv::IsInRemoteControl::Request::SharedPtr req,
    ur_dashboard_msgs::srv::IsInRemoteControl::Response::SharedPtr resp)
{
  try {
    resp->remote_control = this->client_->commandIsInRemoteControl();
    resp->success = true;
  } catch (const urcl::UrException& e) {
    RCLCPP_ERROR(rclcpp::get_logger("Dashboard_Client"), "Service Call failed: '%s'", e.what());
    resp->answer = e.what();
    resp->success = false;
  }
  return true;
}

}  // namespace ur_robot_driver
