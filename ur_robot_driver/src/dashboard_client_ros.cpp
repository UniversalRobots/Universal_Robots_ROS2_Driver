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

#include <ur_robot_driver/dashboard_client_ros.hpp>

#include <string>

namespace ur_robot_driver
{
DashboardClientROS::DashboardClientROS(const rclcpp::Node::SharedPtr& node, const std::string& robot_ip)
  : node_(node), client_(robot_ip)
{
  node_->declare_parameter<double>("receive_timeout", 1);
  connect();

  // Service to release the brakes. If the robot is currently powered off, it will get powered on on the fly.
  brake_release_service_ = createDashboardTriggerSrv("~/brake_release", "brake release\n", "Brake releasing");

  // If this service is called the operational mode can again be changed from PolyScope, and the user password is
  // enabled.
  clear_operational_mode_service_ = createDashboardTriggerSrv("~/clear_operational_mode", "clear operational mode\n",
                                                              "No longer controlling the operational mode\\. "
                                                              "Current "
                                                              "operational mode: "
                                                              "'(MANUAL|AUTOMATIC)'\\.");

  // Close a (non-safety) popup on the teach pendant.
  close_popup_service_ = createDashboardTriggerSrv("~/close_popup", "close popup\n", "closing popup");

  // Close a safety popup on the teach pendant.
  close_safety_popup_service_ =
      createDashboardTriggerSrv("~/close_safety_popup", "close safety popup\n", "closing safety popup");

  // Pause a running program.
  pause_service_ = createDashboardTriggerSrv("~/pause", "pause\n", "Pausing program");

  // Start execution of a previously loaded program
  play_service_ = createDashboardTriggerSrv("~/play", "play\n", "Starting program");

  // Power off the robot motors
  power_off_service_ = createDashboardTriggerSrv("~/power_off", "power off\n", "Powering off");

  // Power on the robot motors. To fully start the robot, call 'brake_release' afterwards.
  power_on_service_ = createDashboardTriggerSrv("~/power_on", "power on\n", "Powering on");

  // Used when robot gets a safety fault or violation to restart the safety. After safety has been rebooted the robot
  // will be in Power Off. NOTE: You should always ensure it is okay to restart the system. It is highly recommended to
  // check the error log before using this command (either via PolyScope or e.g. ssh connection).
  restart_safety_service_ = createDashboardTriggerSrv("~/restart_safety", "restart safety\n", "Restarting safety");

  // Shutdown the robot controller
  shutdown_service_ = createDashboardTriggerSrv("~/shutdown", "shutdown\n", "Shutting down");

  // Stop program execution on the robot
  stop_service_ = createDashboardTriggerSrv("~/stop", "stop\n", "Stopped");

  // Dismiss a protective stop to continue robot movements. NOTE: It is the responsibility of the user to ensure the
  // cause of the protective stop is resolved before calling this service.
  unlock_protective_stop_service_ =
      createDashboardTriggerSrv("~/unlock_protective_stop", "unlock protective stop\n", "Protective stop releasing");

  // Query whether there is currently a program running
  running_service_ = node_->create_service<ur_dashboard_msgs::srv::IsProgramRunning>(
      "~/program_running",
      std::bind(&DashboardClientROS::handleRunningQuery, this, std::placeholders::_1, std::placeholders::_2));

  // Load a robot installation from a file
  get_loaded_program_service_ = node_->create_service<ur_dashboard_msgs::srv::GetLoadedProgram>(
      "~/get_loaded_program", [&](const ur_dashboard_msgs::srv::GetLoadedProgram::Request::SharedPtr req,
                                  ur_dashboard_msgs::srv::GetLoadedProgram::Response::SharedPtr resp) {
        try {
          resp->answer = this->client_.sendAndReceive("get loaded program\n");
          std::smatch match;
          std::regex expected("Loaded program: (.+)");
          resp->success = std::regex_match(resp->answer, match, expected);
          if (resp->success) {
            resp->program_name = match[1];
          }
        } catch (const urcl::UrException& e) {
          RCLCPP_ERROR(rclcpp::get_logger("Dashboard_Client"), "Service Call failed: '%s'", e.what());
          resp->answer = e.what();
          resp->success = false;
        }
        return true;
      });

  // Load a robot installation from a file
  load_installation_service_ = node_->create_service<ur_dashboard_msgs::srv::Load>(
      "~/load_installation", [&](const ur_dashboard_msgs::srv::Load::Request::SharedPtr req,
                                 ur_dashboard_msgs::srv::Load::Response::SharedPtr resp) {
        try {
          resp->answer = this->client_.sendAndReceive("load installation " + req->filename + "\n");
          resp->success = std::regex_match(resp->answer, std::regex("Loading installation: .+"));
        } catch (const urcl::UrException& e) {
          RCLCPP_ERROR(rclcpp::get_logger("Dashboard_Client"), "Service Call failed: '%s'", e.what());
          resp->answer = e.what();
          resp->success = false;
        }
        return true;
      });

  // Load a robot program from a file
  load_program_service_ = node->create_service<ur_dashboard_msgs::srv::Load>(
      "~/load_program", [&](const ur_dashboard_msgs::srv::Load::Request::SharedPtr req,
                            ur_dashboard_msgs::srv::Load::Response::SharedPtr resp) {
        try {
          resp->answer = this->client_.sendAndReceive("load " + req->filename + "\n");
          resp->success = std::regex_match(resp->answer, std::regex("Loading program: .+"));
        } catch (const urcl::UrException& e) {
          RCLCPP_ERROR(rclcpp::get_logger("Dashboard_Client"), "Service Call failed: '%s'", e.what());
          resp->answer = e.what();
          resp->success = false;
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
        try {
          resp->answer = this->client_.sendAndReceive("popup " + req->message + "\n");
          resp->success = std::regex_match(resp->answer, std::regex("showing popup"));
        } catch (const urcl::UrException& e) {
          RCLCPP_ERROR(rclcpp::get_logger("Dashboard_Client"), "Service Call failed: '%s'", e.what());
          resp->answer = e.what();
          resp->success = false;
        }
        return true;
      });

  // Service to query the current program state
  program_state_service_ = node_->create_service<ur_dashboard_msgs::srv::GetProgramState>(
      "~/program_state", [&](const ur_dashboard_msgs::srv::GetProgramState::Request::SharedPtr /*unused*/,
                             ur_dashboard_msgs::srv::GetProgramState::Response::SharedPtr resp) {
        try {
          resp->answer = this->client_.sendAndReceive("programState\n");
          std::smatch match;
          std::regex expected("(STOPPED|PLAYING|PAUSED) (.+)");
          resp->success = std::regex_match(resp->answer, match, expected);
          if (resp->success) {
            resp->state.state = match[1];
            resp->program_name = match[2];
          }
        } catch (const urcl::UrException& e) {
          RCLCPP_ERROR(rclcpp::get_logger("Dashboard_Client"), "Service Call failed: '%s'", e.what());
          resp->answer = e.what();
          resp->success = false;
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
        try {
          resp->answer = this->client_.sendAndReceive("addToLog " + req->message + "\n");
          resp->success = std::regex_match(resp->answer, std::regex("(Added log message|No log message to add)"));
        } catch (const urcl::UrException& e) {
          RCLCPP_ERROR(rclcpp::get_logger("Dashboard_Client"), "Service Call failed: '%s'", e.what());
          resp->answer = e.what();
          resp->success = false;
        }
        return true;
      });

  // General purpose service to send arbitrary messages to the dashboard server
  raw_request_service_ = node_->create_service<ur_dashboard_msgs::srv::RawRequest>(
      "~/raw_request", [&](const ur_dashboard_msgs::srv::RawRequest::Request::SharedPtr req,
                           ur_dashboard_msgs::srv::RawRequest::Response::SharedPtr resp) {
        try {
          resp->answer = this->client_.sendAndReceive(req->query + "\n");
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
      node_->create_service<std_srvs::srv::Trigger>("~/quit", [&](const std_srvs::srv::Trigger::Request::SharedPtr req,
                                                                  std_srvs::srv::Trigger::Response::SharedPtr resp) {
        try {
          resp->message = this->client_.sendAndReceive("quit\n");
          resp->success = std::regex_match(resp->message, std::regex("Disconnected"));
          client_.disconnect();
        } catch (const urcl::UrException& e) {
          RCLCPP_ERROR(rclcpp::get_logger("Dashboard_Client"), "Service Call failed: '%s'", e.what());
          resp->message = e.what();
          resp->success = false;
        }
        return true;
      });
}

bool DashboardClientROS::connect()
{
  timeval tv;
  // Timeout after which a call to the dashboard server will be considered failure if no answer has been received.
  double time_buffer = 0;
  node_->get_parameter("receive_timeout", time_buffer);
  tv.tv_sec = time_buffer;
  tv.tv_usec = 0;
  client_.setReceiveTimeout(tv);
  return client_.connect();
}

bool DashboardClientROS::handleRunningQuery(const ur_dashboard_msgs::srv::IsProgramRunning::Request::SharedPtr req,
                                            ur_dashboard_msgs::srv::IsProgramRunning::Response::SharedPtr resp)
{
  try {
    resp->answer = this->client_.sendAndReceive("running\n");
    std::regex expected("Program running: (true|false)");
    std::smatch match;
    resp->success = std::regex_match(resp->answer, match, expected);

    if (resp->success) {
      resp->program_running = (match[1] == "true");
    }
  } catch (const urcl::UrException& e) {
    RCLCPP_ERROR(rclcpp::get_logger("Dashboard_Client"), "Service Call failed: '%s'", e.what());
    resp->answer = e.what();
    resp->success = false;
  }

  return true;
}

bool DashboardClientROS::handleSavedQuery(ur_dashboard_msgs::srv::IsProgramSaved::Request::SharedPtr req,
                                          ur_dashboard_msgs::srv::IsProgramSaved::Response::SharedPtr resp)
{
  try {
    resp->answer = this->client_.sendAndReceive("isProgramSaved\n");
    std::regex expected("(true|false) ([^\\s]+)");
    std::smatch match;
    resp->success = std::regex_match(resp->answer, match, expected);

    if (resp->success) {
      resp->program_saved = (match[1] == "true");
      resp->program_name = match[2];
    }
  } catch (const urcl::UrException& e) {
    RCLCPP_ERROR(rclcpp::get_logger("Dashboard_Client"), "Service Call failed: '%s'", e.what());
    resp->answer = e.what();
    resp->success = false;
  }

  return true;
}

bool DashboardClientROS::handleSafetyModeQuery(const ur_dashboard_msgs::srv::GetSafetyMode::Request::SharedPtr req,
                                               ur_dashboard_msgs::srv::GetSafetyMode::Response::SharedPtr resp)
{
  try {
    resp->answer = this->client_.sendAndReceive("safetymode\n");
    std::smatch match;
    std::regex expected("Safetymode: (.+)");
    resp->success = std::regex_match(resp->answer, match, expected);
    if (resp->success) {
      if (match[1] == "NORMAL") {
        resp->safety_mode.mode = ur_dashboard_msgs::msg::SafetyMode::NORMAL;
      } else if (match[1] == "REDUCED") {
        resp->safety_mode.mode = ur_dashboard_msgs::msg::SafetyMode::REDUCED;
      } else if (match[1] == "PROTECTIVE_STOP") {
        resp->safety_mode.mode = ur_dashboard_msgs::msg::SafetyMode::PROTECTIVE_STOP;
      } else if (match[1] == "RECOVERY") {
        resp->safety_mode.mode = ur_dashboard_msgs::msg::SafetyMode::RECOVERY;
      } else if (match[1] == "SAFEGUARD_STOP") {
        resp->safety_mode.mode = ur_dashboard_msgs::msg::SafetyMode::SAFEGUARD_STOP;
      } else if (match[1] == "SYSTEM_EMERGENCY_STOP") {
        resp->safety_mode.mode = ur_dashboard_msgs::msg::SafetyMode::SYSTEM_EMERGENCY_STOP;
      } else if (match[1] == "ROBOT_EMERGENCY_STOP") {
        resp->safety_mode.mode = ur_dashboard_msgs::msg::SafetyMode::ROBOT_EMERGENCY_STOP;
      } else if (match[1] == "VIOLATION") {
        resp->safety_mode.mode = ur_dashboard_msgs::msg::SafetyMode::VIOLATION;
      } else if (match[1] == "FAULT") {
        resp->safety_mode.mode = ur_dashboard_msgs::msg::SafetyMode::FAULT;
      }
      // The following are only available in SafetyStatus from 5.5 on
      // else if (match[1] == "AUTOMATIC_MODE_SAFEGUARD_STOP")
      //{
      // resp->safety_mode.mode = ur_dashboard_msgs::msg::SafetyMode::AUTOMATIC_MODE_SAFEGUARD_STOP;
      //}
      // else if (match[1] == "SYSTEM_THREE_POSITION_ENABLING_STOP")
      //{
      // resp->safety_mode.mode = ur_dashboard_msgs::msg::SafetyMode::SYSTEM_THREE_POSITION_ENABLING_STOP;
      //}
    }
  } catch (const urcl::UrException& e) {
    RCLCPP_ERROR(rclcpp::get_logger("Dashboard_Client"), "Service Call failed: '%s'", e.what());
    resp->answer = e.what();
    resp->success = false;
  }
  return true;
}

bool DashboardClientROS::handleRobotModeQuery(const ur_dashboard_msgs::srv::GetRobotMode::Request::SharedPtr req,
                                              ur_dashboard_msgs::srv::GetRobotMode::Response::SharedPtr resp)
{
  try {
    resp->answer = this->client_.sendAndReceive("robotmode\n");
    std::smatch match;
    std::regex expected("Robotmode: (.+)");
    resp->success = std::regex_match(resp->answer, match, expected);
    if (resp->success) {
      if (match[1] == "NO_CONTROLLER") {
        resp->robot_mode.mode = ur_dashboard_msgs::msg::RobotMode::NO_CONTROLLER;
      } else if (match[1] == "DISCONNECTED") {
        resp->robot_mode.mode = ur_dashboard_msgs::msg::RobotMode::DISCONNECTED;
      } else if (match[1] == "CONFIRM_SAFETY") {
        resp->robot_mode.mode = ur_dashboard_msgs::msg::RobotMode::CONFIRM_SAFETY;
      } else if (match[1] == "BOOTING") {
        resp->robot_mode.mode = ur_dashboard_msgs::msg::RobotMode::BOOTING;
      } else if (match[1] == "POWER_OFF") {
        resp->robot_mode.mode = ur_dashboard_msgs::msg::RobotMode::POWER_OFF;
      } else if (match[1] == "POWER_ON") {
        resp->robot_mode.mode = ur_dashboard_msgs::msg::RobotMode::POWER_ON;
      } else if (match[1] == "IDLE") {
        resp->robot_mode.mode = ur_dashboard_msgs::msg::RobotMode::IDLE;
      } else if (match[1] == "BACKDRIVE") {
        resp->robot_mode.mode = ur_dashboard_msgs::msg::RobotMode::BACKDRIVE;
      } else if (match[1] == "RUNNING") {
        resp->robot_mode.mode = ur_dashboard_msgs::msg::RobotMode::RUNNING;
      } else if (match[1] == "UPDATING_FIRMWARE") {
        resp->robot_mode.mode = ur_dashboard_msgs::msg::RobotMode::UPDATING_FIRMWARE;
      }
    }
  } catch (const urcl::UrException& e) {
    RCLCPP_ERROR(rclcpp::get_logger("Dashboard_Client"), "Service Call failed: '%s'", e.what());
    resp->answer = e.what();
    resp->success = false;
  }
  return true;
}
}  // namespace ur_robot_driver
