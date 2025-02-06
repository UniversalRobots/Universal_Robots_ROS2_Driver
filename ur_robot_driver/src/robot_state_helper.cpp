// Copyright 2024, FZI Forschungszentrum Informatik, Created on behalf of Universal Robots A/S
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

#include <ur_robot_driver/robot_state_helper.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/create_server.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "ur_dashboard_msgs/action/set_mode.hpp"
#include "ur_dashboard_msgs/msg/safety_mode.hpp"
#include "ur_dashboard_msgs/msg/robot_mode.hpp"
#include "ur_client_library/ur/datatypes.h"

namespace ur_robot_driver
{
RobotStateHelper::RobotStateHelper(const rclcpp::Node::SharedPtr& node)
  : node_(node)
  , is_started_(false)
  , in_action_(false)
  , robot_mode_(urcl::RobotMode::UNKNOWN)
  , safety_mode_(urcl::SafetyMode::UNDEFINED_SAFETY_MODE)
{
  robot_mode_sub_cb_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  rclcpp::SubscriptionOptions options;
  options.callback_group = robot_mode_sub_cb_;
  // Topic on which the robot_mode is published by the driver
  robot_mode_sub_ = node_->create_subscription<ur_dashboard_msgs::msg::RobotMode>(
      "io_and_status_controller/robot_mode", rclcpp::SensorDataQoS(),
      std::bind(&RobotStateHelper::robotModeCallback, this, std::placeholders::_1), options);
  // Topic on which the safety is published by the driver
  safety_mode_sub_ = node_->create_subscription<ur_dashboard_msgs::msg::SafetyMode>(
      "io_and_status_controller/safety_mode", 1,
      std::bind(&RobotStateHelper::safetyModeCallback, this, std::placeholders::_1));

  unlock_cb_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  restart_cb_ = unlock_cb_;
  power_on_cb_ = unlock_cb_;
  power_off_cb_ = unlock_cb_;
  brake_release_cb_ = unlock_cb_;
  stop_program_cb_ = unlock_cb_;
  play_program_cb_ = unlock_cb_;

  // Service to unlock protective stop
  unlock_protective_stop_srv_ = node_->create_client<std_srvs::srv::Trigger>(
      "dashboard_client/unlock_protective_stop", rclcpp::QoS(rclcpp::KeepLast(10)), unlock_cb_);
  // Service to restart safety
  restart_safety_srv_ = node_->create_client<std_srvs::srv::Trigger>("dashboard_client/restart_safety",
                                                                     rclcpp::QoS(rclcpp::KeepLast(10)), restart_cb_);
  // Service to power on the robot
  power_on_srv_ = node_->create_client<std_srvs::srv::Trigger>("dashboard_client/power_on",
                                                               rclcpp::QoS(rclcpp::KeepLast(10)), power_on_cb_);
  // Service to power off the robot
  power_off_srv_ = node_->create_client<std_srvs::srv::Trigger>("dashboard_client/power_off",
                                                                rclcpp::QoS(rclcpp::KeepLast(10)), power_off_cb_);
  // Service to release the robot's brakes
  brake_release_srv_ = node_->create_client<std_srvs::srv::Trigger>(
      "dashboard_client/brake_release", rclcpp::QoS(rclcpp::KeepLast(10)), brake_release_cb_);
  // Service to stop UR program execution on the robot
  stop_program_srv_ = node_->create_client<std_srvs::srv::Trigger>("dashboard_client/stop",
                                                                   rclcpp::QoS(rclcpp::KeepLast(10)), stop_program_cb_);
  // Service to start UR program execution on the robot
  play_program_srv_ = node_->create_client<std_srvs::srv::Trigger>("dashboard_client/play",
                                                                   rclcpp::QoS(rclcpp::KeepLast(10)), play_program_cb_);
  play_program_srv_->wait_for_service();

  feedback_ = std::make_shared<ur_dashboard_msgs::action::SetMode::Feedback>();
  result_ = std::make_shared<ur_dashboard_msgs::action::SetMode::Result>();
}

void RobotStateHelper::robotModeCallback(ur_dashboard_msgs::msg::RobotMode::SharedPtr msg)
{
  if (robot_mode_ != static_cast<urcl::RobotMode>(msg->mode)) {
    robot_mode_ = urcl::RobotMode(msg->mode);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("robot_state_helper"),
                       "The robot is currently in mode " << robotModeString(robot_mode_) << ".");
    if (in_action_) {
      updateRobotState();
      if (!is_started_) {
        startActionServer();
      }
    }
  }
}

void RobotStateHelper::safetyModeCallback(ur_dashboard_msgs::msg::SafetyMode::SharedPtr msg)
{
  if (safety_mode_ != static_cast<urcl::SafetyMode>(msg->mode)) {
    safety_mode_ = urcl::SafetyMode(msg->mode);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("robot_state_helper"),
                       "The robot is currently in safety mode " << safetyModeString(safety_mode_) << ".");
    updateRobotState();
    if (!is_started_) {
      startActionServer();
    }
  }
}

void RobotStateHelper::doTransition()
{
  if (static_cast<urcl::RobotMode>(goal_->target_robot_mode) < robot_mode_) {
    safeDashboardTrigger(this->power_off_srv_);
  } else {
    switch (safety_mode_) {
      case urcl::SafetyMode::PROTECTIVE_STOP:
        safeDashboardTrigger(this->unlock_protective_stop_srv_);
        break;
      case urcl::SafetyMode::SYSTEM_EMERGENCY_STOP:;
      case urcl::SafetyMode::ROBOT_EMERGENCY_STOP:
        RCLCPP_WARN_STREAM(rclcpp::get_logger("robot_state_helper"), "The robot is currently in safety mode."
                                                                         << safetyModeString(safety_mode_)
                                                                         << ". Please release the EM-Stop to proceed.");
        break;
      case urcl::SafetyMode::VIOLATION:;
      case urcl::SafetyMode::FAULT:
        safeDashboardTrigger(this->restart_safety_srv_);
        break;
      default:
        switch (robot_mode_) {
          case urcl::RobotMode::CONFIRM_SAFETY:
            RCLCPP_WARN_STREAM(rclcpp::get_logger("robot_state_helper"), "The robot is currently in mode "
                                                                             << robotModeString(robot_mode_)
                                                                             << ". It is required to interact with the "
                                                                                "teach pendant at this point.");
            break;
          case urcl::RobotMode::BOOTING:
            RCLCPP_INFO_STREAM(rclcpp::get_logger("robot_state_helper"), "The robot is currently in mode "
                                                                             << robotModeString(robot_mode_)
                                                                             << ". Please wait until the robot is "
                                                                                "booted up...");
            break;
          case urcl::RobotMode::POWER_OFF:
            safeDashboardTrigger(this->power_on_srv_);
            break;
          case urcl::RobotMode::POWER_ON:
            RCLCPP_INFO_STREAM(rclcpp::get_logger("robot_state_helper"), "The robot is currently in mode "
                                                                             << robotModeString(robot_mode_)
                                                                             << ". Please wait until the robot is in "
                                                                                "mode "
                                                                             << robotModeString(urcl::RobotMode::IDLE));
            break;
          case urcl::RobotMode::IDLE:
            safeDashboardTrigger(this->brake_release_srv_);
            break;
          case urcl::RobotMode::BACKDRIVE:
            RCLCPP_INFO_STREAM(rclcpp::get_logger("robot_state_helper"),
                               "The robot is currently in mode "
                                   << robotModeString(robot_mode_) << ". It will automatically return to mode "
                                   << robotModeString(urcl::RobotMode::IDLE) << " once the teach button is released.");
            break;
          case urcl::RobotMode::RUNNING:
            RCLCPP_INFO_STREAM(rclcpp::get_logger("robot_state_helper"),
                               "The robot has reached operational mode " << robotModeString(robot_mode_));
            break;
          default:
            RCLCPP_WARN_STREAM(rclcpp::get_logger("robot_state_helper"), "The robot is currently in mode "
                                                                             << robotModeString(robot_mode_)
                                                                             << ". This won't be handled by this "
                                                                                "helper. Please resolve this "
                                                                                "manually.");
        }
    }
  }
}

bool RobotStateHelper::safeDashboardTrigger(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr srv)
{
  assert(srv != nullptr);
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = srv->async_send_request(request);
  future.wait();
  RCLCPP_INFO_STREAM(rclcpp::get_logger("robot_state_helper"), "Service response received: " << future.get()->message);
  return true;
}

void RobotStateHelper::updateRobotState()
{
  if (is_started_) {
    // Update action feedback
    feedback_->current_robot_mode =
        static_cast<ur_dashboard_msgs::action::SetMode::Feedback::_current_robot_mode_type>(robot_mode_);
    feedback_->current_safety_mode =
        static_cast<ur_dashboard_msgs::action::SetMode::Feedback::_current_safety_mode_type>(safety_mode_);
    current_goal_handle_->publish_feedback(feedback_);

    if (robot_mode_ < static_cast<urcl::RobotMode>(goal_->target_robot_mode) ||
        safety_mode_ > urcl::SafetyMode::REDUCED) {
      // Transition to next mode
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("ur_robot_state_helper"),
                          "Current robot mode is "
                              << robotModeString(robot_mode_) << " while target mode is "
                              << robotModeString(static_cast<urcl::RobotMode>(goal_->target_robot_mode)));
      doTransition();
    } else if (robot_mode_ == static_cast<urcl::RobotMode>(goal_->target_robot_mode)) {
      // Final mode reached
      in_action_ = false;
      result_->success = true;
      result_->message = "Reached target robot mode.";
      if (robot_mode_ == urcl::RobotMode::RUNNING && goal_->play_program) {
        // The dashboard denies playing immediately after switching the mode to RUNNING
        sleep(1);
        safeDashboardTrigger(this->play_program_srv_);
      }
      current_goal_handle_->succeed(result_);
    } else {
      result_->success = false;
      result_->message = "Robot reached higher mode than requested during recovery. This either means that something "
                         "went wrong or that a higher mode was requested from somewhere else (e.g. the teach "
                         "pendant.)";
      current_goal_handle_->abort(result_);
    }
  }
}

void RobotStateHelper::startActionServer()
{
  if (robot_mode_ != urcl::RobotMode::UNKNOWN && safety_mode_ != urcl::SafetyMode::UNDEFINED_SAFETY_MODE) {
    is_started_ = true;
  }
  set_mode_as_ = rclcpp_action::create_server<ur_dashboard_msgs::action::SetMode>(
      node_, "~/set_mode",
      std::bind(&RobotStateHelper::setModeGoalCallback, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&RobotStateHelper::setModeCancelCallback, this, std::placeholders::_1),
      std::bind(&RobotStateHelper::setModeAcceptCallback, this, std::placeholders::_1));
}

void RobotStateHelper::setModeAcceptCallback(const std::shared_ptr<RobotStateHelper::SetModeGoalHandle> goal_handle)
{
  std::thread{ std::bind(&RobotStateHelper::setModeExecute, this, std::placeholders::_1), goal_handle }.detach();
}

void RobotStateHelper::setModeExecute(const std::shared_ptr<RobotStateHelper::SetModeGoalHandle> goal_handle)
{
  in_action_ = true;
  current_goal_handle_ = goal_handle;
  const auto goal = current_goal_handle_->get_goal();
  this->goal_ = goal;
  if (goal_->target_robot_mode > 8 || goal_->target_robot_mode < -1) {
    result_->message = "Requested illegal mode.";
    result_->success = false;
    current_goal_handle_->abort(result_);
  } else {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("robot_state_helper"),
                       "Target mode was set to "
                           << robotModeString(static_cast<urcl::RobotMode>(goal_->target_robot_mode)) << ".");
    switch (static_cast<urcl::RobotMode>(goal_->target_robot_mode)) {
      case urcl::RobotMode::POWER_OFF:
      case urcl::RobotMode::IDLE:
      case urcl::RobotMode::RUNNING:
        if (robot_mode_ != static_cast<urcl::RobotMode>(goal_->target_robot_mode) ||
            safety_mode_ > urcl::SafetyMode::REDUCED) {
          if (goal_->stop_program) {
            safeDashboardTrigger(this->stop_program_srv_);
          }
          doTransition();
        } else {
          updateRobotState();
        }
        break;
      case urcl::RobotMode::NO_CONTROLLER:
      case urcl::RobotMode::DISCONNECTED:
      case urcl::RobotMode::CONFIRM_SAFETY:
      case urcl::RobotMode::BOOTING:
      case urcl::RobotMode::POWER_ON:
      case urcl::RobotMode::BACKDRIVE:
      case urcl::RobotMode::UPDATING_FIRMWARE:
        result_->message = "Requested target mode " +
                           robotModeString(static_cast<urcl::RobotMode>(goal_->target_robot_mode)) +
                           " which cannot be explicitly selected.";
        result_->success = false;
        current_goal_handle_->abort(result_);
        break;
      default:
        result_->message = "Requested illegal mode.";
        result_->success = false;
        current_goal_handle_->abort(result_);
        break;
    }
  }
}

rclcpp_action::GoalResponse RobotStateHelper::setModeGoalCallback(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const ur_dashboard_msgs::action::SetMode::Goal> goal)
{
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse
RobotStateHelper::setModeCancelCallback(const std::shared_ptr<RobotStateHelper::SetModeGoalHandle> goal_handle)
{
  RCLCPP_INFO(rclcpp::get_logger("robot_state_helper"), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::REJECT;
}

}  // namespace ur_robot_driver
