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

#include <mutex>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <ur_robot_driver/robot_state_helper.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/create_server.hpp"
#include "ur_client_library/ur/datatypes.h"

namespace ur_robot_driver
{
RobotStateHelper::RobotStateHelper(const rclcpp::Node::SharedPtr& node)
  : node_(node)
  , robot_mode_(urcl::RobotMode::UNKNOWN)
  , safety_mode_(urcl::SafetyMode::UNDEFINED_SAFETY_MODE)
  , in_action_(false)
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
  program_running_sub = node_->create_subscription<std_msgs::msg::Bool>(
      "io_and_status_controller/robot_program_running", 1,
      [this](std_msgs::msg::Bool::UniquePtr msg) -> void { program_running_ = msg->data; });

  service_cb_grp_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  node->declare_parameter("headless_mode", false);
  headless_mode_ = node->get_parameter("headless_mode").as_bool();

  node->declare_parameter("robot_ip", "192.168.56.101");
  robot_ip_ = node->get_parameter("robot_ip").as_string();

  primary_client_ = std::make_shared<urcl::primary_interface::PrimaryClient>(robot_ip_, notifier_);

  primary_client_->start(0, std::chrono::seconds(10));
  auto robot_version = primary_client_->getRobotVersion();

  if (robot_version->major > 5) {
    RCLCPP_WARN(rclcpp::get_logger("robot_state_helper"), "Running on a PolyScopeX robot. The dashboard server is not "
                                                          "available, therefore the robot_state_helper cannot start "
                                                          "PolyScope programs and restart the safety.");
  } else {
    // Service to restart safety
    restart_safety_srv_ = node_->create_client<std_srvs::srv::Trigger>(
        "dashboard_client/restart_safety", rmw_qos_profile_services_default, service_cb_grp_);
    // Service to start UR program execution on the robot
    play_program_srv_ = node_->create_client<std_srvs::srv::Trigger>("dashboard_client/play",
                                                                     rmw_qos_profile_services_default, service_cb_grp_);
    play_program_srv_->wait_for_service();
  }

  resend_robot_program_srv_ = node_->create_client<std_srvs::srv::Trigger>(
      "io_and_status_controller/resend_robot_program", rmw_qos_profile_services_default, service_cb_grp_);
  resend_robot_program_srv_->wait_for_service();

  feedback_ = std::make_shared<ur_dashboard_msgs::action::SetMode::Feedback>();
  result_ = std::make_shared<ur_dashboard_msgs::action::SetMode::Result>();
  set_mode_as_ = rclcpp_action::create_server<ur_dashboard_msgs::action::SetMode>(
      node_, "~/set_mode",
      std::bind(&RobotStateHelper::setModeGoalCallback, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&RobotStateHelper::setModeCancelCallback, this, std::placeholders::_1),
      std::bind(&RobotStateHelper::setModeAcceptCallback, this, std::placeholders::_1));
}

void RobotStateHelper::robotModeCallback(ur_dashboard_msgs::msg::RobotMode::SharedPtr msg)
{
  if (robot_mode_ != static_cast<urcl::RobotMode>(msg->mode)) {
    robot_mode_ = urcl::RobotMode(msg->mode);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("robot_state_helper"),
                       "The robot is currently in mode " << robotModeString(robot_mode_) << ".");
    if (in_action_) {
      std::scoped_lock lock(goal_mutex_);
      feedback_->current_robot_mode =
          static_cast<ur_dashboard_msgs::action::SetMode::Feedback::_current_robot_mode_type>(robot_mode_.load());
      current_goal_handle_->publish_feedback(feedback_);
    }
  }
}

void RobotStateHelper::safetyModeCallback(ur_dashboard_msgs::msg::SafetyMode::SharedPtr msg)
{
  if (safety_mode_ != static_cast<urcl::SafetyMode>(msg->mode)) {
    safety_mode_ = urcl::SafetyMode(msg->mode);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("robot_state_helper"),
                       "The robot is currently in safety mode " << safetyModeString(safety_mode_) << ".");
    if (in_action_) {
      std::scoped_lock lock(goal_mutex_);
      feedback_->current_safety_mode =
          static_cast<ur_dashboard_msgs::action::SetMode::Feedback::_current_safety_mode_type>(safety_mode_.load());
      current_goal_handle_->publish_feedback(feedback_);
    }
  }
}

bool RobotStateHelper::recoverFromSafety()
{
  switch (safety_mode_) {
    case urcl::SafetyMode::PROTECTIVE_STOP:
      try {
        primary_client_->commandUnlockProtectiveStop();
      } catch (const urcl::UrException& e) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("robot_state_helper"), e.what());
        return false;
      }
      return true;
    case urcl::SafetyMode::SYSTEM_EMERGENCY_STOP:;
    case urcl::SafetyMode::ROBOT_EMERGENCY_STOP:
      RCLCPP_WARN_STREAM(rclcpp::get_logger("robot_state_helper"), "The robot is currently in safety mode."
                                                                       << safetyModeString(safety_mode_)
                                                                       << ". Please release the EM-Stop to proceed.");
      return false;
    case urcl::SafetyMode::VIOLATION:;
    case urcl::SafetyMode::FAULT:
      if (restart_safety_srv_ != nullptr) {
        return safeDashboardTrigger(this->restart_safety_srv_);
      } else {
        return false;
      }
    default:
      // nothing to do
      RCLCPP_DEBUG_STREAM(rclcpp::get_logger("robot_state_helper"), "No safety recovery needed.");
  }
  return true;
}

bool RobotStateHelper::jumpToRobotMode(const urcl::RobotMode target_mode)
{
  try {
    switch (target_mode) {
      case urcl::RobotMode::POWER_OFF:
        primary_client_->commandPowerOff();
        return true;
      case urcl::RobotMode::IDLE:
        primary_client_->commandPowerOn();
        return true;
      case urcl::RobotMode::RUNNING:
        primary_client_->commandBrakeRelease();
        return true;
      default:
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("robot_state_helper"), "Unreachable target robot mode.");
    }
  } catch (const urcl::UrException& e) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("robot_state_helper"), e.what());
  }
  return false;
}

bool RobotStateHelper::doTransition(const urcl::RobotMode target_mode)
{
  if (!recoverFromSafety()) {
    return false;
  }
  switch (robot_mode_) {
    case urcl::RobotMode::CONFIRM_SAFETY:
      RCLCPP_WARN_STREAM(rclcpp::get_logger("robot_state_helper"), "The robot is currently in mode "
                                                                       << robotModeString(robot_mode_)
                                                                       << ". It is required to interact with "
                                                                          "the "
                                                                          "teach pendant at this point.");
      break;
    case urcl::RobotMode::BOOTING:
      RCLCPP_INFO_STREAM(rclcpp::get_logger("robot_state_helper"), "The robot is currently in mode "
                                                                       << robotModeString(robot_mode_)
                                                                       << ". Please wait until the robot is "
                                                                          "booted up...");
      break;
    case urcl::RobotMode::POWER_OFF:
      return jumpToRobotMode(target_mode);
    case urcl::RobotMode::POWER_ON:
      RCLCPP_INFO_STREAM(rclcpp::get_logger("robot_state_helper"), "The robot is currently in mode "
                                                                       << robotModeString(robot_mode_)
                                                                       << ". Please wait until the robot is in "
                                                                          "mode "
                                                                       << robotModeString(urcl::RobotMode::IDLE));
      break;
    case urcl::RobotMode::IDLE:
      return jumpToRobotMode(target_mode);
      break;
    case urcl::RobotMode::BACKDRIVE:
      RCLCPP_INFO_STREAM(rclcpp::get_logger("robot_state_helper"), "The robot is currently in mode "
                                                                       << robotModeString(robot_mode_)
                                                                       << ". It will automatically return to "
                                                                          "mode "
                                                                       << robotModeString(urcl::RobotMode::IDLE)
                                                                       << " once the teach button is "
                                                                          "released.");
      break;
    case urcl::RobotMode::RUNNING:
      if (target_mode == urcl::RobotMode::IDLE) {
        // We cannot engage the brakes directly.
        if (!jumpToRobotMode(urcl::RobotMode::POWER_OFF)) {
          return false;
        }
      }
      return jumpToRobotMode(target_mode);
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
  return false;
}

bool RobotStateHelper::safeDashboardTrigger(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr srv)
{
  assert(srv != nullptr);
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = srv->async_send_request(request);
  future.wait();
  auto result = future.get();
  RCLCPP_INFO_STREAM(rclcpp::get_logger("robot_state_helper"), "Service response received: " << result->message);
  return result->success;
}

void RobotStateHelper::setModeAcceptCallback(const std::shared_ptr<RobotStateHelper::SetModeGoalHandle> goal_handle)
{
  std::thread{ std::bind(&RobotStateHelper::setModeExecute, this, std::placeholders::_1), goal_handle }.detach();
}

bool RobotStateHelper::stopProgram()
{
  try {
    primary_client_->commandStop();
  } catch (const urcl::UrException& e) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("robot_state_helper"), e.what());
    return false;
  }
  return true;
}

void RobotStateHelper::setModeExecute(const std::shared_ptr<RobotStateHelper::SetModeGoalHandle> goal_handle)
{
  {
    std::scoped_lock lock(goal_mutex_);
    current_goal_handle_ = goal_handle;
  }
  in_action_ = true;
  const auto goal = goal_handle->get_goal();
  this->goal_ = goal;
  urcl::RobotMode target_mode;
  try {
    target_mode = static_cast<urcl::RobotMode>(goal->target_robot_mode);
    switch (target_mode) {
      case urcl::RobotMode::POWER_OFF:
      case urcl::RobotMode::IDLE:
      case urcl::RobotMode::RUNNING:
        if (goal_->stop_program && program_running_) {
          if (!stopProgram()) {
            result_->message = "Stopping the program failed.";
            result_->success = false;
            std::scoped_lock lock(goal_mutex_);
            current_goal_handle_->abort(result_);
            return;
          }
        }
        if (robot_mode_ != target_mode || safety_mode_ > urcl::SafetyMode::REDUCED) {
          RCLCPP_INFO_STREAM(rclcpp::get_logger("robot_state_helper"),
                             "Target mode was set to " << robotModeString(target_mode) << ".");
          if (!doTransition(target_mode)) {
            result_->message = "Transition to target mode failed.";
            result_->success = false;
            std::scoped_lock lock(goal_mutex_);
            current_goal_handle_->abort(result_);
            return;
          }
        }
        break;
      case urcl::RobotMode::NO_CONTROLLER:
      case urcl::RobotMode::DISCONNECTED:
      case urcl::RobotMode::CONFIRM_SAFETY:
      case urcl::RobotMode::BOOTING:
      case urcl::RobotMode::POWER_ON:
      case urcl::RobotMode::BACKDRIVE:
      case urcl::RobotMode::UPDATING_FIRMWARE:
        result_->message =
            "Requested target mode " + robotModeString(target_mode) + " which cannot be explicitly selected.";
        result_->success = false;
        {
          std::scoped_lock lock(goal_mutex_);
          current_goal_handle_->abort(result_);
        }
        return;
        break;
      default:
        result_->message = "Requested illegal mode.";
        result_->success = false;
        {
          std::scoped_lock lock(goal_mutex_);
          current_goal_handle_->abort(result_);
        }
        return;
        break;
    }
  } catch (const std::invalid_argument& e) {
    result_->message = e.what();
    result_->success = false;
    {
      std::scoped_lock lock(goal_mutex_);
      current_goal_handle_->abort(result_);
    }
    return;
  }

  // Wait until the robot reached the target mode or something went wrong
  while (robot_mode_ != target_mode && !error_) {
    RCLCPP_INFO(rclcpp::get_logger("robot_state_helper"), "Waiting for robot to reach target mode... Current_mode: %s",
                robotModeString(robot_mode_).c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }

  if (robot_mode_ == target_mode) {
    result_->success = true;
    result_->message = "Reached target robot mode.";
    if (robot_mode_ == urcl::RobotMode::RUNNING && goal_->play_program && !program_running_) {
      if (headless_mode_) {
        result_->success = safeDashboardTrigger(this->resend_robot_program_srv_);
      } else {
        if (play_program_srv_ == nullptr) {
          result_->success = false;
          result_->message = "Play program service not available on this robot.";
        } else {
          // The dashboard denies playing immediately after switching the mode to RUNNING
          std::this_thread::sleep_for(std::chrono::milliseconds(1000));
          result_->success = safeDashboardTrigger(this->play_program_srv_);
        }
      }
    }
    if (result_->success) {
      std::scoped_lock lock(goal_mutex_);
      current_goal_handle_->succeed(result_);
    } else {
      std::scoped_lock lock(goal_mutex_);
      current_goal_handle_->abort(result_);
    }
  } else {
    result_->success = false;
    result_->message = "Robot reached higher mode than requested during recovery. This either means that something "
                       "went wrong or that a higher mode was requested from somewhere else (e.g. the teach "
                       "pendant.)";
    {
      std::scoped_lock lock(goal_mutex_);
      current_goal_handle_->abort(result_);
    }
  }
}

rclcpp_action::GoalResponse RobotStateHelper::setModeGoalCallback(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const ur_dashboard_msgs::action::SetMode::Goal> goal)
{
  (void)uuid;
  if (robot_mode_ == urcl::RobotMode::UNKNOWN) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("robot_state_helper"), "Robot mode is unknown. Cannot accept goal, yet. Is "
                                                                  "the robot switched on and connected to the driver?");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (safety_mode_ == urcl::SafetyMode::UNDEFINED_SAFETY_MODE) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("robot_state_helper"), "Safety mode is unknown. Cannot accept goal, yet. Is "
                                                                  "the robot switched on and connected to the driver?");
    return rclcpp_action::GoalResponse::REJECT;
  }
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
