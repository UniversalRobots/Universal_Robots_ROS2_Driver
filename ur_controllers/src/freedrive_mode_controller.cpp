
// Copyright 2023, FZI Forschungszentrum Informatik, Created on behalf of Universal Robots A/S
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
 * \author  Vincenzo Di Pentima dipentima@fzi.de
 * \date    2024-09-16
 */
//----------------------------------------------------------------------
#include <controller_interface/controller_interface.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <rclcpp/logging.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <ur_controllers/freedrive_mode_controller.hpp>

namespace ur_controllers
{
controller_interface::CallbackReturn FreedriveModeController::on_init()
{
  // Even if the only param I use is tf_prefix, I still need it
  try {
    // Create the parameter listener and get the parameters
    freedrive_param_listener_ = std::make_shared<freedrive_mode_controller::ParamListener>(get_node());
    freedrive_params_ = freedrive_param_listener_->get_params();
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration FreedriveModeController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  const std::string tf_prefix = freedrive_params_.tf_prefix;

  // Get the command interfaces needed for freedrive mode from the hardware interface
  config.names.emplace_back(tf_prefix + "freedrive_mode/async_success");
  config.names.emplace_back(tf_prefix + "freedrive_mode/enable");
  config.names.emplace_back(tf_prefix + "freedrive_mode/abort");

  return config;
}

controller_interface::InterfaceConfiguration ur_controllers::FreedriveModeController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;

  return config;
}

controller_interface::CallbackReturn
ur_controllers::FreedriveModeController::on_configure(const rclcpp_lifecycle::State& previous_state)
{

  start_action_server();
  freedrive_active_ = false;

  const auto logger = get_node()->get_logger();

  if (!freedrive_param_listener_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during configuration");
    return controller_interface::CallbackReturn::ERROR;
  }

  // update the dynamic map parameters
  freedrive_param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  freedrive_params_ = freedrive_param_listener_->get_params();

  return ControllerInterface::on_configure(previous_state);
}

void FreedriveModeController::start_action_server(void)
{
  freedrive_mode_action_server_ = rclcpp_action::create_server<ur_msgs::action::EnableFreedriveMode>(
      get_node(), std::string(get_node()->get_name()) + "/freedrive_mode",
      std::bind(&FreedriveModeController::goal_received_callback, this, std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&FreedriveModeController::goal_cancelled_callback, this, std::placeholders::_1),
      std::bind(&FreedriveModeController::goal_accepted_callback, this, std::placeholders::_1));
  return;
}

controller_interface::CallbackReturn
ur_controllers::FreedriveModeController::on_activate(const rclcpp_lifecycle::State& state)
{

  {
    const std::string interface_name = freedrive_params_.tf_prefix + "freedrive_mode/"
                                                                       "async_success";
    auto it = std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
                           [&](auto& interface) { return (interface.get_name() == interface_name); });
    if (it != command_interfaces_.end()) {
      abort_command_interface_ = *it;
      async_success_command_interface_ = *it;
    } else {
      RCLCPP_ERROR(get_node()->get_logger(), "Did not find '%s' in command interfaces.", interface_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  {
    const std::string interface_name = freedrive_params_.tf_prefix + "freedrive_mode/"
                                                                       "enable";
    auto it = std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
                           [&](auto& interface) { return (interface.get_name() == interface_name); });
    if (it != command_interfaces_.end()) {
      enable_command_interface_ = *it;
    } else {
      RCLCPP_ERROR(get_node()->get_logger(), "Did not find '%s' in command interfaces.", interface_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  {
    const std::string interface_name = freedrive_params_.tf_prefix + "freedrive_mode/"
                                                                       "abort";
    auto it = std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
                           [&](auto& interface) { return (interface.get_name() == interface_name); });
    if (it != command_interfaces_.end()) {
      abort_command_interface_ = *it;
    } else {
      RCLCPP_ERROR(get_node()->get_logger(), "Did not find '%s' in command interfaces.", interface_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  return ControllerInterface::on_activate(state);
}

controller_interface::CallbackReturn
ur_controllers::FreedriveModeController::on_deactivate(const rclcpp_lifecycle::State&)
{
  abort_command_interface_->get().set_value(1.0);
  if (freedrive_active_) {
    const auto active_goal = *rt_active_goal_.readFromRT();
    std::shared_ptr<ur_msgs::action::EnableFreedriveMode::Result> result =
        std::make_shared<ur_msgs::action::EnableFreedriveMode::Result>();
    //result->set__error_string("Deactivating freedrive mode, since the controller is being deactivated.");
    active_goal->setAborted(result);
    rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
    end_goal();
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type ur_controllers::FreedriveModeController::update(const rclcpp::Time& /*time*/,
                                                                              const rclcpp::Duration& /*period*/)
{
  const auto active_goal = *rt_active_goal_.readFromRT();

  if (active_goal && freedrive_active_) {
    // Check if the freedrive mode has been aborted from the hardware interface. E.g. the robot was stopped on the teach
    // pendant.
    if (abort_command_interface_->get().get_value() == 1.0) {
      RCLCPP_INFO(get_node()->get_logger(), "Freedrive mode aborted by hardware, aborting action.");
      std::shared_ptr<ur_msgs::action::EnableFreedriveMode::Result> result =
          std::make_shared<ur_msgs::action::EnableFreedriveMode::Result>();
      active_goal->setAborted(result);
      end_goal();
      return controller_interface::return_type::OK;
    }
  }
  return controller_interface::return_type::OK;
}

rclcpp_action::GoalResponse FreedriveModeController::goal_received_callback(
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const ur_msgs::action::EnableFreedriveMode::Goal> goal)
{
  RCLCPP_INFO(get_node()->get_logger(), "Received new request for freedrive mode activation.");
  // Precondition: Running controller
  if (get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    RCLCPP_ERROR(get_node()->get_logger(), "Can't enable freedrive mode. Freedrive mode controller is not running.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (freedrive_active_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Freedrive mode is already enabled: ignoring new request.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse FreedriveModeController::goal_cancelled_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<ur_msgs::action::EnableFreedriveMode>> goal_handle)
{
  // Check that cancel request refers to currently active goal (if any)
  const auto active_goal = *rt_active_goal_.readFromNonRT();
  if (active_goal && active_goal->gh_ == goal_handle) {
    RCLCPP_INFO(get_node()->get_logger(), "Disabling freedrive mode requested.");

    // Setting interfaces to deactivate freedrive mode
    async_success_command_interface_->get().set_value(ASYNC_WAITING);
    abort_command_interface_->get().set_value(1.0);

    RCLCPP_DEBUG(get_node()->get_logger(), "Waiting for freedrive mode to be disabled.");
    if (!waitForAsyncCommand(
            [&]() { return async_success_command_interface_->get().get_value(); })) {
      RCLCPP_WARN(get_node()->get_logger(), "Could not verify that freedrive mode has been deactivated.");
    }

    bool success = static_cast<bool>(async_success_command_interface_->get().get_value());
    if (success) {
      RCLCPP_INFO(get_node()->get_logger(), "Freedrive mode has been disabled successfully.");
    } else {
      RCLCPP_ERROR(get_node()->get_logger(), "Could not disable freedrive mode.");
    }

    // Mark the current goal as canceled
    auto result = std::make_shared<FreedriveModeAction::Result>();
    active_goal->setCanceled(result);
    rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
    freedrive_active_ = false;
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

void FreedriveModeController::goal_accepted_callback(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<ur_msgs::action::EnableFreedriveMode>> goal_handle)
{
  RCLCPP_INFO_STREAM(get_node()->get_logger(), "Starting freedrive mode.");

  // reset success flag
  async_success_command_interface_->get().set_value(ASYNC_WAITING);

  RCLCPP_DEBUG(get_node()->get_logger(), "Waiting for freedrive mode to be set.");
  if (!waitForAsyncCommand(
          [&]() { return async_success_command_interface_->get().get_value(); })) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not verify that freedrive mode has been activated.");
  }

  bool success = static_cast<bool>(async_success_command_interface_->get().get_value());
  if (success) {
    RCLCPP_INFO(get_node()->get_logger(), "Freedrive mode has been set successfully.");
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not set the freedrive mode.");
  }
  // Action handling will be done from the timer callback to avoid those things in the realtime
  // thread. First, we delete the existing (if any) timer by resetting the pointer and then create a new
  // one.
  //
  RealtimeGoalHandlePtr rt_goal = std::make_shared<RealtimeGoalHandle>(goal_handle);
  rt_goal->execute();
  rt_active_goal_.writeFromNonRT(rt_goal);
  goal_handle_timer_.reset();
  goal_handle_timer_ = get_node()->create_wall_timer(action_monitor_period_.to_chrono<std::chrono::nanoseconds>(),
                                                     std::bind(&RealtimeGoalHandle::runNonRealtime, rt_goal));
  freedrive_active_ = true;
  return;
}

void FreedriveModeController::end_goal()
{
  freedrive_active_ = false;
}

bool FreedriveModeController::waitForAsyncCommand(std::function<double(void)> get_value)
{
  const auto maximum_retries = freedrive_params_.check_io_successful_retries;
  int retries = 0;
  while (get_value() == ASYNC_WAITING) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    retries++;

    if (retries > maximum_retries)
      return false;
  }
  return true;
}
}  // namespace ur_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ur_controllers::FreedriveModeController, controller_interface::ControllerInterface)