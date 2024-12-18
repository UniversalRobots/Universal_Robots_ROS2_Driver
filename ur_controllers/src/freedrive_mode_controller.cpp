
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
  timeout_interval_ = std::chrono::seconds(freedrive_params_.inactive_timeout);

  // Get the command interfaces needed for freedrive mode from the hardware interface
  config.names.emplace_back(tf_prefix + "freedrive_mode/async_success");
  config.names.emplace_back(tf_prefix + "freedrive_mode/enable");
  config.names.emplace_back(tf_prefix + "freedrive_mode/abort");

  return config;
}

controller_interface::InterfaceConfiguration
ur_controllers::FreedriveModeController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;

  return config;
}

controller_interface::CallbackReturn
ur_controllers::FreedriveModeController::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  // Subscriber definition
  enable_freedrive_mode_sub_ = get_node()->create_subscription<std_msgs::msg::Bool>(
      "~/enable_freedrive_mode", 10,
      std::bind(&FreedriveModeController::freedrive_cmd_callback, this, std::placeholders::_1));

  timer_started_ = false;

  const auto logger = get_node()->get_logger();

  if (!freedrive_param_listener_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during configuration");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Update the dynamic map parameters
  freedrive_param_listener_->refresh_dynamic_parameters();

  // Get parameters from the listener in case they were updated
  freedrive_params_ = freedrive_param_listener_->get_params();

  start_logging_thread();

  return ControllerInterface::on_configure(previous_state);
}

controller_interface::CallbackReturn
ur_controllers::FreedriveModeController::on_activate(const rclcpp_lifecycle::State& state)
{
  change_requested_ = false;
  freedrive_active_ = false;
  async_state_ = std::numeric_limits<double>::quiet_NaN();

  first_log_ = false;
  logging_thread_running_ = true;
  logging_requested_ = false;

  {
    const std::string interface_name = freedrive_params_.tf_prefix + "freedrive_mode/"
                                                                     "async_success";
    auto it = std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
                           [&](auto& interface) { return (interface.get_name() == interface_name); });
    if (it != command_interfaces_.end()) {
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
      abort_command_interface_->get().set_value(0.0);
    } else {
      RCLCPP_ERROR(get_node()->get_logger(), "Did not find '%s' in command interfaces.", interface_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  return ControllerInterface::on_activate(state);
}

controller_interface::CallbackReturn
ur_controllers::FreedriveModeController::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/)
{
  abort_command_interface_->get().set_value(1.0);

  stop_logging_thread();

  return CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ur_controllers::FreedriveModeController::on_deactivate(const rclcpp_lifecycle::State&)
{
  freedrive_active_ = false;

  freedrive_sub_timer_.reset();
  timer_started_ = false;

  return CallbackReturn::SUCCESS;
}

controller_interface::return_type ur_controllers::FreedriveModeController::update(const rclcpp::Time& /*time*/,
                                                                                  const rclcpp::Duration& /*period*/)
{
  async_state_ = async_success_command_interface_->get().get_value();

  if (change_requested_) {
    if (freedrive_active_) {
      // Check if the freedrive mode has been aborted from the hardware interface. E.g. the robot was stopped on the
      // teach pendant.
      if (!std::isnan(abort_command_interface_->get().get_value()) &&
          abort_command_interface_->get().get_value() == 1.0) {
        RCLCPP_INFO(get_node()->get_logger(), "Freedrive mode aborted by hardware, aborting request.");
        freedrive_active_ = false;
        return controller_interface::return_type::OK;
      } else {
        RCLCPP_INFO(get_node()->get_logger(), "Received command to start Freedrive Mode.");

        // Set command interface to enable
        enable_command_interface_->get().set_value(1.0);

        async_success_command_interface_->get().set_value(ASYNC_WAITING);
        async_state_ = ASYNC_WAITING;
      }

    } else {
      RCLCPP_INFO(get_node()->get_logger(), "Received command to stop Freedrive Mode.");

      abort_command_interface_->get().set_value(1.0);

      async_success_command_interface_->get().set_value(ASYNC_WAITING);
      async_state_ = ASYNC_WAITING;
    }
    first_log_ = true;
    change_requested_ = false;
  }

  if ((async_state_ == 1.0) && (first_log_)) {
    first_log_ = false;
    logging_requested_ = true;

    // Notify logging thread
    logging_condition_.notify_one();
  }
  return controller_interface::return_type::OK;
}

void FreedriveModeController::freedrive_cmd_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  // Process the freedrive_mode command.
  if (get_node()->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    if (msg->data) {
      if ((!freedrive_active_) && (!change_requested_)) {
        freedrive_active_ = true;
        change_requested_ = true;
        start_timer();
      }
    } else {
      if ((freedrive_active_) && (!change_requested_)) {
        freedrive_active_ = false;
        change_requested_ = true;
      }
    }
  }

  if (freedrive_sub_timer_) {
    freedrive_sub_timer_->reset();
  }
}

void FreedriveModeController::start_timer()
{
  if (!timer_started_) {
    // Start the timer only after the first message is received
    freedrive_sub_timer_ =
        get_node()->create_wall_timer(timeout_interval_, std::bind(&FreedriveModeController::timeout_callback, this));
    timer_started_ = true;

    RCLCPP_INFO(get_node()->get_logger(), "Timer started after receiving first command.");
  }
}

void FreedriveModeController::timeout_callback()
{
  if (timer_started_ && freedrive_active_) {
    RCLCPP_INFO(get_node()->get_logger(), "Freedrive mode will be deactivated since no new message received.");

    freedrive_active_ = false;
    change_requested_ = true;
  }

  timer_started_ = false;
}

void FreedriveModeController::start_logging_thread()
{
  if (!logging_thread_running_) {
    logging_thread_running_ = true;
    logging_thread_ = std::thread(&FreedriveModeController::log_task, this);
  }
}

void FreedriveModeController::stop_logging_thread()
{
  logging_thread_running_ = false;
  if (logging_thread_.joinable()) {
    logging_thread_.join();
  }
}

void FreedriveModeController::log_task()
{
  while (logging_thread_running_) {
    std::unique_lock<std::mutex> lock(log_mutex_);

    auto condition = [this] { return !logging_thread_running_ || logging_requested_; };

    // Wait for the condition
    logging_condition_.wait(lock, condition);

    if (!logging_thread_running_)
      break;

    if (freedrive_active_) {
      RCLCPP_INFO(get_node()->get_logger(), "Freedrive mode has been enabled successfully.");
    } else {
      RCLCPP_INFO(get_node()->get_logger(), "Freedrive mode has been disabled successfully.");
    }

    // Reset to log only once
    logging_requested_ = false;
  }
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
