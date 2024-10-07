
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

#include <rclcpp/logging.hpp>

#include <ur_controllers/freedrive_mode_controller.hpp>
namespace ur_controllers
{
controller_interface::CallbackReturn FreedriveModeController::on_init()
{
  // I shouldn't need this, the only param I use is tf_prefix
  try {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<freedrive_mode_controller::ParamListener>(get_node());
    params_ = param_listener_->get_params();
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

  const std::string tf_prefix = params_.tf_prefix;
  RCLCPP_DEBUG(get_node()->get_logger(), "Configure UR freedrive_mode controller with tf_prefix: %s", tf_prefix.c_str());

  // Get the command interfaces needed for freedrive mode from the hardware interface
  config.names.emplace_back(tf_prefix + "freedrive_mode/freedrive_mode_async_success");
  config.names.emplace_back(tf_prefix + "freedrive_mode/freedrive_mode_cmd");

  return config;
}

controller_interface::InterfaceConfiguration ur_controllers::FreedriveModeController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  const std::string tf_prefix = params_.tf_prefix;
  // Get the state interface indicating whether the hardware interface has been initialized
  config.names.emplace_back(tf_prefix + "system_interface/initialized");

  return config;
}

controller_interface::return_type ur_controllers::FreedriveModeController::update(const rclcpp::Time& /*time*/,
                                                                              const rclcpp::Duration& /*period*/)
{
  //  Take enable and update it
  if (freedrive_mode_enable_)
  {
    enableFreedriveMode();
  }
  
  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn
ur_controllers::FreedriveModeController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  const auto logger = get_node()->get_logger();

  if (!param_listener_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during configuration");
    return controller_interface::CallbackReturn::ERROR;
  }

  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ur_controllers::FreedriveModeController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  while (state_interfaces_[StateInterfaces::INITIALIZED_FLAG].get_value() == 0.0) {
    RCLCPP_INFO(get_node()->get_logger(), "Waiting for system interface to initialize...");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  // Create the publisher that will receive the command to start the freedrive_mode
  try {
    enable_freedrive_mode_sub_ = get_node()->create_subscription<std_msgs::msg::Bool>(
        "~/start_free_drive_mode", 10,
        std::bind(&FreedriveModeController::readFreedriveModeCmd, this, std::placeholders::_1));
  } catch (...) {
    return LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
ur_controllers::FreedriveModeController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  // Stop freedrive mode if this controller is deactivated.
  disableFreedriveMode();
  try {
    set_freedrive_mode_srv_.reset();
  } catch (...) {
    return LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void FreedriveModeController::readFreedriveModeCmd(const std_msgs::msg::Bool::SharedPtr msg)
{
  // Process the freedrive_mode command.
  if(msg->data)
  {
    freedrive_mode_enable_ =  true;
    RCLCPP_INFO(get_node()->get_logger(), "Received command to start Freedrive Mode.");
  } else{
    freedrive_mode_enable_ = false;
    RCLCPP_INFO(get_node()->get_logger(), "Received command to stop Freedrive Mode.");
  }
}

bool FreedriveModeController::enableFreedriveMode()
{
  // reset success flag
  command_interfaces_[CommandInterfaces::FREEDRIVE_MODE_ASYNC_SUCCESS].set_value(ASYNC_WAITING);

  // Shouldn't I have a command set to 1 start it? Like it happens for the disable

  RCLCPP_DEBUG(get_node()->get_logger(), "Waiting for freedrive mode to be set.");
  const auto maximum_retries = params_.check_io_successful_retries;
  int retries = 0;
  while (command_interfaces_[CommandInterfaces::FREEDRIVE_MODE_ASYNC_SUCCESS].get_value() == ASYNC_WAITING) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    retries++;

    if (retries > maximum_retries) {
      resp->success = false;
    }
  }

  resp->success = static_cast<bool>(command_interfaces_[CommandInterfaces::FREEDRIVE_MODE_ASYNC_SUCCESS].get_value());

  if (resp->success) {
    RCLCPP_INFO(get_node()->get_logger(), "Freedrive mode has been set successfully.");
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not set the freedrive mode.");
    return false;
  }

  return true;
}

bool FreedriveModeController::disableFreedriveMode()
{
  command_interfaces_[CommandInterfaces::FREEDRIVE_MODE_ASYNC_SUCCESS].set_value(ASYNC_WAITING);
  command_interfaces_[CommandInterfaces::FREEDRIVE_MODE_CMD].set_value(1);

  RCLCPP_DEBUG(get_node()->get_logger(), "Waiting for freedrive mode to be disabled.");
  while (command_interfaces_[CommandInterfaces::FREEDRIVE_MODE_ASYNC_SUCCESS].get_value() == ASYNC_WAITING) {
    // Asynchronous wait until the hardware interface has set the freedrive mode
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  bool success = static_cast<bool>(command_interfaces_[CommandInterfaces::FREEDRIVE_MODE_ASYNC_SUCCESS].get_value());
  if (success) {
    RCLCPP_INFO(get_node()->get_logger(), "Freedrive mode has been disabled successfully.");
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not disable freedrive mode.");
    return false;
  }
  return true;
}
}  // namespace ur_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ur_controllers::FreedriveModeController, controller_interface::ControllerInterface)
find_package(cartesian_controllers REQUIRED)
