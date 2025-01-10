// Copyright 2025, Universal Robots A/S
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
 * \author  Jacob Larsen jala@universal-robots.com
 * \date    2025-01-07
 *
 *
 *
 *
 */
//----------------------------------------------------------------------

#include <ur_controllers/tool_contact_controller.hpp>
#include "controller_interface/helpers.hpp"
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/bool.hpp"
#include <iostream>

namespace ur_controllers
{

controller_interface::CallbackReturn ToolContactController::on_init()
{
  tool_contact_param_listener_ = std::make_shared<tool_contact_controller::ParamListener>(get_node());
  tool_contact_params_ = tool_contact_param_listener_->get_params();

  // Resize this value depending on reference interfaces to be sent
  reference_interfaces_.resize(1, std::numeric_limits<double>::quiet_NaN());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ToolContactController::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration ToolContactController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  const std::string tf_prefix = tool_contact_params_.tf_prefix;

  // Start or stop tool contact functionality
  config.names.emplace_back(tf_prefix + "tool_contact/enable_cmd");
  config.names.emplace_back(tf_prefix + "tool_contact/async_success");
  config.names.emplace_back(tf_prefix + "end_tool_contact/end_tool_contact_cmd");
  config.names.emplace_back(tf_prefix + "end_tool_contact/end_tool_contact_async_success");
  return config;
}

controller_interface::InterfaceConfiguration ToolContactController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  return config;
}

controller_interface::CallbackReturn ToolContactController::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  tool_contact_enable_sub_ = get_node()->create_subscription<std_msgs::msg::Bool>(
      "~/enable_tool_contact", 10,
      std::bind(&ToolContactController::tool_contact_sub_callback, this, std::placeholders::_1));

  /*Figure out how to get reference interface by name*/
  // const std::string interface_name = tool_contact_params_.tf_prefix + get_node()->get_name() + "tool_contact/enable";
  // auto it = std::find_if(reference_interfaces_.begin(), reference_interfaces_.end(),
  //                        [&](auto& interface) { return (interface.get_name() == interface_name); });
  // if (it != reference_interfaces_.end()) {
  //   reference_interface_ = *it;
  //   reference_interface_->get().set_value(0.0);
  // } else {
  //   RCLCPP_ERROR(get_node()->get_logger(), "Did not find '%s' in command interfaces.", interface_name.c_str());
  //   return controller_interface::CallbackReturn::ERROR;
  // }
  reference_interfaces_[0] = 0.0;
  {
    const std::string interface_name = tool_contact_params_.tf_prefix + "tool_contact/enable_cmd";
    auto it = std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
                           [&](auto& interface) { return (interface.get_name() == interface_name); });
    if (it != command_interfaces_.end()) {
      enable_command_interface_ = *it;
      enable_command_interface_->get().set_value(ASYNC_STANDBY_);
    } else {
      RCLCPP_ERROR(get_node()->get_logger(), "Did not find '%s' in command interfaces.", interface_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  {
    const std::string interface_name = tool_contact_params_.tf_prefix + "tool_contact/async_success";
    auto it = std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
                           [&](auto& interface) { return (interface.get_name() == interface_name); });
    if (it != command_interfaces_.end()) {
      async_success_interface_ = *it;
      async_success_interface_->get().set_value(ASYNC_STANDBY_);
    } else {
      RCLCPP_ERROR(get_node()->get_logger(), "Did not find '%s' in command interfaces.", interface_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ToolContactController::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  tool_contact_enable_sub_.reset();

  return controller_interface::CallbackReturn::SUCCESS;
}

void ToolContactController::tool_contact_sub_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  std::cout << "data received: " << msg->data << std::endl;
  if (msg->data) {
    if (!tool_contact_active_ && !change_requested_) {
      tool_contact_active_ = true;
      change_requested_ = true;
    }
  } else {
    if (tool_contact_active_ && !change_requested_) {
      tool_contact_active_ = false;
      change_requested_ = true;
    }
  }
}

std::vector<hardware_interface::CommandInterface> ToolContactController::on_export_reference_interfaces()
{
  std::vector<hardware_interface::CommandInterface> reference_interfaces;
  reference_interfaces.reserve(1);
  reference_interfaces.push_back(
      hardware_interface::CommandInterface(get_node()->get_name(), "tool_contact/enable", &tool_contact_enable));

  return reference_interfaces;
}

std::vector<hardware_interface::StateInterface> ToolContactController::on_export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.push_back(
      hardware_interface::StateInterface(get_node()->get_name(), "tool_contact/active", &tool_contact_active));

  return state_interfaces;
}

bool ToolContactController::on_set_chained_mode(bool chained_mode)
{
  if (chained_mode) {
    tool_contact_enable_sub_.reset();
  } else {
    tool_contact_enable_sub_ = get_node()->create_subscription<std_msgs::msg::Bool>(
        "~/enable_tool_contact", 10,
        std::bind(&ToolContactController::tool_contact_sub_callback, this, std::placeholders::_1));
  }
  change_requested_ = false;
  tool_contact_active_ = false;
  return true;
}

controller_interface::return_type
ToolContactController::update_reference_from_subscribers(const rclcpp::Time& time, const rclcpp::Duration& period)
{
  if (change_requested_) {
    reference_interfaces_[0] = static_cast<double>(tool_contact_active_);
    change_requested_ = false;
  }
  return controller_interface::return_type::OK;
}

controller_interface::return_type ToolContactController::update_and_write_commands(const rclcpp::Time& time,
                                                                                   const rclcpp::Duration& period)
{
  if (reference_interfaces_[0] != old_reference_val) {
    enable_command_interface_->get().set_value(reference_interfaces_[0]);
    async_success_interface_->get().set_value(ASYNC_WAITING);
    std::cout << "Tool contact interface set to: " << reference_interfaces_[0] << std::endl;
  }

  if (async_success_interface_->get().get_value() == 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to enable tool contact mode.");
    async_success_interface_->get().set_value(ASYNC_STANDBY_);

  } else if (async_success_interface_->get().get_value() == 1.0) {
    RCLCPP_INFO(get_node()->get_logger(), "Tool contact enabled successfully ");
    async_success_interface_->get().set_value(ASYNC_STANDBY_);
  }

  old_reference_val = reference_interfaces_[0];
  return controller_interface::return_type::OK;
}

}  // namespace ur_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ur_controllers::ToolContactController, controller_interface::ChainableControllerInterface)