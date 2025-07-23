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

#include "ur_controllers/pd_torque_controller.hpp"
#include <cmath>
#include <limits>
#include <rclcpp/logging.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/utilities.hpp>

namespace ur_controllers
{
controller_interface::CallbackReturn PDTorqueController::on_init()
{
  try {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<pd_torque_controller::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration PDTorqueController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  const std::string tf_prefix = params_.tf_prefix;

  // Get the command interfaces needed for freedrive mode from the hardware interface
  for (size_t i = 0; i < 6; ++i) {
    config.names.emplace_back(tf_prefix + "pd_control/pd_target_" + std::to_string(i));
  }
  config.names.emplace_back(tf_prefix + "pd_control/pd_control_type");

  return config;
}

controller_interface::InterfaceConfiguration PDTorqueController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::CallbackReturn PDTorqueController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
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

  // Create subscribers
  joints_command_sub_ = get_node()->create_subscription<CmdType>(
      "~/joint_command", rclcpp::SystemDefaultsQoS(),
      [this](const CmdType::SharedPtr msg) { command_callback(msg, ControlType::JOINT_SPACE); });
  // ToDo: We should rather subscribe to a Pose or PoseStamped message here.
  task_space_command_sub_ = get_node()->create_subscription<CmdType>(
      "~/task_space_commands", rclcpp::SystemDefaultsQoS(),
      [this](const CmdType::SharedPtr msg) { command_callback(msg, ControlType::TASK_SPACE); });

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PDTorqueController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  control_type_.store(ControlType::NONE);
  joint_commands_.data.resize(6, std::numeric_limits<double>::quiet_NaN());  // Initialize joint commands to zero
  rt_command_.try_set(joint_commands_);

  RCLCPP_INFO(get_node()->get_logger(), "PDTorqueController activated.");
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PDTorqueController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  control_type_.store(ControlType::NONE);
  RCLCPP_INFO(get_node()->get_logger(), "PDTorqueController deactivated.");
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PDTorqueController::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(get_node()->get_logger(), "PDTorqueController cleaned up.");
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void PDTorqueController::command_callback(const CmdType::SharedPtr msg, const ControlType control_type)
{
  if (!std::all_of(msg->data.cbegin(), msg->data.cend(), [](const auto& value) { return std::isfinite(value); })) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *(get_node()->get_clock()), 1000,
                         "One or more entries aren't finite. Dropping command.");
    return;
  }
  if (get_node()->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    if (msg->data.size() != 6) {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *(get_node()->get_clock()), 1000,
                           "The given command has %zu entries, while it should have 6 entries. Dropping command.",
                           msg->data.size());
      return;
    }
  }
  rt_command_.set(*msg);
  control_type_.store(control_type);
}

controller_interface::return_type PDTorqueController::update(const rclcpp::Time& /*time*/,
                                                             const rclcpp::Duration& /*period*/)
{
  // Check if we have a command
  auto command = rt_command_.try_get();
  if (command.has_value()) {
    joint_commands_ = command.value();
  }

  for (size_t i = 0; i < 6; ++i) {
    if (std::isfinite(joint_commands_.data[i])) {
      if (!command_interfaces_[i].set_value(joint_commands_.data[i])) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to set command interface %zu to value %f", i,
                     joint_commands_.data[i]);
        return controller_interface::return_type::ERROR;
      }
    }
  }

  if (!command_interfaces_[6].set_value(static_cast<double>(control_type_.load()))) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to set control type to value %d",
                 static_cast<int>(control_type_.load()));
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

}  // namespace ur_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ur_controllers::PDTorqueController, controller_interface::ControllerInterface)
