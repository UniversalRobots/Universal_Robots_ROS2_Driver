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

#include <ur_controllers/twist_controller.hpp>

namespace ur_controllers
{
controller_interface::CallbackReturn TwistController::on_init()
{
  param_listener_ = std::make_shared<twist_controller::ParamListener>(get_node());
  controller_params_ = param_listener_->get_params();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TwistController::on_configure(const rclcpp_lifecycle::State& /* previous_state */)
{
  twist_command_subscriber_ = get_node()->create_subscription<TwistStamped>(
      "~/twist", rclcpp::SystemDefaultsQoS(), [this](const std::shared_ptr<TwistStamped> msg) -> void {
        if (!subscriber_is_active_) {
          RCLCPP_WARN(get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
          return;
        }
        if ((msg->header.stamp.sec == 0) && (msg->header.stamp.nanosec == 0)) {
          RCLCPP_WARN_ONCE(get_node()->get_logger(), "Received TwistStamped with zero timestamp, setting it to current "
                                                     "time, this message will only be shown once");
          msg->header.stamp = get_node()->now();
        }

        if (msg->header.frame_id != controller_params_.tf_prefix + "base") {
          // TODO(urfeex): Support transforming the twist?
          RCLCPP_WARN(get_node()->get_logger(),
                      "Received TwistStamped with frame_id '%s'. Currently, it is only supported to command twists in "
                      "'%s'. The command will be ignored.",
                      msg->header.frame_id.c_str(), (controller_params_.tf_prefix + "base").c_str());
          return;
        }

        // TODO(urfeex): Check timestamp. Do not accept old commands.

        received_twist_msg_.set(*msg);
      });

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration TwistController::command_interface_configuration() const
{
  // No command interfaces currently
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  const std::string tf_prefix = controller_params_.tf_prefix;
  config.names.push_back(tf_prefix + "twist/linear_velocity_x");
  config.names.push_back(tf_prefix + "twist/linear_velocity_y");
  config.names.push_back(tf_prefix + "twist/linear_velocity_z");
  config.names.push_back(tf_prefix + "twist/angular_velocity_x");
  config.names.push_back(tf_prefix + "twist/angular_velocity_y");
  config.names.push_back(tf_prefix + "twist/angular_velocity_z");

  return config;
}

controller_interface::InterfaceConfiguration TwistController::state_interface_configuration() const
{
  // TODO(urfeex): Add state interfaces for tcp speed

  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;

  return config;
}

controller_interface::CallbackReturn TwistController::on_activate(const rclcpp_lifecycle::State& state)
{
  reset();
  subscriber_is_active_ = true;

  return ControllerInterface::on_activate(state);
}

controller_interface::CallbackReturn TwistController::on_deactivate(const rclcpp_lifecycle::State&)
{
  subscriber_is_active_ = false;
  reset();
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type TwistController::update(const rclcpp::Time& /*time*/,
                                                          const rclcpp::Duration& /*period*/)
{
  // In case we can't read from the subscriber's buffer, we will not update the command_msg_.
  auto current_cmd = received_twist_msg_.try_get();
  if (current_cmd.has_value()) {
    command_msg_ = current_cmd.value();
  }

  // TODO(urfeex): Check if the command_msg_ is valid, e.g., if the timestamp is not too old.

  bool success = true;
  success &= command_interfaces_[0].set_value(command_msg_.twist.linear.x);
  success &= command_interfaces_[1].set_value(command_msg_.twist.linear.y);
  success &= command_interfaces_[2].set_value(command_msg_.twist.linear.z);
  success &= command_interfaces_[3].set_value(command_msg_.twist.angular.x);
  success &= command_interfaces_[4].set_value(command_msg_.twist.angular.y);
  success &= command_interfaces_[5].set_value(command_msg_.twist.angular.z);

  if (!success) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to write to command interfaces.");
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

void TwistController::reset()
{
  command_msg_.twist.linear.x = 0.0;
  command_msg_.twist.linear.y = 0.0;
  command_msg_.twist.linear.z = 0.0;
  command_msg_.twist.angular.x = 0.0;
  command_msg_.twist.angular.y = 0.0;
  command_msg_.twist.angular.z = 0.0;
  received_twist_msg_.set(command_msg_);
}
}  // namespace ur_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ur_controllers::TwistController, controller_interface::ControllerInterface)
