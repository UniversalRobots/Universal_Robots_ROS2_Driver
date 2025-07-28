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

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/utilities.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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
  const std::string tf_prefix = params_.tf_prefix;

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_node()->get_clock());
  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

  // Create subscribers
  joints_command_sub_ = get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/joint_commands", rclcpp::SystemDefaultsQoS(), [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() != 6) {
          RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *(get_node()->get_clock()), 1000,
                               "The given command has %zu entries, while it should have 6 entries. Dropping command.",
                               msg->data.size());
          return;
        }
        std::array<double, 6> joint_commands;
        std::copy(msg->data.begin(), msg->data.end(), joint_commands.begin());
        command_callback(joint_commands, ControlType::JOINT_SPACE);
      });
  task_space_command_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
      "~/task_space_commands", rclcpp::SystemDefaultsQoS(),
      [this, tf_prefix](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        std::array<double, 6> task_space_command;
        auto target_transformed = *msg;
        if (msg->header.frame_id != tf_prefix + "base") {
          try {
            target_transformed = tf_buffer_->transform(*msg, tf_prefix + "base");
          } catch (const tf2::TransformException& ex) {
            RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *(get_node()->get_clock()), 1000,
                                 "Could not transform %s to robot base: %s. Dropping command.",
                                 msg->header.frame_id.c_str(), ex.what());
          }
        }
        task_space_command[0] = target_transformed.pose.position.x;
        task_space_command[1] = target_transformed.pose.position.y;
        task_space_command[2] = target_transformed.pose.position.z;
        tf2::Quaternion quat_tf;
        tf2::convert(target_transformed.pose.orientation, quat_tf);
        const double angle = quat_tf.getAngle();
        const auto axis = quat_tf.getAxis();
        task_space_command[3] = axis.x() * angle;  // rx
        task_space_command[4] = axis.y() * angle;  // ry
        task_space_command[5] = axis.z() * angle;  // rz
        command_callback(task_space_command, ControlType::TASK_SPACE);
      });

  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn PDTorqueController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  control_type_.store(ControlType::NONE);
  joint_commands_.fill(std::numeric_limits<double>::quiet_NaN());
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

void PDTorqueController::command_callback(const std::array<double, 6> command_vec, const ControlType control_type)
{
  if (!std::all_of(command_vec.cbegin(), command_vec.cend(), [](const auto& value) { return std::isfinite(value); })) {
    RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *(get_node()->get_clock()), 1000,
                         "One or more entries aren't finite. Dropping command.");
    return;
  }
  rt_command_.set(command_vec);
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
    if (std::isfinite(joint_commands_[i])) {
      if (!command_interfaces_[i].set_value(joint_commands_[i])) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to set command interface %zu to value %f", i,
                     joint_commands_[i]);
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
