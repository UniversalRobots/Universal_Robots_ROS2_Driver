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

#include <cmath>
#include <ur_controllers/sine_controller.hpp>
namespace ur_controllers
{

controller_interface::CallbackReturn SineController::on_init()
{
  param_listener_ = std::make_shared<sine_controller::ParamListener>(get_node());
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SineController::on_configure(const rclcpp_lifecycle::State& /* previous_state */)
{
  params_ = param_listener_->get_params();

  if (params_.joints.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (params_.interface_name.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'interface_name' parameter was empty");
    return controller_interface::CallbackReturn::ERROR;
  }

  command_interfaces.clear();
  for (const auto& joint : params_.joints) {
    command_interfaces.push_back(joint + "/" + params_.interface_name);
  }

  for (const auto joint_index : params_.joint_indices_to_control) {
    if (joint_index >= command_interfaces.size()) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Joint index %zu in 'joint_indices_to_control' is out of bounds for the number of joints (%zu)",
                   joint_index, command_interfaces.size());
      return controller_interface::CallbackReturn::ERROR;
    }
    command_interface_indices.push_back(joint_index);
  }

  amplitude_ = params_.amplitude;
  frequency_ = params_.frequency;

  publisher_ = get_node()->create_publisher<CommandPubT>("~/command_sent", rclcpp::SystemDefaultsQoS());
  command_publisher_ = std::make_unique<CommandPublisher>(publisher_);

  command_msg_.data.resize(command_interfaces.size(), 0.0);

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration SineController::command_interface_configuration() const
{
  // No command interfaces currently
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  config.names = command_interfaces;

  return config;
}

controller_interface::InterfaceConfiguration SineController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  return config;
}

controller_interface::return_type SineController::update(const rclcpp::Time& time, const rclcpp::Duration& /* period */)
{
  // if (start_time_.nanoseconds() == 0) {
  // start_time_ = time;

  //}
  // double x = (time - start_time_).seconds();
  double x = start_time_.seconds();
  const auto cycle_time = rclcpp::Duration(std::chrono::milliseconds(2));
  start_time_ += cycle_time;
  double sine = amplitude_ * sin(2 * M_PI * frequency_ * x);

  for (auto index : command_interface_indices) {
    command_msg_.data[index] = sine;
    command_msg_.header.stamp = time;
    if (!command_interfaces_[index].set_value(sine)) {
      RCLCPP_WARN(get_node()->get_logger(), "Unable to set the command interface value %s: value = %f",
                  command_interfaces_[index].get_name().c_str(), sine);
      return controller_interface::return_type::OK;
    }
  }

  command_publisher_->try_publish(command_msg_);

  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn SineController::on_activate(const rclcpp_lifecycle::State& /* previous_state */)
{
  start_time_ = rclcpp::Time(0);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SineController::on_deactivate(const rclcpp_lifecycle::State& /* previous_state */)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

}  // namespace ur_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ur_controllers::SineController, controller_interface::ControllerInterface)
