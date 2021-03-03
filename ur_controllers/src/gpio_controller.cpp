// Copyright (c) 2021 PickNik LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ur_controllers/gpio_controller.h"

namespace ur_controllers
{
controller_interface::InterfaceConfiguration ur_controllers::GPIOController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (size_t i = 0; i < 18; ++i)
  {
    config.names.emplace_back("gpio/standard_digital_output_cmd_" + std::to_string(i));
  }

  for (size_t i = 0; i < 2; ++i)
  {
    config.names.emplace_back("gpio/standard_analog_output_cmd_" + std::to_string(i));
  }

  config.names.emplace_back("gpio/speed_scaling_factor_cmd");

  return config;
}

controller_interface::InterfaceConfiguration ur_controllers::GPIOController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (size_t i = 0; i < 18; ++i)
  {
    config.names.emplace_back("gpio/digital_output_" + std::to_string(i));
    config.names.emplace_back("gpio/digital_input_" + std::to_string(i));
  }

  for (size_t i = 0; i < 11; ++i)
  {
    config.names.emplace_back("gpio/safety_status_bit_" + std::to_string(i));
  }

  for (size_t i = 0; i < 4; ++i)
  {
    config.names.emplace_back("gpio/analog_io_type_" + std::to_string(i));
    config.names.emplace_back("gpio/robot_status_bit_" + std::to_string(i));
  }

  for (size_t i = 0; i < 2; ++i)
  {
    config.names.emplace_back("gpio/tool_analog_input_type_" + std::to_string(i));
    config.names.emplace_back("gpio/tool_analog_input_" + std::to_string(i));
    config.names.emplace_back("gpio/standard_analog_input_" + std::to_string(i));
    config.names.emplace_back("gpio/standard_analog_output_" + std::to_string(i));
  }

  config.names.emplace_back("gpio/robot_mode");
  config.names.emplace_back("gpio/safety_mode");
  config.names.emplace_back("gpio/tool_output_voltage");

  config.names.emplace_back("gpio/tool_mode");
  config.names.emplace_back("gpio/tool_output_current");
  config.names.emplace_back("gpio/tool_temperature");

  config.names.emplace_back("gpio/io_async_success");
  config.names.emplace_back("gpio/scaling_async_success");

  return config;
}

controller_interface::return_type ur_controllers::GPIOController::init(const std::string& controller_name)
{
  return ControllerInterface::init(controller_name);
}

controller_interface::return_type ur_controllers::GPIOController::update()
{
  return controller_interface::return_type::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ur_controllers::GPIOController::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  return LifecycleNodeInterface::on_configure(previous_state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ur_controllers::GPIOController::on_activate(const rclcpp_lifecycle::State& previous_state)
{
  return LifecycleNodeInterface::on_activate(previous_state);
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ur_controllers::GPIOController::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
  return LifecycleNodeInterface::on_deactivate(previous_state);
}

}  // namespace ur_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ur_controllers::GPIOController, controller_interface::ControllerInterface)
