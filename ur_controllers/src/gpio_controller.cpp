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

#include <ur_controllers/gpio_controller.h>

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
  }

  for (size_t i = 0; i < 18; ++i)
  {
    config.names.emplace_back("gpio/digital_input_" + std::to_string(i));
  }

  for (size_t i = 0; i < 11; ++i)
  {
    config.names.emplace_back("gpio/safety_status_bit_" + std::to_string(i));
  }

  for (size_t i = 0; i < 4; ++i)
  {
    config.names.emplace_back("gpio/analog_io_type_" + std::to_string(i));
  }

  for (size_t i = 0; i < 4; ++i)
  {
    config.names.emplace_back("gpio/robot_status_bit_" + std::to_string(i));
  }

  for (size_t i = 0; i < 2; ++i)
  {
    config.names.emplace_back("gpio/tool_analog_input_type_" + std::to_string(i));
  }

  for (size_t i = 0; i < 2; ++i)
  {
    config.names.emplace_back("gpio/tool_analog_input_" + std::to_string(i));
  }

  for (size_t i = 0; i < 2; ++i)
  {
    config.names.emplace_back("gpio/standard_analog_input_" + std::to_string(i));
  }

  for (size_t i = 0; i < 2; ++i)
  {
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
  initMsgs();

  return ControllerInterface::init(controller_name);
}

controller_interface::return_type ur_controllers::GPIOController::update()
{
  return controller_interface::return_type::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ur_controllers::GPIOController::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  try
  {
    // register publisher
    io_pub_ = get_node()->create_publisher<ur_msgs::msg::IOStates>("io_states", rclcpp::SystemDefaultsQoS());

    tool_data_pub_ = get_node()->create_publisher<ur_msgs::msg::ToolDataMsg>("tool_data", rclcpp::SystemDefaultsQoS());

    robot_mode_pub_ =
        get_node()->create_publisher<ur_dashboard_msgs::msg::RobotMode>("robot_mode", rclcpp::SystemDefaultsQoS());

    safety_mode_pub_ =
        get_node()->create_publisher<ur_dashboard_msgs::msg::SafetyMode>("safety_mode", rclcpp::SystemDefaultsQoS());

    set_io_srv_ = get_node()->create_service<ur_msgs::srv::SetIO>(
        "set_speed_slider", std::bind(&GPIOController::setIO, this, std::placeholders::_1, std::placeholders::_2));

    set_speed_slider_srv_ = get_node()->create_service<ur_msgs::srv::SetSpeedSliderFraction>(
        "set_io", std::bind(&GPIOController::setSpeedSlider, this, std::placeholders::_1, std::placeholders::_2));
  }
  catch (...)
  {
    return LifecycleNodeInterface::CallbackReturn::ERROR;
  }

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

bool GPIOController::setIO(ur_msgs::srv::SetIO::Request::SharedPtr req, ur_msgs::srv::SetIO::Response::SharedPtr resp)
{
  return false;
}

bool GPIOController::setSpeedSlider(ur_msgs::srv::SetSpeedSliderFraction::Request::SharedPtr req,
                                    ur_msgs::srv::SetSpeedSliderFraction::Response::SharedPtr resp)
{
  return false;
}

void GPIOController::initMsgs()
{
  io_msg_.digital_in_states.resize(standard_digital_output_cmd_.size());
  io_msg_.digital_out_states.resize(standard_digital_output_cmd_.size());
  io_msg_.analog_in_states.resize(2);
  io_msg_.analog_out_states.resize(2);
}

}  // namespace ur_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ur_controllers::GPIOController, controller_interface::ControllerInterface)
