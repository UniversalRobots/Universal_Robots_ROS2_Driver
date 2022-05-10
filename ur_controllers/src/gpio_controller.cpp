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

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Lovro Ivanov lovro.ivanov@gmail.com
 * \date    2021-02-20
 *
 */
//----------------------------------------------------------------------

#include "ur_controllers/gpio_controller.hpp"

#include <string>

namespace ur_controllers
{
controller_interface::InterfaceConfiguration GPIOController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (size_t i = 0; i < 18; ++i) {
    config.names.emplace_back("gpio/standard_digital_output_cmd_" + std::to_string(i));
  }

  for (size_t i = 0; i < 2; ++i) {
    config.names.emplace_back("gpio/standard_analog_output_cmd_" + std::to_string(i));
  }

  config.names.emplace_back("gpio/io_async_success");

  config.names.emplace_back("speed_scaling/target_speed_fraction_cmd");

  config.names.emplace_back("speed_scaling/target_speed_fraction_async_success");

  return config;
}

controller_interface::InterfaceConfiguration ur_controllers::GPIOController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // digital io
  for (size_t i = 0; i < 18; ++i) {
    config.names.emplace_back("gpio/digital_output_" + std::to_string(i));
  }

  for (size_t i = 0; i < 18; ++i) {
    config.names.emplace_back("gpio/digital_input_" + std::to_string(i));
  }

  // analog io
  for (size_t i = 0; i < 2; ++i) {
    config.names.emplace_back("gpio/standard_analog_output_" + std::to_string(i));
  }

  for (size_t i = 0; i < 2; ++i) {
    config.names.emplace_back("gpio/standard_analog_input_" + std::to_string(i));
  }

  for (size_t i = 0; i < 4; ++i) {
    config.names.emplace_back("gpio/analog_io_type_" + std::to_string(i));
  }

  // tool
  config.names.emplace_back("gpio/tool_mode");
  config.names.emplace_back("gpio/tool_output_voltage");
  config.names.emplace_back("gpio/tool_output_current");
  config.names.emplace_back("gpio/tool_temperature");

  for (size_t i = 0; i < 2; ++i) {
    config.names.emplace_back("gpio/tool_analog_input_" + std::to_string(i));
  }
  for (size_t i = 0; i < 2; ++i) {
    config.names.emplace_back("gpio/tool_analog_input_type_" + std::to_string(i));
  }

  // robot
  config.names.emplace_back("gpio/robot_mode");
  for (size_t i = 0; i < 4; ++i) {
    config.names.emplace_back("gpio/robot_status_bit_" + std::to_string(i));
  }

  // safety
  config.names.emplace_back("gpio/safety_mode");
  for (size_t i = 0; i < 11; ++i) {
    config.names.emplace_back("gpio/safety_status_bit_" + std::to_string(i));
  }
  config.names.emplace_back("system_interface/initialized");

  // program running
  config.names.emplace_back("gpio/program_running");

  return config;
}

controller_interface::return_type ur_controllers::GPIOController::init(const std::string& controller_name)
{
  initMsgs();

  return ControllerInterface::init(controller_name);
}

controller_interface::return_type ur_controllers::GPIOController::update()
{
  publishIO();
  publishToolData();
  publishRobotMode();
  publishSafetyMode();
  publishProgramRunning();
  return controller_interface::return_type::OK;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ur_controllers::GPIOController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void GPIOController::publishIO()
{
  for (size_t i = 0; i < 18; ++i) {
    io_msg_.digital_out_states[i].pin = i;
    io_msg_.digital_out_states[i].state = static_cast<bool>(state_interfaces_[i].get_value());

    io_msg_.digital_in_states[i].pin = i;
    io_msg_.digital_in_states[i].state =
        static_cast<bool>(state_interfaces_[i + StateInterfaces::DIGITAL_INPUTS].get_value());
  }

  for (size_t i = 0; i < 2; ++i) {
    io_msg_.analog_in_states[i].pin = i;
    io_msg_.analog_in_states[i].state =
        static_cast<float>(state_interfaces_[i + StateInterfaces::ANALOG_INPUTS].get_value());
    io_msg_.analog_in_states[i].domain =
        static_cast<uint8_t>(state_interfaces_[i + StateInterfaces::ANALOG_IO_TYPES].get_value());
  }

  for (size_t i = 0; i < 2; ++i) {
    io_msg_.analog_out_states[i].pin = i;
    io_msg_.analog_out_states[i].state =
        static_cast<float>(state_interfaces_[i + StateInterfaces::ANALOG_OUTPUTS].get_value());
    io_msg_.analog_out_states[i].domain =
        static_cast<uint8_t>(state_interfaces_[i + StateInterfaces::ANALOG_IO_TYPES + 2].get_value());
  }

  io_pub_->publish(io_msg_);
}

void GPIOController::publishToolData()
{
  tool_data_msg_.tool_mode = static_cast<uint8_t>(state_interfaces_[StateInterfaces::TOOL_MODE].get_value());
  tool_data_msg_.analog_input_range2 =
      static_cast<uint8_t>(state_interfaces_[StateInterfaces::TOOL_ANALOG_IO_TYPES].get_value());
  tool_data_msg_.analog_input_range3 =
      static_cast<uint8_t>(state_interfaces_[StateInterfaces::TOOL_ANALOG_IO_TYPES + 1].get_value());
  tool_data_msg_.analog_input2 = static_cast<float>(state_interfaces_[StateInterfaces::TOOL_ANALOG_INPUTS].get_value());
  tool_data_msg_.analog_input3 =
      static_cast<float>(state_interfaces_[StateInterfaces::TOOL_ANALOG_INPUTS + 1].get_value());
  tool_data_msg_.tool_output_voltage =
      static_cast<uint8_t>(state_interfaces_[StateInterfaces::TOOL_OUTPUT_VOLTAGE].get_value());
  tool_data_msg_.tool_current = static_cast<float>(state_interfaces_[StateInterfaces::TOOL_OUTPUT_CURRENT].get_value());
  tool_data_msg_.tool_temperature =
      static_cast<float>(state_interfaces_[StateInterfaces::TOOL_TEMPERATURE].get_value());
  tool_data_pub_->publish(tool_data_msg_);
}

void GPIOController::publishRobotMode()
{
  auto robot_mode = static_cast<int8_t>(state_interfaces_[StateInterfaces::ROBOT_MODE].get_value());

  if (robot_mode_msg_.mode != robot_mode) {
    robot_mode_msg_.mode = robot_mode;
    robot_mode_pub_->publish(robot_mode_msg_);
  }
}

void GPIOController::publishSafetyMode()
{
  auto safety_mode = static_cast<uint8_t>(state_interfaces_[StateInterfaces::SAFETY_MODE].get_value());

  if (safety_mode_msg_.mode != safety_mode) {
    safety_mode_msg_.mode = safety_mode;
    safety_mode_pub_->publish(safety_mode_msg_);
  }
}

void GPIOController::publishProgramRunning()
{
  auto program_running_value = static_cast<uint8_t>(state_interfaces_[StateInterfaces::PROGRAM_RUNNING].get_value());
  bool program_running = program_running_value == 1.0 ? true : false;
  if (program_running_msg_.data != program_running) {
    program_running_msg_.data = program_running;
    program_state_pub_->publish(program_running_msg_);
  }
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ur_controllers::GPIOController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  while (state_interfaces_[StateInterfaces::INITIALIZED_FLAG].get_value() == 0.0) {
    RCLCPP_INFO(get_node()->get_logger(), "Waiting for system interface to initialize...");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  try {
    // register publisher
    io_pub_ = get_node()->create_publisher<ur_msgs::msg::IOStates>("~/io_states", rclcpp::SystemDefaultsQoS());

    tool_data_pub_ =
        get_node()->create_publisher<ur_msgs::msg::ToolDataMsg>("~/tool_data", rclcpp::SystemDefaultsQoS());

    robot_mode_pub_ =
        get_node()->create_publisher<ur_dashboard_msgs::msg::RobotMode>("~/robot_mode", rclcpp::SystemDefaultsQoS());

    safety_mode_pub_ =
        get_node()->create_publisher<ur_dashboard_msgs::msg::SafetyMode>("~/safety_mode", rclcpp::SystemDefaultsQoS());

    auto program_state_pub_qos = rclcpp::SystemDefaultsQoS();
    program_state_pub_qos.transient_local();
    program_state_pub_ =
        get_node()->create_publisher<std_msgs::msg::Bool>("~/robot_program_running", program_state_pub_qos);

    set_io_srv_ = get_node()->create_service<ur_msgs::srv::SetIO>(
        "~/set_io", std::bind(&GPIOController::setIO, this, std::placeholders::_1, std::placeholders::_2));

    set_speed_slider_srv_ = get_node()->create_service<ur_msgs::srv::SetSpeedSliderFraction>(
        "~/set_speed_slider",
        std::bind(&GPIOController::setSpeedSlider, this, std::placeholders::_1, std::placeholders::_2));
  } catch (...) {
    return LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ur_controllers::GPIOController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  try {
    // reset publisher
    io_pub_.reset();
    tool_data_pub_.reset();
    robot_mode_pub_.reset();
    safety_mode_pub_.reset();
    program_state_pub_.reset();
    set_io_srv_.reset();
    set_speed_slider_srv_.reset();
  } catch (...) {
    return LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

bool GPIOController::setIO(ur_msgs::srv::SetIO::Request::SharedPtr req, ur_msgs::srv::SetIO::Response::SharedPtr resp)
{
  if (req->fun == req->FUN_SET_DIGITAL_OUT && req->pin >= 0 && req->pin <= 17) {
    // io async success
    command_interfaces_[CommandInterfaces::IO_ASYNC_SUCCESS].set_value(ASYNC_WAITING);
    command_interfaces_[req->pin].set_value(static_cast<double>(req->state));

    RCLCPP_INFO(node_->get_logger(), "Setting digital output '%d' to state: '%1.0f'.", req->pin, req->state);

    while (command_interfaces_[CommandInterfaces::IO_ASYNC_SUCCESS].get_value() == ASYNC_WAITING) {
      // Asynchronous wait until the hardware interface has set the io value
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    resp->success = static_cast<bool>(command_interfaces_[IO_ASYNC_SUCCESS].get_value());
    return resp->success;
  } else if (req->fun == req->FUN_SET_ANALOG_OUT && req->pin >= 0 && req->pin <= 2) {
    // io async success
    command_interfaces_[CommandInterfaces::IO_ASYNC_SUCCESS].set_value(ASYNC_WAITING);
    command_interfaces_[CommandInterfaces::ANALOG_OUTPUTS_CMD + req->pin].set_value(static_cast<double>(req->state));

    RCLCPP_INFO(node_->get_logger(), "Setting analog output '%d' to state: '%1.0f'.", req->pin, req->state);

    while (command_interfaces_[CommandInterfaces::IO_ASYNC_SUCCESS].get_value() == ASYNC_WAITING) {
      // Asynchronous wait until the hardware interface has set the io value
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    resp->success = static_cast<bool>(command_interfaces_[CommandInterfaces::IO_ASYNC_SUCCESS].get_value());
    return resp->success;
  } else {
    resp->success = false;
    return false;
  }
}

bool GPIOController::setSpeedSlider(ur_msgs::srv::SetSpeedSliderFraction::Request::SharedPtr req,
                                    ur_msgs::srv::SetSpeedSliderFraction::Response::SharedPtr resp)
{
  if (req->speed_slider_fraction >= 0.01 && req->speed_slider_fraction <= 1.0) {
    RCLCPP_INFO(node_->get_logger(), "Setting speed slider to %.2f%%.", req->speed_slider_fraction * 100.0);
    // reset success flag
    command_interfaces_[CommandInterfaces::TARGET_SPEED_FRACTION_ASYNC_SUCCESS].set_value(ASYNC_WAITING);
    // set commanding value for speed slider
    command_interfaces_[CommandInterfaces::TARGET_SPEED_FRACTION_CMD].set_value(
        static_cast<double>(req->speed_slider_fraction));

    while (command_interfaces_[CommandInterfaces::TARGET_SPEED_FRACTION_ASYNC_SUCCESS].get_value() == ASYNC_WAITING) {
      // Asynchronouse wait until the hardware interface has set the slider value
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    resp->success =
        static_cast<bool>(command_interfaces_[CommandInterfaces::TARGET_SPEED_FRACTION_ASYNC_SUCCESS].get_value());
  } else {
    RCLCPP_WARN(node_->get_logger(), "The desired speed slider fraction must be within range (0; 1.0]. Request "
                                     "ignored.");
    resp->success = false;
    return false;
  }
  return true;
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
