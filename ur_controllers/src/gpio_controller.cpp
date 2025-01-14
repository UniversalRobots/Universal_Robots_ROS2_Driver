// Copyright (c) 2021 PickNik LLC
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
 * \author  Lovro Ivanov lovro.ivanov@gmail.com
 * \date    2021-02-20
 *
 */
//----------------------------------------------------------------------

#include "ur_controllers/gpio_controller.hpp"

#include <string>

namespace ur_controllers
{
controller_interface::CallbackReturn GPIOController::on_init()
{
  try {
    initMsgs();
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<gpio_controller::ParamListener>(get_node());
    params_ = param_listener_->get_params();
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration GPIOController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  const std::string tf_prefix = params_.tf_prefix;

  for (size_t i = 0; i < 18; ++i) {
    config.names.emplace_back(tf_prefix + "gpio/standard_digital_output_cmd_" + std::to_string(i));
  }

  for (size_t i = 0; i < 2; ++i) {
    config.names.emplace_back(tf_prefix + "gpio/standard_analog_output_cmd_" + std::to_string(i));
  }
  config.names.emplace_back(tf_prefix + "gpio/tool_voltage_cmd");

  config.names.emplace_back(tf_prefix + "gpio/io_async_success");

  config.names.emplace_back(tf_prefix + "speed_scaling/target_speed_fraction_cmd");

  config.names.emplace_back(tf_prefix + "speed_scaling/target_speed_fraction_async_success");

  config.names.emplace_back(tf_prefix + "resend_robot_program/resend_robot_program_cmd");

  config.names.emplace_back(tf_prefix + "resend_robot_program/resend_robot_program_async_success");

  // payload stuff
  config.names.emplace_back(tf_prefix + "payload/mass");
  config.names.emplace_back(tf_prefix + "payload/cog.x");
  config.names.emplace_back(tf_prefix + "payload/cog.y");
  config.names.emplace_back(tf_prefix + "payload/cog.z");
  config.names.emplace_back(tf_prefix + "payload/payload_async_success");

  // FTS sensor
  config.names.emplace_back(tf_prefix + "zero_ftsensor/zero_ftsensor_cmd");
  config.names.emplace_back(tf_prefix + "zero_ftsensor/zero_ftsensor_async_success");

  // hand back control --> make UR-program return
  config.names.emplace_back(tf_prefix + "hand_back_control/hand_back_control_cmd");
  config.names.emplace_back(tf_prefix + "hand_back_control/hand_back_control_async_success");

  config.names.emplace_back(tf_prefix + "gpio/analog_output_domain_cmd");

  return config;
}

controller_interface::InterfaceConfiguration ur_controllers::GPIOController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  const std::string tf_prefix = params_.tf_prefix;

  // digital io
  for (size_t i = 0; i < 18; ++i) {
    config.names.emplace_back(tf_prefix + "gpio/digital_output_" + std::to_string(i));
  }

  for (size_t i = 0; i < 18; ++i) {
    config.names.emplace_back(tf_prefix + "gpio/digital_input_" + std::to_string(i));
  }

  // analog io
  for (size_t i = 0; i < 2; ++i) {
    config.names.emplace_back(tf_prefix + "gpio/standard_analog_output_" + std::to_string(i));
  }

  for (size_t i = 0; i < 2; ++i) {
    config.names.emplace_back(tf_prefix + "gpio/standard_analog_input_" + std::to_string(i));
  }

  for (size_t i = 0; i < 4; ++i) {
    config.names.emplace_back(tf_prefix + "gpio/analog_io_type_" + std::to_string(i));
  }

  // tool
  config.names.emplace_back(tf_prefix + "gpio/tool_mode");
  config.names.emplace_back(tf_prefix + "gpio/tool_output_voltage");
  config.names.emplace_back(tf_prefix + "gpio/tool_output_current");
  config.names.emplace_back(tf_prefix + "gpio/tool_temperature");

  for (size_t i = 0; i < 2; ++i) {
    config.names.emplace_back(tf_prefix + "gpio/tool_analog_input_" + std::to_string(i));
  }
  for (size_t i = 0; i < 2; ++i) {
    config.names.emplace_back(tf_prefix + "gpio/tool_analog_input_type_" + std::to_string(i));
  }

  // robot
  config.names.emplace_back(tf_prefix + "gpio/robot_mode");
  for (size_t i = 0; i < 4; ++i) {
    config.names.emplace_back(tf_prefix + "gpio/robot_status_bit_" + std::to_string(i));
  }

  // safety
  config.names.emplace_back(tf_prefix + "gpio/safety_mode");
  for (size_t i = 0; i < 11; ++i) {
    config.names.emplace_back(tf_prefix + "gpio/safety_status_bit_" + std::to_string(i));
  }
  config.names.emplace_back(tf_prefix + "system_interface/initialized");

  // program running
  config.names.emplace_back(tf_prefix + "gpio/program_running");

  return config;
}

controller_interface::return_type ur_controllers::GPIOController::update(const rclcpp::Time& /*time*/,
                                                                         const rclcpp::Duration& /*period*/)
{
  publishIO();
  publishToolData();
  publishRobotMode();
  publishSafetyMode();
  publishProgramRunning();
  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn
ur_controllers::GPIOController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  const auto logger = get_node()->get_logger();

  if (!param_listener_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }

  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();

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

controller_interface::CallbackReturn
ur_controllers::GPIOController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  while (state_interfaces_[StateInterfaces::INITIALIZED_FLAG].get_value() == 0.0) {
    RCLCPP_INFO(get_node()->get_logger(), "Waiting for system interface to initialize...");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  try {
    auto qos_latched = rclcpp::SystemDefaultsQoS();
    qos_latched.transient_local();
    // register publisher
    io_pub_ = get_node()->create_publisher<ur_msgs::msg::IOStates>("~/io_states", rclcpp::SystemDefaultsQoS());

    tool_data_pub_ =
        get_node()->create_publisher<ur_msgs::msg::ToolDataMsg>("~/tool_data", rclcpp::SystemDefaultsQoS());

    robot_mode_pub_ = get_node()->create_publisher<ur_dashboard_msgs::msg::RobotMode>("~/robot_mode", qos_latched);

    safety_mode_pub_ = get_node()->create_publisher<ur_dashboard_msgs::msg::SafetyMode>("~/safety_mode", qos_latched);

    program_state_pub_ = get_node()->create_publisher<std_msgs::msg::Bool>("~/robot_program_running", qos_latched);
    set_io_srv_ = get_node()->create_service<ur_msgs::srv::SetIO>(
        "~/set_io", std::bind(&GPIOController::setIO, this, std::placeholders::_1, std::placeholders::_2));
    set_analog_output_srv_ = get_node()->create_service<ur_msgs::srv::SetAnalogOutput>(
        "~/set_analog_output",
        std::bind(&GPIOController::setAnalogOutput, this, std::placeholders::_1, std::placeholders::_2));

    set_speed_slider_srv_ = get_node()->create_service<ur_msgs::srv::SetSpeedSliderFraction>(
        "~/set_speed_slider",
        std::bind(&GPIOController::setSpeedSlider, this, std::placeholders::_1, std::placeholders::_2));

    resend_robot_program_srv_ = get_node()->create_service<std_srvs::srv::Trigger>(
        "~/resend_robot_program",
        std::bind(&GPIOController::resendRobotProgram, this, std::placeholders::_1, std::placeholders::_2));

    hand_back_control_srv_ = get_node()->create_service<std_srvs::srv::Trigger>(
        "~/hand_back_control",
        std::bind(&GPIOController::handBackControl, this, std::placeholders::_1, std::placeholders::_2));

    set_payload_srv_ = get_node()->create_service<ur_msgs::srv::SetPayload>(
        "~/set_payload", std::bind(&GPIOController::setPayload, this, std::placeholders::_1, std::placeholders::_2));

    tare_sensor_srv_ = get_node()->create_service<std_srvs::srv::Trigger>(
        "~/zero_ftsensor",
        std::bind(&GPIOController::zeroFTSensor, this, std::placeholders::_1, std::placeholders::_2));
  } catch (...) {
    return LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
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

    RCLCPP_INFO(get_node()->get_logger(), "Setting digital output '%d' to state: '%1.0f'.", req->pin, req->state);

    if (!waitForAsyncCommand([&]() { return command_interfaces_[CommandInterfaces::IO_ASYNC_SUCCESS].get_value(); })) {
      RCLCPP_WARN(get_node()->get_logger(), "Could not verify that io was set. (This might happen when using the "
                                            "mocked interface)");
    }

    resp->success = static_cast<bool>(command_interfaces_[IO_ASYNC_SUCCESS].get_value());
    return resp->success;
  } else if (req->fun == req->FUN_SET_ANALOG_OUT && req->pin >= 0 && req->pin <= 2) {
    // io async success
    command_interfaces_[CommandInterfaces::IO_ASYNC_SUCCESS].set_value(ASYNC_WAITING);
    command_interfaces_[CommandInterfaces::ANALOG_OUTPUTS_CMD + req->pin].set_value(static_cast<double>(req->state));

    RCLCPP_INFO(get_node()->get_logger(), "Setting analog output '%d' to state: '%f'.", req->pin, req->state);

    if (!waitForAsyncCommand([&]() { return command_interfaces_[CommandInterfaces::IO_ASYNC_SUCCESS].get_value(); })) {
      RCLCPP_WARN(get_node()->get_logger(), "Could not verify that io was set. (This might happen when using the "
                                            "mocked interface)");
    }

    resp->success = static_cast<bool>(command_interfaces_[CommandInterfaces::IO_ASYNC_SUCCESS].get_value());
    return resp->success;
  } else if (req->fun == req->FUN_SET_TOOL_VOLTAGE) {
    command_interfaces_[CommandInterfaces::IO_ASYNC_SUCCESS].set_value(ASYNC_WAITING);
    command_interfaces_[CommandInterfaces::TOOL_VOLTAGE_CMD].set_value(static_cast<double>(req->state));

    RCLCPP_INFO(get_node()->get_logger(), "Setting tool voltage to: '%1.0f'.", req->state);

    if (!waitForAsyncCommand([&]() { return command_interfaces_[CommandInterfaces::IO_ASYNC_SUCCESS].get_value(); })) {
      RCLCPP_WARN(get_node()->get_logger(), "Could not verify that io was set. (This might happen when using the "
                                            "mocked interface)");
    }

    resp->success = static_cast<bool>(command_interfaces_[CommandInterfaces::IO_ASYNC_SUCCESS].get_value());
    return resp->success;
  } else {
    resp->success = false;
    return false;
  }
}

bool GPIOController::setAnalogOutput(ur_msgs::srv::SetAnalogOutput::Request::SharedPtr req,
                                     ur_msgs::srv::SetAnalogOutput::Response::SharedPtr resp)
{
  std::string domain_string = "UNKNOWN";
  switch (req->data.domain) {
    case ur_msgs::msg::Analog::CURRENT:
      domain_string = "CURRENT";
      break;
    case ur_msgs::msg::Analog::VOLTAGE:
      domain_string = "VOLTAGE";
      break;
    default:
      RCLCPP_ERROR(get_node()->get_logger(), "Domain must be either 0 (CURRENT) or 1 (VOLTAGE)");
      resp->success = false;
      return false;
  }

  if (req->data.pin < 0 || req->data.pin > 1) {
    RCLCPP_ERROR(get_node()->get_logger(), "Invalid pin selected. Only pins 0 and 1 are allowed.");
    resp->success = false;
    return false;
  }

  command_interfaces_[CommandInterfaces::IO_ASYNC_SUCCESS].set_value(ASYNC_WAITING);
  command_interfaces_[CommandInterfaces::ANALOG_OUTPUTS_CMD + req->data.pin].set_value(
      static_cast<float>(req->data.state));
  command_interfaces_[CommandInterfaces::ANALOG_OUTPUTS_DOMAIN].set_value(static_cast<double>(req->data.domain));

  RCLCPP_INFO(get_node()->get_logger(), "Setting analog output '%d' to state: '%f' in domain %s.", req->data.pin,
              req->data.state, domain_string.c_str());

  if (!waitForAsyncCommand([&]() { return command_interfaces_[CommandInterfaces::IO_ASYNC_SUCCESS].get_value(); })) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not verify that io was set. (This might happen when using the "
                                          "mocked interface)");
  }

  resp->success = static_cast<bool>(command_interfaces_[IO_ASYNC_SUCCESS].get_value());
  return resp->success;
}

bool GPIOController::setSpeedSlider(ur_msgs::srv::SetSpeedSliderFraction::Request::SharedPtr req,
                                    ur_msgs::srv::SetSpeedSliderFraction::Response::SharedPtr resp)
{
  if (req->speed_slider_fraction >= 0.01 && req->speed_slider_fraction <= 1.0) {
    RCLCPP_INFO(get_node()->get_logger(), "Setting speed slider to %.2f%%.", req->speed_slider_fraction * 100.0);
    // reset success flag
    command_interfaces_[CommandInterfaces::TARGET_SPEED_FRACTION_ASYNC_SUCCESS].set_value(ASYNC_WAITING);
    // set commanding value for speed slider
    command_interfaces_[CommandInterfaces::TARGET_SPEED_FRACTION_CMD].set_value(
        static_cast<double>(req->speed_slider_fraction));

    if (!waitForAsyncCommand([&]() {
          return command_interfaces_[CommandInterfaces::TARGET_SPEED_FRACTION_ASYNC_SUCCESS].get_value();
        })) {
      RCLCPP_WARN(get_node()->get_logger(), "Could not verify that target speed fraction was set. (This might happen "
                                            "when using the mocked interface)");
    }
    resp->success =
        static_cast<bool>(command_interfaces_[CommandInterfaces::TARGET_SPEED_FRACTION_ASYNC_SUCCESS].get_value());
  } else {
    RCLCPP_WARN(get_node()->get_logger(), "The desired speed slider fraction must be within range (0; 1.0]. Request "
                                          "ignored.");
    resp->success = false;
    return false;
  }
  return true;
}

bool GPIOController::resendRobotProgram(std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
                                        std_srvs::srv::Trigger::Response::SharedPtr resp)
{
  // reset success flag
  command_interfaces_[CommandInterfaces::RESEND_ROBOT_PROGRAM_ASYNC_SUCCESS].set_value(ASYNC_WAITING);
  // call the service in the hardware
  command_interfaces_[CommandInterfaces::RESEND_ROBOT_PROGRAM_CMD].set_value(1.0);

  if (!waitForAsyncCommand(
          [&]() { return command_interfaces_[CommandInterfaces::RESEND_ROBOT_PROGRAM_ASYNC_SUCCESS].get_value(); })) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not verify that program was sent. (This might happen when using the "
                                          "mocked interface)");
  }
  resp->success =
      static_cast<bool>(command_interfaces_[CommandInterfaces::RESEND_ROBOT_PROGRAM_ASYNC_SUCCESS].get_value());

  if (resp->success) {
    RCLCPP_INFO(get_node()->get_logger(), "Successfully resent robot program");
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not resend robot program");
    return false;
  }

  return true;
}

bool GPIOController::handBackControl(std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
                                     std_srvs::srv::Trigger::Response::SharedPtr resp)
{
  // reset success flag
  command_interfaces_[CommandInterfaces::HAND_BACK_CONTROL_ASYNC_SUCCESS].set_value(ASYNC_WAITING);
  // call the service in the hardware
  command_interfaces_[CommandInterfaces::HAND_BACK_CONTROL_CMD].set_value(1.0);

  if (!waitForAsyncCommand(
          [&]() { return command_interfaces_[CommandInterfaces::HAND_BACK_CONTROL_ASYNC_SUCCESS].get_value(); })) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not verify that hand_back_control was correctly triggered. (This "
                                          "might happen when using the mocked interface)");
  }
  resp->success =
      static_cast<bool>(command_interfaces_[CommandInterfaces::HAND_BACK_CONTROL_ASYNC_SUCCESS].get_value());

  if (resp->success) {
    RCLCPP_INFO(get_node()->get_logger(), "Deactivated control");
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not deactivate control");
    return false;
  }

  return true;
}

bool GPIOController::setPayload(const ur_msgs::srv::SetPayload::Request::SharedPtr req,
                                ur_msgs::srv::SetPayload::Response::SharedPtr resp)
{
  // reset success flag
  command_interfaces_[CommandInterfaces::PAYLOAD_ASYNC_SUCCESS].set_value(ASYNC_WAITING);

  command_interfaces_[CommandInterfaces::PAYLOAD_MASS].set_value(req->mass);
  command_interfaces_[CommandInterfaces::PAYLOAD_COG_X].set_value(req->center_of_gravity.x);
  command_interfaces_[CommandInterfaces::PAYLOAD_COG_Y].set_value(req->center_of_gravity.y);
  command_interfaces_[CommandInterfaces::PAYLOAD_COG_Z].set_value(req->center_of_gravity.z);

  if (!waitForAsyncCommand(
          [&]() { return command_interfaces_[CommandInterfaces::PAYLOAD_ASYNC_SUCCESS].get_value(); })) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not verify that payload was set. (This might happen when using the "
                                          "mocked interface)");
  }

  resp->success = static_cast<bool>(command_interfaces_[CommandInterfaces::PAYLOAD_ASYNC_SUCCESS].get_value());

  if (resp->success) {
    RCLCPP_INFO(get_node()->get_logger(), "Payload has been set successfully");
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not set the payload");
    return false;
  }

  return true;
}

bool GPIOController::zeroFTSensor(std_srvs::srv::Trigger::Request::SharedPtr /*req*/,
                                  std_srvs::srv::Trigger::Response::SharedPtr resp)
{
  // reset success flag
  command_interfaces_[CommandInterfaces::ZERO_FTSENSOR_ASYNC_SUCCESS].set_value(ASYNC_WAITING);
  // call the service in the hardware
  command_interfaces_[CommandInterfaces::ZERO_FTSENSOR_CMD].set_value(1.0);

  if (!waitForAsyncCommand(
          [&]() { return command_interfaces_[CommandInterfaces::ZERO_FTSENSOR_ASYNC_SUCCESS].get_value(); })) {
    RCLCPP_WARN(get_node()->get_logger(), "Could not verify that FTS was zeroed. (This might happen when using the "
                                          "mocked interface)");
  }

  resp->success = static_cast<bool>(command_interfaces_[CommandInterfaces::ZERO_FTSENSOR_ASYNC_SUCCESS].get_value());

  if (resp->success) {
    RCLCPP_INFO(get_node()->get_logger(), "Successfully zeroed the force torque sensor");
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Could not zero the force torque sensor");
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

bool GPIOController::waitForAsyncCommand(std::function<double(void)> get_value)
{
  const auto maximum_retries = params_.check_io_successfull_retries;
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

PLUGINLIB_EXPORT_CLASS(ur_controllers::GPIOController, controller_interface::ControllerInterface)
