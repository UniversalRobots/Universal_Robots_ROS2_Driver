// Copyright 2026, Arthur Haffemayer
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

  /*!\file
   *
   * \author  Arthur Haffemayer <arthur.haffemayer@gmail.com>
   * \date    2026-08-03
   *
   */


#include "ur_controllers/ur_robot_mode_broadcaster.hpp"

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcpputils/split.hpp"
#include "rcutils/logging_macros.h"

namespace ur_controllers
{
URRobotModeBroadcaster::URRobotModeBroadcaster()
{
}

controller_interface::CallbackReturn URRobotModeBroadcaster::on_init()
{
  try {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ur_robot_mode_broadcaster::ParamListener>(get_node());
    params_ = param_listener_->get_params();

    RCLCPP_INFO(get_node()->get_logger(), "Loading UR URRobotModeBroadcaster with tf_prefix: %s",
                params_.tf_prefix.c_str());
  } catch (std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration URRobotModeBroadcaster::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{ controller_interface::interface_configuration_type::NONE };
}

controller_interface::InterfaceConfiguration URRobotModeBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  const std::string tf_prefix = params_.tf_prefix;
  config.names.push_back(tf_prefix + "gpio/robot_control_mode");
  return config;
}

controller_interface::CallbackReturn
URRobotModeBroadcaster::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (!param_listener_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Error encountered during init");
    return controller_interface::CallbackReturn::ERROR;
  }

  // update the dynamic map parameters
  param_listener_->refresh_dynamic_parameters();

  // get parameters from the listener in case they were updated
  params_ = param_listener_->get_params();

  try {
    robot_mode_publisher_ = std::make_shared<realtime_tools::RealtimePublisher<ur_msgs::msg::RobotModeDataMsg>>(
        get_node()->create_publisher<ur_msgs::msg::RobotModeDataMsg>("~/robot_mode_data", rclcpp::SystemDefaultsQoS()));
  } catch (const std::exception& e) {
    // get_node() may throw, logging raw here
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }
  return controller_interface::CallbackReturn::SUCCESS;
}


controller_interface::CallbackReturn
URRobotModeBroadcaster::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type URRobotModeBroadcaster::update(const rclcpp::Time& /*time*/,
                                                                       const rclcpp::Duration& /*period*/)
{
  robot_mode_msg_.control_mode = static_cast<uint8_t>(
      state_interfaces_[0].get_optional().value_or(ur_msgs::msg::RobotModeDataMsg::CONTROL_MODE_UNKNOWN));

  // publish
  robot_mode_publisher_->try_publish(robot_mode_msg_);
  
  return controller_interface::return_type::OK;
}

controller_interface::CallbackReturn
URRobotModeBroadcaster::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/)
{
  robot_mode_publisher_.reset();
  return controller_interface::CallbackReturn::SUCCESS;
}

} // namespace ur_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ur_controllers::URRobotModeBroadcaster, controller_interface::ControllerInterface)

