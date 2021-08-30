// Copyright 2019, FZI Forschungszentrum Informatik
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
 * \author  Marvin Große Besselmann grosse@fzi.de
 * \date    2021-02-10
 *
 */
//----------------------------------------------------------------------

#include "ur_controllers/speed_scaling_state_broadcaster.hpp"

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/qos_event.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcpputils/split.hpp"
#include "rcutils/logging_macros.h"

namespace rclcpp_lifecycle
{
class State;
}  // namespace rclcpp_lifecycle

namespace ur_controllers
{
SpeedScalingStateBroadcaster::SpeedScalingStateBroadcaster()
{
}

controller_interface::InterfaceConfiguration SpeedScalingStateBroadcaster::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{ controller_interface::interface_configuration_type::NONE };
}

controller_interface::InterfaceConfiguration SpeedScalingStateBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names.push_back("speed_scaling/speed_scaling_factor");
  return config;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SpeedScalingStateBroadcaster::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  auto_declare<double>("state_publish_rate", 100.0);

  if (!node_->get_parameter("state_publish_rate", publish_rate_)) {
    RCLCPP_INFO(get_node()->get_logger(), "Parameter 'state_publish_rate' not set");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  } else {
    RCLCPP_INFO(get_node()->get_logger(), "Publisher rate set to : %.1f Hz", publish_rate_);
  }

  try {
    speed_scaling_state_publisher_ =
        get_node()->create_publisher<std_msgs::msg::Float64>("~/speed_scaling", rclcpp::SystemDefaultsQoS());
  } catch (const std::exception& e) {
    // get_node() may throw, logging raw here
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SpeedScalingStateBroadcaster::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  last_publish_time_ = node_->now();
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SpeedScalingStateBroadcaster::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::return_type SpeedScalingStateBroadcaster::update()
{
  if (publish_rate_ > 0.0 && (node_->now() - last_publish_time_) > rclcpp::Duration(1.0 / publish_rate_, 0.0)) {
    // Speed scaling is the only interface of the controller
    speed_scaling_state_msg_.data = state_interfaces_[0].get_value() * 100.0;

    // publish
    speed_scaling_state_publisher_->publish(speed_scaling_state_msg_);
    last_publish_time_ = node_->now();
  }
  return controller_interface::return_type::OK;
}

}  // namespace ur_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ur_controllers::SpeedScalingStateBroadcaster, controller_interface::ControllerInterface)
