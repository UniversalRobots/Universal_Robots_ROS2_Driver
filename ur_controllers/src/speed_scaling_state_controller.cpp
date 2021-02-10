// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2019 FZI Forschungszentrum Informatik
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
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Marvin Große Besselmann grosse@fzi.de
 * \date    2021-02-10
 *
 */
//----------------------------------------------------------------------

#include "ur_controllers/speed_scaling_state_controller.h"

#include <stddef.h>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/qos_event.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcpputils/split.hpp"
#include "rcutils/logging_macros.h"
#include "std_msgs/msg/header.hpp"

namespace rclcpp_lifecycle
{
class State;
}  // namespace rclcpp_lifecycle

namespace ur_controllers
{
const auto kUninitializedValue = std::numeric_limits<double>::quiet_NaN();
using hardware_interface::HW_IF_EFFORT;
using hardware_interface::HW_IF_POSITION;
using hardware_interface::HW_IF_VELOCITY;

SpeedScalingStateController::SpeedScalingStateController()
{
}

controller_interface::InterfaceConfiguration SpeedScalingStateController::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{ controller_interface::interface_configuration_type::NONE };
}

controller_interface::InterfaceConfiguration SpeedScalingStateController::state_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{ controller_interface::interface_configuration_type::ALL };
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SpeedScalingStateController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  try
  {
    speed_scaling_state_publisher_ =
        get_node()->create_publisher<std_msgs::msg::Float64>("speed_scaling_factor", rclcpp::SystemDefaultsQoS());
  }
  catch (const std::exception& e)
  {
    // get_node() may throw, logging raw here
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SpeedScalingStateController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (!init_joint_data())
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
SpeedScalingStateController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

template <typename T>
bool has_any_key(const std::unordered_map<std::string, T>& map, const std::vector<std::string>& keys)
{
  bool found_key = false;
  for (const auto& key_item : map)
  {
    const auto& key = key_item.first;
    if (std::find(keys.cbegin(), keys.cend(), key) != keys.cend())
    {
      found_key = true;
      break;
    }
  }
  return found_key;
}

bool SpeedScalingStateController::init_joint_data()
{
  // loop in reverse order, this maintains the order of values at retrieval time
  for (auto si = state_interfaces_.crbegin(); si != state_interfaces_.crend(); si++)
  {
    // initialize map if name is new
    if (name_if_value_mapping_.count(si->get_name()) == 0)
    {
      name_if_value_mapping_[si->get_name()] = {};
    }
    // add interface name
    name_if_value_mapping_[si->get_name()][si->get_interface_name()] = kUninitializedValue;
  }

  // filter state interfaces that have a speed scaling factor
  // the rest will be ignored for this message
  for (const auto& name_ifv : name_if_value_mapping_)
  {
    const auto& interfaces_and_values = name_ifv.second;
    if (has_any_key(interfaces_and_values, { "speed_scaling_factor" }))
    {
      joint_names_.push_back(name_ifv.first);
    }
  }

  return true;
}

double get_value(const std::unordered_map<std::string, std::unordered_map<std::string, double>>& map,
                 const std::string& name, const std::string& interface_name)
{
  const auto& interfaces_and_values = map.at(name);
  const auto interface_and_value = interfaces_and_values.find(interface_name);
  if (interface_and_value != interfaces_and_values.cend())
  {
    return interface_and_value->second;
  }
  else
  {
    return kUninitializedValue;
  }
}

controller_interface::return_type SpeedScalingStateController::update()
{
  for (const auto& state_interface : state_interfaces_)
  {
    name_if_value_mapping_[state_interface.get_name()][state_interface.get_interface_name()] =
        state_interface.get_value();
    RCLCPP_DEBUG(get_node()->get_logger(), "%s/%s: %f\n", state_interface.get_name().c_str(),
                 state_interface.get_interface_name().c_str(), state_interface.get_value());
  }

  for (auto i = 0ul; i < joint_names_.size(); ++i)
  {
    speed_scaling_state_msg_.data = get_value(name_if_value_mapping_, joint_names_[i], "speed_scaling_factor");
  }

  // publish
  speed_scaling_state_publisher_->publish(speed_scaling_state_msg_);

  return controller_interface::return_type::SUCCESS;
}

}  // namespace ur_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ur_controllers::SpeedScalingStateController, controller_interface::ControllerInterface)
