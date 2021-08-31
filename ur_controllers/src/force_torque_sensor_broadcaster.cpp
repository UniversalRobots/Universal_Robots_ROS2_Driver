// Copyright (c) 2021, PickNik, Inc.
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

#include <string>
#include <vector>

#include "ur_controllers/force_torque_sensor_broadcaster.hpp"

namespace ur_controllers
{
ForceTorqueStateBroadcaster::ForceTorqueStateBroadcaster()
  : controller_interface::ControllerInterface(), wrench_state_publisher_(nullptr)
{
}

controller_interface::return_type ForceTorqueStateBroadcaster::init(const std::string& controller_name)
{
  auto ret = ControllerInterface::init(controller_name);
  if (ret != controller_interface::return_type::OK) {
    return ret;
  }

  try {
    auto_declare<std::vector<std::string>>("state_interface_names", {});
    auto_declare<std::string>("sensor_name", "");
    auto_declare<std::string>("topic_name", "");
    auto_declare<std::string>("frame_id", "");
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::return_type::ERROR;
  }

  return controller_interface::return_type::OK;
}

controller_interface::InterfaceConfiguration ForceTorqueStateBroadcaster::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::InterfaceConfiguration ForceTorqueStateBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  for (const auto& si : fts_params_.state_interfaces_names_) {
    config.names.push_back(fts_params_.sensor_name_ + "/" + si);
  }

  return config;
}

controller_interface::return_type ur_controllers::ForceTorqueStateBroadcaster::update()
{
  geometry_msgs::msg::Vector3 f_vec;
  geometry_msgs::msg::Vector3 t_vec;

  if (state_interfaces_.size() != fts_params_.state_interfaces_names_.size())
    return controller_interface::return_type::ERROR;

  for (auto index = 0ul; index < state_interfaces_.size(); ++index) {
    switch (index) {
      case 0:
        f_vec.set__x(state_interfaces_[index].get_value());
        break;
      case 1:
        f_vec.set__y(state_interfaces_[index].get_value());
        break;
      case 2:
        f_vec.set__z(state_interfaces_[index].get_value());
        break;
      case 3:
        t_vec.set__x(state_interfaces_[index].get_value());
        break;
      case 4:
        t_vec.set__y(state_interfaces_[index].get_value());
        break;
      case 5:
        t_vec.set__z(state_interfaces_[index].get_value());
        break;
      default:
        break;
    }
  }

  wrench_state_msg_.header.stamp = get_node()->get_clock()->now();
  wrench_state_msg_.header.frame_id = fts_params_.frame_id;

  // update wrench state message
  wrench_state_msg_.wrench.set__force(f_vec);
  wrench_state_msg_.wrench.set__torque(t_vec);

  // publish
  wrench_state_publisher_->publish(wrench_state_msg_);

  return controller_interface::return_type::OK;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ForceTorqueStateBroadcaster::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  fts_params_.state_interfaces_names_ = node_->get_parameter("state_interface_names").as_string_array();

  if (fts_params_.state_interfaces_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'state_interface_names' parameter was empty");
    return CallbackReturn::ERROR;
  }

  fts_params_.sensor_name_ = node_->get_parameter("sensor_name").as_string();
  if (fts_params_.sensor_name_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'sensor_name' parameter was empty");
    return CallbackReturn::ERROR;
  }

  fts_params_.topic_name = node_->get_parameter("topic_name").as_string();
  if (fts_params_.topic_name.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'topic_name' parameter was empty");
    return CallbackReturn::ERROR;
  }

  fts_params_.frame_id = node_->get_parameter("frame_id").as_string();
  if (fts_params_.frame_id.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'frame_id' parameter was empty");
    return CallbackReturn::ERROR;
  }

  try {
    // register ft sensor data publisher
    wrench_state_publisher_ = get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
        fts_params_.topic_name, rclcpp::SystemDefaultsQoS());
  } catch (...) {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ForceTorqueStateBroadcaster::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ForceTorqueStateBroadcaster::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

}  // namespace ur_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ur_controllers::ForceTorqueStateBroadcaster, controller_interface::ControllerInterface)
