// Copyright (c) 2021, PickNik, Inc.
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

#include <string>
#include <vector>

#include "ur_controllers/force_torque_sensor_broadcaster.hpp"

namespace ur_controllers
{
ForceTorqueStateBroadcaster::ForceTorqueStateBroadcaster()
  : controller_interface::ControllerInterface(), wrench_state_publisher_(nullptr)
{
}

CallbackReturn ForceTorqueStateBroadcaster::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("state_interface_names", {});
    auto_declare<std::string>("sensor_name", "");
    auto_declare<std::string>("topic_name", "");
    auto_declare<std::string>("frame_id", "");
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
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

controller_interface::return_type
ur_controllers::ForceTorqueStateBroadcaster::update(const rclcpp::Time& time, const rclcpp::Duration& /*period*/)
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

  wrench_state_msg_.header.stamp = time;
  wrench_state_msg_.header.frame_id = fts_params_.frame_id;

  // update wrench state message
  wrench_state_msg_.wrench.set__force(f_vec);
  wrench_state_msg_.wrench.set__torque(t_vec);

  // publish
  wrench_state_publisher_->publish(wrench_state_msg_);

  return controller_interface::return_type::OK;
}

CallbackReturn ForceTorqueStateBroadcaster::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
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
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn ForceTorqueStateBroadcaster::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn ForceTorqueStateBroadcaster::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

}  // namespace ur_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ur_controllers::ForceTorqueStateBroadcaster, controller_interface::ControllerInterface)
