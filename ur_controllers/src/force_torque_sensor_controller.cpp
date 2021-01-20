//
// Created by livanov93 on 12/12/20.
//

#include "ur_controllers/force_torque_sensor_controller.h"

namespace ur_controllers
{
ForceTorqueStateController::ForceTorqueStateController() : controller_interface::ControllerInterface()
{
}

controller_interface::return_type ForceTorqueStateController::init(const std::string& controller_name)
{
  auto ret = ControllerInterface::init(controller_name);
  if (ret != controller_interface::return_type::SUCCESS)
  {
    return ret;
  }

  return controller_interface::return_type::SUCCESS;
}

controller_interface::InterfaceConfiguration ForceTorqueStateController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::NONE;
  return config;
}

controller_interface::InterfaceConfiguration ForceTorqueStateController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  // TODO solve this hardcoded naming
  config.names.emplace_back("tcp_fts_sensor/fx");
  config.names.emplace_back("tcp_fts_sensor/fy");
  config.names.emplace_back("tcp_fts_sensor/fz");
  config.names.emplace_back("tcp_fts_sensor/tx");
  config.names.emplace_back("tcp_fts_sensor/ty");
  config.names.emplace_back("tcp_fts_sensor/tz");
  return config;
}

controller_interface::return_type ur_controllers::ForceTorqueStateController::update()
{
  geometry_msgs::msg::Vector3 fVec;
  geometry_msgs::msg::Vector3 tVec;

  // TODO remove these hardcoded names and create better string filtering within state_interfaces_
  for (const auto& state_interface : state_interfaces_)
  {
    if (state_interface.get_interface_name() == "fx")
    {
      fVec.set__x(state_interface.get_value());
      continue;
    }

    if (state_interface.get_interface_name() == "fy")
    {
      fVec.set__y(state_interface.get_value());
      continue;
    }

    if (state_interface.get_interface_name() == "fz")
    {
      fVec.set__z(state_interface.get_value());
      continue;
    }

    if (state_interface.get_interface_name() == "tx")
    {
      tVec.set__x(state_interface.get_value());
      continue;
    }

    if (state_interface.get_interface_name() == "ty")
    {
      tVec.set__y(state_interface.get_value());
      continue;
    }

    if (state_interface.get_interface_name() == "tz")
    {
      tVec.set__z(state_interface.get_value());
      continue;
    }
  }

  // TODO set frame_id as parameter --> it includes tf listener within controller
  wrench_state_msg_.header.stamp = get_node()->get_clock()->now();
  wrench_state_msg_.header.frame_id = "tool0";

  // update wrench state message
  wrench_state_msg_.wrench.set__force(fVec);
  wrench_state_msg_.wrench.set__torque(tVec);

  // publish
  wrench_state_publisher_->publish(wrench_state_msg_);

  return controller_interface::return_type::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ForceTorqueStateController::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  try
  {
    // TODO make topic name a parameter
    wrench_state_publisher_ =
        get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>("ft_data", rclcpp::SystemDefaultsQoS());
  }
  catch (...)
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ForceTorqueStateController::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (!init_sensor_data())
  {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
  }

  init_sensor_state_msg();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ForceTorqueStateController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
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

bool ForceTorqueStateController::init_sensor_data()
{
  bool has_fts_sensor = false;
  // loop in reverse order, this maintains the order of values at retrieval time
  for (auto si = state_interfaces_.crbegin(); si != state_interfaces_.crend(); si++)
  {
    // TODO remove hardcoded naming
    if (si->get_name() == "tcp_fts_sensor")
    {
      has_fts_sensor = true;
      axis_val_names_.push_back(si->get_interface_name());
    }
  }
  return has_fts_sensor;
}

void ForceTorqueStateController::init_sensor_state_msg()
{
  // default initialization for wrenchstamped state message
}

}  // namespace ur_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ur_controllers::ForceTorqueStateController, controller_interface::ControllerInterface)
