//
// Created by livanov93 on 12/12/20.
//

#include "ur_controllers/force_torque_sensor_controller.h"

namespace ur_controllers
{
ForceTorqueStateController::ForceTorqueStateController()
  : controller_interface::ControllerInterface(), wrench_state_publisher_(nullptr)
{
}

controller_interface::return_type ForceTorqueStateController::init(const std::string& controller_name)
{
  auto ret = ControllerInterface::init(controller_name);
  if (ret != controller_interface::return_type::SUCCESS)
  {
    return ret;
  }

  try
  {
    auto node = get_node();
    node->declare_parameter<std::vector<std::string>>("state_interface_names", {});
    node->declare_parameter<std::string>("sensor_name", "");
    node->declare_parameter<std::string>("topic_name", "");
    node->declare_parameter<std::string>("frame_id", "");
  }
  catch (const std::exception& e)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return controller_interface::return_type::ERROR;
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
  for (const auto& si : fts_params_.state_interfaces_names_)
  {
    config.names.push_back(fts_params_.sensor_name_ + "/" + si);
  }

  return config;
}

controller_interface::return_type ur_controllers::ForceTorqueStateController::update()
{
  geometry_msgs::msg::Vector3 fVec;
  geometry_msgs::msg::Vector3 tVec;

  if (state_interfaces_.size() != fts_params_.state_interfaces_names_.size())
    return controller_interface::return_type::ERROR;

  for (auto index = 0ul; index < state_interfaces_.size(); ++index)
  {
    switch (index)
    {
      case 0:
        fVec.set__x(state_interfaces_[index].get_value());
        break;
      case 1:
        fVec.set__y(state_interfaces_[index].get_value());
        break;
      case 2:
        fVec.set__z(state_interfaces_[index].get_value());
        break;
      case 3:
        tVec.set__x(state_interfaces_[index].get_value());
        break;
      case 4:
        tVec.set__y(state_interfaces_[index].get_value());
        break;
      case 5:
        tVec.set__z(state_interfaces_[index].get_value());
        break;
      default:
        break;
    }
  }

  wrench_state_msg_.header.stamp = get_node()->get_clock()->now();
  wrench_state_msg_.header.frame_id = fts_params_.frame_id;

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
  fts_params_.state_interfaces_names_ = node_->get_parameter("state_interface_names").as_string_array();

  if (fts_params_.state_interfaces_names_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'state_interface_names' parameter was empty");
    return CallbackReturn::ERROR;
  }

  fts_params_.sensor_name_ = node_->get_parameter("sensor_name").as_string();
  if (fts_params_.sensor_name_.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'sensor_name' parameter was empty");
    return CallbackReturn::ERROR;
  }

  fts_params_.topic_name = node_->get_parameter("topic_name").as_string();
  if (fts_params_.topic_name.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'topic_name' parameter was empty");
    return CallbackReturn::ERROR;
  }

  fts_params_.frame_id = node_->get_parameter("frame_id").as_string();
  if (fts_params_.frame_id.empty())
  {
    RCLCPP_ERROR(get_node()->get_logger(), "'frame_id' parameter was empty");
    return CallbackReturn::ERROR;
  }

  try
  {
    // register ft sensor data publisher
    wrench_state_publisher_ = get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>(
        fts_params_.topic_name, rclcpp::SystemDefaultsQoS());
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
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ForceTorqueStateController::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

}  // namespace ur_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ur_controllers::ForceTorqueStateController, controller_interface::ControllerInterface)
