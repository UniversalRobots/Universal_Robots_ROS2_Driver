//
// Created by livanov93 on 12/12/20.
//

#ifndef UR_CONTROLLERS_FORCE_TORQUE_SENSOR_CONTROLLER_H
#define UR_CONTROLLERS_FORCE_TORQUE_SENSOR_CONTROLLER_H

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <controller_interface/controller_interface.hpp>
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

#include <geometry_msgs/msg/wrench_stamped.hpp>

namespace ur_controllers
{
class ForceTorqueStateController : public controller_interface::ControllerInterface
{
public:
  ForceTorqueStateController();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

protected:
  bool init_sensor_data();
  void init_sensor_state_msg();

  //  we store the name of values per axis - compatible interfaces
  std::vector<std::string> axis_val_names_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::WrenchStamped>> wrench_state_publisher_;
  geometry_msgs::msg::WrenchStamped wrench_state_msg_;
};
}  // namespace ur_controllers

#endif  // UR_CONTROLLERS_FORCE_TORQUE_SENSOR_CONTROLLER_H
