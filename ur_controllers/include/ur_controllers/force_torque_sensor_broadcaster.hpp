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

#ifndef UR_CONTROLLERS__FORCE_TORQUE_SENSOR_BROADCASTER_HPP_
#define UR_CONTROLLERS__FORCE_TORQUE_SENSOR_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

namespace ur_controllers
{
using FbkType = geometry_msgs::msg::WrenchStamped;

struct FTStateControllerParams
{
  FTStateControllerParams()
  {
    sensor_name_ = "";
    state_interfaces_names_ = {};
    frame_id = "";
    topic_name = "";
  }

  std::string sensor_name_;
  std::vector<std::string> state_interfaces_names_;
  std::string frame_id;
  std::string topic_name;
};

/**
 * \brief Force torque state controller.
 *
 * This class forwards the command signal down to a set of joints
 * on the specified interface.
 *
 * \param topic_name Name of the topic to publish WrenchStamped msg
 * \param frame_id Frame of the ft sensor
 * \param sensor_name Names of the sensor which data should be published .
 * \param state_interface_names Names of the state interfaces in the sensor.
 *
 * Publishes to:
 * - \b topic_name (geometry_msg::msg::WrenchStamped) : Sensor data.
 */
class ForceTorqueStateBroadcaster : public controller_interface::ControllerInterface
{
public:
  ForceTorqueStateBroadcaster();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::return_type update() override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::return_type init(const std::string& controller_name) override;

protected:
  // param storage
  FTStateControllerParams fts_params_;
  std::shared_ptr<rclcpp::Publisher<FbkType>> wrench_state_publisher_;
  geometry_msgs::msg::WrenchStamped wrench_state_msg_;
};
}  // namespace ur_controllers

#endif  // UR_CONTROLLERS__FORCE_TORQUE_SENSOR_BROADCASTER_HPP_
