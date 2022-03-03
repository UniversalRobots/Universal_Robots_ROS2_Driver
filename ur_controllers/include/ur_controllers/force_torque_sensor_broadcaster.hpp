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

#ifndef UR_CONTROLLERS__FORCE_TORQUE_SENSOR_BROADCASTER_HPP_
#define UR_CONTROLLERS__FORCE_TORQUE_SENSOR_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"

namespace ur_controllers
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
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

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  CallbackReturn on_init() override;

protected:
  // param storage
  FTStateControllerParams fts_params_;
  std::shared_ptr<rclcpp::Publisher<FbkType>> wrench_state_publisher_;
  geometry_msgs::msg::WrenchStamped wrench_state_msg_;
};
}  // namespace ur_controllers

#endif  // UR_CONTROLLERS__FORCE_TORQUE_SENSOR_BROADCASTER_HPP_
