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
 * \date    2021-02-18
 *
 */
//----------------------------------------------------------------------
#ifndef UR_CONTROLLERS__SCALED_JOINT_TRAJECTORY_CONTROLLER_HPP_
#define UR_CONTROLLERS__SCALED_JOINT_TRAJECTORY_CONTROLLER_HPP_

#include "angles/angles.h"
#include "joint_trajectory_controller/joint_trajectory_controller.hpp"
#include "joint_trajectory_controller/trajectory.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"

namespace ur_controllers
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ScaledJointTrajectoryController : public joint_trajectory_controller::JointTrajectoryController
{
public:
  ScaledJointTrajectoryController() = default;
  ~ScaledJointTrajectoryController() override = default;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

protected:
  struct TimeData
  {
    TimeData() : time(0.0), period(rclcpp::Duration::from_nanoseconds(0.0)), uptime(0.0)
    {
    }
    rclcpp::Time time;
    rclcpp::Duration period;
    rclcpp::Time uptime;
  };

private:
  double scaling_factor_;
  realtime_tools::RealtimeBuffer<TimeData> time_data_;
};
}  // namespace ur_controllers

#endif  // UR_CONTROLLERS__SCALED_JOINT_TRAJECTORY_CONTROLLER_HPP_
