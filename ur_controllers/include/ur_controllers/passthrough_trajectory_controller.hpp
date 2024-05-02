// Copyright 2024, Universal Robots A/S
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

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Jacob Larsen jala@universal-robots.com
 * \date    2024-03-11
 *
 *
 *
 *
 */
//----------------------------------------------------------------------

#ifndef UR_CONTROLLERS__PASSTHROUGH_TRAJECTORY_CONTROLLER_HPP_
#define UR_CONTROLLERS__PASSTHROUGH_TRAJECTORY_CONTROLLER_HPP_

#include <stdint.h>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_action/server.hpp"
#include "rclcpp_action/create_server.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/server_goal_handle.hpp"

#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/action/joint_trajectory.hpp"

#include "passthrough_trajectory_controller_parameters.hpp"

namespace ur_controllers
{
enum CommandInterfaces
{
  PASSTHROUGH_TRAJECTORY_TRANSFER_STATE = 0,
  PASSTHROUGH_POINT_WRITTEN = 1,
  PASSTHROUGH_TRAJECTORY_NUMBER_OF_POINTS = 2,
  PASSTHROUGH_TRAJECTORY_CANCEL = 3,
  PASSTHROUGH_TRAJECTORY_DIMENSIONS = 4,
  PASSTHROUGH_TRAJECTORY_POSITIONS_ = 5,
  PASSTHROUGH_TRAJECTORY_VELOCITIES_ = 11,
  PASSTHROUGH_TRAJECTORY_ACCELERATIONS_ = 17,
  PASSTHROUGH_TRAJECTORY_TIME_FROM_START = 23
};

class PassthroughTrajectoryController : public controller_interface::ControllerInterface
{
public:
  PassthroughTrajectoryController() = default;
  ~PassthroughTrajectoryController() override = default;

  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;

  controller_interface::return_type update(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  void start_action_server(void);

  std::shared_ptr<passthrough_trajectory_controller::ParamListener> passthrough_param_listener_;
  passthrough_trajectory_controller::Params passthrough_params_;

  rclcpp_action::Server<control_msgs::action::JointTrajectory>::SharedPtr send_trajectory_action_server_;

  rclcpp_action::GoalResponse goal_received_callback(
      const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const control_msgs::action::JointTrajectory::Goal> goal);

  rclcpp_action::CancelResponse goal_cancelled_callback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::JointTrajectory>> goal_handle);

  void goal_accepted_callback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::JointTrajectory>> goal_handle);

  void
  execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::JointTrajectory>> goal_handle);
  int check_dimensions(std::shared_ptr<const control_msgs::action::JointTrajectory::Goal> goal);
  trajectory_msgs::msg::JointTrajectory active_joint_traj_;
  uint32_t current_point_;
  bool trajectory_active_;
  uint64_t active_trajectory_elapsed_time_;
  uint64_t max_trajectory_time_;
  double scaling_factor_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::JointTrajectory>> current_handle;
};
}  // namespace ur_controllers
#endif  // UR_CONTROLLERS__PASSTHROUGH_TRAJECTORY_CONTROLLER_HPP_
