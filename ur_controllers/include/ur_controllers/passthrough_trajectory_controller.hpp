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
#include "rclcpp/time.hpp"
#include "rclcpp/duration.hpp"

#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "control_msgs/action/follow_joint_trajectory.hpp"
#include "control_msgs/action/joint_trajectory.hpp"

#include "passthrough_trajectory_controller_parameters.hpp"

namespace ur_controllers
{
enum CommandInterfaces
{
  /* The PASSTHROUGH_TRAJECTORY_TRANSFER_STATE value is used to keep track of which stage the transfer is in.
  0.0: No trajectory to forward, the controller is idling and ready to receive a new trajectory.
  1.0: The controller has received and accepted a new trajecotry. When the state is 1.0, the controller will write a
  point to the hardware interface.
  2.0: The hardware interface will read the point written from the controller. The state will switch between 1.0
  and 2.0 until all points have been read by the hardware interface.
  3.0: The hardware interface has read all the points, and will now write all the points to the robot driver.
  4.0: The robot is moving through the trajectory.
  5.0: The robot finished executing the trajectory. */
  PASSTHROUGH_TRAJECTORY_TRANSFER_STATE = 0,
  /* The PASSTHROUGH_TRAJECTORY_CANCEL value is used to indicate whether the trajectory has been cancelled from the
   * hardware interface.*/
  PASSTHROUGH_TRAJECTORY_CANCEL = 1,
  /* The PASSTHROUGH_TRAJECTORY_DIMENSIONS is used to indicate what combination of joint positions, velocities and
   * accelerations the trajectory consists of, see check_dimensions() for a description of what the values mean. */
  PASSTHROUGH_TRAJECTORY_DIMENSIONS = 2,
  /* Arrays to hold the values of a point. */
  PASSTHROUGH_TRAJECTORY_POSITIONS_ = 3,
  PASSTHROUGH_TRAJECTORY_VELOCITIES_ = 9,
  PASSTHROUGH_TRAJECTORY_ACCELERATIONS_ = 15,
  PASSTHROUGH_TRAJECTORY_TIME_FROM_START = 21
};

enum StateInterfaces
{
  /* State interface 0 - 17 are joint state interfaces*/

  SPEED_SCALING_FACTOR = 18,
  NUMBER_OF_JOINTS = 19,
  CONTROLLER_RUNNING = 20
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
  /* Start an action server with an action called: /passthrough_trajectory_controller/forward_joint_trajectory. */
  void start_action_server(void);

  void end_goal();

  bool check_goal_tolerance();

  std::shared_ptr<passthrough_trajectory_controller::ParamListener> passthrough_param_listener_;
  passthrough_trajectory_controller::Params passthrough_params_;

  rclcpp_action::Server<control_msgs::action::FollowJointTrajectory>::SharedPtr send_trajectory_action_server_;

  rclcpp_action::GoalResponse
  goal_received_callback(const rclcpp_action::GoalUUID& uuid,
                         std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal);

  rclcpp_action::CancelResponse goal_cancelled_callback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle);

  void goal_accepted_callback(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle);

  void execute(
      const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle);

  std::vector<std::string> joint_names_;
  std::vector<std::string> state_interface_types_;

  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_position_state_interface_;
  std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> joint_velocity_state_interface_;

  /*
  If there are values in the velocities and accelerations vectors, they should have as many elements as there are
  joints, and all be the same size.
  The return value indicates what combination of joint positions, velocities and accelerations is present;
  0: The trajectory is invalid, and will be rejected.
  1: Only positions are defined for the trajectory.
  2: Positions and velocities are defined for the trajectory.
  3: Positions and accelerations are defined for the trajectory.
  4: Both positions, velocities and accelerations are defined for the trajectory.
  */
  int check_dimensions(std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal);

  trajectory_msgs::msg::JointTrajectory active_joint_traj_;
  std::vector<control_msgs::msg::JointTolerance> path_tolerance_;
  std::vector<control_msgs::msg::JointTolerance> goal_tolerance_;
  rclcpp::Duration goal_time_tolerance_ = rclcpp::Duration::from_nanoseconds(0);

  uint32_t current_point_;
  bool trajectory_active_;
  uint64_t period_ns;
  uint64_t last_time_ns;
  uint64_t now_ns;
  double active_trajectory_elapsed_time_;
  double max_trajectory_time_;
  double scaling_factor_;
  uint32_t number_of_joints_;
  std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> active_goal_;
};
}  // namespace ur_controllers
#endif  // UR_CONTROLLERS__PASSTHROUGH_TRAJECTORY_CONTROLLER_HPP_
