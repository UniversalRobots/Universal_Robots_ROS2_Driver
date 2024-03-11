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

#include "ur_controllers/passthrough_trajectory_controller.hpp"
#include "lifecycle_msgs/msg/state.hpp"

namespace ur_controllers
{
controller_interface::CallbackReturn PassthroughTrajectoryController::on_init()
{
  passthrough_param_listener_ = std::make_shared<passthrough_trajectory_controller::ParamListener>(get_node());
  passthrough_params_ = passthrough_param_listener_->get_params();

  std::cout << "-------------------------------Initialised passthrough "
               "controller-------------------------------------------------"
            << std::endl;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PassthroughTrajectoryController::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  send_trajectory_action_server_ = rclcpp_action::create_server<control_msgs::action::JointTrajectory>(
      get_node(), std::string(get_node()->get_name()) + "/forward_joint_trajectory",
      std::bind(&PassthroughTrajectoryController::goal_received_callback, this, std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&PassthroughTrajectoryController::goal_cancelled_callback, this, std::placeholders::_1),
      std::bind(&PassthroughTrajectoryController::goal_accepted_callback, this, std::placeholders::_1));

  std::cout << "-------------------------------Configured passthrough "
               "controller-------------------------------------------------"
            << std::endl;

  return ControllerInterface::on_configure(previous_state);
}

controller_interface::InterfaceConfiguration PassthroughTrajectoryController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;

  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  conf.names.push_back(passthrough_params_.speed_scaling_interface_name);

  std::cout << conf.names[0] << " / " << conf.names.size() << std::endl;

  std::cout << "------------------------------- Configured passthrough "
               "controller state interface -------------------------------------------------"
            << std::endl;

  return conf;
}

controller_interface::InterfaceConfiguration PassthroughTrajectoryController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  const std::string tf_prefix = passthrough_params_.tf_prefix;

  config.names.emplace_back(tf_prefix + "passthrough_controller/passthrough_trajectory_present");

  config.names.emplace_back(tf_prefix + "passthrough_controller/passthrough_trajectory_number_of_points_");

  for (size_t i = 0; i < 6; ++i) {
    config.names.emplace_back(tf_prefix + "passthrough_controller/passthrough_trajectory_positions_" +
                              std::to_string(i));
  }
  for (size_t i = 0; i < 6; ++i) {
    config.names.emplace_back(tf_prefix + "passthrough_controller/passthrough_trajectory_velocities_" +
                              std::to_string(i));
  }
  for (size_t i = 0; i < 6; ++i) {
    config.names.emplace_back(tf_prefix + "passthrough_controller/passthrough_trajectory_accelerations_" +
                              std::to_string(i));
  }

  config.names.emplace_back(tf_prefix + "passthrough_controller/passthrough_trajectory_time_from_start");

  std::cout << "------------------------------- Configured passthrough "
               "controller command interface -------------------------------------------------"
            << std::endl;

  return config;
}

controller_interface::CallbackReturn PassthroughTrajectoryController::on_activate(const rclcpp_lifecycle::State& state)
{
  std::cout << "-------------------------------Activated passthrough "
               "controller-------------------------------------------------"
            << std::endl;
  return ControllerInterface::on_activate(state);
}

controller_interface::return_type PassthroughTrajectoryController::update(const rclcpp::Time& /*time*/,
                                                                          const rclcpp::Duration& /*period*/)
{
  static int delay = 500;
  if (!delay) {
    std::cout << "-------------------------------Updated passthrough "
                 "controller-------------------------------------------------"
              << std::endl;

    // std::cout << command_interfaces_.at(2).get_name() << " = " << command_interfaces_.at(2).get_value() << std::endl;
    delay = 500;
  }
  delay--;
  return controller_interface::return_type::OK;
}

rclcpp_action::GoalResponse PassthroughTrajectoryController::goal_received_callback(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const control_msgs::action::JointTrajectory::Goal> goal)
{
  RCLCPP_INFO(get_node()->get_logger(), "Received new trajectory to forward to robot");

  // Precondition: Running controller
  if (get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    RCLCPP_ERROR(get_node()->get_logger(), "Can't accept new trajectories. Controller is not running.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PassthroughTrajectoryController::goal_cancelled_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::JointTrajectory>> goal_handle)
{
  RCLCPP_INFO(get_node()->get_logger(), "Canceling active trajectory because cancel callback received.");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PassthroughTrajectoryController::goal_accepted_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::JointTrajectory>> goal_handle)
{
  RCLCPP_INFO(get_node()->get_logger(), "Accepted new trajectory.");
  return;
}

}  // namespace ur_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ur_controllers::PassthroughTrajectoryController, controller_interface::ControllerInterface)
