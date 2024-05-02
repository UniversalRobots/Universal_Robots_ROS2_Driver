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

#include <cmath>
#include "ur_controllers/passthrough_trajectory_controller.hpp"
#include "lifecycle_msgs/msg/state.hpp"

namespace ur_controllers
{
controller_interface::CallbackReturn PassthroughTrajectoryController::on_init()
{
  passthrough_param_listener_ = std::make_shared<passthrough_trajectory_controller::ParamListener>(get_node());
  passthrough_params_ = passthrough_param_listener_->get_params();
  current_point_ = 0;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PassthroughTrajectoryController::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  start_action_server();
  trajectory_active_ = false;

  return ControllerInterface::on_configure(previous_state);
}

void PassthroughTrajectoryController::start_action_server(void)
{
  send_trajectory_action_server_ = rclcpp_action::create_server<control_msgs::action::JointTrajectory>(
      get_node(), std::string(get_node()->get_name()) + "/forward_joint_trajectory",
      std::bind(&PassthroughTrajectoryController::goal_received_callback, this, std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&PassthroughTrajectoryController::goal_cancelled_callback, this, std::placeholders::_1),
      std::bind(&PassthroughTrajectoryController::goal_accepted_callback, this, std::placeholders::_1));
  return;
}

controller_interface::InterfaceConfiguration PassthroughTrajectoryController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;

  conf.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  conf.names.push_back(passthrough_params_.speed_scaling_interface_name);

  return conf;
}

controller_interface::InterfaceConfiguration PassthroughTrajectoryController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  const std::string tf_prefix = passthrough_params_.tf_prefix;

  config.names.emplace_back(tf_prefix + "passthrough_controller/passthrough_trajectory_transfer_state");

  config.names.emplace_back(tf_prefix + "passthrough_controller/passthrough_point_written");

  config.names.emplace_back(tf_prefix + "passthrough_controller/passthrough_trajectory_number_of_points");

  config.names.emplace_back(tf_prefix + "passthrough_controller/passthrough_trajectory_cancel");

  config.names.emplace_back(tf_prefix + "passthrough_controller/passthrough_trajectory_dimensions");

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

  return config;
}

controller_interface::CallbackReturn PassthroughTrajectoryController::on_activate(const rclcpp_lifecycle::State& state)
{
  return ControllerInterface::on_activate(state);
}

controller_interface::return_type PassthroughTrajectoryController::update(const rclcpp::Time& /*time*/,
                                                                          const rclcpp::Duration& period)
{
  // static int delay = 1500;
  // if (!delay) {
  //   std::cout << "Speed scaling : " << state_interfaces_[0].get_value() << std::endl;
  //   std::cout << "Trajectory executing : " << trajectory_active_ << std::endl;
  //   delay = 1500;
  // }

  // if (delay)
  //   delay--;

  if (command_interfaces_[CommandInterfaces::PASSTHROUGH_TRAJECTORY_TRANSFER_STATE].get_value() == 3.0) {
    scaling_factor_ = state_interfaces_[0].get_value();
    active_trajectory_elapsed_time_ +=
        static_cast<int>(scaling_factor_ * ((period.seconds() * pow(10, 9)) + period.nanoseconds()));

    if (active_trajectory_elapsed_time_ > max_trajectory_time_ && trajectory_active_) {
      RCLCPP_WARN(get_node()->get_logger(), "Trajectory should be finished by now. You may want to cancel this goal, "
                                            "if it is not.");
      trajectory_active_ = false;
    }
  }
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

  if (command_interfaces_[CommandInterfaces::PASSTHROUGH_TRAJECTORY_TRANSFER_STATE].get_value() != 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Can't accept new trajectory. A trajectory is already executing.");
    return rclcpp_action::GoalResponse::REJECT;
  }
  // Check dimensions of the trajectory
  if (check_dimensions(goal) == 0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Trajectory rejected, dimensions of trajectory are wrong.");
    return rclcpp_action::GoalResponse::REJECT;
  } else {
    command_interfaces_[CommandInterfaces::PASSTHROUGH_TRAJECTORY_DIMENSIONS].set_value(
        static_cast<double>(check_dimensions(goal)));
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}
// This function should be told how many robot joints there are. Not just run with 6 or 0.
int PassthroughTrajectoryController::check_dimensions(
    std::shared_ptr<const control_msgs::action::JointTrajectory::Goal> goal)
{
  for (uint32_t i = 0; i < goal->trajectory.points.size(); i++) {
    std::string msg;
    if (goal->trajectory.points[i].positions.size() != 6) {
      RCLCPP_ERROR(get_node()->get_logger(), "Can't accept new trajectory. All trajectory points must have positions "
                                             "for all joints of the robot. (6 joint positions per point)");
      msg = "Point nr " + std::to_string(i + 1) +
            " has: " + std::to_string(goal->trajectory.points[i].positions.size()) + " positions.";
      RCLCPP_ERROR(get_node()->get_logger(), msg.c_str());
      return 0;
    }
    if (goal->trajectory.points[i].velocities.size() != 0 && goal->trajectory.points[i].velocities.size() != 6) {
      RCLCPP_ERROR(get_node()->get_logger(), "Can't accept new trajectory. All trajectory points must have velocities "
                                             "for all joints of the robot. (6 joint velocities per point)");
      msg = "Point nr " + std::to_string(i + 1) +
            " has: " + std::to_string(goal->trajectory.points[i].velocities.size()) + " velocities.";
      RCLCPP_ERROR(get_node()->get_logger(), msg.c_str());
      return 0;
    }
    if (goal->trajectory.points[i].accelerations.size() != 0 && goal->trajectory.points[i].accelerations.size() != 6) {
      RCLCPP_ERROR(get_node()->get_logger(), "Can't accept new trajectory. All trajectory points must have "
                                             "accelerations "
                                             "for all joints of the robot. (6 joint accelerations per point)");
      msg = "Point nr " + std::to_string(i + 1) +
            " has: " + std::to_string(goal->trajectory.points[i].accelerations.size()) + " accelerations.";
      RCLCPP_ERROR(get_node()->get_logger(), msg.c_str());
      return 0;
    }
  }
  if (goal->trajectory.points[0].velocities.size() == 6 && goal->trajectory.points[0].accelerations.size() == 0) {
    /*If there are only positions and velocities defined */
    return 2;
  } else if (goal->trajectory.points[0].velocities.size() == 0 &&
             goal->trajectory.points[0].accelerations.size() == 6) {
    /*If there are only positions and accelerations defined */
    return 3;
  } else if (goal->trajectory.points[0].velocities.size() == 6 &&
             goal->trajectory.points[0].accelerations.size() == 6) {
    /*If there are both positions, velocities and accelerations defined */
    return 4;
  } else {
    /*If there are only positions defined */
    return 1;
  }
}

rclcpp_action::CancelResponse PassthroughTrajectoryController::goal_cancelled_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::JointTrajectory>> goal_handle)
{
  RCLCPP_INFO(get_node()->get_logger(), "Cancelling active trajectory because cancel callback received.");
  command_interfaces_[CommandInterfaces::PASSTHROUGH_TRAJECTORY_TRANSFER_STATE].set_value(0.0);
  current_point_ = 0;
  command_interfaces_[CommandInterfaces::PASSTHROUGH_TRAJECTORY_CANCEL].set_value(1.0);
  std::shared_ptr<control_msgs::action::JointTrajectory::Result> result =
      std::make_shared<control_msgs::action::JointTrajectory::Result>();
  goal_handle->canceled(result);
  trajectory_active_ = false;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PassthroughTrajectoryController::goal_accepted_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::JointTrajectory>> goal_handle)
{
  current_handle = goal_handle;
  RCLCPP_INFO(get_node()->get_logger(), "Accepted new trajectory.");
  trajectory_active_ = true;
  active_trajectory_elapsed_time_ = 0;
  current_point_ = 0;

  command_interfaces_[CommandInterfaces::PASSTHROUGH_TRAJECTORY_CANCEL].set_value(0.0);

  active_joint_traj_ = goal_handle->get_goal()->trajectory;

  max_trajectory_time_ = (active_joint_traj_.points.back().time_from_start.sec * pow(10, 9)) +
                         active_joint_traj_.points.back().time_from_start.nanosec;

  command_interfaces_[CommandInterfaces::PASSTHROUGH_TRAJECTORY_NUMBER_OF_POINTS].set_value(
      active_joint_traj_.points.size());

  std::cout << "Size of positions: " << active_joint_traj_.points[0].positions.size() << std::endl;

  std::cout << "Size of velocities: " << active_joint_traj_.points[0].velocities.size() << std::endl;

  std::cout << "Size of accelerations: " << active_joint_traj_.points[0].accelerations.size() << std::endl;

  command_interfaces_[CommandInterfaces::PASSTHROUGH_POINT_WRITTEN].set_value(1.0);
  command_interfaces_[CommandInterfaces::PASSTHROUGH_TRAJECTORY_TRANSFER_STATE].set_value(1.0);

  std::thread{ std::bind(&PassthroughTrajectoryController::execute, this, std::placeholders::_1), goal_handle }
      .detach();
  return;
}

void PassthroughTrajectoryController::execute(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::JointTrajectory>> goal_handle)
{
  rclcpp::Rate loop_rate(200);
  while (command_interfaces_[CommandInterfaces::PASSTHROUGH_TRAJECTORY_TRANSFER_STATE].get_value() != 0.0) {
    if (command_interfaces_[PASSTHROUGH_TRAJECTORY_CANCEL].get_value() == 1.0) {
      RCLCPP_INFO(get_node()->get_logger(), "Trajectory cancelled from hardware interface, aborting action.");
      std::shared_ptr<control_msgs::action::JointTrajectory::Result> result =
          std::make_shared<control_msgs::action::JointTrajectory::Result>();
      goal_handle->abort(result);
      command_interfaces_[CommandInterfaces::PASSTHROUGH_TRAJECTORY_TRANSFER_STATE].set_value(0.0);
      trajectory_active_ = false;
      return;
    }

    if (goal_handle->is_canceling()) {
      std::shared_ptr<control_msgs::action::JointTrajectory::Result> result =
          std::make_shared<control_msgs::action::JointTrajectory::Result>();
      goal_handle->canceled(result);
      return;
    }

    if (command_interfaces_[CommandInterfaces::PASSTHROUGH_TRAJECTORY_TRANSFER_STATE].get_value() == 1.0) {
      // Write a new point to the command interface, if the previous point has been written to the hardware interface.
      if (command_interfaces_[CommandInterfaces::PASSTHROUGH_POINT_WRITTEN].get_value() == 1.0 &&
          current_point_ < active_joint_traj_.points.size()) {
        // Write the time_from_start parameter.
        command_interfaces_[CommandInterfaces::PASSTHROUGH_TRAJECTORY_TIME_FROM_START].set_value(
            active_joint_traj_.points[current_point_].time_from_start.sec +
            (active_joint_traj_.points[current_point_].time_from_start.nanosec / 1000000000));
        // Write the positions for each joint of the robot
        for (int i = 0; i < 6; i++) {
          command_interfaces_[CommandInterfaces::PASSTHROUGH_TRAJECTORY_POSITIONS_ + i].set_value(
              active_joint_traj_.points[current_point_].positions[i]);
          // Optionally, also write velocities and accelerations for each joint.
          if (active_joint_traj_.points[current_point_].velocities.size() > 0) {
            command_interfaces_[CommandInterfaces::PASSTHROUGH_TRAJECTORY_VELOCITIES_ + i].set_value(
                active_joint_traj_.points[current_point_].velocities[i]);
          }

          if (active_joint_traj_.points[current_point_].accelerations.size() > 0) {
            command_interfaces_[CommandInterfaces::PASSTHROUGH_TRAJECTORY_ACCELERATIONS_ + i].set_value(
                active_joint_traj_.points[current_point_].accelerations[i]);
          }
        }
        // Tell hardware interface that this point is ready to be read.
        command_interfaces_[CommandInterfaces::PASSTHROUGH_POINT_WRITTEN].set_value(0.0);
        current_point_++;
      }
      // Check if all points have been written to the hardware
      if (command_interfaces_[CommandInterfaces::PASSTHROUGH_POINT_WRITTEN].get_value() == 1.0 &&
          current_point_ == active_joint_traj_.points.size()) {
        RCLCPP_INFO(get_node()->get_logger(), "All points sent to the hardware interface, trajectory will now "
                                              "execute!");
        command_interfaces_[CommandInterfaces::PASSTHROUGH_TRAJECTORY_TRANSFER_STATE].set_value(2.0);
      }
    }

    if (command_interfaces_[CommandInterfaces::PASSTHROUGH_TRAJECTORY_TRANSFER_STATE].get_value() == 4.0) {
      std::shared_ptr<control_msgs::action::JointTrajectory::Result> result =
          std::make_shared<control_msgs::action::JointTrajectory::Result>();
      goal_handle->succeed(result);
      RCLCPP_INFO(get_node()->get_logger(), "Trajectory executed successfully!");
      std::cout << "It took this long: " << active_trajectory_elapsed_time_ << std::endl;
      command_interfaces_[CommandInterfaces::PASSTHROUGH_TRAJECTORY_TRANSFER_STATE].set_value(0.0);
      trajectory_active_ = false;
      return;
    }
    loop_rate.sleep();
  }
  return;
}
}  // namespace ur_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ur_controllers::PassthroughTrajectoryController, controller_interface::ControllerInterface)
