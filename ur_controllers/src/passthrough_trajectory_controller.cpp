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
  joint_names_ = passthrough_params_.joints;
  state_interface_types_ = passthrough_params_.state_interfaces;
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
  send_trajectory_action_server_ = rclcpp_action::create_server<control_msgs::action::FollowJointTrajectory>(
      get_node(), std::string(get_node()->get_name()) + "/follow_joint_trajectory",
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
  const std::string tf_prefix = passthrough_params_.tf_prefix;

  conf.names.reserve(joint_names_.size() * state_interface_types_.size());
  for (const auto& joint_name : joint_names_) {
    for (const auto& interface_type : state_interface_types_) {
      conf.names.emplace_back(joint_name + "/" + interface_type);
    }
  }
  conf.names.emplace_back(passthrough_params_.speed_scaling_interface_name);
  conf.names.emplace_back(tf_prefix + "passthrough_controller/running");

  return conf;
}

controller_interface::InterfaceConfiguration PassthroughTrajectoryController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  const std::string tf_prefix = passthrough_params_.tf_prefix;

  config.names.emplace_back(tf_prefix + "passthrough_controller/passthrough_trajectory_transfer_state");

  config.names.emplace_back(tf_prefix + "passthrough_controller/passthrough_trajectory_abort");

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
  number_of_joints_ = joint_names_.size();
  // clear out vectors in case of restart
  joint_position_state_interface_.clear();
  joint_velocity_state_interface_.clear();

  for (uint32_t i = 0; i < state_interface_types_.size() * joint_names_.size(); i++) {
    if (state_interfaces_[i].get_interface_name() == "position") {
      joint_position_state_interface_.emplace_back(state_interfaces_[i]);
    } else if (state_interfaces_[i].get_interface_name() == "velocity") {
      joint_velocity_state_interface_.emplace_back(state_interfaces_[i]);
    }
  }

  return ControllerInterface::on_activate(state);
}

controller_interface::return_type PassthroughTrajectoryController::update(const rclcpp::Time& time,
                                                                          const rclcpp::Duration& /* period */)
{
  AsyncInfo temp = info_to_realtime_.get();
  if (temp.info_updated) {
    command_interfaces_[CommandInterfaces::PASSTHROUGH_TRAJECTORY_TRANSFER_STATE].set_value(temp.transfer_state);
    command_interfaces_[PASSTHROUGH_TRAJECTORY_ABORT].set_value(temp.abort);
    temp.info_updated = false;
    info_to_realtime_.set(temp);
  }

  if (command_interfaces_[CommandInterfaces::PASSTHROUGH_TRAJECTORY_TRANSFER_STATE].get_value() !=
      TRANSFER_STATE_IDLE) {
    /* Check if the trajectory has been aborted from the hardware interface. E.g. the robot was stopped on the teach
     * pendant. */
    if (command_interfaces_[PASSTHROUGH_TRAJECTORY_ABORT].get_value() == 1.0) {
      RCLCPP_INFO(get_node()->get_logger(), "Trajectory cancelled from hardware interface, aborting action.");
      std::shared_ptr<control_msgs::action::FollowJointTrajectory::Result> result =
          std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
      active_goal_->abort(result);
      end_goal();
      return controller_interface::return_type::OK;
    }
    /* Check if the goal has been cancelled by the ROS user. */
    if (active_goal_->is_canceling()) {
      std::shared_ptr<control_msgs::action::FollowJointTrajectory::Result> result =
          std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
      active_goal_->canceled(result);
      end_goal();
      return controller_interface::return_type::OK;
    }
    // Write a new point to the command interface, if the previous point has been read by the hardware interface.
    if (command_interfaces_[CommandInterfaces::PASSTHROUGH_TRAJECTORY_TRANSFER_STATE].get_value() ==
        TRANSFER_STATE_WAITING_FOR_POINT) {
      if (current_point_ < active_joint_traj_.points.size()) {
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
          } else {
            command_interfaces_[CommandInterfaces::PASSTHROUGH_TRAJECTORY_VELOCITIES_].set_value(NO_VAL);
          }

          if (active_joint_traj_.points[current_point_].accelerations.size() > 0) {
            command_interfaces_[CommandInterfaces::PASSTHROUGH_TRAJECTORY_ACCELERATIONS_ + i].set_value(
                active_joint_traj_.points[current_point_].accelerations[i]);
          } else {
            command_interfaces_[CommandInterfaces::PASSTHROUGH_TRAJECTORY_ACCELERATIONS_ + i].set_value(NO_VAL);
          }
        }
        // Tell hardware interface that this point is ready to be read.
        command_interfaces_[CommandInterfaces::PASSTHROUGH_TRAJECTORY_TRANSFER_STATE].set_value(
            TRANSFER_STATE_TRANSFERRING);
        current_point_++;

        // Check if all points have been written to the hardware interface.
      } else if (current_point_ == active_joint_traj_.points.size()) {
        RCLCPP_INFO(get_node()->get_logger(), "All points sent to the hardware interface, trajectory will now "
                                              "execute!");
        command_interfaces_[CommandInterfaces::PASSTHROUGH_TRAJECTORY_TRANSFER_STATE].set_value(TRANSFER_STATE_DONE);
      }
      // When the trajectory is finished, report the goal as successful to the client.
    } else if (command_interfaces_[CommandInterfaces::PASSTHROUGH_TRAJECTORY_TRANSFER_STATE].get_value() ==
               TRANSFER_STATE_DONE) {
      std::shared_ptr<control_msgs::action::FollowJointTrajectory::Result> result =
          std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
      // Check if the actual position complies with the tolerances given.
      if (!check_goal_tolerance()) {
        result->error_code = control_msgs::action::FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED;
        result->error_string = "Robot not within tolerances at end of trajectory.";
        active_goal_->abort(result);
        RCLCPP_ERROR(get_node()->get_logger(), "Trajectory failed, goal tolerances not met.");
        // Check if the goal time tolerance was complied with.
      } else if (active_trajectory_elapsed_time_ > (max_trajectory_time_ + goal_time_tolerance_.seconds())) {
        result->error_code = control_msgs::action::FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED;
        result->error_string = "Goal not reached within time tolerance";
        active_goal_->abort(result);
        RCLCPP_ERROR(get_node()->get_logger(), "Trajectory failed, goal time tolerance not met.");
      } else {
        result->error_code = control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL;
        active_goal_->succeed(result);
        RCLCPP_INFO(get_node()->get_logger(), "Trajectory executed successfully!");
      }
      end_goal();
    }
  }
  static bool firstpass = true;
  if (firstpass) {
    now_ns = time.nanoseconds();
    firstpass = false;
  } else {
    last_time_ns = now_ns;
    now_ns = time.nanoseconds();
    period_ns = now_ns - last_time_ns;
  }
  /* Keep track of how long the trajectory has been executing, if it takes too long, send a warning. */
  if (command_interfaces_[CommandInterfaces::PASSTHROUGH_TRAJECTORY_TRANSFER_STATE].get_value() ==
      TRANSFER_STATE_IN_MOTION) {
    scaling_factor_ = state_interfaces_[StateInterfaces::SPEED_SCALING_FACTOR].get_value();

    active_trajectory_elapsed_time_ += static_cast<double>(scaling_factor_ * (period_ns / pow(10, 9)));

    if (active_trajectory_elapsed_time_ > (max_trajectory_time_ + goal_time_tolerance_.seconds()) &&
        trajectory_active_) {
      RCLCPP_WARN(get_node()->get_logger(), "Trajectory should be finished by now. You may want to cancel this goal, "
                                            "if it is not.");
      trajectory_active_ = false;
    }
  }
  AsyncInfo info = { command_interfaces_[CommandInterfaces::PASSTHROUGH_TRAJECTORY_TRANSFER_STATE].get_value(),
                     command_interfaces_[PASSTHROUGH_TRAJECTORY_ABORT].get_value(),
                     state_interfaces_[StateInterfaces::CONTROLLER_RUNNING].get_value(), true };

  info_from_realtime_.set(info);
  return controller_interface::return_type::OK;
}  // namespace ur_controllers

rclcpp_action::GoalResponse PassthroughTrajectoryController::goal_received_callback(
    const rclcpp_action::GoalUUID& /*uuid*/,
    std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal)
{
  RCLCPP_INFO(get_node()->get_logger(), "Received new trajectory.");
  // Precondition: Running controller
  if (get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
    RCLCPP_ERROR(get_node()->get_logger(), "Can't accept new trajectories. Controller is not running.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (info_from_realtime_.get().controller_running != 1.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Trajectory rejected, controller not running in hardware interface.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  if (info_from_realtime_.get().transfer_state != 0.0) {
    RCLCPP_ERROR(get_node()->get_logger(), "Can't accept new trajectory. A trajectory is already executing.");
    return rclcpp_action::GoalResponse::REJECT;
  }

  // Check that all parts of the trajectory are valid.
  if (!check_positions(goal)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Trajectory rejected");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!check_velocities(goal)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Trajectory rejected");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!check_accelerations(goal)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Trajectory rejected");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

bool PassthroughTrajectoryController::check_positions(
    std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal)
{
  for (uint32_t i = 0; i < goal->trajectory.points.size(); i++) {
    if (goal->trajectory.points[i].positions.size() != number_of_joints_) {
      std::string msg;
      msg = "Can't accept new trajectory. All trajectory points must have positions for all joints of the robot. (" +
            std::to_string(number_of_joints_) + " joint positions per point)";
      RCLCPP_ERROR(get_node()->get_logger(), msg.c_str());
      msg = "Point nr " + std::to_string(i + 1) +
            " has: " + std::to_string(goal->trajectory.points[i].positions.size()) + " positions.";
      RCLCPP_ERROR(get_node()->get_logger(), msg.c_str());
      return false;
    }
  }
  return true;
}

bool PassthroughTrajectoryController::check_velocities(
    std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal)
{
  for (uint32_t i = 0; i < goal->trajectory.points.size(); i++) {
    if (goal->trajectory.points[i].velocities.size() != number_of_joints_ &&
        goal->trajectory.points[i].velocities.size() != 0) {
      std::string msg;
      msg = "Can't accept new trajectory. All trajectory points must either not have velocities or have them for all "
            "joints of the robot. (" +
            std::to_string(number_of_joints_) + " joint velocities per point)";
      RCLCPP_ERROR(get_node()->get_logger(), msg.c_str());
      msg = "Point nr " + std::to_string(i + 1) +
            " has: " + std::to_string(goal->trajectory.points[i].velocities.size()) + " velocities.";
      RCLCPP_ERROR(get_node()->get_logger(), msg.c_str());
      return false;
    }
    if (goal->trajectory.points[i].velocities.size() != goal->trajectory.points[0].velocities.size()) {
      std::string msg;
      msg = "Can't accept new trajectory. All trajectory points must have velocities for all joints of the robot. "
            "(" +
            std::to_string(number_of_joints_) + " joint velocities per point)";
      RCLCPP_ERROR(get_node()->get_logger(), msg.c_str());
      msg = "Point nr " + std::to_string(i) + " has: " + std::to_string(goal->trajectory.points[i].velocities.size()) +
            " velocities.";
      RCLCPP_ERROR(get_node()->get_logger(), msg.c_str());
      return false;
    }
  }
  return true;
}

bool PassthroughTrajectoryController::check_accelerations(
    std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal)
{
  for (uint32_t i = 0; i < goal->trajectory.points.size(); i++) {
    if (goal->trajectory.points[i].accelerations.size() != 0 &&
        goal->trajectory.points[i].accelerations.size() != number_of_joints_) {
      std::string msg;
      msg = "Can't accept new trajectory. All trajectory points must either not have accelerations or have them for "
            "all joints of the robot. (" +
            std::to_string(number_of_joints_) + " joint accelerations per point)";
      RCLCPP_ERROR(get_node()->get_logger(), msg.c_str());
      msg = "Point nr " + std::to_string(i) +
            " has: " + std::to_string(goal->trajectory.points[i].accelerations.size()) + " accelerations.";
      RCLCPP_ERROR(get_node()->get_logger(), msg.c_str());
      return false;
    }
    if (goal->trajectory.points[i].accelerations.size() != goal->trajectory.points[0].accelerations.size()) {
      std::string msg;
      msg = "Can't accept new trajectory. All trajectory points must have accelerations for all joints of the "
            "robot. "
            "(" +
            std::to_string(number_of_joints_) + " joint accelerations per point)";
      RCLCPP_ERROR(get_node()->get_logger(), msg.c_str());
      msg = "Point nr " + std::to_string(i) +
            " has: " + std::to_string(goal->trajectory.points[i].accelerations.size()) + " accelerations.";
      RCLCPP_ERROR(get_node()->get_logger(), msg.c_str());
      return false;
    }
  }
  return true;
}

// Called when the action is cancelled by the action client.
rclcpp_action::CancelResponse PassthroughTrajectoryController::goal_cancelled_callback(
    const std::shared_ptr<
        rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> /* goal_handle */)
{
  RCLCPP_INFO(get_node()->get_logger(), "Cancelling active trajectory because cancel callback received.");
  return rclcpp_action::CancelResponse::ACCEPT;
}

// Action goal was accepted, initialise values for a new trajectory.
void PassthroughTrajectoryController::goal_accepted_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle)
{
  RCLCPP_INFO(get_node()->get_logger(), "Accepted new trajectory.");
  trajectory_active_ = true;
  active_trajectory_elapsed_time_ = 0;
  current_point_ = 0;

  active_joint_traj_ = goal_handle->get_goal()->trajectory;
  goal_tolerance_ = goal_handle->get_goal()->goal_tolerance;
  path_tolerance_ = goal_handle->get_goal()->path_tolerance;
  goal_time_tolerance_ = goal_handle->get_goal()->goal_time_tolerance;
  active_goal_ = goal_handle;

  max_trajectory_time_ = active_joint_traj_.points.back().time_from_start.sec +
                         (active_joint_traj_.points.back().time_from_start.nanosec / pow(10, 9));

  // Communicate to update() method, that the action has been triggered.
  AsyncInfo temp = { 1.0, 0.0, 0.0, true };
  info_to_realtime_.set(temp);

  return;
}

bool PassthroughTrajectoryController::check_goal_tolerance()
{
  for (uint32_t i = 0; i < goal_tolerance_.size(); i++) {
    if (std::abs(joint_position_state_interface_[i].get().get_value() - active_joint_traj_.points.back().positions[i]) >
        goal_tolerance_[i].position) {
      return false;
    }
  }
  return true;
}

void PassthroughTrajectoryController::end_goal()
{
  trajectory_active_ = false;
  command_interfaces_[CommandInterfaces::PASSTHROUGH_TRAJECTORY_TRANSFER_STATE].set_value(0.0);
  active_goal_ = NULL;
  current_point_ = 0;
  goal_tolerance_.clear();
  path_tolerance_.clear();
}
}  // namespace ur_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ur_controllers::PassthroughTrajectoryController, controller_interface::ControllerInterface)
