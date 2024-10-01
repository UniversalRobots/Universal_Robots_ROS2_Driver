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

#include <algorithm>
#include <cmath>
#include <controller_interface/controller_interface.hpp>
#include <rclcpp/logging.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <lifecycle_msgs/msg/state.hpp>

#include "ur_controllers/passthrough_trajectory_controller.hpp"

namespace ur_controllers
{

double duration_to_double(const builtin_interfaces::msg::Duration& duration)
{
  return duration.sec + (duration.nanosec / 1000000000);
}

controller_interface::CallbackReturn PassthroughTrajectoryController::on_init()
{
  passthrough_param_listener_ = std::make_shared<passthrough_trajectory_controller::ParamListener>(get_node());
  passthrough_params_ = passthrough_param_listener_->get_params();
  current_index_ = 0;
  joint_names_ = passthrough_params_.joints;
  state_interface_types_ = passthrough_params_.state_interfaces;
  scaling_factor_ = 1.0;
  clock_ = get_node()->get_clock();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn
PassthroughTrajectoryController::on_configure(const rclcpp_lifecycle::State& previous_state)
{
  start_action_server();
  trajectory_active_ = false;

  joint_state_interface_names_.clear();

  joint_state_interface_names_.reserve(joint_names_.size() * state_interface_types_.size());
  for (const auto& joint_name : joint_names_) {
    for (const auto& interface_type : state_interface_types_) {
      joint_state_interface_names_.emplace_back(joint_name + "/" + interface_type);
    }
  }

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

  std::copy(joint_state_interface_names_.cbegin(), joint_state_interface_names_.cend(), std::back_inserter(conf.names));

  conf.names.push_back(passthrough_params_.speed_scaling_interface_name);

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

  for (auto& interface_name : joint_state_interface_names_) {
    auto interface_it = std::find_if(state_interfaces_.begin(), state_interfaces_.end(),
                                     [&](auto& interface) { return (interface.get_name() == interface_name); });
    if (interface_it != state_interfaces_.end()) {
      if (interface_it->get_interface_name() == "position") {
        joint_position_state_interface_.emplace_back(*interface_it);

      } else if (interface_it->get_interface_name() == "velocity") {
        joint_position_state_interface_.emplace_back(*interface_it);
      }
    }
  }

  auto it = std::find_if(state_interfaces_.begin(), state_interfaces_.end(), [&](auto& interface) {
    return (interface.get_name() == passthrough_params_.speed_scaling_interface_name);
  });
  if (it != state_interfaces_.end()) {
    scaling_state_interface_ = *it;
  } else {
    RCLCPP_ERROR(get_node()->get_logger(), "Did not find speed scaling interface in state interfaces.");
    return controller_interface::CallbackReturn::ERROR;
  }

  return ControllerInterface::on_activate(state);
}

controller_interface::return_type PassthroughTrajectoryController::update(const rclcpp::Time& /*time*/,
                                                                          const rclcpp::Duration& period)
{
  const auto active_goal = *rt_active_goal_.readFromRT();

  const auto current_transfer_state =
      command_interfaces_[CommandInterfaces::PASSTHROUGH_TRAJECTORY_TRANSFER_STATE].get_value();

  if (active_goal && trajectory_active_) {
    if (current_transfer_state != TRANSFER_STATE_IDLE) {
      // Check if the trajectory has been aborted from the hardware interface. E.g. the robot was stopped on the teach
      // pendant.
      if (command_interfaces_[PASSTHROUGH_TRAJECTORY_ABORT].get_value() == 1.0) {
        RCLCPP_INFO(get_node()->get_logger(), "Trajectory aborted hardware, aborting action.");
        std::shared_ptr<control_msgs::action::FollowJointTrajectory::Result> result =
            std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
        active_goal->setAborted(result);
        end_goal();
        return controller_interface::return_type::OK;
      }
    }

    active_joint_traj_ = active_goal->gh_->get_goal()->trajectory;

    if (current_index_ == 0 && current_transfer_state == TRANSFER_STATE_IDLE) {
      active_trajectory_elapsed_time_ = rclcpp::Duration(0, 0);
      rclcpp::Duration::from_seconds(duration_to_double(active_joint_traj_.points.back().time_from_start));
      max_trajectory_time_ =
          rclcpp::Duration::from_seconds(duration_to_double(active_joint_traj_.points.back().time_from_start));
      // TODO(fexner): Merge goal tolerances with default tolerances
      goal_tolerance_ = active_goal->gh_->get_goal()->goal_tolerance;
      path_tolerance_ = active_goal->gh_->get_goal()->path_tolerance;
      goal_time_tolerance_ = active_goal->gh_->get_goal()->goal_time_tolerance;

      command_interfaces_[CommandInterfaces::PASSTHROUGH_TRAJECTORY_TRANSFER_STATE].set_value(
          TRANSFER_STATE_WAITING_FOR_POINT);
    }

    // Write a new point to the command interface, if the previous point has been read by the hardware interface.
    if (current_transfer_state == TRANSFER_STATE_WAITING_FOR_POINT) {
      if (current_index_ < active_joint_traj_.points.size()) {
        // TODO(fexner): Make sure that we are writing to the correct interface. Joints could be
        // ordered differently inside the trajectory!
        //  Write the time_from_start parameter.
        command_interfaces_[CommandInterfaces::PASSTHROUGH_TRAJECTORY_TIME_FROM_START].set_value(
            duration_to_double(active_joint_traj_.points[current_index_].time_from_start));

        // Write the positions for each joint of the robot
        for (size_t i = 0; i < joint_names_.size(); i++) {
          command_interfaces_[CommandInterfaces::PASSTHROUGH_TRAJECTORY_POSITIONS_ + i].set_value(
              active_joint_traj_.points[current_index_].positions[i]);
          // Optionally, also write velocities and accelerations for each joint.
          if (active_joint_traj_.points[current_index_].velocities.size() > 0) {
            command_interfaces_[CommandInterfaces::PASSTHROUGH_TRAJECTORY_VELOCITIES_ + i].set_value(
                active_joint_traj_.points[current_index_].velocities[i]);
          } else {
            command_interfaces_[CommandInterfaces::PASSTHROUGH_TRAJECTORY_VELOCITIES_].set_value(NO_VAL);
          }

          if (active_joint_traj_.points[current_index_].accelerations.size() > 0) {
            command_interfaces_[CommandInterfaces::PASSTHROUGH_TRAJECTORY_ACCELERATIONS_ + i].set_value(
                active_joint_traj_.points[current_index_].accelerations[i]);
          } else {
            command_interfaces_[CommandInterfaces::PASSTHROUGH_TRAJECTORY_ACCELERATIONS_ + i].set_value(NO_VAL);
          }
        }
        // Tell hardware interface that this point is ready to be read.
        command_interfaces_[CommandInterfaces::PASSTHROUGH_TRAJECTORY_TRANSFER_STATE].set_value(
            TRANSFER_STATE_TRANSFERRING);
        current_index_++;
        // Check if all points have been written to the hardware interface.
      } else if (current_index_ == active_joint_traj_.points.size()) {
        command_interfaces_[CommandInterfaces::PASSTHROUGH_TRAJECTORY_TRANSFER_STATE].set_value(
            TRANSFER_STATE_TRANSFER_DONE);
      } else {
        RCLCPP_ERROR(get_node()->get_logger(), "Hardware waiting for trajectory point while none is present!");
      }

      // When the trajectory is finished, report the goal as successful to the client.
    } else if (current_transfer_state == TRANSFER_STATE_DONE) {
      std::shared_ptr<control_msgs::action::FollowJointTrajectory::Result> result =
          std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
      // Check if the actual position complies with the tolerances given.
      if (!check_goal_tolerance()) {
        result->error_code = control_msgs::action::FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED;
        result->error_string = "Robot not within tolerances at end of trajectory.";
        active_goal->setAborted(result);
        end_goal();
        RCLCPP_ERROR(get_node()->get_logger(), "Trajectory failed, goal tolerances not met.");
        // TODO(fexner): Goal time check currently disabled due to
        // https://github.com/ros-controls/ros2_control/issues/1769
        //} else if (active_trajectory_elapsed_time_ > (max_trajectory_time_ + goal_time_tolerance_)) {
        //// Check if the goal time tolerance was complied with.
        // result->error_code = control_msgs::action::FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED;
        // result->error_string =
        // "Goal not reached within time tolerance. Missed goal time by " +
        // std::to_string((active_trajectory_elapsed_time_ - max_trajectory_time_ - goal_time_tolerance_).seconds()) +
        // " seconds.";
        // active_goal->setAborted(result);
        // end_goal();
        // RCLCPP_ERROR(get_node()->get_logger(),
        // "Trajectory failed, goal time tolerance not met. Missed goal time by %f seconds.",
        // (active_trajectory_elapsed_time_ - max_trajectory_time_ - goal_time_tolerance_).seconds());
      } else {
        result->error_code = control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL;
        active_goal->setSucceeded(result);
        end_goal();
        RCLCPP_INFO(get_node()->get_logger(), "Trajectory executed successfully!");
      }
    } else if (current_transfer_state == TRANSFER_STATE_IN_MOTION) {
      // Keep track of how long the trajectory has been executing, if it takes too long, send a warning.
      if (scaling_state_interface_.has_value()) {
        scaling_factor_ = scaling_state_interface_->get().get_value();
      }

      active_trajectory_elapsed_time_ += period * scaling_factor_;

      // RCLCPP_INFO(get_node()->get_logger(), "Elapsed trajectory time: %f. Scaling factor: %f, period: %f",
      // active_trajectory_elapsed_time_.seconds(), scaling_factor_, period.seconds());

      if (active_trajectory_elapsed_time_ > (max_trajectory_time_ + goal_time_tolerance_) && trajectory_active_) {
        RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *clock_, 1000,
                             "Trajectory should be finished by now. You may want to cancel this goal, if it is not.");
      }
    }
  }

  return controller_interface::return_type::OK;
}

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

  if (trajectory_active_) {
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
      RCLCPP_ERROR(get_node()->get_logger(), "%s", msg.c_str());
      msg = "Point nr " + std::to_string(i + 1) +
            " has: " + std::to_string(goal->trajectory.points[i].positions.size()) + " positions.";
      RCLCPP_ERROR(get_node()->get_logger(), "%s", msg.c_str());
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
      RCLCPP_ERROR(get_node()->get_logger(), "%s", msg.c_str());
      msg = "Point nr " + std::to_string(i + 1) +
            " has: " + std::to_string(goal->trajectory.points[i].velocities.size()) + " velocities.";
      RCLCPP_ERROR(get_node()->get_logger(), "%s", msg.c_str());
      return false;
    }
    if (goal->trajectory.points[i].velocities.size() != goal->trajectory.points[0].velocities.size()) {
      std::string msg;
      msg = "Can't accept new trajectory. All trajectory points must have velocities for all joints of the robot. "
            "(" +
            std::to_string(number_of_joints_) + " joint velocities per point)";
      RCLCPP_ERROR(get_node()->get_logger(), "%s", msg.c_str());
      msg = "Point nr " + std::to_string(i) + " has: " + std::to_string(goal->trajectory.points[i].velocities.size()) +
            " velocities.";
      RCLCPP_ERROR(get_node()->get_logger(), "%s", msg.c_str());
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
      RCLCPP_ERROR(get_node()->get_logger(), "%s", msg.c_str());
      msg = "Point nr " + std::to_string(i) +
            " has: " + std::to_string(goal->trajectory.points[i].accelerations.size()) + " accelerations.";
      RCLCPP_ERROR(get_node()->get_logger(), "%s", msg.c_str());
      return false;
    }
    if (goal->trajectory.points[i].accelerations.size() != goal->trajectory.points[0].accelerations.size()) {
      std::string msg;
      msg = "Can't accept new trajectory. All trajectory points must have accelerations for all joints of the "
            "robot. "
            "(" +
            std::to_string(number_of_joints_) + " joint accelerations per point)";
      RCLCPP_ERROR(get_node()->get_logger(), "%s", msg.c_str());
      msg = "Point nr " + std::to_string(i) +
            " has: " + std::to_string(goal->trajectory.points[i].accelerations.size()) + " accelerations.";
      RCLCPP_ERROR(get_node()->get_logger(), "%s", msg.c_str());
      return false;
    }
  }
  return true;
}

rclcpp_action::CancelResponse PassthroughTrajectoryController::goal_cancelled_callback(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle)
{
  // Check that cancel request refers to currently active goal (if any)
  const auto active_goal = *rt_active_goal_.readFromNonRT();
  if (active_goal && active_goal->gh_ == goal_handle) {
    RCLCPP_INFO(get_node()->get_logger(), "Cancelling active trajectory requested.");

    // Mark the current goal as canceled
    auto result = std::make_shared<FollowJTrajAction::Result>();
    active_goal->setCanceled(result);
    rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
    trajectory_active_ = false;
  }
  return rclcpp_action::CancelResponse::ACCEPT;
}

// Action goal was accepted, initialise values for a new trajectory.
void PassthroughTrajectoryController::goal_accepted_callback(
    std::shared_ptr<rclcpp_action::ServerGoalHandle<control_msgs::action::FollowJointTrajectory>> goal_handle)
{
  RCLCPP_INFO_STREAM(get_node()->get_logger(), "Accepted new trajectory with "
                                                   << goal_handle->get_goal()->trajectory.points.size() << " points.");
  current_index_ = 0;

  // TODO(fexner): Make sure the trajectory is correctly ordered!

  // Action handling will be done from the timer callback to avoid those things in the realtime
  // thread. First, we delete the existing (if any) timer by resetting the pointer and then create a new
  // one.
  RealtimeGoalHandlePtr rt_goal = std::make_shared<RealtimeGoalHandle>(goal_handle);
  rt_goal->preallocated_feedback_->joint_names = joint_names_;
  rt_goal->execute();
  rt_active_goal_.writeFromNonRT(rt_goal);
  goal_handle_timer_.reset();
  goal_handle_timer_ = get_node()->create_wall_timer(action_monitor_period_.to_chrono<std::chrono::nanoseconds>(),
                                                     std::bind(&RealtimeGoalHandle::runNonRealtime, rt_goal));
  trajectory_active_ = true;
  return;
}

bool PassthroughTrajectoryController::check_goal_tolerance()
{
  for (size_t i = 0; i < active_joint_traj_.joint_names.size(); ++i) {
    const std::string joint_name = active_joint_traj_.joint_names[i];
    auto joint_tolerance_it = std::find_if(goal_tolerance_.begin(), goal_tolerance_.end(),
                                           [&](auto entry) { return entry.name == joint_name; });
    auto position_state_it =
        std::find_if(joint_position_state_interface_.begin(), joint_position_state_interface_.end(),
                     [&](auto entry) { return entry.get().get_name() == joint_name + "/" + "position"; });
    if (std::abs(position_state_it->get().get_value() - active_joint_traj_.points.back().positions[i]) >
        joint_tolerance_it->position) {
      return false;
    }
  }
  return true;
}

void PassthroughTrajectoryController::end_goal()
{
  trajectory_active_ = false;
  command_interfaces_[CommandInterfaces::PASSTHROUGH_TRAJECTORY_TRANSFER_STATE].set_value(TRANSFER_STATE_IDLE);
}
}  // namespace ur_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ur_controllers::PassthroughTrajectoryController, controller_interface::ControllerInterface)
