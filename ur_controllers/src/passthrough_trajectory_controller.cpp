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
#include <sstream>

#include <controller_interface/controller_interface.hpp>
#include <rclcpp/logging.hpp>
#include <builtin_interfaces/msg/duration.hpp>
#include <lifecycle_msgs/msg/state.hpp>

#include "ur_controllers/passthrough_trajectory_controller.hpp"

namespace ur_controllers
{

double duration_to_double(const builtin_interfaces::msg::Duration& duration)
{
  return duration.sec + (duration.nanosec / 1000000000.0);
}

controller_interface::CallbackReturn PassthroughTrajectoryController::on_init()
{
  passthrough_param_listener_ = std::make_shared<passthrough_trajectory_controller::ParamListener>(get_node());
  passthrough_params_ = passthrough_param_listener_->get_params();
  current_index_ = 0;
  auto joint_names = passthrough_params_.joints;
  joint_names_.writeFromNonRT(joint_names);
  number_of_joints_ = joint_names.size();
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

  joint_state_interface_names_.reserve(number_of_joints_ * state_interface_types_.size());

  auto joint_names_internal = joint_names_.readFromRT();
  for (const auto& joint_name : *joint_names_internal) {
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

  for (size_t i = 0; i < number_of_joints_; ++i) {
    config.names.emplace_back(tf_prefix + "trajectory_passthrough/setpoint_positions_" + std::to_string(i));
    config.names.emplace_back(tf_prefix + "trajectory_passthrough/setpoint_velocities_" + std::to_string(i));
    config.names.emplace_back(tf_prefix + "trajectory_passthrough/setpoint_accelerations_" + std::to_string(i));
  }

  config.names.push_back(tf_prefix + "trajectory_passthrough/abort");
  config.names.emplace_back(tf_prefix + "trajectory_passthrough/transfer_state");
  config.names.emplace_back(tf_prefix + "trajectory_passthrough/time_from_start");

  return config;
}

controller_interface::CallbackReturn PassthroughTrajectoryController::on_activate(const rclcpp_lifecycle::State& state)
{
  // clear out vectors in case of restart
  joint_position_state_interface_.clear();
  joint_velocity_state_interface_.clear();
  joint_acceleration_state_interface_.clear();

  for (auto& interface_name : joint_state_interface_names_) {
    auto interface_it = std::find_if(state_interfaces_.begin(), state_interfaces_.end(),
                                     [&](auto& interface) { return (interface.get_name() == interface_name); });
    if (interface_it != state_interfaces_.end()) {
      if (interface_it->get_interface_name() == "position") {
        joint_position_state_interface_.emplace_back(*interface_it);
      } else if (interface_it->get_interface_name() == "velocity") {
        joint_velocity_state_interface_.emplace_back(*interface_it);
      } else if (interface_it->get_interface_name() == "acceleration") {
        joint_acceleration_state_interface_.emplace_back(*interface_it);
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

  {
    const std::string interface_name = passthrough_params_.tf_prefix + "trajectory_passthrough/"
                                                                       "abort";
    auto it = std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
                           [&](auto& interface) { return (interface.get_name() == interface_name); });
    if (it != command_interfaces_.end()) {
      abort_command_interface_ = *it;
    } else {
      RCLCPP_ERROR(get_node()->get_logger(), "Did not find '%s' in command interfaces.", interface_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  const std::string tf_prefix = passthrough_params_.tf_prefix;
  {
    const std::string interface_name = tf_prefix + "trajectory_passthrough/transfer_state";
    auto it = std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
                           [&](auto& interface) { return (interface.get_name() == interface_name); });
    if (it != command_interfaces_.end()) {
      transfer_command_interface_ = *it;
    } else {
      RCLCPP_ERROR(get_node()->get_logger(), "Did not find '%s' in command interfaces.", interface_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  {
    const std::string interface_name = tf_prefix + "trajectory_passthrough/time_from_start";
    auto it = std::find_if(command_interfaces_.begin(), command_interfaces_.end(),
                           [&](auto& interface) { return (interface.get_name() == interface_name); });
    if (it != command_interfaces_.end()) {
      time_from_start_command_interface_ = *it;
    } else {
      RCLCPP_ERROR(get_node()->get_logger(), "Did not find '%s' in command interfaces.", interface_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
  }

  return ControllerInterface::on_activate(state);
}

controller_interface::CallbackReturn PassthroughTrajectoryController::on_deactivate(const rclcpp_lifecycle::State&)
{
  abort_command_interface_->get().set_value(1.0);
  if (trajectory_active_) {
    const auto active_goal = *rt_active_goal_.readFromRT();
    std::shared_ptr<control_msgs::action::FollowJointTrajectory::Result> result =
        std::make_shared<control_msgs::action::FollowJointTrajectory::Result>();
    result->set__error_string("Aborting current goal, since the controller is being deactivated.");
    active_goal->setAborted(result);
    rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
    end_goal();
  }
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type PassthroughTrajectoryController::update(const rclcpp::Time& /*time*/,
                                                                          const rclcpp::Duration& period)
{
  const auto active_goal = *rt_active_goal_.readFromRT();

  const auto current_transfer_state = transfer_command_interface_->get().get_value();

  if (active_goal && trajectory_active_) {
    if (current_transfer_state != TRANSFER_STATE_IDLE) {
      // Check if the trajectory has been aborted from the hardware interface. E.g. the robot was stopped on the teach
      // pendant.
      if (abort_command_interface_->get().get_value() == 1.0 && current_index_ > 0) {
        RCLCPP_INFO(get_node()->get_logger(), "Trajectory aborted by hardware, aborting action.");
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
      max_trajectory_time_ =
          rclcpp::Duration::from_seconds(duration_to_double(active_joint_traj_.points.back().time_from_start));
      transfer_command_interface_->get().set_value(TRANSFER_STATE_WAITING_FOR_POINT);
    }
    auto active_goal_time_tol = goal_time_tolerance_.readFromRT();
    auto joint_mapping = joint_trajectory_mapping_.readFromRT();

    // Write a new point to the command interface, if the previous point has been read by the hardware interface.
    if (current_transfer_state == TRANSFER_STATE_WAITING_FOR_POINT) {
      if (current_index_ < active_joint_traj_.points.size()) {
        //  Write the time_from_start parameter.
        time_from_start_command_interface_->get().set_value(
            duration_to_double(active_joint_traj_.points[current_index_].time_from_start));

        // Write the positions for each joint of the robot
        auto joint_names_internal = joint_names_.readFromRT();
        // We've added the joint interfaces matching the order of the joint names so we can safely access
        // them by the index.
        for (size_t i = 0; i < number_of_joints_; i++) {
          command_interfaces_[i * 3].set_value(
              active_joint_traj_.points[current_index_].positions[joint_mapping->at(joint_names_internal->at(i))]);
          // Optionally, also write velocities and accelerations for each joint.
          if (active_joint_traj_.points[current_index_].velocities.size() > 0) {
            command_interfaces_[i * 3 + 1].set_value(
                active_joint_traj_.points[current_index_].velocities[joint_mapping->at(joint_names_internal->at(i))]);
            if (active_joint_traj_.points[current_index_].accelerations.size() > 0) {
              command_interfaces_[i * 3 + 2].set_value(
                  active_joint_traj_.points[current_index_]
                      .accelerations[joint_mapping->at(joint_names_internal->at(i))]);
            } else {
              command_interfaces_[i * 3 + 2].set_value(NO_VAL);
            }
          } else {
            command_interfaces_[i * 3 + 1].set_value(NO_VAL);
            command_interfaces_[i * 3 + 2].set_value(NO_VAL);
          }
        }
        // Tell hardware interface that this point is ready to be read.
        transfer_command_interface_->get().set_value(TRANSFER_STATE_TRANSFERRING);
        current_index_++;
        // Check if all points have been written to the hardware interface.
      } else if (current_index_ == active_joint_traj_.points.size()) {
        transfer_command_interface_->get().set_value(TRANSFER_STATE_TRANSFER_DONE);
      } else {
        RCLCPP_ERROR(get_node()->get_logger(), "Hardware waiting for trajectory point while none is present!");
      }

      // When the trajectory is finished, report the goal as successful to the client.
    } else if (current_transfer_state == TRANSFER_STATE_DONE) {
      auto result = active_goal->preallocated_result_;
      // Check if the actual position complies with the tolerances given.
      if (!check_goal_tolerance()) {
        result->error_code = control_msgs::action::FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED;
        result->error_string = "Robot not within tolerances at end of trajectory.";
        active_goal->setAborted(result);
        end_goal();
        RCLCPP_ERROR(get_node()->get_logger(), "Trajectory failed, goal tolerances not met.");
      } else if (active_goal_time_tol->nanoseconds() > 0 &&
                 active_trajectory_elapsed_time_ > (max_trajectory_time_ + *active_goal_time_tol)) {
        // Check if the goal time tolerance was complied with.
        result->error_code = control_msgs::action::FollowJointTrajectory::Result::GOAL_TOLERANCE_VIOLATED;
        result->error_string =
            "Goal not reached within time tolerance. Missed goal time by " +
            std::to_string((active_trajectory_elapsed_time_ - max_trajectory_time_ - *active_goal_time_tol).seconds()) +
            " seconds.";
        active_goal->setAborted(result);
        end_goal();
      } else {
        result->error_code = control_msgs::action::FollowJointTrajectory::Result::SUCCESSFUL;
        result->error_string = "Trajectory executed successfully in " +
                               std::to_string(active_trajectory_elapsed_time_.seconds()) +
                               " (scaled) seconds! The real time needed for execution could be longer.";
        active_goal->setSucceeded(result);
        end_goal();
        RCLCPP_INFO(get_node()->get_logger(), "%s", result->error_string.c_str());
      }
    } else if (current_transfer_state == TRANSFER_STATE_IN_MOTION) {
      // Keep track of how long the trajectory has been executing, if it takes too long, send a warning.
      if (scaling_state_interface_.has_value()) {
        scaling_factor_ = scaling_state_interface_->get().get_value();
      }

      active_trajectory_elapsed_time_ += period * scaling_factor_;

      // RCLCPP_INFO(get_node()->get_logger(), "Elapsed trajectory time: %f. Scaling factor: %f, period: %f",
      // active_trajectory_elapsed_time_.seconds(), scaling_factor_, period.seconds());

      if (active_trajectory_elapsed_time_ > (max_trajectory_time_ + *active_goal_time_tol) && trajectory_active_) {
        RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *clock_, 1000,
                             "Trajectory should be finished by now. You may want to cancel this goal, if it is not.");
      }
    }
  } else if (current_transfer_state != TRANSFER_STATE_IDLE && current_transfer_state != TRANSFER_STATE_DONE) {
    // No goal is active, but we are not in IDLE, either. We have been canceled.
    abort_command_interface_->get().set_value(1.0);

  } else if (current_transfer_state == TRANSFER_STATE_DONE) {
    // We have been informed about the finished trajectory. Let's reset things.
    transfer_command_interface_->get().set_value(TRANSFER_STATE_IDLE);
    abort_command_interface_->get().set_value(0.0);
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
  if (!check_goal_positions(goal)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Trajectory rejected");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!check_goal_velocities(goal)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Trajectory rejected");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!check_goal_accelerations(goal)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Trajectory rejected");
    return rclcpp_action::GoalResponse::REJECT;
  }
  if (!check_goal_tolerances(goal)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Trajectory rejected");
    return rclcpp_action::GoalResponse::REJECT;
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

bool PassthroughTrajectoryController::check_goal_tolerances(
    std::shared_ptr<const control_msgs::action::FollowJointTrajectory::Goal> goal)
{
  auto& tolerances = goal->goal_tolerance;
  auto joint_names_internal = joint_names_.readFromRT();
  if (!tolerances.empty()) {
    for (auto& tol : tolerances) {
      auto found_it = std::find(joint_names_internal->begin(), joint_names_internal->end(), tol.name);
      if (found_it == joint_names_internal->end()) {
        RCLCPP_ERROR(get_node()->get_logger(),
                     "Tolerance for joint '%s' given. This joint is not known to this controller.", tol.name.c_str());
        return false;
      }
    }
    if (tolerances.size() != number_of_joints_) {
      RCLCPP_ERROR(get_node()->get_logger(), "Tolerances for %lu joints given. This controller knows %lu joints.",
                   tolerances.size(), number_of_joints_.load());
      return false;
    }
  }
  return true;
}

bool PassthroughTrajectoryController::check_goal_positions(
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

bool PassthroughTrajectoryController::check_goal_velocities(
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

bool PassthroughTrajectoryController::check_goal_accelerations(
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

  // TODO(fexner): Merge goal tolerances with default tolerances

  joint_trajectory_mapping_.writeFromNonRT(create_joint_mapping(goal_handle->get_goal()->trajectory.joint_names));

  // sort goal tolerances to match internal joint order
  std::vector<control_msgs::msg::JointTolerance> goal_tolerances;
  if (!goal_handle->get_goal()->goal_tolerance.empty()) {
    auto joint_names_internal = joint_names_.readFromRT();
    std::stringstream ss;
    ss << "Using goal tolerances\n";
    for (auto& joint_name : *joint_names_internal) {
      auto found_it =
          std::find_if(goal_handle->get_goal()->goal_tolerance.begin(), goal_handle->get_goal()->goal_tolerance.end(),
                       [&joint_name](auto& tol) { return tol.name == joint_name; });
      if (found_it != goal_handle->get_goal()->goal_tolerance.end()) {
        goal_tolerances.push_back(*found_it);
        ss << joint_name << " -- position: " << found_it->position << ", velocity: " << found_it->velocity
           << ", acceleration: " << found_it->acceleration << std::endl;
      }
    }
    RCLCPP_INFO_STREAM(get_node()->get_logger(), ss.str());
  }
  goal_tolerance_.writeFromNonRT(goal_tolerances);
  goal_time_tolerance_.writeFromNonRT(goal_handle->get_goal()->goal_time_tolerance);
  RCLCPP_INFO_STREAM(get_node()->get_logger(),
                     "Goal time tolerance: " << duration_to_double(goal_handle->get_goal()->goal_time_tolerance)
                                             << " sec");

  // Action handling will be done from the timer callback to avoid those things in the realtime
  // thread. First, we delete the existing (if any) timer by resetting the pointer and then create a new
  // one.
  //
  RealtimeGoalHandlePtr rt_goal = std::make_shared<RealtimeGoalHandle>(goal_handle);
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
  auto goal_tolerance = goal_tolerance_.readFromRT();
  auto joint_mapping = joint_trajectory_mapping_.readFromRT();
  auto joint_names_internal = joint_names_.readFromRT();
  if (goal_tolerance->empty()) {
    return true;
  }

  for (size_t i = 0; i < number_of_joints_; ++i) {
    const std::string joint_name = joint_names_internal->at(i);
    const auto& joint_tol = goal_tolerance->at(i);
    const auto& setpoint = active_joint_traj_.points.back().positions[joint_mapping->at(joint_name)];
    const double joint_pos = joint_position_state_interface_[i].get().get_value();
    if (std::abs(joint_pos - setpoint) > joint_tol.position) {
      // RCLCPP_ERROR(
      // get_node()->get_logger(), "Joint %s should be at position %f, but is at position %f, where tolerance is %f",
      // joint_position_state_interface_[i].get().get_name().c_str(), setpoint, joint_pos, joint_tol.position);
      return false;
    }

    if (!active_joint_traj_.points.back().velocities.empty()) {
      const double joint_vel = joint_velocity_state_interface_[i].get().get_value();
      const auto& expected_vel = active_joint_traj_.points.back().velocities[joint_mapping->at(joint_name)];
      if (std::abs(joint_vel - expected_vel) > joint_tol.velocity) {
        return false;
      }
    }
    if (!active_joint_traj_.points.back().accelerations.empty()) {
      const double joint_vel = joint_acceleration_state_interface_[i].get().get_value();
      const auto& expected_vel = active_joint_traj_.points.back().accelerations[joint_mapping->at(joint_name)];
      if (std::abs(joint_vel - expected_vel) > joint_tol.acceleration) {
        return false;
      }
    }
  }

  return true;
}

void PassthroughTrajectoryController::end_goal()
{
  trajectory_active_ = false;
  transfer_command_interface_->get().set_value(TRANSFER_STATE_IDLE);
}

std::unordered_map<std::string, size_t>
PassthroughTrajectoryController::create_joint_mapping(const std::vector<std::string>& joint_names) const
{
  std::unordered_map<std::string, size_t> joint_mapping;
  auto joint_names_internal = joint_names_.readFromNonRT();
  for (auto& joint_name : *joint_names_internal) {
    auto found_it = std::find(joint_names.begin(), joint_names.end(), joint_name);
    if (found_it != joint_names.end()) {
      joint_mapping.insert({ joint_name, found_it - joint_names.begin() });
    }
  }
  return joint_mapping;
}
}  // namespace ur_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(ur_controllers::PassthroughTrajectoryController, controller_interface::ControllerInterface)
