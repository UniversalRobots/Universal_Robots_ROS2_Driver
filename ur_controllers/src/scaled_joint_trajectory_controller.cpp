// Copyright 2019, FZI Forschungszentrum Informatik
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
 * \author  Marvin Große Besselmann grosse@fzi.de
 * \date    2021-02-18
 *
 */
//----------------------------------------------------------------------

#include <memory>
#include <vector>

#include "ur_controllers/scaled_joint_trajectory_controller.hpp"

#include "lifecycle_msgs/msg/state.hpp"

namespace ur_controllers
{

controller_interface::CallbackReturn ScaledJointTrajectoryController::on_init()
{
  // Create the parameter listener and get the parameters
  scaled_param_listener_ = std::make_shared<scaled_joint_trajectory_controller::ParamListener>(get_node());
  scaled_params_ = scaled_param_listener_->get_params();
  if (!scaled_params_.speed_scaling_interface_name.empty()) {
    RCLCPP_INFO(get_node()->get_logger(), "Using scaling state from the hardware from interface %s.",
                scaled_params_.speed_scaling_interface_name.c_str());
  } else {
    RCLCPP_INFO(get_node()->get_logger(), "No scaling interface set. This controller will not use speed scaling.");
  }

  return JointTrajectoryController::on_init();
}

controller_interface::InterfaceConfiguration ScaledJointTrajectoryController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf = JointTrajectoryController::state_interface_configuration();

  if (!scaled_params_.speed_scaling_interface_name.empty()) {
    conf.names.push_back(scaled_params_.speed_scaling_interface_name);
  }

  return conf;
}

controller_interface::CallbackReturn ScaledJointTrajectoryController::on_activate(const rclcpp_lifecycle::State& state)
{
  // Set scaling interfaces
  if (!scaled_params_.speed_scaling_interface_name.empty()) {
    auto it = std::find_if(state_interfaces_.begin(), state_interfaces_.end(), [&](auto& interface) {
      return (interface.get_name() == scaled_params_.speed_scaling_interface_name);
    });
    if (it != state_interfaces_.end()) {
      scaling_state_interface_ = *it;
    } else {
      RCLCPP_ERROR(get_node()->get_logger(), "Did not find speed scaling interface in state interfaces.");
    }
  }

  return JointTrajectoryController::on_activate(state);
}

controller_interface::return_type ScaledJointTrajectoryController::update(const rclcpp::Time& time,
                                                                          const rclcpp::Duration& period)
{
  if (scaling_state_interface_.has_value()) {
    scaling_factor_ = scaling_state_interface_->get().get_optional().value_or(1.0);
  }

  auto logger = this->get_node()->get_logger();
  // update dynamic parameters
  if (param_listener_->is_old(params_)) {
    params_ = param_listener_->get_params();
    default_tolerances_ = get_segment_tolerances(logger, params_);
    // update the PID gains
    // variable use_closed_loop_pid_adapter_ is updated in on_configure only
    if (use_closed_loop_pid_adapter_) {
      update_pids();
    }
  }

  // don't update goal after we sampled the trajectory to avoid any racecondition
  const auto active_goal = *rt_active_goal_.readFromRT();

  // Check if a new trajectory message has been received from Non-RT threads
  const auto current_trajectory_msg = current_trajectory_->get_trajectory_msg();
  auto new_external_msg = new_trajectory_msg_.readFromRT();
  // Discard, if a goal is pending but still not active (somewhere stuck in goal_handle_timer_)
  if (current_trajectory_msg != *new_external_msg && (rt_has_pending_goal_ && !active_goal) == false) {
    fill_partial_goal(*new_external_msg);
    sort_to_local_joint_order(*new_external_msg);
    // TODO(denis): Add here integration of position and velocity
    current_trajectory_->update(*new_external_msg);
  }

  // current state update
  state_current_.time_from_start.set__sec(0);
  read_state_from_state_interfaces(state_current_);

  // currently carrying out a trajectory
  if (has_active_trajectory()) {
    bool first_sample = false;
    joint_trajectory_controller::TrajectoryPointConstIter start_segment_itr, end_segment_itr;
    // if sampling the first time, set the point before you sample
    if (!current_trajectory_->is_sampled_already()) {
      first_sample = true;
      if (params_.interpolate_from_desired_state || params_.open_loop_control) {
        if (std::abs(last_commanded_time_.seconds()) < std::numeric_limits<float>::epsilon()) {
          last_commanded_time_ = time;
        }
        current_trajectory_->set_point_before_trajectory_msg(last_commanded_time_, last_commanded_state_,
                                                             joints_angle_wraparound_);
      } else {
        current_trajectory_->set_point_before_trajectory_msg(time, state_current_, joints_angle_wraparound_);
      }
      traj_time_ = time;
    } else {
      traj_time_ += period * scaling_factor_.load();
    }

    // Sample expected state from the trajectory
    current_trajectory_->sample(traj_time_, interpolation_method_, state_desired_, start_segment_itr, end_segment_itr);
    state_desired_.time_from_start = traj_time_ - current_trajectory_->time_from_start();

    // Sample setpoint for next control cycle
    const bool valid_point = current_trajectory_->sample(traj_time_ + update_period_, interpolation_method_,
                                                         command_next_, start_segment_itr, end_segment_itr, false);

    state_current_.time_from_start = time - current_trajectory_->time_from_start();

    if (valid_point) {
      const rclcpp::Time traj_start = current_trajectory_->time_from_start();
      // this is the time instance
      // - started with the first segment: when the first point will be reached (in the future)
      // - later: when the point of the current segment was reached
      const rclcpp::Time segment_time_from_start = traj_start + start_segment_itr->time_from_start;
      // time_difference is
      // - negative until first point is reached
      // - counting from zero to time_from_start of next point
      double time_difference = traj_time_.seconds() - segment_time_from_start.seconds();
      bool tolerance_violated_while_moving = false;
      bool outside_goal_tolerance = false;
      bool within_goal_time = true;
      const bool before_last_point = end_segment_itr != current_trajectory_->end();
      auto active_tol = active_tolerances_.readFromRT();

      // have we reached the end, are not holding position, and is a timeout configured?
      // Check independently of other tolerances
      if (!before_last_point && !rt_is_holding_ && cmd_timeout_ > 0.0 && time_difference > cmd_timeout_) {
        RCLCPP_WARN(logger, "Aborted due to command timeout");

        new_trajectory_msg_.reset();
        new_trajectory_msg_.initRT(set_hold_position());
      }

      // Check state/goal tolerance
      for (size_t index = 0; index < dof_; ++index) {
        compute_error_for_joint(state_error_, index, state_current_, state_desired_);

        // Always check the state tolerance on the first sample in case the first sample
        // is the last point
        // print output per default, goal will be aborted afterwards
        if ((before_last_point || first_sample) && !rt_is_holding_ &&
            !check_state_tolerance_per_joint(state_error_, index, active_tol->state_tolerance[index],
                                             true /* show_errors */)) {
          tolerance_violated_while_moving = true;
        }
        // past the final point, check that we end up inside goal tolerance
        if (!before_last_point && !rt_is_holding_ &&
            !check_state_tolerance_per_joint(state_error_, index, active_tol->goal_state_tolerance[index],
                                             false /* show_errors */)) {
          outside_goal_tolerance = true;

          if (active_tol->goal_time_tolerance != 0.0) {
            // if we exceed goal_time_tolerance set it to aborted
            if (time_difference > active_tol->goal_time_tolerance) {
              within_goal_time = false;
              // print once, goal will be aborted afterwards
              check_state_tolerance_per_joint(state_error_, index, default_tolerances_.goal_state_tolerance[index],
                                              true /* show_errors */);
            }
          }
        }
      }

      // set values for next hardware write() if tolerance is met
      if (!tolerance_violated_while_moving && within_goal_time) {
        if (use_closed_loop_pid_adapter_) {
          // Update PIDs
          for (auto i = 0ul; i < num_cmd_joints_; ++i) {
            // If effort interface only, add desired effort as feed forward
            // If velocity interface, ignore desired effort
            size_t index_cmd_joint = map_cmd_to_joints_[i];
            tmp_command_[index_cmd_joint] =
                (command_next_.velocities[index_cmd_joint] * ff_velocity_scale_[i]) +
                (has_effort_command_interface_ ? command_next_.effort[index_cmd_joint] : 0.0) +
                pids_[i]->compute_command(state_error_.positions[index_cmd_joint],
                                          state_error_.velocities[index_cmd_joint], period);
          }
        }

        // set values for next hardware write()
        if (has_position_command_interface_) {
          assign_interface_from_point(joint_command_interface_[0], command_next_.positions);
        }
        if (has_velocity_command_interface_) {
          if (use_closed_loop_pid_adapter_) {
            assign_interface_from_point(joint_command_interface_[1], tmp_command_);
          } else {
            assign_interface_from_point(joint_command_interface_[1], command_next_.velocities);
          }
        }
        if (has_acceleration_command_interface_) {
          assign_interface_from_point(joint_command_interface_[2], command_next_.accelerations);
        }
        if (has_effort_command_interface_) {
          if (use_closed_loop_pid_adapter_) {
            assign_interface_from_point(joint_command_interface_[3], tmp_command_);
          } else {
            // If position and effort command interfaces, only pass desired effort
            assign_interface_from_point(joint_command_interface_[3], state_desired_.effort);
          }
        }

        // store the previous command and time used in open-loop control mode
        last_commanded_state_ = command_next_;
        last_commanded_time_ = time;
      }

      if (active_goal) {
        // send feedback
        auto feedback = std::make_shared<FollowJTrajAction::Feedback>();
        feedback->header.stamp = time;
        feedback->joint_names = params_.joints;

        feedback->actual = state_current_;
        feedback->desired = state_desired_;
        feedback->error = state_error_;
        active_goal->setFeedback(feedback);

        // check abort
        if (tolerance_violated_while_moving) {
          auto result = std::make_shared<FollowJTrajAction::Result>();
          result->set__error_code(FollowJTrajAction::Result::PATH_TOLERANCE_VIOLATED);
          result->set__error_string("Aborted due to path tolerance violation");
          active_goal->setAborted(result);
          // TODO(matthew-reynolds): Need a lock-free write here
          // See https://github.com/ros-controls/ros2_controllers/issues/168
          rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
          rt_has_pending_goal_ = false;

          RCLCPP_WARN(logger, "Aborted due to state tolerance violation");

          new_trajectory_msg_.reset();
          new_trajectory_msg_.initRT(set_hold_position());
        } else if (!before_last_point) {
          // check goal tolerance
          if (!outside_goal_tolerance) {
            auto result = std::make_shared<FollowJTrajAction::Result>();
            result->set__error_code(FollowJTrajAction::Result::SUCCESSFUL);
            result->set__error_string("Goal successfully reached!");
            active_goal->setSucceeded(result);
            // TODO(matthew-reynolds): Need a lock-free write here
            // See https://github.com/ros-controls/ros2_controllers/issues/168
            rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
            rt_has_pending_goal_ = false;

            RCLCPP_INFO(logger, "Goal reached, success!");

            new_trajectory_msg_.reset();
            new_trajectory_msg_.initRT(set_success_trajectory_point());
          } else if (!within_goal_time) {
            const std::string error_string =
                "Aborted due to goal_time_tolerance exceeding by " + std::to_string(time_difference) + " seconds";

            auto result = std::make_shared<FollowJTrajAction::Result>();
            result->set__error_code(FollowJTrajAction::Result::GOAL_TOLERANCE_VIOLATED);
            result->set__error_string(error_string);
            active_goal->setAborted(result);
            // TODO(matthew-reynolds): Need a lock-free write here
            // See https://github.com/ros-controls/ros2_controllers/issues/168
            rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
            rt_has_pending_goal_ = false;

            RCLCPP_WARN(logger, "%s", error_string.c_str());

            new_trajectory_msg_.reset();
            new_trajectory_msg_.initRT(set_hold_position());
          }
        }
      } else if (tolerance_violated_while_moving && !rt_has_pending_goal_) {
        // we need to ensure that there is no pending goal -> we get a race condition otherwise
        RCLCPP_ERROR(logger, "Holding position due to state tolerance violation");

        new_trajectory_msg_.reset();
        new_trajectory_msg_.initRT(set_hold_position());
      } else if (!before_last_point && !within_goal_time && !rt_has_pending_goal_) {
        RCLCPP_ERROR(logger, "Exceeded goal_time_tolerance: holding position...");

        new_trajectory_msg_.reset();
        new_trajectory_msg_.initRT(set_hold_position());
      }
      // else, run another cycle while waiting for outside_goal_tolerance
      // to be satisfied (will stay in this state until new message arrives)
      // or outside_goal_tolerance violated within the goal_time_tolerance
    }
  }

  // TODO(fmauch): Remove once merged upstream
  if (state_publisher_->trylock()) {
    state_publisher_->msg_.speed_scaling_factor = scaling_factor_;
    state_publisher_->unlock();
  }
  // end remove once merged upstream

  publish_state(time, state_desired_, state_current_, state_error_);

  return controller_interface::return_type::OK;
}

// Copied from upstream JTC
void ScaledJointTrajectoryController::update_pids()
{
  for (size_t i = 0; i < num_cmd_joints_; ++i) {
    const auto& gains = params_.gains.joints_map.at(params_.joints.at(map_cmd_to_joints_[i]));
    control_toolbox::AntiWindupStrategy antiwindup_strat;
    antiwindup_strat.set_type(gains.antiwindup_strategy);
    antiwindup_strat.i_max = gains.i_clamp;
    antiwindup_strat.i_min = -gains.i_clamp;
    antiwindup_strat.error_deadband = gains.error_deadband;
    antiwindup_strat.tracking_time_constant = gains.tracking_time_constant;
    if (pids_[i]) {
      // update PIDs with gains from ROS parameters
      pids_[i]->set_gains(gains.p, gains.i, gains.d, gains.u_clamp_max, gains.u_clamp_min, antiwindup_strat);
    } else {
      // Init PIDs with gains from ROS parameters
      pids_[i] = std::make_shared<control_toolbox::Pid>(gains.p, gains.i, gains.d, gains.u_clamp_max, gains.u_clamp_min,
                                                        antiwindup_strat);
    }
    ff_velocity_scale_[i] = gains.ff_velocity_scale;
  }
}

}  // namespace ur_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ur_controllers::ScaledJointTrajectoryController, controller_interface::ControllerInterface)
