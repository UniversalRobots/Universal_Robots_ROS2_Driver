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

#include "ur_controllers/scaled_joint_trajectory_controller.hpp"

namespace ur_controllers
{
controller_interface::InterfaceConfiguration ScaledJointTrajectoryController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration conf;
  conf = JointTrajectoryController::state_interface_configuration();
  conf.names.push_back("speed_scaling/speed_scaling_factor");
  return conf;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ScaledJointTrajectoryController::on_activate(const rclcpp_lifecycle::State& state)
{
  TimeData time_data;
  time_data.time = node_->now();
  time_data.period = rclcpp::Duration(0, 0);
  time_data.uptime = node_->now();
  time_data_.initRT(time_data);
  return JointTrajectoryController::on_activate(state);
}

controller_interface::return_type ScaledJointTrajectoryController::update()
{
  if (state_interfaces_.back().get_name() == "speed_scaling")
  {
    scaling_factor_ = state_interfaces_.back().get_value();
  }
  else
  {
    RCLCPP_ERROR(get_node()->get_logger(), "Speed scaling interface not found in hardware interface.");
  }

  if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    if (!is_halted)
    {
      halt();
      is_halted = true;
    }
    return controller_interface::return_type::OK;
  }

  auto resize_joint_trajectory_point = [](trajectory_msgs::msg::JointTrajectoryPoint& point, size_t size) {
    point.positions.resize(size);
    point.velocities.resize(size);
    point.accelerations.resize(size);
  };
  auto compute_error_for_joint = [](JointTrajectoryPoint& error, int index, const JointTrajectoryPoint& current,
                                    const JointTrajectoryPoint& desired) {
    // error defined as the difference between current and desired
    error.positions[index] = angles::shortest_angular_distance(current.positions[index], desired.positions[index]);
    error.velocities[index] = desired.velocities[index] - current.velocities[index];
    error.accelerations[index] = 0.0;
  };

  // Check if a new external message has been received from nonRT threads
  auto current_external_msg = traj_external_point_ptr_->get_trajectory_msg();
  auto new_external_msg = traj_msg_external_point_ptr_.readFromRT();
  if (current_external_msg != *new_external_msg)
  {
    fill_partial_goal(*new_external_msg);
    sort_to_local_joint_order(*new_external_msg);
    traj_external_point_ptr_->update(*new_external_msg);
  }

  JointTrajectoryPoint state_current, state_desired, state_error;
  const auto joint_num = joint_names_.size();
  resize_joint_trajectory_point(state_current, joint_num);

  // current state update
  for (size_t index = 0; index < joint_num; ++index)
  {
    state_current.positions[index] = joint_position_state_interface_[index].get().get_value();
    state_current.velocities[index] = joint_velocity_state_interface_[index].get().get_value();
    state_current.accelerations[index] = 0.0;
  }
  state_current.time_from_start.set__sec(0);

  // currently carrying out a trajectory
  if (traj_point_active_ptr_ && !(*traj_point_active_ptr_)->has_trajectory_msg())
  {
    // if sampling the first time, set the point before you sample
    if (!(*traj_point_active_ptr_)->is_sampled_already())
    {
      (*traj_point_active_ptr_)->set_point_before_trajectory_msg(node_->now(), state_current);
    }
    resize_joint_trajectory_point(state_error, joint_num);

    // Main Speed scaling difference...
    // Adjust time with scaling factor
    TimeData time_data;
    time_data.time = node_->now();
    rcl_duration_value_t period = (time_data.time - time_data_.readFromRT()->time).nanoseconds();
    time_data.period = rclcpp::Duration(scaling_factor_ * period);
    time_data.uptime = time_data_.readFromRT()->uptime + time_data.period;
    rclcpp::Time traj_time = time_data_.readFromRT()->uptime + rclcpp::Duration(period);
    time_data_.writeFromNonRT(time_data);

    // find segment for current timestamp
    joint_trajectory_controller::TrajectoryPointConstIter start_segment_itr, end_segment_itr;
    const bool valid_point =
        (*traj_point_active_ptr_)->sample(traj_time, state_desired, start_segment_itr, end_segment_itr);

    if (valid_point)
    {
      bool abort = false;
      bool outside_goal_tolerance = false;
      const bool before_last_point = end_segment_itr != (*traj_point_active_ptr_)->end();
      for (size_t index = 0; index < joint_num; ++index)
      {
        // set values for next hardware write()
        joint_position_command_interface_[index].get().set_value(state_desired.positions[index]);
        compute_error_for_joint(state_error, index, state_current, state_desired);

        if (before_last_point &&
            !check_state_tolerance_per_joint(state_error, index, default_tolerances_.state_tolerance[index], true))
        {
          abort = true;
        }
        // past the final point, check that we end up inside goal tolerance
        if (!before_last_point &&
            !check_state_tolerance_per_joint(state_error, index, default_tolerances_.goal_state_tolerance[index], true))
        {
          outside_goal_tolerance = true;
        }
      }

      const auto active_goal = *rt_active_goal_.readFromRT();
      if (active_goal)
      {
        // send feedback
        auto feedback = std::make_shared<FollowJTrajAction::Feedback>();
        feedback->header.stamp = node_->now();
        feedback->joint_names = joint_names_;

        feedback->actual = state_current;
        feedback->desired = state_desired;
        feedback->error = state_error;
        active_goal->setFeedback(feedback);

        // check abort
        if (abort || outside_goal_tolerance)
        {
          auto result = std::make_shared<FollowJTrajAction::Result>();

          if (abort)
          {
            RCLCPP_WARN(node_->get_logger(), "Aborted due to state tolerance violation");
            result->set__error_code(FollowJTrajAction::Result::PATH_TOLERANCE_VIOLATED);
          }
          else if (outside_goal_tolerance)
          {
            RCLCPP_WARN(node_->get_logger(), "Aborted due to goal tolerance violation");
            result->set__error_code(FollowJTrajAction::Result::GOAL_TOLERANCE_VIOLATED);
          }
          active_goal->setAborted(result);
          rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
        }

        // check goal tolerance
        if (!before_last_point)
        {
          if (!outside_goal_tolerance)
          {
            auto res = std::make_shared<FollowJTrajAction::Result>();
            res->set__error_code(FollowJTrajAction::Result::SUCCESSFUL);
            active_goal->setSucceeded(res);
            rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());

            RCLCPP_INFO(node_->get_logger(), "Goal reached, success!");
          }
          else if (default_tolerances_.goal_time_tolerance != 0.0)
          {
            // if we exceed goal_time_toleralance set it to aborted
            const rclcpp::Time traj_start = (*traj_point_active_ptr_)->get_trajectory_start_time();
            const rclcpp::Time traj_end = traj_start + start_segment_itr->time_from_start;

            // TODO This will break in speed scaling we have to discuss how to handle the goal
            // time when the robot scales itself down.
            const double difference = node_->now().seconds() - traj_end.seconds();
            if (difference > default_tolerances_.goal_time_tolerance)
            {
              auto result = std::make_shared<FollowJTrajAction::Result>();
              result->set__error_code(FollowJTrajAction::Result::GOAL_TOLERANCE_VIOLATED);
              active_goal->setAborted(result);
              rt_active_goal_.writeFromNonRT(RealtimeGoalHandlePtr());
              RCLCPP_WARN(node_->get_logger(), "Aborted due goal_time_tolerance exceeding by %f seconds", difference);
            }
          }
        }
      }
    }
  }

  publish_state(state_desired, state_current, state_error);
  return controller_interface::return_type::OK;
}

}  // namespace ur_controllers

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ur_controllers::ScaledJointTrajectoryController, controller_interface::ControllerInterface)
